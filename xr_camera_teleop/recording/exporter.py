"""Offline LeRobot v3 export, retaining raw episodes and preserving gesture boundaries."""
import argparse
from bisect import bisect_right
from collections import OrderedDict
import json
from pathlib import Path
import shutil
import struct
import uuid

import msgpack
import numpy as np

from .images import image_bgr
from .schema import NATIVE_JOINT_NAMES, joint_permutation, validate_joint_vector


def indexed_samples(path):
    with Path(path).open("rb") as file:
        while True:
            offset = file.tell()
            prefix = file.read(4)
            if not prefix:
                return
            if len(prefix) != 4:
                raise ValueError("truncated archive")
            length = struct.unpack("<I",prefix)[0]
            if length > 64*1024*1024:
                raise ValueError("oversized archive sample")
            payload = file.read(length)
            if len(payload) != length:
                raise ValueError("truncated archive")
            yield offset,msgpack.unpackb(payload,raw=False)


def frame_key(stamp):
    return (stamp["source_id"],stamp["session_id"],stamp["sequence"])


def prepare_episode(path, fps):
    manifest = json.loads((path/"manifest.json").read_text())
    if not manifest.get("complete"):
        raise ValueError("episode archive not finalized")
    rows,frames = [],{}
    for offset,packet in indexed_samples(path/"samples.msgpack"):
        if packet.get("type") == "dataset_row":
            rows.append(packet)
        elif packet.get("is_image"):
            frames[frame_key(packet["stamp"])] = offset
    rows.sort(key=lambda row:row["local_monotonic_ns"])
    if not rows:
        raise ValueError("no dataset rows")
    times = [row["local_monotonic_ns"] for row in rows]
    start,end = times[0],times[-1]
    selected = []
    for index in range(int((end-start)*fps/1e9)+1):
        timestamp = start+round(index*1e9/fps)
        row = rows[bisect_right(times,timestamp)-1]
        if timestamp-row["local_monotonic_ns"] > max(30_000_000,2e9/fps):
            raise ValueError("numeric sample gap; raw episode preserved")
        if row.get("state") is None or row.get("action") is None:
            raise ValueError("missing state/action; raw episode preserved, no fabricated labels")
        validate_joint_vector(row["state"])
        validate_joint_vector(row["action"])
        if not row.get("clock_calibrated",False):
            raise ValueError("uncalibrated timestamps; raw episode preserved")
        if row.get("missing_sources"):
            raise ValueError("required source gap; raw episode preserved")
        for image in row["images"].values():
            if not image.get("stamp") or frame_key(image["stamp"]) not in frames:
                raise ValueError("camera frame missing from archive")
            if image.get("age_seconds",-1) < 0:
                raise ValueError("future camera frame cannot be an observation")
        selected.append((index,row))
    return manifest,selected,frames


def export_task(task_path, fps=100, reference=None):
    import cv2
    import pyarrow as pa
    import pyarrow.parquet as pq
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    task_path = Path(task_path).expanduser().resolve()
    if not isinstance(fps,int) or not 1 <= fps <= 100:
        raise ValueError("fps must be an integer between 1 and 100")
    names = list(NATIVE_JOINT_NAMES)
    permutation_state = permutation_action = tuple(range(28))
    if reference:
        reference = json.loads(Path(reference).read_text())
        permutation_state = joint_permutation(reference["state_names"])
        permutation_action = joint_permutation(reference["action_names"])
        names = reference["state_names"]
    candidates = sorted((task_path/"raw").glob("episode_*"))
    report = {"fps":fps,"training_order_verified":reference is not None,"episodes":[]}
    valid = []
    for path in candidates:
        try:
            manifest,rows,frames = prepare_episode(path,fps)
            valid.append((path,manifest,rows,frames))
        except Exception as exc:
            report["episodes"].append({"raw_path":str(path),"exported":False,"error":str(exc)})
    if not valid:
        (task_path/"export_report.json").write_text(json.dumps(report,indent=2))
        raise ValueError("no exportable episodes; see export_report.json; raw recordings retained")
    images = tuple(valid[0][2][0][1]["images"])
    features = {
        "observation.state":{"dtype":"float32","shape":(28,),"names":names},
        "action":{"dtype":"float32","shape":(28,),"names":reference["action_names"] if reference else names},
        **{key:{"dtype":"video","shape":(480,640,3),"names":["height","width","channels"]} for key in images},
    }
    stage = task_path/(".lerobot_export_"+uuid.uuid4().hex)
    dataset = LeRobotDataset.create("local/g1_dex3_teleop",fps,features,root=stage,
                                   robot_type="unitree_g1_dex3",vcodec="libsvtav1",video_backend="pyav")
    timing_rows = []
    try:
        for episode_index,(path,manifest,rows,frames) in enumerate(valid):
            cache = OrderedDict()
            with (path/"samples.msgpack").open("rb") as archive:
                for index,row in rows:
                    sample = {"task":manifest["prompt"],
                              "observation.state":np.asarray(row["state"],np.float32)[list(permutation_state)],
                              "action":np.asarray(row["action"],np.float32)[list(permutation_action)]}
                    for key in images:
                        stamp = row["images"][key]["stamp"]
                        identity = frame_key(stamp)
                        if identity not in cache:
                            archive.seek(frames[identity])
                            size = struct.unpack("<I",archive.read(4))[0]
                            packet = msgpack.unpackb(archive.read(size),raw=False)
                            bgr = image_bgr(packet)
                            cache[identity] = cv2.cvtColor(cv2.resize(bgr,(640,480)),cv2.COLOR_BGR2RGB)
                            if len(cache)>8:
                                cache.popitem(last=False)
                        sample[key] = cache[identity]
                    dataset.add_frame(sample)
                    timing_rows.append({"episode_index":episode_index,"frame_index":index,
                                        "source_local_monotonic_ns":row["local_monotonic_ns"],
                                        "source_metadata_json":json.dumps(row,default=str)})
            dataset.save_episode(parallel_encoding=False)
            report["episodes"].append({"raw_path":str(path),"exported":True,"episode_index":episode_index,
                                       "frames":len(rows),"prompt":manifest["prompt"]})
        dataset.finalize()
        (stage/"quality").mkdir()
        pq.write_table(pa.Table.from_pylist(timing_rows),stage/"quality/row_sources.parquet")
        (stage/"quality/export_report.json").write_text(json.dumps(report,indent=2,ensure_ascii=False))
        # Load the actual v3 schema before installing the export.
        check = LeRobotDataset("local/g1_dex3_teleop",root=stage,video_backend="pyav")
        assert check.num_episodes == len(valid)
        assert check[0]["observation.state"].shape == (28,)
        final = task_path/("lerobot" if fps==100 else f"lerobot_{fps}hz")
        backup = None
        if final.exists():
            backup = task_path/(".previous_lerobot_"+uuid.uuid4().hex)
            final.rename(backup)
        try:
            stage.rename(final)
        except Exception:
            if backup:
                backup.rename(final)
            raise
        if backup:
            shutil.rmtree(backup)
        (task_path/"export_report.json").write_text(json.dumps(report,indent=2,ensure_ascii=False))
        return final
    except Exception:
        dataset.finalize()
        report["failed_stage"] = str(stage)
        (task_path/"export_report.json").write_text(json.dumps(report,indent=2,ensure_ascii=False))
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task_directory")
    parser.add_argument("--fps",type=int,default=100)
    parser.add_argument("--reference-manifest")
    args = parser.parse_args()
    print(export_task(args.task_directory,args.fps,args.reference_manifest))


if __name__ == "__main__":
    main()
