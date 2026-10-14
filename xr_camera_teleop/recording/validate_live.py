"""Read-only live capture validation with SYNTHETIC hand gestures; never sends robot commands.

Real camera/DDS data are archived. The synthetic gesture tests the recording path,
not WiLoR's ability to recognize a user's physical hands.
"""
import argparse
import json
from pathlib import Path
import statistics
import tempfile
import time

import numpy as np

from .backend import DatasetBackend


def validation_hand(side,closed=False):
    points = np.zeros((21,3))
    points[:,2] = .5
    sign = 1 if side=="left" else -1
    for start,x in ((5,-.03),(9,0),(13,.018),(17,.035)):
        for j in range(4):
            points[start+j,:2] = [sign*x,-.055-.025*j]
        if closed:
            points[start+2,1],points[start+3,1] = -.055,-.035
    for j in range(4):
        points[j+1,:2] = [-sign*(.035+j*.017),-.01-j*.012]
    return {"keypoints":points.tolist(),"conf":.95,"wrist_pos":points[0].tolist()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",required=True)
    parser.add_argument("--output",default="~/teleop_data/_validation")
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    # Action publishers are intentionally not started; allow time to test stop gesture.
    config["required_source_timeout_seconds"] = 30
    config["export"]["automatic"] = False
    backend = None
    with tempfile.TemporaryDirectory(prefix="xrcam-validation-") as temporary:
        path = Path(temporary)/"config.json"
        path.write_text(json.dumps(config))
        try:
            backend = DatasetBackend(path,root=args.output)
            backend.set_prompt("SYSTEM VALIDATION synthetic recording gestures real PC2 data")
            deadline = time.monotonic()+15
            while time.monotonic()<deadline:
                status = backend.status()
                if (status["timing"]["camera_clock_valid"] and status["timing"]["same_pc2_monotonic_epoch"]
                        and status["clocks"]["camera"] and status["clocks"]["camera"]["probe_count"]>=3):
                    break
                time.sleep(.2)
            else:
                raise RuntimeError("source clocks not ready: "+json.dumps(backend.status()))
            sequence = 0
            def gesture(closed,duration):
                nonlocal sequence
                end = time.monotonic()+duration
                while time.monotonic()<end:
                    now = time.monotonic()
                    backend.observe_hands({"frame_id":sequence,"t_publish":now,"t_capture":now,
                        "hands":{side:validation_hand(side,closed) for side in ("left","right")}})
                    sequence += 1
                    time.sleep(.05)
            gesture(False,1.25)
            deadline = time.monotonic()+20
            while time.monotonic()<deadline and backend.status()["phase"]!="recording":
                if backend.error:
                    raise RuntimeError(backend.error)
                time.sleep(.05)
            if backend.status()["phase"]!="recording":
                raise RuntimeError("start gesture did not start recording")
            print("START gesture accepted",flush=True)
            gesture(False,.3)
            if backend.status()["phase"]!="recording":
                raise RuntimeError("held gesture unexpectedly stopped recording")
            gesture(True,.45)
            gesture(False,1.25)
            print("STOP gesture sent",flush=True)
            deadline = time.monotonic()+90
            while time.monotonic()<deadline:
                status = backend.status()
                if status["phase"]=="ready" and status["transfer"]["state"] in ("verified","incomplete"):
                    break
                if backend.error:
                    raise RuntimeError(backend.error)
                time.sleep(.5)
            else:
                raise RuntimeError("finalization/transfer timed out: "+json.dumps(backend.status()))
            episode = Path(status["last_path"])
            report_path = episode/"pc2"/"bag"/"sync_report.json"
            legacy = episode/"sync_audit.json"
            report = json.loads(report_path.read_text()) if report_path.exists() else json.loads(legacy.read_text())
            transfer = json.loads((episode/"transfer.json").read_text())
            quality = json.loads((episode/"quality.json").read_text())
            clock = report.get("clock", {})
            match = report.get("camera_state_match", {})
            summary = {"episode":str(episode),"gesture_input":"synthetic; real human recognition not tested",
                       "stop_reason":quality["stop_reason"],"stereo":report["stereo"],
                       "clock_verified":clock.get("verified"),
                       "software_clock_invalid_frames":clock.get("camera_clock_invalid_frames"),
                       "camera_to_state_age_ms_median":match.get("state_age_ms_median"),
                       "camera_to_state_age_ms_p95":match.get("state_age_ms_p95"),
                       "camera_to_state_age_ms_max":match.get("state_age_ms_max"),
                       "matched_ratio":match.get("matched_ratio"),
                       "source_rates":{name:info.get("receipt_hz") for name,info in report.get("sources",{}).items()},
                       "transfer_verified":{name:entry.get("transfer_verified",False) for name,entry in transfer["sources"].items()},
                       "issues":report["issues"],"preview_fps":status["preview_fps"]}
            (episode/"validation_report.json").write_text(json.dumps(summary,indent=2))
            print(json.dumps(summary,indent=2),flush=True)
            if quality["stop_reason"]!="gesture":
                raise RuntimeError("episode was not stopped by the gesture")
            if not all(summary["transfer_verified"].values()):
                raise RuntimeError("one or more source transfers failed")
        finally:
            if backend:
                backend.close()


if __name__=="__main__":
    main()
