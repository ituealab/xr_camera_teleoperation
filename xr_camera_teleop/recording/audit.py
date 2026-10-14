"""Strict stereo/DDS time audit. Invalid clocks never fall back to camera receipt time."""
from collections import defaultdict
import json
from pathlib import Path

from .timing import causal_index


def audit_episode(episode, max_state_age_ns=50_000_000, max_action_age_ns=100_000_000):
    root = Path(episode)/"pc2"
    report = {"format":"xrcam-sync-audit-v2","sources":{},"issues":[],
              "dds_time_semantics":"collector_receipt_not_verified_sensor_measurement",
              "limits":{"state_age_ms":max_state_age_ns/1e6,"action_age_ms":max_action_age_ns/1e6},
              "zero_sensor_latency_claimed":False}
    samples = defaultdict(list)
    manifests = {}
    for source in ("camera","telemetry"):
        index = root/source/"index.jsonl"
        manifest_path = root/source/"manifest.json"
        if manifest_path.exists():
            manifests[source] = json.loads(manifest_path.read_text())
        if not index.exists():
            report["issues"].append("missing "+source+" archive")
            continue
        with index.open() as file:
            for line in file:
                packet = json.loads(line)
                name = packet.get("topic",packet.get("source",packet.get("type","unknown")))
                samples[name].append(packet)
    bounds = [(m.get("capture_start_monotonic_ns"),m.get("capture_stop_monotonic_ns")) for m in manifests.values()]
    report["common_capture_window"] = list(bounds[0]) if bounds and all(b==bounds[0] for b in bounds) else None
    if bounds and not all(b==bounds[0] for b in bounds):
        report["issues"].append("source episode boundaries disagree")
    boots = {p["stamp"].get("collector_boot_id") for values in samples.values() for p in values}
    report["collector_boot_ids"] = sorted(b for b in boots if b)
    same_boot = len(boots)==1 and None not in boots
    if not same_boot:
        report["issues"].append("missing or different collector boot: cannot compare monotonic epochs")
    for name,values in samples.items():
        stamps = [p.get("stamp",{}) for p in values]
        gaps = [[a.get("sequence"),b.get("sequence")] for a,b in zip(stamps,stamps[1:])
                if a.get("session_id")==b.get("session_id") and b.get("sequence",0)!=a.get("sequence",-1)+1]
        regressions = sum(b["collector_monotonic_ns"]<a["collector_monotonic_ns"] for a,b in zip(stamps,stamps[1:]))
        times = [s["collector_monotonic_ns"] for s in stamps]
        report["sources"][name] = {"samples":len(values),"sequence_gaps":gaps,"time_regressions":regressions,
            "receipt_hz":(len(times)-1)*1e9/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else None}
        if gaps or regressions:
            report["issues"].append("sequence/time discontinuity: "+name)
    left = samples.get("/zed2i/zed_node/left/image_rect_color",[])
    right = samples.get("/zed2i/zed_node/right/image_rect_color",[])
    window = report["common_capture_window"]
    def in_window(packet):
        when = packet["stamp"].get("capture_monotonic_ns")
        if not window or when is None:
            return True
        return (window[0] is None or when>=window[0]) and (window[1] is None or when<=window[1])
    left,right = [p for p in left if in_window(p)],[p for p in right if in_window(p)]
    def identity(packet):
        stamp = packet["stamp"]
        return (stamp.get("collector_boot_id"),stamp.get("source_timestamp_ns"))
    left_ids,right_ids = {identity(p) for p in left},{identity(p) for p in right}
    report["stereo"] = {"exact_timestamp_pairs":len(left_ids&right_ids),"left_only":len(left_ids-right_ids),
                        "right_only":len(right_ids-left_ids),"unpaired_frames_preserved":True}
    if not left or not right:
        report["issues"].append("required stereo source missing")
    elif left_ids!=right_ids:
        report["issues"].append("unpaired stereo frames")
    right_lookup = {identity(p):p for p in right}
    dds = {name:sorted(values,key=lambda p:p["stamp"]["collector_monotonic_ns"])
           for name,values in samples.items() if name.startswith("rt/")}
    expected = ["rt/lowstate","rt/dex3/left/state","rt/dex3/right/state",
                "rt/arm_sdk" if "rt/arm_sdk" in dds else "rt/lowcmd",
                "rt/dex3/left/cmd","rt/dex3/right/cmd"]
    for topic in expected:
        if topic not in dds:
            report["issues"].append("required DDS source missing: "+topic)
    dds_times = {name:[p["stamp"]["collector_monotonic_ns"] for p in values] for name,values in dds.items()}
    matches = []
    invalid_clocks,stale_matches = 0,0
    for frame in left:
        stamp = frame["stamp"]
        valid = same_boot and stamp.get("capture_clock_valid",False) and stamp.get("capture_monotonic_ns") is not None
        partner = right_lookup.get(identity(frame))
        if partner is not None:
            valid = valid and partner["stamp"].get("capture_clock_valid",False)
        reference = stamp.get("capture_monotonic_ns") if valid else None
        invalid_clocks += not valid
        ages,selected = {},{}
        for topic in expected:
            limit = max_state_age_ns if topic.endswith("state") else max_action_age_ns
            index = causal_index(dds_times.get(topic,[]),reference,limit) if valid else None
            packet = dds[topic][index] if index is not None else None
            ages[topic] = (reference-packet["stamp"]["collector_monotonic_ns"])/1e6 if packet else None
            selected[topic] = ({"sequence":packet["stamp"]["sequence"],"session_id":packet["stamp"]["session_id"],
                                "collector_monotonic_ns":packet["stamp"]["collector_monotonic_ns"],
                                "payload_file":packet.get("payload_file"),"payload_offset":packet.get("payload_offset")}
                               if packet else None)
        stale_matches += any(value is None for value in selected.values())
        matches.append({"camera_sequence":stamp["sequence"],"camera_header_ns":stamp.get("source_timestamp_ns"),
                        "right_camera_sequence":partner["stamp"]["sequence"] if partner else None,
                        "capture_monotonic_estimate_ns":reference,"capture_clock_relation_verified":bool(valid),
                        "capture_clock_uncertainty_ns":stamp.get("capture_clock_uncertainty_ns"),
                        "dds_causal_receipt_age_ms":ages,"dds_samples":selected,
                        "synchronized_sample_valid":bool(valid and partner and all(selected.values()))})
    if invalid_clocks:
        report["issues"].append("invalid camera clock samples: "+str(invalid_clocks))
    if stale_matches:
        report["issues"].append("missing/stale DDS matches: "+str(stale_matches))
    report["camera_dds_matches"] = matches
    report["incomplete"] = bool(report["issues"])
    (Path(episode)/"sync_audit.json").write_text(json.dumps(report,indent=2))
    return report
