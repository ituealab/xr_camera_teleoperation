"""Pure timing/synchronization analyzer for recorded episodes.

Operates on the recorder's lightweight timing index (index.jsonl), not the bag
itself, so it runs on MSI without ROS. It answers the two separate questions:
  (1) do camera and robot times share one verified clock domain?
  (2) is a sufficiently recent robot state available for each camera frame?

It never claims zero physical latency; software clock agreement is not sensor truth.
"""
from bisect import bisect_right
from collections import defaultdict

STATE_TOPICS = ("rt/lowstate", "rt/dex3/left/state", "rt/dex3/right/state")
ACTION_TOPICS = ("rt/lowcmd", "rt/arm_sdk", "rt/dex3/left/cmd", "rt/dex3/right/cmd")


def _camera_time(record):
    if record.get("capture_clock_valid") and record.get("capture_monotonic_ns") is not None:
        return record["capture_monotonic_ns"]
    return None


def _sample_time(record):
    if record.get("is_image"):
        return _camera_time(record)
    return record.get("collector_monotonic_ns")


def build_report(records, left_topic, right_topic, limits=None,
                 state_topics=STATE_TOPICS, action_topics=ACTION_TOPICS, require_actions=False):
    limits = {"state_age_ms": 20.0, "action_age_ms": 100.0, "pair_ratio": 0.99,
              "required_rate_ratio": 0.7, "expected_rates_hz": {}} if limits is None else limits
    state_topics = tuple(state_topics)
    action_topics = tuple(action_topics)
    report = {"format": "xrcam-sync-report-v1", "issues": [], "sources": {},
              "clock": {"camera_clock_invalid_frames": 0, "clock_epochs": [], "verified": False},
              "dds_time_semantics": "listener_receipt_not_verified_sensor_measurement",
              "physical_latency_verified": False,
              "limits": limits}
    by_topic = defaultdict(list)
    for record in records:
        by_topic[record["topic"]].append(record)
    def _boot(record):
        return record.get("boot_id") or record.get("collector_boot_id")
    boots = {_boot(record) for record in records}
    report["clock"]["boot_ids"] = sorted(value for value in boots if value)
    same_boot = len(boots) == 1 and None not in boots
    if not records:
        report["issues"].append("no timing records")
    if not same_boot:
        report["issues"].append("missing or multiple collector boot ids; monotonic epochs not comparable")
    for topic, values in by_topic.items():
        gaps, regressions = [], 0
        times = []
        for previous, current in zip(values, values[1:]):
            if (previous.get("session_id") == current.get("session_id")
                    and current.get("sequence", 0) != previous.get("sequence", -1) + 1):
                gaps.append([previous.get("sequence"), current.get("sequence")])
        for previous, current in zip(values, values[1:]):
            a, b = previous.get("collector_monotonic_ns"), current.get("collector_monotonic_ns")
            if a is not None and b is not None:
                if b < a:
                    regressions += 1
                times.append(a)
        if values:
            last = values[-1].get("collector_monotonic_ns")
            if last is not None:
                times.append(last)
        rate = (len(times) - 1) * 1e9 / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else None
        report["sources"][topic] = {"samples": len(values), "sequence_gaps": gaps,
                                    "time_regressions": regressions, "receipt_hz": rate}
        if gaps or regressions:
            report["issues"].append("sequence/time discontinuity: " + topic)
    left = by_topic.get(left_topic, [])
    right = by_topic.get(right_topic, [])
    if not left or not right:
        report["issues"].append("required stereo source missing")
    for frame in left + right:
        if not frame.get("capture_clock_valid", False):
            report["clock"]["camera_clock_invalid_frames"] += 1
        report["clock"]["clock_epochs"].append(frame.get("capture_clock_epoch"))
    def identity(record):
        return (_boot(record), record.get("source_timestamp_ns"))
    left_ids = {identity(frame) for frame in left if frame.get("source_timestamp_ns")}
    right_ids = {identity(frame) for frame in right if frame.get("source_timestamp_ns")}
    paired = left_ids & right_ids
    if len(left_ids)!=len(left) or len(right_ids)!=len(right):
        report["issues"].append("duplicate or missing camera header timestamps")
    report["stereo"] = {"left": len(left), "right": len(right), "pairs": len(paired),
                        "left_only": len(left_ids - right_ids), "right_only": len(right_ids - left_ids)}
    valid_left = [frame for frame in left if same_boot and _camera_time(frame) is not None]
    if not valid_left:
        report["issues"].append("no camera frames with a verified clock")
    if left and right:
        ratio = len(paired) / max(1, len(left_ids | right_ids))
        report["stereo"]["pair_ratio"] = ratio
        if ratio < limits["pair_ratio"]:
            report["issues"].append("stereo pairing below limit: %.3f" % ratio)
    state_series, action_series = {}, {}
    for topic, values in by_topic.items():
        times = sorted(_sample_time(record) for record in values if _sample_time(record) is not None)
        if topic in state_topics:
            state_series[topic] = times
        elif topic in action_topics:
            action_series[topic] = times
    for topic in state_topics:
        if topic not in state_series:
            report["issues"].append("required state source missing: " + topic)
    ages, matched, unmatched = [], 0, 0
    match_rows = []
    for frame in valid_left:
        when = _camera_time(frame)
        row = {"sequence": frame.get("sequence"), "capture_monotonic_ns": when, "state_ages_ms": {}, "action_ages_ms": {}}
        complete = True
        for topic in state_topics:
            series = state_series.get(topic)
            if not series:
                row["state_ages_ms"][topic] = None
                complete = False
                continue
            index = bisect_right(series, when) - 1
            age_ms = (when - series[index]) / 1e6 if index >= 0 else None
            row["state_ages_ms"][topic] = age_ms
            if age_ms is None or age_ms > limits["state_age_ms"]:
                complete = False
            if age_ms is not None:
                ages.append(age_ms)
        for topic, series in action_series.items():
            index = bisect_right(series, when) - 1
            row["action_ages_ms"][topic] = (when - series[index]) / 1e6 if index >= 0 else None
        match_rows.append(row)
        if complete:
            matched += 1
        else:
            unmatched += 1
    ages.sort()
    def percentile(p):
        return ages[min(len(ages) - 1, int(len(ages) * p))] if ages else None
    report["camera_state_match"] = {"frames": len(valid_left), "matched": matched, "unmatched": unmatched,
                                    "state_age_ms_median": percentile(.5), "state_age_ms_p95": percentile(.95),
                                    "state_age_ms_max": ages[-1] if ages else None,
                                    "matched_ratio": matched / len(valid_left) if valid_left else 0.0,
                                    "rows": match_rows}
    if valid_left and unmatched:
        report["issues"].append("frames without a recent-enough state: %d" % unmatched)
    missing_actions = [topic for topic in action_topics if topic not in action_series]
    stale_action_frames = sum(any(row["action_ages_ms"].get(topic) is None or
        row["action_ages_ms"][topic]>limits["action_age_ms"] for topic in action_topics) for row in match_rows)
    report["actions"] = {"required":require_actions,"missing_topics":missing_actions,"stale_or_missing_frames":stale_action_frames}
    if require_actions and (missing_actions or stale_action_frames):
        report["issues"].append("action data missing/stale (separate from state timing)")
    expected = limits.get("expected_rates_hz", {})
    for topic, expected_hz in expected.items():
        actual = report["sources"].get(topic, {}).get("receipt_hz")
        if actual is None or actual < expected_hz * limits["required_rate_ratio"]:
            report["issues"].append("rate below expectation: %s (%.1f < %.1f)" % (topic, actual or 0, expected_hz * limits["required_rate_ratio"]))
    epochs = set(report["clock"]["clock_epochs"])-{None}
    if len(epochs)>1:
        report["issues"].append("camera clock epoch changed during episode")
    report["clock"]["verified"] = bool(same_boot and left and right and len(epochs)<=1
                                           and report["clock"]["camera_clock_invalid_frames"] == 0)
    if not report["clock"]["verified"]:
        report["issues"].append("camera/system clock not verified for all frames")
    report["incomplete"] = bool(report["issues"])
    report["verdict"] = "ok" if not report["incomplete"] else "issues"
    return report
