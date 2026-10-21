"""Plain rosbag analysis using camera header vs recorder receipt Unix time.

Native Unitree messages have no verified sensor measurement header. This report
cannot recover publisher sequences/boot identity or certify physical clock sync.
No timestamps are shifted, invented or relabelled as monotonic time.
"""
from bisect import bisect_right
from collections import defaultdict
from pathlib import Path
import statistics
import struct

from .bag_storage import database_paths,open_readonly

IMAGE_TYPES = ("sensor_msgs/msg/Image","sensor_msgs/msg/CompressedImage")


def _image_header(data):
    if data is None or len(data)<12 or data[:2] not in (b"\x00\x01",b"\x00\x00"):
        raise ValueError("invalid/truncated Image CDR header")
    endian = "<" if data[1]==1 else ">"
    sec,ns = struct.unpack_from(endian+"iI",data,4)
    if sec<0 or ns>=1_000_000_000:
        raise ValueError("invalid ROS stamp")
    return sec*1_000_000_000+ns


def read_records(directory):
    records = []
    paths,_ = database_paths(directory)
    for path in paths:
        connection = open_readonly(path)
        try:
            topics = connection.execute("SELECT id,name,type FROM topics").fetchall()
            for topic_id,topic,kind in topics:
                image = kind in IMAGE_TYPES
                field = "substr(data,1,12)" if image else "NULL"
                query = "SELECT timestamp,%s FROM messages WHERE topic_id=? ORDER BY id" % field
                for received,prefix in connection.execute(query,(topic_id,)):
                    record = {"topic":topic,"ros_type":kind,"is_image":image,
                              "bag_receive_unix_ns":received,"source_sequence":None}
                    if image:
                        try:
                            record["camera_header_unix_ns"] = _image_header(prefix)
                        except (ValueError,struct.error) as exc:
                            record["camera_header_unix_ns"] = None
                            record["header_error"] = str(exc)
                    records.append(record)
        finally:
            connection.close()
    return records


def _stats(values):
    ordered = sorted(values)
    return {"median":statistics.median(ordered),"p95":ordered[min(len(ordered)-1,int(len(ordered)*.95))],
            "max":ordered[-1]} if ordered else {"median":None,"p95":None,"max":None}


def build_plain_report(directory,left_topic,right_topic=None,limits=None,state_topics=(),action_topics=(),
                       require_actions=False,context=None):
    settings = {"state_age_ms":20.,"action_age_ms":100.,"pair_ratio":.99,
                "required_rate_ratio":.7,"expected_rates_hz":{},"max_camera_lag_ms":2000.}
    settings.update(limits or {})
    records = read_records(directory)
    by_topic = defaultdict(list)
    for record in records:
        by_topic[record["topic"]].append(record)
    issues = []
    report = {"format":"xrcam-plain-sync-v2","plain_rosbag_report":True,
              "comparison_time_basis":"camera_header_unix_vs_native_rosbag_receipt_unix",
              "physical_latency_verified":False,"limits":settings,"sources":{},"issues":issues,
              "limitations":["Native state/action times are recorder receipt times, not verified sensor/publication times.",
                             "Publisher sequence and missed-before-receipt messages are not observable from this bag.",
                             "Header/receipt plausibility does not independently verify a physical camera clock."],
              "clock":{"verified":None,"assessment":"not_independently_observable_from_plain_bag",
                       "recorder_boot_id":(context or {}).get("recorder_boot_id",(context or {}).get("boot_id")),
                       "source_boot_identity_recorded":False,"camera_clock_invalid_frames":0}}
    if not records:
        issues.append("bag contains no messages")
    for topic,values in by_topic.items():
        received = [v["bag_receive_unix_ns"] for v in values]
        regressions = sum(b<a for a,b in zip(received,received[1:]))
        ordered = sorted(received)
        span = (ordered[-1]-ordered[0])/1e9 if ordered else 0
        gaps = [(b-a)/1e6 for a,b in zip(ordered,ordered[1:])]
        report["sources"][topic] = {"samples":len(values),"receipt_hz":(len(values)-1)/span if span>0 else None,
            "receipt_span_seconds":span,"max_receipt_gap_ms":max(gaps) if gaps else None,
            "time_regressions":regressions,"sequence_gaps":None,"sequence_observable":False}
        if regressions:
            issues.append("recorder wall time regression: "+topic)
    camera_topics = [left_topic]+([right_topic] if right_topic else [])
    report["camera_mode"] = "stereo" if right_topic else "mono"
    valid_images = {}
    for topic in camera_topics:
        values = by_topic.get(topic,[])
        good,lags,bad = [],[],0
        for frame in values:
            header = frame.get("camera_header_unix_ns")
            lag = (frame["bag_receive_unix_ns"]-header)/1e6 if header is not None else None
            if header is None or header<=0 or lag<0 or lag>settings["max_camera_lag_ms"]:
                bad += 1
            else:
                good.append(header)
                lags.append(lag)
        valid_images[topic] = sorted(good)
        report["clock"]["camera_clock_invalid_frames"] += bad
        source = report["sources"].setdefault(topic,{"samples":0,"sequence_observable":False,"sequence_gaps":None})
        ordered_headers = sorted(v["camera_header_unix_ns"] for v in values if v.get("camera_header_unix_ns") is not None)
        span = (ordered_headers[-1]-ordered_headers[0])/1e9 if len(ordered_headers)>1 else 0
        source.update(valid_header_frames=len(good),invalid_or_implausible_headers=bad,
                      header_hz=(len(ordered_headers)-1)/span if span>0 else None,
                      max_header_gap_ms=max((b-a)/1e6 for a,b in zip(ordered_headers,ordered_headers[1:])) if len(ordered_headers)>1 else None,
                      capture_to_recorder_ms=_stats(lags),duplicate_headers=len(ordered_headers)-len(set(ordered_headers)))
        if not values:
            issues.append("required image topic missing: "+topic)
        if bad:
            issues.append("invalid/future/stale camera header: %s (%s frames)" % (topic,bad))
        if source["duplicate_headers"]:
            issues.append("duplicate image headers: "+topic)
    report["clock"]["header_receipt_plausible"] = bool(all(valid_images[t] for t in camera_topics)
                                                       and not report["clock"]["camera_clock_invalid_frames"])
    left_ids,right_ids = set(valid_images[left_topic]),set(valid_images.get(right_topic,[]))
    paired = left_ids&right_ids
    ratio = len(paired)/len(left_ids|right_ids) if left_ids|right_ids else 0
    report["stereo"] = {"enabled":bool(right_topic),"left":len(by_topic.get(left_topic,[])),"right":len(by_topic.get(right_topic,[])),
        "pairs":len(paired),"left_only":len(left_ids-right_ids),"right_only":len(right_ids-left_ids),
        "pair_ratio":ratio,"pair_ratio_definition":"intersection_over_union_of_distinct_plausible_headers"}
    if not right_topic:
        report["stereo"].update(pairs=None,left_only=None,right_only=None,pair_ratio=None)
    if right_topic and ratio<settings["pair_ratio"]:
        issues.append("stereo pairing below limit: %.3f" % ratio)
    state_topics,action_topics = tuple(state_topics),tuple(action_topics)
    series = {topic:sorted(v["bag_receive_unix_ns"] for v in by_topic.get(topic,[]))
              for topic in set(state_topics+action_topics)}
    for topic in state_topics:
        if not series[topic]:
            issues.append("required state topic missing: "+topic)
    common_sources = [valid_images[t] for t in camera_topics]+[series[t] for t in state_topics]
    if common_sources and all(common_sources):
        start,end = max(v[0] for v in common_sources),min(v[-1] for v in common_sources)
        window = (start,end) if start<=end else None
    else:
        window = None
    report["common_time_window"] = {"start_unix_ns":window[0],"end_unix_ns":window[1],
                                   "duration_seconds":(window[1]-window[0])/1e9} if window else None
    if window is None:
        issues.append("camera and required state have no common time window")
    ages,action_ages,rows = [],[],[]
    matched,action_bad,edge_frames,common_matched = 0,0,0,0
    for when in valid_images[left_topic]:
        row = {"camera_header_unix_ns":when,"state_ages_ms":{},"action_ages_ms":{},
               "outside_common_window":window is None or not window[0]<=when<=window[1]}
        for topics,key,aggregate in ((state_topics,"state_ages_ms",ages),(action_topics,"action_ages_ms",action_ages)):
            for topic in topics:
                times = series[topic]
                index = bisect_right(times,when)-1
                age = (when-times[index])/1e6 if index>=0 else None
                row[key][topic] = age
                if age is not None:
                    aggregate.append(age)
        state_ok = bool(state_topics) and all(v is not None and v<=settings["state_age_ms"] for v in row["state_ages_ms"].values())
        action_ok = bool(action_topics) and all(v is not None and v<=settings["action_age_ms"] for v in row["action_ages_ms"].values())
        matched += state_ok
        action_bad += not action_ok
        edge_frames += row["outside_common_window"]
        common_matched += state_ok and not row["outside_common_window"]
        rows.append(row)
    age_stats = _stats(ages)
    report["camera_state_match"] = {"frames":len(rows),"matched":matched,"unmatched":len(rows)-matched,
        "state_age_ms_median":age_stats["median"],"state_age_ms_p95":age_stats["p95"],"state_age_ms_max":age_stats["max"],
        "matched_ratio":matched/len(rows) if rows else 0,"startup_or_end_edge_frames":edge_frames,
        "common_window_frames":len(rows)-edge_frames,"common_window_matched":common_matched,
        "time_semantics":"causal camera header vs recorder state receipt (not sensor ground truth)","rows":rows}
    if len(rows)>matched:
        issues.append("frames without recent-enough state: %s" % (len(rows)-matched))
    report["actions"] = {"required":require_actions,"missing_topics":[t for t in action_topics if not series[t]],
                         "stale_or_missing_frames":action_bad,"age_ms":_stats(action_ages)}
    if require_actions and (not action_topics or action_bad):
        issues.append("action data missing/stale (separate from state timing)")
    for topic,expected in settings["expected_rates_hz"].items():
        source = report["sources"].setdefault(topic,{"samples":0,"sequence_observable":False,"sequence_gaps":None})
        # Camera cadence uses original headers; receipt batching must not be
        # mistaken for capture cadence. Native messages expose receipt only.
        camera = any(v["is_image"] for v in by_topic.get(topic,[]))
        basis = "header_hz" if camera else "receipt_hz"
        measured = source.get(basis)
        minimum = expected*settings["required_rate_ratio"]
        source.update(expected_hz=expected,minimum_hz=minimum,rate_basis=basis,
                      rate_below_limit=measured<minimum if measured is not None else None)
        if measured is None:
            issues.append("configured topic rate unavailable: "+topic)
        elif measured<minimum:
            issues.append("observed rate below limit: %s (%.3f < %.3f Hz; %s)" % (topic,measured,minimum,basis))
    report["incomplete"] = bool(issues)
    report["verdict"] = "issues" if issues else "ok"
    return report
