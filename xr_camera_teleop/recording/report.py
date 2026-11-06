"""Episode quality report from an MCAP bag.

Camera headers carry PC2 capture time and every log time is receipt on MSI; the clock
guard bounds the difference. Native Unitree messages have no sensor stamp, so state and
action times are receipt times, and no timestamp is ever shifted or invented.
"""
from bisect import bisect_right
from collections import defaultdict
from pathlib import Path
import json
import statistics
import struct
import subprocess
import sys

from mcap.exceptions import McapError
from mcap.reader import NonSeekingReader

TICK_OFFSET = 16  # /lowstate tick, one per ms: u32 after the CDR header, version and modes.


def bag_file(directory):
    return next(Path(directory).glob("*.mcap"))


def _messages(path):
    """Stream the file: this needs no footer, so a crashed recorder's bag still reads
    up to its last whole chunk. Yields (channel, message) and returns True at the footer."""
    with Path(path).open("rb") as file:
        try:
            for _,channel,message in NonSeekingReader(file).iter_messages(log_time_order=False):
                yield channel,message
        except (McapError,EOFError,struct.error):
            return False
    return True


def topics_with_messages(directory,wanted):
    """Which wanted topics already have messages in a bag that is still being written."""
    seen = set()
    try:
        path = bag_file(directory)
    except StopIteration:
        return seen
    for channel,_ in _messages(path):
        seen.add(channel.topic)
        if wanted<=seen:
            break
    return seen


def read_bag(path,camera_topics,tick_topic):
    times,headers,ticks = defaultdict(list),defaultdict(list),[]
    messages = _messages(path)
    closed = False
    while True:
        try:
            channel,message = next(messages)
        except StopIteration as end:
            closed = end.value
            break
        times[channel.topic].append(message.log_time)
        if channel.topic in camera_topics:
            headers[channel.topic].append((message.log_time,_image_stamp(message.data)))
        elif channel.topic==tick_topic:
            ticks.append(struct.unpack_from("<I",message.data,TICK_OFFSET)[0])
    return {"times":times,"headers":headers,"ticks":ticks,"closed":closed}


def _image_stamp(data):
    if len(data)<12 or data[:2] not in (b"\x00\x01",b"\x00\x00"):
        return None
    sec,ns = struct.unpack_from(("<" if data[1]==1 else ">")+"iI",data,4)
    return sec*1_000_000_000+ns if sec>=0 and ns<1_000_000_000 else None


def _stats(values):
    ordered = sorted(values)
    return {"median":statistics.median(ordered),"p95":ordered[min(len(ordered)-1,int(len(ordered)*.95))],
            "max":ordered[-1]} if ordered else {"median":None,"p95":None,"max":None}


def _gaps_ms(ordered):
    return [(b-a)/1e6 for a,b in zip(ordered,ordered[1:])]


def _hz(ordered):
    span = (ordered[-1]-ordered[0])/1e9 if ordered else 0
    return (len(ordered)-1)/span if span>0 else None


def _source(values):
    ordered = sorted(values)
    return {"samples":len(ordered),"hz":_hz(ordered),"max_gap_ms":max(_gaps_ms(ordered),default=None)}


def _ages_ms(times,when):
    index = bisect_right(times,when)-1
    return (when-times[index])/1e6 if index>=0 else None


def _tick_loss(ticks):
    """The robot's own counter shows real loss, which receipt times alone cannot."""
    if not ticks:
        return None
    unique = sorted(set(ticks))
    span = unique[-1]-unique[0]+1
    return {"expected":span,"received":len(unique),"missing_percent":100*(span-len(unique))/span,
            "longest_gap_ms":max((b-a-1 for a,b in zip(unique,unique[1:])),default=0)}


def _camera(frames,expected_hz,max_lag_ms):
    valid,lags,invalid = [],[],0
    for received,stamp in frames:
        lag = (received-stamp)/1e6 if stamp else None
        if lag is None or lag<0 or lag>max_lag_ms:
            invalid += 1
        else:
            valid.append(stamp)
            lags.append(lag)
    ordered = sorted(valid)
    return ordered,{"frames":len(frames),"invalid_or_implausible_stamps":invalid,"hz":_hz(ordered),
                    "expected_hz":expected_hz,"max_gap_ms":max(_gaps_ms(ordered),default=None),
                    "duplicate_stamps":len(ordered)-len(set(ordered)),"capture_to_recorder_ms":_stats(lags)}


def build_report(directory,config):
    """Pure summary of one bag; config is the recording config."""
    limits = config["limits"]
    cameras,state_topics,action_topics = config["camera_topics"],config["state_topics"],config["action_topics"]
    bag = read_bag(bag_file(directory),cameras,config["tick_topic"])
    times = {topic:sorted(values) for topic,values in bag["times"].items()}
    issues = []
    if not bag["closed"]:
        issues.append("bag has no footer: recorder did not close it; data up to the last chunk is readable")
    for topic in config["topics"]:
        if topic not in times:
            issues.append("topic has no messages: "+topic)
    report = {"format":"xrcam-episode-report-v1","time_basis":"camera capture on PC2 vs receipt on MSI",
              "sources":{topic:_source(values) for topic,values in times.items()},"cameras":{},
              "state_ticks":_tick_loss(bag["ticks"]),"closed":bag["closed"],"issues":issues}
    ticks = report["state_ticks"]
    if ticks and ticks["missing_percent"]>limits["tick_missing_percent"]:
        issues.append("state ticks missing: %.2f%%" % ticks["missing_percent"])
    for topic in cameras:
        expected = config["expected_rates_hz"].get(topic)
        stamps,summary = _camera(bag["headers"].get(topic,[]),expected,limits["max_camera_lag_ms"])
        report["cameras"][topic] = summary
        if summary["invalid_or_implausible_stamps"]:
            issues.append("implausible camera stamps: %s (%d frames)" % (topic,summary["invalid_or_implausible_stamps"]))
        if summary["duplicate_stamps"]:
            issues.append("duplicate camera stamps: "+topic)
        if expected and (summary["hz"] or 0)<expected*limits["rate_ratio"]:
            issues.append("camera rate below limit: %s (%s Hz)" % (topic,summary["hz"]))
        states = [times[name] for name in state_topics if name in times]
        actions = [times[name] for name in action_topics if name in times]
        # Frames before every source has started or after one has stopped are episode
        # edges, not sync faults.
        first = max((series[0] for series in [stamps]+states if series),default=0)
        last = min((series[-1] for series in [stamps]+states if series),default=0)
        inside = [when for when in stamps if first<=when<=last]
        state_ages = [[_ages_ms(series,when) for series in states] for when in inside]
        action_ages = [[_ages_ms(series,when) for series in actions] for when in inside]
        late = sum(any(age is None or age>limits["state_age_ms"] for age in ages) or len(ages)<len(state_topics)
                   for ages in state_ages)
        stale = sum(any(age is None or age>limits["action_age_ms"] for age in ages) or len(ages)<len(action_topics)
                    for ages in action_ages)
        summary.update(frames_in_window=len(inside),frames_without_recent_state=late,
                       frames_without_recent_action=stale,
                       state_age_ms=_stats(age for ages in state_ages for age in ages if age is not None),
                       action_age_ms=_stats(age for ages in action_ages for age in ages if age is not None))
        if late:
            issues.append("frames without recent state: %s (%d)" % (topic,late))
        if config["require_actions"] and stale:
            issues.append("frames without recent action: %s (%d)" % (topic,stale))
    report["incomplete"] = bool(issues)
    report["verdict"] = "issues" if issues else "ok"
    return report


def build_report_isolated(directory,config,timeout=1800):
    """A niced child, so reading a long bag cannot stall the UI process or the next episode."""
    result = subprocess.run(["nice","-n","10",sys.executable,"-m","xr_camera_teleop.recording.report"],
                            input=json.dumps({"directory":str(directory),"config":config}).encode(),
                            stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                            cwd=str(Path(__file__).resolve().parents[2]),timeout=timeout)
    if result.returncode:
        raise RuntimeError("report failed: "+result.stderr.decode(errors="replace")[-500:])
    return json.loads(result.stdout)


if __name__=="__main__":
    request = json.load(sys.stdin)
    json.dump(build_report(request["directory"],request["config"]),sys.stdout)
