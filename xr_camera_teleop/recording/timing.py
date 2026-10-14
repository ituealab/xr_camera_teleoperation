"""PC2 clock-domain bridge; Python 3.8 compatible, independent of ROS/DDS.

Maps a ZED image header to the SAME kernel monotonic epoch used at DDS receipt.
Software clock checks do not measure exposure timing or DDS sensor transport latency.
"""
import time


def clock_pair(ros_now_ns):
    before = time.monotonic_ns()
    unix_before = time.time_ns()
    ros = int(ros_now_ns())
    unix_after = time.time_ns()
    after = time.monotonic_ns()
    return {"clock_pair_monotonic_ns":(before+after)//2,
            "clock_pair_unix_ns":(unix_before+unix_after)//2,
            "collector_ros_now_ns":ros,"clock_pair_span_ns":after-before,
            "clock_pair_unix_span_ns":unix_after-unix_before}


class HeaderClockBridge:
    def __init__(self, max_pair_span_ns=2_000_000, max_clock_step_ns=2_000_000,
                 max_header_lag_ns=500_000_000):
        self.max_pair_span_ns = max_pair_span_ns
        self.max_clock_step_ns = max_clock_step_ns
        self.max_header_lag_ns = max_header_lag_ns
        self.offset = None
        self.stable_since = None
        self.epoch = 0

    def map(self, header_ns, pair, sim_time=False):
        mono = pair["clock_pair_monotonic_ns"]
        ros = pair["collector_ros_now_ns"]
        offset = ros-mono
        span = pair["clock_pair_span_ns"]
        reasons = []
        if sim_time:
            reasons.append("ROS simulation time cannot align physical DDS")
        if span < 0 or span > self.max_pair_span_ns:
            reasons.append("clock pair scheduling uncertainty")
        if pair["clock_pair_unix_span_ns"] < 0:
            reasons.append("wall clock went backwards during sample")
        if abs(ros-pair["clock_pair_unix_ns"]) > self.max_pair_span_ns:
            reasons.append("ROS and system wall clocks disagree")
        if self.offset is not None and abs(offset-self.offset) > self.max_clock_step_ns:
            self.epoch += 1
            self.stable_since = mono
        if self.stable_since is None:
            self.stable_since = mono
        self.offset = offset
        lag = ros-header_ns
        if header_ns <= 0 or lag < 0 or lag > self.max_header_lag_ns:
            reasons.append("zero/future/stale image header")
        capture = header_ns-offset
        # A header predating a wall-clock jump must not be mapped with the new offset.
        if self.epoch and capture < self.stable_since:
            reasons.append("image crosses a wall-clock step")
        valid = not reasons
        return {"capture_monotonic_ns":capture if valid else None,
                "capture_clock_valid":valid,"capture_clock_errors":reasons,
                "capture_clock_epoch":self.epoch,"capture_clock_uncertainty_ns":max(0,span)//2,
                "header_receipt_lag_ns":lag,"timestamp_semantics":"zed_sdk_image_header",
                "sensor_transport_latency_verified":False}


def causal_index(times, target_ns, max_age_ns):
    from bisect import bisect_right
    index = bisect_right(times,target_ns)-1
    if index<0 or target_ns-times[index]>max_age_ns:
        return None
    return index
