import pytest
from xr_camera_teleop.recording.timing import HeaderClockBridge,causal_index
from xr_camera_teleop.recording.clocks import ClockMapper,ClockProbe


def pair(mono,unix,span=1000):
    return {"clock_pair_monotonic_ns":mono,"collector_ros_now_ns":unix,
            "clock_pair_unix_ns":unix,"clock_pair_span_ns":span,"clock_pair_unix_span_ns":span}


def test_camera_mapping_preserves_known_capture_time_despite_callback_delay():
    bridge = HeaderClockBridge()
    mapped = bridge.map(100_000_000_000,pair(5_020_000_000,100_020_000_000))
    assert mapped["capture_monotonic_ns"] == 5_000_000_000
    assert mapped["capture_clock_uncertainty_ns"] == 500
    assert mapped["capture_clock_valid"]
    # Sample from 3 ms after capture must NOT be associated with the image.
    samples = [4_995_000_000,5_003_000_000]
    assert causal_index(samples,mapped["capture_monotonic_ns"],50_000_000) == 0
    assert causal_index([5_003_000_000],mapped["capture_monotonic_ns"],50_000_000) is None


@pytest.mark.parametrize("header,measurement,sim",[
    (0,pair(5_000_000_000,100_000_000_000),False),
    (100_000_000_001,pair(5_000_000_000,100_000_000_000),False),
    (99_000_000_000,pair(5_000_000_000,100_000_000_000),False),
    (100_000_000_000,pair(5_000_000_000,100_000_000_000,10_000_000),False),
    (100_000_000_000,pair(5_000_000_000,100_000_000_000),True),
])
def test_invalid_clock_samples_never_produce_capture_time(header,measurement,sim):
    value = HeaderClockBridge().map(header,measurement,sim_time=sim)
    assert not value["capture_clock_valid"] and value["capture_monotonic_ns"] is None


def test_wall_clock_step_rejects_header_from_previous_epoch():
    bridge = HeaderClockBridge()
    assert bridge.map(100_000_000_000,pair(5_000_000_000,100_000_000_000))["capture_clock_valid"]
    result = bridge.map(100_010_000_000,pair(5_010_000_000,100_110_000_000))
    assert not result["capture_clock_valid"]
    assert "image crosses a wall-clock step" in result["capture_clock_errors"]
    fresh = bridge.map(100_120_000_000,pair(5_030_000_000,100_130_000_000))
    assert fresh["capture_clock_valid"]


def test_state_matching_has_age_limit_and_clock_mapping_round_trip():
    assert causal_index([100],1000,100) is None
    mapper = ClockMapper()
    mapper.add("one",ClockProbe(11_000_000_000,1_001_000_000,1_001_100_000,11_002_100_000))
    assert mapper.to_remote(mapper.to_local(2_000_000_000)) == 2_000_000_000
