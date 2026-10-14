import pytest
from xr_camera_teleop.recording.clocks import ClockMapper, ClockProbe
from xr_camera_teleop.recording.telemetry import positions


def test_remote_monotonic_epoch_and_network_delay():
    mapper = ClockMapper()
    # Local clock is remote + 10s; symmetric one-way transport 1ms.
    mapper.add("one", ClockProbe(11_000_000_000, 1_001_000_000,
                                 1_001_100_000, 11_002_100_000))
    assert mapper.to_local(2_000_000_000) == 12_000_000_000
    assert mapper.calibration()["uncertainty_estimate_ns"] == 1_000_000
    mapper.add("two", ClockProbe(30, 10, 10, 30))
    assert mapper.calibration()["probe_count"] == 1
    assert mapper.to_local(15) == 35


def test_invalid_clock_probe_rejected():
    with pytest.raises(ValueError):
        ClockMapper().add("x", ClockProbe(0, 20, 50, 10))


def test_extraction_uses_actual_arm_motor_offsets():
    data = {"motor_state": [{"q": i+.5} for i in range(35)]}
    assert positions(data, arm=True) == [i+.5 for i in range(15,29)]
    assert positions({"motor_cmd": [{"q": i} for i in range(7)]}, command=True) == list(range(7))
