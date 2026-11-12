"""sim/gpu.py preflight verdicts and messages; tools/sim_perf.py parsing of SONIC's Loop timing lines."""
from xr_camera_teleop.sim import gpu
from xr_camera_teleop.tools.sim_perf import sonic_timing


def test_preflight_flags_a_stale_budget_and_says_why():
    assert gpu.Preflight(150.0).ok and gpu.Preflight(None).ok
    bad = gpu.Preflight(20.0, ["kernel: NVRM: RmHandleDNotifierEvent: Failed to handle ACPI D-Notifier event"])
    assert not bad.ok and "20 W" in bad.message() and "reboot" in bad.message()
    assert "D-Notifier" in bad.message()
    assert "not readable" in gpu.Preflight(20.0, None).message()
    assert "No NVRM" in gpu.Preflight(20.0, []).message()


def test_sonic_timing_lines():
    line = ("Loop timing - LowState age: 1ms, Obs: 90us, Policy: 1200us, Obs 2 Motor Command: 2500us, "
            "Post processing: 9us, Planner - Gather Input: 3us, Model: 9000us, Convert50Hz: 4us, Total: 9100us\n")
    r = sonic_timing(line * 3 + "noise\n")
    assert r["sonic_timing_lines"] == 3 and r["sonic_policy_ms_median"] == 1.2
    assert r["sonic_obs2cmd_ms_p99"] == 2.5 and r["sonic_planner_model_ms_median"] == 9.0
    assert sonic_timing("nothing") == {}
