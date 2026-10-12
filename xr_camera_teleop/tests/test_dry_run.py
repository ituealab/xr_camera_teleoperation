"""The offline backend must never reach DDS, including shutdown."""
import builtins
import sys
import numpy as np
import pytest
from xr_camera_teleop.protocol import RobotStatus, pack_status, unpack_status
from xr_camera_teleop.robot.node import RobotConfig, build_hardware, parse_args
from xr_camera_teleop.robot.dry_run import DryRunArm


def test_dry_run_never_imports_hardware(monkeypatch):
    from xr_camera_teleop.robot import ik
    marker = object()
    monkeypatch.setattr(ik, "make_arm_ik", lambda **kw: marker)
    original = builtins.__import__
    def guarded(name, *args, **kwargs):
        assert not name.startswith(("unitree_sdk2py", "cyclonedds", "dex_retargeting"))
        assert name not in ("xr_camera_teleop.robot.vendor.robot_arm",
                            "xr_camera_teleop.robot.vendor.robot_hand_unitree",
                            "xr_camera_teleop.robot.vendor.motion_switcher", "xr_camera_teleop.robot.camera")
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, "__import__", guarded)
    arm, hand, solver = build_hardware(RobotConfig(dry_run=True, hands=True, robot_camera=True))
    assert isinstance(arm, DryRunArm) and hand is None and solver is marker
    arm.ctrl_dual_arm(np.ones(14), np.zeros(14))
    assert np.allclose(arm.target, 1)
    arm.ctrl_dual_arm_go_home()
    arm.stop()
    assert np.allclose(arm.q, 0)


@pytest.mark.parametrize("flag", ["--motion", "--sim", "--debug", "--robot-camera"])
def test_dry_run_rejects_conflicting_modes(flag):
    with pytest.raises(SystemExit):
        parse_args(["--dry-run", flag])


def test_dry_run_status_roundtrip():
    assert unpack_status(pack_status(RobotStatus(session=1, seq=1, mode="hold", dry_run=True))).dry_run


def test_virtual_arm_keeps_converging_after_last_command():
    now = [0.0]
    arm = DryRunArm(30, 1.0, clock=lambda: now[0])
    arm.ctrl_dual_arm(np.ones(14), np.zeros(14))
    now[0] = 0.25
    assert np.allclose(arm.get_current_dual_arm_q(), 0.25)
    now[0] = 1.5
    assert np.allclose(arm.get_current_dual_arm_q(), 1.0)
    arm.ctrl_dual_arm(np.zeros(14), np.zeros(14))
    now[0] = 3.0
    assert np.allclose(arm.get_current_dual_arm_q(), 0.0)


def test_real_robot_requires_explicit_control_path():
    with pytest.raises(SystemExit):
        parse_args([])
    assert parse_args(["--motion", "--prepare"]).prepare


def test_debug_requires_explicit_flag_and_excludes_motion():
    assert parse_args(["--debug", "--prepare"]).debug
    with pytest.raises(SystemExit):
        parse_args(["--debug", "--motion"])
    with pytest.raises(SystemExit):
        parse_args(["--debug", "--sim"])


def test_real_hardware_cannot_disable_startup_collision_check():
    with pytest.raises(SystemExit):
        parse_args(["--debug", "--no-self-collision"])


def test_startup_contact_override_requires_home_preparation():
    with pytest.raises(SystemExit):
        parse_args(["--debug", "--allow-startup-contact"])
    assert parse_args(["--debug", "--prepare", "--allow-startup-contact"]).allow_startup_contact
