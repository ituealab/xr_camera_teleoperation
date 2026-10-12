import numpy as np
import pytest
from xr_camera_teleop.robot.limits import JointTargetLimiter


def test_low_speed_command_advances_when_motor_initially_sticks():
    limiter = JointTargetLimiter(np.ones(14))
    measured = np.ones(14)
    first = limiter.step(np.zeros(14), measured, .3, .004)
    for _ in range(9):
        last = limiter.step(np.zeros(14), measured, .3, .004)
    assert np.allclose(first, .9988)
    assert np.allclose(last, .988)


def test_stalled_motor_lead_is_bounded_instead_of_accumulating_large_torque():
    limiter = JointTargetLimiter(np.ones(14))
    for _ in range(250):
        q = limiter.step(np.zeros(14), np.ones(14), .3, .004)
    assert np.allclose(q, .95)


def test_tracking_motor_reaches_home_with_bounded_command_steps():
    measured = np.ones(14)
    limiter = JointTargetLimiter(measured)
    for _ in range(1000):
        q = limiter.step(np.zeros(14), measured, .3, .004)
        assert np.max(np.abs(q - measured)) <= .0012 + 1e-12
        measured = q
    assert np.allclose(measured, 0)


def test_direction_change_and_near_target_do_not_overshoot():
    limiter = JointTargetLimiter(np.zeros(14))
    assert np.allclose(limiter.step(np.ones(14), np.zeros(14), .3, .004), .0012)
    assert np.allclose(limiter.step(-np.ones(14), np.zeros(14), .3, .004), 0)
    assert np.allclose(limiter.step(np.full(14, .0005), np.zeros(14), .3, .004), .0005)


def test_invalid_target_is_rejected():
    limiter = JointTargetLimiter(np.zeros(14))
    with pytest.raises(ValueError):
        limiter.step(np.full(14, np.nan), np.zeros(14), .3, .004)


def test_preview_arm_speed_applies_independently_per_joint():
    limiter = JointTargetLimiter(np.zeros(2))
    # A large shoulder target must not slow a small wrist target.
    out = limiter.step(np.array([2.0, .005]), np.zeros(2), 4., .004)
    assert np.allclose(out, [.016, .005])


def test_hand_preview_speed_reaches_target_without_overshoot():
    from xr_camera_teleop.robot.limits import slew_targets
    q = np.zeros(7)
    target = np.array([.04, .1, .3, -.3, .5, -.5, 0])
    first = slew_targets(q, target, 8., .01)
    assert np.allclose(first, [.04, .08, .08, -.08, .08, -.08, 0])
    for _ in range(10):
        q = slew_targets(q, target, 8., .01)
    assert np.allclose(q, target)
