"""Thumb path through the taught poses, on synthetic hands with exact feature values."""
import numpy as np
import pytest

from xr_camera_teleop.robot import thumb_shaping as ts

PALM = 0.07


def hand(index_gap, middle_gap, curled=False):
    """(25,3) hand: straight fingers along +x (or folded back), thumb tip at the given gaps."""
    joints = np.zeros((25, 3))
    for row, chain in enumerate(ts.FINGER_CHAINS):
        y = (1.5 - row) * PALM / 3
        for link, joint in enumerate(chain):
            joints[joint] = (0.09 + 0.03 * link, y, 0.0)
        if curled:
            joints[chain[1]] = (0.11, y, -0.03)
            joints[chain[2]] = (0.08, y, -0.04)
            joints[chain[3]] = (0.06, y, -0.02)
    index, middle = joints[ts.INDEX_TIP], joints[ts.MIDDLE_TIP]
    separation = np.linalg.norm(index - middle)
    a, b = index_gap * PALM, middle_gap * PALM
    x = (a * a - b * b + separation * separation) / (2 * separation)
    joints[ts.THUMB_TIP] = index + (middle - index) * x / separation + (0, 0, np.sqrt(max(a * a - x * x, 0)))
    return joints


@pytest.mark.parametrize("gap", [ts.OPEN_GAP, 2.0])
def test_open_hand_holds_the_open_thumb(gap):
    assert np.allclose(ts.thumb_target("left", hand(gap, gap + .1)), ts.OPEN_THUMB)


def test_cup_gap_reaches_the_cup_pose():
    assert np.allclose(ts.thumb_target("left", hand(ts.CUP_GAP, ts.CUP_GAP)), ts.CUP_THUMB)


def test_gap_narrows_continuously_from_open_to_pinch():
    gaps = np.linspace(ts.OPEN_GAP, ts.CONTACT_GAP, 40)
    path = np.array([ts.thumb_target("left", hand(g, g + .3)) for g in gaps])
    assert np.max(np.abs(np.diff(path, axis=0))) < 0.25


def test_contact_reaches_the_pinch_of_the_nearer_fingertip():
    assert np.allclose(ts.thumb_target("left", hand(.2, .5)), ts.INDEX_PINCH_THUMB)
    assert np.allclose(ts.thumb_target("left", hand(.5, .2)), ts.MIDDLE_PINCH_THUMB)


def test_a_fist_overrides_the_pinch_path():
    assert np.allclose(ts.thumb_target("left", hand(.7, .7, curled=True)), ts.FIST_THUMB)


def test_right_hand_mirrors_the_left():
    joints = hand(.2, .5)
    assert np.allclose(ts.thumb_target("right", joints), ts.thumb_target("left", joints) * (1, -1, -1))


def test_cup_keeps_dexpilot_fingers_and_open_hand_takes_the_open_pose():
    curled = np.array((0, 0, 0, -.8, -1.0, -.7, -.9))
    shaper = ts.ThumbShaper()
    assert np.allclose(shaper.shape("left", curled, hand(ts.CUP_GAP, ts.CUP_GAP))[3:], curled[3:])
    assert np.allclose(shaper.shape("left", curled, hand(2.0, 2.1))[3:], ts.OPEN_POSE["left"][3:], atol=.02)
