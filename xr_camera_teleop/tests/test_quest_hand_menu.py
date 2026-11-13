"""operator/quest/hand_menu.py: the panel's hand gesture arms on an open left palm toward the eyes and opens only on
the right index tip touching it; fists, pinches, palms up and reaching never open it."""
import numpy as np

from xr_camera_teleop.fullbody.frames import pose
from xr_camera_teleop.fullbody.xr_frame import Hand, Head, Stamp, XRFrame
from xr_camera_teleop.operator.quest import hand_menu as hm
from xr_camera_teleop.tests.test_quest_ui import EYE, hand_joints

PALM = EYE + np.array([0.3, 0.1, -0.3])
TO_EYE = (EYE - PALM) / np.linalg.norm(EYE - PALM)


def open_left(palm=PALM, normal=TO_EYE):
    return hand_joints(palm, normal, [0.3, 0.6, 0.5], "left")


def fist(j):
    """Curl the long fingers of a hand back toward the palm (straightness ~0.3)."""
    j = j.copy()
    for a, b in hm.LONG_FINGERS:
        mc = j[a]
        for k in range(a + 1, b + 1):
            j[k] = mc + 0.25 * (j[k] - mc) * (1 if k < b else -1)
    return j


def right_index_at(point):
    j = hand_joints(PALM + [0.0, -0.25, 0.0], [0, 0, -1.0], [0.0, 1.0, 0.0], "right")
    return j + (np.asarray(point) - j[hm.INDEX_TIP])


def tap_point(left):
    return hm.palm_centre(left) + 0.015 * hm.palm_normal(left, "left")


class Op:
    def __init__(self):
        self.menu, self.t, self.seq = hm.HandMenu(), 10.0, 0
        self.left = self.right = None
        self.left_fresh = True

    def frame(self):
        self.seq += 1
        s = Stamp(self.t, self.t, self.seq, True)
        hands = {"left": Hand(), "right": Hand()}
        if self.left is not None:
            ls = s if self.left_fresh else Stamp(self.t - 1.0, self.t - 1.0, self.seq, True)
            hands["left"] = Hand(ls, pose(np.eye(3), self.left[0]), self.left, 0.08)
        if self.right is not None:
            hands["right"] = Hand(s, pose(np.eye(3), self.right[0]), self.right, 0.08)
        return XRFrame(seq=self.seq, t_pub=self.t, session=1, connected=True, head=Head(s, pose(np.eye(3), EYE)),
                       hands=hands)

    def run(self, seconds, dt=1 / 60):
        fired = 0
        for _ in range(round(seconds / dt)):
            self.t += dt
            fired += self.menu.step(self.frame(), self.t)
        return fired


def test_finger_straightness_tells_an_open_hand_from_a_fist():
    j = open_left()
    assert hm.finger_straightness(j) > 0.99
    assert hm.finger_straightness(fist(j)) < 0.5


def test_palm_normal_points_out_of_the_palm_for_either_hand():
    for side in ("left", "right"):
        j = hand_joints([0, 0, 1.0], [0, 0, -1.0], [1.0, 0, 0], side)                # palm down
        assert np.allclose(hm.palm_normal(j, side), [0, 0, -1.0], atol=1e-9)


def test_arms_after_the_hold_and_opens_on_the_tap():
    op = Op()
    op.left = open_left()
    assert op.run(0.25) == 0 and not op.menu.armed
    assert op.run(0.1) == 0 and op.menu.armed
    op.right = right_index_at(tap_point(op.left) + [0.0, 0.0, 0.03])
    assert op.run(0.2) == 1
    assert not op.menu.armed


def test_a_tap_without_the_raised_palm_does_nothing():
    op = Op()
    op.left = open_left(normal=-TO_EYE)                       # palm away from the eyes
    op.right = right_index_at(tap_point(op.left))
    assert op.run(2.0) == 0 and not op.menu.armed


def test_a_fist_or_a_pinch_with_the_palm_up_never_arms():
    op = Op()
    op.left = fist(open_left())
    op.right = right_index_at(tap_point(open_left()))
    assert op.run(2.0) == 0 and not op.menu.armed


def test_the_finger_near_but_not_on_the_palm_does_not_open():
    op = Op()
    op.left = open_left()
    op.run(0.4)
    op.right = right_index_at(tap_point(op.left) + [0.0, 0.0, 0.06])
    assert op.run(2.0) == 0


def test_the_arm_survives_the_left_hand_vanishing_while_tapping():
    op = Op()
    op.left = open_left()
    op.run(0.4)
    op.left_fresh = False                                     # occluded by the right hand
    op.right = right_index_at(tap_point(op.left))
    assert op.run(0.2) == 1


def test_the_arm_expires_after_the_grace():
    op = Op()
    op.left = open_left()
    op.run(0.4)
    op.left_fresh = False
    op.run(0.7)
    assert not op.menu.armed
    op.right = right_index_at(tap_point(op.left))
    assert op.run(0.5) == 0


def test_after_opening_the_palm_must_turn_away_first():
    op = Op()
    op.left = open_left()
    op.run(0.4)
    op.right = right_index_at(tap_point(op.left))
    assert op.run(0.2) == 1
    assert op.run(2.0) == 0 and not op.menu.armed            # still up and touching
    op.left = open_left(normal=-TO_EYE)
    op.run(0.1)
    op.left = open_left()
    assert op.run(0.5) == 1


def test_require_release_blocks_a_palm_that_is_already_up():
    op = Op()
    op.left = open_left()
    op.run(0.1)
    op.menu.require_release()                                 # the panel opened with the stick meanwhile
    op.right = right_index_at(tap_point(op.left))
    assert op.run(1.0) == 0
