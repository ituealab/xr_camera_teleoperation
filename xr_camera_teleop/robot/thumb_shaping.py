"""Post-retargeting Dex3 open-hand shaping (DexPilot output is left untouched).

DexPilot chases the human thumb-tip position, which spins the Dex3 thumb about
its palm-normal axis (thumb_0), mimicking the human thumb sideways. Here thumb_0
holds the robot's own open rotation; only a pinch moves the whole thumb toward the
robot's own taught pinch pose, so otherwise the thumb folds in one plane (thumb_1/thumb_2 share a parallel axis): from the
open pose toward the palm. Within that plane thumb_1/thumb_2 are re-solved to get
the thumb tip as close as possible to DexPilot's thumb tip, so pinches still close.
Toward the flat open palm of the recording gesture (recording/gestures.py) the
joints blend to the robot's open pose read from lowstate: the thumb by its
clearance from the index/middle tips, the fingers by their extension.
"""
from dataclasses import dataclass, field

import numpy as np

# WebXR 25-joint layout (operator/mano_to_webxr.py): finger MCP..tip chains.
WRIST, THUMB_TIP, INDEX_TIP, MIDDLE_TIP = 0, 4, 9, 14
FINGER_CHAINS = ((6, 7, 8, 9), (11, 12, 13, 14), (16, 17, 18, 19), (21, 22, 23, 24))
INDEX_MCP, PINKY_MCP = 6, 21

# Dex3 API order: thumb_0..2, middle_0..1, index_0..1. Read verbatim from each
# robot hand's lowstate held fully open (left thumb_0 set by hand on 2026-10-21);
# thumb_0 is a few deg past the URDF limit, which the real hand reaches (the sim
# preview clips it).
OPEN_POSE = {"left": (-1.148, -0.664, -0.032, -0.029, -0.017, -0.038, -0.03),
             "right": (-1.077, 0.555, -0.093, -0.107, -0.094, 0.134, -0.103)}
# thumb_0..2 of each robot hand held in a thumb-index pinch (lowstate, 2026-10-22). The
# real thumb_0 is offset ~1.2 rad from DexPilot's (URDF) thumb_0, so DexPilot's thumb
# pose does not reach a real pinch; a pinch drives the thumb toward these values instead.
PINCH_THUMB = {"left": (-1.772, 0.564, 0.680), "right": (-1.798, -0.628, -0.898)}
THUMB_JOINTS = ("thumb_0", "thumb_1", "thumb_2")


def _ramp(x, lo, hi):
    t = np.clip((x - lo) / (hi - lo), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def hand_openness(webxr25):
    """(fingers, thumb) in 0..1: continuous parts of the open-palm gesture test.

    fingers = mean extension (chain straightness and reach) of the four fingers;
    thumb = how clear the thumb tip is of both the index and the middle tip.
    Ramps fitted on live WiLoR hands: flat open palm straightness ~0.97,
    reach ~1.8, thumb/palm-width ~1.5; fist ~0.1 / ~0.7; pinch thumb ~0.1-0.4.
    """
    w = webxr25[WRIST]
    fingers = []
    for chain in FINGER_CHAINS:
        bones = np.diff(webxr25[list(chain)], axis=0)
        dirs = bones / (np.linalg.norm(bones, axis=1)[:, None] + 1e-9)
        straight = np.min(np.sum(dirs[:-1] * dirs[1:], axis=1))
        reach = np.linalg.norm(webxr25[chain[-1]] - w) / (np.linalg.norm(webxr25[chain[0]] - w) + 1e-9)
        fingers.append(0.5 * (_ramp(straight, 0.3, 0.95) + _ramp(reach, 0.9, 1.75)))
    palm = np.linalg.norm(webxr25[INDEX_MCP] - webxr25[PINKY_MCP]) + 1e-9
    thumb = min(np.linalg.norm(webxr25[THUMB_TIP] - webxr25[INDEX_TIP]),
                np.linalg.norm(webxr25[THUMB_TIP] - webxr25[MIDDLE_TIP])) / palm
    # Fist thumb ~0.5-0.67 must stay closed too, hence the high lower edge; the upper
    # edge is the lowest live open-palm value, so the ramp is as wide (slow) as possible.
    return float(np.mean(fingers)), float(_ramp(thumb, 0.7, 1.35))


def pinch_contact(webxr25):
    """0..1: thumb tip touching the index or middle tip (live pinch ~0.1-0.4, fist >= 0.5)."""
    palm = np.linalg.norm(webxr25[INDEX_MCP] - webxr25[PINKY_MCP]) + 1e-9
    thumb = min(np.linalg.norm(webxr25[THUMB_TIP] - webxr25[INDEX_TIP]),
                np.linalg.norm(webxr25[THUMB_TIP] - webxr25[MIDDLE_TIP])) / palm
    return float(1.0 - _ramp(thumb, 0.2, 0.5))


class _ThumbPlaneIK:
    """thumb_1/thumb_2 (parallel axes) reaching a thumb-tip point at fixed thumb_0."""

    def __init__(self, side):
        import pinocchio as pin
        from xr_camera_teleop.paths import ASSETS
        self.pin = pin
        self.model = pin.buildModelFromUrdf(str(ASSETS / f"unitree_hand/unitree_dex3_{side}.urdf"))
        self.data = self.model.createData()
        self.idx = [self.model.joints[self.model.getJointId(f"{side}_hand_{j}_joint")].idx_q for j in THUMB_JOINTS]
        self.tip = self.model.getFrameId("thumb_tip")
        self.lo = self.model.lowerPositionLimit[self.idx[1:]]
        self.hi = self.model.upperPositionLimit[self.idx[1:]]

    def tip_pos(self, thumb3):
        q = self.pin.neutral(self.model)
        q[self.idx] = thumb3
        self.pin.framesForwardKinematics(self.model, self.data, q)
        return self.data.oMf[self.tip].translation.copy()

    def solve(self, thumb0, target, q12, iters=8, eps=1e-4):
        q12 = np.clip(np.array(q12, dtype=float), self.lo, self.hi)
        for _ in range(iters):
            p = self.tip_pos((thumb0, *q12))
            J = np.stack([(self.tip_pos((thumb0, *(q12 + eps * e))) - p) / eps for e in np.eye(2)], 1)
            step = np.linalg.lstsq(J.T @ J + 1e-6 * np.eye(2), J.T @ (target - p), rcond=None)[0]
            q12 = np.clip(q12 + step, self.lo, self.hi)
        return q12


@dataclass
class ThumbShaper:
    _ik: dict = field(default_factory=dict, repr=False)

    def shape(self, side, q7, hand):
        """One hand: q7 in Dex3 API order, hand = (25,3) WebXR joints."""
        out = np.array(q7, dtype=float)
        pose = np.asarray(OPEN_POSE[side])
        fingers, thumb = hand_openness(hand)
        if side not in self._ik:
            self._ik[side] = _ThumbPlaneIK(side)
        ik = self._ik[side]
        # thumb_0 holds the open rotation; only as the human thumb tip reaches the
        # index/middle tip (pinch_contact is smooth) does the thumb move to the robot pinch.
        pinch = np.asarray(PINCH_THUMB[side])
        p = pinch_contact(hand)
        thumb0 = pose[0] + p * (pinch[0] - pose[0])
        # Otherwise fold in that fixed plane toward DexPilot's thumb tip.
        q12 = ik.solve(thumb0, ik.tip_pos(out[:3]), out[1:3])
        q12 = p * pinch[1:] + (1.0 - p) * q12
        out[0] = thumb * pose[0] + (1.0 - thumb) * thumb0
        out[1:3] = thumb * pose[1:3] + (1.0 - thumb) * q12
        # Fingers also wait for thumb clearance: in a pinch the ring/pinky stay straight.
        w = fingers * thumb
        out[3:] = w * pose[3:] + (1.0 - w) * out[3:]
        return out

    def __call__(self, target, hands):
        target = np.asarray(target, dtype=float)
        return np.concatenate((self.shape("left", target[:7], hands[0]),
                               self.shape("right", target[7:], hands[1])))
