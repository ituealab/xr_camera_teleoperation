"""Post-retargeting Dex3 shaping: the thumb follows taught robot poses, the fingers DexPilot.

DexPilot chases the human thumb-tip position through a URDF whose thumb_0 is offset
from the real joint, so its thumb neither opens nor opposes correctly. Here the thumb
moves continuously along a path through poses taught on the real hands: open -> cup
-> index or middle pinch, chosen by the thumb's gap to the fingertips, so a wider
object stops it earlier on the path. Curled ring/pinky fingers (straight in a pinch)
pull it to the fist pose instead. Toward the flat open palm of the recording gesture
(recording/gestures.py) the fingers blend to the robot's open pose from lowstate.
"""
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
# thumb_0..2 taught on both limp hands (2026-10-30) and averaged so the hands match.
# Written for the left hand; the right mirrors thumb_1 and thumb_2.
OPEN_THUMB = np.array((-1.11, -0.61, 0.03))
CUP_THUMB = np.array((-0.95, 0.20, 0.79))
INDEX_PINCH_THUMB = np.array((-1.99, 0.34, 0.79))
MIDDLE_PINCH_THUMB = np.array((-0.37, 0.59, 0.63))
FIST_THUMB = np.array((-1.13, 0.69, 1.54))
# Thumb-to-nearest-fingertip gap in palm widths, live WiLoR hands (2026-10-30):
# open 1.5-1.9, cup ~1.1, pinch contact 0.15-0.25, fist 0.6-0.8.
OPEN_GAP, CUP_GAP, CONTACT_GAP = 1.6, 1.1, 0.3
# Ring/pinky extension: pinch ~1.0, cup ~0.95, fist ~0.05.
FIST_CURLED, FIST_STRAIGHT = 0.15, 0.6


def _ramp(x, lo, hi):
    t = np.clip((x - lo) / (hi - lo), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def finger_extension(webxr25):
    """Extension 0..1 (chain straightness and reach) of index, middle, ring, pinky."""
    w = webxr25[WRIST]
    fingers = []
    for chain in FINGER_CHAINS:
        bones = np.diff(webxr25[list(chain)], axis=0)
        dirs = bones / (np.linalg.norm(bones, axis=1)[:, None] + 1e-9)
        straight = np.min(np.sum(dirs[:-1] * dirs[1:], axis=1))
        reach = np.linalg.norm(webxr25[chain[-1]] - w) / (np.linalg.norm(webxr25[chain[0]] - w) + 1e-9)
        fingers.append(0.5 * (_ramp(straight, 0.3, 0.95) + _ramp(reach, 0.9, 1.75)))
    return np.array(fingers)


def thumb_gap(webxr25):
    """Thumb tip to the nearer of the index/middle tips, in palm widths."""
    palm = np.linalg.norm(webxr25[INDEX_MCP] - webxr25[PINKY_MCP]) + 1e-9
    return min(np.linalg.norm(webxr25[THUMB_TIP] - webxr25[INDEX_TIP]),
               np.linalg.norm(webxr25[THUMB_TIP] - webxr25[MIDDLE_TIP])) / palm


def middle_share(webxr25):
    """0..1: the thumb is nearer the middle tip (1) than the index tip (0)."""
    di = np.linalg.norm(webxr25[THUMB_TIP] - webxr25[INDEX_TIP])
    dm = np.linalg.norm(webxr25[THUMB_TIP] - webxr25[MIDDLE_TIP])
    return float(_ramp(di / (di + dm + 1e-9), 0.35, 0.65))


def thumb_target(side, webxr25):
    """thumb_0..2 for one hand."""
    gap = thumb_gap(webxr25)
    pinch = INDEX_PINCH_THUMB + middle_share(webxr25) * (MIDDLE_PINCH_THUMB - INDEX_PINCH_THUMB)
    q = OPEN_THUMB + (1.0 - _ramp(gap, CUP_GAP, OPEN_GAP)) * (CUP_THUMB - OPEN_THUMB)
    q = q + (1.0 - _ramp(gap, CONTACT_GAP, CUP_GAP)) * (pinch - CUP_THUMB)
    ring_pinky = float(np.mean(finger_extension(webxr25)[2:]))
    q = q + (1.0 - _ramp(ring_pinky, FIST_CURLED, FIST_STRAIGHT)) * (FIST_THUMB - q)
    return q * (1.0, -1.0, -1.0) if side == "right" else q


class ThumbShaper:
    def shape(self, side, q7, hand):
        """One hand: q7 in Dex3 API order, hand = (25,3) WebXR joints."""
        out = np.array(q7, dtype=float)
        out[:3] = thumb_target(side, hand)
        # Only a thumb opened past the cup grip counts: a grip's nearly straight fingers stay curled.
        w = float(np.mean(finger_extension(hand))) * _ramp(thumb_gap(hand), CUP_GAP, OPEN_GAP)
        out[3:] = w * np.asarray(OPEN_POSE[side])[3:] + (1.0 - w) * out[3:]
        return out

    def __call__(self, target, hands):
        target = np.asarray(target, dtype=float)
        return np.concatenate((self.shape("left", target[:7], hands[0]),
                               self.shape("right", target[7:], hands[1])))
