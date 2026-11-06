"""Two open palms facing the camera, release-before-toggle, on new frames only."""
import numpy as np


def open_palm(hand, side):
    from xr_camera_teleop.operator.mano_to_webxr import keypoint_wrist_frame
    kp = np.asarray(hand.get("keypoints", []), float)
    if kp.shape != (21, 3) or not np.isfinite(kp).all():
        return False
    if float(hand.get("conf",0)) < .5 or not .05 < kp[0,2] < .9:
        return False
    # Require straight MCP->PIP->DIP->tip chains, not just fingertip-wrist distances.
    for chain in ((5,6,7,8), (9,10,11,12), (13,14,15,16), (17,18,19,20)):
        bones = np.diff(kp[list(chain)], axis=0)
        lengths = np.linalg.norm(bones, axis=1)
        if np.min(lengths) < .001:
            return False
        directions = bones / lengths[:, None]
        if np.min(np.sum(directions[:-1]*directions[1:], axis=1)) < .80:
            return False
        if np.linalg.norm(kp[chain[-1]]-kp[0]) < 1.35*np.linalg.norm(kp[chain[0]]-kp[0]):
            return False
    palm_width = np.linalg.norm(kp[5]-kp[17])
    if palm_width < .01 or np.linalg.norm(kp[4]-kp[8]) < palm_width*.45:
        return False
    rotation, _ = keypoint_wrist_frame(kp, side)
    # Keypoint normal is BACK of hand; palm faces camera => back points camera +Z.
    camera_to_hand = kp[0]/np.linalg.norm(kp[0])
    return bool(np.dot(rotation[:,2],camera_to_hand) > .65)


class GestureToggle:
    def __init__(self, hold_seconds=1, release_seconds=.3, max_gap=.4, min_frames=5):
        self.hold_seconds, self.release_seconds, self.max_gap = hold_seconds, release_seconds, max_gap
        self.last_frame = None
        self.last_time = None
        self.since = None
        self.release_since = None
        self.armed = True
        self.frames = 0
        self.min_frames = min_frames

    def require_release(self):
        self.armed = False
        self.since = self.release_since = None
        self.frames = 0

    def status(self):
        return {"armed":self.armed,"hold_started":self.since,"hold_frames":self.frames,
                "hold_seconds":self.hold_seconds,"release_required":not self.armed}

    def update(self, frame_id, timestamp, detected):
        if frame_id == self.last_frame:
            return False
        if self.last_time is not None and timestamp <= self.last_time:
            return False
        if self.last_time is not None and (timestamp-self.last_time > self.max_gap or timestamp < self.last_time):
            self.since = self.release_since = None
            self.frames = 0
        self.last_frame, self.last_time = frame_id, timestamp
        # Missing hands cannot count as deliberately releasing the open-palm gesture.
        if detected is None:
            self.since = self.release_since = None
            self.frames = 0
            return False
        if not detected:
            self.since = None
            self.frames = 0
            if self.release_since is None:
                self.release_since = timestamp
            if timestamp-self.release_since >= self.release_seconds:
                self.armed = True
            return False
        self.release_since = None
        if not self.armed:
            return False
        if self.since is None:
            self.since = timestamp
        self.frames += 1
        if timestamp-self.since >= self.hold_seconds and self.frames>=self.min_frames:
            self.armed = False
            self.since = None
            return True
        return False
