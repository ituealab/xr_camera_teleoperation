"""OPTIONAL robot head-camera client. DISABLED BY DEFAULT and never used by the teleop loop.

The operator watches the robot camera outside this program. This thin wrapper only exists so that the feature
is "off, not deleted": it is started only by `xrcam-robot --robot-camera` and imports `teleimager`
(unitreerobotics/teleimager, NOT vendored, not a dependency of this package) lazily.
"""
from __future__ import annotations

import logging

log = logging.getLogger("xrcam.robot.camera")


class RobotCamera:
    def __init__(self, host: str = "192.168.123.164", request_bgr: bool = True):
        try:
            from teleimager.image_client import ImageClient
        except ImportError as e:
            raise RuntimeError("--robot-camera needs the teleimager package "
                               "(pip install git+https://github.com/unitreerobotics/teleimager) and its image "
                               "server on PC2") from e
        self._ImageClient, self.host, self.request_bgr = ImageClient, host, request_bgr
        self.client = None

    def start(self):
        self.client = self._ImageClient(host=self.host, request_bgr=self.request_bgr)
        log.info("robot camera client connected to %s: %s", self.host, self.client.get_cam_config())
        return self

    def head_frame(self):
        """Latest head-camera frame (BGR ndarray) or None."""
        return None if self.client is None else self.client.get_head_frame().bgr

    def close(self):
        if self.client is not None:
            self.client.close()
            self.client = None
