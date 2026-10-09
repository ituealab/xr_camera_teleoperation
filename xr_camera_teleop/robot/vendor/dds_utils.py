# Vendored from unitreerobotics/xr_teleoperate @ 817fb00 (Apache-2.0, see LICENSE-THIRD-PARTY).
# Source file: teleop/robot_control/dds_utils.py
# MODIFIED: logging_mp replaced by the standard logging module.
import logging
import time

logger_mp = logging.getLogger(__name__)


def wait_for_dds(is_ready, controller_name, timeout=5.0, poll_interval=0.1):
    logger_mp.warning(f"[{controller_name}] Waiting to subscribe dds...")
    deadline = time.monotonic() + timeout
    while not is_ready():
        if time.monotonic() >= deadline:
            error_msg = f"[{controller_name}] Failed to subscribe dds within {timeout:.1f} seconds."
            logger_mp.error(error_msg)
            raise TimeoutError(error_msg)
        time.sleep(poll_interval)
    logger_mp.info(f"[{controller_name}] Subscribe dds ok.")
