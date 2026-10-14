import json
import threading
import time

import pytest

from xr_camera_teleop.recording.local_archive import LocalArchive


def test_stop_keeps_late_image_from_before_exposure_boundary_and_excludes_future(tmp_path):
    archive = LocalArchive(tmp_path,"camera")
    end = time.monotonic_ns()
    def metadata(sequence,capture,received):
        return {"topic":"left","is_image":True,"stamp":{"sequence":sequence,
                "capture_clock_valid":True,"capture_monotonic_ns":capture,
                "collector_monotonic_ns":received}}
    archive.submit(metadata(0,end-100_000_000,end-90_000_000),b"before")
    request = {"episode_id":"episode_boundary","prompt":"Test","capture_start_monotonic_ns":end-1_000_000_000}
    archive.command("start",request)
    result = {}
    stop_thread = threading.Thread(target=lambda:result.update(archive.command("stop",dict(request,capture_stop_monotonic_ns=end))))
    stop_thread.start()
    deadline = time.monotonic()+1
    while archive.capture_cutoff_ns is None and time.monotonic()<deadline:
        time.sleep(.001)
    archive.submit(metadata(1,end-1_000_000,end+20_000_000),b"late-but-valid")
    archive.submit(metadata(2,end+1_000_000,end+30_000_000),b"future-frame")
    stop_thread.join(timeout=5)
    assert not stop_thread.is_alive()
    assert result["samples"]==2
    assert result["capture_start_monotonic_ns"]==request["capture_start_monotonic_ns"]
    assert result["capture_stop_monotonic_ns"]==end
    payload = b"".join((archive.root/"episode_boundary"/item["name"]).read_bytes()
                       for item in result["files"] if item["name"].endswith(".bin"))
    assert payload==b"beforelate-but-valid"


def test_reboot_between_clock_calibration_and_start_is_rejected(tmp_path):
    archive = LocalArchive(tmp_path,"telemetry")
    with pytest.raises(RuntimeError,match="mixed-epoch"):
        archive.command("start",{"episode_id":"episode_boot","prompt":"Test","capture_clock_boot_id":"old-boot"})
    assert not archive.status()["recording"]
