import hashlib
import json
import threading

import pytest

from xr_camera_teleop.recording.local_archive import LocalArchive,archive_http
from xr_camera_teleop.recording.transfer import download_archive


def test_original_payload_prebuffer_common_clock_and_verified_copy(tmp_path):
    archive = LocalArchive(tmp_path/"pc2","camera",chunk_bytes=12)
    archive.submit({"topic":"left","stamp":{"collector_monotonic_ns":100,"source_timestamp_ns":90}},b"original-left")
    archive.command("start",{"episode_id":"episode_test","prompt":"Pick apple"})
    archive.submit({"topic":"right","stamp":{"collector_monotonic_ns":101,"source_timestamp_ns":90}},b"original-right")
    result = archive.command("stop",{"episode_id":"episode_test","reason":"manual"})
    assert result["samples"] == 2 and result["complete"]
    assert result["queue_losses"] == 0
    server = archive_http(archive,"127.0.0.1",0)
    try:
        copy = tmp_path/"msi"
        final = download_archive("127.0.0.1",server.server_port,"episode_test",copy)
        assert final["transfer_verified"] and final["source_retained"]
        lines = [json.loads(s) for s in (copy/"index.jsonl").read_text().splitlines()]
        assert lines[0]["stamp"]["source_timestamp_ns"] == lines[1]["stamp"]["source_timestamp_ns"]
        assert (copy/lines[0]["payload_file"]).read_bytes() == b"original-left"
        # Corrupt a downloaded chunk: retry re-downloads and verifies rather than accepting it.
        (copy/lines[0]["payload_file"]).write_bytes(b"corrupt")
        download_archive("127.0.0.1",server.server_port,"episode_test",copy)
        for file in final["files"]:
            assert hashlib.sha256((copy/file["name"]).read_bytes()).hexdigest()==file["sha256"]
        assert (archive.root/"episode_test"/"manifest.json").exists()
    finally:
        server.shutdown()
        server.server_close()


def test_duplicate_start_idempotent_and_overwrite_rejected(tmp_path):
    archive = LocalArchive(tmp_path,"telemetry")
    request = {"episode_id":"episode_test","prompt":"Task"}
    archive.command("start",request)
    assert archive.command("start",request)["recording"]
    with pytest.raises(RuntimeError):
        archive.command("start",dict(request,episode_id="episode_other"))
    archive.command("stop",dict(request,incomplete=True))
    assert archive.command("stop",request)["incomplete"]
    with pytest.raises(RuntimeError):
        archive.command("start",request)
