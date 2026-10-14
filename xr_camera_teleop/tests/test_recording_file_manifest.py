import hashlib
import json

from xr_camera_teleop.recording.bag_recorder import file_manifest


def test_file_manifest_hashes_every_bag_file_and_excludes_the_manifest(tmp_path):
    (tmp_path / "bag").mkdir()
    (tmp_path / "bag" / "bag_0.db3").write_bytes(b"sqlite")
    (tmp_path / "index.jsonl").write_text('{"a":1}\n')
    (tmp_path / "sync_report.json").write_text("{}")
    (tmp_path / "manifest.json").write_text("{}")
    entries = {entry["name"]: entry for entry in file_manifest(tmp_path)}
    assert set(entries) == {"bag/bag_0.db3", "index.jsonl", "sync_report.json"}
    assert entries["bag/bag_0.db3"]["sha256"] == hashlib.sha256(b"sqlite").hexdigest()
    assert entries["index.jsonl"]["bytes"] == len(b'{"a":1}\n')
