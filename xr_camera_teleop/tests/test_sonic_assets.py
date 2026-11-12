"""SONIC asset manifest: loading, hashing, verification rules."""
import hashlib
import shutil
import subprocess

import pytest
import yaml

from xr_camera_teleop.sonic import assets as A


def _sha(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


@pytest.fixture
def tree(tmp_path):
    """A fake SONIC layout: git source checkout + models root with a planner, a binary and a motion dir."""
    src = tmp_path / "src"
    (src / "external_dependencies" / "sdk").mkdir(parents=True)
    (src / "README").write_text("sonic")
    models = tmp_path / "models"
    (models / "planner" / "V2").mkdir(parents=True)
    (models / "planner" / "V2" / "planner.onnx").write_bytes(b"planner")
    (models / "deploy").mkdir()
    (models / "deploy" / "bin").write_bytes(b"binary")
    ref = models / "ref"
    (ref / "clip_a").mkdir(parents=True)
    (ref / "clip_a" / "joint_pos.csv").write_text("1,2,3")
    (ref / "clip_b.csv").write_text("4,5")
    commit = None
    if shutil.which("git"):
        def git(*a):
            return subprocess.run(["git", "-C", str(src), *a], check=True, capture_output=True, text=True).stdout
        git("init", "-q")
        git("add", "-A")
        git("-c", "user.name=t", "-c", "user.email=t@t", "commit", "-qm", "init")
        commit = git("rev-parse", "HEAD").strip()
    manifest = {
        "roots": {"src": str(src), "models": str(models)},
        "source": {"root": "src", "commit": commit or "0" * 40, "python_paths": [".", "external_dependencies/sdk"]},
        "files": {
            "planner": {"root": "models", "path": "planner/V2/planner.onnx", "sha256": _sha(b"planner")},
            "deploy_binary": {"root": "models", "path": "deploy/bin", "sha256": _sha(b"binary")},
        },
        "dirs": {"reference_motions": {"root": "models", "path": "ref", "sha256": A.sha256_tree(ref)}},
        "deploy": {"dds_domain": 42, "planner_version": "V2"},
    }
    path = tmp_path / "sonic_assets.yaml"
    path.write_text(yaml.safe_dump(manifest))
    return path, manifest, src, models


def _failed(checks):
    return {c.name for c in checks if not c.ok}


@pytest.mark.skipif(shutil.which("git") is None, reason="needs git")
def test_pinned_tree_verifies(tree):
    path, _, src, models = tree
    m = A.load(path)
    assert m.dds_domain == 42 and m.path("planner") == models / "planner" / "V2" / "planner.onnx"
    assert m.path("source") == src and m.python_paths == (src / ".", src / "external_dependencies" / "sdk")
    assert _failed(A.verify(m)) == set()


@pytest.mark.skipif(shutil.which("git") is None, reason="needs git")
def test_tampered_file_and_dir_fail_only_with_hashing(tree):
    path, _, _, models = tree
    (models / "deploy" / "bin").write_bytes(b"patched")
    (models / "ref" / "clip_a" / "joint_pos.csv").write_text("1,2,4")
    m = A.load(path)
    assert _failed(A.verify(m)) == {"deploy_binary", "reference_motions"}
    assert _failed(A.verify(m, hashes=False)) == set()


@pytest.mark.skipif(shutil.which("git") is None, reason="needs git")
def test_dirty_or_wrong_commit_source_fails(tree):
    path, manifest, src, _ = tree
    (src / "README").write_text("patched upstream")
    assert "source" in _failed(A.verify(A.load(path)))
    subprocess.run(["git", "-C", str(src), "checkout", "-q", "--", "README"], check=True)
    manifest["source"]["commit"] = "f" * 40
    path.write_text(yaml.safe_dump(manifest))
    assert _failed(A.verify(A.load(path))) == {"source"}


def test_missing_asset_and_unpinned_asset_fail(tree):
    path, manifest, _, _ = tree
    manifest["files"]["deploy_binary"]["path"] = "deploy/nope"
    del manifest["files"]["planner"]["sha256"]
    path.write_text(yaml.safe_dump(manifest))
    failed = _failed(A.verify(A.load(path)))
    assert {"deploy_binary", "planner"} <= failed


def test_tree_hash_ignores_order_and_mtime_but_sees_renames_and_symlink_targets(tmp_path):
    d = tmp_path / "d"
    (d / "sub").mkdir(parents=True)
    (d / "a").write_bytes(b"x")
    (d / "sub" / "b").write_bytes(b"y")
    h0 = A.sha256_tree(d)
    (d / "a").touch()
    assert A.sha256_tree(d) == h0
    (d / "a").rename(d / "c")
    assert A.sha256_tree(d) != h0
    (d / "c").rename(d / "a")
    (d / "link").symlink_to("a")
    h1 = A.sha256_tree(d)
    assert h1 != h0
    (d / "link").unlink()
    (d / "link").symlink_to("sub/b")
    assert A.sha256_tree(d) != h1


@pytest.mark.parametrize("path, version, ok", [
    ("/m/planner/target_vel/V2/planner_sonic.onnx", "V2", True),
    ("/m/planner/V1/planner_sonic.onnx", "V2", False),
    ("/m/V0_backup/target_vel/V2/planner_sonic.onnx", "V2", False),   # the binary checks "V0" first
    ("/m/planner/planner_sonic.onnx", "V2", False),
    ("/m/planner/V2/planner_sonic.onnx", "V3", False),
])
def test_planner_version_token_follows_the_binary_search_order(path, version, ok):
    assert (A.planner_version_problem(A.Path(path), version) is None) == ok


def test_missing_key_is_a_manifest_error(tmp_path):
    p = tmp_path / "m.yaml"
    p.write_text(yaml.safe_dump({"roots": {}, "source": {"root": "x", "commit": "c"}}))
    with pytest.raises(A.ManifestError):
        A.load(p)


def test_repo_manifest_parses():
    m = A.load()
    assert m.planner_version == "V2" and {"encoder", "decoder", "planner", "deploy_binary"} <= set(m.assets)
