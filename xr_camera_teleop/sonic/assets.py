#!/usr/bin/env python
"""SONIC asset manifest: where the external SONIC source, models and deploy binary live, and proof they are the pinned ones.

Nothing of SONIC is copied into this repo (PLAN D10). `configs/sonic_assets.yaml` names each asset by root + relative
path and pins it by git commit (source) or sha256 (files, directories). Everything that needs a SONIC path asks this
module, so a moved checkout is a one-line config change.

    data/xr-env/bin/python -m xr_camera_teleop.sonic.assets verify            # all checks, exit 1 on any failure
    data/xr-env/bin/python -m xr_camera_teleop.sonic.assets path planner      # print one resolved path (for scripts)
    data/xr-env/bin/python -m xr_camera_teleop.sonic.assets hash planner      # print the sha256 to pin a new asset

Directory hashes (`sha256_tree`) cover relative paths and file contents, not mtimes or permissions.
"""
from __future__ import annotations

import argparse
import hashlib
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

from xr_camera_teleop.paths import CONFIGS

MANIFEST = CONFIGS / "sonic_assets.yaml"
_CHUNK = 1 << 22


class ManifestError(ValueError):
    pass


@dataclass(frozen=True)
class Asset:
    name: str
    path: Path
    kind: str                       # "file" | "dir"
    sha256: str | None = None


@dataclass(frozen=True)
class Manifest:
    source_root: Path
    source_commit: str
    python_paths: tuple[Path, ...]  # what the sim venv puts on sys.path (gear_sonic, unitree_sdk2py)
    assets: dict[str, Asset]
    dds_domain: int                 # the deploy binary's compiled-in DDS domain; the sim follows it
    planner_version: str            # the deploy binary reads its planner version from a token in the path

    def path(self, name: str) -> Path:
        if name == "source":
            return self.source_root
        try:
            return self.assets[name].path
        except KeyError:
            raise ManifestError(f"unknown asset {name!r}; known: source, {', '.join(self.assets)}") from None


def load(path: Path | str = MANIFEST) -> Manifest:
    raw = yaml.safe_load(Path(path).read_text())
    try:
        roots = {k: Path(v).expanduser() for k, v in raw["roots"].items()}
        src = raw["source"]
        source_root = roots[src["root"]]
        assets = {}
        for kind, section in (("file", "files"), ("dir", "dirs")):
            for name, a in (raw.get(section) or {}).items():
                if name in assets or name == "source":
                    raise ManifestError(f"duplicate asset name {name!r}")
                assets[name] = Asset(name, roots[a["root"]] / a["path"], kind, a.get("sha256"))
        deploy = raw["deploy"]
        return Manifest(source_root=source_root, source_commit=str(src["commit"]),
                        python_paths=tuple(source_root / p for p in src.get("python_paths", [])),
                        assets=assets, dds_domain=int(deploy["dds_domain"]),
                        planner_version=str(deploy["planner_version"]))
    except KeyError as e:
        raise ManifestError(f"{path}: missing key {e}") from None


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(_CHUNK):
            h.update(chunk)
    return h.hexdigest()


def sha256_tree(root: Path) -> str:
    """Hash of a directory: sorted 'relative/posix/path\\0<file sha256>\\n' lines; a symlink contributes
    'path\\0-> target\\n' (its target text, not the target's bytes, so a lib's soname links are not hashed twice)."""
    h = hashlib.sha256()
    for p in sorted(q for q in root.rglob("*") if q.is_symlink() or q.is_file()):
        content = f"-> {p.readlink()}" if p.is_symlink() else sha256_file(p)
        h.update(f"{p.relative_to(root).as_posix()}\0{content}\n".encode())
    return h.hexdigest()


def digest(asset: Asset) -> str:
    return sha256_file(asset.path) if asset.kind == "file" else sha256_tree(asset.path)


def git_state(repo: Path) -> tuple[str | None, bool]:
    """(HEAD commit or None, working tree dirty)."""
    def run(*args):
        return subprocess.run(["git", "-C", str(repo), *args], capture_output=True, text=True, check=False)
    head = run("rev-parse", "HEAD")
    if head.returncode != 0:
        return None, False
    status = run("status", "--porcelain", "--untracked-files=no")
    return head.stdout.strip(), bool(status.stdout.strip())


def planner_version_problem(path: Path, version: str) -> str | None:
    """The deploy binary picks the planner version with path.find("V0"), then "V1", then "V2" (first hit wins)."""
    tokens = ("V0", "V1", "V2")
    if version not in tokens:
        return f"planner_version {version!r} is not one of {tokens}"
    s = str(path)
    first = next((t for t in tokens if t in s), None)
    if first != version:
        return f"planner path would be read as {first or 'no version'} by the deploy binary, manifest says {version}"
    return None


@dataclass(frozen=True)
class Check:
    name: str
    ok: bool
    detail: str


def verify(m: Manifest, hashes: bool = True) -> list[Check]:
    """Every check, failed or not, in a stable order. `hashes=False` checks presence only (fast)."""
    out = []
    head, dirty = git_state(m.source_root) if m.source_root.is_dir() else (None, False)
    if head is None:
        out.append(Check("source", False, f"{m.source_root}: not a git checkout"))
    else:
        ok = head == m.source_commit and not dirty
        out.append(Check("source", ok, f"{head[:12]}{' (dirty)' if dirty else ''}, pinned {m.source_commit[:12]}"))
    for p in m.python_paths:
        out.append(Check(f"python_path {p.name}", p.is_dir(), str(p)))
    for a in m.assets.values():
        present = a.path.is_file() if a.kind == "file" else a.path.is_dir()
        if not present:
            out.append(Check(a.name, False, f"missing {a.kind}: {a.path}"))
        elif a.sha256 is None:
            out.append(Check(a.name, False, "no sha256 pinned in the manifest"))
        elif hashes:
            got = digest(a)
            out.append(Check(a.name, got == a.sha256, "sha256 ok" if got == a.sha256 else f"sha256 {got} != pinned"))
        else:
            out.append(Check(a.name, True, "present (hash not checked)"))
    if "planner" in m.assets:
        problem = planner_version_problem(m.assets["planner"].path, m.planner_version)
        out.append(Check("planner_version", problem is None, problem or f"path token {m.planner_version}"))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--manifest", type=Path, default=MANIFEST)
    sub = ap.add_subparsers(dest="cmd", required=True)
    v = sub.add_parser("verify", help="check commit, presence, sha256 and the planner version token")
    v.add_argument("--quick", action="store_true", help="presence only, skip hashing (~3 GB)")
    sub.add_parser("path", help="print one resolved path").add_argument("name")
    sub.add_parser("hash", help="print the sha256 of one asset as it is on disk").add_argument("name")
    sub.add_parser("dds-domain", help="print the deploy binary's DDS domain")
    sub.add_parser("python-paths", help="print the sys.path entries for the sim venv, one per line")
    a = ap.parse_args(argv)
    m = load(a.manifest)
    if a.cmd == "path":
        print(m.path(a.name))
    elif a.cmd == "hash":
        print(digest(m.assets[a.name]))
    elif a.cmd == "dds-domain":
        print(m.dds_domain)
    elif a.cmd == "python-paths":
        print("\n".join(str(p) for p in m.python_paths))
    else:
        checks = verify(m, hashes=not a.quick)
        w = max(len(c.name) for c in checks)
        for c in checks:
            print(f"{'ok  ' if c.ok else 'FAIL'} {c.name:<{w}}  {c.detail}")
        return 0 if all(c.ok for c in checks) else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
