"""Filesystem locations shared by the package (assets, configs, runtime data, caches)."""
from __future__ import annotations

import os
from pathlib import Path

PKG_DIR = Path(__file__).resolve().parent
ASSETS = PKG_DIR / "assets"
G1_DIR = ASSETS / "g1"
G1_URDF = G1_DIR / "g1_body29_hand14.urdf"
G1_MJCF = G1_DIR / "g1_body29_hand14.xml"
DEX3_YML = ASSETS / "unitree_hand" / "unitree_dex3.yml"

# repo checkout root (configs/, data/, envs/); with a site-packages install this is not a checkout,
# set XRCAM_DATA / XRCAM_CONFIGS instead.
ROOT = PKG_DIR.parent
CONFIGS = Path(os.environ.get("XRCAM_CONFIGS", ROOT / "configs"))
DATA = Path(os.environ.get("XRCAM_DATA", ROOT / "data"))


def cache_dir() -> Path:
    """User cache directory for derived files (pinocchio model cache); created on demand."""
    base = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    d = base / "xr_camera_teleop"
    d.mkdir(parents=True, exist_ok=True)
    return d
