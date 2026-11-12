#!/usr/bin/env python
"""Build the sim's MuJoCo scene from the upstream SONIC G1 scene, without copying or editing it.

Upstream `scene_43dof.xml` (G1 29 DoF + Dex3, floor, light) is loaded with MjSpec; this module adds the ZED 2i
stereo cameras, the render settings and the static course geometry of `configs/sim_courses.yaml`, and writes
`data/sim/scenes/<course>.xml` with absolute mesh paths into the SONIC checkout. Physics of the robot is not changed.

    data/sonic-sim-venv/bin/python -m xr_camera_teleop.sim.scene_builder --course flat

Frames: torso_link is x forward, y left, z up. A MuJoCo camera looks along its -z with +y up in the image.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from xr_camera_teleop.paths import CONFIGS, DATA

COURSES = CONFIGS / "sim_courses.yaml"
SCENES_DIR = DATA / "sim" / "scenes"


@dataclass(frozen=True)
class StereoCamera:
    body: str
    center: tuple[float, float, float]
    pitch_down_deg: float
    baseline: float
    fovy_deg: float
    resolution: tuple[int, int]
    names: tuple[str, str]


def load_config(path: Path | str = COURSES) -> dict:
    return yaml.safe_load(Path(path).read_text())


def stereo_camera(cfg: dict) -> StereoCamera:
    c = cfg["robot_camera"]
    return StereoCamera(body=c["body"], center=tuple(map(float, c["center"])), pitch_down_deg=float(c["pitch_down_deg"]),
                        baseline=float(c["baseline"]), fovy_deg=float(c["fovy_deg"]),
                        resolution=tuple(map(int, c["resolution"])), names=tuple(c["names"]))


def _ry(deg: float) -> np.ndarray:
    a = np.radians(deg)
    return np.array([[np.cos(a), 0.0, np.sin(a)], [0.0, 1.0, 0.0], [-np.sin(a), 0.0, np.cos(a)]])


def _mat_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> unit quaternion (w, x, y, z), w >= 0."""
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0
    y = np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0
    z = np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def stereo_poses(cam: StereoCamera) -> list[tuple[str, np.ndarray, np.ndarray]]:
    """[(name, pos, quat_wxyz)] of the left and right eye in the mount body.

    The camera frame of the CAD (x along the optical axis, y left, z up) is Ry(pitch_down) of the body frame;
    the eyes sit at +-baseline/2 along its y. MuJoCo's camera axes are image-right = -y, image-up = z, back = -x.
    """
    R_cad = _ry(cam.pitch_down_deg)
    R_mj = R_cad @ np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    q = _mat_to_quat(R_mj)
    c = np.asarray(cam.center)
    half = R_cad[:, 1] * cam.baseline / 2.0
    return [(cam.names[0], c + half, q), (cam.names[1], c - half, q.copy())]


def _euler_deg_to_quat(rpy_deg) -> np.ndarray:
    """[roll, pitch, yaw] degrees -> quaternion (w, x, y, z) of Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = np.radians(np.asarray(rpy_deg, dtype=float))
    cr, sr, cp, sp, cy, sy = np.cos(r / 2), np.sin(r / 2), np.cos(p / 2), np.sin(p / 2), np.cos(y / 2), np.sin(y / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def build_spec(cfg: dict, course: str, sonic_root: Path):
    """MjSpec of the upstream base scene + render settings + stereo cameras + course boxes."""
    import mujoco

    if course not in cfg["courses"]:
        raise KeyError(f"unknown course {course!r}; known: {', '.join(cfg['courses'])}")
    base = Path(sonic_root) / cfg["base_scene"]
    spec = mujoco.MjSpec.from_file(str(base))
    if not Path(spec.meshdir).is_absolute():          # the generated XML lives elsewhere
        spec.meshdir = str((base.parent / spec.meshdir).resolve())

    r = cfg["render"]
    spec.visual.global_.offwidth, spec.visual.global_.offheight = map(int, r["offscreen_size"])
    spec.visual.quality.shadowsize = int(r["shadow_size"])
    group = int(r["robot_collision_group"])
    for g in spec.geoms:
        if g.parent.name != "world" and (g.contype or g.conaffinity):
            g.group = group

    cam = stereo_camera(cfg)
    body = spec.body(cam.body)
    if body is None:
        raise KeyError(f"robot_camera.body {cam.body!r} not in {base}")
    for name, pos, quat in stereo_poses(cam):
        body.add_camera(name=name, pos=pos.tolist(), quat=quat.tolist(), fovy=cam.fovy_deg,
                        resolution=list(cam.resolution))

    for el in (cfg["courses"][course] or {}).get("elements", []):
        if el.get("type") != "box":
            raise ValueError(f"course {course}: unsupported element type {el.get('type')!r} (only 'box')")
        spec.worldbody.add_geom(name=el["name"], type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(map(float, el["pos"])),
                                size=list(map(float, el["size"])),
                                quat=_euler_deg_to_quat(el.get("euler_deg", [0, 0, 0])).tolist(),
                                friction=[float(el.get("friction", 1.0)), 0.005, 0.0001],
                                rgba=list(map(float, el.get("rgba", [0.55, 0.55, 0.5, 1.0]))))
    return spec


def write_scene(course: str, sonic_root: Path, cfg: dict | None = None, out_dir: Path = SCENES_DIR) -> Path:
    """Compile the course scene and write it as standalone MJCF; returns the path."""
    cfg = load_config() if cfg is None else cfg
    spec = build_spec(cfg, course, sonic_root)
    spec.compile()
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"{course}.xml"
    tmp = out.with_suffix(".xml.tmp")
    tmp.write_text(spec.to_xml())
    tmp.replace(out)        # atomic: a running sim never reads a half-written scene
    return out


def main(argv=None) -> int:
    from xr_camera_teleop.sonic import assets

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--course", default="flat")
    ap.add_argument("--config", type=Path, default=COURSES)
    ap.add_argument("--out-dir", type=Path, default=SCENES_DIR)
    a = ap.parse_args(argv)
    print(write_scene(a.course, assets.load().source_root, load_config(a.config), a.out_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
