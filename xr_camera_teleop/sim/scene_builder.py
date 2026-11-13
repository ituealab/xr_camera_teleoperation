#!/usr/bin/env python
"""Build the sim's MuJoCo scene from the upstream SONIC G1 scene, without copying or editing it.

Upstream `scene_43dof.xml` (G1 29 DoF + Dex3, floor, light) is loaded with MjSpec; this module adds the ZED 2i
stereo cameras, the render settings and the course of `configs/sim_courses.yaml` (its geometry: `sim/courses.py`),
and writes `data/sim/scenes/<course>.xml` with absolute mesh paths into the SONIC checkout. Physics of the robot is
not changed; its Dex3 thumb_0 hinges are re-zeroed to the real hands' joint convention (sim/dex3.py).

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
from xr_camera_teleop.sim import courses, dex3

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


def _add_part(spec, part: courses.Part) -> None:
    """One course part (sim/courses.py) as MjSpec elements in the world."""
    import mujoco

    quat = _euler_deg_to_quat(np.degrees(part.euler)).tolist()
    kw = dict(name=part.name, pos=list(part.pos), quat=quat, rgba=list(part.rgba))
    c = part.contact
    if c is None:
        kw.update(contype=0, conaffinity=0)
    else:
        kw.update(friction=list(c.friction), condim=int(c.condim), solref=list(c.solref), solimp=list(c.solimp))
    if part.kind == "box":
        spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=list(part.size), **kw)
    elif part.kind == "disc":
        spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[part.size[0], part.size[1], 0.0], **kw)
    elif part.kind == "hfield":
        z = np.asarray(part.elevation)
        spec.add_hfield(name=part.name, size=list(part.size), nrow=z.shape[0], ncol=z.shape[1],
                        userdata=z.ravel().tolist())          # row 0 at -y, as Part (tests/test_sim_courses.py rays)
        spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_HFIELD, hfieldname=part.name, **kw)
    elif part.kind == "free_box":
        body = spec.worldbody.add_body(name=part.name, pos=kw.pop("pos"), quat=kw.pop("quat"))
        body.add_freejoint(name=f"{part.name}_free")
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=list(part.size), mass=float(part.mass), **kw)
    else:
        raise ValueError(f"unknown part kind {part.kind!r}")


def build_spec(cfg: dict, course: str, sonic_root: Path):
    """MjSpec of the upstream base scene (Dex3 thumb_0 re-zeroed, sim/dex3.py) + render settings + stereo cameras +
    the course (sim/courses.py)."""
    import mujoco

    parts = courses.course_parts(cfg, course)          # validates the course before the heavy load
    base = Path(sonic_root) / cfg["base_scene"]
    spec = mujoco.MjSpec.from_file(str(base))
    if not Path(spec.meshdir).is_absolute():          # the generated XML lives elsewhere
        spec.meshdir = str((base.parent / spec.meshdir).resolve())
    dex3.apply_hardware_zero(spec)                    # Dex3 commands and state in the real hands' thumb_0 angles

    r = cfg["render"]
    spec.visual.global_.offwidth, spec.visual.global_.offheight = map(int, r["offscreen_size"])
    spec.visual.quality.shadowsize = int(r["shadow_size"])
    group = int(r["robot_collision_group"])
    for g in spec.geoms:                              # the robot's only: the course is added below
        if g.parent.name != "world" and (g.contype or g.conaffinity):
            g.group = group

    cam = stereo_camera(cfg)
    body = spec.body(cam.body)
    if body is None:
        raise KeyError(f"robot_camera.body {cam.body!r} not in {base}")
    for name, pos, quat in stereo_poses(cam):
        body.add_camera(name=name, pos=pos.tolist(), quat=quat.tolist(), fovy=cam.fovy_deg,
                        resolution=list(cam.resolution))

    # Added after the robot, so the robot's bodies and joints keep their indices: upstream's DefaultEnv addresses
    # qpos by joint id (pelvis free joint first). A free part's joint name must not contain a robot joint keyword
    # (hip, knee, ankle, waist, shoulder, elbow, wrist, left_hand, right_hand): upstream sorts joints by name.
    for part in parts:
        _add_part(spec, part)
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
    ap.add_argument("--list", action="store_true", help="list the courses")
    ap.add_argument("--config", type=Path, default=COURSES)
    ap.add_argument("--out-dir", type=Path, default=SCENES_DIR)
    a = ap.parse_args(argv)
    if a.list:
        print("\n".join(load_config(a.config)["courses"]))
        return 0
    print(write_scene(a.course, assets.load().source_root, load_config(a.config), a.out_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
