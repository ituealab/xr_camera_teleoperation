# xr_camera_teleop — interface contract

Goal: WiLoR (monocular RGB hand reconstruction) replaces the VR headset input of Unitree's xr_teleoperate. The operator
laptop produces end-effector commands and Dex3 targets; `ee_runtime` from robot-runtime (G1's PC2) runs the IK, collision
checks and motor output. Outputs of the laptop side:

1. RGB overlay (served by the perception server, `operator/perception/overlay.py`).
2. MuJoCo "floating hands": human wrists + 21 keypoints per hand in 3D.
3. `ee_cmd` (below): wrist targets + Dex3 retargeting input to the robot, RELATIVE (clutch) mapping; a local G1
   preview (same IK code) in the live UI.

The WiLoR model (code + weights) is NOT part of this repo: `operator/perception/engine.py` loads it from `$WILOR_ROOT`
(default `third_party/WiLoR`, git-ignored). Camera helpers copied from the original `wilor_teleop/` live in
`xr_camera_teleop/operator/camera/`.

## Environments

| Env | Path | Used by |
| --- | --- | --- |
| Perception (torch, WiLoR, pyzed) | `data/wilor-venv` | `operator/perception/server.py` only |
| Operator / consumers (py3.10, mujoco, pinocchio 3.1 + casadi, numpy 1.26) | `data/xr-env` (micromamba) | everything else on the laptop, sim, tests |
| Former robot env (py3.10, pinocchio 3.1 + casadi, unitree_sdk2py, cyclonedds 0.10.2, dex_retargeting; Dex3 tools only) | `~/xrcam-env` on PC2 | `robot/node.py` |

Add packages to `data/xr-env` with `data/xr-env/bin/python -m pip install ...`.

## Wire format (perception -> consumers)

ZMQ PUB, `tcp://127.0.0.1:5590`, single-part msgpack messages (no topic frame),
consumers use SUB with CONFLATE=1 (latest only). One message per processed frame:

```text
{
  "v": 1,
  "frame_id": int,
  "t_capture": float,      # time.monotonic() at frame dequeue, seconds
  "t_publish": float,      # time.monotonic() at publish
  "image_size": [w, h],
  "intrinsics": {"fx","fy","cx","cy"},
  "source": "live" | "replay",
  "max_hand_depth": float,  # OPTIONAL. Hands with wrist z beyond this (m) are not the operator's; overrides the
                            # consumers' 0.9 m default (Omron overhead camera: table distance + 0.05)
  "cam_R_level": [[3x3]],  # OPTIONAL. Camera levelling from the ZED IMU gravity (ZedSdkSource only):
                           # v_level = R @ v_cam (OpenCV axes), +y_level = gravity, yaw untouched
  "hands": {               # key present only when that hand is detected this frame
    "left"|"right": {
      "conf": float,                 # detector box confidence
      "wrist_pos": [3],              # metres, CAMERA frame (OpenCV: +x right, +y down, +z forward)
      "wrist_rot": [[3x3]],          # proper SO(3), camera frame, MANO root orientation
                                     # (left hand: handedness mirror already resolved, det=+1)
      "keypoints": [[21x3]],         # metres, CAMERA frame, MANO/OpenPose order, index 0 = wrist
      "keypoints_2d": [[21x2]]       # full-image pixels
    }
  }
}
```

MANO/OpenPose 21 order: 0 wrist; thumb 1-4; index 5-8; middle 9-12; ring 13-16;
pinky 17-20 (each finger: MCP/CMC, PIP, DIP, tip).

Python client (stdlib + pyzmq + msgpack only, importable from both envs):
`xr_camera_teleop/operator/perception/client.py` -> `HandStreamClient(addr).latest(max_age=0.3) -> dict | None`.
(This stream never leaves the laptop: its `t_publish` staleness check uses the shared host clock.)

HTTP (same server, port 8770): `/` simple page, `/overlay.mjpg`, `/latest.json`, `/health`.

## Camera tilt

Consumers compute `p_W = R_W_CAM @ R_level @ p_cam` (same for rotations/keypoints) with
`R_level = msg["cam_R_level"]` when present, else a fixed pitch from the CLI `--cam-pitch-deg`
(positive = camera looking DOWN; `operator/camlevel.py`, default 0 = level). The `--record-video`
`.json` sidecar carries `cam_R_level` (mean over the first second) when an IMU was available.

## Frames used downstream

- `CAM`: OpenCV camera frame above.
- `W`: viz/robot world, z up, x forward, y left. `R_W_CAM = [[0,0,1],[-1,0,0],[0,-1,0]]`
  (camera forward -> x, camera right -> -y, camera down -> -z). Camera treated
  as the operator's head, looking forward.
- The robot IK / Dex3 retargeting expect `TeleData.left/right_wrist_pose` in robot basis, waist
  origin, Unitree arm URDF initial-pose convention, and `left/right_hand_pos`
  as (25,3) WebXR-ordered joints expressed in the wrist frame (conventions: docstring and constants of
  `robot/vendor/tv_wrapper.py`, vendored from televuer).

## Relative (clutch) mapping

On calibrate (both hands seen, operator in a comfortable neutral):
`H_ref = human wrist pose (W)`, `R_ref = robot neutral wrist pose (waist)`.
Then each frame:
`p_robot = p_R_ref + k * (p_H - p_H_ref)` (k default 0.8, tunable, per-axis optional)
`R_robot = (R_H R_H_ref^T) R_R_ref` expressed in W, i.e. delta rotation applied in world.
Auto-calibration: both hands detected and nearly still (within 3 cm of their 1 s window mean) for 1.0 s.
A hand lost longer than the stale timeout (1 s) drifts back to the robot neutral (0.15 m/s, 0.8 rad/s)
and is re-anchored without a jump when it returns.
Camera placement therefore does not matter; only motion is transferred.
Handedness guard: once the operator heading is known (after calibration), with both hands seen the
"left" wrist must not be > 3 cm right of the "right" wrist (W +y = operator's left). Swapped for 0.15 s
-> stderr warning, both arms and hands return to the robot neutral until the sides are correct for 0.3 s;
calibration is postponed while the labels are swapped.
Fingers are not clutched (dex-retargeting uses hand-local vectors).

## UI <-> bridge: ee_cmd and robot_status (protocol v2)

Local ZMQ between `live_ui` and `deploy/ee_bridge.py`; the schema is the header of `protocol.py`. `ee_cmd` carries
`torso_link` palm poses (`operator/ee_frames.py` converts the clutch's L_ee/R_ee targets) and 14 Dex3 motor radians.

Bridge behaviour (`operator/gate.py`, pure logic with an injectable clock; staleness = local receive time):

| State | Condition | Result |
| --- | --- | --- |
| HOLD | no command yet, `follow=false`, or latched | forwards nothing; ee_runtime holds |
| FOLLOW | `follow=true`, not latched, last accepted command < 0.3 s old | `/ee/command`, TTL 0.3 s |
| STALE_HOLD | 0.3 s <= age < 1.5 s | forwards nothing |
| HOLD + latch | age >= 1.5 s | forwards nothing, latches |

Latch: set at start-up, on a link timeout and on a new UI session; cleared only when a message with `follow=false` is
accepted. `home_seq`: the first message of a session sets the baseline; an increase calls `/ee/home` once, skipped while
home is active because a second call stops it. No command is forwarded while homing.

## Self-collision protection

`robot.ik.make_arm_ik()` returns `SafeG1_29_ArmIK` (plain subclass of the vendored upstream `G1_29_ArmIK`, same
`solve_ik(left, right, q, dq) -> (q14, tauff14)`). Layer 1: sphere/capsule constraints in the IPOPT problem
(`robot/self_collision.SphereModel`, data in the generated `robot/g1_collision_spheres.py`, regenerate with
`tools/fit_self_collision.py`). Layer 2: hpp-fcl check of every output on the URDF collision meshes
(`self_collision.MeshGate`); a colliding/NaN q is replaced by the last safe q. Left-vs-right hand/wrist contact is allowed
in both layers. Branch escape: a stuck warm-started branch is replaced (per arm, smooth 2 rad/s joint-space transition) by a
home-seeded solution when that is clearly better.
`ik.reset_to_home()` = rate-limited, collision-free joint-space return to q=0 + IK state reset; wired to
`WiLoRTeleWrapper.on_home_request` (called by `reset()`/`request_reset()` and the L/R swap latch): on the laptop to the local
preview IK and to `OperatorLink.request_home()` (-> `home_seq` -> robot). Off switch: `xrcam-robot --no-self-collision` /
`XRCAM_SELF_COLLISION=0` (`make_arm_ik(self_collision=False)` = plain vendored upstream IK). On a solver failure the current
motor q is held (as upstream) if it is collision-free.

## Directory layout

```text
xr_camera_teleop/
  protocol.py   ctl.py   paths.py
  operator/     wrapper.py (WiLoRTeleWrapper), clutch.py, filters.py, mano_to_webxr.py, camlevel.py, synthetic_stream.py,
                link.py, gate.py, ee_frames.py, live_ui.py
    perception/ server.py, client.py, engine.py, sources.py, overlay.py, replay.py
    camera/     camera_*.py, preprocessing.py
  robot/        ik.py, self_collision.py, g1_collision_spheres.py (generated), thumb_shaping.py   (shared with the preview)
    vendor/     robot_arm_ik.py, hand_retargeting.py, tv_wrapper.py, ... (see NOTICE)
  sim/          g1_kinematic.py
  viz/  tools/  assets/  tests/   (pure-numpy unit tests, run with data/xr-env/bin/python -m pytest)
```
