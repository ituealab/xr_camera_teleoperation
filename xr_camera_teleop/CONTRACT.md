# xr_camera_teleop — interface contract

Goal: WiLoR (monocular RGB hand reconstruction) replaces the VR headset input of Unitree's xr_teleoperate. The operator
laptop produces end-effector commands; the robot node (G1's PC2) runs the IK (vendored `G1_29_ArmIK` + self-collision),
Dex3 retargeting and DDS. Outputs of the laptop side:

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
| Robot (py3.10, pinocchio 3.1 + casadi, unitree_sdk2py, cyclonedds 0.10.2, dex_retargeting) | `~/xrcam-env` on PC2 | `robot/node.py` |

Set-up: `deploy/robot_setup.sh` (see README). Add packages to `data/xr-env` with `data/xr-env/bin/python -m pip install ...`.

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

## Operator -> robot: ee_cmd (protocol v1)

Implemented and validated in `xr_camera_teleop/protocol.py`. ZMQ PUB (operator binds `tcp://*:5591`) / SUB with CONFLATE
(robot connects). One single-part msgpack map per operator control tick (~30 Hz):

```text
{
  "v": 1, "type": "ee_cmd",
  "session": int,       # random 52-bit id of the operator process; a restart = new session, seq restarts
  "seq": int,           # >= 0, +1 per message within a session; the robot rejects seq <= last accepted
  "t_send": float,      # operator time.monotonic(); informational ONLY, never compared with the robot's clock
  "follow": bool,       # operator engaged: robot follows only while true
  "home_seq": int,      # counter, +1 per home request; the robot runs IK reset_to_home when it increases
  "hand_active": bool,  # TeleData.motion_data_ready: hand input valid (Dex3 retargeting enabled)
  "left_wrist": [16], "right_wrist": [16],   # 4x4 SE(3) row-major, robot waist frame (the IK's L_ee / R_ee targets)
  "left_hand": [75], "right_hand": [75]      # (25,3) wrist-local WebXR joints in the Unitree hand frame, metres
}
```

Validation on the robot (rejected messages are counted in `robot_status.rejected` / `last_reject` and never feed the
watchdog): msgpack decodes; `v == 1`; `type == "ee_cmd"`; ints / bools of the right type; 16 / 75 finite floats;
|wrist translation| <= 3 m, last matrix row [0 0 0 1], rotation orthonormal with det +1 (tol 1e-2); |hand joint| <= 1 m;
`seq` strictly increasing within a session. A different `session` is accepted as a restarted operator and re-latches the robot
(see below).

Robot behaviour (`robot/watchdog.py`, pure logic with an injectable clock; staleness = robot-local receive time):

| State | Condition | Arms / hands |
| --- | --- | --- |
| HOLD | no command yet, `follow=false`, or latched | hold current / last target |
| FOLLOW | `follow=true`, not latched, last accepted command < `stale` (0.3 s) | IK on the targets, Dex3 retargeting |
| STALE_HOLD | `stale` <= age < `home` (1.5 s) | hold the last safe target |
| HOME | age >= `home` (one-shot) | IK `reset_to_home`, latch |

Latch: set at robot start-up, on a link timeout and on a new operator session; cleared only when a message with
`follow=false` is accepted (so a robot never starts to follow because the link happens to say "follow").
`home_seq`: the first message of a session sets the baseline; an increase triggers `reset_to_home` once.

## Robot -> operator: robot_status (protocol v1)

ZMQ PUB (robot binds `tcp://*:5592`), SUB with CONFLATE on the laptop, ~10 Hz, msgpack map:

```text
{"v": 1, "type": "robot_status", "session": int, "seq": int, "t_send": float,
 "mode": "init"|"hold"|"follow"|"stale_hold"|"homing"|"stopping",
 "latched": bool, "connected": bool, "age_ms": float|null, "watchdog": "ok"|"stale"|"timeout"|"none",
 "ee_session": int|null, "ee_seq": int|null, "follow": bool, "rejected": int, "last_reject": str,
 "ik_ms": float, "ik_mode": "track"|"transition"|"home", "q": [14], "motion": bool, "sim": bool, "vel_limit": float}
```

`connected` = an accepted `ee_cmd` is younger than the stale timeout. The operator treats the status as lost after 1 s
(local receive time) and releases its `follow` flag when the robot stays `latched` for 0.6 s after `follow=true`.

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
                link.py, node.py, live_ui.py
    perception/ server.py, client.py, engine.py, sources.py, overlay.py, replay.py
    camera/     camera_*.py, preprocessing.py
  robot/        node.py, watchdog.py, ik.py, self_collision.py, g1_collision_spheres.py (generated), camera.py (optional)
    vendor/     robot_arm_ik.py, robot_arm.py, robot_hand_unitree.py, hand_retargeting.py, tv_wrapper.py, ... (see NOTICE)
  sim/          g1_dds_mujoco.py, g1_kinematic.py, latency_probe.py
  viz/  tools/  assets/  tests/   (pure-numpy unit tests, run with data/xr-env/bin/python -m pytest)
```
