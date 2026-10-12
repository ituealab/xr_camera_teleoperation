# xr_camera_teleoperation

Camera-based teleoperation of the Unitree G1 (29-DoF, Dex3-1 hands). A monocular RGB camera and the
[WiLoR](https://github.com/rolpotamias/WiLoR) hand-reconstruction model replace the VR headset of Unitree's
[xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate). The operator laptop only sends **end-effector
commands**; the robot (G1's PC2, Jetson) runs the IK, the Dex3 retargeting and talks DDS.

```
LAPTOP (operator)                                          ROBOT PC2 (or the laptop, against the MuJoCo sim)
camera -> WiLoR server  (operator/perception, GPU)
          ZMQ :5590 (hand keypoints, local only)
       -> WiLoRTeleWrapper  (ego clutch, auto-calibration, filters,
                             MANO -> WebXR, handedness guard)
       -> ee_cmd  PUB tcp://*:5591   ───────────────────►   xrcam-robot: SUB ee_cmd (latest only)
          live UI :8780 (overlay | floating hands | local        validate -> watchdog gate
          G1 preview, calibrate / reset / start robot)            safe G1_29 IK (self-collision) + Dex3 retargeting
                                    ◄───────────────────        DDS rt/arm_sdk (--motion) or rt/lowcmd, rt/dex3/*
          robot_status SUB :5592     (mode, link, watchdog,       robot_status PUB tcp://*:5592
                                      IK ms, q)
```

* Nothing of the camera path runs on the robot, and the teleop loop never subscribes to the robot's head camera
  (watch it outside; `xrcam-robot --robot-camera` is an optional, off-by-default thin client, see `robot/camera.py`).
* The robot holds its **current posture** until the operator engages (`r` / UI button) **and** fresh `ee_cmd` arrive.
  No `ee_cmd` for > 0.3 s: hold the last safe q; for > 1.5 s: collision-free joint-space return to home, latched
  until the operator re-engages. Clocks of the two machines are never compared (local receive time + seq).

## Layout

| Path | What |
| --- | --- |
| `xr_camera_teleop/protocol.py` | `ee_cmd` / `robot_status` msgpack schema, version, validation, seq/session ordering |
| `xr_camera_teleop/ctl.py` | local control sockets + key reader, `xrcam-ctl` (replaces upstream's `xr_ctl.py`) |
| `xr_camera_teleop/operator/` | laptop side: `wrapper.py` (WiLoRTeleWrapper), `clutch.py`, `filters.py`, `mano_to_webxr.py`, `camlevel.py`, `link.py` (ee_cmd publisher + robot status), `node.py` (`xrcam-operator`), `live_ui.py` (`xrcam-ui`) |
| `xr_camera_teleop/operator/perception/` | WiLoR server (ZMQ :5590, HTTP :8770), client, `replay.py` (synthetic / recorded hands), frame sources, overlay |
| `xr_camera_teleop/operator/camera/` | camera helpers (ZED intrinsics, Omron FH, RGB, preprocessing) |
| `xr_camera_teleop/robot/` | robot side: `node.py` (`xrcam-robot`), `watchdog.py`, `ik.py` (SafeG1_29_ArmIK), `self_collision.py`, `g1_collision_spheres.py` (generated), `camera.py` (optional) |
| `xr_camera_teleop/robot/vendor/` | minimal upstream code (arm IK, arm / Dex3 controllers, retargeting, frame constants), each file headed with its source and changes |
| `xr_camera_teleop/sim/` | MuJoCo G1 DDS simulator (`xrcam-sim`), kinematic preview, latency probe |
| `xr_camera_teleop/viz/`, `tools/`, `tests/` | visualisation, offline tools (`e2e_loopback`, `bench_ik`, `render_session`, calibration), unit tests |
| `xr_camera_teleop/assets/` | `g1/` (URDF + MJCF + meshes) and `unitree_hand/` (Dex3 URDFs + `unitree_dex3.yml`): only what is used |
| `deploy/` | `robot_setup.sh` (on PC2), `push_to_robot.sh` (rsync + setup) |
| `envs/` | `robot-conda.yml` (conda-forge part), freeze files of the two reference environments |
| `configs/camera/` | camera calibration (Omron FH) |
| `LICENSE-THIRD-PARTY`, `NOTICE` | Apache-2.0 text of xr_teleoperate (+ MIT televuer) and the list of vendored / modified files |

Docs: [`RUN.md`](xr_camera_teleop/RUN.md) (commands, sim, real robot, self-collision), [`CONTRACT.md`](xr_camera_teleop/CONTRACT.md)
(wire formats, frames, clutch), [`GUIDE.md`](xr_camera_teleop/GUIDE.md) (live UI usage, Turkish).

## Install

Three Python environments, because the dependencies do not mix:

| Env | For | How |
| --- | --- | --- |
| robot env (`~/xrcam-env` on PC2) | `xrcam-robot`, IK, Dex3 | `bash deploy/robot_setup.sh` (see below) |
| operator env (`data/xr-env` on the laptop) | operator, UI, sim, tests | same script, `XRCAM_OPERATOR=1` |
| perception env (`data/wilor-venv`) | WiLoR server (torch, GPU) | `envs/perception.freeze.txt` + external WiLoR checkout |

**Why conda-forge for pinocchio.** The IK uses `pinocchio.casadi`. The PyPI `pin` wheels (2.7.0, 3.3.1, 3.7.0, 4.1.0
were tried) do not contain it; only conda-forge's `pinocchio` does. `deploy/robot_setup.sh` therefore installs a
user-space micromamba (no system packages), creates the env from `envs/robot-conda.yml` (python 3.10, pinocchio 3.1.0,
casadi 3.6.7, hpp-fcl, numpy 1.26.4, nlopt 2.7.1: the laptop's versions, so the IK is bit-identical on both machines) and then
`uv pip install`s the rest: this package, `cyclonedds==0.10.2` (built against `~/cyclonedds_ws` on aarch64, PyPI has no
aarch64 wheel), `unitree_sdk2py`, the silencht `dex_retargeting` fork (pinned commit; it needs `torch==2.3.0`, CPU is
enough) with `--no-deps` so that no PyPI `pin` shadows the conda one.

```bash
# robot (PC2, Ubuntu 20.04 aarch64): from the laptop
bash deploy/push_to_robot.sh            # rsync (without data/) to unitree@192.168.123.164:~/xr_camera_teleoperation, then setup there
bash deploy/push_to_robot.sh --dry-run  # only show what would be copied
# or on the robot:  cd ~/xr_camera_teleoperation && bash deploy/robot_setup.sh

# operator laptop (same recipe + MuJoCo / OpenCV)
XRCAM_ENV=$PWD/data/xr-env XRCAM_OPERATOR=1 TORCH_INDEX=https://download.pytorch.org/whl/cpu bash deploy/robot_setup.sh
```

`pip install -e .[robot]` also works in a plain venv for everything that does not need pinocchio (it installs the
pip parts; `import xr_camera_teleop.robot.node`, the protocol and the watchdog tests run there; PyPI `pin` is then
only pulled in by dex_retargeting's metadata and cannot do the IK). Importing `robot.node` needs neither torch,
mujoco nor opencv: DDS, pinocchio/casadi and dex_retargeting are imported when the hardware is built.

WiLoR model code and weights are **not** in this repo: clone WiLoR anywhere and set `WILOR_ROOT` (default
`third_party/WiLoR`, git-ignored); only the perception server reads it.

Console scripts (after `pip install -e .`): `xrcam-robot`, `xrcam-operator`, `xrcam-ui`, `xrcam-sim`, `xrcam-perception`,
`xrcam-replay`, `xrcam-ctl`. In the checkout, `python -m xr_camera_teleop.<module>` from the repo root does the same
(e.g. `data/xr-env/bin/python -m xr_camera_teleop.robot.node`). Do not run Python from inside `xr_camera_teleop/`: the
`operator` sub-package would shadow the standard-library module of the same name.

## Run the simulation (laptop only, loopback)

```bash
PY=data/xr-env/bin/python
$PY -m xr_camera_teleop.sim.g1_dds_mujoco --domain 1 --iface lo                       # 1: simulator (add --headless)
$PY -m xr_camera_teleop.operator.perception.replay --synthetic                        # 2: hands (or the live WiLoR server)
$PY -m xr_camera_teleop.robot.node --sim --network-interface lo                       # 3: robot node (holds, waits)
$PY -m xr_camera_teleop.operator.node                                                 # 4: operator; press r to engage
#   or the web UI instead of 4:  $PY -m xr_camera_teleop.operator.live_ui             #    http://127.0.0.1:8780/
```

Operator keys: `r` start following, `s` stop (robot holds), `h` home / reset, `c` calibrate, `space` clutch pause,
`q` quit (robot gets a home request). Robot keys: `q` stop (arms go home, exit), `h` home. Without a TTY:
`xrcam-ctl operator start|stop|home|calibrate|quit|status`, `xrcam-ctl robot stop|home|status`.

Automated end-to-end check (headless, private ports, DDS domain 1 on `lo`, ~70 s):
`$PY -m xr_camera_teleop.tools.e2e_loopback` (arms follow a hand step, freeze when the operator is SIGKILLed, go home
by the watchdog, `xrcam-ctl robot stop` exits cleanly).

## Run on the real robot

Read this before the first test. Everything here is a checklist, nothing starts by itself.

1. **First tests with the robot hung / supported** and a person at the remote. Remote damping: **L2+B** at any time.
2. `--motion` (recommended): the arms go through `rt/arm_sdk`, blended with the robot's own balance controller; the robot
   must be in **Regular mode (R1+X)**, not in Running mode. Without `--motion` the node puts the robot into debug mode
   (`rt/lowcmd`): it will **not** balance, use it only hung.
3. Keep the velocity limit low: `--arm-vel-limit 3.0` is the default (upstream's was 30 rad/s); lower it for the first runs
   (`--arm-vel-limit 1.5`). Keep `--k` <= 0.8 on the operator.
4. Start the robot node first, then the operator (the robot refuses `follow` until it has seen `follow=false`):
   ```bash
   # on PC2 (use `ip a` to find the NIC on 192.168.123.x, e.g. eth0)
   ~/xrcam-env/bin/xrcam-robot --motion --network-interface eth0 --arm-vel-limit 1.5 --operator tcp://<laptop-ip>:5591
   # on the laptop
   data/xr-env/bin/python -m xr_camera_teleop.operator.node --robot tcp://192.168.123.164:5592    # or operator.live_ui --robot ...
   ```
   Hold both hands in a comfortable neutral until the clutch is calibrated, then press `r` (UI: "Start robot").
   The first command is the neutral pose, so start with the arms near home.
5. Stop: `q` on the robot (arms home, arm_sdk weight ramp 1 -> 0, clean exit), `s` on the operator (robot holds), or just kill
   the operator: the watchdog holds after 0.3 s and returns home after 1.5 s (`--stale-timeout`, `--home-timeout`).
6. Re-measure the IK time on the robot before relying on 30 Hz (next section).

## IK timing

Measured on the development laptop (x86-64, RTX 5090 machine, loaded by other processes; `tools/bench_ik`, 480 solves of
random reachable targets, 1/4 of them pulled into the torso; pinocchio 3.1.0, casadi 3.6.7):

| path | median | p95 | max |
| --- | --- | --- | --- |
| free space (fast path, unconstrained problem) | 1.7 ms | 2.4 ms | 4.4 ms |
| constrained (IPOPT with self-collision constraints) | 15 ms | 51 ms | 103 ms |
| plain upstream IK (no self-collision) | 1.7 ms | 2.1 ms | 15 ms |

**These numbers must be re-measured on PC2 (Jetson Orin NX, aarch64):** `~/xrcam-env/bin/python -m xr_camera_teleop.tools.bench_ik`.
If the constrained solves are too slow there, lower `--frequency` or run `xrcam-robot --no-self-collision` (debug only: no
self-collision protection, branch escape or joint-space home).

## The ee_cmd protocol

Full field list in [`xr_camera_teleop/CONTRACT.md`](xr_camera_teleop/CONTRACT.md) and `protocol.py`. One msgpack map per
control tick (~30 Hz) over ZMQ PUB (operator, `tcp://*:5591`) / SUB with CONFLATE (robot):

```text
{"v": 1, "type": "ee_cmd", "session": <random id of the operator process>, "seq": <+1 per message>, "t_send": <info only>,
 "follow": bool, "home_seq": <counter, +1 per home request>, "hand_active": bool,
 "left_wrist": [16], "right_wrist": [16],      # 4x4 row-major, robot waist frame = IK targets
 "left_hand": [75], "right_hand": [75]}        # (25,3) wrist-local hand joints, Dex3 retargeting input
```

The robot rejects (and counts) messages with a wrong version/type, non-finite or out-of-range values, a non-rotation wrist
matrix, or `seq` not increasing within a session. A new `session` (operator restart) re-latches the robot: `follow=true` is
ignored until a message with `follow=false` has been seen (also at robot start-up and after a watchdog timeout). The robot
answers with `robot_status` (mode `hold|follow|stale_hold|homing|stopping`, `latched`, `connected`, `age_ms`, watchdog state,
IK ms and mode, arm q) at 10 Hz on `tcp://*:5592`; the live UI and `xrcam-operator` show it.

## What is vendored, and from where

Upstream `unitreerobotics/xr_teleoperate` @ `817fb00` (Apache-2.0) is no longer a submodule and is not patched at run time;
the needed parts live in `xr_camera_teleop/robot/vendor/` (each file starts with its source path, the commit and a
"MODIFIED" note; the full list is in [`NOTICE`](NOTICE), licenses in [`LICENSE-THIRD-PARTY`](LICENSE-THIRD-PARTY)):

| Vendored file | Upstream source | Main changes |
| --- | --- | --- |
| `robot_arm_ik.py` | `teleop/robot_control/robot_arm_ik.py` (G1_29_ArmIK) | no meshcat, package-relative assets, model cache in `~/.cache` |
| `robot_arm.py` | `teleop/robot_control/robot_arm.py` (G1_29 controller, joint enums) | velocity limit is an argument (3.0 rad/s default, also in sim), starts at the current posture, `stop()` |
| `robot_hand_unitree.py` | `teleop/robot_control/robot_hand_unitree.py` (Dex3_1_Controller) | thread instead of process, `set_input()`, starts at the current hand state |
| `hand_retargeting.py` | `teleop/robot_control/hand_retargeting.py` (Dex3 only) | package assets, lazy `dex_retargeting` import |
| `weighted_moving_filter.py`, `motion_switcher.py`, `dds_utils.py` | `teleop/utils/*`, `teleop/robot_control/dds_utils.py` | trimmed, logging |
| `tv_wrapper.py` | `televuer/.../tv_wrapper.py` @ `766de45` (MIT) | frame constants and `TeleData` only |

Assets (`assets/g1`, `assets/unitree_hand`) are unmodified copies of the files the URDF/MJCF/yml reference. Not vendored:
`dex_retargeting` (silencht fork, pinned commit, installed from git), `unitree_sdk2_python` (pinned commit), `teleimager`
(only imported lazily by the optional `--robot-camera`).

## Tests

```bash
data/xr-env/bin/python -m pytest -q xr_camera_teleop/tests          # ~2 min, no GPU / robot / network needed
```

Covers the protocol (round trip, version, NaN, ranges, seq/session), the watchdog gate and the robot node with fake
hardware, the operator link over loopback ZMQ, wrapper / clutch / filters / frames, self-collision and IK behaviour,
perception schema and client. Hardware, real DDS and the Jetson are **not** covered by any automated test.

### Offline robot check (no DDS)

```bash
xrcam-robot --dry-run --operator tcp://192.168.123.100:5591 --status-bind tcp://*:5592
```

Runs the real collision-checked IK, command validation, watchdog and status publisher with in-memory arms
starting at zero. Does not initialize DDS, switch robot modes, subscribe to the robot camera, or build the
Dex3 controller. Status includes `dry_run=true`; joint values are virtual, not measured robot state.
This checks arm IK and transport only; Dex3 retargeting and physical tracking require separate checks.
`--dry-run` rejects `--motion`, `--sim`, and `--robot-camera`.

### Two-stage session on PC2 (supported robot, explicit lowcmd)

The lowcmd path requires the Unitree motion service to be released first. The node checks this before
creating any motor publishers. It holds leg/waist joints at their measured startup posture; it provides
no balance controller. A person must have the remote ready (L2+B damping).

```bash
cd ~/xr_camera_teleoperation
bash deploy/robot_session.sh prepare           # stage 1: arms q=0, then ready; target acceptance OFF
bash deploy/robot_session.sh status            # wait for stage=ready
bash deploy/robot_session.sh enable            # stage 2: allow target acceptance
# Then engage Raider's operator/UI after it has sent follow=false.
bash deploy/robot_session.sh hold              # disable target acceptance, hold last targets
bash deploy/robot_session.sh home              # return arms home; disables target acceptance
bash deploy/robot_session.sh stop              # home and exit
```

For offline validation: `prepare --dry-run`. Status carries `stage` and `targets_enabled`;
`enable` is rejected before measured arm home is within 0.03 rad. The old operator follow state
cannot trigger movement immediately after enabling: it must first release follow, then engage again.
`XRCAM_IK_HZ` sets the IK loop rate and `XRCAM_ARM_VEL_LIMIT` sets the arm speed limit.
Real hardware always needs explicit `--debug` (lowcmd) or `--motion` (arm_sdk).
