# xr_camera_teleoperation

Camera-based teleoperation of the Unitree G1 (29-DoF, Dex3-1 hands). A monocular RGB camera and the
[WiLoR](https://github.com/rolpotamias/WiLoR) hand-reconstruction model replace the VR headset of Unitree's
[xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate). The operator laptop turns hands into
**end-effector commands**; `ee_runtime` from [robot-runtime](https://github.com/ituealab/robot-runtime) on the robot
(G1's PC2, Jetson) owns the motors, IK and collision checks, the same path policy inference uses.

```
MSI (operator)                                             ROBOT PC2
camera -> WiLoR server  (operator/perception, GPU)
          ZMQ :5590 (hand keypoints, local only)
       -> live UI :8780: clutch, auto-calibration, filters,
          Dex3 retargeting, G1 preview
          ee_cmd :5591 / robot_status :5592 (local ZMQ)
       -> ROS bridge (Humble container): gate, /ee/home
                       /ee/command ──────────────────────►   ee_runtime: IK, collision checks,
                       /ee/status, /ee/state ◄───────────      lowcmd and Dex3 output
       <- episodes: one MCAP per episode ◄───────────────    ZED, RealSense, preview, clock echo
```

* The robot holds until the operator engages (R) **and** fresh commands arrive. When commands stop, the arms hold;
  losing the UI or the link never moves them. See [`RUN.md`](xr_camera_teleop/RUN.md).

## Layout

| Path | What |
| --- | --- |
| `xr_camera_teleop/protocol.py` | `ee_cmd` / `robot_status` msgpack schema, version, validation, seq/session ordering |
| `xr_camera_teleop/ctl.py` | local control sockets + key reader, `xrcam-ctl` (replaces upstream's `xr_ctl.py`) |
| `xr_camera_teleop/operator/` | laptop side: `wrapper.py` (WiLoRTeleWrapper), `clutch.py`, `filters.py`, `mano_to_webxr.py`, `camlevel.py`, `link.py` (ee_cmd publisher + robot status), `node.py` (`xrcam-operator`), `live_ui.py` (`xrcam-ui`) |
| `xr_camera_teleop/operator/perception/` | WiLoR server (ZMQ :5590, HTTP :8770), client, `replay.py` (synthetic / recorded hands), frame sources, overlay |
| `xr_camera_teleop/operator/camera/` | camera helpers (ZED intrinsics, Omron FH, RGB, preprocessing) |
| `xr_camera_teleop/robot/` | code the UI's local preview shares with the robot: `ik.py`, `self_collision.py`, `thumb_shaping.py` |
| `xr_camera_teleop/robot/vendor/` | minimal upstream code (retargeting, frame constants), each file headed with its source and changes |
| `xr_camera_teleop/sim/` | kinematic preview |
| `xr_camera_teleop/viz/`, `tools/`, `tests/` | visualisation, offline tools (`bench_ik`, `render_session`, calibration), unit tests |
| `xr_camera_teleop/assets/` | `g1/` (URDF + MJCF + meshes) and `unitree_hand/` (Dex3 URDFs + `unitree_dex3.yml`): only what is used |
| `deploy/` | `xr_teleop_*.sh` launchers, `ee_bridge.py`, `ros/` (Humble image), PC2 camera services |
| `envs/` | `robot-conda.yml` (conda-forge part), freeze files of the two reference environments |
| `configs/camera/` | camera calibration (Omron FH) |
| `LICENSE-THIRD-PARTY`, `NOTICE` | Apache-2.0 text of xr_teleoperate (+ MIT televuer) and the list of vendored / modified files |

Docs: [`RUN.md`](xr_camera_teleop/RUN.md) (commands, sim, real robot, self-collision), [`CONTRACT.md`](xr_camera_teleop/CONTRACT.md)
(wire formats, frames, clutch), [`GUIDE.md`](xr_camera_teleop/GUIDE.md) (live UI usage, Turkish).

## Install

Two Python environments, because the dependencies do not mix:

| Env | For | How |
| --- | --- | --- |
| operator env (`data/xr-env` on the laptop) | UI, preview, tests | conda-forge pinocchio from `envs/robot-conda.yml`, then `pip install -e .[operator]` |
| perception env (`data/wilor-venv`) | WiLoR server (torch, GPU) | `envs/perception.freeze.txt` + external WiLoR checkout |
| SONIC sim env (`data/sonic-sim-venv`) | MuJoCo sim of the SONIC full-body path + its tools | `deploy/setup_sonic_sim_env.sh` (`envs/sonic-sim.freeze.txt` + pinned SONIC checkout, `configs/sonic_assets.yaml`); see [`docs/quest-sonic/P1_sim.md`](docs/quest-sonic/P1_sim.md) |

**Why conda-forge for pinocchio.** The UI's preview IK uses `pinocchio.casadi`, which the PyPI `pin` wheels do not contain.
The robot side is set up by robot-runtime (`~/robot_runtime_ws`) and needs nothing from these environments.

WiLoR model code and weights are **not** in this repo: clone WiLoR anywhere and set `WILOR_ROOT` (default
`third_party/WiLoR`, git-ignored); only the perception server reads it.

Console scripts (after `pip install -e .`): `xrcam-ui`, `xrcam-perception`,
`xrcam-replay`, `xrcam-ctl`. In the checkout, `python -m xr_camera_teleop.<module>` from the repo root does the same
(e.g. `data/xr-env/bin/python -m xr_camera_teleop.operator.live_ui`). Do not run Python from inside `xr_camera_teleop/`: the
`operator` sub-package would shadow the standard-library module of the same name.

## Run

See [`RUN.md`](xr_camera_teleop/RUN.md). The `ee_cmd` protocol is in [`protocol.py`](xr_camera_teleop/protocol.py).

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

Covers the protocol (round trip, version, NaN, ranges, seq/session), the bridge's command gate, target frames, the
operator link over loopback ZMQ, wrapper / clutch / filters / frames, self-collision and IK behaviour,
perception schema and client. Hardware, real DDS and the Jetson are **not** covered by any automated test.
