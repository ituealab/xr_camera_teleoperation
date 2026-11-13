# Running WiLoR -> UI -> ROS bridge -> ee_runtime -> G1

Motors, IK and collision checks belong to `ee_runtime` in [robot-runtime](https://github.com/ituealab/robot-runtime)
(branch `teleop-support` until it merges). This repo turns camera frames into `/ee/command`.

## What runs where

- **MSI:** Omron bridge, WiLoR server, `live_ui` (clutch, filters, Dex3 retargeting, preview), the ROS bridge
  (`deploy/ee_bridge.py` in the `xrcam-ros:humble` container), one recorder container per episode.
- **PC2:** `ee_runtime` (launched by `ee_teleop.launch.py`), ZED, RealSense, preview, clock echo.

## Start

On the robot, with the robot supported, then on MSI:

```bash
xr_teleop up
```

```bash
xr_teleop start
```

`up` starts the cameras, asks before releasing balance, starts `ee_runtime` and sends the arms home. An arm starting in
contact with the body moves at no more than 0.3 rad/s until that contact clears. In the UI, R starts and stops following,
H sends the arms home.

Stop with `xr_teleop stop` on MSI, then `xr_teleop down` on the robot, which sends the arms home first.
Check with `xr_teleop status` and `xr_teleop logs` on either machine.

## What the robot does

| Event | Result |
| --- | --- |
| R not pressed, or `follow=false` | ee_runtime holds the last command |
| UI, bridge or link stops for > 0.3 s | commands expire, ee_runtime holds; the arms do not go home |
| Silence > 1.5 s, or the UI restarted | latched: R must be released and pressed again |
| Restarting or closing the UI | holds; nothing moves |
| IK rejects a pose | arm holds, fingers keep following |
| H | `/ee/home`, collision-checked; `home_blocked` in the status line if no safe step exists |
| Malformed `ee_cmd` | counted as `rejected` in the status line |

Arm speed is 4 rad/s with 0.05 rad of measured lead at 100 Hz output. Dex3 joints are limited to 8 rad/s and 0.5 rad of
lead, so a blocked thumb sees at most 0.75 Nm. These live in robot-runtime's `ee_teleop.launch.py`.

## Testing without motors

Run `ee_runtime` on an isolated ROS domain with `simulation_only:=true simulation_domain:=47`, against a stand-in that
publishes state there. Real motors listen on domain 0. Setting `ROS_DOMAIN_ID=47` for the bridge exercises the whole chain.

The UI alone needs no robot:

```bash
data/xr-env/bin/python -m xr_camera_teleop.operator.synthetic_stream
```

```bash
data/xr-env/bin/python -m xr_camera_teleop.operator.live_ui --no-dataset
```

## Meta Quest 3S as the hand source

The Quest browser streams WebXR head, hand and controller tracking to `operator/quest/server.py`. It publishes two
streams: the same perception message as the WiLoR server on its own ports (ZMQ :5594, HTTP :8771), so the clutch,
filters, retargeting and preview are unchanged, and the full-body `xr_frame` on ZMQ :5596 (`fullbody/xr_frame.py`).
It needs its own venv with Vuer 0.1.6: `python3.12 -m venv data/quest-env`, then
`data/quest-env/bin/pip install -r envs/quest-env.freeze.txt`. (`data/quest-venv`, Vuer 0.0.60 + TeleVuer from
`envs/quest.freeze.txt`, is only used by the raw TeleVuer probe `tools/quest_probe.py`.) Make a certificate for MSI's
Wi-Fi address in `data/quest-cert/` (`openssl req -x509 ... -addext "subjectAltName=IP:192.168.1.105"`).

```bash
data/quest-env/bin/python -m xr_camera_teleop.operator.quest.server
```

The page reconnects by itself after a server restart or stop of any length and stays in VR (Vuer 0.1.6 alone gives
up after 5 tries; `operator/quest/vuer_app.py` injects the fix). A page opened before that fix needs one
`deploy/quest_adb.sh open` and Enter VR.

```bash
data/xr-env/bin/python -m xr_camera_teleop.operator.live_ui --source quest --no-publish --no-dataset
```

On the Quest open `https://192.168.1.105:8012/?ws=wss://192.168.1.105:8012`, typing `https://` (the browser otherwise
tries plain HTTP and reports that it cannot connect), accept the certificate, press Enter VR, put the controllers down.
The server asks the browser for 90 Hz updates (Vuer's default is 30); the distinct sample rate is the headset's XR
frame rate (about 90 Hz with nothing else rendered, about 43 Hz with the stereo view in the P0 measurement,
`docs/quest-sonic/P0_quest_probe.md`). `tools/quest_probe.py` is the raw TeleVuer probe used to bring the headset up.

### xr_frame recording (full-body teleop, P3)

One command records a 5-minute `xr_frame` log from the headset into `data/fullbody-runs/<time>/` and validates it
(timestamps, per-source rates and jitter, staleness exactly as the full-body node sees it, input modes and switches):

```bash
deploy/xr_frame_record.sh            # --minutes N, --no-view; `deploy/xr_frame_record.sh stop` stops the server
```

It starts the Quest server with the synthetic stereo view (the render load of the robot view) unless one is already
running, waits for Enter VR, records, and prints the validation (also `validation.json` in the run directory).
The operator, with the headset on:

| Minute | Do |
|---|---|
| 0-1 | Both controllers in hand. Look around, walk a step, push both sticks, press A/B and X/Y. |
| 1-2 | Put **both** controllers down; both hands free and in view. Crouch once, lean forward and sideways. |
| 2-3 | Pick both controllers up, walk a step, put both down again, use the hands. |
| 3-4 | Both hands tracked, moving. |
| 4-5 | Both controllers back in hand. Once, around 4:30, recenter (hold the Meta button). |

The Quest 3S in Quest Browser never tracks a free hand while the other holds a controller (docs/quest-sonic/
P0_quest_probe.md §6), so the operator rule is: both controllers to walk, both down to use the hands.
`--require-mixed` (validator and recorder) also demands that case, for a platform that delivers it.

`tools/xr_frame_validate.py <run>` re-validates a run; `tools/quest_emulator.mjs` drives the server with an
emulated Quest in headless Chrome (no headset; rates are the browser's, not the Quest's).

## Full-body teleop in simulation (Quest 3S -> SONIC -> MuJoCo G1)

The headset operator drives a simulated G1 (29 DoF + Dex3) through NVIDIA SONIC: the stick walks, the head's yaw
turns the robot, the head's position leans and crouches it, tracked hands drive the wrists and Dex3 fingers, and a
headset panel starts, stops and tunes it. Plan and phase documents: `docs/quest-sonic/` (`PLAN.md`; the morning
guide in Turkish: `SABAH.md`; results: `P8_results.md`).

```bash
deploy/xr_fullbody_sim.sh morning      # sim (morning course) + viewer + SONIC + camera bridge + Quest server + node
deploy/xr_fullbody_sim.sh status       # which processes run, which course
deploy/xr_fullbody_sim.sh stop         # everything (or `stop quest` etc. for one process)
```

`morning` is idempotent (it starts only what is not running) and prints what to do: open `https://192.168.1.105` in
Quest Browser, Enter VR; press a thumbstick for the panel, **Başlat**, stand straight and hold **A** for 1 s (or
**Kalibre et ve devral**); **DURDUR** on the panel ends it. Holding a controller walks (arms parked); with both
controllers down the hands drive the arms (D8).

The stack runs as the user `emin` (its SONIC venv interpreter is in emin's home). From another account, e.g. at the
desk: `sudo -u emin -H deploy/xr_fullbody_sim.sh morning`. The MuJoCo viewer goes to the desk's display: the launcher
uses `DISPLAY` if the shell has one, else the seat's Xwayland, allowed either once with `xhost +SI:localuser:emin`
in the desk session or, with passwordless sudo, through the desk user's X cookie. Logs: `~emin/.local/state/xr_fullbody/`;
every run is recorded under `data/fullbody-runs/<time>/` and replays (`tools/fullbody_replay.py`).

Verification without a person (one stack at a time: DDS domain 42 and the Quest ports are fixed):

```bash
data/xr-env/bin/python -m xr_camera_teleop.tools.acceptance --runs 3             # PLAN §0, emulated Quest, morning course
data/xr-env/bin/python -m xr_camera_teleop.tools.acceptance --only cube --runs 5 # cube pick/place rate (table course)
data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.course_test ramp5 --repeat 5   # synthetic operator over a course
data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.sonic_drive starts --count 20  # gantry: start -> stand
data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.sim_perf --duration 60         # PLAN §7a, while it runs
```

## Updating

PC2 has no GitHub credentials, so copy robot-runtime with a git bundle from MSI, then on PC2:

```bash
source ~/g1_ros_env/env.sh
```

```bash
colcon --log-base ~/robot_runtime_ws/log build --base-paths ~/robot_runtime/ros2 --build-base ~/robot_runtime_ws/build --install-base ~/robot_runtime_ws/install --symlink-install --packages-select robot_control ee_runtime
```

Copy this repo's PC2 files and rebuild the MSI image after changing `deploy/ros/`:

```bash
XRCAM_RECORDING_TARGET=unitree-eth bash deploy/push_recording.sh
```

```bash
bash deploy/build_ros_image.sh
```

`deploy/ros/robot_runtime_interfaces` is a copy of robot-runtime's package: update it when those messages change.
