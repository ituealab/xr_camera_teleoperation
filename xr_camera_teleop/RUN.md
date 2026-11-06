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

The Quest browser streams WebXR hand tracking to `operator/quest/server.py`, which publishes the same message as the
WiLoR server on its own ports (ZMQ :5594, HTTP :8771), so the clutch, filters, retargeting and preview are unchanged.
It needs its own venv (vuer pins numpy below 2): `python3 -m venv data/quest-venv`, then
`data/quest-venv/bin/pip install -r envs/quest.freeze.txt`. `params-proto` must stay below 3, or `from vuer import Vuer`
fails. Make a certificate for MSI's Wi-Fi address in `data/quest-cert/` (`openssl req -x509 ... -addext
"subjectAltName=IP:192.168.1.105"`).

```bash
data/quest-venv/bin/python -m xr_camera_teleop.operator.quest.server
```

```bash
data/xr-env/bin/python -m xr_camera_teleop.operator.live_ui --source quest --no-publish --no-dataset
```

On the Quest open `https://192.168.1.105:8012/?ws=wss://192.168.1.105:8012`, typing `https://` (the browser otherwise
tries plain HTTP and reports that it cannot connect), accept the certificate, press Enter VR, put the controllers down.
The server asks the browser for 90 Hz hand updates (Vuer's default is 30) and measured about 92 Hz of distinct samples.
`tools/quest_probe.py` is the raw TeleVuer probe used to bring the headset up.

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
