# GUI-controlled recording

One episode is one MCAP bag recorded on MSI. The MSI UI runs `ros2 bag record` (ROS 2 Humble, in the
`xrcam-recorder` container) directly, so a finished episode is on disk the moment it stops: nothing
is copied, and PC2 only publishes.

Recording on PC2 lost about 5.5% of `/lowstate` ticks with all topics, against 0.02% on MSI. Live teleop
was unaffected: the recorder adds about 210 Mbit/s to the 1 Gbit link.

## What runs where

- MSI: UI (`recording/backend.py`), one recorder container per episode (`recording/session.py`),
  report (`recording/report.py`).
- PC2: ZED, RealSense, preview on :5604, clock echo on UDP :5603.

## Episode files

`~/teleop_data/<prompt_slug>__<hash>/raw/episode_*/`:

- `bag/bag_0.mcap`: `configs/recording.json` `topics`, as original CDR. Log time is receipt on MSI;
  headers are unchanged. Chunked and uncompressed, since 80% of the payload is already JPEG.
- `report.json`: per-topic rates, camera delay, state and action ages per frame, and `state_ticks`.
- `manifest.json`: prompt, stop reason, `complete`, `incomplete`, `verdict`, `issues`.
- `operator.msgpack`: concatenated msgpack of perception, `ee_cmd` and gesture events, which exist
  only on MSI. Read it with `msgpack.Unpacker`.
- `clocks.json`: PC2 minus MSI offset history. `recorder.log`: rosbag2 output.

`complete` means the bag has its MCAP footer. `incomplete` means anything needs attention.

## Timing

Camera headers carry PC2 capture time and state/action times are MSI receipt, so pairing needs the
two clocks to agree.

- Prevention: PC2 follows MSI through chrony on the wired link (MSI `/etc/chrony/conf.d/xr-teleop.conf`,
  PC2 `/etc/chrony/chrony.conf`). Never point MSI at PC2: PC2's RTC reads March 1970 at every boot.
- Detection: `WallClockGuard` probes the echo every second and shows the offset in the UI. A start is
  refused when the offset is provably over `wall_clock_limit_ms` or the echo is silent, and an episode
  that drifts past it is marked incomplete. Timestamps are never rewritten; `clocks.json` allows
  correction later.

`/lowstate` has a 1 ms tick, so `state_ticks` measures real loss. About 4% of ticks repeat at the
publisher, which is normal.

## Failure behaviour

- Start waits until every `required_topics` entry has messages in the growing bag, and names any
  silent topic. A camera that is down blocks recording.
- The recorder exiting mid-episode stops the episode as incomplete.
- A crashed recorder leaves no footer. `report.py` streams the file, so everything up to the last
  whole chunk still reads; seeking readers fail on such a file, and `mcap recover` repairs it.
- A recorder left running by a crashed UI is stopped when the UI next starts.

## Commands

On MSI, once and after changing `deploy/recorder/`:

```bash
bash deploy/build_recorder.sh
```

Copy the PC2 services, without starting anything:

```bash
XRCAM_RECORDING_TARGET=unitree-eth bash deploy/push_recording.sh
```

On PC2:

```bash
python3 deploy/recording_services.py start all
python3 deploy/recording_services.py status all
python3 deploy/recording_services.py stop all
```

`start all` confirms the ZED before starting RealSense, because starting them together can kill the
ZED driver with `Camera detection timeout`. RealSense takes the D435i exclusively, so stop the ROS 1
`realsense` launcher first. These services never start motor control.

Palm gesture start/stop stays off until `gesture_enabled` is set.

## Not built yet

Nothing converts episodes to LeRobot. The old exporter read a sidecar that no longer exists; a new one
should read the MCAP bag.
