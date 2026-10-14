# GUI-controlled ROS bag recording

## Current system

The authoritative training archive is one local PC2 **rosbag2/SQLite3 bag** per episode.
MSI GUI controls start/stop via port 5603 and transfers finalized files via 5605.
Transferred nested files are length/SHA-256 checked before atomic installation.
PC2 originals remain. Automatic LeRobot export is disabled.

GUI preview is independent: ZED left -> latest-only JPEG :5604 -> MSI `/zed.mjpg`.
Preview compression never modifies original stereo pixels stored in the bag.

## Main training topics

- `/zed2i/zed_node/left/image_rect_color`, `/zed2i/zed_node/right/image_rect_color`: original CDR RGB.
- Corresponding camera_info, existing `/joint_states`, `/lf/lowstate` recorded unchanged when present.
- `/teleop/state/{arms,left_hand,right_hand}`: native measured DDS samples with named joints.
- `/teleop/action/{arms,left_hand,right_hand}`: native published desired DDS commands; no fabricated labels.
- `/teleop/robot_state`, `/teleop/robot_action`: 28-dimensional sample-and-hold snapshots up to 100 Hz.
- `/teleop/timing`: source stamps/per-part ages/order/clock identities.
- `/teleop/dds_raw`: full original DDS fields as ROS String JSON, including tick and command gains.
- `/teleop/recording_events`: immutable prompt/start/stop/episode boundaries.
- `/teleop/operator`: forwarded WiLoR/ZMQ and GUI gesture events with original MSI stamps.
- Existing auxiliary ROS sensors/RealSense are retained when available. ZED depth/disparity/confidence/point cloud excluded.
- End-effector FK disabled by default; existing targets may be preserved in auxiliary operator messages.

No retargeting or motor-controller files are modified by recording deployment.

## Timestamps

Camera bag storage time uses the validated ZED IMAGE Unix timestamp; original header preserved.
DDS JointState header/storage time uses the native DDS listener's Unix receipt time (NOT monotonic
uptime). DDS listener timestamp is taken before application queues. Full monotonic/Unix/boot/session
sequence information is retained in `/teleop/timing`, `/teleop/dds_raw` and `index.jsonl`.
State sensor measurement latency is not asserted to be zero.
Camera invalid/zero/future/stale headers, clock steps and mixed PC2 boots are flagged; start refuses
invalid/stale camera clock and stale DDS. Stereo matching uses original header timestamps, not equal
sequence IDs. Sync report separately lists state age, action absence/staleness and stereo gaps.

Native Foxy C++ byte writer uses the installed rosbag2_cpp storage API; no CLI recording process.
Only the archive worker touches the writer. Closing stdin writes metadata.yaml; unexpected parent
death closes stdin too. The actual bag SQLite counts must match the timing index before completion.

## Recording lifecycle

Prompt is always visible in GUI and revision-checked across tabs; captured at trigger.
Open palms facing camera held ~1s trigger once; observed release is required before the next trigger.
Missing hands and repeated stale frames cannot count as release. Starting/stopping ignores additional
toggles. Two-second requested pre-roll is backed by a three-second bounded PC2 buffer.
Late frames captured before the stop boundary are drained for 0.5s. Stop boundary rejects future samples.
All finalization/transfer work is off the 100 Hz receiver and UI pipeline.
Source gaps finalize incomplete. Action absence is reported independently from camera/state clock errors.

For this handover `gesture_enabled=false`: do not enable until the user requests the trial.
GUI remains `--no-publish`; robot motor control is not started by recording services.

## Storage

PC2: `~/.local/share/xrcam_bags/episode_*/{bag/*.db3,bag/metadata.yaml,index.jsonl,sync_report.json,manifest.json}`.
MSI: `~/teleop_data/<prompt_slug>__<hash>/raw/episode_*/pc2/bag/` contains the verified copy.
MSI auxiliary `samples.msgpack` and quality/transfer/clocks files remain as diagnostic sidecars;
the main camera/state/action/operator streams are in the ROS bag.

## Commands

On MSI, copy only recording files (no service launch):

```bash
bash deploy/push_recording.sh --dry-run
bash deploy/push_recording.sh
```

On PC2:

```bash
bash deploy/build_bag_writer.sh
python3 deploy/recording_services.py start all   # telemetry, bag manager, JPEG preview ONLY
python3 deploy/recording_services.py status all
python3 deploy/recording_services.py stop all
```

These sources subscribe to existing cameras; they do not start the ZED driver or motor control.
Managed PID state includes boot identity and process start ticks. No `pkill -f` matching broad SSH commands.
Do not run legacy `cameras` source simultaneously with `bag` (same ports).

## Verification / trial boundary

Offline tests cover original CDR, prebuffer/stop boundaries, native joint order and Unix stamps,
duplicate commands, gesture release, missing/future samples, nested checksum transfer and ROS bag
message counts. Native G1 synthetic-file tests verified CDR bytes and Unix timestamps and that a parent
crash closes/finalizes the writer. No live training episode was started for the final handover.
When the user requests the trial, validate physical palms, actual source rates, bag pixel/state/action
coverage and sync_report. Physical sensor latency and guaranteed loss-free 30/100 Hz are not claimed.
