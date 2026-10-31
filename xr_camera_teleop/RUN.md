# Running WiLoR -> operator -> ee_cmd -> robot node -> G1 (sim and real: same code path)

Live web UI (3 panels) and the Turkish usage guide: [GUIDE.md](GUIDE.md) (`operator/live_ui.py`, port 8780).
Wire formats: [CONTRACT.md](CONTRACT.md). Overview / install: [../README.md](../README.md).

Two programs talk over ZMQ (no DDS between them):

* **operator** (laptop): `xrcam-operator` (headless) or `xrcam-ui` (web UI): WiLoR stream -> `WiLoRTeleWrapper` ->
  `ee_cmd` PUB `tcp://*:5591`; subscribes to the robot's `robot_status` (`:5592`).
* **robot node** (`xrcam-robot`, on PC2 or on the laptop with `--sim`): SUB `ee_cmd` -> watchdog gate -> safe IK +
  Dex3 retargeting -> `G1_29_ArmController` (`rt/lowcmd` or `rt/arm_sdk`) + `Dex3_1_Controller` (`rt/dex3/*`);
  PUB `robot_status`. `--sim` only selects DDS domain 1 (real: domain 0).

Env: `PY=data/xr-env/bin/python` (laptop; python 3.10, pinocchio 3.1.0 + casadi, unitree_sdk2py, cyclonedds 0.10.2,
dex_retargeting, mujoco). Hand stream: `operator/perception/server.py` (live, `data/wilor-venv`) or
`operator/perception/replay.py --synthetic` / `sim/latency_probe.py publish`. Every command below is `$PY -m <module>` from the repo
root (or the console script after `pip install -e .`).

## Simulation (loopback, domain 1)

```bash
# terminal 1: simulator (viewer window, 60 Hz passive viewer; add --headless for none)
$PY -m xr_camera_teleop.sim.g1_dds_mujoco --domain 1 --iface lo
# terminal 2: hand stream (live WiLoR server, or synthetic)
$PY -m xr_camera_teleop.operator.perception.replay --synthetic
# terminal 3: robot node: holds the current posture, waits for the operator
$PY -m xr_camera_teleop.robot.node --sim --network-interface lo
# terminal 4: operator (needs a TTY for the keys; else use xrcam-ctl)
$PY -m xr_camera_teleop.operator.node            # or: $PY -m xr_camera_teleop.operator.live_ui
```

Operator keys (terminal 4): `r` start following (the clutch calibrates when both hands are seen still for 1 s - or press `c` -
so hold a comfortable neutral, hands near the home pose, then `r`), `s` stop (robot holds where it is), `h` home / reset
(clutch uncalibrated + robot IK joint-space home), `c` calibrate, `space` pause the clutch, `q` quit (robot gets a home request).
Robot keys: `q` stop (arms go home, clean exit), `h` home. The robot only follows when `follow` AND fresh `ee_cmd`:
it ignores `follow=true` until it has seen `follow=false` after start-up, a link timeout or an operator restart (the operator
releases its own follow flag in that case and prints why).
Without a TTY: `$PY -m xr_camera_teleop.ctl operator start|stop|home|calibrate|pause|quit|status`, `... ctl robot stop|home|status`.

Operator options: `--addr tcp://127.0.0.1:5590` (hand stream), `--robot tcp://<robot>:5592` (status), `--ee-bind tcp://*:5591`,
`--rate 30`, `--k 0.8`, `--no-auto-calibrate`, `--cam-pitch-deg`, `--start` (engage 1 s after start). `operator.live_ui` takes the
same link options plus `--no-publish` (preview only) and `--no-self-collision`.
Robot options: `--sim`, `--motion`, `--network-interface`, `--operator tcp://<laptop>:5591`, `--status-bind`, `--frequency 30`,
`--arm-vel-limit 3.0`, `--stale-timeout 0.3`, `--home-timeout 1.5`, `--no-hands`, `--no-self-collision`, `--robot-camera` (optional, off).
Sim options: `--headless`, `--report-self-collision` (checks the arm q on the URDF meshes every 50 ms and prints contacts; the MJCF
has contype=0, so the sim itself never blocks them), `--elastic-band` (free base hung from a virtual spring instead of welded pelvis),
`--dt 0.0005`, `--view-hz 30`, `--trace f.npz`, `--summary f.json`, `--duration S`.
Debug-only kinematic tool (no DDS): `$PY -m xr_camera_teleop.sim.g1_kinematic`.

Automated: `$PY -m xr_camera_teleop.tools.e2e_loopback` runs sim + robot node + synthetic stream + operator headless
on private ports, SIGKILLs the operator and checks follow / hold / watchdog-home / clean exit from the sim trace.

Notes: `--motion` in sim = rt/arm_sdk with the weight in motor_cmd[29].q; Unitree's locomotion/balance
controller is closed source, so the sim holds the legs (and un-blended joints) with a PD hold at q=0.
The sim prints RTF and step-time p50/p99 every 5 s. The MotionSwitcher call (debug mode) is skipped in `--sim`.

## Watchdog and start/stop semantics (robot side)

| Situation | Robot |
| --- | --- |
| start-up, `follow=false`, latched, no `ee_cmd` yet | holds the current posture (hands hold too) |
| `follow=true`, last accepted `ee_cmd` < 0.3 s old | follows: IK on the wrist targets, Dex3 retargeting |
| 0.3 s < age < 1.5 s | holds the last safe target (resumes without a latch if `ee_cmd` return before 1.5 s) |
| age > 1.5 s | IK `reset_to_home` (collision-free, 2 rad/s), then holds; **latched** until `follow=false` is seen |
| `home_seq` increased (UI Reset, handedness-swap latch) | IK `reset_to_home`, same as above, no latch |
| rejected message (version, NaN, range, seq not increasing) | counted (`rejected` in `robot_status`), does not feed the watchdog |
| local `q` / SIGINT / SIGTERM / `xrcam-ctl robot stop` | IK home move, then `ctrl_dual_arm_go_home` (+ arm_sdk weight ramp 1 -> 0 with `--motion`), threads stopped, exit code 0 |

## Real robot (domain 0)

One command on each machine, with the robot supported:

```bash
xr_teleop up     # on the robot: cameras, developer mode, arms to home, enable
xr_teleop start  # on MSI: Omron, WiLoR, UI
```

Then press R in the UI at http://127.0.0.1:8780/. `xr_teleop up` starts the cameras first, so a camera
fault stops it before balance is released. It asks for confirmation unless the robot is already in
developer mode; `--yes` skips that. To stop, run `xr_teleop down` on the robot and `xr_teleop stop` on
MSI. `xr_teleop hold` freezes in place and L2+B is remote damping at any time. Install the launchers
by symlinking `deploy/xr_teleop_robot.sh` and `deploy/xr_teleop_msi.sh` to `~/.local/bin/xr_teleop`.

To run the node by hand:

```bash
# on PC2 (find the NIC on 192.168.123.x with `ip a`)
~/xrcam-env/bin/xrcam-robot --motion --network-interface eth0 --arm-vel-limit 1.5 --operator tcp://<laptop-ip>:5591
# on the laptop
$PY -m xr_camera_teleop.operator.node --robot tcp://192.168.123.164:5592
```
- `--motion`: arms via rt/arm_sdk blended with the robot's own controller (robot in "Regular mode", R1+X, not debug).
  Without `--motion` the node enters debug mode (releases the Unitree controller; the robot will NOT balance:
  only with the robot hung/supported).
- Safety: the arm controller clips arm targets to `--arm-vel-limit` (default 3.0 rad/s; upstream's default was 30). Use
  1.5..3 for first tests. The node starts at the CURRENT posture (upstream drove the arms to q=0 on start). Start with arms
  near home because the clutch maps to the IK home pose and the first command is the neutral pose. Keep `--k` <= 0.8. Use
  the robot's e-stop / remote damping (L2+B) at any time.
- Dry run the same commands first against the sim on domain 0 (loopback): `g1_dds_mujoco.py --domain 0 --iface lo` and
  `--network-interface lo` on the node, without `--sim` (do not do this with the real robot connected).
- Deployment: `bash deploy/push_to_robot.sh` (rsync + setup, see ../README.md). IK timing must be re-measured on PC2:
  `~/xrcam-env/bin/python -m xr_camera_teleop.tools.bench_ik`.

## Self-collision protection, branch escape, reset to home (default ON)

The upstream `G1_29_ArmIK` has no self-collision handling (arm into torso / hand into hand). Every IK user
(`robot/node.py`, `operator/live_ui.py`, `sim/g1_kinematic.py`, `tools/render_session.py`) now gets
`xr_camera_teleop/robot/ik.SafeG1_29_ArmIK` (a plain subclass of the vendored `G1_29_ArmIK`): `robot.ik.make_arm_ik()` returns it
(the robot node and the UI's local preview use it; `--no-self-collision` / `XRCAM_SELF_COLLISION=0` give the plain vendored IK).

1. **IK constraints**: arm links as 25 spheres per arm, body (torso 5 slabs, head, pelvis, hips/thighs) as 15 capsules,
   fitted to the URDF collision meshes (`tools/fit_self_collision.py` -> `robot/g1_collision_spheres.py`, every mesh
   vertex covered); margin 5 mm vs body, 1 cm arm vs arm. 818 pairs (each arm vs body; left vs right arm EXCEPT
   hand/wrist vs hand/wrist: hands may touch / clap / rest wrist on wrist), soft-min aggregated into 73 smooth IPOPT
   inequalities. Fast path: upstream's unconstrained problem first; if its result satisfies the constraints it is the
   answer (~1.1 ms vs 0.8 ms upstream), otherwise the constrained problem (~11 ms median): the arm stops at the body
   (hand ~2 cm from the chest) instead of going through it.
2. **Post-solve gate**: each output is checked on the real meshes (hpp-fcl, ~0.1 ms; hand/wrist vs hand/wrist contact
   allowed). If upstream's 4-tap moving average cuts through the body, the first safe point toward the raw solution
   is used; colliding or non-finite q is never returned (last safe q + tauff, rate-limited stderr warning).
3. **Branch escape** (fixes "arms stay twisted after unusual hand motions"): the warm-started IK (seed = current q,
   `0.1|q-q_last|^2`, joint limits) stays on a joint-limit-pinned / wound branch when wrist-rotation targets leave the
   wrist range (+-113/92 deg) - also upstream (25/40 random excursions stuck). Every 0.25 s (backing off to 2 s while
   nothing better is found) when the tracking cost is not small, a solve seeded from home is compared per arm; if it
   is clearly better (< 0.5x cost and > 0.05 lower) the arm moves there in joint space at 2 rad/s, collision-free
   (straight line, else a collision-constrained projected step), following the operator on the new branch; then
   tracking resumes. 0/40 stuck with it.
4. **Reset to home**: `SafeG1_29_ArmIK.reset_to_home()` drives both arms to q=0 in joint space (2 rad/s, collision-free,
   targets ignored meanwhile), then clears warm start / filter / last safe q / escape state. Hooked to
   `wrapper.reset()` and to the L/R swap latch: on the laptop via `robot.ik.connect_ik(ik, wrapper)` (local preview), on the
   robot through the `home_seq` counter of `ee_cmd` (operator `on_home_request` -> `link.request_home()` -> node
   `reset_to_home`): live UI **Reset** (H), operator key `h` / `xrcam-ctl operator home`, the watchdog timeout and
   `xrcam-ctl robot home` (robot local). Recalibrate afterwards (auto-calibration).

Timing: see `tools/bench_ik` and the README ("IK timing"): free space ~1.7 ms median, in contact (constrained) ~15 ms median /
~50 ms p95 on the (loaded) laptop; an escape check that needs a cold constrained solve costs 20-55 ms (at most every 2 s while
the target is held inside the body). Re-measure on PC2.
Disable (debug only): `xrcam-robot --no-self-collision`, `live_ui --no-self-collision`, or `XRCAM_SELF_COLLISION=0` (`WILOR_XR_SELF_COLLISION=0` still works)
for everything (upstream IK: no escape, no joint-space reset). Tests: `$PY -m pytest -q xr_camera_teleop/tests/test_self_collision.py -s`.
The live UI status line shows `self-collision: ON ...` with constrained / gated / escapes / homing.
Limitations: the IK model has legs/waist locked at 0 and the Dex3 fingers straight; the upper arm may not come closer to
the torso than at q=0 (mesh gap there 8.6 mm); hands are NOT kept apart at all (they can also be pushed into each
other); escape/home transitions are joint-space moves the operator did not command (2 rad/s).

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

### Initial contact and low-speed startup

The first supported-robot test may start with wrists touching the hips. Only after explicit user consent,
`XRCAM_ALLOW_STARTUP_CONTACT=1 bash deploy/robot_session.sh prepare` uses a joint-space home at no more
than 0.3 rad/s for this initial phase. Target acceptance stays OFF. After home, SafeG1 IK checks apply again.
The arm publisher slews from its previous command, with at most 0.05 rad lead over measured position;
limiting every command against measured position instead would send only 0.0012 rad error at 0.3 rad/s
and 250 Hz, preventing target accumulation when the motor initially sticks. Hardware verification of
this fix remains necessary; the offline limiter tests do not demonstrate actual motor tracking.

### Preview speed profile (requires deliberate restart)

`XRCAM_SPEED_PROFILE=preview bash deploy/robot_session.sh settings` prints settings without any DDS
traffic. The profile selects arms 4 rad/s, Dex3 targets 8 rad/s, and a 60 Hz requested robot control loop,
matching the live UI kinematic preview. The UI publisher now defaults to 60 Hz (`--rate 60`).
Arm slew limits each joint independently, with the existing 0.05 rad measured-position lead cap.
Dex3 publishes at 100 Hz and limits targets at 8 rad/s; hand input updates before arm IK.
Gains, robot modes, collision checks and the startup-contact home cap (0.3 rad/s) stay as configured.

To select it on the next explicitly announced real launch:
`XRCAM_SPEED_PROFILE=preview bash deploy/robot_session.sh prepare`. Then complete stage 1,
explicitly enable stage 2, and engage the operator. Writing/deploying these files alone changes no
running process. The Orin may miss 60 Hz during expensive IK solves; a configured rate is not a
verified achieved rate, and physical speed differs from a kinematic preview due to actuator dynamics.


### Independent Dex3 pipeline

The robot uses two spawned hand workers: latest-input DexPilot retargeting and
100 Hz DDS publication. Neither shares the arm IK interpreter. Retargeting
processes each received input version at most once and skips intermediate
frames; it does not queue old gestures. DDS domain/interface are passed
explicitly from the robot configuration (real: domain 0; sim: domain 1).

Publication starts from the latest measured hand posture after both workers
initialize. The preview profile retains the 8 rad/s per-joint hand target limit.
Pause, re-engage epoch changes, expired input/targets (`--stale-timeout`) and a
failed retarget worker hold the last published position. A result computed
before a pause cannot be applied after re-engaging. Shutdown joins both workers.

`xrcam-ctl robot status` includes `hands`: measured publication/retarget rates,
latest solve duration, target age, worker health and worker errors. The UI status
line displays publication and retarget rates after the UI and robot are restarted.
Copying these files does not activate them in a running session. Restarting the
robot uses the existing two-stage prepare/home and then enable/follow flow;
it can move the robot and must be announced to the operator before execution.
