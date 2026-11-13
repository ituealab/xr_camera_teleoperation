#!/usr/bin/env bash
# Full-body teleop in simulation (PLAN §3, §4): one tmux session per process, logs in ~/.local/state/xr_fullbody.
#   sim    MuJoCo sim (upstream gear_sonic physics, our scene + real-time loop, viewer and robot cameras)
#   sonic  prebuilt SONIC deploy binary (zmq_manager input on :5556, telemetry on :5557), relaunched whenever it
#          exits (SONIC's `stop` ends the process); the sim's gantry lowers / re-hangs the robot with SONIC's control
#   bridge sim/camera_bridge.py: the sim's robot camera (:5555) -> the Quest server's stereo input (:5605)
#   quest  Quest server (data/quest-env): headset at https://<MSI>/, xr_frame :5596, robot view, control panel
#          (panel actions :5597, node status :5595; docs/quest-sonic/P5_ui.md)
#   node   fullbody_node (data/xr-env), started with the arguments after `--`; its session ends when it exits
#
#   deploy/xr_fullbody_sim.sh morning [--no-viewer]   # the headset test (docs/quest-sonic/SABAH.md): everything
#                                                     # on the morning course, the viewer on the desk screen;
#                                                     # idempotent (starts what is not running)
#   deploy/xr_fullbody_sim.sh start [--course NAME] [--no-viewer] [--no-cameras] [--no-quest] [-- NODE_ARGS...]
#       e.g. start -- --source quest                                        (the live Quest server, :5596)
#            start -- --source script --scenario all                        (scripted operator)
#            start -- --source quest --xr-replay data/fullbody-runs/<time>  (a recorded headset session)
#   deploy/xr_fullbody_sim.sh node NODE_ARGS...      # (re)start only the node
#   deploy/xr_fullbody_sim.sh stop [NAME] | status | logs   # stop everything, or one process
#   deploy/xr_fullbody_sim.sh key 9                  # sim key: 9 = band on/off, backspace = reset robot
#   deploy/xr_fullbody_sim.sh attach sim|sonic|bridge|quest|node   # the process's terminal (sonic: `o` = emergency
#                                                                  # stop; relaunched)
#
# The viewer needs a display. Without DISPLAY / WAYLAND_DISPLAY in this shell (e.g. started over ssh as another
# user than the one at the desk), the sim goes to the desk's X display (the seat's Xwayland): directly if that user
# allowed it once (`xhost +SI:localuser:$USER` in their session), otherwise with their X cookie when this user has
# passwordless sudo. Failing both, the sim runs without the viewer and says so.
set -euo pipefail
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
LOG_DIR="$HOME/.local/state/xr_fullbody"
SIM_PY="$REPO/data/sonic-sim-venv/bin/python"
NODE_PY="$REPO/data/xr-env/bin/python"
QUEST_PY="$REPO/data/quest-env/bin/python"
QUEST_HEALTH=http://127.0.0.1:8771/health
LIVE_FILE="$REPO/data/sim/live.json"           # the running sim's course etc. (sim/live.py)
QUEST_URL=https://192.168.1.105                # the MSI on aksoy-lab Wi-Fi (PLAN §8a); :443 -> :8012
PREFIX=xr-fullbody
NAMES=(sim sonic bridge quest node)

running() { tmux has-session -t "=$PREFIX-$1" 2>/dev/null; }
# The tmux server outlives this script. When this script is the one that starts it, it goes into a systemd user
# scope of its own, out of the caller's cgroup: closing the caller's terminal or session (an SSH login, an agent's
# shell) then cannot take the whole stack down with it.
tmux_new_session() {
  if ! tmux list-sessions >/dev/null 2>&1 && command -v systemd-run >/dev/null \
      && systemctl --user show-environment >/dev/null 2>&1; then
    systemd-run --user --scope --quiet --collect --unit="$PREFIX-tmux-$(date +%s)" tmux new-session "$@"
  else
    tmux new-session "$@"
  fi
}
launch() {
  local name=$1; shift
  local cmd
  printf -v cmd '%q ' "$@"
  # tee keeps the process attached to the tmux terminal (stdin keys) while logging everything
  printf -v cmd '%s 2>&1 | tee -i %q' "$cmd" "$LOG_DIR/$name.log"
  tmux_new_session -d -s "$PREFIX-$name" -c "$REPO" -x 200 -y 50 "bash -c $(printf '%q' "set -o pipefail; $cmd")"
}
stop_one() {
  local name=$1 pid
  running "$name" || return 0
  pid=$(tmux display-message -p -t "=$PREFIX-$name:" '#{pane_pid}')
  pkill -INT -P "$pid" 2>/dev/null || true
  for ((i=0; i<10; i++)); do running "$name" || return 0; sleep 1; done
  tmux kill-session -t "=$PREFIX-$name"
}
wait_log() {   # wait_log name pattern seconds
  for ((i=0; i<$3; i++)); do
    grep -q "$2" "$LOG_DIR/$1.log" 2>/dev/null && return 0
    running "$1" || { printf '%s exited. Log: %s/%s.log\n' "$1" "$LOG_DIR" "$1" >&2; return 1; }
    sleep 1
  done
  printf '%s not ready after %s s. Log: %s/%s.log\n' "$1" "$3" "$LOG_DIR" "$1" >&2
  return 1
}
sim_course() {   # the running sim's course, if it announced one
  [[ -f "$LIVE_FILE" ]] && sed -n 's/.*"course": "\([^"]*\)".*/\1/p' "$LIVE_FILE" || true
}

# Sets VIEW_ENV (env assignments for the sim) and VIEW_HOW; returns 1 when there is no usable display.
VIEW_ENV=()
VIEW_HOW=""
find_display() {
  if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    VIEW_HOW="this session's display (${DISPLAY:-$WAYLAND_DISPLAY})"
    return 0
  fi
  local line disp auth
  # the desk's Xwayland (GNOME on Wayland starts one per seat session): its display and its cookie file
  line=$(ps -eo args= | grep -m1 -E '^/usr/bin/Xwayland :[0-9]+' || true)
  [[ -n "$line" ]] || return 1
  disp=$(grep -oE ' :[0-9]+' <<<"$line" | head -1 | tr -d ' ')
  auth=$(sed -n 's/.* -auth \([^ ]*\).*/\1/p' <<<"$line")
  if DISPLAY=$disp timeout 3 xdpyinfo >/dev/null 2>&1; then
    VIEW_ENV=(DISPLAY="$disp")
    VIEW_HOW="the desk display $disp (allowed with xhost)"
    return 0
  fi
  if [[ -n "$auth" ]] && sudo -n test -r "$auth" 2>/dev/null; then
    local mine="$LOG_DIR/desk.xauth"
    (umask 077 && sudo -n cat "$auth" >"$mine")
    if DISPLAY=$disp XAUTHORITY=$mine timeout 3 xdpyinfo >/dev/null 2>&1; then
      VIEW_ENV=(DISPLAY="$disp" XAUTHORITY="$mine")
      VIEW_HOW="the desk display $disp (with the desk user's X cookie)"
      return 0
    fi
  fi
  return 1
}

start_node() {
  stop_one node
  launch node "$NODE_PY" -u -m xr_camera_teleop.fullbody.node "$@"
  printf 'Node started (%s). Watch: %s attach node | log %s/node.log\n' "$*" "$0" "$LOG_DIR"
}

# start_stack COURSE VIEWER(0|1) CAMERAS(0|1) QUEST(0|1): sim, SONIC, bridge, Quest server (each only if not running)
start_stack() {
  local course=$1 viewer=$2 cameras=$3 quest=$4
  local sim_args=(--course "$course")
  ((viewer)) || sim_args+=(--no-viewer)
  ((cameras)) || sim_args+=(--no-cameras)
  mkdir -p "$LOG_DIR"
  "$SIM_PY" -m xr_camera_teleop.sonic.assets verify >/dev/null || "$SIM_PY" -m xr_camera_teleop.sonic.assets verify
  if running sim && [[ "$(sim_course)" != "$course" ]]; then
    printf 'The sim runs course %s, not %s: stop it first (%s stop).\n' "$(sim_course)" "$course" "$0" >&2
    return 1
  fi
  if ! running sim; then
    local env_cmd=()
    if ((viewer)); then
      if find_display; then
        env_cmd=(env "${VIEW_ENV[@]}")
        printf 'Viewer: %s\n' "$VIEW_HOW"
      else
        sim_args+=(--no-viewer)
        printf 'No display for the viewer: running without it. At the desk, run once:  xhost +SI:localuser:%s\n' \
          "$USER" >&2
      fi
    fi
    launch sim "${env_cmd[@]}" "$SIM_PY" -u -m xr_camera_teleop.sim.run_sonic_sim "${sim_args[@]}"
    wait_log sim '\[sim\] running' 30
    grep -F 'GPU WARNING' "$LOG_DIR/sim.log" >&2 || true     # the sim's GPU preflight (sim/gpu.py), loudly
  fi
  if ! running sonic; then
    launch sonic "$SIM_PY" -u -m xr_camera_teleop.sonic.run_deploy --respawn
    wait_log sonic 'Init Done' 120
  fi
  printf 'Sim (course %s) and SONIC are up; the robot hangs on the gantry until SONIC starts. Logs: %s\n' \
    "$course" "$LOG_DIR"
  if ((cameras)) && ! running bridge; then
    launch bridge "$SIM_PY" -u -m xr_camera_teleop.sim.camera_bridge
  fi
  if ((quest)) && ! running quest; then
    if curl -fsS --max-time 2 "$QUEST_HEALTH" >/dev/null 2>&1; then
      printf 'Another Quest server already answers on %s: not starting ours (stop it to get the panel).\n' \
        "$QUEST_HEALTH" >&2
    else
      local cam=()
      ((cameras)) && cam=(--camera tcp://127.0.0.1:5605 --view ego)
      launch quest "$QUEST_PY" -u -m xr_camera_teleop.operator.quest.server "${cam[@]}"
      wait_log quest 'Quest browser' 60
    fi
  fi
}

[[ -x "$SIM_PY" ]] || { printf 'Missing %s: run deploy/setup_sonic_sim_env.sh\n' "$SIM_PY" >&2; exit 1; }
case "${1:-}" in
  morning)
    viewer=1
    [[ "${2:-}" == --no-viewer ]] && viewer=0
    if running sim && [[ "$(sim_course)" != morning ]]; then
      printf 'Restarting: the running stack is on course %s.\n' "$(sim_course)"
      "$0" stop
    fi
    start_stack morning "$viewer" 1 1
    running node || start_node --source quest
    sleep 2
    for name in "${NAMES[@]}"; do
      running "$name" || { printf 'FAILED: %s is not running. Log: %s/%s.log\n' "$name" "$LOG_DIR" "$name" >&2; exit 1; }
    done
    if ! ((viewer)); then view="off (--no-viewer)"
    elif grep -q 'running without the viewer\|no DISPLAY' "$LOG_DIR/sim.log"; then view="NOT SHOWN (see above)"
    else view="on the desk screen"; fi
    cat <<EOF

  Ready (sim viewer: $view).
  1. Quest: open $QUEST_URL and press Enter VR. You see the robot's camera.
  2. Press a thumbstick for the panel: Başlat, then stand straight and look ahead and press Kalibre et ve devral
     (or hold A for 1 s). Walk with the stick; put the controllers down to use your hands.
  3. End: DURDUR on the panel (or both grips + trigger). Shut everything down: $0 stop

EOF
    ;;
  start)
    shift
    course=$(sed -n 's/^course: *\([^ #]*\).*/\1/p' "$REPO/configs/sonic_sim.yaml")
    viewer=1
    cameras=1
    quest=1
    while (($#)); do
      case "$1" in
        --course) course=$2; shift ;;
        --no-viewer) viewer=0 ;;
        --no-cameras) cameras=0 ;;
        --no-quest) quest=0 ;;
        --) shift; break ;;
        *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
      esac
      shift
    done
    start_stack "$course" "$viewer" "$cameras" "$quest"
    if ((quest)) && running quest; then
      printf 'Quest server up: open %s on the headset, Enter VR.\n' "$QUEST_URL"
    fi
    if (($#)); then
      start_node "$@"
    else
      printf 'Drive it: %s start -- --source quest   (or: %s node ...)\n' "$0" "$0"
    fi
    ;;
  node)
    shift
    (($#)) || { printf 'Usage: %s node NODE_ARGS... (see fullbody/node.py --help)\n' "$0" >&2; exit 2; }
    running sonic || { printf 'sim/SONIC are not running: %s start first\n' "$0" >&2; exit 1; }
    start_node "$@"
    ;;
  stop)
    if [[ -n "${2:-}" ]]; then
      [[ " ${NAMES[*]} " == *" $2 "* ]] || { printf 'Unknown process %s (%s)\n' "$2" "${NAMES[*]}" >&2; exit 2; }
      stop_one "$2"
      printf 'Stopped %s.\n' "$2"
      exit 0
    fi
    stop_one node       # SIGINT: the node disengages and leaves SONIC standing
    stop_one quest
    stop_one bridge
    stop_one sonic      # SIGINT ends the respawn loop and the binary
    stop_one sim        # with its viewer and cameras
    rm -f "$LOG_DIR/desk.xauth"
    printf 'Stopped.\n'
    ;;
  status)
    for name in "${NAMES[@]}"; do
      if running "$name"; then printf '%s: running\n' "$name"; else printf '%s: stopped\n' "$name"; fi
    done
    running sim && printf 'course: %s\n' "$(sim_course)"
    ;;
  key)
    [[ -n "${2:-}" ]] || { printf 'Usage: %s key 9|backspace|up|down|left|right\n' "$0" >&2; exit 2; }
    running sim || { printf 'sim is not running\n' >&2; exit 1; }
    tmux send-keys -t "=$PREFIX-sim:" "$2" Enter
    ;;
  attach)
    tmux attach -t "=$PREFIX-${2:?sim, sonic, bridge, quest or node}"
    ;;
  logs)
    tail -n 40 -F "$LOG_DIR/sim.log" "$LOG_DIR/sonic.log" "$LOG_DIR/bridge.log" "$LOG_DIR/quest.log" "$LOG_DIR/node.log"
    ;;
  *) printf 'Usage: %s morning [--no-viewer] | start [--course NAME] [--no-viewer] [--no-cameras] [--no-quest] [-- NODE_ARGS] | node NODE_ARGS | stop [NAME] | status | key K | attach NAME | logs\n' "$0" >&2; exit 2 ;;
esac
