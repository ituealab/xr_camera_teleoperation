#!/usr/bin/env bash
# Full-body teleop in simulation (PLAN §3, §4): one tmux session per process, logs in ~/.local/state/xr_fullbody.
#   sim    MuJoCo sim (upstream gear_sonic physics, our scene + real-time loop, viewer and robot cameras)
#   sonic  prebuilt SONIC deploy binary (zmq_manager input on :5556, telemetry on :5557), relaunched whenever it
#          exits (SONIC's `stop` ends the process); the sim's gantry drops / re-hangs the robot with SONIC's control
#   bridge sim/camera_bridge.py: the sim's robot camera (:5555) -> the Quest server's stereo input (:5605)
#   quest  Quest server (data/quest-env): headset at https://<MSI>/, xr_frame :5596, robot view, control panel
#          (panel actions :5597, node status :5595; docs/quest-sonic/P5_ui.md)
#   node   fullbody_node (data/xr-env), started with the arguments after `--`; its session ends when it exits
#
#   deploy/xr_fullbody_sim.sh start [--no-viewer] [--no-cameras] [--no-quest] [-- NODE_ARGS...]
#       the headset session of the acceptance test: start -- --source quest
#       e.g. start -- --source script --scenario all                        (scripted operator)
#            start -- --source quest --xr-replay data/fullbody-runs/<time>  (a recorded headset session)
#            start -- --source quest                                        (the live Quest server, :5596)
#   deploy/xr_fullbody_sim.sh node NODE_ARGS...      # (re)start only the node
#   deploy/xr_fullbody_sim.sh stop | status | logs
#   deploy/xr_fullbody_sim.sh key 9                  # sim key: 9 = drop/re-hang band, backspace = reset robot
#   deploy/xr_fullbody_sim.sh attach sim|sonic|bridge|quest|node   # the process's terminal (sonic: `o` = emergency
#                                                                  # stop; relaunched)
set -euo pipefail
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
LOG_DIR="$HOME/.local/state/xr_fullbody"
SIM_PY="$REPO/data/sonic-sim-venv/bin/python"
NODE_PY="$REPO/data/xr-env/bin/python"
QUEST_PY="$REPO/data/quest-env/bin/python"
QUEST_HEALTH=http://127.0.0.1:8771/health
PREFIX=xr-fullbody

running() { tmux has-session -t "=$PREFIX-$1" 2>/dev/null; }
launch() {
  local name=$1; shift
  local cmd
  printf -v cmd '%q ' "$@"
  # tee keeps the process attached to the tmux terminal (stdin keys) while logging everything
  printf -v cmd '%s 2>&1 | tee -i %q' "$cmd" "$LOG_DIR/$name.log"
  tmux new-session -d -s "$PREFIX-$name" -c "$REPO" -x 200 -y 50 "bash -c $(printf '%q' "set -o pipefail; $cmd")"
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

start_node() {
  stop_one node
  launch node "$NODE_PY" -u -m xr_camera_teleop.fullbody.node "$@"
  printf 'Node started (%s). Watch: %s attach node | log %s/node.log\n' "$*" "$0" "$LOG_DIR"
}

[[ -x "$SIM_PY" ]] || { printf 'Missing %s: run deploy/setup_sonic_sim_env.sh\n' "$SIM_PY" >&2; exit 1; }
case "${1:-start}" in
  start)
    shift || true
    sim_args=()
    cameras=1
    quest=1
    while (($#)); do
      case "$1" in
        --no-viewer) sim_args+=(--no-viewer) ;;
        --no-cameras) sim_args+=(--no-cameras); cameras=0 ;;
        --no-quest) quest=0 ;;
        --) shift; break ;;
        *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
      esac
      shift
    done
    mkdir -p "$LOG_DIR"
    "$SIM_PY" -m xr_camera_teleop.sonic.assets verify
    if ! running sim; then
      launch sim "$SIM_PY" -u -m xr_camera_teleop.sim.run_sonic_sim "${sim_args[@]}"
      wait_log sim '\[sim\] running' 30
      grep -F 'GPU WARNING' "$LOG_DIR/sim.log" >&2 || true     # the sim's GPU preflight (sim/gpu.py), loudly
    fi
    if ! running sonic; then
      launch sonic "$SIM_PY" -u -m xr_camera_teleop.sonic.run_deploy --respawn
      wait_log sonic 'Init Done' 120
    fi
    printf 'Sim and SONIC are up (robot on the elastic band, SONIC waiting for command start). Logs: %s\n' "$LOG_DIR"
    if ((cameras)) && ! running bridge; then
      launch bridge "$SIM_PY" -u -m xr_camera_teleop.sim.camera_bridge
    fi
    if ((quest)) && ! running quest; then
      if curl -fsS --max-time 2 "$QUEST_HEALTH" >/dev/null 2>&1; then
        printf 'Another Quest server already answers on %s: not starting ours (stop it to get the panel).\n' \
          "$QUEST_HEALTH" >&2
      else
        cam=()
        ((cameras)) && cam=(--camera tcp://127.0.0.1:5605 --view ego)
        launch quest "$QUEST_PY" -u -m xr_camera_teleop.operator.quest.server "${cam[@]}"
        wait_log quest 'Quest browser' 60
        printf 'Quest server up: open https://192.168.1.105/ on the headset, Enter VR.\n'
      fi
    fi
    if (($#)); then
      start_node "$@"
    else
      printf 'Drive it: %s start -- --source script --scenario square   (or: %s node ...)\n' "$0" "$0"
    fi
    ;;
  node)
    shift
    (($#)) || { printf 'Usage: %s node NODE_ARGS... (see fullbody/node.py --help)\n' "$0" >&2; exit 2; }
    running sonic || { printf 'sim/SONIC are not running: %s start first\n' "$0" >&2; exit 1; }
    start_node "$@"
    ;;
  stop)
    stop_one node       # SIGINT: the node disengages and leaves SONIC standing
    stop_one quest
    stop_one bridge
    stop_one sonic      # SIGINT ends the respawn loop and the binary
    stop_one sim
    printf 'Stopped.\n'
    ;;
  status)
    for name in sim sonic bridge quest node; do
      if running "$name"; then printf '%s: running\n' "$name"; else printf '%s: stopped\n' "$name"; fi
    done
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
  *) printf 'Usage: %s start [--no-viewer] [--no-cameras] [--no-quest] [-- NODE_ARGS] | node NODE_ARGS | stop | status | key K | attach NAME | logs\n' "$0" >&2; exit 2 ;;
esac
