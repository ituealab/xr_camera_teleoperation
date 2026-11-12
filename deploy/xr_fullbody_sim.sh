#!/usr/bin/env bash
# Full-body teleop in simulation (PLAN §3, §4): one tmux session per process, logs in ~/.local/state/xr_fullbody.
# P1 brings up the first two processes; later phases add camera_bridge, quest_server and fullbody_node here.
#   sim    MuJoCo sim (upstream gear_sonic physics, our scene + real-time loop, viewer and robot cameras)
#   sonic  prebuilt SONIC deploy binary (zmq_manager input on :5556, telemetry on :5557)
#
#   deploy/xr_fullbody_sim.sh start [--no-viewer] [--no-cameras]  # start both (the robot hangs from the sim's elastic band)
#   deploy/xr_fullbody_sim.sh stop | status | logs
#   deploy/xr_fullbody_sim.sh key 9                  # sim key: 9 = drop/re-hang band, backspace = reset robot
#   deploy/xr_fullbody_sim.sh attach sim|sonic       # the process's terminal (sonic: `o` = emergency stop)
set -euo pipefail
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
LOG_DIR="$HOME/.local/state/xr_fullbody"
SIM_PY="$REPO/data/sonic-sim-venv/bin/python"
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

[[ -x "$SIM_PY" ]] || { printf 'Missing %s: run deploy/setup_sonic_sim_env.sh\n' "$SIM_PY" >&2; exit 1; }
case "${1:-start}" in
  start)
    shift || true
    sim_args=()
    for arg in "$@"; do
      case "$arg" in
        --no-viewer) sim_args+=(--no-viewer) ;;
        --no-cameras) sim_args+=(--no-cameras) ;;
        *) printf 'Unknown option: %s\n' "$arg" >&2; exit 2 ;;
      esac
    done
    mkdir -p "$LOG_DIR"
    "$SIM_PY" -m xr_camera_teleop.sonic.assets verify
    if ! running sim; then
      launch sim "$SIM_PY" -u -m xr_camera_teleop.sim.run_sonic_sim "${sim_args[@]}"
      wait_log sim '\[sim\] running' 30
    fi
    if ! running sonic; then
      launch sonic "$SIM_PY" -u -m xr_camera_teleop.sonic.run_deploy
      wait_log sonic 'Init Done' 120
    fi
    printf 'Sim and SONIC are up (robot on the elastic band, SONIC waiting for command start).\n'
    printf 'Drive it: %s -m xr_camera_teleop.tools.sonic_drive start\nLogs: %s\n' "$SIM_PY" "$LOG_DIR"
    ;;
  stop)
    if running sonic; then tmux send-keys -t "=$PREFIX-sonic:" o; sleep 2; fi   # SONIC's own emergency stop
    stop_one sonic
    stop_one sim
    printf 'Stopped.\n'
    ;;
  status)
    for name in sim sonic; do
      if running "$name"; then printf '%s: running\n' "$name"; else printf '%s: stopped\n' "$name"; fi
    done
    ;;
  key)
    [[ -n "${2:-}" ]] || { printf 'Usage: %s key 9|backspace|up|down|left|right\n' "$0" >&2; exit 2; }
    running sim || { printf 'sim is not running\n' >&2; exit 1; }
    tmux send-keys -t "=$PREFIX-sim:" "$2" Enter
    ;;
  attach)
    tmux attach -t "=$PREFIX-${2:?sim or sonic}"
    ;;
  logs)
    tail -n 40 -F "$LOG_DIR/sim.log" "$LOG_DIR/sonic.log"
    ;;
  *) printf 'Usage: %s start [--no-viewer] [--no-cameras] | stop | status | key K | attach sim|sonic | logs\n' "$0" >&2; exit 2 ;;
esac
