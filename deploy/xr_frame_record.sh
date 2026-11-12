#!/usr/bin/env bash
# P3 exit recording (docs/quest-sonic/PLAN.md §9): a validated xr_frame log from the real headset.
#
#   deploy/xr_frame_record.sh [--minutes 5] [--no-view]   # serve if needed, wait for Enter VR, record, validate
#   deploy/xr_frame_record.sh serve [--no-view]           # only make sure the Quest server is up (tmux)
#   deploy/xr_frame_record.sh stop                        # stop the server and the test view started here
#
# The Quest server runs in tmux session xr-p3-quest (log: ~/.local/state/xr_teleop/xr-p3-quest.log) with the
# synthetic stereo view (tools/stereo_test_source.py, 960 px per eye, session xr-p3-stereo), so the headset renders
# the load the robot view will put on it. A Quest server that is already answering on :8771 is used as it is.
# The run lands in data/fullbody-runs/<time>/ (run.msgpack, meta.json, validation.json). The operator script is in
# xr_camera_teleop/RUN.md ("xr_frame recording").
set -euo pipefail
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
LOG_DIR="$HOME/.local/state/xr_teleop"
QUEST_PY="$REPO/data/quest-env/bin/python"
XR_PY="$REPO/data/xr-env/bin/python"
HEALTH=http://127.0.0.1:8771/health

running() { tmux has-session -t "=$1" 2>/dev/null; }
http_ok() { curl -fsS --max-time 2 "$1" >/dev/null 2>&1; }
launch() {
  local name=$1; shift
  local cmd
  printf -v cmd '%q ' "$@"
  printf -v cmd '%s > %q 2>&1' "exec $cmd" "$LOG_DIR/$name.log"
  tmux new-session -d -s "$name" -c "$REPO" "bash -c $(printf '%q' "$cmd")"
}
stop_one() {
  local name=$1 pid
  running "$name" || return 0
  pid=$(tmux display-message -p -t "=$name:" '#{pane_pid}')
  kill -INT "$pid" 2>/dev/null || true
  for ((i = 0; i < 10; i++)); do running "$name" || return 0; sleep 1; done
  tmux kill-session -t "=$name"
}

serve() {
  local view=$1
  mkdir -p "$LOG_DIR"
  if http_ok "$HEALTH"; then
    printf 'Quest server already running (%s).\n' "$HEALTH"
    return 0
  fi
  local cam=()
  if [[ "$view" == ego ]]; then
    running xr-p3-stereo || launch xr-p3-stereo "$QUEST_PY" -u -m xr_camera_teleop.tools.stereo_test_source \
      --width 960 --height 540
    cam=(--camera tcp://127.0.0.1:5605 --view ego)
  fi
  launch xr-p3-quest "$QUEST_PY" -u -m xr_camera_teleop.operator.quest.server "${cam[@]}"
  for ((i = 0; i < 60; i++)); do
    http_ok "$HEALTH" && { printf 'Quest server up: https://192.168.1.105/ on the headset, then Enter VR.\n'; return 0; }
    running xr-p3-quest || break
    sleep 1
  done
  printf 'Quest server did not start; log: %s/xr-p3-quest.log\n' "$LOG_DIR" >&2
  return 1
}

cmd=record
case "${1:-}" in serve|stop|record) cmd=$1; shift ;; esac
minutes=5
view=ego
while (($#)); do
  case "$1" in
    --minutes) minutes=$2; shift 2 ;;
    --no-view) view=none; shift ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
  esac
done

case "$cmd" in
  serve) serve "$view" ;;
  stop)
    stop_one xr-p3-quest
    stop_one xr-p3-stereo
    printf 'Stopped.\n' ;;
  record)
    serve "$view"
    seconds=$(awk -v m="$minutes" 'BEGIN { printf "%d", m * 60 }')
    cd "$REPO"
    exec "$XR_PY" -m xr_camera_teleop.tools.xr_frame_record --seconds "$seconds" ;;
esac
