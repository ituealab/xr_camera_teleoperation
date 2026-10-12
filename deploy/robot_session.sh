#!/usr/bin/env bash
# Run on PC2. Two-stage lowcmd session for a supported robot; never selects motion mode implicitly.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_DIR="${XRCAM_ENV:-$HOME/xrcam-env}"
PY="$ENV_DIR/bin/python"
LOG_DIR="${XRCAM_SESSION_DIR:-$REPO/data/session-20261009}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
cd "$REPO"
case "${XRCAM_SPEED_PROFILE:-slow}" in
  slow) ;;
  preview) source "$REPO/deploy/preview-speed.env" ;;
  *) echo "unknown speed profile; choose slow or preview" >&2; exit 2 ;;
esac
case "${1:-status}" in
  prepare)
    shift
    mkdir -p "$LOG_DIR"
    if "$PY" -m xr_camera_teleop.ctl robot status | "$PY" -c 'import json,sys; sys.exit(0 if json.load(sys.stdin).get("ok") else 1)'; then
      echo "A robot node is already running; inspect status or stop it before prepare" >&2
      exit 1
    fi
    CONTROL=(--debug --network-interface eth0)
    if [[ "${1:-}" == --dry-run ]]; then CONTROL=(--dry-run); shift; fi
    CONTACT=()
    [[ "${XRCAM_ALLOW_STARTUP_CONTACT:-0}" == 1 ]] && CONTACT=(--allow-startup-contact)
    nohup "$PY" -u -m xr_camera_teleop.robot.node "${CONTROL[@]}" "${CONTACT[@]}" --prepare \
      --operator tcp://192.168.123.100:5591 --status-bind tcp://*:5592 \
      --arm-vel-limit "${XRCAM_ARM_VEL_LIMIT:-0.3}" --hand-vel-limit "${XRCAM_HAND_VEL_LIMIT:-8}" --frequency "${XRCAM_IK_HZ:-5}" "$@" \
      > "$LOG_DIR/robot.log" 2>&1 < /dev/null &
    echo "$!" > "$LOG_DIR/robot.pid"
    echo "Prepare process started: $(cat "$LOG_DIR/robot.pid"); inspect $LOG_DIR/robot.log and status"
    ;;
  settings)
    printf "profile=%s arm_rad_s=%s hand_rad_s=%s control_hz=%s operator_hz=%s\n" "${XRCAM_SPEED_PROFILE:-slow}" "${XRCAM_ARM_VEL_LIMIT:-0.3}" "${XRCAM_HAND_VEL_LIMIT:-8}" "${XRCAM_IK_HZ:-5}" "${XRCAM_OPERATOR_HZ:-30}"
    ;;
  enable|hold|home|stop|status)
    "$PY" -m xr_camera_teleop.ctl robot "$1"
    ;;
  *) echo "usage: $0 prepare [--dry-run] | enable | hold | home | stop | status" >&2; exit 2 ;;
esac
