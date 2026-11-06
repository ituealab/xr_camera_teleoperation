#!/usr/bin/env bash
# Robot-side launcher, installed as ~/.local/bin/xr_teleop. ee_runtime owns the motors.
set -euo pipefail
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
SERVICES=(python3 "$REPO/deploy/recording_services.py")
RUNTIME="${ROBOT_RUNTIME:-$HOME/robot_runtime}"
STATE="$HOME/.local/state/xr_teleop"
LOG="$STATE/ee_runtime.log"
PID="$STATE/ee_runtime.pid"
IK_BACKEND=sqp
FRAX_PYTHON="${FRAX_PYTHON:-$HOME/robot_runtime_ws/frax-env/bin/python}"

ros_env() {
  # G1_ROS_ENV_SOURCED too: an interactive shell exports it, and env.sh would then skip the setup wiped here.
  unset PYTHONPATH PYTHONHOME ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH G1_ROS_ENV_SOURCED
  set +u
  source "$RUNTIME/scripts/runtime-env.sh"
  set -u
}

running() { [[ -f "$PID" ]] && kill -0 "$(cat "$PID")" 2>/dev/null; }

confirm_supported() {
  "$HOME/.local/bin/developer" status | grep -q 'motion servisi bırakılmış' && return
  [[ "${1:-}" == --yes ]] && return
  printf 'Denge kontrolü kapanacak. Robot askıda / destekli mi? [e/H] '
  local answer=""
  read -r answer || true
  [[ "$answer" == [eEyY] ]] || { printf 'İptal edildi; hiçbir şey hareket etmedi.\n' >&2; exit 1; }
}

start_runtime() {
  running && return
  mkdir -p "$STATE"
  local backend=(ik_backend:=sqp)
  if [[ "$IK_BACKEND" == frax ]]; then
    backend=(ik_backend:=frax frax_python:="$FRAX_PYTHON")
  fi
  printf 'IK: %s\n' "$IK_BACKEND"
  # Own session/process group: stop must reach the launcher's node and solver children too.
  setsid nohup ros2 launch ee_runtime ee_teleop.launch.py enabled:=true allow_startup_contact:=true \
    solver_python:="$HOME/robot_runtime_ws/ik-env/bin/python" \
    kinematics_source:="$RUNTIME/packages/g1-kinematics/src" "${backend[@]}" > "$LOG" 2>&1 < /dev/null &
  echo "$!" > "$PID"
}

# Anchored to the python executable so shells whose command line mentions these names do not match.
motor_writers() {
  pgrep -f '^[^ ]*python[^ ]* ([^ ]*/ee_runtime_node( |$)|-u -m ee_runtime[.]solver_worker|-u [^ ]*/frax_server[.]py)' || true
}

stop_runtime() {
  if running; then
    local pgid
    pgid="$(ps -o pgid= -p "$(cat "$PID")" | tr -d ' ')"
    kill -TERM -- "-$pgid" 2>/dev/null || true
    for ((i=0; i<10; i++)); do
      kill -0 -- "-$pgid" 2>/dev/null || break
      sleep 1
    done
    kill -KILL -- "-$pgid" 2>/dev/null || true
  fi
  rm -f "$PID"
  local left
  left="$(motor_writers)"
  [[ -z "$left" ]] || { printf 'UYARI: ee_runtime süreçleri hâlâ çalışıyor: %s\n' "$left" >&2; return 1; }
}

case "${1:-help}" in
  up)
    shift
    yes=""
    quest=""
    for arg in "$@"; do
      case "$arg" in
        --yes) yes=--yes ;;
        --quest) quest=1 ;;
        --frax) IK_BACKEND=frax ;;
        --sqp) IK_BACKEND=sqp ;;
        *) printf 'Bilinmeyen seçenek: %s\n' "$arg" >&2; exit 2 ;;
      esac
    done
    if [[ "$IK_BACKEND" == frax && ! -x "$FRAX_PYTHON" ]]; then
      printf 'frax ortamı yok: %s (robot_runtime/scripts/setup-frax-env.sh)\n' "$FRAX_PYTHON" >&2; exit 1
    fi
    # Cameras first: a camera fault should stop here, before balance is released.
    printf 'Kameralar başlatılıyor...\n'
    "${SERVICES[@]}" start all > /dev/null
    # ZED left/right pairs for the Quest headset view (MSI: xr_teleop start --quest)
    [[ -z "$quest" ]] || "${SERVICES[@]}" start stereo > /dev/null
    confirm_supported "$yes"
    "$HOME/.local/bin/developer" start
    ros_env
    printf 'ee_runtime başlatılıyor, kollar home konumuna gidecek...\n'
    start_runtime
    sleep 3
    running || { printf 'ee_runtime başlamadı. Log: %s\n' "$LOG" >&2; exit 1; }
    python3 "$REPO/deploy/ee_home.py" || { printf 'Hazır olmadı. Log: %s\n' "$LOG" >&2; exit 1; }
    printf 'Hazır. MSI arayüzünde R ile başlat.\n'
    ;;
  home)
    ros_env
    python3 "$REPO/deploy/ee_home.py"
    ;;
  down)
    if running; then
      ros_env
      printf 'Kollar home konumuna gidiyor...\n'
      python3 "$REPO/deploy/ee_home.py" --timeout 30 || printf 'Home tamamlanmadı; ee_runtime yine de durduruluyor.\n' >&2
    fi
    stop_runtime
    "${SERVICES[@]}" stop all > /dev/null
    printf 'ee_runtime ve kameralar durduruldu.\n'
    ;;
  status)
    if running; then printf 'ee_runtime: çalışıyor\n'; else printf 'ee_runtime: kapalı\n'; fi
    "${SERVICES[@]}" status
    ;;
  logs)
    exec tail -n 40 -F "$LOG"
    ;;
  help|-h|--help)
    printf 'Kullanım: xr_teleop up [--yes] [--frax|--sqp] [--quest]|down|home|status|logs\n'
    printf 'up: kameralar, developer modu, ee_runtime, kollar home. Sonra MSI arayüzünde R.\n'
    printf '    --frax: frax IK (daha hassas takip), varsayılan --sqp.\n'
    printf '    --quest: Quest gözlüğü için ZED stereo yayınını da başlatır.\n'
    printf 'down: kollar home, ee_runtime ve kameralar durur.\n'
    ;;
  *) printf 'Bilinmeyen komut: %s\n' "$1" >&2; exit 2 ;;
esac
