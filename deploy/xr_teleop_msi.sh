#!/usr/bin/env bash
# MSI-side launcher, installed as ~/.local/bin/xr_teleop.
set -euo pipefail
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
LOG_DIR="$HOME/.local/state/xr_teleop"
export WILOR_ROOT="${WILOR_ROOT:-$HOME/code/wilor_faruk/third_party/WiLoR}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
URL=http://127.0.0.1:8780/

running() { tmux has-session -t "=xr-teleop-$1" 2>/dev/null; }
http_ok() { curl -fsS --max-time 2 "$1" >/dev/null 2>&1; }
wait_http() {
  local name=$1 url=$2
  for ((i=0; i<120; i++)); do
    if http_ok "$url"; then return 0; fi
    if ! running "$name"; then
      printf '%s başlatılamadı. Log: %s/%s.log\n' "$name" "$LOG_DIR" "$name" >&2
      return 1
    fi
    sleep 1
  done
  printf 'Hazır olması zaman aşımına uğradı: %s. Log: %s/%s.log\n' "$name" "$LOG_DIR" "$name" >&2
  return 1
}
launch() {
  local name=$1; shift
  local cmd
  printf -v cmd '%q ' "$@"
  printf -v cmd '%s > %q 2>&1' "exec $cmd" "$LOG_DIR/$name.log"
  tmux new-session -d -s "xr-teleop-$name" -c "$REPO" "bash -c $(printf '%q' "$cmd")"
}
stop_one() {
  local name=$1 pid
  if running "$name"; then
    pid=$(tmux display-message -p -t "=xr-teleop-$name" '#{pane_pid}')
    kill -INT "$pid" 2>/dev/null || true
    for ((i=0; i<10; i++)); do
      running "$name" || return 0
      sleep 1
    done
    tmux kill-session -t "=xr-teleop-$name"
  fi
}

QUEST_CAMERA="${XR_QUEST_CAMERA:-tcp://192.168.123.164:5605}"
case "${1:-start}" in
  start)
    shift || true
    source_=wilor
    view=ego
    for arg in "$@"; do
      case "$arg" in
        --quest) source_=quest ;;
        --immersive) view=immersive ;;
        *) printf 'Bilinmeyen seçenek: %s\n' "$arg" >&2; exit 2 ;;
      esac
    done
    mkdir -p "$LOG_DIR"
    # Serialize repeated launches without passing the lock to the background processes.
    exec 9>"$LOG_DIR/launch.lock"
    flock -n 9 || { printf 'Başlatma zaten devam ediyor.\n'; exit 1; }
    docker image inspect xrcam-ros:humble > /dev/null 2>&1 || bash "$REPO/deploy/build_ros_image.sh"
    if [[ "$source_" == quest ]]; then
      if running perception; then
        printf 'WiLoR çalışıyor; önce xr_teleop stop.\n' >&2; exit 1
      fi
      if ! running quest; then
        if http_ok http://127.0.0.1:8771/health; then
          printf '8771 portunda başka bir Quest sunucusu çalışıyor; önce onu durdur.\n' >&2
          exit 1
        fi
        printf 'Quest sunucusu başlatılıyor (robot kamerası: %s, %s)...\n' "$QUEST_CAMERA" "$view"
        launch quest "$REPO/data/quest-venv/bin/python" -u -m xr_camera_teleop.operator.quest.server \
          --camera "$QUEST_CAMERA" --view "$view" 9>&-
      fi
      wait_http quest http://127.0.0.1:8771/health
    else
      if running quest; then
        printf 'Quest sunucusu çalışıyor; önce xr_teleop stop.\n' >&2; exit 1
      fi
      if ! http_ok http://127.0.0.1:8766/status; then
        sudo -n systemctl start omron-camera.service || {
          printf 'Kamera köprüsünü başlat: sudo systemctl start omron-camera.service\n' >&2
          exit 1
        }
        sleep 2
      fi
      [[ -e /dev/omron-fh ]] || { printf '/dev/omron-fh bulunamadı.\n' >&2; exit 1; }
      if ! running perception; then
        if http_ok http://127.0.0.1:8770/health; then
          printf '8770 portunda başka bir sunucu çalışıyor; önce onu durdur.\n' >&2
          exit 1
        fi
        printf 'Omron / WiLoR başlatılıyor...\n'
        launch perception "$REPO/data/wilor-venv/bin/python" -u -m xr_camera_teleop.operator.perception.server --source omron 9>&-
      fi
      wait_http perception http://127.0.0.1:8770/health
    fi
    if ! running bridge; then
      printf 'ROS köprüsü başlatılıyor...\n'
      launch bridge "$REPO/data/xr-env/bin/python" -u "$REPO/deploy/run_bridge.py" 9>&-
    fi
    if ! running ui; then
      if http_ok "$URL"; then
        printf '8780 portunda başka bir arayüz çalışıyor; önce onu durdur.\n' >&2
        exit 1
      fi
      printf 'Web arayüzü başlatılıyor...\n'
      launch ui "$REPO/data/xr-env/bin/python" -u -m xr_camera_teleop.operator.live_ui --source "$source_" 9>&-
    fi
    wait_http ui "$URL"
    printf 'Hazır: %s\nLoglar: %s\n' "$URL" "$LOG_DIR"
    if [[ "$source_" == quest ]]; then
      for ip in $(hostname -I); do
        case "$ip" in 192.168.1.*) printf 'Quest tarayıcısında: https://%s:8012/?ws=wss://%s:8012\n' "$ip" "$ip" ;; esac
      done
    fi
    # SSH launches can use the existing MSI desktop session to open the browser.
    if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
      while IFS='=' read -r key value; do
        case "$key" in
          DISPLAY|WAYLAND_DISPLAY|XAUTHORITY|DBUS_SESSION_BUS_ADDRESS|XDG_RUNTIME_DIR)
            export "$key=$value" ;;
        esac
      done < <(systemctl --user show-environment 2>/dev/null || true)
    fi
    if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
      nohup xdg-open "$URL" >"$LOG_DIR/browser.log" 2>&1 9>&- &
    else
      printf 'Tarayıcı için MSI masaüstünde %s adresini aç.\n' "$URL"
    fi
    ;;
  stop)
    stop_one ui
    stop_one bridge
    stop_one perception
    stop_one quest
    printf 'WiLoR/Quest, köprü ve arayüz durduruldu. Robot olduğu yerde tutuyor.\n'
    ;;
  status)
    for name in perception quest bridge ui; do
      if running "$name"; then printf '%s: çalışıyor\n' "$name"; else printf '%s: kapalı\n' "$name"; fi
    done
    curl -fsS --max-time 2 http://127.0.0.1:8766/status || true
    printf '\nArayüz: %s\n' "$URL"
    ;;
  logs)
    tail -n 40 -F "$LOG_DIR/perception.log" "$LOG_DIR/quest.log" "$LOG_DIR/bridge.log" "$LOG_DIR/ui.log"
    ;;
  *) printf 'Kullanım: xr_teleop [start [--quest [--immersive]]|stop|status|logs]\n' >&2; exit 2 ;;
esac
