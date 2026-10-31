#!/usr/bin/env bash
# MSI-side launcher, installed as ~/.local/bin/xr_teleop.
set -euo pipefail
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
LOG_DIR="$HOME/.local/state/xr_teleop"
export WILOR_ROOT="${WILOR_ROOT:-$HOME/code/wilor_faruk/third_party/WiLoR}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
ROBOT="${XRCAM_ROBOT:-tcp://192.168.123.164:5592}"
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

case "${1:-start}" in
  start)
    mkdir -p "$LOG_DIR"
    # Serialize repeated launches without passing the lock to the background processes.
    exec 9>"$LOG_DIR/launch.lock"
    flock -n 9 || { printf 'Başlatma zaten devam ediyor.\n'; exit 1; }
    docker image inspect xrcam-recorder:humble > /dev/null 2>&1 || bash "$REPO/deploy/build_recorder.sh"
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
    if ! running ui; then
      if http_ok "$URL"; then
        printf '8780 portunda başka bir arayüz çalışıyor; önce onu durdur.\n' >&2
        exit 1
      fi
      printf 'Web arayüzü başlatılıyor...\n'
      launch ui "$REPO/data/xr-env/bin/python" -u -m xr_camera_teleop.operator.live_ui --robot "$ROBOT" 9>&-
    fi
    wait_http ui "$URL"
    printf 'Hazır: %s\nRobot: %s\nLoglar: %s\n' "$URL" "$ROBOT" "$LOG_DIR"
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
    stop_one perception
    printf 'WiLoR ve arayüz durduruldu.\n'
    ;;
  status)
    for name in perception ui; do
      if running "$name"; then printf '%s: çalışıyor\n' "$name"; else printf '%s: kapalı\n' "$name"; fi
    done
    curl -fsS --max-time 2 http://127.0.0.1:8766/status || true
    printf '\nArayüz: %s\n' "$URL"
    ;;
  logs)
    tail -n 40 -F "$LOG_DIR/perception.log" "$LOG_DIR/ui.log"
    ;;
  *) printf 'Kullanım: xr_teleop [start|stop|status|logs]\n' >&2; exit 2 ;;
esac
