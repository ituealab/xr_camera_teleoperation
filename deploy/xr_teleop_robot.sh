#!/usr/bin/env bash
# Robot-side launcher, installed as ~/.local/bin/xr_teleop.
set -euo pipefail
# Avoid ROS / flux_dex3 Python dependencies shadowing xrcam-env packages.
unset PYTHONPATH PYTHONHOME
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
SESSION=(bash "$REPO/deploy/robot_session.sh")
SERVICES=(python3 "$REPO/deploy/recording_services.py")
READY_TIMEOUT_SECONDS=120

field() {
  "${SESSION[@]}" status | "$HOME/xrcam-env/bin/python" -c "import json,sys; print(json.load(sys.stdin).get('$1'))"
}

prepare() {
  export XRCAM_ALLOW_STARTUP_CONTACT="${XRCAM_ALLOW_STARTUP_CONTACT:-1}"
  "${SESSION[@]}" prepare "$@"
}

confirm_supported() {
  "$HOME/.local/bin/developer" status | grep -q 'motion servisi bırakılmış' && return
  [[ "${1:-}" == --yes ]] && return
  printf 'Denge kontrolü kapanacak. Robot askıda / destekli mi? [e/H] '
  local answer=""
  read -r answer || true
  [[ "$answer" == [eEyY] ]] || { printf 'İptal edildi; hiçbir şey hareket etmedi.\n' >&2; exit 1; }
}

wait_for_home() {
  local stage=""
  for ((i=0; i<READY_TIMEOUT_SECONDS; i++)); do
    stage="$(field stage 2>/dev/null || true)"
    [[ "$stage" == ready ]] && return
    sleep 1
  done
  printf 'Robot hazır olmadı (stage=%s). Log: xr_teleop logs\n' "$stage" >&2
  exit 1
}

case "${1:-help}" in
  up)
    shift
    # Cameras first: a camera fault should stop here, before balance is released.
    printf 'Kameralar başlatılıyor...\n'
    "${SERVICES[@]}" start all > /dev/null
    confirm_supported "${1:-}"
    "$HOME/.local/bin/developer" start
    printf 'Kollar home konumuna gidiyor...\n'
    prepare
    wait_for_home
    "${SESSION[@]}" enable > /dev/null
    printf 'Hazır. MSI arayüzünde R ile başlat.\n'
    ;;
  down)
    "${SESSION[@]}" stop || true
    "${SERVICES[@]}" stop all > /dev/null
    printf 'Robot düğümü ve kameralar durduruldu.\n'
    ;;
  start|prepare)
    shift
    prepare "$@"
    ;;
  enable|hold|home|stop|status|settings)
    exec "${SESSION[@]}" "$@"
    ;;
  logs)
    exec tail -n 40 -F "${XRCAM_SESSION_DIR:-$REPO/data/session-20261009}/robot.log"
    ;;
  help|-h|--help)
    printf 'Kullanım: xr_teleop up [--yes]|down|start|status|enable|hold|home|stop|settings|logs\n'
    printf 'up: kameralar, developer modu, kollar home, enable. Sonra MSI arayüzünde R.\n'
    printf 'down: robot düğümü ve kameraları durdurur.\n'
    ;;
  *) printf 'Bilinmeyen komut: %s\n' "$1" >&2; exit 2 ;;
esac
