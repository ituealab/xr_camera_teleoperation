#!/usr/bin/env bash
# Full-body Quest teleop on the real robot (PC2), the counterpart of deploy/xr_fullbody_sim.sh (PLAN D11: same
# process graph, the DDS end is the robot). docs/quest-sonic/ROBOT.md. Aliases (deploy/setup_robot_fullbody.sh):
#   sonic-quest  = this sonic    SONIC v1.1 (zmq_manager, configs/sonic_robot.yaml) in the FOREGROUND of this
#                                terminal: `o` = emergency stop. Never relaunched by anything: every start ramps all
#                                joints to the default pose, so it is always the operator's command.
#   quest        = this start    camera (RealSense IR stereo -> :5605), Quest server (https://<robot>/, :5596 xr_frame,
#                                panel), fullbody_node (--source quest, the robot SONIC config); one tmux session each,
#                                logs in ~/.local/state/xr_fullbody; idempotent (starts only what is not running)
#   quest stop [NAME] | status | logs | attach camera|quest|node
set -euo pipefail
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
LOG_DIR="$HOME/.local/state/xr_fullbody"
NODE_PY="$REPO/data/xr-env/bin/python"
QUEST_PY="$REPO/data/quest-env/bin/python"
CAMERA_PY="$REPO/data/camera-env/bin/python"
QUEST_HEALTH=http://127.0.0.1:8771/health
ROBOT_IP="${ROBOT_IP:-192.168.1.164}"           # aksoy-lab, static (ROBOT.md)
QUEST_URL="https://$ROBOT_IP"                   # :443 -> :8012 (quest-https-redirect.service)
SONIC_CONFIG="$REPO/configs/sonic_robot.yaml"
CAMERA_ARGS=(--bind tcp://127.0.0.1:5605 --mode 1280x800@30 --quality 80)
PREFIX=xr-fullbody
NAMES=(camera quest node)

running() { tmux has-session -t "=$PREFIX-$1" 2>/dev/null; }
launch() {
  local name=$1; shift
  local cmd
  printf -v cmd '%q ' "$@"
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
    running "$1" || { printf '%s kapandı. Log: %s/%s.log\n' "$1" "$LOG_DIR" "$1" >&2; return 1; }
    sleep 1
  done
  printf '%s %s s içinde hazır olmadı. Log: %s/%s.log\n' "$1" "$3" "$LOG_DIR" "$1" >&2
  return 1
}

free_camera() {   # Unitree's video hubs hold the RealSense's UVC nodes (g1_ros_env/tools/free-camera)
  if /unitree/sbin/mscli getenable video_hub_pc4_chest 2>/dev/null | grep -q enable:0; then return 0; fi
  sudo -n "$HOME/g1_ros_env/tools/free-camera" >/dev/null || printf 'UYARI: Unitree video servisleri durdurulamadı; kamera meşgul olabilir.\n' >&2
}

start() {
  [[ -x "$NODE_PY" && -x "$QUEST_PY" ]] || { printf 'Ortam yok: önce  bash %s/deploy/setup_robot_fullbody.sh\n' "$REPO" >&2; exit 1; }
  mkdir -p "$LOG_DIR"
  if ! running camera; then
    if [[ -f "$REPO/deploy/realsense_ir_stream.py" ]]; then
      free_camera
      launch camera "$CAMERA_PY" -u "$REPO/deploy/realsense_ir_stream.py" "${CAMERA_ARGS[@]}"
    else
      printf 'UYARI: deploy/realsense_ir_stream.py yok; kamerasız devam.\n' >&2
    fi
  fi
  if ! running quest; then
    if curl -fsS --max-time 2 "$QUEST_HEALTH" >/dev/null 2>&1; then
      printf 'Başka bir Quest sunucusu %s adresinde cevap veriyor; bizimki başlatılmadı.\n' "$QUEST_HEALTH" >&2
    else
      launch quest "$QUEST_PY" -u -m xr_camera_teleop.operator.quest.server --camera tcp://127.0.0.1:5605 --view ego
      wait_log quest 'Quest browser' 60
    fi
  fi
  if ! running node; then
    launch node "$NODE_PY" -u -m xr_camera_teleop.fullbody.node --source quest --sonic-config "$SONIC_CONFIG"
  fi
  sleep 2
  local failed=0
  for name in "${NAMES[@]}"; do
    running "$name" || { printf 'BAŞARISIZ: %s çalışmıyor. Log: %s/%s.log\n' "$name" "$LOG_DIR" "$name" >&2; failed=1; }
  done
  ((failed)) && exit 1
  cat <<EOF

  Hazır. SONIC ayrı terminalde açık olmalı (sonic-quest).
  1. Quest: $QUEST_URL adresini aç, Enter VR. Robotun kamerasını görürsün.
  2. Panel için bir thumbstick'e bas: Başlat, sonra dik dur ve ileri bak, "Kalibre et ve devral" (ya da A'yı 1 sn tut).
  3. Bitiş: paneldeki DURDUR (ya da iki grip + tetik). Kapat: quest stop, SONIC terminalinde Ctrl-C.

EOF
}

sonic() {
  local mode
  mode=$("$HOME/.local/bin/developer" status 2>&1 || true)
  if ! grep -q 'motion servisi bırakılmış' <<<"$mode"; then
    printf 'Robot developer modunda değil (%s).\nÖnce: developer start   (denge kontrolü kapanır: robot askıda olmalı)\n' "$mode" >&2
    exit 1
  fi
  printf 'SONIC açılınca bütün eklemler varsayılan duruşa gider. Robot askıda / destekli mi? [e/H] '
  local answer=""
  read -r answer || true
  [[ "$answer" == [eEyY] ]] || { printf 'İptal edildi; hiçbir şey hareket etmedi.\n' >&2; exit 1; }
  printf 'Acil durdurma: bu terminalde  o  tuşu.\n'
  exec "$NODE_PY" -u -m xr_camera_teleop.sonic.run_deploy --config "$SONIC_CONFIG"
}

case "${1:-start}" in
  start) start ;;
  sonic) sonic ;;
  stop)
    if [[ -n "${2:-}" ]]; then stop_one "$2"; else for n in node quest camera; do stop_one "$n"; done; fi
    printf 'Durduruldu.\n' ;;
  status)
    for n in "${NAMES[@]}"; do printf '%-7s %s\n' "$n" "$(running "$n" && echo çalışıyor || echo kapalı)"; done
    curl -fsS --max-time 2 "$QUEST_HEALTH" 2>/dev/null | head -c 400 && echo || true
    printf 'Quest adresi: %s\n' "$QUEST_URL" ;;
  logs) tail -n 30 "$LOG_DIR"/{camera,quest,node}.log 2>/dev/null || true ;;
  attach) tmux attach -t "=$PREFIX-${2:?attach camera|quest|node}" ;;
  *) printf 'Kullanım: quest [start] | stop [camera|quest|node] | status | logs | attach NAME ; sonic-quest\n' >&2; exit 2 ;;
esac
