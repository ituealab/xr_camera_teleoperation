#!/bin/bash
# Meta Quest over ADB from MSI: keep it awake and reachable, open the teleop page, take screenshots.
#
#   deploy/quest_adb.sh keepalive      # once: wireless ADB up, screen never off, proximity sensor ignored
#   deploy/quest_adb.sh watch          # loop (systemd quest-keepalive.service): re-applies keepalive after a
#                                      # Quest reboot; needs the USB cable plugged in to re-enable wireless ADB
#   deploy/quest_adb.sh open [URL]     # open the teleop page in Quest Browser (Enter VR stays a user click)
#   deploy/quest_adb.sh shot [FILE]    # screenshot of the headset view (PNG)
#   deploy/quest_adb.sh status
#
# Uses its own adb server port so another user's adb server on 5037 is left alone.
set -euo pipefail

export ANDROID_ADB_SERVER_PORT="${QUEST_ADB_PORT:-5039}"
QUEST_IP="${QUEST_IP:-192.168.1.106}"
QUEST_URL="${QUEST_URL:-https://192.168.1.105/}"
TCP="$QUEST_IP:5555"

usb_serial() { adb devices | awk 'NR>1 && $2=="device" && $1 !~ /:/ {print $1; exit}'; }
tcp_ok() { adb devices | awk -v t="$TCP" '$1==t && $2=="device" {f=1} END {exit !f}'; }

ensure_tcp() {
  tcp_ok && return 0
  adb connect "$TCP" >/dev/null 2>&1 || true
  tcp_ok && return 0
  local s; s=$(usb_serial)
  if [ -n "$s" ]; then
    adb -s "$s" tcpip 5555 >/dev/null
    sleep 3
    adb connect "$TCP" >/dev/null 2>&1 || true
  fi
  tcp_ok
}

awake() {
  adb -s "$TCP" shell am broadcast -a com.oculus.vrpowermanager.prox_close >/dev/null
  adb -s "$TCP" shell svc power stayon true
  adb -s "$TCP" shell settings put system screen_off_timeout 2147483647
}

case "${1:-status}" in
  keepalive)
    ensure_tcp || { echo "Quest not reachable at $TCP (plug the USB cable in once)" >&2; exit 1; }
    awake; echo "Quest $TCP: wireless ADB, awake, proximity sensor ignored" ;;
  watch)
    adb start-server >/dev/null 2>&1
    last=""
    while true; do
      if ensure_tcp; then
        boot=$(adb -s "$TCP" shell getprop sys.boot_completed 2>/dev/null | tr -d '\r')
        id=$(adb -s "$TCP" shell cat /proc/sys/kernel/random/boot_id 2>/dev/null | tr -d '\r')
        if [ "$boot" = "1" ] && [ "$id" != "$last" ]; then
          awake && last="$id" && echo "$(date -Is) keepalive applied (boot $id)"
        fi
        # the proximity override expires when the headset sleeps; re-assert it cheaply
        adb -s "$TCP" shell am broadcast -a com.oculus.vrpowermanager.prox_close >/dev/null 2>&1 || true
      fi
      sleep 30
    done ;;
  open)
    ensure_tcp
    adb -s "$TCP" shell am start -a android.intent.action.VIEW -d "${2:-$QUEST_URL}" com.oculus.browser >/dev/null
    echo "opened ${2:-$QUEST_URL}" ;;
  shot)
    ensure_tcp
    out="${2:-quest-$(date +%Y%m%d-%H%M%S).png}"
    adb -s "$TCP" exec-out screencap -p > "$out"; echo "$out" ;;
  status)
    adb devices -l
    tcp_ok && adb -s "$TCP" shell dumpsys power | grep -E "mWakefulness=|mStayOn=" || true ;;
  *) echo "usage: $0 keepalive|watch|open [URL]|shot [FILE]|status" >&2; exit 2 ;;
esac
