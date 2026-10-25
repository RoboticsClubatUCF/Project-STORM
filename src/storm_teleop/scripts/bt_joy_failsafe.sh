#!/usr/bin/env bash
# =============================================================================
# bt_joy_failsafe.sh: drop a dead Bluetooth controller in ~1 s instead of ~20 s.
#
# Until the BT link times out, joy_node keeps republishing the last state, so a
# held deadman keeps driving. Shortening the link supervision timeout lets the
# kernel drop the controller fast; cmd_vel_timeout then stops the rover.
# Run by udev/99-storm-bt-joy.rules when a joystick appears.
#
# Usage:
#   sudo ./bt_joy_failsafe.sh                 # 1000 ms
#   sudo TIMEOUT_MS=800 ./bt_joy_failsafe.sh
# =============================================================================
set -uo pipefail

TIMEOUT_MS="${TIMEOUT_MS:-1000}"
SLOTS=$(( TIMEOUT_MS * 1000 / 625 ))   # 0.625 ms baseband slots

log() { echo "bt_joy_failsafe.sh: $*"; logger -t bt_joy_failsafe "$*" 2>/dev/null || true; }

if [[ "${EUID}" -ne 0 ]]; then
  echo "bt_joy_failsafe.sh: must run as root." >&2
  exit 1
fi

if ! command -v hcitool >/dev/null 2>&1; then
  log "hcitool not found (sudo apt install bluez)."
  exit 1
fi

# Give the HID link a moment to settle after the input device appears
sleep 1

links="$(hcitool con | awk '$2 == "ACL" {print $3, $9}')"
if [[ -z "$links" ]]; then
  log "no Bluetooth links."
  exit 0
fi

status=0
while read -r mac role; do
  if [[ "$role" != "MASTER" && "$role" != "CENTRAL" ]]; then
    hcitool sr "$mac" master >/dev/null 2>&1 || true
    sleep 0.5
  fi
  if hcitool lst "$mac" "$SLOTS" >/dev/null 2>&1; then
    log "${mac}: link supervision timeout ${TIMEOUT_MS} ms."
  else
    log "${mac}: couldn't set link supervision timeout (rover not master on this link?)."
    status=1
  fi
done <<< "$links"

exit "$status"