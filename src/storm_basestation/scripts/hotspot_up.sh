#!/usr/bin/env bash
# =============================================================================
# hotspot_up.sh: WiFi hotspot on the base station Jetson for viewing laptops.
#
# Run once; NetworkManager keeps the connection and brings it up at boot.
# Re-running replaces it with the new settings.
#
#   sudo PASSWORD=<8+ chars> ./hotspot_up.sh
#   sudo PASSWORD=... IFACE=wlan1 SSID=STORM-BASE BAND=a ./hotspot_up.sh
#
# ipv4.method shared hands out addresses on 10.42.0.0/24 and NATs laptop
# traffic out to the rover over Ethernet, so ws://<rover-ip>:8765 just works.
# =============================================================================
set -euo pipefail

IFACE="${IFACE:-wlan0}"
SSID="${SSID:-STORM-BASE}"
BAND="${BAND:-bg}"        # bg = 2.4 GHz, a = 5 GHz
CON_NAME="storm-hotspot"

if [[ "${EUID}" -ne 0 ]]; then
  echo "hotspot_up.sh: must run as root." >&2
  exit 1
fi

if [[ -z "${PASSWORD:-}" || ${#PASSWORD} -lt 8 ]]; then
  echo "hotspot_up.sh: set PASSWORD (at least 8 characters)." >&2
  exit 1
fi

if ! nmcli -t -f DEVICE dev | grep -qx "${IFACE}"; then
  echo "hotspot_up.sh: no device '${IFACE}'. Check 'nmcli dev'." >&2
  exit 1
fi

nmcli con delete "${CON_NAME}" >/dev/null 2>&1 || true
nmcli con add type wifi ifname "${IFACE}" con-name "${CON_NAME}" autoconnect yes \
  ssid "${SSID}" \
  802-11-wireless.mode ap \
  802-11-wireless.band "${BAND}" \
  ipv4.method shared \
  ipv4.addresses 10.42.0.1/24 \
  ipv6.method disabled \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "${PASSWORD}"
nmcli con up "${CON_NAME}"

echo "hotspot_up.sh: '${SSID}' up on ${IFACE}, base station at 10.42.0.1."