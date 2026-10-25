#!/usr/bin/env bash
# =============================================================================
# link_up.sh: static IP on the base Jetson's Ethernet port to the radio.
#
#   sudo ./link_up.sh
#   sudo LINK_IFACE=eth1 ./link_up.sh
#
# Creates or updates the NetworkManager profile "storm-link" from
# config/network.env, so it persists and comes back at boot. Safe to re-run:
# it only touches the link when something differs. In Docker it talks to the
# host's NetworkManager over the mounted system D-Bus socket.
# =============================================================================
set -euo pipefail

CON_NAME="storm-link"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for d in "${SCRIPT_DIR}/../config" "${SCRIPT_DIR}/../../share/storm_basestation/config"; do
  if [[ -f "${d}/network.env" ]]; then
    # Environment wins over network.env
    while IFS='=' read -r key value; do
      if [[ "$key" =~ ^[A-Z_]+$ && -z "${!key:-}" ]]; then export "${key}=${value}"; fi
    done < "${d}/network.env"
    break
  fi
done

BASE_IP="${BASE_IP:-192.168.1.60}"
LINK_PREFIX="${LINK_PREFIX:-24}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "link_up.sh: must run as root." >&2
  exit 1
fi

if ! nm_err="$(nmcli -t general status 2>&1 >/dev/null)"; then
  echo "link_up.sh: can't reach NetworkManager: ${nm_err}" >&2
  if [[ ! -S /run/dbus/system_bus_socket ]]; then
    echo "link_up.sh: /run/dbus/system_bus_socket is not a socket; is it mounted from the host?" >&2
  fi
  exit 1
fi

if [[ -z "${LINK_IFACE:-}" ]]; then
  mapfile -t ethernet < <(nmcli -t -f DEVICE,TYPE dev | awk -F: '$2 == "ethernet" {print $1}')
  if (( ${#ethernet[@]} != 1 )); then
    echo "link_up.sh: found ${#ethernet[@]} Ethernet devices (${ethernet[*]:-none}); set LINK_IFACE in network.env." >&2
    exit 1
  fi
  LINK_IFACE="${ethernet[0]}"
fi

declare -A want=(
  [connection.interface-name]="${LINK_IFACE}"
  [connection.autoconnect]="yes"
  [connection.autoconnect-priority]="10"
  [ipv4.method]="manual"
  [ipv4.addresses]="${BASE_IP}/${LINK_PREFIX}"
  [ipv4.never-default]="yes"
  [ipv4.routes]="224.0.0.0/4"
  [ipv6.method]="disabled"
)

changes=()
if nmcli -t -f NAME con show | grep -qx "${CON_NAME}"; then
  for field in "${!want[@]}"; do
    have="$(nmcli -g "${field}" con show "${CON_NAME}")"
    [[ "$have" == "${want[$field]}" ]] || changes+=("${field}" "${want[$field]}")
  done
  if (( ${#changes[@]} )); then
    nmcli con mod "${CON_NAME}" "${changes[@]}"
    status="updated"
  else
    status="already set"
  fi
else
  args=()
  for field in "${!want[@]}"; do args+=("${field}" "${want[$field]}"); done
  nmcli con add type ethernet con-name "${CON_NAME}" "${args[@]}" >/dev/null
  status="created"
fi

active_on="$(nmcli -g GENERAL.DEVICES con show "${CON_NAME}" 2>/dev/null || true)"
if [[ "$status" != "already set" || "$active_on" != "${LINK_IFACE}" ]]; then
  if ! nmcli con up "${CON_NAME}" >/dev/null 2>&1; then
    echo "link_up.sh: ${CON_NAME} saved but not up (cable or radio unplugged?); NetworkManager brings it up when the link appears."
    exit 0
  fi
fi

echo "link_up.sh: ${CON_NAME} ${status}, ${LINK_IFACE} at ${BASE_IP}/${LINK_PREFIX}."
