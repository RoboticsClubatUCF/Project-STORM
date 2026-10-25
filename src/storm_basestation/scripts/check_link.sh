#!/usr/bin/env bash
# =============================================================================
# check_link.sh: checks the base station's link to the rover, layer by layer.
#
#   ./check_link.sh
#   ROVER_IP=192.168.1.50 ./check_link.sh      # override config/network.env
#
# Needs the rover running teleop.launch.py with the same ROS_DOMAIN_ID.
# The DDS check runs without FASTRTPS_DEFAULT_PROFILES_FILE first, so passing
# it means multicast discovery crosses the radios. Exits 1 on any failure.
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR=""
for d in "${SCRIPT_DIR}/../config" "${SCRIPT_DIR}/../../share/storm_basestation/config"; do
  [[ -d "$d" ]] && CONFIG_DIR="$(cd "$d" && pwd)" && break
done

# Environment wins over network.env
if [[ -f "${CONFIG_DIR}/network.env" ]]; then
  while IFS='=' read -r key value; do
    [[ "$key" =~ ^[A-Z_]+$ && -z "${!key:-}" ]] && export "${key}=${value}"
  done < "${CONFIG_DIR}/network.env"
fi

ROVER_IP="${ROVER_IP:-192.168.1.50}"
BASE_IP="${BASE_IP:-192.168.1.60}"
ROVER_RADIO_IP="${ROVER_RADIO_IP:-192.168.1.21}"
BASE_RADIO_IP="${BASE_RADIO_IP:-192.168.1.20}"
LINK_PREFIX="${LINK_PREFIX:-24}"
BRIDGE_PORT="${BRIDGE_PORT:-8765}"
ROVER_NODE="${ROVER_NODE:-/teleop_twist_joy_node}"
SPIN_TIME="${SPIN_TIME:-5}"
DDS_GROUP="239.255.0.1"

PEERS_FILE=""
[[ -f "${CONFIG_DIR}/fastdds_peers.xml" ]] && PEERS_FILE="${CONFIG_DIR}/fastdds_peers.xml"

failed=0
ok()   { echo "[ OK ] $1"; }
warn() { echo "[WARN] $1"; [[ -n "${2:-}" ]] && echo "       $2"; }
fail() { echo "[FAIL] $1"; [[ -n "${2:-}" ]] && echo "       $2"; failed=1; }

finish() {
  if (( failed )); then
    echo "check_link.sh: link to rover is NOT ready."
    exit 1
  fi
  echo "check_link.sh: link to rover is ready."
  exit 0
}

# 1) Link interface
iface="$(ip -o -4 addr show | awk -v ip="${BASE_IP}" '{split($4, a, "/"); if (a[1] == ip) print $2}' | head -n1)"
if [[ -z "$iface" ]]; then
  fail "No interface has ${BASE_IP}." \
       "sudo ${SCRIPT_DIR}/link_up.sh"
  finish
fi
if ip link show "$iface" | grep -q "LOWER_UP"; then
  ok "${iface} is up with ${BASE_IP}"
else
  fail "${iface} has ${BASE_IP} but no carrier." "Check the cable to the radio, then: sudo nmcli con up storm-link"
  finish
fi

# 2) Multicast flag
if ip link show "$iface" | grep -q "MULTICAST"; then
  ok "Multicast enabled on ${iface}"
else
  fail "Multicast disabled on ${iface}." "sudo ip link set ${iface} multicast on"
fi

# 3) Multicast route
mc_dev="$(ip route get "${DDS_GROUP}" 2>/dev/null | grep -o 'dev [^ ]*' | awk '{print $2}' | head -n1)"
if [[ "$mc_dev" == "$iface" ]]; then
  ok "Multicast to ${DDS_GROUP} routes via ${iface}"
else
  warn "Multicast to ${DDS_GROUP} routes via ${mc_dev:-nothing}, not ${iface}." \
       "sudo ip route add 224.0.0.0/4 dev ${iface}"
fi

# 4) Each hop: base radio, rover radio, rover
pings() { ping -c 3 -W 1 "$1" >/dev/null 2>&1; }

if pings "${BASE_RADIO_IP}"; then
  ok "Base radio ${BASE_RADIO_IP} answers ping"
else
  fail "Base radio ${BASE_RADIO_IP} doesn't answer ping." "Check its power and the cable from ${iface}."
  finish
fi
if pings "${ROVER_RADIO_IP}"; then
  ok "Rover radio ${ROVER_RADIO_IP} answers ping"
else
  fail "Rover radio ${ROVER_RADIO_IP} doesn't answer ping." "The radios aren't linked: check rover radio power, pairing and antennas."
  finish
fi
if pings "${ROVER_IP}"; then
  ok "Rover ${ROVER_IP} answers ping"
else
  fail "Rover ${ROVER_IP} doesn't answer ping." "Radios are linked, so check the rover Jetson is on, its cable to the radio, and its storm-link IP."
  finish
fi

# 5) foxglove_bridge
if timeout 2 bash -c "</dev/tcp/${ROVER_IP}/${BRIDGE_PORT}" 2>/dev/null; then
  ok "foxglove_bridge open on ${ROVER_IP}:${BRIDGE_PORT}"
else
  warn "foxglove_bridge not reachable on ${ROVER_IP}:${BRIDGE_PORT} (driving still works)." \
       "Is the rover launch running with enable_bridge:=true?"
fi

# 6) DDS discovery over multicast
if ! command -v ros2 >/dev/null 2>&1; then
  fail "ros2 not found." "source /opt/ros/humble/setup.bash"
  finish
fi

# $1: Fast DDS profiles file, empty for default multicast discovery
dds_sees_rover() {
  if [[ -n "$1" ]]; then
    FASTRTPS_DEFAULT_PROFILES_FILE="$1" ros2 node list --no-daemon --spin-time "${SPIN_TIME}"
  else
    env -u FASTRTPS_DEFAULT_PROFILES_FILE ros2 node list --no-daemon --spin-time "${SPIN_TIME}"
  fi 2>/dev/null | grep -qx "${ROVER_NODE}"
}

if dds_sees_rover ""; then
  ok "DDS multicast discovery sees ${ROVER_NODE} (domain ${ROS_DOMAIN_ID:-0})"
  finish
fi

# 7) Multicast failed: does unicast work?
if [[ -n "$PEERS_FILE" ]] && dds_sees_rover "$PEERS_FILE"; then
  fail "Radios drop multicast; unicast discovery works." \
       "Enable multicast pass-through on the radios, or export FASTRTPS_DEFAULT_PROFILES_FILE=${PEERS_FILE}"
else
  fail "DDS can't see ${ROVER_NODE} on domain ${ROS_DOMAIN_ID:-0}." \
       "Check the rover runs teleop.launch.py use_joy:=false with the same ROS_DOMAIN_ID."
fi
finish
