#!/usr/bin/env bash
# =============================================================================
# can0_up.sh: bring up the SocketCAN interface for the drivetrain SPARKs.
#
# Idempotent: safe to run when the interface is already up (it re-applies the
# config). Needs CAP_NET_ADMIN, so run as root (or via the storm-can0.service
# systemd unit).
#
# Usage:
#   sudo ./can0_up.sh                 # can0 @ 1 Mbit/s
#   sudo IFACE=can1 BITRATE=500000 ./can0_up.sh
#
# restart-ms 100 enables automatic bus-off recovery: if the controller goes
# bus-off from an error storm, the kernel re-initialises it after 100 ms
# instead of leaving it dead until a manual restart.
# =============================================================================
set -euo pipefail

IFACE="${IFACE:-can0}"
BITRATE="${BITRATE:-1000000}"
RESTART_MS="${RESTART_MS:-100}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "can0_up.sh: must run as root (needs CAP_NET_ADMIN)." >&2
  exit 1
fi

if ! ip link show "${IFACE}" >/dev/null 2>&1; then
  echo "can0_up.sh: interface '${IFACE}' not found. Is the CAN adapter plugged in?" >&2
  exit 1
fi

# Take it down first so we can (re)apply the bitrate cleanly.
ip link set "${IFACE}" down || true
ip link set "${IFACE}" up type can bitrate "${BITRATE}" restart-ms "${RESTART_MS}"

echo "can0_up.sh: ${IFACE} up @ ${BITRATE} bit/s (restart-ms ${RESTART_MS})."
ip -details -brief link show "${IFACE}" || true
