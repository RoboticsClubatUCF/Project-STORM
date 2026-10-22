#!/usr/bin/env bash
# Launch the integrated ROSMASTER X3 Gazebo and Nav2 stack.
# Pass "slam" to start mapping and optionally pass a map YAML path.

set -euo pipefail

slam_arg="slam:=False"
if [[ "${1:-}" == "slam" ]]; then
  slam_arg="slam:=True"
fi

map_args=()
if [[ -n "${2:-}" ]]; then
  map_args=("map:=${2}")
fi

exec ros2 launch yahboom_rosmaster_bringup rosmaster_x3_navigation.launch.py \
  world_file:=cafe.world \
  use_sim_time:=true \
  "${slam_arg}" \
  "${map_args[@]}"
