#!/usr/bin/env bash
# Run from a Linux desktop / WSLg session. Forward optional dashboard arguments.
set -e
workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
if [ ! -f "$workspace/install/setup.bash" ]; then
  echo 'Build the workspace with colcon before launching the dashboard.' >&2
  exit 1
fi
source "$workspace/install/setup.bash"
# Support the existing local development dependency without modifying system Python.
if ! python3 -c 'import pyproj' 2>/dev/null; then
  if [ -d "$workspace/.deps/mission-python/pyproj" ]; then
    export PYTHONPATH="$workspace/.deps/mission-python${PYTHONPATH:+:$PYTHONPATH}"
  else
    echo 'Install the declared dependencies using rosdep (python3-pyproj is missing).' >&2
    exit 1
  fi
fi
exec ros2 run storm_dashboard dashboard "$@"
