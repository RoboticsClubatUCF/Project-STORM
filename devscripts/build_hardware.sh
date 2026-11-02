#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
revision=11d2d13787fc32e508690926efc3c9749e58770e
if [[ ! -d .deps/sparkcan-src/.git ]]; then
  git clone https://github.com/grayson-arendt/sparkcan.git .deps/sparkcan-src
fi
git -C .deps/sparkcan-src checkout --detach "$revision"
colcon build --symlink-install --packages-up-to storm_hardware --cmake-args \
  -DSTORM_BUILD_CAN_DRIVER=ON -DFETCHCONTENT_SOURCE_DIR_SPARKCAN="$PWD/.deps/sparkcan-src"
