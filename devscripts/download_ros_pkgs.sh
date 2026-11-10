#!/usr/bin/env bash
apt update
set -eo pipefail
cd "$(dirname "$0")/.."
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
