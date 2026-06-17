# !/bin/bash

export DEBIAN_FRONTEND=noninteractive

rosdep install --from-paths src --ignore-src -r -y
