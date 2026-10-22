# !/bin/bash
rosdep install --from-paths src --ignore-src -r -y

sudo apt update
sudo apt install --yes tmux
sudo apt install --yes ros-$ROS_DISTRO-foxglove-bridge
sudo apt install --yes libgz-sensors6-lidar
sudo apt install --yes libgz-sensors6-dev libgz-sensors6-gpu-lidar libgz-sensors6-gpu-lidar-dev 