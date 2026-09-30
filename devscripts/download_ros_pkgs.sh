# !/bin/bash
rosdep install --from-paths src --ignore-src -r -y

apt install tmux --yes
apt install ros-$ROS_DISTRO-foxglove-bridge --yes
apt install libgz-sensors6-lidar libgz-sensors6 --yes
apt install libgz-sensors6-lidar libgz-sensors6-dev libgz-sensors6-gpu-lidar libgz-sensors6-gpu-lidar-dev --yes