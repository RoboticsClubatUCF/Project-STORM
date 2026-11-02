FROM osrf/ros:jazzy-desktop
RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake build-essential python3-colcon-common-extensions python3-rosdep \
    ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
    ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*
