FROM osrf/ros:jazzy-desktop-full

ENV ROS_DISTRO=$ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get -y --no-install-recommends install clangd clang git make
RUN git clone https://github.com/grayson-arendt/sparkcan.git

RUN cd sparkcan && cmake -S . -B ./build && cmake --build ./build
RUN cd sparkcan/build && make install

RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake build-essential python3-colcon-common-extensions python3-rosdep \
    ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
    ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-xacro \
    ros-jazzy-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y --no-install-recommends \
  clang-format clang-tidy \
  && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
