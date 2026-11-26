FROM osrf/ros:jazzy-desktop-full

EXPOSE 8764/tcp
EXPOSE 8765/tcp

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
    can-utils iproute2 \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y --no-install-recommends \
  clang-format clang-tidy \
  && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

SHELL ["bash", "-c"]

RUN bash devscripts/download_ros_pkgs.sh
RUN bash devscripts/build.sh

# RUN --mount=source=/sys/devices/platform/c310000.mttcan/,target=/dev/can0,type=bind

ENTRYPOINT ["ros2", "launch", "storm_bringup", "robot.launch.py"]