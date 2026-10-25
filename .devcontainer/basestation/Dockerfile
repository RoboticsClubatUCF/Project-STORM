ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-joy \
      ros-${ROS_DISTRO}-foxglove-bridge \
      joystick \
    && rm -rf /var/lib/apt/lists/*

FROM base AS dev
ARG ROS_DISTRO
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      git \
      python3-colcon-common-extensions \
      python3-rosdep \
      sudo \
    && rm -rf /var/lib/apt/lists/*
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && usermod -aG input,dialout $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc
USER $USERNAME

FROM base AS runtime
# Source-tree layout lets base_station.launch.py find joy_teleop.yaml without a colcon build
COPY src/storm_basestation /ws/src/storm_basestation
COPY src/storm_teleop/config/joy_teleop.yaml /ws/src/storm_teleop/config/joy_teleop.yaml
WORKDIR /ws
CMD ["ros2", "launch", "/ws/src/storm_basestation/launch/base_station.launch.py"]
