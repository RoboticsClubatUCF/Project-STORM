FROM osrf/ros:jazzy-desktop-full

ENV ROS_DISTRO=$ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get -y --no-install-recommends install clangd clang git make
RUN git clone https://github.com/grayson-arendt/sparkcan.git

RUN cd sparkcan && cmake -S . -B ./build && cmake --build ./build
RUN cd sparkcan/build && make install

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc