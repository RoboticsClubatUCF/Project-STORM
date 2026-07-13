FROM osrf/ros:humble-desktop-full

RUN export DEBIAN_FRONTEND=noninteractive && apt-get update && apt-get -y --no-install-recommends install clangd clang git make
RUN git clone https://github.com/grayson-arendt/sparkcan.git

RUN cd sparkcan && cmake -S . -B ./build && cmake --build ./build
RUN cd sparkcan/build && make install

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc