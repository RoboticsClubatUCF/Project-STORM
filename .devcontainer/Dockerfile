FROM osrf/ros:humble-desktop-full

RUN export DEBIAN_FRONTEND=noninteractive && apt-get update && apt-get -y --no-install-recommends install clangd clang git make

# SparkCan install
RUN git clone https://github.com/grayson-arendt/sparkcan.git

RUN cd sparkcan && cmake -S . -B ./build && cmake --build ./build
RUN cd sparkcan/build && make install

# JetsonGPIO install
RUN git clone https://github.com/pjueon/JetsonGPIO.git

RUN cd JetsonGPIO && cmake -S . -B ./build && cmake --build ./build
RUN cd JetsonGPIO/build && make install

# spdlog install
RUN git clone https://github.com/gabime/spdlog.git
RUN cd spdlog && cmake -S . -B ./build && cmake --build ./build
RUN cd spdlog/build && make install

# Inject ros startup script into default bash config
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc