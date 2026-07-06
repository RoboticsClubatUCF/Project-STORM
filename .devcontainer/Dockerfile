FROM osrf/ros:humble-desktop-full

RUN export DEBIAN_FRONTEND=noninteractive && apt-get update && apt-get -y --no-install-recommends install clangd clang git make software-properties-common

# Add libgpiod headeres
RUN add-apt-repository universe && apt-get update
RUN apt-get -y --no-install-recommends install libgpiod-dev

# SparkCan install
RUN git clone https://github.com/grayson-arendt/sparkcan.git

RUN cd sparkcan && cmake -S . -B ./build && cmake --build ./build
RUN cd sparkcan/build && make install

# JetsonGPIO install
RUN git clone https://github.com/Rubberazer/JETGPIO.git

# RUN cd JETGPIO && sudo make
# RUN cd JETGPIO && sudo make install

# spdlog install
RUN git clone https://github.com/gabime/spdlog.git
RUN cd spdlog && cmake -S . -B ./build && cmake --build ./build
RUN cd spdlog/build && make install

# Inject ros startup script into default bash config
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc