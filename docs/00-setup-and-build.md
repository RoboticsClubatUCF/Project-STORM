# Install, workspace, and build

This repository targets Ubuntu 24.04 and ROS 2 Jazzy, following Automatic Addison's [Jazzy installation tutorial](https://automaticaddison.com/how-to-install-ros-2-jazzy/) and [workspace tutorial](https://automaticaddison.com/how-to-create-a-workspace-in-ros-2-jazzy/).

## Prerequisites

Use Ubuntu 24.04 (a native install or Ubuntu virtual machine). Install ROS 2 Jazzy Desktop and the standard development tools from the linked ROS 2 instructions. Then make sure the command-line tools are available:

```bash
sudo apt update
sudo apt install -y git python3-rosdep python3-colcon-common-extensions
```

Initialize rosdep once if it has not already been initialized:

```bash
sudo rosdep init
rosdep update
```

## Create the workspace and clone

The repository root contains the ROS packages, so clone it directly into the workspace's `src` directory:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Tyler-P-Thomas/yahboom_rosmaster.git
cd ~/ros2_ws
```

Source the Jazzy environment in each new terminal:

```bash
source /opt/ros/jazzy/setup.bash
```

Install the dependencies declared by the packages:

```bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
```

Build and source the overlay:

```bash
colcon build --symlink-install
source install/setup.bash
```

After rebuilding, source `install/setup.bash` again in every terminal that will use the workspace. To verify package discovery:

```bash
ros2 pkg list | grep yahboom_rosmaster
```

## Build one package and its dependencies

From `~/ros2_ws`, build a selected package and any workspace packages it depends on:

```bash
colcon build --symlink-install --packages-up-to yahboom_rosmaster_navigation
source install/setup.bash
```

Use the package-specific commands in the remaining guides only after the workspace has built and been sourced.
