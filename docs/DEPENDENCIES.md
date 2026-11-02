# Dependencies and setup

Build and run on Ubuntu 24.04 with ROS 2 Jazzy. On Windows, use the Ubuntu-24.04
WSL distribution and build in its Linux checkout. Gazebo Harmonic provides the
simulator. WSLg is needed for graphical windows under WSL.

| Dependency group | Purpose | Required by |
| --- | --- | --- |
| ament_cmake, colcon, rosdep | Build, install, and resolve package dependencies | Workspace |
| launch, launch_ros, ament_index_python, xacro, robot_state_publisher | Launch composition and rover description | Description and bringup |
| ros2_control, controller_manager, diff_drive_controller, joint_state_broadcaster | Wheel control and feedback | All backends |
| Nav2 servers and plugins, SLAM Toolbox, RViz | Navigation, mapping, localization, and visualization | Navigation |
| ros_gz_sim, ros_gz_bridge, gz_ros2_control | Simulation and ROS bridges | Gazebo |
| joy, teleop_twist_joy | Manual joystick driving | Teleoperation |
| sparkcan, SocketCAN, SPARK MAX controllers | Physical CAN backend | Optional hardware build and bench use |
| ament_cmake_pytest, pytest, PyYAML, ament_cmake_gtest | Package tests | Test builds |
| NumPy and Pillow | Regenerating MDRS assets | Terrain generation only |

Each package's `package.xml` declares its ROS dependencies. Install ROS 2 Jazzy
and the colcon and rosdep tools first. Run `sudo rosdep init` once on a fresh
system, then run the following in the repository:

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
rosdep check --from-paths src --ignore-src --rosdistro jazzy
bash devscripts/build.sh
source install/setup.bash
```

Use `bash devscripts/build_hardware.sh` in a fresh shell to enable the CAN
plugin. The script fetches the pinned sparkcan revision and installs it into
the overlay. Source `install/setup.bash` after building. Physical operation
requires the robot's actual SocketCAN settings and a separate lidar driver.
See the [hardware workflow](../README.md#physical-rover-integration) and
[third-party notices](../THIRD_PARTY.md).
