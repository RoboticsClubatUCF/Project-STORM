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
| velodyne_driver, velodyne_pointcloud, velodyne_laserscan, urg_node | Physical lidar drivers and VLP-16 scan conversion | Sensor package |
| sparkcan, SocketCAN, SPARK MAX controllers | Physical CAN backend | Optional hardware build and bench use |
| ament_cmake_pytest, pytest, PyYAML, ament_cmake_gtest | Package tests | Test builds |
| NumPy, Pillow, pyproj | Terrain planning, geographic conversion, dashboard relief/depth processing | Mission and dashboard packages, terrain generation |
| ament_cmake_python, Python standard HTTP/subprocess libraries | Installed Python modules and local command API | Mission and dashboard packages |
| lifecycle_msgs, controller_manager_msgs | Read Nav2 lifecycle and drive-controller readiness | Dashboard |

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
requires the robot's actual SocketCAN settings and a configured lidar.
See the [hardware workflow](../README.md#physical-rover-integration) and
[third-party notices](../THIRD_PARTY.md).
