# Start a custom robot project from this stack

Use this repository as a reference for the ROS 2 package layout and integration points, then put the new robot in its own repository and workspace. This keeps the new robot independent from the ROSMASTER X3 assets and avoids package-name collisions when both workspaces are sourced.

This workflow follows the Jazzy tutorials for a [mobile robot description](https://automaticaddison.com/create-and-visualize-a-mobile-robot-with-urdf-ros-2-jazzy/), [Gazebo simulation](https://automaticaddison.com/how-to-simulate-a-mobile-robot-in-gazebo-ros-2-jazzy/), [localization and SLAM](https://automaticaddison.com/sensor-fusion-and-robot-localization-using-ros-2-jazzy/), and [Nav2](https://automaticaddison.com/autonomous-navigation-for-a-mobile-robot-using-ros-2-jazzy/).

## 1. Create a separate workspace

Start with a fresh workspace so you can build the new robot without the ROSMASTER packages being present in the same overlay:

```bash
mkdir -p ~/my_robot_ws/src
cd ~/my_robot_ws/src
git clone --branch codex/jazzy-tutorial-alignment \
  https://github.com/Tyler-P-Thomas/yahboom_rosmaster.git my_robot
cd my_robot
git switch -c custom/my-robot
```

Rename the GitHub repository after creating your own copy or fork. Keep the tutorial/reference repository as an upstream remote if you want to track its changes.

## 2. Choose a package layout

For a small mobile robot, use a focused package set:

| Package | Reuse from this project | Replace for the new robot |
| --- | --- | --- |
| `my_robot_description` | Xacro decomposition, launch pattern, install layout | Links, joints, meshes, inertial values, sensor frames |
| `my_robot_gazebo` | World/launch/bridge organization | Robot spawn, sensor topics, simulation plugins and model assets |
| `my_robot_bringup` | Launch composition and argument pattern | Package names, launch defaults, robot-specific options |
| `my_robot_localization` | EKF file and launch structure | Input topics, frames, sensor fields, covariance values |
| `my_robot_navigation` | Nav2 parameter organization and goal examples | Footprint, kinematics, velocity limits, topics, maps |
| `my_robot_docking` | Optional ArUco detection and docking pattern | Omit if docking is not a requirement; otherwise replace camera/tag transforms |
| `mecanum_drive_controller` | Keep only for a four-wheel mecanum base | For differential drive, use a differential-drive controller and matching joint configuration |

The `yahboom_rosmaster` metapackage is optional. Keep a metapackage if you want one package to depend on the full robot stack; otherwise remove it and build the individual packages directly.

## 3. Rename package identities before changing robot geometry

First inventory references to the original package and robot names:

```bash
rg -n "yahboom_rosmaster|rosmaster_x3|mecanum_drive_controller" .
```

For each package you keep, update the package name consistently in its `package.xml`, `project(...)` declaration in `CMakeLists.txt`, Python package metadata where applicable, launch files, Xacro package lookups, and dependency declarations. Update package-share lookups such as `FindPackageShare` and `$(find ...)` at the same time.

Use one naming scheme throughout, for example `my_robot_description`, `my_robot_gazebo`, and `my_robot_bringup`. Do not rename frame IDs or mesh filenames in bulk: those names may be referenced by sensors, controller configuration, RViz, Gazebo, and Nav2.

## 4. Replace the robot-specific model

In `my_robot_description`:

1. Replace the top-level Xacro and the included base, wheel, and sensor Xacros.
2. Add the new visual meshes and check every mesh URI resolves from the installed package.
3. Define collision and inertial geometry separately from visual meshes.
4. Preserve standard frame names where possible: `base_link`, `odom`, and `map`. Add `base_footprint` only if the model and navigation configuration use it.
5. Update the ros2_control Xacro and controller YAML together. List the exact command/state interfaces and joint names used by the hardware or simulator.
6. Update the RViz configuration's fixed frame and robot description topic.

Visualize the model using the description launch before enabling Gazebo or Nav2:

```bash
ros2 launch my_robot_description robot_state_publisher.launch.py
```

## 5. Match simulation and drive control to the chassis

In `my_robot_gazebo`, update the robot spawn description, control plugin, sensor plugins, and bridge configuration so they publish the topics used by the ROS nodes. Keep the simulator's joint names and command interfaces identical to the ros2_control configuration.

For mecanum wheels, tune wheel radius, wheel separation, wheel offsets, and controller geometry from the new robot's measurements. For differential drive, use a differential drive controller and remove mecanum-specific parameters and lateral velocity assumptions.

Launch simulation by itself first. Verify the robot appears in Gazebo, wheel joints are available, lidar/camera topics exist, and the base moves with a low command velocity before adding localization or navigation.

## 6. Reconfigure localization and navigation

In `my_robot_localization`, set the EKF input topics, frame IDs, and enabled state variables to match the sensors that are actually present. Check that the transform chain is connected and does not have two nodes publishing the same transform.

In `my_robot_navigation`:

- Set the robot footprint or radius from the new chassis dimensions.
- Set controller plugin and velocity/acceleration limits to match the drive base.
- Point costmaps at the real lidar or depth topics and their correct message types.
- Match `global_frame`, `odom_frame`, and `robot_base_frame` to the TF tree.
- Create a new map for the new environment; do not reuse the cafe/house maps from this repository.
- Set `use_sim_time` consistently across simulation, localization, SLAM, and Nav2.

Follow the [SLAM guide](03-localization-and-slam.md) to create a map, then test Nav2 with that map. Add docking only after navigation and camera transforms work.

## 7. Build and verify in stages

From `~/my_robot_ws`:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash
```

Build after each stage rather than changing every subsystem at once:

1. `my_robot_description`: build and view the model in RViz.
2. `my_robot_gazebo`: simulate and verify sensor topics and wheel control.
3. `my_robot_localization`: verify odometry and TF.
4. `my_robot_navigation`: create/load a map and send a Nav2 goal.
5. `my_robot_docking`: add only if ArUco docking is required.

Useful checks include:

```bash
ros2 pkg list | grep my_robot
ros2 topic list
ros2 control list_controllers
ros2 run tf2_tools view_frames
ros2 doctor --report
```

Update the root README and a `docs/` guide as each stage becomes runnable. Document the robot's sensor model, frame tree, topics, measured dimensions, controller type, and launch commands; those are the values a new user must change for a different design.
