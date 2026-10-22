# ROSMASTER X3 Jazzy guide

This guide follows Automatic Addison's ROS 2 Jazzy tutorial sequence and maps each tutorial to the package and command in this repository. The repository is a multi-package workspace: clone it as one directory under `ros2_ws/src`, then build from the workspace root.

## Learning path

1. [Install and build](00-setup-and-build.md): Ubuntu 24.04, ROS 2 Jazzy, workspace setup, dependencies, and build.
2. [Robot description](01-robot-description.md): Xacro, meshes, TF, and RViz.
3. [Gazebo simulation](02-gazebo-simulation.md): launch the ROSMASTER X3 and its ROS 2 control stack.
4. [Localization and mapping](03-localization-and-slam.md): EKF configuration, SLAM Toolbox, and saving a map.
5. [Nav2 navigation](04-nav2-navigation.md): load a saved map, set goals, and tune the controller.
6. [ArUco docking](05-docking.md): launch the detector and configure the dock pose.
7. [Development and troubleshooting](06-development-and-troubleshooting.md): build individual packages and inspect common ROS 2 issues.
8. [Project Bash aliases](08-project-aliases.md): load short commands for builds, launches, and diagnostics.\n8. [Start a custom robot project](07-custom-robot-project.md): reuse this stack's elements for a different robot design.

## Tutorial alignment

The package names, source files, and launch flow are kept close to the [Jazzy branch of the reference repository](https://github.com/automaticaddison/yahboom_rosmaster/tree/jazzy). The documentation links to the corresponding [Jazzy tutorials](https://automaticaddison.com/tutorials/) instead of copying their prose. Local configuration remains robot-specific: for example, the Nav2 parameters and mecanum controller settings are tuned for the ROSMASTER X3.

## Repository packages

- `yahboom_rosmaster_description`: Xacro robot model, meshes, and description launch.
- `yahboom_rosmaster_gazebo`: Gazebo worlds, models, bridge configuration, and simulation launch.
- `mecanum_drive_controller`: ros2_control controller for the four mecanum wheels.
- `yahboom_rosmaster_bringup`: combined robot, simulation, localization, and navigation launch files.
- `yahboom_rosmaster_localization`: robot_localization EKF configuration.
- `yahboom_rosmaster_navigation`: Nav2 parameters, maps, goal scripts, and velocity relay.
- `yahboom_rosmaster_docking`: ArUco pose publisher and dock configuration.
- `yahboom_rosmaster_system_tests`: sample controller executable and system-test package scaffold.
