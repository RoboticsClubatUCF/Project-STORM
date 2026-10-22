# Gazebo simulation

This guide follows Automatic Addison's [mobile robot Gazebo tutorial for Jazzy](https://automaticaddison.com/how-to-simulate-a-mobile-robot-in-gazebo-ros-2-jazzy/). The simulation package uses Gazebo Sim and the `ros_gz` bridge; the controller package provides mecanum wheel control through ros2_control.

## Launch the simulator

Build and source the workspace, then run:

```bash
ros2 launch yahboom_rosmaster_gazebo yahboom_rosmaster.gazebo.launch.py
```

To choose one of the included worlds:

```bash
ros2 launch yahboom_rosmaster_gazebo yahboom_rosmaster.gazebo.launch.py world_file:=cafe.world
ros2 launch yahboom_rosmaster_gazebo yahboom_rosmaster.gazebo.launch.py world_file:=house.world
```

The launch file exposes arguments for the world, initial pose, simulated time, RViz, robot state publisher, and controller loading. Use `ros2 launch yahboom_rosmaster_gazebo yahboom_rosmaster.gazebo.launch.py --show-args` to see the current argument descriptions.

## Drive the simulated robot

The combined bringup script starts the robot with Nav2 and Gazebo. Use the optional `slam` argument to enable online mapping:

```bash
ros2 launch yahboom_rosmaster_bringup rosmaster_x3_navigation.launch.py slam:=True world_file:=cafe.world
```

For a saved map, add `slam:=False map:=/absolute/path/to/map.yaml`. The script at `yahboom_rosmaster_bringup/scripts/rosmaster_x3_navigation.sh` is an optional wrapper; call it with `bash` and pass `slam` and, optionally, a map YAML path. Do not run two Gazebo launch files at the same time for the same robot.

## Inspect topics and controllers

In another sourced terminal:

```bash
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /odometry/filtered
ros2 control list_controllers
```

The exact topic set depends on the selected launch options and whether simulation or physical hardware is active.
