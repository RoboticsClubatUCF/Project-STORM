# Localization and mapping

This guide follows Automatic Addison's Jazzy tutorials for [sensor fusion and robot localization](https://automaticaddison.com/sensor-fusion-and-robot-localization-using-ros-2-jazzy/) and [SLAM](https://automaticaddison.com/building-a-map-of-the-environment-using-slam-ros-2-jazzy/).

## EKF localization

The EKF settings are in `yahboom_rosmaster_localization/config/ekf.yaml`. The launch file starts robot_localization with that configuration:

```bash
ros2 launch yahboom_rosmaster_localization ekf_gazebo.launch.py
```

The integrated bringup launch already includes the EKF. Use the same `use_sim_time` setting for the EKF, Nav2, and Gazebo. In simulation it should be enabled; on a physical robot use the system clock unless the robot publishes simulated time.

Check the localization topics and transforms:

```bash
ros2 topic echo /odometry/filtered
ros2 run tf2_tools view_frames
```

## Build a map with SLAM Toolbox

Start the combined simulation with SLAM enabled:

```bash
ros2 launch yahboom_rosmaster_bringup rosmaster_x3_navigation.launch.py slam:=True world_file:=cafe.world
```

Use RViz's 2D teleoperation tool or the assisted teleoperation node to move through the environment slowly. Keep the lidar scan visible and close loops through previously mapped areas to improve the map.

Save the map to a writable directory:

```bash
mkdir -p ~/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/rosmaster_x3_map
```

This produces a YAML metadata file and an image file. Preserve both files together; the YAML refers to the image by filename.

## Continue with a saved map

Stop the SLAM launch and start Nav2 with SLAM disabled, passing the YAML file created above:

```bash
ros2 launch yahboom_rosmaster_bringup rosmaster_x3_navigation.launch.py \
  slam:=False \
  map:=/home/$USER/ros2_ws/maps/rosmaster_x3_map.yaml \
  world_file:=cafe.world
```

Replace the path if you saved the map elsewhere. The repository also includes example maps for the cafe and house worlds.
