# Jazzy validation

Tested on November 2, 2026 in Ubuntu 24.04 under WSL with ROS 2 Jazzy and
Gazebo Harmonic. Each runtime test used its own ROS domain and Gazebo partition.

| Check | Result |
| --- | --- |
| rosdep check | All system dependencies satisfied |
| Default colcon build | Eight packages built |
| Optional CAN build | Pinned sparkcan vendor and Jazzy plugin compiled |
| Package tests | Twelve colcon test records, zero failures |
| Mock base | Both controllers active, stamped velocity subscription, odometry in base_footprint |
| Arena SLAM | NavigateToPose to (1.5, 0) succeeded |
| Arena saved map | AMCL initial pose accepted and NavigateToPose to (1, 0) succeeded |
| MDRS terrain | Both controllers active and NavigateToPose to (-1, 1) succeeded |

Runtime goals use map-frame coordinates. Physical CAN activation and driving
remain to be tested on the bench. This revision did not remeasure outdoor
localization accuracy. The earlier terrain test measured roughly 0.7 m of
position error against Gazebo ground truth.

## Sensor and visualization follow-up

- Jazzy built `storm_sensors` and the updated bringup. Launch checks rejected
  invalid connection settings and a missing integrated VLP-16 mount.
- The headless MDRS test received the planar scan, VLP-16 cloud, IMU, GNSS,
  RGB image, depth image, camera info, and joint states.
- A depth sample after tilting the camera contained 178,575 finite values
  out of 230,400 pixels, with valid ranges from 0.54 m to 19.63 m.
- In `truth_pose` mode, a short Nav2 goal succeeded and ROS odometry
  matched the horizontal Gazebo pose afterward.

These checks used WSL and Gazebo. Physical lidar and Jetson deployment still
need bench testing. Ground-truth mode bypasses the wheel/SLAM estimate.
