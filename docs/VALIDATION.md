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
