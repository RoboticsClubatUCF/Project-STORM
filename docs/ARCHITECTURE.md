# Architecture and interfaces

STORM runs the same six-wheel rover description with Gazebo, a mock controller
backend, or the SPARK MAX SocketCAN driver. Top-level launches compose these
pieces through the ROS package index so installed assets are discoverable.

| Package | Function and entry point |
| --- | --- |
| `storm_description` | Defines chassis, six wheel joints, and lidar mount. `description.launch.py` publishes the description and robot transforms. |
| `storm_control` | Owns the shared wheel geometry, command timeout, and controller settings. `control.launch.py` starts the standalone manager for mock or CAN hardware. |
| `storm_hardware` | Exports the optional SPARK CAN system plugin. Converts motor RPM and rotations to wheel rad/s and radians. |
| `storm_sparkcan_vendor` | Builds and installs the pinned CAN library when the physical driver is enabled. |
| `storm_teleop` | `teleop.launch.py` translates deadman joystick input to stamped velocity commands. |
| `storm_navigation` | `navigation.launch.py` manages planning, control, recovery, smoothing, waypoints, and either SLAM or AMCL. `rviz.launch.py` opens the STORM view. |
| `storm_gazebo` | `simulation.launch.py` starts Gazebo, spawns STORM, activates its controllers, and bridges simulated time and lidar. |
| `storm_bringup` | `simulation.launch.py` assembles Gazebo and optional Nav2. `robot.launch.py` assembles mock or CAN control with optional Nav2. |

| Interface | Type or transform | Owner |
| --- | --- | --- |
| `/cmd_vel_nav` | `geometry_msgs/msg/TwistStamped` | Nav2 controller or recovery behavior |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Velocity smoother during navigation or joystick during manual driving |
| `/odom` | `nav_msgs/msg/Odometry` | Differential drive controller |
| `/joint_states` | `sensor_msgs/msg/JointState` | Joint state broadcaster |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo bridge or physical lidar driver |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo bridge in simulation |
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM Toolbox in mapping mode or map server in localization mode |
| `map → odom` | Dynamic TF | SLAM Toolbox or AMCL, according to launch mode |
| `odom → base_footprint` | Dynamic TF | Differential drive controller |
| `base_footprint → base_link → wheels / lidar_link` | Fixed or joint TF | Robot state publisher |

Gazebo owns its controller manager in simulation. Mock and CAN launches use a
standalone manager. Use one command producer at a time. The drive controller
times out after 0.5 seconds, and the velocity smoother times out after 0.3 seconds.
The lidar mount in the description is provisional for physical hardware.

Arena navigation uses a static map layer. The MDRS profile uses a rolling
global costmap for local exploration. See [terrain details](URC_TERRAIN.md)
for the limits of outdoor localization and traversability.
