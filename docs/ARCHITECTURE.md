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
| `storm_sensors` | `lidar.launch.py` starts the physical VLP-16 or serial Hokuyo and can publish a measured mount transform. |
| `storm_navigation` | `navigation.launch.py` manages planning, control, recovery, smoothing, waypoints, and either SLAM or AMCL. `rviz.launch.py` opens the STORM view. |
| `storm_gazebo` | `simulation.launch.py` starts Gazebo, spawns STORM, activates its controllers, and bridges simulated time and lidar. |
| `storm_bringup` | `simulation.launch.py` assembles Gazebo and optional Nav2. `robot.launch.py` assembles mock or CAN control with optional Nav2. |
| `storm_mission` | `mission` sequences terrain routes and short Nav2 actions for WGS84 targets. Publishes `mission/route` and records per-target arrival checks. Currently requires simulated truth pose. |
| `storm_dashboard` | Loopback web/API service, shared mission planner, ROS telemetry bridge, supervised visible launches, leased manual velocity and automated test jobs. |
| `storm_perception` | Fits local ground from VLP-16 points and publishes separate ground-clearing and object-marking observations. |

| Interface | Type or transform | Owner |
| --- | --- | --- |
| `/cmd_vel_nav` | `geometry_msgs/msg/TwistStamped` | Nav2 controller or recovery behavior |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Velocity smoother during navigation or joystick during manual driving |
| `/odom` | `nav_msgs/msg/Odometry` | Differential drive controller |
| `/joint_states` | `sensor_msgs/msg/JointState` | Joint state broadcaster |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo bridge or physical lidar driver |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo bridge in simulation |
| `/simulation/ground_truth/odom` | `nav_msgs/msg/Odometry` | Independent Gazebo horizontal truth bridge, available with either pose mode |
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM Toolbox in mapping mode or map server in localization mode |
| `map → odom` | Dynamic TF | SLAM Toolbox or AMCL, according to launch mode |
| `odom → base_footprint` | Dynamic TF | Differential drive controller |
| `base_footprint → base_link → wheels / lidar_link` | Fixed or joint TF | Robot state publisher |

Gazebo owns its controller manager in simulation. Mock and CAN launches use a
standalone manager. Use one command producer at a time. The drive controller
times out after 0.5 seconds, and the velocity smoother times out after 0.3 seconds.
In optional `truth_pose` simulation mode, Gazebo supplies `/odom` and
`odom → base_footprint`. Wheel odometry is available on `/wheel/odom`, and
a static transform anchors `map` to `odom`.
The lidar mount in the description is provisional for physical hardware.

Arena navigation uses a static map layer. The MDRS profile uses a rolling
global costmap for local exploration. See [terrain details](URC_TERRAIN.md)
for the limits of outdoor localization and traversability.

The URC velocity chain adds Collision Monitor after smoothing. Dashboard
manual commands share its `/cmd_vel_checked` input, and its checked output
is `/cmd_vel`. Ground/object point clouds populate nearby obstacle layers
independently of the distant DEM route. See [local navigation](LOCAL_NAVIGATION.md).
