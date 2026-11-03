# Project STORM — ROS 2 Jazzy

| Guide | Use it to |
| --- | --- |
| [Architecture and interfaces](docs/ARCHITECTURE.md) | Trace commands, sensor data, and transforms through the rover |
| [Dependencies and setup](docs/DEPENDENCIES.md) | Prepare a Jazzy workspace and choose a hardware backend |
| [Adding components](docs/EXTENDING.md) | Add a ROS package, sensor, controller, or test world |
| [Running tests](docs/TESTING.md) | Run package tests and reproduce simulation checks |
| [Physical sensor bringup](docs/SENSORS.md) | Bench-test the VLP-16 or Hokuyo and check Jetson compatibility |
| [MDRS terrain](docs/URC_TERRAIN.md) | Inspect the terrain source, coordinates, and limitations |
| [Validation results](docs/VALIDATION.md) | See what has already been exercised |

A local Jazzy development version of RoboticsClubatUCF/Project-STORM. The rover
geometry and CAN ID layout come from STORM. The simulation includes a STORM test
arena and the previously requested MDRS terrain.

Target: **Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic**. Build and run inside Linux;
the Windows checkout is a source/review copy. The original repository is preserved.

## Package layout

| Package | Responsibility |
| --- | --- |
| `storm_description` | Original rover geometry, sensor mounting frame, selectable ros2_control backend |
| `storm_hardware` | Jazzy SPARK MAX CAN SystemInterface, hardware validation tests |
| `storm_sparkcan_vendor` | Essential third-party CAN library, pinned upstream revision |
| `storm_control` | Shared six-wheel controller configuration and mock/CAN manager launch |
| `storm_teleop` | Deadman joystick, stamped velocity output |
| `storm_sensors` | Physical VLP-16 and serial Hokuyo bench launch |
| `storm_navigation` | Nav2, lifecycle management, SLAM, AMCL, maps, STORM RViz view |
| `storm_gazebo` | Gazebo plugins, bridge, arena, optional real MDRS terrain |
| `storm_bringup` | Application launch files combining the above |

## Install and build

Install ROS 2 Jazzy on Ubuntu 24.04 first. Initialize rosdep once if needed with
`sudo rosdep init`, then `rosdep update`. In this workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
bash devscripts/build.sh
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

The default build includes mock hardware and simulation. The physical CAN plugin
is opt-in so normal simulation builds do not download or require a CAN library.
To build it, use `bash devscripts/build_hardware.sh`, then source the workspace
again. This builds the essential vendor dependency at the recorded Git revision
and installs its library into the ROS overlay; no global library installation is
needed. Run `colcon test --packages-select storm_hardware` after enabling it.

## Gazebo + Nav2

```bash
ros2 launch storm_bringup simulation.launch.py
# Headless:
ros2 launch storm_bringup simulation.launch.py gui:=false rviz:=false
# Real MDRS terrain from the previous URC work:
ros2 launch storm_bringup simulation.launch.py environment:=urc
```

SLAM mapping starts by default. In RViz choose **Nav2 Goal** in observed free
space. Nav2 supports NavigateToPose, NavigateThroughPoses, and waypoint following.
Stop the current launch before starting another instance. ROS_DOMAIN_ID and
GZ_PARTITION are deliberately not hardcoded; set them consistently in every
terminal when isolating robots or running independent simulations.

For the supplied arena map, stop mapping and launch localization:

```bash
ros2 launch storm_bringup simulation.launch.py mode:=localization \
  map:=$(ros2 pkg prefix --share storm_navigation)/maps/storm_arena.yaml
```

Set **2D Pose Estimate** at the robot's spawn pose in RViz before issuing goals.
AMCL does not silently assume a physical robot's initial pose. Save a new map:

```bash
ros2 run nav2_map_server map_saver_cli -f "$HOME/storm_map" --ros-args -p use_sim_time:=true
```

## Manual driving and mock hardware

```bash
# Simulator only, without competing autonomous velocity publishers:
ros2 launch storm_bringup simulation.launch.py navigation:=false
# Another sourced terminal:
ros2 launch storm_teleop teleop.launch.py use_sim_time:=true
# Or a supported keyboard driver with stamped output:
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p use_sim_time:=true

# Controller/description testing without Gazebo or CAN:
ros2 launch storm_bringup robot.launch.py hardware:=mock
```

Only one command producer should be active. Teleoperation and navigation are
separate launches; simultaneous operation requires a configured command arbiter.

## Physical rover integration

After building the CAN plugin, configure the actual SocketCAN interface using
the robot's verified bitrate. This repository does not invent a CAN bitrate or
change a host network interface. Then:

```bash
ros2 launch storm_bringup robot.launch.py hardware:=can can_interface:=can0
```

Use the integrated VLP-16 launch in the [sensor guide](docs/SENSORS.md#vlp-16-with-nav2),
or start a separate lidar driver. The driver must publish `sensor_msgs/LaserScan`
on `scan` with a calibrated transform into `base_link`. The model's `lidar_link`
mount is provisional and must match the sensor. Start navigation only when the
scan, wheel feedback, and transforms are verified:

```bash
ros2 launch storm_navigation navigation.launch.py mode:=slam use_sim_time:=false
```

The CAN plugin is ported and compile-tested against installed Jazzy APIs, but no
physical motors are exercised in software validation. Startup rejects missing
motors; lifecycle stops and non-finite data errors command zero output. Motor
flash is not rewritten on lifecycle transitions. Encoder sign conventions,
watchdog behavior, feedback freshness, and braking still need bench qualification.

## Interfaces and architecture

SI units and REP-103 axes: x forward, y left, z up. REP-105 transform ownership:

```text
map -- SLAM Toolbox OR AMCL --> odom
odom -- diff_drive_controller --> base_footprint
base_footprint -- robot_state_publisher --> base_link --> wheels / lidar_link
```

All velocity commands use `geometry_msgs/msg/TwistStamped`:

```text
Nav2 controller/behaviors -> cmd_vel_nav -> velocity_smoother -> cmd_vel
                                                           -> diff_drive_controller
```

The diff-drive controller publishes `odom` and times out after 0.5 seconds.
The smoother stops on stale navigation input after 0.3 seconds. Gazebo owns the
only controller manager in simulation. Mock/CAN bringup owns one standalone
manager instead. Simulation uses Gazebo `clock`; physical and mock bringup default
to wall time. Nav2 and simulation packages do not depend on the physical driver.

The STORM footprint includes all six wheels. Arena navigation uses a static
SLAM/map layer; URC uses a rolling global costmap with unknown-space tracking.
The MDRS import is bare-earth venue terrain, not an official mission course.
It lacks rocks, soil deformation, and suspension dynamics. Existing outdoor
tests showed localization drift; 2D Nav2 costmaps do not assess tip-over risk.
See `docs/URC_TERRAIN.md`, terrain provenance, and `docs/VALIDATION.md`.

## Development

Use `params_file:=/path/config.yaml` to supply navigation tuning. Launch files
validate missing map/world paths. Common description code does not import Gazebo.
Custom Nav2 BT plugins can be added as new packages and configured through Nav2;
the current implementation intentionally uses the installed standard BTs.

Maintainer email addresses retain the upstream placeholder and should be replaced
by the team's contact before publishing. No upstream repository has been pushed.

References: https://docs.ros.org/en/jazzy/ and https://docs.nav2.org/jazzy/ and
https://control.ros.org/jazzy/
