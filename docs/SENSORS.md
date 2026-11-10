# Physical sensor bringup

STORM's onboard computer is the NVIDIA Jetson Orin Nano Developer Kit linked
in the lab inventory. Check the installed JetPack/L4T image on the rover before
choosing the ROS 2 and camera SDK deployment.

```bash
tr -d '\0' </proc/device-tree/model; echo
cat /etc/nv_tegra_release
cat /etc/os-release
uname -m
```

The current workspace targets Jazzy on Ubuntu 24.04. JetPack 6 uses Ubuntu
22.04. NVIDIA documents a JetPack 7 path for Orin Nano. The original Jetson Nano
is a different board. Match the deployment and camera SDK to the installed
image before installing drivers. [NVIDIA Orin Nano setup](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/quick_start.html)

## First bench test

In a Jazzy shell, from the workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/storm_sensors --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --packages-select storm_sensors
source install/setup.bash
ros2 launch storm_sensors lidar.launch.py --show-args
```

Select one lidar per ROS domain. Use the real sensor address or device path:

```bash
ros2 launch storm_sensors lidar.launch.py lidar:=vlp16 device_ip:="$VLP16_IP"
# Or, for the USB/serial Hokuyo:
ros2 launch storm_sensors lidar.launch.py lidar:=hokuyo serial_port:="$HOKUYO_PORT"
```

The Jetson Ethernet interface must be reachable from the VLP-16 and receive its
configured UDP destination port. The launch defaults to UDP 2368. It uses the
upstream VLP-16 calibration by default and accepts `calibration:=/path/to/file.yaml`
for a unit-specific file. The scan is a single vertical ring, with upstream
automatic ring selection by default. Inspect the ring against real obstacles
before relying on it for outdoor navigation. The raw 3D output is
`/velodyne_points`. [Velodyne driver stack](https://github.com/ros-drivers/velodyne)

For Hokuyo, identify the exact model on the label and inspect
`/dev/serial/by-id/`. The profile assumes a USB/serial URG using the driver's
default baud rate. Confirm the model and connection on the bench.
[ROS URG driver](https://github.com/ros-drivers/urg_node)

```bash
ros2 topic info /scan --verbose
timeout 10 ros2 topic echo /scan --once --field header
timeout 10 ros2 topic hz /scan
```

The expected frames are `vlp16_link` and `hokuyo_link`. For standalone data
checks, omit the transform. Before using Nav2, supply the measured transform
with `mount:="$LIDAR_MOUNT"` where the variable contains six space-separated
values in metres and radians, ordered x y z roll pitch yaw. Alternatively,
publish that same transform from the calibrated robot description. Use one
publisher per child frame. Confirm it with `ros2 run tf2_ros tf2_echo base_link
vlp16_link` or substitute `hokuyo_link`. Use wall time for physical bringup.
The sensor launch starts only lidar nodes and the optional static transform.

## VLP-16 with Nav2

The VLP-16 is STORM's selected physical Nav2 lidar. After verifying its scan
on the bench, stop the standalone sensor launch and exercise integrated
bringup with the mock base:

```bash
ros2 launch storm_bringup robot.launch.py hardware:=mock vlp16:=true \
  vlp16_ip:="$VLP16_IP" vlp16_mount:="$LIDAR_MOUNT" navigation:=true
```

Use the measured mount in metres and radians. The VLP-16 profile passes its
selected ring to `/scan`. Verify the scan sees relevant obstacles at the
installed height and angle before switching to `hardware:=can`.
Override `vlp16_ring`, `vlp16_udp_port`, or `vlp16_calibration` as needed.
Both integrated lidar and navigation are opt-in.

## Remaining inventory

| Sensor | Integration status |
| --- | --- |
| Advanced Navigation SPATIAL | Confirm serial or Ethernet, ANPP settings, mounting, and timing. The vendor driver documents Humble. Verify Jazzy support and standard IMU/GNSS messages before feeding a localization filter. |
| Generic RGB and Logitech 1080p | Identify each USB video device and supported modes before adding camera profiles and calibration. |
| Original ZED | Model confirmed by the rover owner. Whether the repeated inventory entry denotes a second camera remains unconfirmed. Match the ZED SDK and ROS wrapper to the Jetson image before physical bringup. |
| ArduCam 5 MP RPi 3 | Confirm sensor module, connector, carrier board, and supported Jetson driver before wiring. |
| NEO v1.1 encoders | Already represented by SPARK CAN position and velocity feedback. Verify signs and scale against the wheels on the bench. |

Vendor references: [SPATIAL ROS 2 driver](https://github.com/advanced-navigation/ros2-driver)
and [ZED ROS 2 integration](https://docs.stereolabs.com/docs/integrations/ros-2).

## Gazebo sensor suite

The arena and MDRS presets both include the following simulated interfaces.
The mounts and sensor models are representative and use simulation time.

| ROS topic | Simulation model |
| --- | --- |
| `/scan` | 720-bin projection of the VLP-16's nearest horizontal rings, not a separate simulated sensor |
| `/velodyne_points` | VLP-16-style 16-ring point cloud at 10 Hz |
| `/imu/data`, `/gps/fix` | SPATIAL-style IMU and GNSS outputs at 50 Hz and 5 Hz |
| `/camera/image_raw`, `/camera/depth/image_raw`, `/camera/camera_info` | Original-ZED-oriented 672×376 RGB-D proxy at 15 Hz and 90 degree horizontal field of view |
| `/joint_states`, `/odom` | Wheel feedback from Gazebo ros2_control |

The cloud and derived scan share one VLP-16 source. The scan uses actual samples
within 1.25 degrees of horizontal; it does not add an artificial horizontal beam.
The camera proxy supports testing image pipelines without modeling ZED stereo
processing, or CSI interface. GNSS uses a synthetic origin in the arena and
an approximate MDRS origin outdoors. The IMU and GNSS streams are available
to localization experiments. Current Nav2 bringup still uses wheel odometry
and lidar-based localization.

For visual and navigation checks on sparse terrain, add `truth_pose:=true`
to the simulation launch. Gazebo supplies `/odom` and the base transform in
this mode. Wheel odometry remains on `/wheel/odom` for comparison. The map
is anchored to odom and SLAM scan matching and loop closure are disabled.
Use the default `truth_pose:=false` to test the actual wheel/lidar estimate.
This is a simulation reference mode. Physical localization still needs tuning
and validation.

The simulated RGB-D camera points slightly downward to capture nearby
ground. The RViz depth pane uses a fixed 0.5–20 m display scale. Pixels with
no return can remain dark.

### Range limits and interpretation

The executable profiles are recorded in
`storm_gazebo/config/sensor_limits.yaml`. The VLP-16 has 16 channels over a
30 degree vertical field and a 100 m maximum range. The 1 m simulated near blind
zone is conservative. The simulation samples fewer horizontal returns than the
physical device and does not qualify reflectivity-dependent detection distance.

The original ZED proxy uses a conservative 0.5–20 m depth operating window.
Some legacy datasheet revisions advertise up to 25 m; 20 m is this project's
software cap, not a universal hardware maximum. Depth is optical-axis distance.
Stereo confidence, lighting, texture, and the ZED SDK are not simulated. RGB
rendering extends to 150 m for scene visibility, which does not extend depth
measurement range. See the manufacturer references in the profile YAML.

`storm_perception/sensor_limits` validates `/simulation/vlp16/points` and
`/simulation/camera/depth_unvalidated` before publishing the canonical sensor
topics. Out-of-range values become invalid samples, rather than fabricated
returns at the boundary. Consumers should use `/velodyne_points`, `/scan`, and
`/camera/depth/image_raw`; the `/simulation/` inputs are diagnostic raw data.
Local obstacle processing crops lidar points to a 10 m horizontal radius. A
terrain prior visible beyond that radius is not live sensing. GNSS and IMU are
not distance-ranging sensors; their errors and dynamic limits need separate
physical qualification.

To audit observed ranges in a running scene with the same ROS domain:

```bash
python3 devscripts/test_sensor_ranges.py --seconds 8 --output /tmp/storm-sensor-ranges.json
colcon test --packages-select storm_perception storm_gazebo
```

The audit checks observed bounds and the sole `/scan` publisher. It is not a
complete boundary-target, reflectivity, or occlusion qualification. See the
[kilometre trial](validation/2026-11-09-headless-1km.md) for live audit results.

```bash
ros2 launch storm_bringup simulation.launch.py environment:=urc \
  gui:=false rviz:=false navigation:=false
```

In another terminal with the same overlay and ROS domain, verify any stream:

```bash
ros2 topic echo /velodyne_points --once --field header --qos-reliability best_effort
ros2 topic echo /imu/data --once --field header --qos-reliability best_effort
ros2 topic echo /gps/fix --once --qos-reliability best_effort
ros2 topic echo /camera/depth/image_raw --once --field header --qos-reliability best_effort
```
