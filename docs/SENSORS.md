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
| ZED RGB-D | Confirm the camera model and whether the repeated inventory entry denotes a second camera. Match the ZED SDK and ROS wrapper to the Jetson image. |
| ArduCam 5 MP RPi 3 | Confirm sensor module, connector, carrier board, and supported Jetson driver before wiring. |
| NEO v1.1 encoders | Already represented by SPARK CAN position and velocity feedback. Verify signs and scale against the wheels on the bench. |

Vendor references: [SPATIAL ROS 2 driver](https://github.com/advanced-navigation/ros2-driver)
and [ZED ROS 2 integration](https://docs.stereolabs.com/docs/integrations/ros-2).

## Gazebo sensor suite

The arena and MDRS presets both include the following simulated interfaces.
The mounts and sensor models are representative and use simulation time.

| ROS topic | Simulation model |
| --- | --- |
| `/scan` | Existing planar lidar used by Nav2 and SLAM, also suitable for testing planar Hokuyo consumers |
| `/velodyne_points` | VLP-16-style 16-ring point cloud at 10 Hz |
| `/imu/data`, `/gps/fix` | SPATIAL-style IMU and GNSS outputs at 50 Hz and 5 Hz |
| `/camera/image_raw`, `/camera/depth/image_raw`, `/camera/camera_info` | Shared 640×360 RGB-D proxy at 10 Hz for color and depth consumers |
| `/joint_states`, `/odom` | Wheel feedback from Gazebo ros2_control |

The cloud and planar scan are generated separately. The camera proxy supports
testing image pipelines without modeling each camera's optics, ZED stereo
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
ground. The RViz depth pane uses a fixed 0.2–20 m display scale. Pixels with
no return can remain dark.

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
