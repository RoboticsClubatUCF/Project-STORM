# storm_teleop

ROS 2 teleop stack for a 6-wheel skid-steer URC rover:

```
Xbox joystick → joy_node → teleop_twist_joy → /cmd_vel
    → diff_drive_controller (ros2_control, 50 Hz)
    → SparkCanHardware plugin → sparkcan / SocketCAN (can0)
    → 6× REV SPARK MAX (onboard velocity PID) → NEO V1.1 motors
```

Each SPARK MAX runs its own onboard velocity PID, so the plugin sends velocity
setpoints (not duty cycle) and the control loop stays fast regardless of the
50 Hz ros2_control rate.

## Dependencies

- ROS 2 (ament_cmake) with `ros2_control`, `diff_drive_controller`,
  `joint_state_broadcaster`, `joy`, `teleop_twist_joy`,
  `robot_state_publisher`, `xacro`
- [`sparkcan`](https://github.com/grayson-arendt/sparkcan): installed as a
  system library (headers in `/usr/local/include`, `libsparkcan.so` in
  `/usr/local/lib`)
- `yaml-cpp`: used by the plugin to parse `config/sparks.yaml`

## Hardware / CAN setup

The 6 SPARK MAXs are addressed by CAN ID (set in `description/rover.urdf.xacro`):

| CAN ID | Joint                    | Side  | Inverted |
|--------|--------------------------|-------|----------|
| 1      | front_left_wheel_joint   | left  | yes      |
| 2      | middle_left_wheel_joint  | left  | yes      |
| 3      | back_left_wheel_joint    | left  | yes      |
| 4      | back_right_wheel_joint   | right | no       |
| 5      | middle_right_wheel_joint | right | no       |
| 6      | front_right_wheel_joint  | right | no       |

### Bring up `can0`

Use the provided script (idempotent; sets `restart-ms` for automatic bus-off
recovery). Adjust bitrate to your bus via env vars:

```bash
sudo scripts/can0_up.sh                       # can0 @ 1 Mbit/s
sudo IFACE=can0 BITRATE=500000 scripts/can0_up.sh
```

To bring `can0` up automatically at boot, install the systemd unit:

```bash
sudo cp systemd/storm-can0.service /etc/systemd/system/
sudo cp scripts/can0_up.sh /usr/local/bin/can0_up.sh
sudo chmod +x /usr/local/bin/can0_up.sh
sudo systemctl daemon-reload
sudo systemctl enable --now storm-can0.service
```

Verify: `ip -details link show can0` → shows `UP`, the bitrate, and `restart-ms`.

### Bluetooth controller failsafe

A Bluetooth controller that dies (battery, range) stays connected for up to
20 s, and joy_node keeps sending its last input, so the rover keeps driving.
Install the udev rule so the link drops after 1 s instead:

```bash
sudo apt install bluez                        # provides hcitool
sudo cp scripts/bt_joy_failsafe.sh /usr/local/bin/bt_joy_failsafe.sh
sudo chmod +x /usr/local/bin/bt_joy_failsafe.sh
sudo cp udev/99-storm-bt-joy.rules /etc/udev/rules.d/
sudo udevadm control --reload
```

Reconnect the controller, then check `journalctl -t bt_joy_failsafe` for
`link supervision timeout 1000 ms`.

### Controller watchdog

`joy_watchdog` sits between `joy_node` (`/joy/raw`) and `teleop_twist_joy`
(`/joy`). DS4/DualSense controllers stream motion data nonstop, so when it
goes quiet for 250 ms the watchdog sends a released-deadman `/joy` and blocks
input until the controller is back. It runs wherever `joy_node` runs (rover or
base station). Controllers without a motion sensor pass straight through with
a warning. The user running the launch needs to be in the `input` group:

```bash
sudo usermod -aG input $USER    # log out and back in
```

## Build

```bash
cd ~/ros2_ws          # your colcon workspace
colcon build --packages-select storm_teleop
source install/setup.bash
```

## Run

```bash
ros2 launch storm_teleop teleop.launch.py
```

This starts `robot_state_publisher`, `ros2_control_node`, the
`diff_drive_controller` + `joint_state_broadcaster` spawners, `joy_node`,
`teleop_twist_joy`, the [storm_telemetry](../storm_telemetry) nodes and the
foxglove_bridge from [storm_basestation](../storm_basestation).

Launch args:
- `use_joy` (default `true`)
- `enable_telemetry` (default `true`)
- `enable_bridge` (default `true`)
- `can_interface` (default `can0`, used by the plugin and can_health_node)
- `joy_overrides` (optional YAML layered over `joy_teleop.yaml`)

### Full-speed turbo (SpaceU)

```bash
ros2 launch storm_teleop teleop-spaceu.launch.py   # same args as teleop.launch.py
```

Default turbo is 3.5 m/s / 4.4 rad/s, sized so full stick on both axes stays
under the 5.4 m/s wheel ceiling. The SpaceU launch raises turbo to the hard
ceilings (5.4 m/s / 12.5 rad/s) via `config/joy_teleop_spaceu.yaml`.
Acceleration limits are the same in both. At full SpaceU turbo, combined
forward + turn saturates the motors and arcs come out wider than commanded.

### Driving from a base station

```bash
export ROS_DOMAIN_ID=42   # same on both, not 0

# rover
ros2 launch storm_teleop teleop.launch.py use_joy:=false

# laptop (no build needed)
ros2 launch /path/to/src/storm_basestation/launch/base_station.launch.py
```

`/joy` goes over DDS; Lichtblick connects to the bridge separately (see
[storm_basestation](../storm_basestation)). If WiFi drops, `cmd_vel_timeout` stops the
rover after 0.5 s.

Check the link on the rover with `ros2 topic hz /joy` (~20 Hz). If nothing
shows up, multicast is probably blocked; set up unicast peers.

## Controller mapping (wired Xbox, xpad driver)

- **Hold A**: deadman/enable (release = ramp to stop at accel limits)
- **Left stick Y**: forward/back
- **Left stick X**: turn
- **A + RB**: turbo

Verify your pad's axis/button numbers with:

```bash
ros2 run joy joy_node
ros2 topic echo /joy
```

## Configuration

| File                        | What it configures |
|-----------------------------|--------------------|
| `config/sparks.yaml`        | SPARK MAX onboard velocity **PID gains** (`kp/ki/kd/kf`). Single source of truth for tuning. |
| `config/controllers.yaml`   | `diff_drive_controller` kinematics, velocity/accel limits, odometry, `cmd_vel_timeout`. |
| `config/joy_teleop.yaml`    | Joystick axes/buttons, speed scales, deadman/turbo. |
| `config/joy_teleop_spaceu.yaml` | Full-speed turbo overrides used by `teleop-spaceu.launch.py`. |
| `description/rover.urdf.xacro` | Robot model + `<ros2_control>` block (CAN IDs, inversion, `gear_ratio`, `burn_flash`, `require_all_sparks`, `spark_config` path, `can_interface` from the launch arg). |

### PID gains (`config/sparks.yaml`)

The launch file resolves this file's absolute path and hands it to the plugin
via the URDF `spark_config` param; the plugin parses it in `on_init`.

- Precedence: `sparks.yaml` value → built-in default in
  `spark_can_hardware.hpp`. A missing file/key is non-fatal; the plugin warns
  and uses the default so the robot still comes up.
- `kF` (feedforward) does most of the work: `output ≈ kF × setpoint_rpm`,
  with `kF ≈ 1 / free_speed_RPM ≈ 1/5676` for NEO V1.1. Tune on the real robot.

### Persisting config to the SPARKs (`burn_flash`)

`<param name="burn_flash">` in the URDF defaults to `false`. The plugin
re-applies the full config (motor type, brake mode, PID, …) over CAN on every
launch, so flashing each boot only wears the limited flash. Set it to `true`
for a **single** launch to commit a persistent baseline to the devices, then
set it back to `false`.

### Pre-flight SPARK gate (`require_all_sparks`)

`<param name="require_all_sparks">` in the URDF defaults to `true`. During
`on_configure` the plugin waits for periodic status frames and checks bus
voltage for each of the 6 SPARKs. If any are missing and this is `true`, the
configure transition **fails** and the controller never activates, so the
rover won't drive with a dead wheel. Set it to `false` in the URDF to allow
coming up with fewer than all six (limp mode). It is not a launch arg.

## Telemetry

CAN health, computer stats, battery voltage and per-SPARK status live in
[storm_telemetry](../storm_telemetry). The plugin exposes `voltage` and
`online` state interfaces on every wheel joint, which
`joint_state_broadcaster` publishes on `/dynamic_joint_states`.

## SPARK status and dropouts

The plugin reads SPARK status (velocity, position, voltage) from its own
listen-only CAN socket, not sparkcan's getters. sparkcan shares one socket
across all six `SparkMax` objects and each one's thread reads from it, so each
frame reaches a random thread and is thrown away if it's for another device.
State through sparkcan only updated a fraction of the time. sparkcan is still
used for all sending.

- A SPARK with no status frame for `status_timeout_ms` (URDF, default 100 ms)
  is **offline**. The rover keeps driving on the rest (limp mode), its
  velocity reads 0 and its last position is held.
- When it comes back, or reports a reset (sticky `HasReset` fault, e.g. after
  a brownout), the plugin re-applies its config before sending setpoints. A
  rebooted SPARK only has its flash config, which may not match (inversion,
  PID), unless `burn_flash` was used.
- Odometry is less accurate while limping: a held position still counts in
  that side's average.
- `require_all_sparks` still only gates startup.
