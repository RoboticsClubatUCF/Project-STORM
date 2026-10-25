# storm_telemetry

Rover-side health publishers. Started by `storm_teleop/teleop.launch.py`
(`enable_telemetry:=false` to skip), or standalone:

```bash
ros2 launch storm_telemetry telemetry.launch.py can_interface:=can0
```

| Node | Topic | Type |
|------|-------|------|
| `can_health_node` | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |
| `system_monitor_node` | `/telemetry/system` | `storm_interfaces/SystemTelemetry` |
| `battery_monitor_node` | `/telemetry/battery` | `storm_interfaces/BatteryStatus` |
| `spark_monitor_node` | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |

Params live in `config/telemetry.yaml`.

## can_health_node

Reads CAN health from sysfs counters and `ip -details link show`. It never
opens a CAN socket, so it doesn't compete with the hardware plugin for the bus.

It reports link up/down, controller state (`ERROR-ACTIVE` is the normal state;
`WARNING`/`PASSIVE`/`BUS-OFF` are not) and rising rx/tx error counts. That
catches bus-level failure, not a single silent SPARK.

```bash
ros2 topic echo /diagnostics
ros2 run rqt_robot_monitor rqt_robot_monitor
```

## system_monitor_node

CPU %, load, hottest thermal zone, memory, root disk usage and uptime from
`/proc` and `/sys`, plus WiFi link quality and signal from
`/proc/net/wireless`. Anything unavailable is NaN.

## battery_monitor_node

Pack voltage as seen by the 6 SPARK MAXs. The plugin exposes each SPARK's
input voltage as a `voltage` state interface, and `joint_state_broadcaster`
publishes it on `/dynamic_joint_states`.

- Only SPARKs the plugin reports as `online` count.
- `level` uses the lowest reading, smoothed over `filter_time_constant`
  (5 s) so acceleration sag doesn't trip it. The `voltage` fields stay raw.
- Thresholds are set for an FRC 12 V SLA: LOW at 11.5 V, CRITICAL at 10.5 V.
  0 disables either.
- If `/dynamic_joint_states` stops for `stale_timeout`, voltages go NaN and
  `level` goes UNKNOWN.

## spark_monitor_node

One `/diagnostics` entry per SPARK (`online`, `voltage`, `drops` since launch)
plus a `Drivetrain` summary: OK with all six, WARN in limp mode, ERROR with
none. Shows up in Lichtblick's diagnostics panel and the dashboard as-is.

The plugin decides online/offline: a SPARK with no status frame for
`status_timeout_ms` (URDF, default 100 ms) is offline. The rover keeps driving
on the rest.
