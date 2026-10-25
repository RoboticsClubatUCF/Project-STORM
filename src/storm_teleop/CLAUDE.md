# storm_teleop

ROS 2 teleop for a 6-wheel URC rover: joy → teleop_twist_joy → diff_drive_controller → SparkCanHardware → 6× SPARK MAX over CAN.

## Style
- Comments short and to the point, one line where possible
- Explanations go in chat, not in code or README
- Avoid em dashes in comments and docs; use a comma, colon or new sentence
- C++: 2-space indent, `snake_case_` members, log via `rclcpp::get_logger("SparkCanHardware")`
- Config files keep the `# ===` header banner and inline `# unit, note` comments

## Project notes
- Humble/Iron-era config (`use_stamped_vel`, `cmd_vel_unstamped`)
- Speed limiter clamps dv by sign, not speed: keep `min_acceleration = -max_acceleration`
- Joy scales are negative on purpose (inverted sticks)
- Normal mode is the indoor cap; turbo is outdoors only
- Base station is a Jetson: runs `storm_basestation/launch/base_station.launch.py` from source, no build (no sparkcan there), hosts the STORM-BASE hotspot and storm-dashboard for viewing laptops
- foxglove_bridge runs on the rover (storm_basestation/bridge.launch.py, included by teleop.launch.py)
- can_health_node lives in storm_telemetry; battery voltage comes from the plugin's `voltage` state interface
- Plugin reads SPARK status via its own CAN socket (SparkStatusReader); sparkcan's getters drop frames (shared socket), use it for TX only
- Mid-drive SPARK dropout = limp mode + report, not stop
- `sparkcan` is a system library, not a ROS package
- Can't build or run ROS on the dev Mac; test on the rover

## Workflow
- Don't commit unless asked