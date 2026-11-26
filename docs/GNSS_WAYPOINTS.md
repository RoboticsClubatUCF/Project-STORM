# GNSS waypoint missions

`storm_mission` accepts ordered WGS84 targets, plans terrain routes, and sends
short `NavigateToPose` goals to Nav2. Routing and execution run in the local
ROS graph without prompts between targets. Nav2 owns local avoidance and velocity.

Execution currently requires the MDRS simulation with `truth_pose:=true`.
GNSS supplies target coordinates, runtime validity checks, and independent
arrival measurements. It does not yet correct the rover's estimated pose.
Gazebo supplies that pose. Field use needs measured GNSS/IMU mounts and a
validated global localization configuration.

## Build and preview

```bash
cd ~/Project-STORM-jazzy
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/storm_mission --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --packages-select storm_mission
source install/setup.bash
ros2 run storm_mission mission --mission \
  "$(ros2 pkg prefix --share storm_mission)/missions/mdrs_long_route.json"
```

Preview prints projected targets, terrain path lengths, and short legs, plus
total length and leg count. It sends no actions. The included long itinerary
visits synthetic practice points about 24 m, 151 m, and 1,001 m east of the datum.

The versioned JSON format uses array order for visitation. Ids must be unique,
nonempty strings and coordinates finite WGS84 decimal degrees. Unknown fields
and unsupported frames are rejected.

```json
{
  "version": 1,
  "frame": "WGS84",
  "waypoints": [
    {"id": "near", "latitude": 38.40645833333333, "longitude": -110.79163},
    {"id": "far", "latitude": 38.40645833333333, "longitude": -110.78045}
  ]
}
```

Existing `--goal-gnss LAT LON` and `--goal-xy X Y` remain available. Select
exactly one goal option or `--mission`. Preview starts at `--start-xy` (default
0, 0). Execution uses live TF and replans each target from the current pose.
The entire itinerary is checked for terrain routes before the first action.
Every target must lie inside the selected DEM.

## Execute a short mission

In the first sourced terminal, start the simulator.

```bash
ros2 launch storm_bringup simulation.launch.py \
  environment:=urc_mission truth_pose:=true
```

In a second sourced terminal, run the smoke itinerary. It drives about a metre
northwest and returns to the datum. Use a new report filename for each run.
Use the same ROS domain and Gazebo partition in both terminals if you set them.

```bash
ros2 run storm_mission mission --mission \
  "$(ros2 pkg prefix --share storm_mission)/missions/mdrs_smoke.json" \
  --max-leg 0.6 --allow-unobserved --execute --report /tmp/storm-smoke-01.json
```

`--allow-unobserved` explicitly permits unknown costmap endpoints for DEM-backed
simulation trials. Without it, endpoints must be observed free. Occupied
endpoints remain rejected. This option does not establish traversability of
unknown terrain. See [terrain limits](URC_MISSION.md).

Every GNSS target requires both map-frame and geodesic antenna-to-target error
within `--arrival-radius` (default 1 m, optionally tighter). Nav2 success alone
does not advance the mission. A failed arrival check stops execution. The
simulated antenna sits above the rover center. A physical antenna offset needs
explicit handling when comparing antenna and rover positions.

## Runtime interfaces and failure behavior

| Interface | Purpose |
| --- | --- |
| `navigate_to_pose` action | Sequential short goals to Nav2 |
| `/gps/fix`, `sensor_msgs/msg/NavSatFix` | Fix validity, freshness, arrival distance |
| TF `map` to `base_footprint` | Live pose with two-second freshness limit |
| `/global_costmap/costmap` | Endpoint checks in the rolling map |
| `mission/route`, `nav_msgs/msg/Path` | Current target's full terrain route, reliable and transient local |

A GNSS itinerary waits for a valid fix before moving. During navigation and
approach waits, the runner checks fix status, coordinate validity, timestamp
age (at most two simulation seconds), and receipt age (at most five wall
seconds). Failure stops further goals and requests cancellation of a pending
or active action. Rejection, abort, timeout, and Ctrl+C also stop execution.
Unconfirmed cancellation or termination is reported as an error. These checks
do not qualify RTK performance or prove one-metre positioning accuracy. GNSS
covariance and receiver accuracy still need qualification.

RViz's **Mission terrain route** Path display shows `/mission/route` using
transient-local durability. Nav2's `/plan` remains the short planning view.
The route is refreshed at each mission target. It is not cleared on exit.

`--report` records input targets, datum, arrival radius, status, and per-target
errors. It checkpoints at target boundaries using file replacement. A failed
or cancelled target is marked accordingly, with the mission reason recorded.
When cancelling an active action, `active_goal_terminated` records whether a
terminal result was observed. Existing reports are rejected. Reports are
records, not resume commands. A power loss may leave a `running` record that
needs inspection before another run. A report write failure stops execution.

## Localization and competition extensions

Conversion uses the terrain provenance's fixed ENU datum, aligned with the
simulator NavSat origin. Do not substitute UTM coordinate subtraction for this
conversion. The DEM uses UTM grid axes while Gazebo NavSat uses ENU. The small
alignment approximation and elevation-datum limits are described in the terrain
documentation.

The next localization integration should follow the standard
[Nav2 GPS architecture](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html),
using `robot_localization` and `navsat_transform_node` with wheel odometry,
IMU, and GNSS. Assign exactly one owner per dynamic TF and convert waypoints
using that established map datum. Retain the truth-pose execution guard until
the replacement path is implemented and validated.

The [URC2027 Q&A](https://urc.marssociety.org/home/requirements-guidelines/qa)
informs distance and arrival practice. Target recognition, indication,
communications-loss trials, and abort/return behavior remain separate work.
A `home` waypoint is an ordinary planned stop, not an automatic abort policy.

## Tests

```bash
colcon test --packages-select storm_mission --event-handlers console_direct+
colcon test-result --verbose
```

Tests cover terrain routing, bounded legs, the datum origin, invalid coordinates,
strict file parsing, ordered outbound/return planning, failure at a later stop,
and basic fix validity. Run the short mission before the kilometre itinerary.
