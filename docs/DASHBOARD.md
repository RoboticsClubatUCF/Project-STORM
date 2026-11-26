# STORM mission console

`storm_dashboard` provides an offline terrain map, clickable WGS84 waypoint
planning, live simulator telemetry, held-input teleoperation, and visible
Gazebo/RViz test automation. Its browser UI and command client use the same
loopback HTTP API. Assets are served locally, with no map keys, CDN, or online
tile service. The map comes from the repository's checksum-verified USGS DEM.

## Install and run

Build in Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic. A desktop or WSLg
session is required for simulator windows. Stop another simulator in the same
ROS domain before opening a scene through the console.

```bash
cd ~/Project-STORM-jazzy
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --packages-up-to storm_dashboard
source install/setup.bash
bash devscripts/run_dashboard.sh
```

Open **http://localhost:8765** in a Windows or Linux browser. WSL forwards the
loopback port. The helper sources the overlay and supports the existing local
development `pyproj` dependency when the system package is absent. In a fully
installed environment, `ros2 run storm_dashboard dashboard` also works directly.
`--open-browser` requests the default Linux browser, if configured. `--port 8766`
changes the listening port, and `--offline` enables planning without ROS/process
control. Offline mode still needs the installed terrain and mission packages.

To isolate the console, set these before launching it. Child applications
inherit the ROS domain. Each scene appends a unique suffix to the Gazebo
partition base; read its actual value from `processes.simulation.partition`
when issuing external Gazebo commands. HTTP-only agent clients do not need
ROS environment variables. Independent ROS tools must use the same domain.

```bash
export ROS_DOMAIN_ID=76
export GZ_PARTITION=storm_dashboard_visible
bash devscripts/run_dashboard.sh
```

## Map and waypoint workflow

Click **Add points**, then click the terrain map in visitation order. Drag to
pan and scroll or use +/− to zoom. **Fit site** shows the entire DEM. **Center
on rover** zooms to the current truth pose for short-range driving.

The waypoint list shows geographic coordinates. Remove individual points,
clear the list, or import a version-1 WGS84 mission JSON file. Export produces
the same format accepted by `storm_mission`. Import order determines visitation
order. For programmatic reordering, send a new ordered array using `plan.set`.
Plans are saved atomically in the application's data directory and restored
on startup. The UI never treats an imported file as executable instructions.

**Preview terrain route** runs the shared DEM planner from live truth, or from
the site datum when offline. The orange route follows terrain-grid slopes.
Unpreviewed points are connected with a dashed guide. This guide is not a
validated route. The map is bare-earth relief, not satellite imagery or an
official competition layout. The roughly 8 m DEM does not resolve small hazards.

**Open Gazebo + RViz** starts a visible MDRS mission scene with truth-pose
navigation. Wait for truth, controller, and Nav2 readiness, then **Start waypoint
mission**. Execution snapshots the current plan into a separate mission file.
Edits and manual driving are blocked while a mission or automated test runs.
Every GNSS stop still uses the mission runner's map and geodesic arrival checks.

**Terrain-guided travel beyond sensor range** is enabled by default for this
simulation workflow. The DEM supplies distant routing while the live VLP-16
ground/object pipeline supplies nearby costmap evidence and Collision Monitor
checks final velocity. Known occupied endpoints remain blocked. Uncheck the
option for strict observed endpoints. The terrain prior does not establish
field traversability. See [local navigation](LOCAL_NAVIGATION.md).

## Gazebo truth and navigation estimate

The green rover marker and trail use `/simulation/ground_truth/odom`, bridged
from Gazebo's `OdometryPublisher`. The plugin is now available independently
of navigation's pose selection. Its output is a two-dimensional horizontal
simulation reference, not a GNSS measurement or a full six-axis ground truth.
The purple marker uses fresh TF from `map` to `base_footprint`.

The console's scene uses `truth_pose:=true`, so the navigation estimate follows
Gazebo truth. The displayed difference should be near zero, with small timing
differences between topic and TF samples. These runs do not measure GNSS/IMU
localization performance. Stale markers disappear instead of retaining a
misleading live appearance. Sensor chips show recent receipt, and Nav2 readiness
comes from lifecycle services. Drive readiness uses controller-manager state.

## Teleoperation

Enable **Teleoperation**, then hold WASD, arrow keys, or an on-screen direction
button. Forward speed is bounded to 0.3 m/s and turning to 0.6 rad/s. Opposing
directions cancel. Release, tab/window blur, a hidden page, or the central stop
button clears input. The backend's 350 ms lease expires even if the browser
closes or a release request is lost. The ROS bridge publishes stamped velocity
commands and sends zero commands after disabling manual control.

Teleop requires this console's simulator, fresh truth, an active controller,
active collision monitoring, and recent filtered object observations. Commands
pass through `/cmd_vel_checked` and Collision Monitor before `/cmd_vel`.
It is mutually exclusive with console-owned missions and tests. The **Stop
motion** button clears teleop and cancels the active mission/test while leaving
the scene available for inspection. **Close simulator** also shuts down owned
launch processes. This is a simulation control, not a physical emergency stop.
Use this console as the sole command source in its ROS domain. External Nav2
clients, joystick nodes, or velocity publishers are outside its arbitration.

Mission cancellation uses a reliable, transient-local `/mission/cancel`
`std_msgs/msg/String` message carrying the current execution's unique mission
id. The runner checks that id, cancels its Nav2 action cooperatively, and waits
for the terminal result before writing its cancelled report. A token from an
earlier run cannot cancel a new run. If the runner does not exit within twelve
seconds, the console closes its own simulator before forcing process cleanup.
Check `active_goal_terminated` in the report for an active-goal cancellation.

## Visible automated tests

Select a test and click **Run selected test**. Each starts a fresh scene with
`gui:=true rviz:=true`. Stop the current scene first. Progress and errors appear
in the UI, and windows remain open after the test so the user can inspect them.

| Test id | Automated check |
| --- | --- |
| `sensors` | Waits for truth, active drive/Nav2, RViz discovery, GNSS, scan, VLP-16 cloud, RGB, IMU, and finite positive depth samples. Visual rendering remains a human check. |
| `waypoints` | Runs the installed two-stop GNSS outbound/return mission in a fresh scene and checks its success report. |
| `deadman` | Refreshes a 0.15 m/s forward command for two wall seconds, stops refreshing, and verifies movement plus at most 0.12 m drift during the later stopping observation. |

The deadman test measures a settled interval after lease expiry, not an
instantaneous braking-distance guarantee. Host load affects simulation speed
and distance during wall-time command intervals. Test startup and execution
have bounded deadlines. Failures cancel any owned mission. Cancelled tests
leave the simulator open. Stop/close commands only target processes started
by this application, never an unrelated user's ROS process.

An agent or developer can run the same tests without browser interaction.
The backend must already be running. This suite closes this console's idle
scene between tests, refuses an existing active mission/test, and leaves the
last scene open.

```bash
python3 devscripts/test_dashboard.py --test all
# Or select sensors, waypoints, or deadman:
python3 devscripts/test_dashboard.py --test deadman
```

## Agent command API

Inspect **GET `/api/schema`** for the versioned command contract and **GET
`/api/state`** for waypoints, plan, telemetry age, owned process status, test
progress, log tails, and the latest mission report. **GET `/api/site`** gives
map bounds and datum. All mutations use **POST `/api/commands`** with JSON.

```bash
ros2 run storm_dashboard command schema
ros2 run storm_dashboard command plan.set \
  '{"waypoints":[{"id":"approach","x":-1,"y":1},{"id":"home","x":0,"y":0}]}'
ros2 run storm_dashboard command plan.preview
ros2 run storm_dashboard command simulation.start
ros2 run storm_dashboard command status
ros2 run storm_dashboard command mission.start '{"allow_unobserved":true}'
ros2 run storm_dashboard command mission.cancel
ros2 run storm_dashboard command teleop.enable
ros2 run storm_dashboard command teleop.drive '{"linear":0.15,"angular":0}'
# The single drive command expires after 350 ms; refresh only while intentionally driving.
ros2 run storm_dashboard command teleop.disable
ros2 run storm_dashboard command simulation.stop
ros2 run storm_dashboard command test.run '{"id":"waypoints"}'
ros2 run storm_dashboard command test.cancel
```

Equivalent HTTP requests need no ROS installation on the agent side.

```bash
curl -s http://localhost:8765/api/schema
curl -s -X POST http://localhost:8765/api/commands \
  -H 'Content-Type: application/json' \
  -d '{"command":"plan.add","args":{"id":"near","x":-1,"y":1}}'
```

`plan.add` and `plan.set` accept either world metres `x/y` or geographic
`latitude/longitude`, with optional unique `id`. At most 100 targets are allowed.
Clicks are converted back through the same ENU transform as the mission runner.
The DEM's UTM-versus-ENU approximation still applies. HTTP 409 indicates an
active-operation/readiness conflict, 400 invalid input, and 500 an operation
failure. Mutations return `{ok:true,result:...}` or an error object. Test runs
are asynchronous. Poll state until `passed`, `failed`, or `cancelled` and until
`busy` clears before another test. A navigation process exiting alone does not
prove arrival. Inspect the mission report's status and errors.

The server binds only `127.0.0.1`. Host/Origin checks and JSON-only mutations
prevent ordinary cross-origin browser requests. Commands are allowlisted,
paths and launch arguments are controlled by the backend, and no arbitrary
shell endpoint exists. This local service has no multi-user authentication.
Do not expose it to a network without a separately designed authenticated
gateway. Remote rover/C2 transport is outside this implementation.

## Reports, dependencies, and extension points

Default records live in `~/.local/state/storm_dashboard/`, configurable through
`--data-dir`. Every scene and mission gets a unique directory with launch logs.
Missions include their input snapshot and the existing JSON arrival report.
Tests write a separate JSON result. Plan persistence is separate from execution
records. Restart never automatically resumes motion.

The Python backend uses the standard HTTP/subprocess libraries, ROS `rclpy`,
TF, lifecycle and controller-manager services, and NumPy/Pillow for map and
depth processing. It imports the shared `storm_mission.planner` module rather
than duplicating geographic conversion or terrain routing. The frontend uses
plain JavaScript and Canvas, with no Node build step. Package dependencies are
declared in `package.xml` and installed through ament.

Add tests to `Application.TESTS` and the bounded `run_test` workflow. New agent
operations belong in the command allowlist and schema with validation and
regression tests. Add telemetry subscriptions in `RosBridge`; keep ROS spinning
separate from HTTP workers. Future field localization must retain independent
truth tracking for simulation and explicitly establish the geographic map datum.

```bash
colcon test --packages-select storm_mission storm_dashboard
colcon test-result --verbose
```

Known limits include the coarse DEM, simulation-only truth navigation, no
mission resume, no communications-loss policy, and no physical rover command
gateway. The console supports URC practice and does not certify full rules
conformance. See [validation](VALIDATION.md) for exercised behavior.

### Diagnosing a stopped waypoint itinerary

The console reports `navigation_blockers` in `/api/state` and shows the same
details above the mission controls. Lifecycle state `unconfigured` or `inactive`
means Nav2 cannot execute goals. If startup remains stuck, inspect the scene's
`simulation.log`; stop that scene and launch a fresh one before retrying. A
restart is a recovery action, not evidence that an intermittent middleware
startup timeout has been permanently resolved. Missions never resume themselves.

During execution, the status displays completed stops, total targets, and the
current target. The mission report records reached stops in order. Execution
stops on the first failed leg or arrival check; later targets are not silently
skipped. A failed mission must be explicitly planned and started again.

`No observed free approach to next waypoint` means the endpoint policy could
not find an eligible approach in the fresh rolling costmap. For simulated
long-distance DEM routes, enable **Terrain-guided travel beyond sensor range**.
This permits unknown endpoint cells while keeping occupied cells, local object
processing, and Collision Monitor checks in effect. It does not extend sensor
range or establish that distant terrain is free of obstacles. Standalone mission
commands require explicit `--allow-unobserved` for this simulation policy.
