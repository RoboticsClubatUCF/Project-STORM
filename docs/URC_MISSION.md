# URC2027 mission-scale practice

The official Q&A clarifies that Delivery and Autonomy may take the rover up
to 1 km from C2, with total travel potentially longer. It also describes
natural hill routing and a 1 m full-scoring distance for the specified
Autonomy targets. [URC2027 rules and Q&A](https://urc.marssociety.org/home/requirements-guidelines)

The `urc_mission` preset uses a 2.56 km × 2.56 km USGS request centered on
MDRS. Its rendered grid spans about 2.55 km per side between sample centers,
with approximately 8 m spacing and 204,800 collision triangles. The original
`urc` preset remains the smaller, finer test world. Both use the same
simulated sensor stack. [USGS 3DEP](https://www.usgs.gov/3d-elevation-program)

```bash
cd ~/Project-STORM-jazzy
source install/setup.bash
ros2 launch storm_bringup simulation.launch.py environment:=urc_mission truth_pose:=true
```

Add `gui:=false rviz:=false` for headless runs. The truth-pose option is useful
for terrain and controller checks. Use the default estimator when measuring
localization drift.

The accelerated [1 km trial](validation/2026-11-09-headless-1km.md) reached
782.88 m before a StopZone intervention and leg timeout. The distant goal is
not yet validated. The [headless runner](HEADLESS_VALIDATION.md) preserves the
normal physics step, rover speeds, and local collision chain.

The 1 m arrival criterion used by the mission runner is an engineering check;
it is not a Delivery scoring rule. The 2027 Delivery requirements describe
staged tasks, approximate WGS84 pickup/dropoff coordinates, terrain and radio
challenges up to 1 km from the start, and payload/manipulation work. Course
details and stage thresholds are not represented by this navigation test.
Neither simulator acceleration nor an eventual GNSS arrival would establish
Delivery task completion or compliance with the mission's real time budget.

| Practice reference | World x, y in metres | Purpose |
| --- | --- | --- |
| C2 spawn | 0, 0 | Start of the practice course. Blue post sits 2 m south. |
| Green reference | 23.93, 0 | Near-range approach; post sits 2 m south |
| Amber reference | 151.53, 0 | Multi-leg traversal; post sits 2 m south |
| Red reference | 996.88, 0 | Approximate maximum C2 range; post sits 2 m south |

These are synthetic visual posts with no collision geometry. They represent
distance annotations. They sit south of the centerline because GPU lidar also
sees visual geometry, which must not occupy the intended practice goals.
Target recognition, ArUco imagery, radio outages,
astronaut behavior, and delivery or servicing interactions need separate
scenario models. The DEM represents broad terrain slopes. Its sampling
does not resolve individual rocks, tracks, or small ledges.

Start with a short goal and inspect the rolling global costmap. For a longer
run, the mission runner below sends successive short goals inside the
current costmap. The 40 m rolling window extends about 20 m in each
direction from its center. The terrain route gives the runner a direction
beyond that window.

## Long-distance route planner and runner

See [GNSS waypoint missions](GNSS_WAYPOINTS.md) for ordered geographic target
files, runtime fix checks, full-route visualization, and per-stop reports.

Build the mission package after installing its declared dependencies:

```bash
rosdep install --from-paths src/storm_mission --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --packages-select storm_mission
source install/setup.bash
colcon test --packages-select storm_mission
colcon test-result --verbose
```

Preview the route to the red post without sending a motion command:

```bash
ros2 run storm_mission mission --goal-xy 996.88 0 > /tmp/storm-route.json
```

Use `--goal-gnss LAT LON` to enter a WGS84 target. The runner projects the
coordinates into local east and north using the Gazebo world origin. The
map must be anchored to that origin and aligned with its axes. The default
15° DEM edge limit is a practice setting. It is not a competition slope
limit or a rover capability measurement. The planner weights distance and
slope and splits the route into legs of at most 9 m by default.

For execution, start the mission world in truth-pose mode as shown above.
In a second sourced terminal on the same ROS domain, run:

```bash
ros2 run storm_mission mission --goal-xy -1 1 --max-leg 0.6 --execute
```

The runner defaults to observed costmap endpoints and looks for a shorter
approach when the next endpoint is unavailable. Sparse terrain can leave
cells unobserved. For simulation trials that use the DEM as the prior,
add `--allow-unobserved`. Occupied endpoints still fail the check. This
option does not validate whether unknown terrain is safe. Nav2 owns
local path and obstacle checks. The DEM does not capture small hazards or
guarantee traction and rollover clearance.

Run the mission process onboard or beside the simulator in the same local
ROS graph. It sends legs without operator prompts. If a leg is rejected,
aborts, or times out, the runner stops issuing goals. Ctrl+C requests
cancellation of an active goal. Follow the competition return-to-start or
return-to-previous-point procedure separately after an aborted attempt.
The runner checks final map-frame error against a 1 m radius by default.
For a GNSS target it also checks a fresh fix against the target geodesically.
That check uses the sensor antenna position.

Execution currently requires the Gazebo ground-truth odometry bridge.
Field use needs a calibrated GNSS/IMU map frame, independently qualified
obstacle avoidance, a return procedure, and the competition's indicator
and target-detection logic. This tool exercises the route-finding portion
of the mission. Radio loss and the full scoring procedure remain to be
tested. [URC2027 Q&A](https://urc.marssociety.org/home/requirements-guidelines/qa)

In the MDRS simulation, the red-post preview produced a 1,029.9 m route
with 135 legs. The short three-leg run above completed with
`--allow-unobserved` and 0.195 m of final map-frame error. The full route
has been planned but has not been driven end to end.

For repeatable trials, record elapsed simulation time, distance traveled,
horizontal error against Gazebo pose, and whether the rover reaches the
target without intervention. For a target covered by the 1 m criterion,
measure the final distance to its coordinates independently of the Nav2
action result.

The source GeoTIFF and provenance are stored under
`src/storm_gazebo/terrain/mdrs_mission/`. To regenerate the mesh from the
cached data, run:

```bash
python3 devscripts/fetch_urc_terrain.py --span 2560 --samples 321 \
  --output src/storm_gazebo/terrain/mdrs_mission
```
