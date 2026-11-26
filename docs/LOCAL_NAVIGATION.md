# Long routes with local collision checks

The terrain planner and object detector operate at different scales. A distant
GNSS target does not need to be visible in today's sensor scan. The rover must
still react to objects as they enter its nearby sensor coverage.

```text
WGS84 itinerary + DEM slope prior
    -> terrain route -> short map-frame goals -> Nav2 planner/controller
VLP-16 -> local ground fitting -> ground clearing + object marking -> costmaps
Nav2 -> velocity smoother -> cmd_vel_checked -> Collision Monitor -> cmd_vel
UI manual drive -----------> cmd_vel_checked ---------^
```

The URC profile retains a 40 m rolling global costmap and allows unknown space
in NavFn. The mission relay may select unknown intermediate endpoints using
the terrain prior, while rejecting known occupied endpoints. An 8 m local
costmap supports DWB obstacle avoidance. Its initial free-space assumption
does not assert that unseen terrain has been measured safe. The independent
collision layer stops commands if required sensor observations expire.

## Local perception

`storm_perception/ground_filter` consumes `/velodyne_points`, transforms it
into `base_footprint` at the measurement timestamp, and processes a 10 m
radius around the rover. A bounded RANSAC fit estimates one nearby ground
plane, constrained to a 25 degree slope and a ground intercept within 0.5 m
of the rover reference. Refinement uses the supported points. Missing ground,
stale input, or unavailable TF causes observations to be withheld.

The node publishes two PointCloud2 topics at up to about 5 wall Hz.

| Topic | Use |
| --- | --- |
| `/perception/ground` | Supported ground samples clear costmap rays; they never mark obstacles |
| `/perception/obstacles` | Samples at least 0.12 m and at most 2 m above the fitted plane mark objects |

Ground tolerance is 0.08 m. These are practice settings, not measured STORM
traversability limits. `ground_tolerance`, `obstacle_height`, and
`max_ground_slope` are ROS parameters on the filter. Input within 0.65 m of
the rover reference is excluded to reduce self returns. Output positions
retain their actual coordinates; relative height is used for classification.

Both local and global obstacle layers use object points for marking and ground
points for clearing. Inflation adds footprint clearance. This avoids the old
dependency on a horizontal scan receiving ground returns before marking an
approach free. Ground-ray clearing and marking in 2D remain approximations.

## Final velocity checking

The URC Nav2 launch starts lifecycle-managed `nav2_collision_monitor`. The
velocity smoother and dashboard manual controls publish upstream of it.
Collision Monitor alone supplies their final `/cmd_vel` output. Arena profiles
without a collision-monitor section retain their existing command chain.

The monitor reads the filtered object cloud and `/scan` projected from the same
VLP-16 cloud. These are two processing views of one sensor, not independent
sensor redundancy. Stop and slowdown
polygons are symmetric to cover forward and reverse motion and are expressed
in the rover frame.

| Zone | Half extent x / y | Action |
| --- | --- | --- |
| StopZone | 1.1 / 0.7 m | Stop when at least three observed points lie in the zone |
| SlowZone | 1.8 / 1.0 m | Reduce command to 40% with at least three points |

Either required source older than one simulation second blocks motion. Missing
ground segmentation therefore cannot silently turn into an empty clear scene.
These polygons need physical stopping-distance and coverage qualification before
field use. They are not a certified emergency stop.

The console requires active collision monitoring and recent filtered objects
before enabling a mission or manual drive. **Terrain-guided travel beyond sensor
range** is enabled by default in this simulation workflow. Uncheck it to retain
strict observed-endpoint selection. The standalone mission CLI still defaults
to strict endpoints and supports `--allow-unobserved` explicitly.

## Run and inspect

```bash
cd ~/Project-STORM-jazzy
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --packages-up-to storm_dashboard
source install/setup.bash
bash devscripts/run_dashboard.sh
```

Close an old console scene and restart the backend when upgrading from the
earlier direct-velocity version. Open a fresh Gazebo/RViz scene in the console.
Wait for **OBJECTS** and **COLLISION** readiness, preview a route, and start it.
The headless 1 km trial stopped near 783 m after a StopZone intervention and
leg timeout; the full route remains unvalidated. See the
[trial report](validation/2026-11-09-headless-1km.md).

```bash
ros2 topic echo /collision_monitor_state
ros2 topic info /perception/obstacles --verbose
ros2 topic info /cmd_vel --verbose
colcon test --packages-select storm_perception storm_navigation storm_dashboard
colcon test-result --verbose
python3 devscripts/test_dashboard.py --test all
```

The separate physical-obstacle check requires an idle console-owned simulator
and the same ROS domain as the backend. It derives the current scene's unique
Gazebo partition from the API, spawns a red box, holds a forward command,
checks marked costmap cells and StopZone output, then removes the box. If box
removal cannot be confirmed, it closes that test scene.

```bash
export ROS_DOMAIN_ID=76  # use the domain chosen for this console
python3 devscripts/test_local_obstacles.py
```

## Limits and next work

One local plane is a conservative initial segmentation method. Rough terrain,
multiple levels, or steep local transitions can produce false obstacles or
withheld observations. Low objects below the threshold, holes, drop-offs, and
the lidar's near blind region require additional handling. A loss of reflected
points does not prove free ground. The raw planar scan can conservatively stop
the rover on a nearby uphill surface. Adjustments need representative tests.

For field deployment, qualify sensor extrinsics, local elevation/negative
obstacle detection, timing and stopping distances, and GNSS/IMU localization.
Keep this terrain-prior assumption explicit. A DEM alone cannot certify the
nearby ground. Benchmark the perception workload on the Orin Nano.

Scene launches use a unique Gazebo partition to avoid discovery from a previous
run. The console exposes it under `processes.simulation.partition`. Child cleanup
captures process ancestry because Gazebo creates separate GUI/server groups.
Readiness polling uses a steady clock and resets service futures between scenes,
so restarting simulation time does not leave the console's health checks paused.

Implementation follows the standard Nav2
[Collision Monitor](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/)
and [obstacle-layer](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/obstacle/)
interfaces. See [validation results](VALIDATION.md) for the scope actually tested.
