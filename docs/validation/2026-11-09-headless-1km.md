# Accelerated headless kilometre test — 9 November 2026

**Result: FAILED. The 1 km goal is not validated.**

The actual three-stop WGS84 itinerary was executed through the installed mission
runner, Nav2, Gazebo dynamics, range-limited sensors, and Collision Monitor.
The final attempt used Gazebo's server only, without Gazebo GUI or RViz, on an
isolated ROS domain (78) and Gazebo partition. WSLg provided sensor rendering
using `D3D12 (NVIDIA GeForce RTX 5060 Laptop GPU)`.

| Measurement | Result |
| --- | --- |
| Requested / achieved simulation factor | 4.0 / 2.42029 |
| Mission wall duration | 1,332.33 s (22.21 min) |
| Mission simulation duration | 3,224.62 s (53.74 min) |
| Physics step | 0.001 s, unchanged |
| Near stop, approximately 23.83 m | Passed; 0.04718 m GNSS error |
| Middle stop, approximately 151.38 m | Passed; 0.23842 m GNSS error |
| Far stop, approximately 1,000.56 m | Failed; local leg timed out |
| Final Gazebo truth x / y | 782.22896 / -31.98120 m |
| Final straight-line distance from datum | 782.88246 m |
| Far-target progress | Leg 87 of 118; requested endpoint (789.53, -31.90) m |
| Collision state transitions | 29 SlowZone, 1 StopZone, 28 clear |
| Active action terminated after cancellation | Confirmed by mission runner |

Average net progress was 782.88246 / 3,224.621 = **0.24278 m/s (0.87402 km/h)**
in simulation time, including stops and recovery. This is displacement divided
by elapsed time, not total ground-track distance divided by time: the full truth
trajectory was not recorded. Nav2 and the velocity smoother cap translation at
0.30 m/s, acceleration/deceleration magnitude is 0.5 m/s², and SlowZone scales
commands to 40%. Turns, local goal approaches, and the final safety stop reduce
the average. These are simulation software settings, not qualified hardware
speed limits. Acceleration changes wall duration rather than rover speed.

Collision Monitor logged `Robot to stop due to StopZone polygon` before the
rover became stationary. Nav2 subsequently reported `Failed to make progress`,
attempted recoveries, and ultimately the mission runner's 90-wall-second leg
deadline expired. The log contains no ground-filter `Obstacle observations
withheld` warnings. The stop is therefore associated with detected points in
the stop polygon, rather than an observed ground-filter data-withholding event.
The saved evidence does **not** establish which sensor source supplied those
points or whether they represented genuine obstructing terrain versus a ground
classification/projection error. A local reproduction with raw cloud, segmented
cloud, scan, TF, costmaps, and collision-state recording near this location is
needed before changing the collision policy. No safety checks were disabled.

There was also a transient NavFn planning error earlier in the traversal, followed
by continued progress. Controller-rate warnings occurred during recovery. Gazebo
reported a segmentation fault during shutdown after the failed mission had been
recorded; this was not the initiating cause of the mission failure.

An eight-wall-second audit during motion passed: 717,352 finite lidar samples
remained within 1–100 m (maximum 99.99720 m); depth stayed within 0.5–20 m.
The derived scan bounds and local object radius also passed. At that sampled
location the local object stream published valid empty clouds. This range audit
does not explain the later StopZone trigger.

Two earlier partial attempts were explicitly cancelled to investigate software
rendering. The EGL attempt achieved approximately 1.84x and stopped around
120 m; display rendering without a forced driver achieved approximately 1.97x
and stopped around 61 m. They are not completed validation attempts. Selecting
the available NVIDIA driver was scoped to the final process environment, with
no system graphics changes. The older STORM visual-test world temporarily paused
for resources was restored after the test.

Evidence is in [the final run directory](2026-11-09-headless-1km-03/status.json):
`status.json`, `mission.json`, `mission.log`, `simulation.log`, `sensor-ranges.json`,
the generated `world.sdf`, and the input `waypoints.json`. World and mission
SHA-256 values are recorded in the status. The reusable command and dependencies
are documented in [headless validation](../HEADLESS_VALIDATION.md).

The modified launch packages built successfully; the Gazebo sensor model test and
targeted Ruff checks passed. This is truth-localized simulation with conservative
sensor limits. It does not validate physical GNSS fusion, payload handling, radio
range, or full URC delivery compliance.
