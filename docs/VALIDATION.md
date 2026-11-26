# Jazzy validation

Tested on November 2, 2026 in Ubuntu 24.04 under WSL with ROS 2 Jazzy and
Gazebo Harmonic. Each runtime test used its own ROS domain and Gazebo partition.

| Check | Result |
| --- | --- |
| rosdep check | All system dependencies satisfied |
| Default colcon build | Eight packages built |
| Optional CAN build | Pinned sparkcan vendor and Jazzy plugin compiled |
| Package tests | Twelve colcon test records, zero failures |
| Mock base | Both controllers active, stamped velocity subscription, odometry in base_footprint |
| Arena SLAM | NavigateToPose to (1.5, 0) succeeded |
| Arena saved map | AMCL initial pose accepted and NavigateToPose to (1, 0) succeeded |
| MDRS terrain | Both controllers active and NavigateToPose to (-1, 1) succeeded |

Runtime goals use map-frame coordinates. Physical CAN activation and driving
remain to be tested on the bench. This revision did not remeasure outdoor
localization accuracy. The earlier terrain test measured roughly 0.7 m of
position error against Gazebo ground truth.

## Sensor and visualization follow-up

- Jazzy built `storm_sensors` and the updated bringup. Launch checks rejected
  invalid connection settings and a missing integrated VLP-16 mount.
- The headless MDRS test received the planar scan, VLP-16 cloud, IMU, GNSS,
  RGB image, depth image, camera info, and joint states.
- A depth sample after tilting the camera contained 178,575 finite values
  out of 230,400 pixels, with valid ranges from 0.54 m to 19.63 m.
- In `truth_pose` mode, a short Nav2 goal succeeded and ROS odometry
  matched the horizontal Gazebo pose afterward.

These checks used WSL and Gazebo. Physical lidar and Jetson deployment still
need bench testing. Ground-truth mode bypasses the wheel/SLAM estimate.

## GNSS waypoint extension — November 4, 2026

The mission package built in Ubuntu 24.04 with ROS 2 Jazzy. Its 17 pytest
cases passed. A preview of `mdrs_long_route.json` produced an ordered
1,034.07 m route with 140 short legs. That itinerary has not been driven
end to end.

An isolated Gazebo Harmonic `urc_mission` run used `truth_pose:=true`,
`--allow-unobserved`, and the installed `mdrs_smoke.json` itinerary with
0.6 m maximum legs. Both GNSS targets completed. The northwest stop had
0.194 m map error and 0.203 m geodesic GNSS error. The return stop had
0.195 m map error and 0.197 m GNSS error. Both satisfied the 1 m threshold.

A subsequent geographic goal received injected `NavSatFix` messages with
status -1 during execution. The runner failed with its GNSS watchdog reason,
cancelled the active Nav2 action, and observed a terminal result. The JSON
report recorded `failed` and `active_goal_terminated: true`.

These checks exercise the waypoint application with simulated GNSS and
Gazebo truth localization. They do not validate GNSS/IMU fusion, receiver
accuracy, or complete URC mission compliance. The full route display was
configured in RViz but has not received a separate visual inspection.

## Mission console — November 4, 2026

Built `storm_dashboard`, the shared `storm_mission` Python module, and the
independent Gazebo truth bridge on Jazzy. The 14 dashboard pytest cases and
17 mission cases passed, with Ruff checks passing. These cover input validation,
projection round trips, ordered preview, plan persistence, leased velocity,
mission/manual exclusivity, visible launch arguments, HTTP commands, and
cross-origin rejection.

Browser checks exercised responsive mobile and desktop layouts, clicked map
waypoint creation, terrain route preview, and held-button manual driving.
The manual drive changed Gazebo truth by about 0.34 m. Native Gazebo and RViz
windows were brought forward and inspected, showing the MDRS scene, rover,
navigation/sensor displays, and camera panes.

`devscripts/test_dashboard.py --test all` passed through the public HTTP API,
opening a fresh visible Gazebo/RViz scene for each test. Sensors delivered
positive finite depth returns. The teleop lease test traveled 0.27898 m and
measured approximately 0.00000005 m drift in its later settled interval.
The outbound/return mission succeeded with geodesic GNSS errors of
0.21969 m and 0.18401 m, and map errors of 0.19544 m and 0.19480 m.

An additional test cancelled a moving mission through the API. It exposed
unconfirmed action cancellation when interrupting the Python process. The
console now uses an execution-specific ROS cancellation token. Repeating the
moving test produced `status: cancelled` and `active_goal_terminated: true`.
The simulator remained available afterward. Forced process cleanup remains
a fallback, with simulator shutdown if cooperative cancellation times out.

These are simulation results using truth-pose navigation. They do not validate
physical GNSS localization, field stopping performance, communications loss,
or complete URC rules conformance. The 1 km route remains preview-only.

## Terrain routing with local object checks — November 4, 2026

Added local VLP-16 ground-plane fitting, separate clearing and marking clouds,
and lifecycle-managed Collision Monitor after velocity smoothing. UI manual
drive uses the same checked input. The terrain-prior mission policy permits
unknown distant endpoints only after local collision readiness in the console.
The standalone CLI retains explicit unknown-space opt-in.

Ground-segmentation tests passed on flat and sloped synthetic ground with
elevated objects, and rejected wall-only or insufficient ground samples.
Configuration and backend checks cover the command chain, ground/object roles,
and blocking motion when local collision readiness is absent.

The updated visible sensor, deadman, and GNSS outbound/return suite passed.
The deadman trial moved 0.29404 m and recorded 0.00000008 m settled drift.
The two GNSS stops had 0.21336 m and 0.20383 m geodesic error.

A physical Gazebo box was placed 1.7 m ahead of the rover. Holding a forward
request moved it 0.16791 m before StopZone intervened. The checked output was
0 m/s, filtered objects contained 1,032 points, and 341 occupied local cells
were within 0.6 m of the box center. The box was removed afterward.

A subsequent mission to x=15.95016 m, y=0 succeeded through three local legs,
with 0.19548 m map error and 0.17969 m GNSS error. The user's previous plan was
restored. This exceeds the area that previously blocked strict endpoint
selection, but does not establish end-to-end kilometre traversal.

Fresh-scene regression initially exposed stalled Gazebo discovery and readiness
after time resets. Per-scene transport partitions, cleanup of captured Gazebo
descendants, steady-clock health polling, and service-request retries were
added. A later complete three-scene test sequence passed. An early physical
box trial also exposed an incorrect spawn-pose assumption in the test utility;
explicit create-command coordinates corrected it before the passing trial.

The single-plane classifier and stop polygons remain simulation settings.
Small/negative obstacles, rough terrain, latency and braking bounds, and
physical Orin Nano deployment still need qualification. RViz's existing map
shader issue is separate and was not corrected by these navigation changes.

## 9 November 2026 — headless kilometre trial

The accelerated three-stop trial reached the near and middle GNSS stops but
failed at 782.88 m from the datum after a StopZone intervention and leg timeout.
The 1 km goal remains unvalidated. Achieved acceleration was 2.42x with the normal
1 ms physics step and rover limits. See the [failure report and evidence](validation/2026-11-09-headless-1km.md)
for the confirmed observations, unresolved sensor-source question, and next
reproduction needed. No collision gates were bypassed.
