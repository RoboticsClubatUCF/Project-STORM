# Navigation evaluation and evidence

This package captures STORM's GNSS mission planning, local collision processing,
mission console, realistic sensor bounds, and accelerated headless test tools.
**The 1 km goal remains unvalidated.** A passing package test suite does not
override the failed simulation trial.

| Evaluation | Outcome | Evidence |
| --- | --- | --- |
| GNSS smoke outbound and return, 9 November | Both stops passed; GNSS errors 0.21223 and 0.19815 m | [Waypoint diagnosis](2026-11-09-waypoint-diagnosis.md) |
| Three-stop 24 m / 12 m / home itinerary | All three passed; GNSS errors 0.19174, 0.24367, 0.21443 m | [Execution report](2026-11-09-waypoints.json) |
| Range audit during local travel | Passed lidar, derived scan, depth, local object bounds | [Range report](2026-11-09-sensor-ranges.json) |
| Accelerated headless 1 km itinerary | Two stops passed; stopped 782.88 m from datum, far leg 87/118 timed out | [Evaluation](2026-11-09-headless-1km.md), [machine-readable status](2026-11-09-headless-1km-03/status.json) |
| Sensor range audit during kilometre trial | Passed; 717,352 finite lidar samples within 100 m | [Range report](2026-11-09-headless-1km-03/sensor-ranges.json) |
| Package build, tests, targeted lint and JavaScript syntax | Passed; 12 packages built; colcon reported 63 test records with zero failures/errors | [Packaging checks](2026-11-09-package-checks.json) |

The headless trial averaged 2.42029x simulation speed, covering 53.74 simulated
minutes in 22.21 wall minutes. Average **net progress** was 0.24278 m/s
(0.87402 km/h), including the final stop and recoveries. Total ground-track
distance was not recorded, so this is not a measured path-length average speed.
See the trial evaluation for the 0.30 m/s software cap and slowdown settings.

Two earlier headless partial attempts were cancelled to investigate software
rendering. Their status and logs are retained in the `*-01` and `*-02` directories;
they are not successful kilometre runs. The final `*-03` attempt selected the
available NVIDIA device for server-only sensor rendering. No system graphics
configuration or collision policy was changed to force a passing result.

The earlier physical-box stopping and teleop-deadman trials are recorded in
[the validation history](../VALIDATION.md). They predate the latest sensor range
profile and do not substitute for fresh obstacle qualification with that profile.

## Remaining blockers and next experiment

The long-run log confirms StopZone activation, repeated Nav2 progress failures,
and a confirmed terminal action after cancellation. It does not identify whether
the stopping points came from the projected scan or segmented object cloud, nor
whether they represented impassable terrain or a ground interpretation error.
Reproduce the area near x=782.23 m, y=-31.98 m with raw and filtered clouds, scan,
TF, local/global costmaps, command stages, and collision-state recording before
changing thresholds or rerunning the complete itinerary. Keep the failed run as
a regression case. Do not bypass StopZone to claim success.

Intermittent Nav2 lifecycle startup timeouts remain unresolved; restarting a
scene recovered the exercised case. Gazebo also reported a shutdown segmentation
fault after the failed kilometre mission was already recorded. These are separate
from the confirmed point-triggered StopZone event.

Current execution uses simulation truth localization. The coarse USGS DEM and
idealized sensor models do not qualify real GNSS fusion, braking/traction,
negative obstacles, payload handling, RF coverage, or complete URC compliance.
See [headless commands and dependencies](../HEADLESS_VALIDATION.md),
[sensor profiles](../SENSORS.md), and [the mission console](../DASHBOARD.md).
