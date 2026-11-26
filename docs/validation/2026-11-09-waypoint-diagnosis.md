# Waypoint execution diagnosis — 9 November 2026

The initial console scene `scene-a3d26fb626` was stationary because Nav2 never
completed lifecycle startup. The controller was inactive (state 2); the planner,
BT navigator, and Collision Monitor were unconfigured (state 1). Its log reported
a timeout sending the `/controller_server/change_state` response. This was a
startup failure, before waypoint execution. The underlying intermittent service
transport failure is not yet isolated; a fresh scene recovered successfully.

Older long-route reports `mission-ce1a8d8de5` and `mission-76b453486c` failed on
the first target with `No observed free approach to next waypoint`. Strict
observed-space endpoint selection could not progress over the distant route.
The console's current simulation policy explicitly permits unknown endpoint
cells when terrain-guided travel is enabled. Occupied cells and the local
collision command chain remain checked. Sensor range is not extended.

A fresh scene completed the two-stop GNSS smoke itinerary, with GNSS errors
0.21223 m and 0.19815 m. The separate three-stop route uses x=24 m, x=12 m,
then the origin, all at y=0. All three stops succeeded, with GNSS errors
0.19174 m, 0.24367 m, and 0.21443 m, through 5, 3, and 3 planned legs
respectively. Its machine-readable report is `2026-11-09-waypoints.json`.
The original saved plan was restored by the diagnostic runner after execution.
During motion, an eight-second range audit observed 435,724 valid lidar returns,
none outside the 1–100 m limits; the farthest was 96.54473 m. Scan, depth,
and local object bounds also passed. See `2026-11-09-sensor-ranges.json`.

Dashboard changes expose readiness blockers, name inactive/unconfigured
lifecycle nodes, reject execution if any reported lifecycle node is inactive,
and display completed stops and the current target. Seventeen dashboard tests
and JavaScript syntax validation passed. No automatic skip, retry, or resumption
of failed goals was introduced. These checks do not qualify the full 1 km route,
physical GNSS localization, or delivery payload handling.
