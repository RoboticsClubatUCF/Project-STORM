# Headless long-distance validation

`devscripts/test_headless_mission.py` runs the existing three-stop WGS84 mission
(`near`, `middle`, then `far` at approximately 1,000.56 m east of the datum)
through the actual mission runner, Nav2, rover dynamics, and rendered sensors.
Gazebo runs as a server without a GUI; RViz is not launched. EGL rendering is
the default. `--render-backend display` uses an existing WSLg/X display for
sensor rendering while still running only the server.
The output directory must not exist, so earlier evidence cannot be overwritten.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 devscripts/test_headless_mission.py \
  --output docs/validation/headless-1km-$(date +%Y%m%d-%H%M%S) \
  --rate 4 --domain 78
```

Use a ROS domain without other rover publishers. Each run also has its own
Gazebo transport partition. The runner checks lifecycle activation, the drive
controller, GNSS, lidar, the derived scan, local objects, truth, and clock before
sending any goals. It checkpoints progress every 15 wall seconds. The default
wall deadline is two hours. Interrupting the runner requests cooperative mission
cancellation, waits for it, and stops only its captured simulation descendants.

Only a generated world copy's target real-time factor changes. The 1 ms physics
step, rover velocity limits, VLP-16 range, depth limits, terrain, and collision
checks remain those of the installed simulation. The measured acceleration is
reported separately from the requested factor, since available CPU/GPU capacity
can prevent reaching the target. The script does not teleport the rover or
increase its commanded driving speed. Rendering remains enabled for the sensors.

Evidence includes `simulation.log`, `mission.log`, `mission.json`, a snapshot
of `waypoints.json`, the generated `world.sdf`, and a continually replaced
`status.json`. A passing result requires all three ordered GNSS stops to succeed
under the mission runner's 1 m arrival criterion and final Gazebo truth to reach
the kilometre region. The report records actual wall/simulation duration,
achieved factor, message counts, observed Collision Monitor state transitions,
and source world/mission SHA-256 values. Collision state transitions are command
interventions, not measurements of physical contact or a certificate of collision
freedom. Navigation uses simulation truth localization. This test does not
validate physical GNSS fusion, manipulation/payloads, radio range, negative
obstacles, or complete URC delivery compliance.

Dependencies are the built project's ROS/Gazebo packages, Python standard library,
`rclpy`, `ament_index_python`, and the existing `storm_dashboard` termination
helper and its declared dependencies. If using the development `pyproj` fallback,
use the same `PYTHONPATH` setup as `devscripts/run_dashboard.sh`.

On this WSL machine, Mesa initially selected `llvmpipe` (software rendering),
including through the display backend. For a process-scoped NVIDIA run:

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
  python3 devscripts/test_headless_mission.py \
    --output docs/validation/headless-1km-nvidia \
    --rate 4 --domain 78 --render-backend display
```

This does not change system graphics settings. Check `renderer` in `status.json`
for the actual selected device. Microsoft documents the adapter selector in
[GPU selection in WSLg](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg).

Gazebo references: [headless rendering](https://gazebosim.org/api/sim/8/headless_rendering.html)
and [SDF physics configuration](https://gazebosim.org/docs/harmonic/sdf_worlds/).
