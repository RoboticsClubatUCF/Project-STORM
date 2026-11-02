# Running tests

Run commands from the Linux checkout in Ubuntu 24.04. Source the same overlay
and set the same ROS domain in every terminal used for a runtime test.

## Package tests

```bash
cd ~/Project-STORM-jazzy
source /opt/ros/jazzy/setup.bash
bash devscripts/build.sh
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

| Test suite | Coverage |
| --- | --- |
| `storm_description` | Xacro expands all three backends with matching six-wheel interfaces and frames |
| `storm_navigation` | Stamped command settings, rover footprint, and relocatable saved map |
| `storm_hardware` | CAN configuration accepts valid wheel data and rejects invalid gear ratio, IDs, and interfaces |

Use `colcon test --packages-select storm_description storm_navigation` to run
only the description and navigation checks. For the CAN suite, first run
`bash devscripts/build_hardware.sh` and source the overlay again, then run
`colcon test --packages-select storm_hardware`. These tests exercise driver
initialization without configuring or commanding physical motors. Inspect
`build/<package>/test_results/` for XML and `log/latest_test/` for console output.

## Runtime smoke tests

In both terminals, use an available test domain and Gazebo partition:

```bash
cd ~/Project-STORM-jazzy
source install/setup.bash
export ROS_DOMAIN_ID=54 GZ_PARTITION=storm_test
```

Launch one scenario at a time in the first terminal. Stop it with Ctrl+C
before moving to the next scenario.

| Scenario | Launch |
| --- | --- |
| Mock base | `ros2 launch storm_bringup robot.launch.py hardware:=mock` |
| Arena SLAM | `ros2 launch storm_bringup simulation.launch.py gui:=false rviz:=false` |
| Arena AMCL | `ros2 launch storm_bringup simulation.launch.py gui:=false rviz:=false mode:=localization map:=$(ros2 pkg prefix --share storm_navigation)/maps/storm_arena.yaml` |
| MDRS terrain | `ros2 launch storm_bringup simulation.launch.py gui:=false rviz:=false environment:=urc` |

Check control in the second terminal:

```bash
ros2 control list_controllers
ros2 topic info /cmd_vel --verbose
timeout 10 ros2 topic echo /odom --once --field child_frame_id
```

Expect both controllers to be active, a `TwistStamped` subscription on
`/cmd_vel`, and `base_footprint` as the odometry child frame. For the Nav2
scenarios, also check lidar, TF, and the navigation lifecycle state:

```bash
timeout 10 ros2 topic echo /scan --once --field header
timeout 10 ros2 run tf2_ros tf2_echo map base_footprint
ros2 lifecycle get /bt_navigator
```

For arena AMCL, initialize the pose at the simulated origin before checking
`map → base_footprint` or sending a goal:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}'
```

With the arena navigator active, send a short goal and expect `SUCCEEDED`:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}'
```

For MDRS, the previously tested goal was `(-1.0, 1.0)` with quaternion
`z=0.9238795, w=0.3826834`. Choose a goal inside the observed costmap.
An action result measures success in the navigation frame. Compare against
Gazebo ground truth when evaluating outdoor position accuracy.

If the navigator remains inactive, check the scan and odometry first, then the
transform from `map` to `base_footprint`. If a CLI command sees no topics,
verify the sourced overlay and ROS domain in both terminals. Historical
outcomes are recorded in [validation results](VALIDATION.md).
