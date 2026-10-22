# Development and troubleshooting

For workspace, build, launch, and diagnostic shortcuts, see the [project Bash aliases guide](08-project-aliases.md).

## Build and inspect

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash
```

Build a package and its dependencies with `colcon build --packages-up-to <package_name>`. To find launch files and executables:

```bash
ros2 pkg executables yahboom_rosmaster_navigation
ros2 pkg prefix --share yahboom_rosmaster_description
ros2 launch yahboom_rosmaster_gazebo yahboom_rosmaster.gazebo.launch.py --show-args
```

## Useful diagnostics

```bash
ros2 doctor --report
ros2 node list
ros2 topic list
ros2 topic info /scan
ros2 topic echo /tf
ros2 run tf2_tools view_frames
```

If a launch file cannot find a package, source `/opt/ros/jazzy/setup.bash` and then `~/ros2_ws/install/setup.bash` in that terminal. If a package or launch file is not discovered after a build, confirm the build completed and re-source the overlay.

## Common issues

- **Missing ROS packages:** run `rosdep install --from-paths src --ignore-src --rosdistro jazzy -y` from `~/ros2_ws`, then rebuild.
- **Map not found:** pass the full path to a map YAML file with `map:=`; verify that the referenced image file sits next to the YAML.
- **No robot motion in simulation:** check that controllers loaded with `ros2 control list_controllers`, and that the velocity command reaches the controller through the relay.
- **No lidar data:** inspect `/scan`, then check the Gazebo bridge configuration and the lidar plugin in the Xacro model.
- **TF extrapolation or missing transforms:** use consistent `use_sim_time` values, inspect the TF tree, and verify that the EKF and robot state publisher are running.
- **ArUco marker pose missing:** verify the camera namespace, rectified image and camera-info topics, `4X4_50` dictionary, marker ID, camera calibration, and camera optical frame. Inspect `/tf` for the expected `marker_<id>` child frame.
