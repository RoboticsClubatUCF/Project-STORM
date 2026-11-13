# Adding components

Add functionality to the package that owns its interface. Keep robot geometry
in the description, simulator plugins in Gazebo, and application launch
composition in bringup. Give new project packages the `storm_` prefix.

## Package template

From the workspace root in a Jazzy shell:

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg create --destination-directory src --build-type ament_cmake \
  --license MIT storm_example
```

Replace the example name and maintainer details. Declare direct dependencies in
`package.xml`, including `exec_depend` for packages used by launch files and
`test_depend` for test tools. For a package with `launch/` and `config/`
directories, add this before `ament_package()` in `CMakeLists.txt`:

```cmake
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
```

Resolve installed paths with `get_package_share_directory` in Python launch
files. Expose runtime choices as launch arguments and keep tunable parameters
in YAML. Test the package alone before including it in application bringup.

## Integration checklist by component

To add a geographic practice itinerary, create a version-1 WGS84 JSON file
under `storm_mission/missions` and rebuild that package. Keep mission execution
above Nav2's action interface. See [GNSS waypoint missions](GNSS_WAYPOINTS.md)
for schema, projection assumptions, and the planned localization interface.

| Component | Changes | Verify |
| --- | --- | --- |
| Sensor | Add the measured mount and named frame to `storm_description`. Put simulator plugins and bridge rules in `storm_gazebo`. Launch the physical driver separately or include it in robot bringup. | Topic type, frame ID, timestamp, rate, and TF to `base_link`. |
| Hardware backend | Add a ros2_control plugin and plugin XML, declare its dependencies, and wire its backend selection into the description and control launch. Match joint names and SI interfaces. | Plugin discovery, valid and invalid configuration, lifecycle transitions, and feedback before enabling physical motion. |
| Controller | Configure its plugin and joints in `storm_control/config/controllers.yaml`. Add activation to the manager that owns the selected backend. | Controller activation, interface claims, command type, timeout, and odometry ownership. |
| Nav2 plugin or tuning | Update the relevant server in a copy of the arena or MDRS parameter file. Declare the plugin dependency. Pass the file with `params_file:=/absolute/path/config.yaml`. | Server activation and a representative goal. Check recovery behavior if the change affects it. |
| Test world | Add the SDF and required installed assets under `storm_gazebo`. Use `world:=/absolute/path/world.sdf` and a suitable `spawn_z`. | Collision, spawn pose, clock, lidar, and a short goal. Use a map generated for that world when running AMCL. |

Follow the existing frame ownership in the [architecture guide](ARCHITECTURE.md).
For example, an estimator that takes over `odom → base_footprint` also requires
disabling the controller's TF broadcast. Keep simulated time consistent across
the graph and retain the stamped command interface unless all consumers are
updated together.

```bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --packages-up-to storm_example
source install/setup.bash
colcon test --packages-select storm_example
colcon test-result --verbose
```

Use [the test guide](TESTING.md) for integration checks after wiring the
component into bringup.
