# Robot description and RViz

This guide follows Automatic Addison's [mobile robot URDF tutorial for Jazzy](https://automaticaddison.com/create-and-visualize-a-mobile-robot-with-urdf-ros-2-jazzy/). The robot model is written as Xacro and split into the base, mecanum wheels, sensors, control, and top-level robot files.

## Where the model lives

- Top-level robot: `yahboom_rosmaster_description/urdf/robots/rosmaster_x3.urdf.xacro`
- Base and wheel geometry: `yahboom_rosmaster_description/urdf/mech/`
- Camera, lidar, and IMU: `yahboom_rosmaster_description/urdf/sensors/`
- ros2_control and simulator plugins: `yahboom_rosmaster_description/urdf/control/`
- Visual meshes: `yahboom_rosmaster_description/meshes/`

Keep mesh paths package-relative and preserve the Xacro include structure when editing the model. The description package installs its Xacro, mesh, configuration, launch, and RViz files into the package share directory.

## View the robot

From a sourced Jazzy workspace:

```bash
ros2 launch yahboom_rosmaster_description robot_state_publisher.launch.py
```

The launch file starts robot_state_publisher and RViz by default. Useful launch arguments include `robot_name`, `prefix`, `use_rviz`, `use_jsp`, `jsp_gui`, and `use_sim_time`. For a static model view, leave simulated time disabled.

To inspect the generated transforms in another terminal:

```bash
ros2 topic echo /robot_description
ros2 run tf2_tools view_frames
```

## Changes to the model

When adding a link or sensor, update the Xacro include and its fixed or movable joint, then confirm its frame appears in RViz. Keep the robot's visual and collision geometry separate: detailed meshes are useful for display, while simple collision geometry improves simulation stability.
