# Nav2 navigation

This guide follows Automatic Addison's Jazzy tutorials for [installing Nav2](https://automaticaddison.com/how-to-install-ros-2-navigation-nav2-ros-2-jazzy/), [autonomous mobile robot navigation](https://automaticaddison.com/autonomous-navigation-for-a-mobile-robot-using-ros-2-jazzy/), and the [Simple Commander API](https://automaticaddison.com/getting-started-with-the-simple-commander-api-ros-2-jazzy/).

## Install Nav2 and dependencies

The launch file depends on `nav2_bringup`, Nav2 plugins, SLAM Toolbox, robot_localization, and Gazebo integration. Install dependencies with rosdep as described in [setup and build](00-setup-and-build.md). If rosdep cannot resolve an apt package on your installation, install the missing Jazzy package by name using the Jazzy package index.

## Start with a saved map

The repository includes example map YAML and image pairs for the cafe and house Gazebo worlds. Run the integrated launch with the map that matches the chosen world:

```bash
ros2 launch yahboom_rosmaster_bringup rosmaster_x3_navigation.launch.py \
  slam:=False \
  map:=$(ros2 pkg prefix --share yahboom_rosmaster_navigation)/maps/cafe_world_map.yaml \
  world_file:=cafe.world
```

For the house example, use `house_world_map.yaml` and `house.world`. The map and simulated world need to describe the same layout.

## Set a goal

The launch starts Nav2, RViz, the velocity relay, and the repository's `nav_to_pose.py` helper. In RViz, set the initial pose if required, then use the Nav2 goal tool to send a goal. The helper scripts in `yahboom_rosmaster_navigation/scripts/` provide examples of goal submission through the Simple Commander API.

## Controller configuration

The main launch uses `config/rosmaster_x3_nav2_default_params.yaml`. An additional Regulated Pure Pursuit profile is available at `config/rosmaster_x3_nav2_regulated_pure_pursuit_controller.yaml` for comparison and tuning. Keep the robot footprint, velocity limits, sensor topics, and controller selection consistent with the mecanum base and URDF.

## Save and reuse a map

For a map created with SLAM Toolbox, use its YAML file in the `map:=` argument. Keep the image and YAML together and pass an absolute YAML path when the map is outside the installed package share directory.
