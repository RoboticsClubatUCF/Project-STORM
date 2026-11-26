# Storm ARM

This workspace adds a small simulated three-joint arm to the ROS Jazzy devcontainer starter. MoveIt plans arm trajectories, `ros2_control`'s mock hardware executes them, and RViz shows the robot, interactive goal, and planned path. No physical arm is required.

## Start the demo

1. Open this repository in VS Code and reopen it in the devcontainer. On Windows, use the existing WSL/NVIDIA profile at `.devcontainer/wsl/devcontainer.json` (with Docker Desktop's WSL 2 integration and GUI support configured).
2. In the container terminal, build and source the workspace:

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

3. Launch RViz and MoveIt:

   ```bash
   ros2 launch storm_arm demo.launch.py
   ```

In RViz, use the **MotionPlanning** panel. Move the orange goal marker, choose **Plan**, then **Plan & Execute** to animate the arm using the simulated controller. The `home` and `folded` named states are available as planning targets.

For a headless ROS/MoveIt startup check, append `launch_rviz:=false` to the launch command.

## Arm model

The arm has a base yaw joint and shoulder/elbow pitch joints. Its planning group is `arm`, with `tool0` as the end-effector link. The KDL inverse-kinematics solver has only three degrees of freedom, so it cannot satisfy arbitrary six-dimensional pose targets; use reachable marker poses and expect some orientations not to solve.
