# !/bin/bash

# Create a session and start the first process in window 0
tmux new-session -d -s background_jobs -n 'gz' 'ign gazebo -v 4 -r visualize_lidar.sdf'

# Create a second window (window 1) running gz-bridge and foxglove
tmux new-window -t background_jobs:1 -n 'gz-bridge-car' 'ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
tmux new-window -t background_jobs:2 -n 'gz-bridge-lidar' 'ros2 run ros_gz_bridge parameter_bridge /lidar2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar2:=/laser_scan'
tmux new-window -t background_jobs:3 -n 'foxglove-websockets' 'ros2 launch foxglove_bridge foxglove_bridge_launch.xml'

# Attach to the session
tmux attach-session -t background_jobs