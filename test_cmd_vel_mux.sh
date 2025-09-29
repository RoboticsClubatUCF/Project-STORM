#!/bin/bash

# Test script for cmd_vel_mux
echo "Testing cmd_vel_mux package..."

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "1. Starting cmd_vel_mux node..."
ros2 run cmd_vel_mux cmd_vel_mux_node --ros-args --params-file src/cmd_vel_mux/config/cmd_vel_mux_params.yaml &
CMD_VEL_MUX_PID=$!

# Wait for node to start
sleep 3

echo "2. Checking available topics..."
ros2 topic list | grep -E "(cmd_vel|input)"

echo "3. Checking cmd_vel_mux node info..."
ros2 node info /cmd_vel_mux

echo "4. Testing message publication..."
echo "Publishing to high priority input (joystick)..."
ros2 topic pub /input/joystick geometry_msgs/msg/Twist "linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}" --times 5 &

sleep 2

echo "5. Checking output topic..."
timeout 5 ros2 topic echo /cmd_vel_mux/output --no-arr

echo "6. Cleaning up..."
kill $CMD_VEL_MUX_PID
pkill -f "ros2 topic pub"

echo "Test completed!"