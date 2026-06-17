"""
teleop.launch.py — Single launch file that brings up the full teleop stack.

Currently runs everything on the rover (onboard computer).
When you set up a base station later, split joy_node + teleop_twist_joy
into a separate launch on the laptop and let /cmd_vel cross via DDS.

Launch with:
    ros2 launch storm_teleop teleop.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('storm_teleop')

    # --- Paths ---
    xacro_file    = os.path.join(pkg_dir, 'description', 'rover.urdf.xacro')
    controllers   = os.path.join(pkg_dir, 'config', 'controllers.yaml')
    joy_teleop    = os.path.join(pkg_dir, 'config', 'joy_teleop.yaml')

    # Process xacro → URDF string. ParameterValue(..., value_type=str) prevents
    # the launch system from re-parsing the XML as YAML.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    # NODES

    # 1) robot_state_publisher: publishes /robot_description + TF from URDF
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # 2) ros2_control_node (controller_manager): loads hardware plugin,
    #    manages controller lifecycle
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers,
        ],
        output='screen',
    )

    # 3) Spawn diff_drive_controller (delayed 2s to let controller_manager start)
    diff_drive_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller'],
                output='screen',
            ),
        ],
    )

    # 4) Spawn joint_state_broadcaster
    jsb_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            ),
        ],
    )

    # 5) joy_node: reads /dev/input/js0 → publishes /joy
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_teleop],
        output='screen',
    )

    # 6) teleop_twist_joy: /joy → /cmd_vel with deadman + turbo
    #    Remap output to match diff_drive_controller's input topic
    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_teleop],
        remappings=[
            ('cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'),
        ],
        output='screen',
    )

    return LaunchDescription([
        robot_state_pub,
        control_node,
        diff_drive_spawner,
        jsb_spawner,
        joy_node,
        teleop_twist_joy,
    ])
