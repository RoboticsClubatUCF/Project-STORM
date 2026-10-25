"""
teleop.launch.py: Single launch file that brings up the full teleop stack.

Runs on the rover.

Controller on the rover:
    ros2 launch storm_teleop teleop.launch.py

Controller on the base station:
    rover:   ros2 launch storm_teleop teleop.launch.py use_joy:=false
    base:    ros2 launch storm_basestation base_station.launch.py

Full-speed turbo (SpaceU): see teleop-spaceu.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _can_iface_down(iface='can0'):
    """Read-only pre-flight check: True if the CAN interface is not up.

    Reads /sys/class/net/<iface>/operstate, never attempts privileged
    bring-up (that's can0_up.sh / storm-can0.service). Missing interface or
    'down' operstate counts as down.
    """
    try:
        with open(f'/sys/class/net/{iface}/operstate') as f:
            return f.read().strip() == 'down'
    except OSError:
        return True  # interface not present


def _launch_setup(context):
    pkg_dir = get_package_share_directory('storm_teleop')

    can_interface = LaunchConfiguration('can_interface').perform(context)
    joy_overrides = LaunchConfiguration('joy_overrides').perform(context)

    # --- Paths ---
    xacro_file    = os.path.join(pkg_dir, 'description', 'rover.urdf.xacro')
    controllers   = os.path.join(pkg_dir, 'config', 'controllers.yaml')
    joy_teleop    = os.path.join(pkg_dir, 'config', 'joy_teleop.yaml')
    sparks_config = os.path.join(pkg_dir, 'config', 'sparks.yaml')

    # Process xacro → URDF string. ParameterValue(..., value_type=str) prevents
    # the launch system from re-parsing the XML as YAML. The spark_config arg
    # hands the SPARK config YAML's absolute path to the hardware plugin.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' spark_config:=', sparks_config,
                 ' can_interface:=', can_interface]),
        value_type=str,
    )

    # --- Read-only CAN pre-flight check ---
    # Warns loudly if the interface is down so the failure is obvious in the
    # log; the real stop is the plugin's require_all_sparks gate. Bring-up
    # itself is can0_up.sh / storm-can0.service (needs privilege).
    can_precheck = []
    if _can_iface_down(can_interface):
        can_precheck.append(LogInfo(msg=(
            f'[storm_teleop] WARNING: {can_interface} is DOWN. Run '
            '`sudo scripts/can0_up.sh` or enable storm-can0.service before '
            'expecting the drivetrain to come up.')))

    # Teleop params: base file, then optional overrides layered on top
    teleop_params = [joy_teleop]
    if joy_overrides:
        teleop_params.append(joy_overrides)

    # ======================== NODES ========================

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

    # 5) joy_node: reads /dev/input/js0 → publishes /joy/raw
    #    joy_watchdog forwards it to /joy while the controller is alive
    #    Both off with use_joy:=false
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_teleop],
        remappings=[('joy', 'joy/raw')],
        condition=IfCondition(LaunchConfiguration('use_joy')),
        output='screen',
    )

    joy_watchdog = Node(
        package='storm_teleop',
        executable='joy_watchdog.py',
        name='joy_watchdog',
        parameters=[joy_teleop],
        condition=IfCondition(LaunchConfiguration('use_joy')),
        output='screen',
    )

    # 6) teleop_twist_joy: /joy → /cmd_vel with deadman + turbo
    #    Remap output to match diff_drive_controller's input topic
    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=teleop_params,
        remappings=[
            ('cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'),
        ],
        output='screen',
    )

    # 7) storm_telemetry: CAN health, computer stats, battery
    telemetry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('storm_telemetry'), 'launch', 'telemetry.launch.py')),
        launch_arguments={'can_interface': can_interface}.items(),
        condition=IfCondition(LaunchConfiguration('enable_telemetry')),
    )

    # 8) foxglove_bridge for Lichtblick
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('storm_basestation'), 'launch', 'bridge.launch.py')),
        condition=IfCondition(LaunchConfiguration('enable_bridge')),
    )

    return [
        *can_precheck,
        robot_state_pub,
        control_node,
        diff_drive_spawner,
        jsb_spawner,
        joy_node,
        joy_watchdog,
        teleop_twist_joy,
        telemetry,
        bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_telemetry', default_value='true',
            description='Run storm_telemetry (CAN health, computer stats, battery).',
        ),
        DeclareLaunchArgument(
            'enable_bridge', default_value='true',
            description='Run foxglove_bridge for Lichtblick (ws://<rover>:8765).',
        ),
        DeclareLaunchArgument(
            'use_joy', default_value='true',
            description='Run joy_node on the rover. Set false when using the base station.',
        ),
        DeclareLaunchArgument(
            'can_interface', default_value='can0',
            description='SocketCAN interface for the SPARKs (plugin + health node).',
        ),
        DeclareLaunchArgument(
            'joy_overrides', default_value='',
            description='Optional YAML layered over joy_teleop.yaml for teleop_twist_joy.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
