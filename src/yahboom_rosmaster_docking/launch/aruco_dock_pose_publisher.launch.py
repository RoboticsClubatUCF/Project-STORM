#!/usr/bin/env python3
"""Launch the camera rectifier, ArUco tracker, and Nav2 dock-pose adapter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name_docking = 'yahboom_rosmaster_docking'
    config_path = PathJoinSubstitution([
        FindPackageShare(package=package_name_docking),
        'config',
        'aruco_tracker.yaml',
    ])

    aruco_config_file = LaunchConfiguration('aruco_config_file')
    camera_frame_type = LaunchConfiguration('camera_frame_type')
    camera_namespace = LaunchConfiguration('camera_namespace')
    marker_id = LaunchConfiguration('marker_id')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declarations = [
        DeclareLaunchArgument(
            'aruco_config_file', default_value=config_path,
            description='Full path to the ArUco tracker parameter file'),
        DeclareLaunchArgument(
            'camera_frame_type', default_value='_depth_optical_frame',
            description='Camera optical frame suffix used by this robot'),
        DeclareLaunchArgument(
            'camera_namespace', default_value='cam_1',
            description='Namespace containing the camera topics and tracker'),
        DeclareLaunchArgument(
            'marker_id', default_value='0',
            description='ArUco marker ID used for the docking station'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time when true'),
    ]

    rectify_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='rectify_node',
        namespace=camera_namespace,
        remappings=[
            ('image', 'color/image_raw'),
            ('image_rect', 'color/image_rect'),
            ('camera_info', 'color/camera_info'),
        ],
        parameters=[{
            'queue_size': 5,
            'interpolation': 1,
            'use_sim_time': use_sim_time,
            'image_transport': 'raw',
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    rectify_container = ComposableNodeContainer(
        name='camera_image_processing',
        namespace=camera_namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[rectify_node],
        output='screen',
    )

    aruco_tracker = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        namespace=camera_namespace,
        name='aruco_tracker',
        parameters=[aruco_config_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    dock_pose_publisher = Node(
        package='yahboom_rosmaster_docking',
        executable='detected_dock_pose_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'parent_frame': [
                camera_namespace, TextSubstitution(text=''), camera_frame_type,
            ],
            'child_frame': ['marker_', TextSubstitution(text=''), marker_id],
            'publish_rate': 10.0,
        }],
        output='screen',
    )

    return LaunchDescription([
        *declarations,
        rectify_container,
        aruco_tracker,
        dock_pose_publisher,
    ])
