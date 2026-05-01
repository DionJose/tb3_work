#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.resources import has_resource

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # -------------------------------
    # ENV VARIABLES (TB3)
    # -------------------------------
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')
    LDS_MODEL = os.environ.get('LDS_MODEL', 'LDS-01')

    # -------------------------------
    # TB3 PARAMETERS
    # -------------------------------
    namespace = LaunchConfiguration('namespace', default='')
    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    if ROS_DISTRO == 'humble':
        tb3_param_dir = os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            ROS_DISTRO,
            TURTLEBOT3_MODEL + '.yaml')
    else:
        tb3_param_dir = os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            TURTLEBOT3_MODEL + '.yaml')

    # -------------------------------
    # LIDAR SETUP
    # -------------------------------
    if LDS_MODEL == 'LDS-01':
        lidar_pkg_dir = get_package_share_directory('hls_lfcd_lds_driver')
        lidar_launch = 'hlds_laser.launch.py'
    elif LDS_MODEL == 'LDS-02':
        lidar_pkg_dir = get_package_share_directory('ld08_driver')
        lidar_launch = 'ld08.launch.py'
    elif LDS_MODEL == 'LDS-03':
        lidar_pkg_dir = get_package_share_directory('coin_d4_driver')
        lidar_launch = 'single_lidar_node.launch.py'
    else:
        lidar_pkg_dir = get_package_share_directory('hls_lfcd_lds_driver')
        lidar_launch = 'hlds_laser.launch.py'

    # -------------------------------
    # CAMERA PARAMETERS
    # -------------------------------
    camera = LaunchConfiguration('camera', default='0')
    width = LaunchConfiguration('width', default='640')
    height = LaunchConfiguration('height', default='480')
    format_param = LaunchConfiguration('format', default='')
    use_image_view = LaunchConfiguration('use_image_view', default='false')

    # -------------------------------
    # CAMERA NODES (COMPOSABLE)
    # -------------------------------
    composable_nodes = [
        ComposableNode(
            package='camera_ros',
            plugin='camera::CameraNode',
            parameters=[{
                'camera': camera,
                'sensor_mode': '1640:1232',
                'width': width,
                'height': height,
                'format': format_param,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    if has_resource('packages', 'image_view'):
        composable_nodes.append(
            ComposableNode(
                package='image_view',
                plugin='image_view::ImageViewNode',
                remappings=[('/image', '/camera/image_raw')],
                condition=IfCondition(use_image_view),
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )

    camera_container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )
    
    image_compressed_republisher = Node(
    package='image_transport',
    executable='republish',
    name='image_compressed_republisher',
    arguments=[
        'raw',
        'compressed',
        '--ros-args',
        '-r', 'in:=/camera/image_raw',
        '-r', 'out:=/camera/image_raw/compressed'
    ],
    output='screen'
    )

    # -------------------------------
    # RETURN LAUNCH DESCRIPTION
    # -------------------------------
    return LaunchDescription([

        # ---- Arguments ----
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('usb_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        DeclareLaunchArgument('camera', default_value='0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('format', default_value=''),
        DeclareLaunchArgument('use_image_view', default_value='false'),

        # ---- Namespace ----
        PushRosNamespace(namespace),

        # ---- Robot State Publisher ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # ---- LiDAR ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [lidar_pkg_dir, '/launch/', lidar_launch]),
            launch_arguments={
                'port': '/dev/ttyUSB0',
                'frame_id': 'base_scan'
            }.items(),
        ),

        # ---- TurtleBot3 Node ----
        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'
        ),

        # ---- Camera Container ----
        camera_container,
        image_compressed_republisher,
    ])
