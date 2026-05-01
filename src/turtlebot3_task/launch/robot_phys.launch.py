#!/usr/bin/env python3
"""
TurtleBot3 Waffle Pi - Full System Launch File with Competition Logic
ROS 2 Humble
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def pkg_file(package: str, *path_parts: str) -> str:
    return os.path.join(get_package_share_directory(package), *path_parts)

def generate_launch_description() -> LaunchDescription:

    # --- 1. NEW COMPETITION ARGUMENTS ---
    red_marker_arg = DeclareLaunchArgument(
        "red_marker",
        default_value="23",
        description="ArUco ID assigned to the RED goal"
    )
    blue_marker_arg = DeclareLaunchArgument(
        "blue_marker",
        default_value="0",
        description="ArUco ID assigned to the BLUE goal"
    )

    # Standard arguments from your file
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    slam_arg = DeclareLaunchArgument(
        "slam", default_value="true",
        description="Launch SLAM Toolbox when true",
    )
    slam_params_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=pkg_file("turtlebot3_navigation2", "param", "waffle_pi.yaml"),
        description="Full path to the SLAM Toolbox parameter file",
    )
    camera_device_arg = DeclareLaunchArgument(
        "camera_device", default_value="/dev/video0",
        description="V4L2 device path for the Raspberry Pi Camera",
    )

    # Configurations
    use_sim_time  = LaunchConfiguration("use_sim_time")
    slam          = LaunchConfiguration("slam")
    slam_params   = LaunchConfiguration("slam_params_file")
    camera_device = LaunchConfiguration("camera_device")
    # New competition configs
    red_id        = LaunchConfiguration("red_marker")
    blue_id       = LaunchConfiguration("blue_marker")

    # --- 2. NEW PARAMETER STORAGE NODE ---
    # This node holds the "Truth" of which ID belongs to which color
    parameter_storage_node = Node(
        package='demo_nodes_cpp',
        executable='parameter_blackboard',
        name='competition_logic',
        parameters=[{
            'red_goal_id': red_id,
            'blue_goal_id': blue_id,
        }]
    )

    # --- 3. EXISTING NODES (UNCHANGED) ---
    # Robot State Publisher
    urdf_file = pkg_file("turtlebot3_description", "urdf", "turtlebot3_waffle_pi.urdf")
    with open(urdf_file, "r") as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
    )

    # TurtleBot3 Base Node
    tb3_param_file = pkg_file("turtlebot3_node", "param", "waffle_pi.yaml")
    turtlebot3_node = Node(
        package="turtlebot3_node",
        executable="turtlebot3_ros",
        name="turtlebot3_node",
        output="screen",
        parameters=[tb3_param_file, {"use_sim_time": use_sim_time}],
        remappings=[("odom", "odom"), ("cmd_vel", "cmd_vel"), ("imu", "imu")],
    )

    # LiDAR Node
    lidar_node = Node(
        package="hls_lfcd_lds_driver",
        executable="hlds_laser_publisher",
        name="hlds_laser_publisher",
        output="screen",
        parameters=[{"port": "/dev/ttyUSB0", "frame_id": "base_scan", "use_sim_time": use_sim_time}],
        remappings=[("scan", "scan")],
    )

    # Camera Node
    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        output="screen",
        parameters=[{
            "video_device": camera_device,
            "image_size": [640, 480],
            "camera_frame_id": "camera_rgb_optical_frame",
            "pixel_format": "YUYV",
            "use_sim_time": use_sim_time,
        }],
        remappings=[("image_raw", "/camera/image_raw"), ("camera_info", "/camera/camera_info")],
    )

    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        condition=IfCondition(slam),
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
        remappings=[("scan", "scan")],
    )

    # --- 4. ASSEMBLE ---
    return LaunchDescription([
        # Arguments
        red_marker_arg,
        blue_marker_arg,
        use_sim_time_arg,
        slam_arg,
        slam_params_arg,
        camera_device_arg,

        LogInfo(msg="=== TurtleBot3 Physical Robot — Competition Mode Ready ==="),

        # Nodes
        parameter_storage_node, # New storage node
        robot_state_publisher,
        turtlebot3_node,
        lidar_node,
        camera_node,
        slam_toolbox_node,
    ])