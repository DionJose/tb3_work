#!/usr/bin/env python3
"""
TurtleBot3 Waffle Pi - Full System Launch File
ROS 2 Humble

Launches:
  - Robot state publisher (URDF / TF tree)
  - LiDAR (LDS-02 via hls_lfcd_lds_driver)
  - Raspberry Pi Camera v2 (via v4l2_camera)
  - IMU + base sensors (turtlebot3_node)
  - SLAM Toolbox (online async mode)
  - Image compressed republisher (image_transport)

Usage:
  ros2 launch turtlebot3_waffle_pi_launch.py
  ros2 launch turtlebot3_waffle_pi_launch.py slam:=false   # skip SLAM
  ros2 launch turtlebot3_waffle_pi_launch.py use_sim_time:=true  # Gazebo
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ---------------------------------------------------------------------------
# Helper: resolve a file path inside a ROS 2 package share directory
# ---------------------------------------------------------------------------
def pkg_file(package: str, *path_parts: str) -> str:
    return os.path.join(get_package_share_directory(package), *path_parts)


def generate_launch_description() -> LaunchDescription:

    # -----------------------------------------------------------------------
    # Launch arguments
    # -----------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="true",
        description="Launch SLAM Toolbox when true",
    )
    slam_params_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=pkg_file("turtlebot3_navigation2", "param", "waffle_pi.yaml"),
        description="Full path to the SLAM Toolbox parameter file",
    )
    camera_device_arg = DeclareLaunchArgument(
        "camera_device",
        default_value="/dev/video0",
        description="V4L2 device path for the Raspberry Pi Camera",
    )

    # Convenience references to launch configuration values
    use_sim_time  = LaunchConfiguration("use_sim_time")
    slam          = LaunchConfiguration("slam")
    slam_params   = LaunchConfiguration("slam_params_file")
    camera_device = LaunchConfiguration("camera_device")

    # -----------------------------------------------------------------------
    # 1. Robot State Publisher  (URDF + TF tree)
    # -----------------------------------------------------------------------
    urdf_file = pkg_file(
        "turtlebot3_description", "urdf", "turtlebot3_waffle_pi.urdf"
    )
    robot_description = Command(["cat ", urdf_file])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }
        ],
    )

    # -----------------------------------------------------------------------
    # 2. TurtleBot3 Base Node  (OpenCR board — motors, odometry, IMU)
    # -----------------------------------------------------------------------
    tb3_param_file = pkg_file(
        "turtlebot3_node", "param", "waffle_pi.yaml"
    )
    turtlebot3_node = Node(
        package="turtlebot3_node",
        executable="turtlebot3_ros",
        name="turtlebot3_node",
        output="screen",
        parameters=[
            tb3_param_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("odom", "odom"),
            ("cmd_vel", "cmd_vel"),
            ("imu", "imu"),
        ],
    )

    # -----------------------------------------------------------------------
    # 3. LiDAR  (LDS-02 — default on Waffle Pi)
    #    Driver: hls_lfcd_lds_driver  →  publishes /scan
    # -----------------------------------------------------------------------
    lidar_node = Node(
        package="hls_lfcd_lds_driver",
        executable="hlds_laser_publisher",
        name="hlds_laser_publisher",
        output="screen",
        parameters=[
            {
                "port": "/dev/ttyUSB0",   # change if your LiDAR is on a different port
                "frame_id": "base_scan",
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[("scan", "scan")],
    )

    # -----------------------------------------------------------------------
    # 4. Camera  (Raspberry Pi Camera v2 via v4l2_camera)
    #    Publishes: /image_raw, /camera_info
    # -----------------------------------------------------------------------
    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        output="screen",
        parameters=[
            {
                "video_device": camera_device,
                "image_size": [640, 480],
                "image_width": 640,
                "image_height": 480,
                "camera_frame_id": "camera_rgb_optical_frame",
                "pixel_format": "YUYV",
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[
            ("image_raw", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
        ],
    )

    # -----------------------------------------------------------------------
    # 5. SLAM Toolbox  (online async — builds a map in real time)
    # -----------------------------------------------------------------------
    slam_toolbox_node = Node(
        condition=IfCondition(slam),
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("scan", "scan")],
    )

    # -----------------------------------------------------------------------
    # 6. Image Compressed Republisher  (image_transport)
    #    Subscribes: /camera/image_raw  (raw)
    #    Publishes:  /camera/image_raw/compressed  (compressed)
    # -----------------------------------------------------------------------
    image_compressed_republisher = Node(
        package="image_transport",
        executable="republish",
        name="image_compressed_republisher",
        output="screen",
        arguments=["raw", "compressed"],
        remappings=[
            ("in",  "/camera/image_raw"),
            ("out/compressed", "/camera/image_raw/compressed"),
        ],
    )

    # -----------------------------------------------------------------------
    # Assemble LaunchDescription
    # -----------------------------------------------------------------------
    return LaunchDescription(
        [
            # Arguments
            use_sim_time_arg,
            slam_arg,
            slam_params_arg,
            camera_device_arg,

            # Info banner
            LogInfo(msg="=== TurtleBot3 Waffle Pi — ROS 2 Humble — Full Launch ==="),

            # Nodes
            robot_state_publisher,
            turtlebot3_node,
            lidar_node,
            camera_node,
            slam_toolbox_node,
            image_compressed_republisher,
        ]
    )
