#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Set environment variables for models
    pkg_aice_sim = get_package_share_directory('turtlebot3_task')
    models_path = os.path.join(pkg_aice_sim, 'models')
    os.environ['GAZEBO_RESOURCE_PATH'] = models_path + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')
    os.environ['GAZEBO_MODEL_PATH']    = models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')

    # Package paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    tb3_launch_dir = os.path.join(pkg_tb3_gazebo, 'launch')

    # Launch Configurations
    world_file = os.path.join(pkg_aice_sim, 'worlds', 'world.world')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    # Competition Logic Arguments (Assigning Color to ID)
    red_id = LaunchConfiguration('red_marker', default='23')
    blue_id = LaunchConfiguration('blue_marker', default='0')

    declare_red_marker = DeclareLaunchArgument(
        'red_marker', default_value='23',
        description='ArUco ID assigned to the RED goal')
    declare_blue_marker = DeclareLaunchArgument(
        'blue_marker', default_value='0',
        description='ArUco ID assigned to the BLUE goal')

    parameter_storage_node = Node(
    package='demo_nodes_cpp',
    executable='parameter_blackboard',
    name='competition_logic',
    parameters=[{
        'red_goal_id': LaunchConfiguration('red_marker'),
        'blue_goal_id': LaunchConfiguration('blue_marker'),
    }]
)

    # Gazebo Server & Client
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file, 'extra_gazebo_args': '--verbose'}.items())

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')))

    # Robot State Publisher - We attach the parameters here
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'red_goal_id': red_id,
            'blue_goal_id': blue_id
        }.items())

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items())

    return LaunchDescription([
        declare_red_marker,
        declare_blue_marker,
        parameter_storage_node,
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_turtlebot,
    ])