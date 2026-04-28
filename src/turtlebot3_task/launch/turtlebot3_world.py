#!/usr/bin/env python3
"""
turtlebot3_world.launch.py
-------------
Launches a Gazebo Classic simulation with the AICE2011 v3_block arena and
spawns a TurtleBot3 waffle_pi in the centre of the arena.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Set environment variables at module load time so they apply to all subprocesses
pkg_aice_sim = get_package_share_directory('turtlebot3_task')
models_path = os.path.join(pkg_aice_sim, 'models')

os.environ['GAZEBO_RESOURCE_PATH'] = models_path + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')
os.environ['GAZEBO_MODEL_PATH']    = models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    tb3_launch_dir = os.path.join(pkg_tb3_gazebo, 'launch')

    world_file = os.path.join(pkg_aice_sim, 'worlds', 'world.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose       = LaunchConfiguration('x_pose',       default='0.0')
    y_pose       = LaunchConfiguration('y_pose',       default='0.0')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock')
    declare_x_pose = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='TurtleBot3 spawn X position')
    declare_y_pose = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='TurtleBot3 spawn Y position')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'extra_gazebo_args': '--verbose',
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_x_pose,
        declare_y_pose,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_turtlebot,
    ])