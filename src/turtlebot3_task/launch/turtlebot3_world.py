#!/usr/bin/env python3
"""
turtlebot3_world.launch.py
-------------
Launches a Gazebo Classic simulation with the AICE2011 v3_block arena and
spawns a TurtleBot3 waffle_pi in the centre of the arena.

Usage:
    ros2 launch ros2_ws turtlebot3_world.launch.py
    ros2 launch ros2_ws sim.launch.py x_pose:=0.0 y_pose:=0.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Package share directories
    pkg_aice_sim   = get_package_share_directory('turtlebot3_task')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    tb3_launch_dir = os.path.join(pkg_tb3_gazebo, 'launch')

    # Models path (install location, works for any username)
    models_path = os.path.join(pkg_aice_sim, 'models')

    # Fix hardcoded paths in world file at launch time
    world_src = os.path.join(pkg_aice_sim, 'worlds', 'world.world')
    with open(world_src, 'r') as f:
        world_contents = f.read()

    # Replace any hardcoded /home/<whoever>/ros2_ws/... with the actual install path
    import re
    world_contents = re.sub(
        r'file:///home/[^/]+/ros2_ws/[^"]+/models/([^/]+)/materials/scripts/([^"]+)',
        lambda m: f'file://{models_path}/{m.group(1)}/materials/scripts/{m.group(2)}',
        world_contents
    )

    # Write fixed world to a temp file
    import tempfile
    tmp_world = tempfile.NamedTemporaryFile(
        mode='w', suffix='.world', delete=False
    )
    tmp_world.write(world_contents)
    tmp_world.flush()
    world_file = tmp_world.name

    # Launch arguments
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

    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'extra_gazebo_args': '--verbose',
        }.items(),
    )

    # Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # robot_state_publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Spawn TurtleBot3
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
        # Set GAZEBO_RESOURCE_PATH automatically — no ~/.bashrc edit needed
        SetEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH',
            models_path + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')
        ),
        declare_use_sim_time,
        declare_x_pose,
        declare_y_pose,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_turtlebot,
    ])