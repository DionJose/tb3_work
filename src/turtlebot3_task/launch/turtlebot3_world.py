#!/usr/bin/env python3
"""
turtlebot3_world.py  (ROS 2 launch file)
-----------------------------------------
Launches Gazebo with the AICE2011 arena, spawns TurtleBot3, and
publishes the colour→wall-ID assignment so navigation.py can read it.

Usage
-----
  ros2 launch turtlebot3_task turtlebot3_world.py

  ros2 launch turtlebot3_task turtlebot3_world.py \
      marker_north:=7  marker_south:=42 \
      marker_east:=23  marker_west:=0   \
      red_marker:=23   blue_marker:=0   \
      x_pose:=0.0      y_pose:=0.0

Argument summary
----------------
  marker_north/south/east/west : ArUco IDs physically on each wall in Gazebo
  red_marker                   : which of those IDs is the RED goal wall
  blue_marker                  : which of those IDs is the BLUE goal wall
  x_pose / y_pose              : robot spawn position

The wall_id_publisher node publishes [red_marker, blue_marker] on
/arena/wall_ids (Int32MultiArray) at 1 Hz so navigation.py receives
the assignment even if it starts after the launch file.
"""

import os
import re
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ── Package paths ────────────────────────────────────────────────────────────
pkg_aice_sim = get_package_share_directory('turtlebot3_task')
models_path  = os.path.join(pkg_aice_sim, 'models')

os.environ['GAZEBO_RESOURCE_PATH'] = (
    models_path + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', ''))
os.environ['GAZEBO_MODEL_PATH'] = (
    models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''))

_WORLD_TEMPLATE = os.path.join(pkg_aice_sim, 'worlds', 'world.world')

_DEFAULT_NORTH = 7
_DEFAULT_SOUTH = 42
_DEFAULT_EAST  = 23
_DEFAULT_WEST  = 0


# ── World SDF patching ───────────────────────────────────────────────────────

def _patch_world(template_path, models_dir, north, south, east, west):
    """
    Replace marker IDs and switch model:// URIs to absolute file:// paths.
    Scopes every substitution to the correct wall block so swapped/duplicate
    IDs never corrupt each other.
    """
    with open(template_path, 'r') as fh:
        sdf = fh.read()

    walls = [
        ('arena_wall_north', 'wall_north', north),
        ('arena_wall_south', 'wall_south', south),
        ('arena_wall_east',  'wall_east',  east),
        ('arena_wall_west',  'wall_west',  west),
    ]

    for uri_prefix, model_name, new_id in walls:
        abs_material = '{}/{}/materials/scripts/marker_{}.material'.format(
            models_dir, uri_prefix, new_id)

        # URI: replace model:// or file:// with absolute file:// path
        sdf = re.sub(
            r'(?:model|file)://{}/materials/scripts/marker_\d+\.material'.format(
                re.escape(uri_prefix)),
            'file://' + abs_material,
            sdf, count=1,
        )

        # Material name: scoped inside this wall's <model> block only
        def replace_in_block(m, nid=new_id):
            return re.sub(r'ArUcoMarker\d+', 'ArUcoMarker{}'.format(nid),
                          m.group(0), count=1)
        sdf = re.sub(
            r'<model name="{}".*?</model>'.format(re.escape(model_name)),
            replace_in_block, sdf, count=1, flags=re.DOTALL,
        )

    return sdf


def _write_world(sdf):
    """Write to a stable path (not random temp) to avoid gzserver race."""
    path = os.path.join(pkg_aice_sim, 'worlds', '_active_arena.world')
    with open(path, 'w') as fh:
        fh.write(sdf)
    return path


# ── Launch description ───────────────────────────────────────────────────────

def generate_launch_description():

    declare_marker_north = DeclareLaunchArgument(
        'marker_north', default_value=str(_DEFAULT_NORTH),
        description='ArUco ID on the NORTH wall [0-49].')
    declare_marker_south = DeclareLaunchArgument(
        'marker_south', default_value=str(_DEFAULT_SOUTH),
        description='ArUco ID on the SOUTH wall [0-49].')
    declare_marker_east = DeclareLaunchArgument(
        'marker_east', default_value=str(_DEFAULT_EAST),
        description='ArUco ID on the EAST wall [0-49].')
    declare_marker_west = DeclareLaunchArgument(
        'marker_west', default_value=str(_DEFAULT_WEST),
        description='ArUco ID on the WEST wall [0-49].')

    declare_red_marker = DeclareLaunchArgument(
        'red_marker', default_value=str(_DEFAULT_EAST),
        description='Which wall ID is the RED goal (must match one of the marker_* args).')
    declare_blue_marker = DeclareLaunchArgument(
        'blue_marker', default_value=str(_DEFAULT_WEST),
        description='Which wall ID is the BLUE goal (must match one of the marker_* args).')

    declare_x_pose = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Robot spawn X (m).')
    declare_y_pose = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Robot spawn Y (m).')

    def launch_setup(context, *args, **kwargs):
        north      = int(LaunchConfiguration('marker_north').perform(context))
        south      = int(LaunchConfiguration('marker_south').perform(context))
        east       = int(LaunchConfiguration('marker_east').perform(context))
        west       = int(LaunchConfiguration('marker_west').perform(context))
        red_id     = int(LaunchConfiguration('red_marker').perform(context))
        blue_id    = int(LaunchConfiguration('blue_marker').perform(context))
        x_pose     = LaunchConfiguration('x_pose').perform(context)
        y_pose     = LaunchConfiguration('y_pose').perform(context)

        # Validate colour assignments reference actual wall IDs
        all_ids = {north, south, east, west}
        for name, val in (('red_marker', red_id), ('blue_marker', blue_id)):
            if val not in all_ids:
                raise ValueError(
                    f'[turtlebot3_world] {name}={val} does not match any '
                    f'marker_* value {all_ids}.')

        # Patch and write world
        patched = _patch_world(_WORLD_TEMPLATE, models_path,
                               north, south, east, west)
        world_file = _write_world(patched)

        print(
            '\n[turtlebot3_world] Walls  : N={} S={} E={} W={}'
            '\n[turtlebot3_world] Goals  : red={} blue={}'
            '\n[turtlebot3_world] World  : {}\n'.format(
                north, south, east, west, red_id, blue_id, world_file),
            file=sys.stderr)

        pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
        pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
        tb3_launch_dir = os.path.join(pkg_tb3_gazebo, 'launch')

        gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={
                'world': world_file,
                'extra_gazebo_args': '--verbose',
            }.items())

        gzclient = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')))

        robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items())

        spawn_turtlebot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items())

        # Publishes [red_id, blue_id] on /arena/wall_ids at 1 Hz
        # so navigation.py knows the colour assignment regardless of
        # start order.
        wall_id_publisher = Node(
            package='turtlebot3_task',
            executable='wall_id_publisher.py',
            name='wall_id_publisher',
            parameters=[{
                'wall_id_red':  red_id,
                'wall_id_blue': blue_id,
            }],
            output='screen',
        )

        return [
            gzserver,
            gzclient,
            robot_state_publisher,
            spawn_turtlebot,
            wall_id_publisher,
        ]

    return LaunchDescription([
        declare_marker_north,
        declare_marker_south,
        declare_marker_east,
        declare_marker_west,
        declare_red_marker,
        declare_blue_marker,
        declare_x_pose,
        declare_y_pose,
        OpaqueFunction(function=launch_setup),
    ])