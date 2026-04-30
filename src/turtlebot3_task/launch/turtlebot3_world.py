#!/usr/bin/env python3
import os
import re
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

pkg_aice_sim = get_package_share_directory('turtlebot3_task')
models_path  = os.path.join(pkg_aice_sim, 'models')
os.environ['GAZEBO_MODEL_PATH'] = models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')

_WORLD_TEMPLATE = os.path.join(pkg_aice_sim, 'worlds', 'world.world')

def _patch_world(template_path, models_dir, north, south, east, west):
    with open(template_path, 'r') as fh:
        sdf = fh.read()
    walls = [
        ('arena_wall_north', 'wall_north', north),
        ('arena_wall_south', 'wall_south', south),
        ('arena_wall_east',  'wall_east',  east),
        ('arena_wall_west',  'wall_west',  west),
    ]
    for uri_prefix, model_name, new_id in walls:
        abs_material = f'{models_dir}/{uri_prefix}/materials/scripts/marker_{new_id}.material'
        sdf = re.sub(r'(?:model|file)://{}/materials/scripts/marker_\d+\.material'.format(re.escape(uri_prefix)),
                     'file://' + abs_material, sdf, count=1)
        def replace_in_block(m, nid=new_id):
            return re.sub(r'ArUcoMarker\d+', f'ArUcoMarker{nid}', m.group(0), count=1)
        sdf = re.sub(f'<model name="{re.escape(model_name)}".*?</model>', replace_in_block, sdf, count=1, flags=re.DOTALL)
    return sdf

def generate_launch_description():
    # Setup arguments for markers
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])

def launch_setup(context):
    # World patching logic and node execution here...
    # (Abbreviated for clarity - launches Gazebo + TB3 + wall_id_publisher)
    pass