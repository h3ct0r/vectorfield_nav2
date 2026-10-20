# Copyright 2026 Héctor Azpúrua
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Nav2 with the vector-field controller on Nav2's TB3 loopback simulator (Jazzy and later).

Includes nav2_bringup's TB3 loopback wrapper (robot_state_publisher, map_server, the loopback
simulator and Nav2 without localization) with the merged parameters and its own RViz disabled,
then starts initial_pose_publisher (the loopback simulator publishes no TF before it receives
``initialpose``) and RViz with vectorfield.rviz.

Example::

    ros2 launch vectorfield_bringup loopback_demo.launch.py motion_model:=omnidirectional
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from vectorfield_bringup import demo

# The scenario overlay this demo always applies, before the `scenarios` argument.
SCENARIO = 'loopback'

# The wrapper's name per distro: underscore on Kilted and later, dot on Jazzy.
WRAPPERS = ('tb3_loopback_simulation_launch.py', 'tb3_loopback_simulation.launch.py')


def loopback_wrapper():
    """Return the installed TB3 loopback wrapper of nav2_bringup."""
    launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    for name in WRAPPERS:
        path = os.path.join(launch_dir, name)
        if os.path.isfile(path):
            return path
    raise FileNotFoundError(f'no TB3 loopback wrapper ({" or ".join(WRAPPERS)}) in {launch_dir}')


def _launch_setup(context):
    motion_model = LaunchConfiguration('motion_model').perform(context)
    scenarios = [SCENARIO] + demo.split_list(LaunchConfiguration('scenarios').perform(context))
    map_yaml = demo.resolve_config(LaunchConfiguration('map').perform(context), 'maps')
    params_file = demo.merged_params_file(motion_model, scenarios)
    pose = [
        ParameterValue(LaunchConfiguration(f'initial_pose_{axis}'), value_type=float)
        for axis in ('x', 'y', 'yaw')
    ]
    wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(loopback_wrapper()),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_rviz': 'False',
        }.items(),
    )
    return [
        wrapper,
        demo.initial_pose_node(*pose),
        demo.rviz_node(LaunchConfiguration('use_rviz')),
    ]


def generate_launch_description():
    """Build the loopback simulator demo."""
    refusal = demo.refuse_unsupported('loopback_demo.launch.py', demo.JAZZY_AND_LATER)
    if refusal is not None:
        return refusal
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'motion_model',
                default_value='differential',
                choices=list(demo.MOTION_MODELS),
                description='Controller overlay config/controller/<motion_model>.yaml.',
            ),
            DeclareLaunchArgument(
                'map',
                default_value='open_10x10',
                description='Map: a name in maps/ (open_10x10, corridor_door, corridor_corner) '
                'or a map YAML path.',
            ),
            DeclareLaunchArgument(
                'scenarios',
                default_value='',
                description='Extra overlays, comma separated, merged after loopback: names in '
                'config/scenario or YAML paths.',
            ),
            DeclareLaunchArgument(
                'use_rviz', default_value='true', description='Start RViz with vectorfield.rviz.'
            ),
            DeclareLaunchArgument(
                'initial_pose_x', default_value='0.0', description='Initial map x (m).'
            ),
            DeclareLaunchArgument(
                'initial_pose_y', default_value='0.0', description='Initial map y (m).'
            ),
            DeclareLaunchArgument(
                'initial_pose_yaw', default_value='0.0', description='Initial map yaw (rad).'
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
