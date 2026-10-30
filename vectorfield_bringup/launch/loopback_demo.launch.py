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

Starts the simulator side of nav2_bringup's TB3 loopback wrapper: the TurtleBot3 Waffle's
robot_state_publisher and Nav2's loopback simulator (the time source), the latter once
map_server serves the map (``wait_for_map``: Jazzy's simulator fetches the map once and cannot
use the empty one an inactive map_server returns). The simulator stops after Nav2 at a shutdown
(``vectorfield_bringup.demo.shutdown_last()``), so Nav2 shuts down on a running clock, as with
our own simulators. Everything else (map_server, Nav2 in one component container,
initial_pose_publisher and RViz) comes from ``vectorfield_bringup.demo.nav2_demo_actions()``, as
in kinematic_sim_demo.

Example::

    ros2 launch vectorfield_bringup loopback_demo.launch.py motion_model:=omnidirectional
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from vectorfield_bringup import demo, params

# The scenario overlay this demo always applies, before the `scenarios` argument.
SCENARIO = 'loopback'


def simulator_actions(params_file):
    """Return robot_state_publisher and the simulator, once the map is served."""
    urdf = os.path.join(
        get_package_share_directory('nav2_minimal_tb3_sim'), 'urdf', 'turtlebot3_waffle.urdf'
    )
    with open(urdf, encoding='utf-8') as stream:
        robot_description = stream.read()
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_loopback_sim'), 'loopback_simulation.launch.py'
            )
        ),
        launch_arguments={'params_file': params_file}.items(),
    )
    wait_for_map = Node(
        package=demo.PACKAGE, executable='wait_for_map', name='wait_for_map', output='screen'
    )

    def after_map(event, _context):
        if event.returncode == 0:
            return [demo.shutdown_last([simulator], ['loopback_simulator'])]
        return [EmitEvent(event=Shutdown(reason='map_server served no map'))]

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
            output='log',
        ),
        wait_for_map,
        RegisterEventHandler(OnProcessExit(target_action=wait_for_map, on_exit=after_map)),
    ]


def _launch_setup(context, distro):
    motion_model = LaunchConfiguration('motion_model').perform(context)
    scenarios = [SCENARIO] + demo.split_list(LaunchConfiguration('scenarios').perform(context))
    map_yaml = demo.resolve_config(LaunchConfiguration('map').perform(context), 'maps')
    params_file = demo.merged_params_file(motion_model, scenarios)
    composed = IfCondition(LaunchConfiguration('use_composition')).evaluate(context)
    pose = tuple(
        ParameterValue(LaunchConfiguration(f'initial_pose_{axis}'), value_type=float)
        for axis in ('x', 'y', 'yaw')
    )
    return demo.nav2_demo_actions(
        context,
        distro,
        params_file,
        map_yaml,
        simulator_actions(params_file),
        pose,
        LaunchConfiguration('use_rviz'),
        composed,
    )


def generate_launch_description():
    """Build the loopback simulator demo."""
    refusal = demo.refuse_unsupported('loopback_demo.launch.py', demo.JAZZY_AND_LATER)
    if refusal is not None:
        return refusal
    distro = params.ros_distro()
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
                'use_composition',
                default_value='true',
                description='Run map_server, the Nav2 servers and their lifecycle managers in one '
                'component container (nav2_container).',
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
            OpaqueFunction(function=_launch_setup, args=[distro]),
        ]
    )
