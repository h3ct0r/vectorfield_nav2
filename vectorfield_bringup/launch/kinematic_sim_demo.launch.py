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

"""Nav2 with the vector-field controller on vectorfield_sim, on every supported distro.

Starts the kinematic simulator (the time source), the static identity TFs base_footprint ->
base_link -> base_scan, map_server under lifecycle_manager_map_server, Nav2's
navigation_launch.py with the merged parameters (plus lifecycle_manager_navigation on Lyrical
and later, whose navigation_launch.py has none), initial_pose_publisher and RViz. With
``use_composition`` (the default) map_server, the Nav2 servers and the managers share one
component container, as in nav2_bringup. On Humble, collision_monitor (under
lifecycle_manager_collision_monitor, in the same container) turns ``cmd_vel`` into
``cmd_vel_safe`` and the simulator listens there; on Jazzy and later the stock smoother ->
collision_monitor -> ``cmd_vel`` chain is used as is. Everything but the simulator and its TFs
comes from ``vectorfield_bringup.demo.nav2_demo_actions()``, which the MuJoCo examples
(vectorfield_mujoco) share.

Examples::

    ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py motion_model:=omnidirectional
    ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py scenarios:=demo_obstacles
    ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py map:=corridor_door use_rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from vectorfield_bringup import demo, params

# The scenario overlay this demo always applies, before the `scenarios` argument.
SCENARIO = 'kinematic_sim'


def _static_identity(parent, child):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_tf_{child}',
        arguments=['--frame-id', parent, '--child-frame-id', child],
        output='log',
    )


def _launch_setup(context, distro):
    motion_model = LaunchConfiguration('motion_model').perform(context)
    scenarios = [SCENARIO] + demo.split_list(LaunchConfiguration('scenarios').perform(context))
    map_yaml = demo.resolve_config(LaunchConfiguration('map').perform(context), 'maps')
    # skid_steer: the controller compensates the simulator's slip (same values for both).
    params_file = params.write_params(demo.kinematic_sim_params(motion_model, scenarios))
    composed = IfCondition(LaunchConfiguration('use_composition')).evaluate(context)
    pose = {
        axis: ParameterValue(LaunchConfiguration(f'initial_pose_{axis}'), value_type=float)
        for axis in ('x', 'y', 'yaw')
    }

    sim = Node(
        package='vectorfield_sim',
        executable='kinematic_sim',
        name='vectorfield_sim',
        parameters=[
            params_file,
            {
                # Humble has no collision_monitor in its command chain; bringup adds one.
                'cmd_vel_topic': demo.sim_cmd_vel_topic(distro),
                'initial_pose_x': pose['x'],
                'initial_pose_y': pose['y'],
                'initial_pose_yaw': pose['yaw'],
            },
        ],
        output='screen',
    )
    return demo.nav2_demo_actions(
        context,
        distro,
        params_file,
        map_yaml,
        [
            sim,
            _static_identity('base_footprint', 'base_link'),
            _static_identity('base_link', 'base_scan'),
        ],
        (pose['x'], pose['y'], pose['yaw']),
        LaunchConfiguration('use_rviz'),
        composed,
    )


def generate_launch_description():
    """Build the kinematic simulator demo."""
    refusal = demo.refuse_unsupported('kinematic_sim_demo.launch.py')
    if refusal is not None:
        return refusal
    distro = params.ros_distro()
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'motion_model',
                default_value='differential',
                choices=list(demo.MOTION_MODELS),
                description='Controller overlay config/controller/<motion_model>.yaml; also the '
                'simulated base.',
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
                description='Extra overlays, comma separated, merged after kinematic_sim: names '
                'in config/scenario (e.g. demo_obstacles) or YAML paths.',
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
