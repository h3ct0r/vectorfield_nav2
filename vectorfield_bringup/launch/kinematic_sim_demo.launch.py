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
collision_monitor -> ``cmd_vel`` chain is used as is.

Examples::

    ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py motion_model:=omnidirectional
    ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py scenarios:=demo_obstacles
    ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py map:=corridor_door use_rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.parameter_descriptions import ParameterValue

from vectorfield_bringup import demo, params

# The scenario overlay this demo always applies, before the `scenarios` argument.
SCENARIO = 'kinematic_sim'

# With use_composition, map_server, Nav2's servers and the lifecycle managers share this
# component container (nav2_bringup's name), so they need no DDS discovery between them.
CONTAINER = 'nav2_container'

# The lifecycle managers this file starts keep wall time: on simulated time a bond timeout that
# starts before /clock has been discovered starts at time 0, and the first /clock message then
# expires it at once.
MANAGER_CLOCK = {'use_sim_time': False}


def _static_identity(parent, child):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_tf_{child}',
        arguments=['--frame-id', parent, '--child-frame-id', child],
        output='log',
    )


def _lifecycle_node(package, plugin, executable, name, parameters, composed):
    if composed:
        return ComposableNode(package=package, plugin=plugin, name=name, parameters=parameters)
    return Node(
        package=package, executable=executable, name=name, parameters=parameters, output='screen'
    )


def _lifecycle_manager(name, node_names, composed):
    parameters = [{'autostart': True, 'node_names': list(node_names), **MANAGER_CLOCK}]
    return _lifecycle_node(
        'nav2_lifecycle_manager',
        'nav2_lifecycle_manager::LifecycleManager',
        'lifecycle_manager',
        name,
        parameters,
        composed,
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
    humble = distro == 'humble'

    managed = [
        _lifecycle_node(
            'nav2_map_server',
            'nav2_map_server::MapServer',
            'map_server',
            'map_server',
            [{'yaml_filename': map_yaml, 'use_sim_time': True}],
            composed,
        ),
        _lifecycle_manager('lifecycle_manager_map_server', ['map_server'], composed),
    ]
    # Lyrical's navigation_launch.py no longer starts a lifecycle manager (bringup_launch.py does).
    navigation_nodes = demo.navigation_lifecycle_nodes(context)
    if navigation_nodes:
        managed.append(
            _lifecycle_manager('lifecycle_manager_navigation', navigation_nodes, composed)
        )
    if humble:
        # Humble's navigation_launch.py has no collision_monitor; this one turns cmd_vel into
        # cmd_vel_safe. Composed, it shares the container with its manager: a manager in another
        # process can lose a change_state response to discovery under load ("failed to send
        # response ... client will not receive response"), and Humble's manager then waits for
        # it forever.
        managed += [
            _lifecycle_node(
                'nav2_collision_monitor',
                'nav2_collision_monitor::CollisionMonitor',
                'collision_monitor',
                'collision_monitor',
                [demo.share('config', 'humble', 'collision_monitor.yaml'), {'use_sim_time': True}],
                composed,
            ),
            _lifecycle_manager(
                'lifecycle_manager_collision_monitor', ['collision_monitor'], composed
            ),
        ]
    if composed:
        managed = [
            # As in nav2_bringup: the container's parameter file is the only one that reaches
            # the costmaps' own nodes (local_costmap/local_costmap and so on).
            demo.component_container(
                CONTAINER,
                [
                    ParameterFile(params_file, allow_substs=True),
                    {'autostart': True, 'use_sim_time': True},
                ],
                distro,
            ),
            LoadComposableNodes(target_container=CONTAINER, composable_node_descriptions=managed),
        ]

    sim = Node(
        package='vectorfield_sim',
        executable='kinematic_sim',
        name='vectorfield_sim',
        parameters=[
            params_file,
            {
                # Humble has no collision_monitor in its command chain; bringup adds one.
                'cmd_vel_topic': 'cmd_vel_safe' if humble else 'cmd_vel',
                'initial_pose_x': pose['x'],
                'initial_pose_y': pose['y'],
                'initial_pose_yaw': pose['yaw'],
            },
        ],
        output='screen',
    )

    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'namespace': '',
            'use_sim_time': 'True',
            'params_file': params_file,
            'autostart': 'True',
            'use_composition': 'True' if composed else 'False',
            'container_name': CONTAINER,
        }.items(),
    )

    actions = managed + [
        sim,
        _static_identity('base_footprint', 'base_link'),
        _static_identity('base_link', 'base_scan'),
        navigation,
        demo.initial_pose_node(pose['x'], pose['y'], pose['yaw']),
        demo.rviz_node(LaunchConfiguration('use_rviz')),
    ]
    return actions


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
