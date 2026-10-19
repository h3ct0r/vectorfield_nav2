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

"""The curve follower on vectorfield_sim, on every supported distro.

Includes kinematic_sim_demo.launch.py (vectorfield_sim, map_server, Nav2 and RViz) with the
``curve_follower`` scenario overlay merged after ``kinematic_sim``, and starts the curve follower
(``curve_follower``) with ``lifecycle_manager_curve_follower``. Nav2 stays up: its local costmap
is the follower's obstacle input, its action statuses drive the follower's interlock,
send_curve.py approaches a curve with NavigateToPose, and the follower's ``cmd_vel_nav`` goes
through Nav2's velocity smoother and collision_monitor (on Humble, bringup's collision_monitor).

The follower's parameters are the merged ``FollowPath`` block (motion model, speeds, limits,
obstacle mode), then the merged ``curve_follower`` section.

Examples::

    ros2 launch vectorfield_bringup curve_follower_demo.launch.py
    ros2 run vectorfield_bringup send_curve.py --shape lemniscate_bernoulli --laps 3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vectorfield_bringup import demo, params

SCENARIO = 'curve_follower'

# The kinematic demo's arguments, passed through unchanged (scenarios gets SCENARIO first).
FORWARDED = (
    'motion_model',
    'map',
    'use_rviz',
    'use_composition',
    'initial_pose_x',
    'initial_pose_y',
    'initial_pose_yaw',
)


def _launch_setup(context):
    motion_model = LaunchConfiguration('motion_model').perform(context)
    scenarios = [SCENARIO] + demo.split_list(LaunchConfiguration('scenarios').perform(context))
    arguments = {name: LaunchConfiguration(name).perform(context) for name in FORWARDED}
    arguments['scenarios'] = ','.join(scenarios)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo.share('launch', 'kinematic_sim_demo.launch.py')),
        launch_arguments=arguments.items(),
    )

    merged = params.merge(demo.overlay_paths(motion_model, ['kinematic_sim'] + scenarios))
    follower_file = params.write_params(
        {demo.FOLLOWER: {'ros__parameters': demo.follower_parameters(merged)}}
    )
    follower = Node(
        package='vectorfield_curve_follower',
        executable='curve_follower',
        name=demo.FOLLOWER,
        parameters=[follower_file, {'use_sim_time': True}],
        output='screen',
    )
    # Wall time, as kinematic_sim_demo's managers: a bond timeout started on simulated time
    # before /clock is discovered would expire on the first /clock message.
    manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_curve_follower',
        parameters=[{'autostart': True, 'node_names': [demo.FOLLOWER], 'use_sim_time': False}],
        output='screen',
    )
    return [simulation, follower, manager]


def generate_launch_description():
    """Build the curve follower demo."""
    refusal = demo.refuse_unsupported('curve_follower_demo.launch.py')
    if refusal is not None:
        return refusal
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'motion_model',
                default_value='differential',
                choices=list(demo.MOTION_MODELS),
                description='Controller overlay config/controller/<motion_model>.yaml, for Nav2 '
                'and the follower; also the simulated base.',
            ),
            DeclareLaunchArgument(
                'map',
                default_value='open_10x10',
                description='Map: a name in maps/ (open_10x10, corridor_door) or a map YAML path.',
            ),
            DeclareLaunchArgument(
                'scenarios',
                default_value='',
                description='Extra overlays, comma separated, merged after curve_follower: '
                'names in config/scenario (e.g. demo_obstacles) or YAML paths.',
            ),
            DeclareLaunchArgument(
                'use_rviz', default_value='true', description='Start RViz with vectorfield.rviz.'
            ),
            DeclareLaunchArgument(
                'use_composition',
                default_value='true',
                description='Run map_server, the Nav2 servers and their lifecycle managers in one '
                'component container (the follower runs in its own process).',
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
