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

"""The experimental 3D follower on point_sim_3d, on every supported distro.

Starts ``point_sim_3d`` (a kinematic point robot that publishes ``/odom``, TF ``map →
base_link`` and ``/clock``), the 3D follower ``follower_3d`` with
``lifecycle_manager_follower_3d``, and RViz with ``rviz/follower_3d.rviz``. There is no map and
no Nav2. The follower runs the CBF filter against the simulator's ``obstacle_cloud``: a point
cloud of the spheres in its ``obstacle_spheres`` parameter (none at start), voxelised by the
follower. ``moving_frame:=platform`` makes the simulator also move a frame ``platform`` (TF and
``platform/odom``) for the ``moving`` example.

Examples::

    ros2 launch vectorfield_bringup follower3d_demo.launch.py
    ros2 run vectorfield_bringup send_curve3d.py --shape viviani --laps 2
    ros2 param set /point_sim_3d obstacle_spheres "[0.6, 0.0, 1.3, 0.25]"
    ros2 launch vectorfield_bringup follower3d_demo.launch.py moving_frame:=platform
    ros2 run vectorfield_bringup send_curve3d.py --shape moving --laps 0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vectorfield_bringup import demo

FOLLOWER = 'follower_3d'


def follower_parameters(context):
    """Return the follower's parameters of the demo (and the params_file, if given)."""
    parameters = {
        'use_sim_time': True,
        'cmd_frame': LaunchConfiguration('cmd_frame').perform(context),
        'obstacle_mode': 'cbf',
        'body_radius': 0.2,
        'voxel_cloud_topic': 'obstacle_cloud',
        'voxel_resolution': 0.1,
        'voxel_grid_size': 48,
        'moving_frame_reference': 'map',
    }
    params_file = LaunchConfiguration('params_file').perform(context)
    return [parameters] + ([params_file] if params_file else [])


def _launch_setup(context):
    moving_frame = LaunchConfiguration('moving_frame').perform(context)

    def number(name):
        return float(LaunchConfiguration(name).perform(context))

    sim = Node(
        package='vectorfield_follower_3d',
        executable='point_sim_3d',
        name='point_sim_3d',
        output='screen',
        parameters=[
            {
                'real_time_factor': number('real_time_factor'),
                'initial_x': number('initial_x'),
                'initial_y': number('initial_y'),
                'initial_z': number('initial_z'),
                'initial_yaw': number('initial_yaw'),
                'velocity_time_constant': number('velocity_time_constant'),
                'moving_frame': moving_frame,
                'moving_frame_topic': f'{moving_frame or "moving_frame"}/odom',
                'moving_frame_origin': [1.5, 0.0, 1.0],
                'moving_frame_linear': [0.1, 0.0, 0.0],
                'moving_frame_angular': [0.0, 0.0, 0.1],
            }
        ],
    )
    follower = Node(
        package='vectorfield_follower_3d',
        executable='follower_3d',
        name=FOLLOWER,
        output='screen',
        parameters=follower_parameters(context),
    )
    # Wall time, as the other demos' managers: a bond timeout started on simulated time before
    # /clock is discovered would expire on the first /clock message.
    manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_follower_3d',
        output='screen',
        parameters=[{'autostart': True, 'node_names': [FOLLOWER], 'use_sim_time': False}],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', demo.share('rviz', 'follower_3d.rviz')],
        parameters=[{'use_sim_time': True}],
        output='log',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )
    return [sim, follower, manager, rviz]


def generate_launch_description():
    """Build the 3D follower demo."""
    refusal = demo.refuse_unsupported('follower3d_demo.launch.py')
    if refusal is not None:
        return refusal
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_rviz', default_value='true', description='Start RViz with follower_3d.rviz.'
            ),
            DeclareLaunchArgument(
                'cmd_frame',
                default_value='global',
                choices=['global', 'body'],
                description='Frame of the velocity setpoints: global (ENU in map) or body.',
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value='',
                description='Extra parameter YAML for follower_3d, applied after the demo '
                'parameters.',
            ),
            DeclareLaunchArgument(
                'real_time_factor',
                default_value='1.0',
                description='Simulated seconds per wall second of point_sim_3d.',
            ),
            DeclareLaunchArgument(
                'moving_frame',
                default_value='',
                description='Name of a frame point_sim_3d moves at 0.1 m/s and 0.1 rad/s '
                '(e.g. platform); empty: none.',
            ),
            DeclareLaunchArgument(
                'velocity_time_constant',
                default_value='0.0',
                description='First-order lag of the simulated velocity (s).',
            ),
            DeclareLaunchArgument('initial_x', default_value='0.0', description='Start x (m).'),
            DeclareLaunchArgument('initial_y', default_value='0.0', description='Start y (m).'),
            DeclareLaunchArgument('initial_z', default_value='1.0', description='Start z (m).'),
            DeclareLaunchArgument(
                'initial_yaw', default_value='0.0', description='Start yaw (rad).'
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
