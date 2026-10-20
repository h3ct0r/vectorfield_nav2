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

"""Shared pieces of the demo launch files: distro checks, overlay and map lookup, RViz."""

import copy
import os

from vectorfield_bringup import params

PACKAGE = 'vectorfield_bringup'

#: The motion models with a controller overlay in ``config/controller``.
MOTION_MODELS = ('differential', 'skid_steer', 'omnidirectional', 'ackermann')

#: Distros every demo supports.
SUPPORTED_DISTROS = ('humble', 'jazzy', 'kilted', 'lyrical', 'rolling')

#: Distros with Nav2's loopback simulator and the Gazebo TurtleBot simulations.
JAZZY_AND_LATER = SUPPORTED_DISTROS[1:]

FOLLOWER = 'curve_follower'


class UnsupportedDistroError(RuntimeError):
    """The running ROS distro cannot run this demo."""


def require_distro(launch_name, allowed=SUPPORTED_DISTROS, distro=None):
    """Return the running distro, or raise a clear error if ``launch_name`` cannot run on it."""
    if distro is None:
        distro = params.ros_distro()
    if distro not in allowed:
        message = (
            f'{launch_name} runs on ROS 2 {", ".join(allowed)}; this is '
            f'ROS_DISTRO={distro or "<unset>"}.'
        )
        if distro in SUPPORTED_DISTROS:
            message += ' kinematic_sim_demo.launch.py runs on every supported distro.'
        raise UnsupportedDistroError(message)
    return distro


def refuse_unsupported(launch_name, allowed=SUPPORTED_DISTROS, distro=None):
    """Return None on a supported distro, else a launch description that fails with the reason.

    The error is raised while the launch runs rather than while the file loads, so ``ros2 launch``
    prints only the message and exits with an error.
    """
    try:
        require_distro(launch_name, allowed, distro)
        return None
    except UnsupportedDistroError as error:
        reason = error
    from launch import LaunchDescription
    from launch.actions import OpaqueFunction

    def fail(_context):
        raise reason

    return LaunchDescription([OpaqueFunction(function=fail)])


def share(*parts):
    """Return a path in this package's share directory."""
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory(PACKAGE), *parts)


def split_list(value):
    """Split a comma- or space-separated launch argument into its non-empty items."""
    return [item for item in value.replace(',', ' ').split() if item]


def resolve_config(value, directory, share_dir=None):
    """Return ``value`` as a YAML path: a path is kept, a bare name is looked up in ``directory``.

    ``directory`` is relative to the package share (for example ``config/scenario``).

    Raises:
        FileNotFoundError: if the file does not exist.

    """
    if os.path.dirname(value) or value.endswith('.yaml'):
        path = os.path.abspath(os.path.expanduser(value))
    else:
        base = share_dir if share_dir is not None else share()
        path = os.path.join(base, directory, value + '.yaml')
    if not os.path.isfile(path):
        raise FileNotFoundError(f'{value!r}: no such file ({path})')
    return path


def controller_overlay(motion_model, share_dir=None):
    """Return ``config/controller/<motion_model>.yaml``."""
    if motion_model not in MOTION_MODELS:
        raise ValueError(f'motion_model {motion_model!r} is not one of {", ".join(MOTION_MODELS)}')
    return resolve_config(motion_model, os.path.join('config', 'controller'), share_dir)


def overlay_paths(motion_model, scenarios, share_dir=None):
    """Return the controller overlay followed by the scenario overlays, in merge order.

    Args:
        motion_model: One of :data:`MOTION_MODELS`.
        scenarios: Scenario names (``config/scenario/<name>.yaml``) or YAML paths.
        share_dir: The package share directory (looked up by default).

    """
    paths = [controller_overlay(motion_model, share_dir)]
    scenario_dir = os.path.join('config', 'scenario')
    paths += [resolve_config(name, scenario_dir, share_dir) for name in scenarios]
    return paths


def merged_params_file(motion_model, scenarios):
    """Merge the overlays onto nav2_bringup's parameters and return the written file."""
    return params.write_params(params.merge(overlay_paths(motion_model, scenarios)))


#: vectorfield_sim's node name and its skid-steer slip defaults (``slip_eta_psi``,
#: ``slip_x_icr``; ``kSkidSteerSlip`` in vectorfield_sim/plant.hpp).
SIM = 'vectorfield_sim'
SIM_SKID_SLIP = (0.75, 0.05)


def with_slip_compensation(merged, eta, x_icr):
    """Return a copy of ``merged`` whose FollowPath compensates a skid slip (``eta``, ``x_icr``).

    Sets ``skid_yaw_rate_efficiency`` and ``skid_icr_offset`` (motion_models.md, skid-slip).
    """
    result = copy.deepcopy(merged)
    controller = result.setdefault('controller_server', {}).setdefault('ros__parameters', {})
    follow = controller.setdefault('FollowPath', {})
    follow['skid_yaw_rate_efficiency'] = float(eta)
    follow['skid_icr_offset'] = float(x_icr)
    return result


def kinematic_sim_params(motion_model, scenarios, share_dir=None):
    """Merge the overlays for a vectorfield_sim demo; a skid-steer controller compensates its slip.

    For ``skid_steer`` the slip is the merged ``vectorfield_sim`` section's ``slip_eta_psi`` and
    ``slip_x_icr`` (:data:`SIM_SKID_SLIP` when unset). Both are written to that section and,
    through :func:`with_slip_compensation`, to FollowPath, so the simulated plant and the
    controller use the same values. Other models are merged unchanged. ``share_dir`` as in
    :func:`overlay_paths`.
    """
    merged = params.merge(overlay_paths(motion_model, scenarios, share_dir))
    if motion_model != 'skid_steer':
        return merged
    sim = merged.setdefault(SIM, {}).setdefault('ros__parameters', {})
    eta = float(sim.get('slip_eta_psi', SIM_SKID_SLIP[0]))
    x_icr = float(sim.get('slip_x_icr', SIM_SKID_SLIP[1]))
    sim['slip_eta_psi'] = eta
    sim['slip_x_icr'] = x_icr
    return with_slip_compensation(merged, eta, x_icr)


def follower_parameters(merged):
    """Return the curve follower's parameters, taken from merged Nav2 parameters.

    The follower declares the controller's common keys unprefixed, so the ``FollowPath`` block
    (motion model, speeds, limits, obstacle mode; ``plugin`` left out) configures it like the
    controller. The merged ``curve_follower`` section, if any, then overrides it.
    """
    controller = merged.get('controller_server', {}).get('ros__parameters', {})
    values = {k: v for k, v in controller.get('FollowPath', {}).items() if k != 'plugin'}
    values.update(merged.get(FOLLOWER, {}).get('ros__parameters', {}))
    return values


def navigation_lifecycle_nodes(context=None):
    """Return the nodes of nav2_bringup's navigation_launch.py that it leaves unmanaged.

    Up to Jazzy, navigation_launch.py starts lifecycle_manager_navigation itself, and this
    returns an empty tuple. Lyrical moved the manager to bringup_launch.py and exports the node
    list as ``nav2_bringup.navigation_launch.get_lifecycle_nodes``; a launch file that includes
    navigation_launch.py directly must then start a manager for these nodes.
    """
    from ament_index_python.packages import get_package_share_directory

    launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
    )
    with open(launch_file, encoding='utf-8') as stream:
        if 'nav2_lifecycle_manager' in stream.read():
            return ()
    from nav2_bringup.navigation_launch import get_lifecycle_nodes

    return tuple(get_lifecycle_nodes(context))


def component_container(name, parameters, distro):
    """Return a component container started as nav2_bringup starts its own on ``distro``.

    component_container_isolated up to Kilted; Lyrical deprecates it for ``component_container
    --isolated``. Either way every component gets its own single-threaded executor.
    """
    from launch_ros.actions import Node

    if distro in ('humble', 'jazzy', 'kilted'):
        container = {'executable': 'component_container_isolated', 'arguments': []}
    else:
        container = {
            'executable': 'component_container',
            'arguments': ['--isolated', '--executor-type', 'single-threaded'],
        }
    return Node(
        package='rclcpp_components', name=name, parameters=parameters, output='screen', **container
    )


def rviz_node(use_rviz):
    """Return the RViz node with ``rviz/vectorfield.rviz``, started when ``use_rviz`` is true."""
    from launch.conditions import IfCondition
    from launch_ros.actions import Node

    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', share('rviz', 'vectorfield.rviz')],
        parameters=[{'use_sim_time': True}],
        output='log',
        condition=IfCondition(use_rviz),
    )


def initial_pose_node(x, y, yaw):
    """Return initial_pose_publisher for the given map pose (launch substitutions or values)."""
    from launch_ros.actions import Node

    return Node(
        package=PACKAGE,
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        parameters=[{'x': x, 'y': y, 'yaw': yaw, 'use_sim_time': True}],
        output='screen',
    )


def gazebo_demo(robot, world_map, x, y, yaw, map_offset=(0.0, 0.0)):
    """Return the launch description of the ``<robot>_gazebo_demo`` launch file (Jazzy and later).

    It includes nav2_bringup's ``<robot>_simulation_launch.py`` (Gazebo, AMCL and Nav2) with the
    merged parameters and its own RViz disabled, initial_pose_publisher at the spawn pose in the
    map frame, which AMCL needs, and RViz with vectorfield.rviz.

    Args:
        robot: ``tb3`` or ``tb4``.
        world_map: The map of the stock world in ``nav2_bringup/maps`` (without ``.yaml``).
        x: Default spawn x in the Gazebo world frame (m), the stock world's value.
        y: Default spawn y (m).
        yaw: Default spawn yaw (rad).
        map_offset: The stock map's (x, y) minus the world's: the initial pose is the spawn pose
            shifted by it. The depot map's origin is the TB4 spawn point (-8, 0) of its world.

    """
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration

    refusal = refuse_unsupported(f'{robot}_gazebo_demo.launch.py', JAZZY_AND_LATER)
    if refusal is not None:
        return refusal

    def launch_setup(context):
        from ament_index_python.packages import get_package_share_directory

        motion_model = LaunchConfiguration('motion_model').perform(context)
        scenarios = split_list(LaunchConfiguration('scenarios').perform(context))
        arguments = {
            'params_file': merged_params_file(motion_model, scenarios),
            'use_rviz': 'False',
            'headless': LaunchConfiguration('headless').perform(context),
            'x_pose': LaunchConfiguration('x_pose').perform(context),
            'y_pose': LaunchConfiguration('y_pose').perform(context),
            'yaw': LaunchConfiguration('yaw').perform(context),
        }
        # Always passed: this file's own empty `map` would otherwise reach the wrapper.
        nav2_bringup = get_package_share_directory('nav2_bringup')
        map_value = LaunchConfiguration('map').perform(context)
        if map_value:
            arguments['map'] = resolve_config(map_value, 'maps')
        else:
            arguments['map'] = os.path.join(nav2_bringup, 'maps', f'{world_map}.yaml')
        simulation = os.path.join(nav2_bringup, 'launch', f'{robot}_simulation_launch.py')
        spawn = [float(arguments[name]) for name in ('x_pose', 'y_pose', 'yaw')]
        pose = (spawn[0] + map_offset[0], spawn[1] + map_offset[1], spawn[2])
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation), launch_arguments=arguments.items()
            ),
            initial_pose_node(*pose),
            rviz_node(LaunchConfiguration('use_rviz')),
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'motion_model',
                default_value='differential',
                choices=['differential', 'skid_steer'],
                description='Controller overlay config/controller/<motion_model>.yaml (the '
                'simulated robot is differential drive).',
            ),
            DeclareLaunchArgument(
                'map',
                default_value='',
                description=f"Map YAML matching the world; empty: nav2_bringup's {world_map}.",
            ),
            DeclareLaunchArgument(
                'scenarios',
                default_value='',
                description='Extra overlays, comma separated: names in config/scenario or YAML '
                'paths.',
            ),
            DeclareLaunchArgument(
                'use_rviz', default_value='true', description='Start RViz with vectorfield.rviz.'
            ),
            DeclareLaunchArgument(
                'headless', default_value='True', description='Run Gazebo without its GUI.'
            ),
            DeclareLaunchArgument(
                'x_pose',
                default_value=str(x),
                description='Spawn x in the Gazebo world (m); the initial pose follows it.',
            ),
            DeclareLaunchArgument('y_pose', default_value=str(y), description='Spawn y (m).'),
            DeclareLaunchArgument('yaw', default_value=str(yaw), description='Spawn yaw (rad).'),
            OpaqueFunction(function=launch_setup),
        ]
    )
