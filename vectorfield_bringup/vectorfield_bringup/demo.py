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

"""Shared pieces of the demo launch files: distro checks, overlay and map lookup, RViz.

:func:`nav2_demo_actions` is the Nav2 side of a simulator demo (map_server, Nav2's servers and
their lifecycle managers, Humble's collision_monitor, the initial pose and RViz), around the
simulator's own actions; :func:`curve_follower_actions` adds the curve follower and
:func:`follower3d_actions` the 3D follower. The kinematic demos and the MuJoCo examples of
vectorfield_mujoco share them.
"""

import copy
import os
import re

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


def lifecycle_manager_library():
    """Return the library of Nav2's lifecycle manager to preload into a container, or None.

    A LifecycleManager adds a pre-shutdown callback to the global context and never removes it
    (Humble to Lyrical). A container unloads the component's library when it stops, and the
    context destroys that callback at exit with the library gone: a segmentation fault after
    every clean shutdown. A preloaded library is never unmapped.
    """
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix

    try:
        prefix = get_package_prefix('nav2_lifecycle_manager')
    except PackageNotFoundError:
        return None
    library = os.path.join(prefix, 'lib', 'libnav2_lifecycle_manager_core.so')
    return library if os.path.isfile(library) else None


def lifecycle_manager_preload():
    """Return ``LD_PRELOAD`` with :func:`lifecycle_manager_library` added, or None without it."""
    library = lifecycle_manager_library()
    if library is None:
        return None
    return ' '.join(p for p in (os.environ.get('LD_PRELOAD'), library) if p)


def preload_lifecycle_manager(actions):
    """Return a group of ``actions`` whose processes preload Nav2's lifecycle manager library.

    For a container that another launch file starts, such as nav2_bringup's Gazebo wrappers:
    ``LD_PRELOAD`` is set in the group's own environment, so every process of the included file
    loads the library, and only the container uses it (:func:`lifecycle_manager_library`).
    """
    from launch.actions import GroupAction, SetEnvironmentVariable

    preload = lifecycle_manager_preload()
    if preload is None:
        return GroupAction(list(actions))
    return GroupAction([SetEnvironmentVariable('LD_PRELOAD', preload), *actions], scoped=True)


def component_container(name, parameters):
    """Return this package's ``isolated_container``, the demos' component container.

    Every component gets its own single-threaded executor, as in nav2_bringup's container. At a
    shutdown signal it has the Nav2 lifecycle managers among its components shut their nodes
    down before the context shuts down (Nav2's own shutdown path cleans a node up while its
    callbacks run, a crash at exit now and then), and on Humble it exits where
    component_container_isolated waits forever. The demos' containers hold a Nav2 lifecycle
    manager, whose library is preloaded (:func:`lifecycle_manager_library`).
    """
    from launch_ros.actions import Node

    container = {}
    preload = lifecycle_manager_preload()
    if preload:
        container['additional_env'] = {'LD_PRELOAD': preload}
    return Node(
        package=PACKAGE,
        executable='isolated_container',
        name=name,
        parameters=parameters,
        output='screen',
        **container,
    )


def shutdown_last(actions, executables):
    """Return a group of ``actions`` whose processes named ``executables`` stop after Nav2.

    A simulator that is not ours stops at the shutdown signal, and Nav2's nodes then wait on
    simulated time, and on transforms that no longer come, while they shut down, until launch
    kills them. vectorfield_sim's ``shutdown_last`` holds the signal back from those processes
    until no node reads /clock (at most 4 s), as our own simulators do. It is applied through the
    scoped ``launch-prefix`` and ``launch-prefix-filter`` launch configurations, so it reaches
    the processes of an included launch file that the group starts.

    Args:
        actions: The launch actions, e.g. the include of a simulator's launch file.
        executables: The basenames of the executables to run under ``shutdown_last``.

    """
    from ament_index_python.packages import get_package_prefix
    from launch.actions import GroupAction, SetLaunchConfiguration

    wrapper = os.path.join(get_package_prefix(SIM), 'lib', SIM, 'shutdown_last')
    pattern = '^({})$'.format('|'.join(re.escape(name) for name in executables))
    return GroupAction(
        [
            SetLaunchConfiguration('launch-prefix', wrapper),
            SetLaunchConfiguration('launch-prefix-filter', pattern),
            *actions,
        ],
        scoped=True,
    )


def rviz_node(use_rviz, config='vectorfield.rviz'):
    """Return the RViz node with ``rviz/<config>``, started when ``use_rviz`` is true."""
    from launch.conditions import IfCondition
    from launch_ros.actions import Node

    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', share('rviz', config)],
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


#: With use_composition, map_server, Nav2's servers and the lifecycle managers share this
#: component container (nav2_bringup's name), so they need no DDS discovery between them.
NAV2_CONTAINER = 'nav2_container'

#: With use_composition, the curve follower and its lifecycle manager share this container.
FOLLOWER_CONTAINER = 'curve_follower_container'

#: The parameters of every lifecycle manager the demos start, besides autostart and node_names.
#:
#: - No bonds: at a shutdown signal the managed nodes break their bonds in their pre-shutdown
#:   callbacks, and a bonded manager then resets them from its own thread while the shutdown
#:   cleans them up. On Lyrical that deadlocks the container (cleaning up bt_navigator or a
#:   costmap removes a pre-shutdown callback, which waits for the context while the context's
#:   shutdown waits for the manager's thread) or crashes it (map_server reset in the middle of
#:   its own cleanup). A demo then gives up restarting a crashed node.
#: - Wall time: on simulated time a timeout that starts before /clock has been discovered
#:   starts at time 0, and the first /clock message then expires it at once.
MANAGER_PARAMETERS = {'bond_timeout': 0.0, 'use_sim_time': False}


def sim_cmd_vel_topic(distro):
    """Return the command topic a simulator listens on: ``cmd_vel_safe`` on Humble.

    Humble's navigation_launch.py has no collision_monitor; :func:`nav2_demo_actions` adds one
    that turns ``cmd_vel`` into ``cmd_vel_safe``.
    """
    return 'cmd_vel_safe' if distro == 'humble' else 'cmd_vel'


def lifecycle_node(package, plugin, executable, name, parameters, composed):
    """Return a node as a ComposableNode (``composed``) or as its own process."""
    from launch_ros.actions import Node
    from launch_ros.descriptions import ComposableNode

    if composed:
        return ComposableNode(package=package, plugin=plugin, name=name, parameters=parameters)
    return Node(
        package=package, executable=executable, name=name, parameters=parameters, output='screen'
    )


def lifecycle_manager(name, node_names, composed):
    """Return a lifecycle manager with autostart for ``node_names``: :data:`MANAGER_PARAMETERS`."""
    parameters = [{'autostart': True, 'node_names': list(node_names), **MANAGER_PARAMETERS}]
    return lifecycle_node(
        'nav2_lifecycle_manager',
        'nav2_lifecycle_manager::LifecycleManager',
        'lifecycle_manager',
        name,
        parameters,
        composed,
    )


def without_bonds(actions):
    """Return ``actions`` in a scope where every node gets ``bond_timeout`` 0.0, as a global.

    For a lifecycle manager another launch file starts with parameters of its own and no
    parameter file (Humble's and Jazzy's navigation_launch.py): its ``bond_timeout`` is then the
    one in :data:`MANAGER_PARAMETERS`. Nodes that declare no such parameter ignore it.
    """
    from launch.actions import GroupAction
    from launch_ros.actions import SetParameter

    return GroupAction(
        [SetParameter('bond_timeout', MANAGER_PARAMETERS['bond_timeout']), *actions], scoped=True
    )


def nav2_demo_actions(
    context, distro, params_file, map_yaml, sim_actions, initial_pose, use_rviz, composed=True
):
    """Return the Nav2 side of a simulator demo, around the simulator's own actions.

    In order: map_server under lifecycle_manager_map_server; lifecycle_manager_navigation on
    Lyrical and later, whose navigation_launch.py has none; on Humble, collision_monitor
    (``cmd_vel`` to ``cmd_vel_safe``, :func:`sim_cmd_vel_topic`) under
    lifecycle_manager_collision_monitor; with ``composed`` all of them in one component container
    (:data:`NAV2_CONTAINER`) that also loads the Nav2 servers; then ``sim_actions``, nav2_bringup's
    navigation_launch.py with ``params_file`` (up to Jazzy its own manager, :func:`without_bonds`),
    initial_pose_publisher and RViz.

    Args:
        context: The launch context (navigation_launch.py's node list on Lyrical needs it).
        distro: The running ROS distro.
        params_file: The merged Nav2 parameter file.
        map_yaml: The map YAML for map_server.
        sim_actions: The simulator's actions (the time source and its TFs).
        initial_pose: The initial map pose (x, y, yaw): launch substitutions or values.
        use_rviz: Start RViz (a substitution or a string).
        composed: Run map_server, the Nav2 servers and the managers in one container.

    """
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import LoadComposableNodes
    from launch_ros.descriptions import ParameterFile

    managed = [
        lifecycle_node(
            'nav2_map_server',
            'nav2_map_server::MapServer',
            'map_server',
            'map_server',
            [{'yaml_filename': map_yaml, 'use_sim_time': True}],
            composed,
        ),
        lifecycle_manager('lifecycle_manager_map_server', ['map_server'], composed),
    ]
    # Lyrical's navigation_launch.py no longer starts a lifecycle manager (bringup_launch.py does).
    navigation_nodes = navigation_lifecycle_nodes(context)
    if navigation_nodes:
        managed.append(
            lifecycle_manager('lifecycle_manager_navigation', navigation_nodes, composed)
        )
    if distro == 'humble':
        # Humble's navigation_launch.py has no collision_monitor; this one turns cmd_vel into
        # cmd_vel_safe. Composed, it shares the container with its manager: a manager in another
        # process can lose a change_state response to discovery under load ("failed to send
        # response ... client will not receive response"), and Humble's manager then waits for
        # it forever.
        managed += [
            lifecycle_node(
                'nav2_collision_monitor',
                'nav2_collision_monitor::CollisionMonitor',
                'collision_monitor',
                'collision_monitor',
                [share('config', 'humble', 'collision_monitor.yaml'), {'use_sim_time': True}],
                composed,
            ),
            lifecycle_manager(
                'lifecycle_manager_collision_monitor', ['collision_monitor'], composed
            ),
        ]
    if composed:
        managed = [
            # As in nav2_bringup: the container's parameter file is the only one that reaches
            # the costmaps' own nodes (local_costmap/local_costmap and so on).
            component_container(
                NAV2_CONTAINER,
                [
                    ParameterFile(params_file, allow_substs=True),
                    {'autostart': True, 'use_sim_time': True},
                ],
            ),
            LoadComposableNodes(
                target_container=NAV2_CONTAINER, composable_node_descriptions=managed
            ),
        ]

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
            'container_name': NAV2_CONTAINER,
        }.items(),
    )
    if not navigation_nodes:
        # navigation_launch.py starts lifecycle_manager_navigation itself.
        navigation = without_bonds([navigation])
    return (
        managed
        + list(sim_actions)
        + [navigation, initial_pose_node(*initial_pose), rviz_node(use_rviz)]
    )


def curve_follower_actions(merged, composed=True):
    """Return the curve follower and lifecycle_manager_curve_follower for merged parameters.

    The follower's parameters are :func:`follower_parameters` of ``merged``. With ``composed``
    both run in :data:`FOLLOWER_CONTAINER`.
    """
    from launch_ros.actions import LoadComposableNodes, Node
    from launch_ros.descriptions import ComposableNode

    follower_file = params.write_params(
        {FOLLOWER: {'ros__parameters': follower_parameters(merged)}}
    )
    follower_params = [follower_file, {'use_sim_time': True}]
    manager_parameters = [{'autostart': True, 'node_names': [FOLLOWER], **MANAGER_PARAMETERS}]
    if composed:
        # One process: a manager in another process can lose a change_state response to
        # discovery under load, and the manager then waits for it forever.
        nodes = [
            ComposableNode(
                package='vectorfield_curve_follower',
                plugin='vectorfield_curve_follower::CurveFollower',
                name=FOLLOWER,
                parameters=follower_params,
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_curve_follower',
                parameters=manager_parameters,
            ),
        ]
        return [
            component_container(FOLLOWER_CONTAINER, []),
            LoadComposableNodes(
                target_container=FOLLOWER_CONTAINER, composable_node_descriptions=nodes
            ),
        ]
    follower = Node(
        package='vectorfield_curve_follower',
        executable='curve_follower',
        name=FOLLOWER,
        parameters=follower_params,
        output='screen',
    )
    manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_curve_follower',
        parameters=manager_parameters,
        output='screen',
    )
    return [follower, manager]


#: The 3D follower's node name (follower3d_demo and vectorfield_mujoco's drone).
FOLLOWER_3D = 'follower_3d'


def follower3d_actions(parameters, use_rviz):
    """Return the 3D follower, lifecycle_manager_follower_3d and RViz with follower_3d.rviz.

    The simulator (point_sim_3d, or vectorfield_mujoco's mujoco_sim for a drone) is the caller's.

    Args:
        parameters: The follower's parameters: dictionaries and YAML paths, in order.
        use_rviz: Start RViz (a substitution or a string).

    """
    from launch_ros.actions import Node

    follower = Node(
        package='vectorfield_follower_3d',
        executable='follower_3d',
        name=FOLLOWER_3D,
        output='screen',
        parameters=list(parameters),
    )
    manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_follower_3d',
        output='screen',
        parameters=[{'autostart': True, 'node_names': [FOLLOWER_3D], **MANAGER_PARAMETERS}],
    )
    return [follower, manager, rviz_node(use_rviz, 'follower_3d.rviz')]


#: The scenario overlay the Gazebo demos always apply, before their ``scenarios`` argument.
GAZEBO_SCENARIO = 'gazebo'

#: The Gazebo server and the bridge that publishes its /clock (and the robot's odometry and TF).
GAZEBO_CLOCK_EXECUTABLES = ('gz', 'parameter_bridge')


def gazebo_demo(robot, world_map, x, y, yaw, map_offset=(0.0, 0.0)):
    """Return the launch description of the ``<robot>_gazebo_demo`` launch file (Jazzy and later).

    It includes nav2_bringup's ``<robot>_simulation_launch.py`` (Gazebo, AMCL and Nav2) with the
    merged parameters (``gazebo`` scenario overlay first) and its own RViz disabled,
    initial_pose_publisher at the spawn pose in the map frame, which AMCL needs, and RViz with
    vectorfield.rviz. Gazebo and its ros_gz_bridge stop after Nav2 (:func:`shutdown_last`), and
    the wrapper's Nav2 container preloads the lifecycle manager's library, as the demos' own
    containers do (:func:`preload_lifecycle_manager`).

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
        extra = split_list(LaunchConfiguration('scenarios').perform(context))
        scenarios = [GAZEBO_SCENARIO] + extra
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
        simulation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation), launch_arguments=arguments.items()
        )
        return [
            preload_lifecycle_manager([shutdown_last([simulation], GAZEBO_CLOCK_EXECUTABLES)]),
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
                description='Extra overlays, comma separated, merged after gazebo: names in '
                'config/scenario or YAML paths.',
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
