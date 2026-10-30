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

"""The MuJoCo examples: robot configs, scenarios and the launch description they share.

Parameters are merged with :func:`vectorfield_bringup.params.merge` onto nav2_bringup's file, in
this order (:func:`merged_params`):

1. the bringup motion-model overlay of the robot (``config/controller/<model>.yaml``);
2. for the curve follower, bringup's ``config/scenario/curve_follower.yaml``;
3. ``config/mujoco_sim.yaml``: the simulator's defaults and the velocity smoother's scaling;
4. ``config/nav2.yaml``: Nav2 settings of the ground examples;
5. ``config/robots/<robot>.yaml``: footprint, limits within the actuators, slip compensation;
6. the scenario's ``params`` (Nav2 behaviour only, never a limit).

An aerial robot (:data:`AERIAL_ROBOTS`) has no Nav2 and no motion-model overlay: its parameters
are ``config/mujoco_sim.yaml``, ``config/robots/<robot>.yaml`` (the 3D follower's limits, body
radius and CBF) and the scenario's ``params``, deep-merged without nav2_bringup's file.

A scenario (``config/scenarios/<name>.yaml``, :func:`load_scenario`) holds the unmapped
``obstacles``, an ``initial_pose`` (and, for a drone, ``initial_z``) and what ``run_scenario``
drives: ``waypoints`` (Nav2, one NavigateToPose goal each), a ``curve`` (a
``vectorfield_bringup.curves.EXAMPLES`` shape) with ``laps`` (the curve follower) or a
``curve3d`` (a ``vectorfield_bringup.curves3d.EXAMPLES`` shape) with ``laps`` (the 3D follower).
Its optional ``recording`` sets how ``record_path`` films it (:data:`RECORDING_KEYS`): the
``camera`` (chase, follow, top or fixed), the camera's ``view``
(:func:`vectorfield_mujoco.recorder.parse_view`), the markers to ``hide`` (namespaces,
``ns/id`` markers, Path topics) and the seconds of the robot's path drawn (``trail``), so that
the clip shows the scenario's point legibly.

:func:`launch_description` builds the generic launches (``mujoco_nav2.launch.py``,
``mujoco_curve_follower.launch.py``, ``mujoco_drone.launch.py``) and the example launches, which
only preset its arguments.
"""

import math
import os

import yaml

PACKAGE = 'vectorfield_mujoco'

#: Robot key → the bringup motion-model overlay its config is merged onto.
ROBOT_MODELS = {
    'tiago': 'differential',
    'scout_mini': 'skid_steer',
    'tidybot': 'omnidirectional',
    'ackermann_car': 'ackermann',
}

#: The aerial robots: the 3D follower drives them, without Nav2.
AERIAL_ROBOTS = ('x2',)

#: What drives the robot: Nav2 (NavigateToPose with the vector-field controller), the curve
#: follower (FollowCurve, with Nav2 up for the approach and the costmap) or the 3D follower
#: (FollowCurve3, an aerial robot).
DRIVERS = ('nav2', 'curve_follower', 'follower_3d')

#: The keys a scenario file may hold.
SCENARIO_KEYS = (
    'description',
    'initial_pose',
    'obstacles',
    'waypoints',
    'curve',
    'curve3d',
    'laps',
    'initial_z',
    'speed',
    'params',
    'recording',
)

#: The keys of a scenario's ``recording``.
RECORDING_KEYS = ('camera', 'view', 'hide', 'trail')

#: The recording cameras (:class:`vectorfield_mujoco.recorder.CameraRig`).
RECORD_CAMERAS = ('chase', 'follow', 'top', 'fixed')

#: Overlay topics drawn into the MuJoCo views, per driver.
OVERLAY_TOPICS = {
    'nav2': ['/plan', '/FollowPath/markers'],
    'curve_follower': ['/curve_follower/markers'],
    'follower_3d': ['/follower_3d/markers'],
}

#: velocity_smoother keys that only the motion-model and robot overlays set.
SMOOTHER_LIMITS = ('max_velocity', 'min_velocity', 'max_accel', 'max_decel')


def share(*parts, share_dir=None):
    """Return a path in this package's share directory (or ``share_dir``)."""
    if share_dir is None:
        from ament_index_python.packages import get_package_share_directory

        share_dir = get_package_share_directory(PACKAGE)
    return os.path.join(share_dir, *parts)


def robot_config(robot, share_dir=None):
    """Return ``config/robots/<robot>.yaml``."""
    if robot not in ROBOT_MODELS and robot not in AERIAL_ROBOTS:
        known = sorted(ROBOT_MODELS) + list(AERIAL_ROBOTS)
        raise ValueError(f'robot {robot!r} is not one of {", ".join(known)}')
    return share('config', 'robots', f'{robot}.yaml', share_dir=share_dir)


def resolve_scenario(value, share_dir=None):
    """Return a scenario path: a path is kept, a bare name is ``config/scenarios/<name>.yaml``.

    Raises:
        FileNotFoundError: The file does not exist.

    """
    if os.path.dirname(value) or value.endswith('.yaml'):
        path = os.path.abspath(os.path.expanduser(value))
    else:
        path = share('config', 'scenarios', f'{value}.yaml', share_dir=share_dir)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'scenario {value!r}: no such file ({path})')
    return path


def _pose(value, what):
    try:
        pose = [float(v) for v in value]
    except (TypeError, ValueError):
        raise ValueError(f'{what} must be [x, y, yaw], not {value!r}') from None
    if len(pose) != 3:
        raise ValueError(f'{what} must be [x, y, yaw], not {value!r}')
    return pose


def validate_scenario(data, name='scenario'):
    """Return a scenario mapping with its values checked and normalised.

    Raises:
        ValueError: An unknown key, a malformed value, more than one route (``waypoints``,
            ``curve``, ``curve3d``), or ``params`` that set velocity smoother limits.

    """
    from vectorfield_bringup import curves, curves3d
    from vectorfield_mujoco import simulation

    if not isinstance(data, dict):
        raise ValueError(f'{name}: a scenario must be a mapping')
    unknown = set(data) - set(SCENARIO_KEYS)
    if unknown:
        raise ValueError(f'{name}: unknown keys {sorted(unknown)}; known: {list(SCENARIO_KEYS)}')
    result = dict(data)
    result['obstacles'] = simulation.parse_obstacles(data.get('obstacles') or [])
    result['initial_pose'] = _pose(data.get('initial_pose', [0.0, 0.0, 0.0]), 'initial_pose')
    routes = [key for key in ('waypoints', 'curve', 'curve3d') if key in data]
    if len(routes) > 1:
        raise ValueError(f'{name}: give one route (waypoints, curve or curve3d), not {routes}')
    try:
        result['initial_z'] = float(data.get('initial_z', 0.0))
    except (TypeError, ValueError):
        raise ValueError(f'{name}: initial_z must be a height (m)') from None
    if not math.isfinite(result['initial_z']) or result['initial_z'] < 0.0:
        raise ValueError(f'{name}: initial_z must be a height >= 0 (m)')
    if 'waypoints' in data:
        waypoints = data['waypoints']
        if not isinstance(waypoints, list) or not waypoints:
            raise ValueError(f'{name}: waypoints must be a non-empty list of [x, y, yaw]')
        result['waypoints'] = [_pose(w, 'a waypoint') for w in waypoints]
    for key, module in (('curve', curves), ('curve3d', curves3d)):
        if key in data and (
            data[key] not in module.EXAMPLES or data[key] in module.MOVING_EXAMPLES
        ):
            static = sorted(set(module.EXAMPLES) - set(module.MOVING_EXAMPLES))
            raise ValueError(f'{name}: {key} must be one of {static}, not {data[key]!r}')
    if 'curve' in data or 'curve3d' in data:
        laps = data.get('laps', 1)
        if not isinstance(laps, int) or isinstance(laps, bool) or laps < 1:
            raise ValueError(f'{name}: laps must be a whole number >= 1, not {laps!r}')
        result['laps'] = laps
    elif 'laps' in data:
        raise ValueError(f'{name}: laps needs a curve or a curve3d')
    result['speed'] = float(data.get('speed', 0.0))
    params = data.get('params') or {}
    if not isinstance(params, dict):
        raise ValueError(f'{name}: params must be a Nav2 parameter mapping')
    smoother = params.get('velocity_smoother', {}).get('ros__parameters', {})
    if set(smoother) & set(SMOOTHER_LIMITS):
        raise ValueError(f'{name}: params must not set the velocity smoother limits')
    result['params'] = params
    result['recording'] = _recording(data.get('recording'), name)
    return result


def _recording(value, name):
    from vectorfield_mujoco import recorder

    recording = {} if value is None else value
    if not isinstance(recording, dict):
        raise ValueError(f'{name}: recording must be a mapping of {list(RECORDING_KEYS)}')
    unknown = set(recording) - set(RECORDING_KEYS)
    if unknown:
        raise ValueError(
            f'{name}: recording: unknown keys {sorted(unknown)}; known: {list(RECORDING_KEYS)}'
        )
    camera = recording.get('camera', 'chase')
    if camera not in RECORD_CAMERAS:
        raise ValueError(f'{name}: recording camera must be one of {list(RECORD_CAMERAS)}')
    try:
        view = recorder.parse_view(recording.get('view'))
    except ValueError as e:
        raise ValueError(f'{name}: recording: {e}') from None
    hide = recording.get('hide') or []
    if not isinstance(hide, list) or not all(isinstance(h, str) and h for h in hide):
        raise ValueError(f'{name}: recording hide must be a list of marker namespaces')
    try:
        trail = float(recording.get('trail', 0.0))
    except (TypeError, ValueError):
        trail = math.nan
    if not math.isfinite(trail) or trail < 0.0:
        raise ValueError(f'{name}: recording trail must be a duration >= 0 (s)')
    return {'camera': camera, 'view': view, 'hide': hide, 'trail': trail}


def load_scenario(path):
    """Load and validate a scenario file (:func:`validate_scenario`)."""
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    return validate_scenario(data, os.path.basename(path))


def driver_of(scenario):
    """Return the driver a scenario's route needs (one of :data:`DRIVERS`) or None."""
    if 'curve3d' in scenario:
        return 'follower_3d'
    if 'curve' in scenario:
        return 'curve_follower'
    if 'waypoints' in scenario:
        return 'nav2'
    return None


def overlay_paths(robot, driver, share_dir=None, bringup_share_dir=None):
    """Return the overlay files of :func:`merged_params`, in merge order (steps 1 to 5).

    The 3D follower's (``follower_3d``) are only steps 3 and 5: an aerial robot has no Nav2.
    """
    from vectorfield_bringup import demo

    if driver not in DRIVERS:
        raise ValueError(f'driver {driver!r} is not one of {", ".join(DRIVERS)}')
    if driver == 'follower_3d':
        return [
            share('config', 'mujoco_sim.yaml', share_dir=share_dir),
            robot_config(robot, share_dir),
        ]
    scenarios = ['curve_follower'] if driver == 'curve_follower' else []
    return demo.overlay_paths(ROBOT_MODELS.get(robot, robot), scenarios, bringup_share_dir) + [
        share('config', 'mujoco_sim.yaml', share_dir=share_dir),
        share('config', 'nav2.yaml', share_dir=share_dir),
        robot_config(robot, share_dir),
    ]


def merged_params(robot, driver, scenario_params=None, share_dir=None, bringup_share_dir=None):
    """Return the merged parameters of an example (see the module docstring).

    Nav2's for the ground drivers; for ``follower_3d`` the overlays alone, without nav2_bringup's
    file.
    """
    from vectorfield_bringup import params

    paths = overlay_paths(robot, driver, share_dir, bringup_share_dir)
    if driver == 'follower_3d':
        merged = {}
        for path in paths:
            merged = params.deep_merge(merged, params.load_yaml(path))
        return params.deep_merge(merged, scenario_params) if scenario_params else merged
    # The scenario's params last, through merge()'s substitutions, behaviour-tree paths and
    # Humble's plugin names like every overlay.
    return params.merge(paths + ([scenario_params] if scenario_params else []))


def parse_obstacles_argument(value, scenario, share_dir=None):
    """Return the obstacles of the ``obstacles`` launch argument.

    ``''``: the scenario's; ``none``: no obstacles; a YAML list (``[{type: box, ...}]``); else a
    scenario name or path whose obstacles are used.
    """
    from vectorfield_mujoco import simulation

    value = value.strip()
    if not value:
        return list(scenario.get('obstacles', []))
    if value.lower() == 'none':
        return []
    if value.startswith('['):
        return simulation.parse_obstacles(value)
    return load_scenario(resolve_scenario(value, share_dir))['obstacles']


# ------------------------------------------------------------------------------------- launch


def _launch_setup(context, driver, example):
    from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
    from launch.conditions import IfCondition
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    from vectorfield_bringup import demo, params

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    distro = params.ros_distro()
    aerial = driver == 'follower_3d'
    robot = arg('robot')
    robots = list(AERIAL_ROBOTS) if aerial else sorted(ROBOT_MODELS)
    if robot not in robots:
        raise ValueError(f'robot {robot!r} is not one of {", ".join(robots)}')
    # A drone may fly without a map (no walls); Nav2 needs one.
    map_yaml = demo.resolve_config(arg('world'), 'maps') if arg('world') or not aerial else ''
    if example is not None:
        scenario_file = resolve_scenario(example)
        run = IfCondition(arg('scenario')).evaluate(context)
    else:
        scenario_file = resolve_scenario(arg('scenario')) if arg('scenario') else None
        run = scenario_file is not None
    scenario = load_scenario(scenario_file) if scenario_file else validate_scenario({})
    if run and driver_of(scenario) not in (None, driver):
        raise ValueError(
            f'{os.path.basename(scenario_file)} needs the {driver_of(scenario)} launch, '
            f'not {driver}'
        )
    obstacles = parse_obstacles_argument(arg('obstacles'), scenario)
    pose = [
        float(arg(f'initial_pose_{axis}') or default)
        for axis, default in zip(('x', 'y', 'yaw'), scenario['initial_pose'])
    ]
    composed = not aerial and IfCondition(arg('use_composition')).evaluate(context)

    merged = merged_params(robot, driver, scenario['params'])
    params_file = params.write_params(merged)
    # An explicit record_camera takes that camera's defaults; else the scenario's camera and view.
    recording = scenario['recording']
    view = {} if arg('record_camera') else recording['view']
    view = {k: list(v) if isinstance(v, tuple) else v for k, v in view.items()}
    sim_parameters = {
        'robot': robot,
        'world_map': map_yaml,
        'obstacles': yaml.safe_dump(obstacles, default_flow_style=True).strip(),
        'initial_pose_x': pose[0],
        'initial_pose_y': pose[1],
        'initial_pose_yaw': pose[2],
        'viewer': IfCondition(arg('viewer')).evaluate(context),
        'camera_topic': arg('camera_topic'),
        'record_path': arg('record_path'),
        'record_camera': arg('record_camera') or recording['camera'],
        'record_view': yaml.safe_dump(view, default_flow_style=True).strip() if view else '',
        'record_hide': recording['hide'] or [''],
        'record_trail': recording['trail'],
        'real_time_factor': float(arg('real_time_factor')),
        'overlay_topics': OVERLAY_TOPICS[driver],
    }
    if aerial:
        sim_parameters['initial_pose_z'] = float(arg('initial_pose_z') or scenario['initial_z'])
    else:
        # Humble has no collision_monitor in its command chain; bringup adds one.
        sim_parameters['cmd_vel_topic'] = demo.sim_cmd_vel_topic(distro)
    sim = Node(
        package=PACKAGE,
        executable='mujoco_sim',
        name='mujoco_sim',
        parameters=[params_file, sim_parameters],
        output='screen',
        # A shutdown finishes the recording: give ffmpeg time before SIGTERM.
        sigterm_timeout='20',
    )
    if aerial:
        # As follower3d_demo: the CBF filter runs against the simulator's obstacle cloud.
        follower = {
            'use_sim_time': True,
            'cmd_frame': arg('cmd_frame'),
            'voxel_cloud_topic': 'obstacle_cloud',
        }
        actions = [sim] + demo.follower3d_actions([params_file, follower], arg('use_rviz'))
    else:
        actions = demo.nav2_demo_actions(
            context, distro, params_file, map_yaml, [sim], pose, arg('use_rviz'), composed
        )
    if driver == 'curve_follower':
        actions += demo.curve_follower_actions(merged, distro, composed)
    if run and driver_of(scenario) is not None:
        # No name: a __node remapping would rename both of the runner's nodes.
        runner = Node(
            package=PACKAGE, executable='run_scenario', arguments=[scenario_file], output='screen'
        )
        actions.append(runner)
        if IfCondition(arg('exit_on_done')).evaluate(context):

            def finished(event, _context):
                outcome = 'succeeded' if event.returncode == 0 else 'failed'
                return [
                    LogInfo(
                        msg=f'Scenario {outcome} (exit code {event.returncode}); shutting down'
                    ),
                    EmitEvent(event=Shutdown(reason=f'scenario {outcome}')),
                ]

            actions.append(
                RegisterEventHandler(OnProcessExit(target_action=runner, on_exit=finished))
            )
    return actions


def launch_description(driver, example=None, **defaults):
    """Return the launch description of a MuJoCo example.

    Args:
        driver: One of :data:`DRIVERS`.
        example: The example's scenario name: the ``scenario`` argument is then ``true`` (drive
            it) or ``false`` (only its world); without it, ``scenario`` is a scenario name or
            path (empty: none).
        defaults: Argument defaults: ``robot``, ``world``, ``obstacles``, ``scenario``,
            ``use_rviz``, ``viewer``, ``camera_topic``, ``record_path``, ``record_camera``,
            ``exit_on_done``, ``real_time_factor``, ``initial_pose_x`` (and ``_y``, ``_yaw``);
            the ground drivers' ``use_composition``; ``follower_3d``'s ``cmd_frame`` and
            ``initial_pose_z``.

    """
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    from vectorfield_bringup import demo

    if driver not in DRIVERS:
        raise ValueError(f'driver {driver!r} is not one of {", ".join(DRIVERS)}')
    generic = 'mujoco_drone' if driver == 'follower_3d' else f'mujoco_{driver}'
    refusal = demo.refuse_unsupported(f'{example or generic}.launch.py')
    if refusal is not None:
        return refusal
    scenario_help = (
        f'true: drive the {example} scenario (config/scenarios/{example}.yaml) with '
        'run_scenario; false: only its world'
        if example
        else 'A scenario: a name in config/scenarios or a YAML path; its obstacles are added and '
        'run_scenario drives its route. Empty: none.'
    )
    aerial = driver == 'follower_3d'
    robots = list(AERIAL_ROBOTS) if aerial else sorted(ROBOT_MODELS)
    arguments = [
        ('robot', robots[0] if aerial else 'scout_mini', f'Robot: {", ".join(robots)}.'),
        (
            'world',
            '' if aerial else 'open_10x10',
            'Map: a name in vectorfield_bringup/maps (open_10x10, corridor_door, '
            'corridor_corner) or a map YAML path; its walls are built in MuJoCo.'
            + (' Empty: none.' if aerial else ''),
        ),
        (
            'obstacles',
            '',
            "Unmapped obstacles: empty for the scenario's, none, a scenario name or path, or a "
            'YAML list such as "[{type: box, pos: [2, 0], size: [0.2, 0.2, 0.4]}]".',
        ),
        ('scenario', 'true' if example else '', scenario_help),
        (
            'exit_on_done',
            'false',
            'Shut the launch down when the scenario finishes (true for recordings and tests).',
        ),
        (
            'use_rviz',
            'true',
            f'Start RViz with {"follower_3d" if aerial else "vectorfield"}.rviz.',
        ),
        ('use_composition', 'true', 'Run the Nav2 servers in component containers.'),
        ('viewer', 'false', 'Open the MuJoCo viewer (needs a display).'),
        ('camera_topic', '', 'Publish an offscreen camera Image on this topic (empty: off).'),
        ('record_path', '', 'Record an MP4 here, in simulated time (empty: off).'),
        (
            'record_camera',
            '',
            "Recording camera: chase, follow, top or fixed; empty: the scenario's recording "
            'camera and view (else chase).',
        ),
        ('real_time_factor', '1.0', 'Simulated seconds per wall second.'),
        ('initial_pose_x', '', "Initial map x (m); empty: the scenario's (else 0)."),
        ('initial_pose_y', '', "Initial map y (m); empty: the scenario's (else 0)."),
        ('initial_pose_yaw', '', "Initial map yaw (rad); empty: the scenario's (else 0)."),
    ]
    if aerial:
        # No Nav2: no containers.
        arguments = [a for a in arguments if a[0] != 'use_composition'] + [
            (
                'initial_pose_z',
                '',
                "Initial height (m); empty: the scenario's initial_z (else 0, on the floor).",
            ),
            (
                'cmd_frame',
                'global',
                "Frame of the 3D follower's velocity setpoints: global or body (heading frame).",
            ),
        ]
    unknown = set(defaults) - {name for name, _, _ in arguments}
    if unknown:
        raise ValueError(f'unknown launch argument defaults {sorted(unknown)}')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name, default_value=str(defaults.get(name, value)), description=description
            )
            for name, value, description in arguments
        ]
        + [OpaqueFunction(function=_launch_setup, args=[driver, example])]
    )
