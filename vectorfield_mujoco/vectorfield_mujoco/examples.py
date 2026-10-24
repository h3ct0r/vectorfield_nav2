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
4. ``config/robots/<robot>.yaml``: footprint, limits within the actuators, slip compensation;
5. the scenario's ``params`` (Nav2 behaviour only, never a limit).

A scenario (``config/scenarios/<name>.yaml``, :func:`load_scenario`) holds the unmapped
``obstacles``, an ``initial_pose`` and what ``run_scenario`` drives: ``waypoints`` (Nav2, one
NavigateToPose goal each) or a ``curve`` (a ``vectorfield_bringup.curves.EXAMPLES`` shape) with
``laps`` (the curve follower).

:func:`launch_description` builds both generic launches (``mujoco_nav2.launch.py``,
``mujoco_curve_follower.launch.py``) and the example launches, which only preset its arguments.
"""

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

#: What drives the robot: Nav2 (NavigateToPose with the vector-field controller) or the curve
#: follower (FollowCurve, with Nav2 up for the approach and the costmap).
DRIVERS = ('nav2', 'curve_follower')

#: The keys a scenario file may hold.
SCENARIO_KEYS = (
    'description',
    'initial_pose',
    'obstacles',
    'waypoints',
    'curve',
    'laps',
    'speed',
    'params',
)

#: Overlay topics drawn into the MuJoCo views, per driver.
OVERLAY_TOPICS = {
    'nav2': ['/plan', '/FollowPath/markers'],
    'curve_follower': ['/curve_follower/markers'],
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
    if robot not in ROBOT_MODELS:
        raise ValueError(f'robot {robot!r} is not one of {", ".join(sorted(ROBOT_MODELS))}')
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
        ValueError: An unknown key, a malformed value, both or neither of ``waypoints`` and
            ``curve`` when a route is given, or ``params`` that set velocity smoother limits.

    """
    from vectorfield_bringup import curves
    from vectorfield_mujoco import simulation

    if not isinstance(data, dict):
        raise ValueError(f'{name}: a scenario must be a mapping')
    unknown = set(data) - set(SCENARIO_KEYS)
    if unknown:
        raise ValueError(f'{name}: unknown keys {sorted(unknown)}; known: {list(SCENARIO_KEYS)}')
    result = dict(data)
    result['obstacles'] = simulation.parse_obstacles(data.get('obstacles') or [])
    result['initial_pose'] = _pose(data.get('initial_pose', [0.0, 0.0, 0.0]), 'initial_pose')
    if 'waypoints' in data and 'curve' in data:
        raise ValueError(f'{name}: give waypoints or a curve, not both')
    if 'waypoints' in data:
        waypoints = data['waypoints']
        if not isinstance(waypoints, list) or not waypoints:
            raise ValueError(f'{name}: waypoints must be a non-empty list of [x, y, yaw]')
        result['waypoints'] = [_pose(w, 'a waypoint') for w in waypoints]
    if 'curve' in data:
        if data['curve'] not in curves.EXAMPLES or data['curve'] in curves.MOVING_EXAMPLES:
            static = sorted(set(curves.EXAMPLES) - set(curves.MOVING_EXAMPLES))
            raise ValueError(f'{name}: curve must be one of {static}, not {data["curve"]!r}')
        laps = data.get('laps', 1)
        if not isinstance(laps, int) or isinstance(laps, bool) or laps < 1:
            raise ValueError(f'{name}: laps must be a whole number >= 1, not {laps!r}')
        result['laps'] = laps
    elif 'laps' in data:
        raise ValueError(f'{name}: laps needs a curve')
    result['speed'] = float(data.get('speed', 0.0))
    params = data.get('params') or {}
    if not isinstance(params, dict):
        raise ValueError(f'{name}: params must be a Nav2 parameter mapping')
    smoother = params.get('velocity_smoother', {}).get('ros__parameters', {})
    if set(smoother) & set(SMOOTHER_LIMITS):
        raise ValueError(f'{name}: params must not set the velocity smoother limits')
    result['params'] = params
    return result


def load_scenario(path):
    """Load and validate a scenario file (:func:`validate_scenario`)."""
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    return validate_scenario(data, os.path.basename(path))


def driver_of(scenario):
    """Return the driver a scenario's route needs (``'nav2'``, ``'curve_follower'``) or None."""
    if 'curve' in scenario:
        return 'curve_follower'
    if 'waypoints' in scenario:
        return 'nav2'
    return None


def overlay_paths(robot, driver, share_dir=None, bringup_share_dir=None):
    """Return the overlay files of :func:`merged_params`, in merge order (steps 1 to 4)."""
    from vectorfield_bringup import demo

    if driver not in DRIVERS:
        raise ValueError(f'driver {driver!r} is not one of {", ".join(DRIVERS)}')
    scenarios = ['curve_follower'] if driver == 'curve_follower' else []
    return demo.overlay_paths(ROBOT_MODELS.get(robot, robot), scenarios, bringup_share_dir) + [
        share('config', 'mujoco_sim.yaml', share_dir=share_dir),
        robot_config(robot, share_dir),
    ]


def merged_params(robot, driver, scenario_params=None, share_dir=None, bringup_share_dir=None):
    """Return the merged Nav2 parameters of an example (see the module docstring)."""
    from vectorfield_bringup import params

    merged = params.merge(overlay_paths(robot, driver, share_dir, bringup_share_dir))
    if scenario_params:
        merged = params.deep_merge(merged, scenario_params)
    return merged


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
    robot = arg('robot')
    if robot not in ROBOT_MODELS:
        raise ValueError(f'robot {robot!r} is not one of {", ".join(sorted(ROBOT_MODELS))}')
    map_yaml = demo.resolve_config(arg('world'), 'maps')
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
    composed = IfCondition(arg('use_composition')).evaluate(context)

    merged = merged_params(robot, driver, scenario['params'])
    params_file = params.write_params(merged)
    sim = Node(
        package=PACKAGE,
        executable='mujoco_sim',
        name='mujoco_sim',
        parameters=[
            params_file,
            {
                'robot': robot,
                'world_map': map_yaml,
                'obstacles': yaml.safe_dump(obstacles, default_flow_style=True).strip(),
                'initial_pose_x': pose[0],
                'initial_pose_y': pose[1],
                'initial_pose_yaw': pose[2],
                # Humble has no collision_monitor in its command chain; bringup adds one.
                'cmd_vel_topic': demo.sim_cmd_vel_topic(distro),
                'viewer': IfCondition(arg('viewer')).evaluate(context),
                'camera_topic': arg('camera_topic'),
                'record_path': arg('record_path'),
                'record_camera': arg('record_camera'),
                'real_time_factor': float(arg('real_time_factor')),
                'overlay_topics': OVERLAY_TOPICS[driver],
            },
        ],
        output='screen',
        # A shutdown finishes the recording: give ffmpeg time before SIGTERM.
        sigterm_timeout='20',
    )
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
            ``exit_on_done``, ``real_time_factor``, ``initial_pose_x`` (and ``_y``, ``_yaw``).

    """
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    from vectorfield_bringup import demo

    if driver not in DRIVERS:
        raise ValueError(f'driver {driver!r} is not one of {", ".join(DRIVERS)}')
    refusal = demo.refuse_unsupported(f'{example or "mujoco_" + driver}.launch.py')
    if refusal is not None:
        return refusal
    scenario_help = (
        f'true: drive the {example} scenario (config/scenarios/{example}.yaml) with '
        'run_scenario; false: only its world'
        if example
        else 'A scenario: a name in config/scenarios or a YAML path; its obstacles are added and '
        'run_scenario drives its route. Empty: none.'
    )
    arguments = [
        ('robot', 'scout_mini', f'Robot: {", ".join(sorted(ROBOT_MODELS))}.'),
        (
            'world',
            'open_10x10',
            'Map: a name in vectorfield_bringup/maps (open_10x10, corridor_door, '
            'corridor_corner) or a map YAML path; its walls are built in MuJoCo.',
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
        ('use_rviz', 'true', 'Start RViz with vectorfield.rviz.'),
        ('use_composition', 'true', 'Run the Nav2 servers in component containers.'),
        ('viewer', 'false', 'Open the MuJoCo viewer (needs a display).'),
        ('camera_topic', '', 'Publish an offscreen camera Image on this topic (empty: off).'),
        ('record_path', '', 'Record an MP4 here, in simulated time (empty: off).'),
        ('record_camera', 'chase', 'Recording camera: chase, top or fixed.'),
        ('real_time_factor', '1.0', 'Simulated seconds per wall second.'),
        ('initial_pose_x', '', "Initial map x (m); empty: the scenario's (else 0)."),
        ('initial_pose_y', '', "Initial map y (m); empty: the scenario's (else 0)."),
        ('initial_pose_yaw', '', "Initial map yaw (rad); empty: the scenario's (else 0)."),
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
