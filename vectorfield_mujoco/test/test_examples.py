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

"""Checks of the examples' configs, scenarios and launch files (vectorfield_mujoco.examples).

The shipped-config rules of vectorfield_bringup apply to every robot config merged onto its
motion-model overlay: full [x, y, theta] smoother lists covering the controller's limits, y only
for the omnidirectional base, the costmaps' footprint; and on top of them the controller's limits
stay within the actuators' (read from the compiled models; the Menagerie robots need their
checkout) and every example's merged parameters pass vectorfield's validate_config. Scenarios
load, name a route their launch can drive, and their obstacles cross the robot's lidar plane.
"""

import ast
import importlib.util
import math
from pathlib import Path

import pytest
import yaml

from vectorfield_mujoco import examples, simulation

PACKAGE = Path(__file__).resolve().parents[1]
SHARE = str(PACKAGE)
ROBOTS = sorted(examples.ROBOT_MODELS)
SCENARIOS = sorted((PACKAGE / 'config' / 'scenarios').glob('*.yaml'))
EXAMPLE_LAUNCHES = sorted(
    p for p in (PACKAGE / 'launch').glob('*.launch.py') if not p.name.startswith('mujoco_')
)
COMMON_PARAMETERS = 'vectorfield_common_params.yaml'


def _defaults():
    """Return the controller's parameter defaults from the installed parameter definitions."""
    from ament_index_python.packages import get_package_share_directory

    path = Path(get_package_share_directory('vectorfield_ros')) / 'params' / COMMON_PARAMETERS
    defaults = {}
    for group in yaml.safe_load(path.read_text(encoding='utf-8')).values():
        defaults.update({k: v['default_value'] for k, v in group.items() if isinstance(v, dict)})
    return defaults


def _node(merged, name):
    return merged[name]['ros__parameters']


def _launch_presets(path):
    """Return the keyword arguments of the launch_description() call of an example launch."""
    for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
        if isinstance(node, ast.Call) and getattr(node.func, 'attr', '') == 'launch_description':
            presets = {k.arg: ast.literal_eval(k.value) for k in node.keywords}
            presets['driver'] = ast.literal_eval(node.args[0])
            return presets
    raise AssertionError(f'{path.name} does not call examples.launch_description')


def _merged(robot, driver='nav2', scenario_params=None):
    return examples.merged_params(robot, driver, scenario_params, share_dir=SHARE)


def _limits(merged):
    return {**_defaults(), **_node(merged, 'controller_server')['FollowPath']}


@pytest.mark.parametrize('robot', ROBOTS)
def test_robot_config_merges_onto_its_model(robot):
    """FollowPath stays the vector-field controller of the robot's model, with known keys."""
    merged = _merged(robot)
    follow_path = _node(merged, 'controller_server')['FollowPath']
    assert follow_path['plugin'] == 'vectorfield_nav2_controller::VectorFieldController'
    assert follow_path['motion_model'] == examples.ROBOT_MODELS[robot]
    unknown = set(follow_path) - set(_defaults()) - {'plugin'}
    assert not unknown, sorted(unknown)
    overlay = yaml.safe_load(Path(examples.robot_config(robot, SHARE)).read_text())
    assert 'plugin' not in overlay['controller_server']['ros__parameters']['FollowPath']


@pytest.mark.parametrize('robot', ROBOTS)
def test_smoother_limits_cover_the_controller(robot):
    """Full [x, y, theta] lists, x and theta at least the controller's, y only for omni."""
    merged = _merged(robot)
    limits = _limits(merged)
    smoother = _node(merged, 'velocity_smoother')
    for key in examples.SMOOTHER_LIMITS:
        assert len(smoother[key]) == 3, key
    max_v, min_v = smoother['max_velocity'], smoother['min_velocity']
    max_a, max_d = smoother['max_accel'], smoother['max_decel']
    assert max_v[0] >= limits['max_linear_vel'] and -min_v[0] >= limits['max_linear_vel']
    assert max_v[2] >= limits['max_angular_vel'] and -min_v[2] >= limits['max_angular_vel']
    assert max_a[0] >= limits['max_linear_accel'] and -max_d[0] >= limits['max_linear_decel']
    assert max_a[2] >= limits['max_angular_accel'] and -max_d[2] >= limits['max_angular_accel']
    if limits['motion_model'] == 'omnidirectional':
        assert max_v[1] == -min_v[1] >= limits['max_lateral_vel'] > 0.0
        assert max_a[1] >= limits['max_linear_accel'] and -max_d[1] >= limits['max_linear_decel']
    else:
        assert max_v[1] == min_v[1] == max_a[1] == max_d[1] == 0.0
    assert smoother['scale_velocities'] is True
    assert limits['desired_speed'] <= limits['max_linear_vel']


@pytest.mark.parametrize('robot', ROBOTS)
def test_costmaps_share_the_footprint(robot):
    """Both costmaps carry the robot's footprint (or radius); the collision monitor follows it."""
    merged = _merged(robot)
    local = merged['local_costmap']['local_costmap']['ros__parameters']
    global_ = merged['global_costmap']['global_costmap']['ros__parameters']
    overlay = yaml.safe_load(Path(examples.robot_config(robot, SHARE)).read_text())
    shape = overlay['local_costmap']['local_costmap']['ros__parameters']
    assert set(shape) & {'footprint', 'robot_radius'}
    for key, value in shape.items():
        assert local[key] == global_[key] == value
    monitor = merged.get('collision_monitor', {}).get('ros__parameters', {})
    for polygon in monitor.get('polygons', []):
        assert 'published_footprint' in monitor[polygon].get('footprint_topic', ''), polygon


def _model_limits(robot):
    """Return the robot's (linear, angular) speed limits from its actuators, or skip."""
    from vectorfield_mujoco import menagerie, registry, scene

    entry = registry.get(robot)
    if entry.source == 'menagerie' and not menagerie.is_available():
        pytest.skip('MuJoCo Menagerie not fetched (fetch_menagerie)')
    pytest.importorskip('mujoco')
    world = scene.make_world(robot)
    model, g = world.model, world.geometry
    ranges = {
        role: float(model.actuator_ctrlrange[model.actuator(world.name(name)).id, 1])
        for role, name in entry.actuators.items()
    }
    if entry.adapter in ('diff_drive', 'skid_steer'):
        wheel = min(ranges.values())
        return wheel * g['radius'], wheel * g['radius'] / (0.5 * g['track']), g
    if entry.adapter == 'ackermann':
        wheel = min(ranges['rear_left'], ranges['rear_right'])
        return wheel * g['radius'], math.inf, g
    return min(ranges['x'], ranges['y']), ranges['theta'], g


@pytest.mark.parametrize('robot', ROBOTS)
def test_limits_are_within_the_actuators(robot):
    """The controller's and the smoother's limits are within what the actuators can do."""
    linear, angular, geometry = _model_limits(robot)
    merged = _merged(robot)
    limits = _limits(merged)
    smoother = _node(merged, 'velocity_smoother')
    assert max(limits['max_linear_vel'], smoother['max_velocity'][0]) <= linear
    assert limits['max_angular_vel'] <= angular
    assert limits['rotate_to_heading_angular_vel'] <= angular
    if limits['motion_model'] == 'ackermann':
        # The inner front wheel's angle at min_turning_radius fits the steering hinges.
        radius = limits['min_turning_radius']
        inner = math.atan(geometry['wheelbase'] / (radius - 0.5 * geometry['steer_track']))
        assert inner <= geometry['max_steer']
        assert limits['reference_point_offset'] == 0.0  # base_link is the rear axle


def _validate(merged, plugin='FollowPath', follower_only=()):
    vf = pytest.importorskip('vectorfield')
    from vectorfield_tools import config

    loaded = config.config_from_mapping(merged, plugin=plugin)
    assert set(loaded.unknown) <= set(follower_only), loaded.unknown
    return vf.validate_config(loaded.config, loaded.costmap)


def test_scout_slip_compensation_is_identified():
    """The Scout Mini compensates its skid slip (identified with identify_skid_slip.py)."""
    follow_path = _node(_merged('scout_mini'), 'controller_server')['FollowPath']
    assert 0.3 < follow_path['skid_yaw_rate_efficiency'] < 1.0
    assert (
        abs(follow_path['skid_icr_offset'])
        < 0.5 * _limits(_merged('scout_mini'))['control_point_offset']
    )


@pytest.mark.parametrize('scenario', SCENARIOS, ids=lambda p: p.stem)
def test_scenario_loads_and_matches_its_launch(scenario):
    """Each scenario has an example launch whose driver can drive its route."""
    loaded = examples.load_scenario(scenario)
    launch = PACKAGE / 'launch' / f'{scenario.stem}.launch.py'
    assert launch.is_file()
    presets = _launch_presets(launch)
    assert presets['example'] == scenario.stem
    assert examples.driver_of(loaded) == presets['driver']
    assert presets['robot'] in examples.ROBOT_MODELS


@pytest.mark.parametrize('launch', EXAMPLE_LAUNCHES, ids=lambda p: p.name)
def test_example_validates(launch):
    """The merged parameters of every example pass validate_config, without warnings."""
    presets = _launch_presets(launch)
    scenario = examples.load_scenario(
        PACKAGE / 'config' / 'scenarios' / f'{launch.name[:-10]}.yaml'
    )
    merged = _merged(presets['robot'], presets['driver'], scenario['params'])
    report = _validate(merged)
    assert not list(report.errors) and not list(report.warnings), report.summary()
    if presets['driver'] == 'curve_follower':
        from vectorfield_bringup import demo

        follower = {demo.FOLLOWER: {'ros__parameters': demo.follower_parameters(merged)}}
        # moving_frame_reference is the follower's own key (bringup's curve_follower overlay).
        report = _validate(follower, demo.FOLLOWER, follower_only=('moving_frame_reference',))
        assert not list(report.errors), report.summary()


@pytest.mark.parametrize('launch', EXAMPLE_LAUNCHES, ids=lambda p: p.name)
def test_obstacles_cross_the_lidar_plane(launch):
    """Every unmapped obstacle of an example is taller than its robot's lidar."""
    from vectorfield_mujoco import menagerie, registry, scene

    presets = _launch_presets(launch)
    scenario = examples.load_scenario(
        PACKAGE / 'config' / 'scenarios' / f'{launch.name[:-10]}.yaml'
    )
    if not scenario['obstacles']:
        return
    if registry.get(presets['robot']).source == 'menagerie' and not menagerie.is_available():
        pytest.skip('MuJoCo Menagerie not fetched (fetch_menagerie)')
    pytest.importorskip('mujoco')
    world = scene.make_world(presets['robot'])
    height = float(world.data.site_xpos[world.scan_site][2])
    assert simulation.unseen_obstacles(scenario['obstacles'], height) == []


def test_scenario_validation_rejects_mistakes():
    """Unknown keys, both route kinds, bad laps and smoother limits are rejected."""
    for bad in (
        {'waypoint': [[1, 0, 0]]},
        {'waypoints': [[1, 0, 0]], 'curve': 'ellipse'},
        {'waypoints': [[1, 0]]},
        {'waypoints': []},
        {'curve': 'moving'},
        {'curve': 'ellipse', 'laps': 0},
        {'laps': 2},
        {'initial_pose': [0, 0]},
        {'params': {'velocity_smoother': {'ros__parameters': {'max_velocity': [1, 0, 1]}}}},
    ):
        with pytest.raises(ValueError):
            examples.validate_scenario(bad)
    plain = examples.validate_scenario({})
    assert plain['obstacles'] == [] and plain['initial_pose'] == [0.0, 0.0, 0.0]
    assert examples.driver_of(plain) is None


def test_obstacles_argument():
    """The launch's obstacles argument: the scenario's, none, a YAML list or a scenario."""
    scenario = examples.load_scenario(PACKAGE / 'config' / 'scenarios' / 'differential.yaml')
    assert examples.parse_obstacles_argument('', scenario) == scenario['obstacles']
    assert examples.parse_obstacles_argument('none', scenario) == []
    inline = '[{type: cylinder, pos: [1, 1], size: [0.1, 0.3]}]'
    assert examples.parse_obstacles_argument(inline, scenario)[0]['type'] == 'cylinder'
    path = str(PACKAGE / 'config' / 'scenarios' / 'ackermann.yaml')
    assert examples.parse_obstacles_argument(path, {}) == examples.load_scenario(path)['obstacles']


def _runner():
    spec = importlib.util.spec_from_file_location(
        'run_scenario', PACKAGE / 'scripts' / 'run_scenario.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_runner_commands():
    """run_scenario drives waypoints with navigate_waypoints.py and curves with send_curve.py."""
    pytest.importorskip('rclpy')
    runner = _runner()
    route = runner.client_command(
        examples.validate_scenario({'waypoints': [[1, 0, 0], [2, 1, 1.5]]})
    )
    assert Path(route[0]).name == 'navigate_waypoints.py'
    assert route[1:] == ['--poses', '1,0,0 2,1,1.5']
    curve = runner.client_command(
        examples.validate_scenario({'curve': 'superellipse', 'laps': 2, 'speed': 0.3})
    )
    assert Path(curve[0]).name == 'send_curve.py'
    assert curve[1:] == ['--shape', 'superellipse', '--laps', '2', '--speed', '0.3']
    with pytest.raises(ValueError):
        runner.client_command(examples.validate_scenario({}))
