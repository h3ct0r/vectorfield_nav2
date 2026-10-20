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

"""Checks of the shipped overlays, merged onto the running distro's nav2_params.yaml.

They enforce the overlay rules of plan section 8: velocity_smoother ownership, the omni and
Ackermann specifics, and that the overlays only name real controller parameters. The copy-paste
templates of ``config/examples`` follow the same rules. The controller validation of each overlay
and template (validateFromNode) runs in vectorfield_system_tests.
"""

import re
from pathlib import Path

import pytest
import yaml

from vectorfield_bringup import demo, params

PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parent
CONTROLLER = PACKAGE / 'config' / 'controller'
SCENARIO = PACKAGE / 'config' / 'scenario'
TEMPLATE_PREFIX = 'nav2_params_vectorfield_'
EXAMPLES = PACKAGE / 'config' / 'examples'
TEMPLATES = sorted(EXAMPLES.glob(TEMPLATE_PREFIX + '*.yaml'))
SHIM = 'nav2_rotation_shim_controller::RotationShimController'
SHIM_KEYS = {'angular_dist_threshold', 'forward_sampling_distance', 'rotate_to_goal_heading'}
PARAMETER_FILES = (
    REPOSITORY / 'vectorfield_ros' / 'params' / 'vectorfield_common_params.yaml',
    REPOSITORY / 'vectorfield_nav2_controller' / 'params' / 'vectorfield_legacy_path_params.yaml',
)
SMOOTHER_LIMITS = ('max_velocity', 'min_velocity', 'max_accel', 'max_decel')
PLUGIN = 'vectorfield_nav2_controller::VectorFieldController'


def _defaults():
    """Return the controller's parameter defaults from the generate_parameter_library files."""
    defaults = {}
    for path in PARAMETER_FILES:
        if not path.is_file():
            pytest.skip(f'{path} is not available')
        for group in yaml.safe_load(path.read_text(encoding='utf-8')).values():
            defaults.update(
                {
                    name: spec['default_value']
                    for name, spec in group.items()
                    if isinstance(spec, dict) and 'default_value' in spec
                }
            )
    return defaults


def _merged(model, scenarios=('kinematic_sim',)):
    overlays = demo.overlay_paths(model, list(scenarios), str(PACKAGE))
    return params.merge(overlays)


def _node(merged, name):
    return merged[name]['ros__parameters']


@pytest.mark.parametrize('model', demo.MOTION_MODELS)
def test_controller_overlay_replaces_follow_path(model):
    """FollowPath is ours, names the overlay's model and holds only real parameters."""
    defaults = _defaults()
    follow_path = _node(_merged(model), 'controller_server')['FollowPath']
    assert follow_path['plugin'] == PLUGIN
    assert follow_path['motion_model'] == model
    unknown = set(follow_path) - set(defaults) - {'plugin'}
    assert not unknown, f'{model}.yaml sets unknown FollowPath parameters {sorted(unknown)}'
    restated = {k: v for k, v in follow_path.items() if k in defaults and k != 'motion_model'}
    assert all(isinstance(v, type(defaults[k])) for k, v in restated.items()), restated


@pytest.mark.parametrize('model', demo.MOTION_MODELS)
def test_smoother_limits_cover_the_controller(model):
    """Full [x, y, theta] lists, x and theta at least the controller's, y only for omni."""
    _assert_smoother_covers_the_controller(_merged(model), model)


def _assert_smoother_covers_the_controller(merged, model):
    defaults = _defaults()
    limits = {**defaults, **_node(merged, 'controller_server')['FollowPath']}
    smoother = _node(merged, 'velocity_smoother')
    for key in SMOOTHER_LIMITS:
        assert len(smoother[key]) == 3, key
    max_v, min_v = smoother['max_velocity'], smoother['min_velocity']
    max_a, max_d = smoother['max_accel'], smoother['max_decel']
    assert max_v[0] >= limits['max_linear_vel']
    assert max_v[2] >= limits['max_angular_vel']
    assert -min_v[2] >= limits['max_angular_vel']
    assert max_a[0] >= limits['max_linear_accel']
    assert -max_d[0] >= limits['max_linear_decel']
    assert max_a[2] >= limits['max_angular_accel']
    assert -max_d[2] >= limits['max_angular_accel']
    if model == 'omnidirectional':
        assert max_v[1] == -min_v[1] >= limits['max_lateral_vel'] > 0.0
        assert max_a[1] >= limits['max_linear_accel']
        assert -max_d[1] >= limits['max_linear_decel']
        assert _node(merged, 'controller_server')['min_y_velocity_threshold'] <= 0.001
    else:
        assert max_v[1] == min_v[1] == max_a[1] == max_d[1] == 0.0
    assert smoother['scale_velocities'] is True


@pytest.mark.parametrize('model', demo.MOTION_MODELS)
def test_sim_matches_the_controller(model):
    """vectorfield_sim simulates the overlay's model (and the Ackermann turning radius)."""
    merged = _merged(model)
    follow_path = _node(merged, 'controller_server')['FollowPath']
    sim = _node(merged, 'vectorfield_sim')
    assert sim['motion_model'] == model
    if model == 'ackermann':
        assert sim['min_turning_radius'] == follow_path['min_turning_radius'] > 0.0


def test_ackermann_overlay():
    """Dubins Hybrid-A* with the car's radius, a position goal checker and a tree without Spin."""
    merged = _merged('ackermann')
    follow_path = _node(merged, 'controller_server')['FollowPath']
    assert follow_path['obstacle_mode'] == 'none'
    planner = _node(merged, 'planner_server')['GridBased']
    assert planner['plugin'].replace('/', '::') == 'nav2_smac_planner::SmacPlannerHybrid'
    assert planner['motion_model_for_search'] == 'DUBIN'
    assert planner['minimum_turning_radius'] == follow_path['min_turning_radius']
    assert 'use_astar' not in planner
    checker = _node(merged, 'controller_server')['general_goal_checker']
    assert checker['plugin'] == 'nav2_controller::PositionGoalChecker'
    assert 'yaw_goal_tolerance' not in checker
    tree = Path(_node(merged, 'bt_navigator')['default_nav_to_pose_bt_xml'])
    assert tree.is_absolute() and tree.is_file()
    assert tree.name == 'navigate_w_replanning_time.xml'
    assert 'Spin' not in tree.read_text(encoding='utf-8')


def test_humble_plugin_names():
    """On Humble no merged plugin name uses '::' for Smac, NavFn or behaviors."""
    if params.ros_distro() != 'humble':
        pytest.skip('Humble only')
    text = yaml.safe_dump(_merged('ackermann'))
    for package in params.HUMBLE_SLASH_PACKAGES:
        assert f'{package}::' not in text


@pytest.mark.parametrize('path', sorted(SCENARIO.glob('*.yaml')), ids=lambda p: p.stem)
def test_scenarios_never_set_smoother_limits(path):
    """Scenario overlays leave the smoother limits to the motion-model overlays."""
    overlay = params.load_yaml(str(path))
    smoother = overlay.get('velocity_smoother', {}).get('ros__parameters', {})
    assert set(smoother) <= {'scale_velocities'}
    assert 'controller_server' not in overlay, 'controller parameters belong in config/controller'


@pytest.mark.parametrize(
    'path',
    sorted(CONTROLLER.glob('*.yaml'))
    + sorted(SCENARIO.glob('*.yaml'))
    + sorted(EXAMPLES.glob('*.yaml')),
    ids=str,
)
def test_overlays_never_set_stamped_cmd_vel(path):
    """enable_stamped_cmd_vel stays at each node's distro default (plan section 8)."""
    assert 'enable_stamped_cmd_vel' not in path.read_text(encoding='utf-8')


@pytest.mark.parametrize('model', demo.MOTION_MODELS)
def test_vectorfield_sim_demos_compensate_the_skid_slip(model):
    """Overlays leave the slip compensation off; the sim demos set it to the sim's slip."""
    follow_path = _node(_merged(model), 'controller_server')['FollowPath']
    assert follow_path.get('skid_yaw_rate_efficiency', 1.0) == 1.0
    assert follow_path.get('skid_icr_offset', 0.0) == 0.0
    merged = demo.kinematic_sim_params(model, ['kinematic_sim'], str(PACKAGE))
    if model != 'skid_steer':
        assert merged == _merged(model)
        return
    follow_path = _node(merged, 'controller_server')['FollowPath']
    sim = _node(merged, demo.SIM)
    assert (sim['slip_eta_psi'], sim['slip_x_icr']) == demo.SIM_SKID_SLIP
    assert follow_path['skid_yaw_rate_efficiency'] == sim['slip_eta_psi']
    assert follow_path['skid_icr_offset'] == sim['slip_x_icr']


def test_kinematic_sim_params_follow_a_changed_sim_slip(tmp_path):
    """A scenario that changes vectorfield_sim's slip changes the compensation with it."""
    scenario = tmp_path / 'slip.yaml'
    scenario.write_text(
        yaml.safe_dump({demo.SIM: {'ros__parameters': {'slip_eta_psi': 0.9}}}), encoding='utf-8'
    )
    merged = demo.kinematic_sim_params(
        'skid_steer', ['kinematic_sim', str(scenario)], str(PACKAGE)
    )
    follow_path = _node(merged, 'controller_server')['FollowPath']
    assert follow_path['skid_yaw_rate_efficiency'] == 0.9
    assert follow_path['skid_icr_offset'] == demo.SIM_SKID_SLIP[1]


def test_sim_skid_slip_matches_vectorfield_sim():
    """demo.SIM_SKID_SLIP restates kSkidSteerSlip of vectorfield_sim/plant.hpp."""
    header = REPOSITORY / 'vectorfield_sim' / 'include' / 'vectorfield_sim' / 'plant.hpp'
    if not header.is_file():
        pytest.skip(f'{header} is not available')
    match = re.search(r'kSkidSteerSlip\{([^,}]+),([^}]+)\}', header.read_text(encoding='utf-8'))
    assert match, 'kSkidSteerSlip not found'
    assert (float(match.group(1)), float(match.group(2))) == demo.SIM_SKID_SLIP


@pytest.mark.parametrize('model', demo.MOTION_MODELS)
def test_merged_file_is_written(model, tmp_path):
    """The merged file keeps every stock server and loads back unchanged."""
    merged = _merged(model, ('kinematic_sim', 'demo_obstacles'))
    stock = params.load_yaml(params.stock_params_file())
    assert set(stock) <= set(merged)
    path = params.write_params(merged, str(tmp_path / 'merged.yaml'))
    assert params.load_yaml(path) == merged


@pytest.mark.parametrize('model', demo.MOTION_MODELS)
def test_demo_obstacles_keep_the_global_plan(model):
    """demo_obstacles drops only the global obstacle layer; the local costmap keeps its own."""
    merged = _merged(model, ('kinematic_sim', 'demo_obstacles'))
    stock = params.load_yaml(params.stock_params_file())
    plugins = _node(merged['global_costmap'], 'global_costmap')['plugins']
    stock_plugins = _node(stock['global_costmap'], 'global_costmap')['plugins']
    assert 'obstacle_layer' in stock_plugins
    assert plugins == [p for p in stock_plugins if p != 'obstacle_layer']
    assert (
        _node(merged['local_costmap'], 'local_costmap')['plugins']
        == _node(stock['local_costmap'], 'local_costmap')['plugins']
    )


@pytest.mark.parametrize('model', demo.MOTION_MODELS)
def test_follower_parameters(model):
    """curve_follower_demo gives the follower FollowPath without its plugin, then its section."""
    merged = _merged(model, ('kinematic_sim', 'curve_follower'))
    follow_path = _node(merged, 'controller_server')['FollowPath']
    follower = demo.follower_parameters(merged)
    assert 'plugin' not in follower
    assert follower['motion_model'] == model
    assert all(follower[k] == v for k, v in follow_path.items() if k != 'plugin')
    assert (
        follower['moving_frame_reference']
        == _node(merged, demo.FOLLOWER)['moving_frame_reference']
    )


def _template_model(path):
    return path.stem[len(TEMPLATE_PREFIX) :]


def test_every_template_exists():
    """One template per motion model that has its own Nav2 setup (skid-steer: as differential)."""
    assert [_template_model(p) for p in TEMPLATES] == [
        'ackermann',
        'differential',
        'omnidirectional',
    ]


@pytest.mark.parametrize('path', TEMPLATES, ids=lambda p: p.stem)
def test_template_follow_path(path):
    """A template's FollowPath is ours, names its model and holds only real parameters."""
    defaults = _defaults()
    server = _node(params.load_yaml(str(path)), 'controller_server')
    follow_path = server['FollowPath']
    assert follow_path['plugin'] == PLUGIN
    assert follow_path['motion_model'] == _template_model(path)
    unknown = set(follow_path) - set(defaults) - {'plugin'}
    assert not unknown, f'{path.name} sets unknown FollowPath parameters {sorted(unknown)}'
    typed = {k: v for k, v in follow_path.items() if k in defaults}
    assert all(isinstance(v, type(defaults[k])) for k, v in typed.items()), typed
    assert server['controller_plugins'] == ['FollowPath']
    assert server['goal_checker_plugins'] == ['general_goal_checker']


@pytest.mark.parametrize('path', TEMPLATES, ids=lambda p: p.stem)
def test_template_merges_onto_the_stock_file(path):
    """Merged like a scenario overlay, a template keeps the overlay rules."""
    model = _template_model(path)
    merged = params.merge([str(path)])
    _assert_smoother_covers_the_controller(merged, model)
    server = _node(merged, 'controller_server')
    assert server['FollowPath']['plugin'] == PLUGIN
    checker = server['general_goal_checker']
    if model == 'ackermann':
        assert checker['plugin'] == 'nav2_controller::PositionGoalChecker'
        assert 'yaw_goal_tolerance' not in checker
    else:
        assert checker['plugin'] == 'nav2_controller::SimpleGoalChecker'
        assert checker['yaw_goal_tolerance'] > 0.0
    monitor = _node(merged, 'collision_monitor')
    assert monitor['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert monitor['cmd_vel_out_topic'] == 'cmd_vel'


def test_ackermann_template():
    """Dubins Hybrid-A* with the controller's radius and a stock tree without Spin."""
    from ament_index_python.packages import get_package_share_directory

    (path,) = [p for p in TEMPLATES if _template_model(p) == 'ackermann']
    template = params.load_yaml(str(path))
    follow_path = _node(template, 'controller_server')['FollowPath']
    assert follow_path['obstacle_mode'] == 'none'
    planner = _node(template, 'planner_server')['GridBased']
    assert planner['plugin'] == 'nav2_smac_planner::SmacPlannerHybrid'
    assert planner['motion_model_for_search'] == 'DUBIN'
    assert planner['minimum_turning_radius'] == follow_path['min_turning_radius'] > 0.0
    tree = _node(template, 'bt_navigator')['default_nav_to_pose_bt_xml']
    prefix = '$(find-pkg-share nav2_bt_navigator)/'
    assert tree.startswith(prefix)
    stock = Path(get_package_share_directory('nav2_bt_navigator')) / tree[len(prefix) :]
    assert stock.is_file()
    assert 'Spin' not in stock.read_text(encoding='utf-8')


@pytest.mark.parametrize('layout', ['humble_jazzy', 'lyrical'])
def test_rotation_shim_examples(layout):
    """The shim wraps our controller, with our keys where the distro's shim looks for them."""
    defaults = _defaults()
    merged = _merged(
        'differential', ('kinematic_sim', str(EXAMPLES / f'rotation_shim_{layout}.yaml'))
    )
    follow_path = _node(merged, 'controller_server')['FollowPath']
    assert follow_path['plugin'] == SHIM
    if layout == 'lyrical':
        ours = follow_path['primary_controller']
        assert ours['plugin'] == PLUGIN
        assert set(follow_path) - {'plugin', 'primary_controller'} <= SHIM_KEYS | set(defaults)
    else:
        assert follow_path['primary_controller'] == PLUGIN
        ours = {
            k: v for k, v in follow_path.items() if k not in SHIM_KEYS | {'primary_controller'}
        }
    unknown = set(ours) - set(defaults) - {'plugin'}
    assert not unknown, f'rotation_shim_{layout}.yaml sets unknown parameters {sorted(unknown)}'
    assert ours['use_rotate_to_field'] is False
    assert ours['obstacle_mode'] == 'none'
    assert ours['rotate_to_heading_angular_vel'] <= ours['max_angular_vel']
