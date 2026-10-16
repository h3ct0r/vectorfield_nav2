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
Ackermann specifics, and that the overlays only name real controller parameters. The controller
validation of each overlay (validateFromNode) runs in vectorfield_system_tests.
"""

from pathlib import Path

import pytest
import yaml

from vectorfield_bringup import demo, params

PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parent
CONTROLLER = PACKAGE / 'config' / 'controller'
SCENARIO = PACKAGE / 'config' / 'scenario'
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
    defaults = _defaults()
    merged = _merged(model)
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
    'path', sorted(CONTROLLER.glob('*.yaml')) + sorted(SCENARIO.glob('*.yaml')), ids=str
)
def test_overlays_never_set_stamped_cmd_vel(path):
    """enable_stamped_cmd_vel stays at each node's distro default (plan section 8)."""
    assert 'enable_stamped_cmd_vel' not in path.read_text(encoding='utf-8')


@pytest.mark.parametrize('model', demo.MOTION_MODELS)
def test_merged_file_is_written(model, tmp_path):
    """The merged file keeps every stock server and loads back unchanged."""
    merged = _merged(model, ('kinematic_sim', 'demo_obstacles'))
    stock = params.load_yaml(params.stock_params_file())
    assert set(stock) <= set(merged)
    path = params.write_params(merged, str(tmp_path / 'merged.yaml'))
    assert params.load_yaml(path) == merged
