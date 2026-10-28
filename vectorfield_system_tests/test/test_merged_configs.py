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

"""Checks of the installed overlays merged onto the running distro's nav2_params.yaml.

For every motion model × scenario merge (shipped-config validation):

- the velocity smoother's limits are at least the controller's on every commanded axis (x and θ,
  and y for the omnidirectional model), so the smoother never cuts a command the controller
  planned within its own limits;
- no scenario overlay sets the smoother's velocity or acceleration keys;
- the curve_follower merge lists static_layer in the local costmap.

The controller validation of each overlay runs in test_shipped_configs.
"""

from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from vectorfield_bringup import params

BRINGUP = Path(get_package_share_directory('vectorfield_bringup'))
CONTROLLER = sorted((BRINGUP / 'config' / 'controller').glob('*.yaml'))
SCENARIO = sorted((BRINGUP / 'config' / 'scenario').glob('*.yaml'))
COMMON_PARAMETERS = (
    Path(get_package_share_directory('vectorfield_ros'))
    / 'params'
    / 'vectorfield_common_params.yaml'
)
SMOOTHER_KEYS = ('max_velocity', 'min_velocity', 'max_accel', 'max_decel')


def _defaults():
    """Return the controller's parameter defaults from the installed parameter definitions."""
    defaults = {}
    for group in yaml.safe_load(COMMON_PARAMETERS.read_text(encoding='utf-8')).values():
        defaults.update(
            {
                name: spec['default_value']
                for name, spec in group.items()
                if isinstance(spec, dict) and 'default_value' in spec
            }
        )
    return defaults


def _node(merged, name):
    return merged[name]['ros__parameters']


def test_overlays_are_installed():
    """Every motion model and the four scenarios are installed."""
    assert [path.stem for path in CONTROLLER] == [
        'ackermann',
        'differential',
        'omnidirectional',
        'skid_steer',
    ]
    assert {'curve_follower', 'demo_obstacles', 'kinematic_sim', 'loopback'} <= {
        path.stem for path in SCENARIO
    }


@pytest.mark.parametrize('scenario', SCENARIO, ids=lambda path: path.stem)
@pytest.mark.parametrize('controller', CONTROLLER, ids=lambda path: path.stem)
def test_smoother_limits_cover_the_controller(controller, scenario):
    """The smoother passes every command within the controller's limits, on every axis."""
    merged = params.merge([str(controller), str(scenario)])
    limits = {**_defaults(), **_node(merged, 'controller_server')['FollowPath']}
    smoother = _node(merged, 'velocity_smoother')
    max_v, min_v = smoother['max_velocity'], smoother['min_velocity']
    max_a, max_d = smoother['max_accel'], smoother['max_decel']
    reverses = limits['allow_reversing'] or limits['drive_backward']
    axes = [(0, 'max_linear_vel', 'max_linear_accel', 'max_linear_decel', reverses)]
    if limits['motion_model'] == 'omnidirectional':
        axes.append((1, 'max_lateral_vel', 'max_linear_accel', 'max_linear_decel', True))
    axes.append((2, 'max_angular_vel', 'max_angular_accel', 'max_angular_accel', True))
    for axis, velocity, accel, decel, both_ways in axes:
        assert max_v[axis] >= limits[velocity], (axis, velocity)
        if both_ways:
            assert -min_v[axis] >= limits[velocity], (axis, velocity)
        assert max_a[axis] >= limits[accel], (axis, accel)
        assert -max_d[axis] >= limits[decel], (axis, decel)


@pytest.mark.parametrize('scenario', SCENARIO, ids=lambda path: path.stem)
def test_scenarios_leave_smoother_limits_alone(scenario):
    """Only the motion-model overlays set the smoother's velocity and acceleration keys."""
    overlay = params.load_yaml(str(scenario))
    smoother = overlay.get('velocity_smoother', {}).get('ros__parameters', {})
    assert not set(smoother) & set(SMOOTHER_KEYS), sorted(smoother)


@pytest.mark.parametrize('controller', CONTROLLER, ids=lambda path: path.stem)
def test_curve_follower_merge_lists_static_layer(controller):
    """The curve follower's local costmap sees the map (static_layer)."""
    scenario = BRINGUP / 'config' / 'scenario' / 'curve_follower.yaml'
    merged = params.merge([str(controller), str(scenario)])
    local = merged['local_costmap']['local_costmap']['ros__parameters']
    assert 'static_layer' in local['plugins'], local['plugins']
