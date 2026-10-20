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

"""Unit tests of vectorfield_bringup.params and the pure helpers of vectorfield_bringup.demo.

They use small parameter files written to a temporary directory and a fake share-directory
lookup, so they need no ROS installation beyond the package itself.
"""

import os

import pytest
import yaml

from vectorfield_bringup import demo, params

BASE = {
    'bt_navigator': {
        'ros__parameters': {
            'global_frame': 'map',
            'plugin_lib_names': ['a_bt_node', 'b_bt_node'],
        }
    },
    'controller_server': {
        'ros__parameters': {
            'controller_frequency': 20.0,
            'general_goal_checker': {
                'plugin': 'nav2_controller::SimpleGoalChecker',
                'xy_goal_tolerance': 0.25,
                'yaw_goal_tolerance': 0.25,
            },
            'FollowPath': {
                'plugin': 'nav2_mppi_controller::MPPIController',
                'time_steps': 56,
                'critics': ['GoalCritic'],
            },
        }
    },
    'planner_server': {
        'ros__parameters': {
            'GridBased': {
                'plugin': 'nav2_navfn_planner::NavfnPlanner',
                'use_astar': False,
                'tolerance': 0.5,
            }
        }
    },
    'velocity_smoother': {
        'ros__parameters': {
            'max_velocity': [0.26, 0.0, 1.0],
            'scale_velocities': False,
        }
    },
}


def _write(directory, name, data):
    path = directory / name
    path.write_text(yaml.safe_dump(data), encoding='utf-8')
    return str(path)


@pytest.fixture
def base_file(tmp_path):
    """Return a small stand-in for nav2_params.yaml."""
    return _write(tmp_path, 'nav2_params.yaml', BASE)


@pytest.fixture
def share(tmp_path):
    """Return a share lookup whose nav2_bt_navigator holds navigate_w_replanning_time.xml."""
    trees = tmp_path / 'share' / 'nav2_bt_navigator' / 'behavior_trees'
    trees.mkdir(parents=True)
    (trees / 'navigate_w_replanning_time.xml').write_text('<root/>', encoding='utf-8')

    def share_directory(package):
        return str(tmp_path / 'share' / package)

    return share_directory


def _params(merged, node):
    return merged[node]['ros__parameters']


def test_deep_merge_merges_dicts_and_replaces_other_values():
    """Dicts merge key by key; scalars and lists replace; the inputs are left unchanged."""
    base = {'a': {'x': 1, 'y': [1, 2, 3]}, 'b': 2}
    overlay = {'a': {'y': [4], 'z': 3}, 'c': {'k': 1}}
    merged = params.deep_merge(base, overlay)
    assert merged == {'a': {'x': 1, 'y': [4], 'z': 3}, 'b': 2, 'c': {'k': 1}}
    assert base == {'a': {'x': 1, 'y': [1, 2, 3]}, 'b': 2}
    merged['c']['k'] = 2
    assert overlay['c']['k'] == 1


def test_follow_path_is_replaced_wholesale(tmp_path, base_file, share):
    """No key of the stock controller survives in FollowPath."""
    overlay = {
        'controller_server': {
            'ros__parameters': {'FollowPath': {'plugin': 'mine::Controller', 'k': 1.0}}
        }
    }
    merged = params.merge(
        [_write(tmp_path, 'o.yaml', overlay)],
        base=base_file,
        distro='jazzy',
        share_directory=share,
    )
    follow_path = _params(merged, 'controller_server')['FollowPath']
    assert follow_path == {'plugin': 'mine::Controller', 'k': 1.0}
    assert _params(merged, 'controller_server')['controller_frequency'] == 20.0


def test_follow_path_is_replaced_even_with_the_same_plugin(tmp_path, base_file, share):
    """A second overlay that restates FollowPath replaces the first one's block too."""
    first = {'controller_server': {'ros__parameters': {'FollowPath': {'plugin': 'p', 'a': 1}}}}
    second = {'controller_server': {'ros__parameters': {'FollowPath': {'plugin': 'p', 'b': 2}}}}
    merged = params.merge(
        [_write(tmp_path, 'first.yaml', first), _write(tmp_path, 'second.yaml', second)],
        base=base_file,
        distro='jazzy',
        share_directory=share,
    )
    assert _params(merged, 'controller_server')['FollowPath'] == {'plugin': 'p', 'b': 2}


def test_a_changed_plugin_replaces_its_block(tmp_path, base_file, share):
    """Changing a block's plugin drops the old plugin's keys; keeping it merges."""
    overlay = {
        'controller_server': {
            'ros__parameters': {
                'general_goal_checker': {
                    'plugin': 'nav2_controller::PositionGoalChecker',
                    'xy_goal_tolerance': 0.3,
                }
            }
        },
        'planner_server': {'ros__parameters': {'GridBased': {'tolerance': 0.25}}},
    }
    merged = params.merge(
        [_write(tmp_path, 'o.yaml', overlay)],
        base=base_file,
        distro='jazzy',
        share_directory=share,
    )
    checker = _params(merged, 'controller_server')['general_goal_checker']
    assert checker == {'plugin': 'nav2_controller::PositionGoalChecker', 'xy_goal_tolerance': 0.3}
    planner = _params(merged, 'planner_server')['GridBased']
    assert planner == {
        'plugin': 'nav2_navfn_planner::NavfnPlanner',
        'use_astar': False,
        'tolerance': 0.25,
    }


def test_later_overlays_win_and_lists_are_replaced(tmp_path, base_file, share):
    """Overlays apply in order; a list is replaced, never concatenated."""
    first = {'velocity_smoother': {'ros__parameters': {'max_velocity': [0.5, 0.0, 2.0]}}}
    second = {'velocity_smoother': {'ros__parameters': {'scale_velocities': True}}}
    third = {'velocity_smoother': {'ros__parameters': {'max_velocity': [0.6, 0.0, 2.5]}}}
    files = [
        _write(tmp_path, f'{index}.yaml', data)
        for index, data in enumerate((first, second, third))
    ]
    merged = params.merge(files, base=base_file, distro='jazzy', share_directory=share)
    smoother = _params(merged, 'velocity_smoother')
    assert smoother == {'max_velocity': [0.6, 0.0, 2.5], 'scale_velocities': True}
    assert _params(merged, 'bt_navigator')['plugin_lib_names'] == ['a_bt_node', 'b_bt_node']


def test_empty_overlay_is_a_no_op(tmp_path, base_file, share):
    """An empty overlay file changes nothing."""
    empty = tmp_path / 'empty.yaml'
    empty.write_text('# nothing here\n', encoding='utf-8')
    merged = params.merge([str(empty)], base=base_file, distro='jazzy', share_directory=share)
    assert merged == BASE


def test_non_mapping_overlay_is_rejected(tmp_path, base_file, share):
    """A parameter file must be a mapping."""
    bad = tmp_path / 'bad.yaml'
    bad.write_text('- 1\n- 2\n', encoding='utf-8')
    with pytest.raises(ValueError, match='mapping'):
        params.merge([str(bad)], base=base_file, distro='jazzy', share_directory=share)


def test_stock_placeholders_are_substituted(tmp_path, share):
    """Lyrical's costmap-filter placeholders get navigation_launch.py's defaults."""
    base = {
        'local_costmap': {
            'local_costmap': {
                'ros__parameters': {
                    'keepout_filter': {'enabled': 'KEEPOUT_ZONE_ENABLED'},
                    'speed_filter': {'enabled': 'SPEED_ZONE_ENABLED', 'topic': 'speed_limit'},
                }
            }
        }
    }
    merged = params.merge(
        [], base=_write(tmp_path, 'base.yaml', base), distro='lyrical', share_directory=share
    )
    costmap = merged['local_costmap']['local_costmap']['ros__parameters']
    assert costmap['keepout_filter'] == {'enabled': False}
    assert costmap['speed_filter'] == {'enabled': False, 'topic': 'speed_limit'}


def test_bare_bt_xml_names_resolve_to_nav2_bt_navigator(tmp_path, base_file, share):
    """A bare tree name becomes the absolute stock path; paths and substitutions stay."""
    overlay = {
        'bt_navigator': {
            'ros__parameters': {
                'default_nav_to_pose_bt_xml': 'navigate_w_replanning_time.xml',
                'default_nav_through_poses_bt_xml': '/opt/trees/mine.xml',
            }
        }
    }
    merged = params.merge(
        [_write(tmp_path, 'o.yaml', overlay)],
        base=base_file,
        distro='jazzy',
        share_directory=share,
    )
    navigator = _params(merged, 'bt_navigator')
    expected = os.path.join(
        share('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_time.xml'
    )
    assert navigator['default_nav_to_pose_bt_xml'] == expected
    assert navigator['default_nav_through_poses_bt_xml'] == '/opt/trees/mine.xml'
    assert navigator['global_frame'] == 'map'

    substituted = '$(find-pkg-share x)/tree.xml'
    assert params.resolve_bt_xml({'default_nav_to_pose_bt_xml': substituted}, share) == {
        'default_nav_to_pose_bt_xml': substituted
    }


def test_missing_bt_xml_is_an_error(tmp_path, base_file, share):
    """A bare name that is not a stock tree fails at merge time, not in bt_navigator."""
    overlay = {'bt_navigator': {'ros__parameters': {'default_nav_to_pose_bt_xml': 'nope.xml'}}}
    with pytest.raises(FileNotFoundError, match='nope.xml'):
        params.merge(
            [_write(tmp_path, 'o.yaml', overlay)],
            base=base_file,
            distro='jazzy',
            share_directory=share,
        )


@pytest.mark.parametrize('distro', ['humble', 'jazzy', 'lyrical'])
def test_humble_rename_table(tmp_path, base_file, share, distro):
    """Humble gets the package/Class form for Smac, NavFn and behaviors; no one else does."""
    overlay = {
        'planner_server': {
            'ros__parameters': {'GridBased': {'plugin': 'nav2_smac_planner::SmacPlannerHybrid'}}
        },
        'behavior_server': {
            'ros__parameters': {
                'spin': {'plugin': 'nav2_behaviors::Spin'},
                'names': ['nav2_navfn_planner::NavfnPlanner', 'nav2_controller::X'],
            }
        },
    }
    merged = params.merge(
        [_write(tmp_path, 'o.yaml', overlay)], base=base_file, distro=distro, share_directory=share
    )
    planner = _params(merged, 'planner_server')['GridBased']['plugin']
    behaviors = _params(merged, 'behavior_server')
    controller = _params(merged, 'controller_server')
    if distro == 'humble':
        assert planner == 'nav2_smac_planner/SmacPlannerHybrid'
        assert behaviors['spin']['plugin'] == 'nav2_behaviors/Spin'
        assert behaviors['names'] == ['nav2_navfn_planner/NavfnPlanner', 'nav2_controller::X']
    else:
        assert planner == 'nav2_smac_planner::SmacPlannerHybrid'
        assert behaviors['spin']['plugin'] == 'nav2_behaviors::Spin'
    # Type names of other packages are never renamed.
    assert controller['general_goal_checker']['plugin'] == 'nav2_controller::SimpleGoalChecker'
    assert controller['FollowPath']['plugin'] == 'nav2_mppi_controller::MPPIController'


def test_distro_comes_from_the_environment(monkeypatch, tmp_path, base_file, share):
    """Without a distro argument, ROS_DISTRO decides whether the Humble table applies."""
    grid = {'GridBased': {'plugin': 'nav2_smac_planner::S'}}
    overlay = {'planner_server': {'ros__parameters': grid}}
    files = [_write(tmp_path, 'o.yaml', overlay)]
    monkeypatch.setenv('ROS_DISTRO', 'humble')
    merged = params.merge(files, base=base_file, share_directory=share)
    assert _params(merged, 'planner_server')['GridBased']['plugin'] == 'nav2_smac_planner/S'
    monkeypatch.setenv('ROS_DISTRO', 'jazzy')
    merged = params.merge(files, base=base_file, share_directory=share)
    assert _params(merged, 'planner_server')['GridBased']['plugin'] == 'nav2_smac_planner::S'


def test_default_base_is_nav2_bringup_params(share):
    """The base file is nav2_bringup/params/nav2_params.yaml from the share lookup."""
    path = params.stock_params_file(share)
    assert path == os.path.join(share('nav2_bringup'), 'params', 'nav2_params.yaml')


def test_write_params_round_trips(tmp_path):
    """The written file loads back to the same parameters, keeping the key order."""
    path = params.write_params(BASE, str(tmp_path / 'out.yaml'))
    assert params.load_yaml(path) == BASE
    assert list(params.load_yaml(path)) == list(BASE)
    temporary = params.write_params(BASE)
    try:
        assert os.path.basename(temporary).startswith('vectorfield_nav2_params_')
        assert params.load_yaml(temporary) == BASE
    finally:
        os.remove(temporary)


def test_require_distro():
    """Supported distros pass through; others raise a message naming the alternatives."""
    assert demo.require_distro('x.launch.py', distro='humble') == 'humble'
    assert demo.require_distro('x.launch.py', demo.JAZZY_AND_LATER, 'lyrical') == 'lyrical'
    with pytest.raises(demo.UnsupportedDistroError, match='ROS_DISTRO=humble'):
        demo.require_distro('loopback_demo.launch.py', demo.JAZZY_AND_LATER, 'humble')
    with pytest.raises(demo.UnsupportedDistroError, match='<unset>') as unset:
        demo.require_distro('x.launch.py', distro='')
    assert 'kinematic_sim_demo' not in str(unset.value)


def test_with_slip_compensation_sets_follow_path():
    """The two compensation keys are set in FollowPath; nothing else changes."""
    follow = {'motion_model': 'skid_steer', 'skid_icr_offset': 0.0}
    merged = {'controller_server': {'ros__parameters': {'FollowPath': follow}}, 'other': {}}
    result = demo.with_slip_compensation(merged, 0.8, -0.02)
    assert _params(result, 'controller_server')['FollowPath'] == {
        'motion_model': 'skid_steer',
        'skid_yaw_rate_efficiency': 0.8,
        'skid_icr_offset': -0.02,
    }
    assert result['other'] == {}
    assert follow == {'motion_model': 'skid_steer', 'skid_icr_offset': 0.0}, 'not modified'


def test_split_list():
    """Launch list arguments accept commas and spaces."""
    assert demo.split_list('') == []
    assert demo.split_list('a, b c,,d') == ['a', 'b', 'c', 'd']


def test_overlay_lookup(tmp_path):
    """Names resolve in the share directory, paths are kept, and missing files fail early."""
    (tmp_path / 'config' / 'controller').mkdir(parents=True)
    (tmp_path / 'config' / 'scenario').mkdir(parents=True)
    controller = tmp_path / 'config' / 'controller' / 'differential.yaml'
    scenario = tmp_path / 'config' / 'scenario' / 'kinematic_sim.yaml'
    extra = tmp_path / 'extra.yaml'
    for path in (controller, scenario, extra):
        path.write_text('{}\n', encoding='utf-8')
    paths = demo.overlay_paths('differential', ['kinematic_sim', str(extra)], str(tmp_path))
    assert paths == [str(controller), str(scenario), str(extra)]
    with pytest.raises(ValueError, match='motion_model'):
        demo.overlay_paths('tricycle', [], str(tmp_path))
    with pytest.raises(FileNotFoundError, match='nope'):
        demo.overlay_paths('differential', ['nope'], str(tmp_path))


def test_refuse_unsupported():
    """A refused launch fails when it runs, with the reason; a supported distro gets None."""
    assert demo.refuse_unsupported('x.launch.py', distro='jazzy') is None
    refusal = demo.refuse_unsupported('loopback_demo.launch.py', demo.JAZZY_AND_LATER, 'humble')
    (action,) = refusal.entities
    with pytest.raises(demo.UnsupportedDistroError, match='kinematic_sim_demo'):
        action.execute(None)
