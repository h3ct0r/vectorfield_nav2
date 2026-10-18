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

"""CoreConfig from ROS parameter YAML files and ``NAME=VALUE`` overrides.

The controller's parameters (plan section 5.4) carry the names of the CoreConfig fields, so
``FollowPath.desired_speed`` sets ``config.field.desired_speed``. Accepted layouts:

- a Nav2 parameter file: ``controller_server.ros__parameters.<plugin>.*``, plus the server's
  ``controller_frequency`` and the local costmap's ``footprint`` or ``robot_radius`` and size;
- any ``<node>.ros__parameters`` mapping, such as the curve follower's;
- flat keys, bare (``desired_speed``) or prefixed with the plugin name
  (``FollowPath.desired_speed``).
"""

from dataclasses import dataclass, field

import numpy as np
import vectorfield as vf

from ._cli import InputError, assignment, require

DEFAULT_PLUGIN = 'FollowPath'

#: ROS parameters without a CoreConfig field: visualisation and the legacy path handler.
ROS_ONLY = frozenset(
    {
        'plugin',
        'visualize',
        'field_grid_size',
        'field_grid_resolution',
        'field_grid_rate',
        'field_grid_style',
        'publish_obstacle_distance',
        'prune_distance',
        'max_robot_pose_search_dist',
        'transform_tolerance',
        'reject_unit_path',
    }
)


def _field_groups():
    config = vf.CoreConfig()
    return {
        name: group
        for group in vf.CoreConfig._fields
        for name in type(getattr(config, group))._fields
    }


#: Parameter name -> CoreConfig group, from the bindings' field lists.
PARAMETER_GROUPS = _field_groups()


@dataclass
class LoadedConfig:
    """A CoreConfig with what was read into it."""

    config: vf.CoreConfig = field(default_factory=vf.CoreConfig)
    applied: dict = field(default_factory=dict)
    ignored: list = field(default_factory=list)
    unknown: list = field(default_factory=list)
    costmap: object = None


def _footprint_points(value):
    if isinstance(value, str):
        yaml = require('yaml')
        value = yaml.safe_load(value)
    points = np.asarray(value, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise InputError(f'footprint must be a list of [x, y] points, got {value!r}')
    return points


def set_parameter(config, name, value):
    """Set one parameter by its ROS / CoreConfig name; raises InputError when it does not fit."""
    if name == 'footprint':
        config.node.footprint.polygon = _footprint_points(value)
        return
    if name == 'robot_radius':
        config.node.footprint.radius = float(value)
        return
    group = PARAMETER_GROUPS.get(name)
    if group is None:
        raise InputError(f'unknown parameter {name!r}')
    target = getattr(config, group)
    current = getattr(target, name)
    if isinstance(current, float) and isinstance(value, (int, float)) and value is not True:
        value = float(value)
    try:
        setattr(target, name, value)
    except (TypeError, ValueError) as error:
        raise InputError(f'{name}: cannot set {value!r} ({error})') from None


def get_parameter(config, name):
    """Return the value of one parameter by name."""
    group = PARAMETER_GROUPS.get(name)
    if group is None:
        raise InputError(f'unknown parameter {name!r}')
    return getattr(getattr(config, group), name)


def _select(data, plugin):
    """(parameters, server parameters, local costmap parameters) of a parsed YAML mapping."""
    if not isinstance(data, dict):
        raise InputError('expected a mapping of parameters')
    server = data.get('controller_server')
    if isinstance(server, dict):
        server = server.get('ros__parameters', server)
        params = server.get(plugin)
        if not isinstance(params, dict):
            raise InputError(f'controller_server.ros__parameters has no {plugin!r} block')
        costmap = data.get('local_costmap', {})
        costmap = costmap.get('local_costmap', costmap) if isinstance(costmap, dict) else {}
        costmap = costmap.get('ros__parameters', costmap) if isinstance(costmap, dict) else {}
        return params, server, costmap
    nodes = [v for v in data.values() if isinstance(v, dict) and 'ros__parameters' in v]
    if nodes:
        if len(nodes) > 1:
            raise InputError('several nodes with ros__parameters; pass a single node file')
        data = nodes[0]['ros__parameters']
    prefix = f'{plugin}.'
    flat = {}
    for key, value in data.items():
        key = str(key)
        if key == plugin and isinstance(value, dict):
            flat.update(value)
        else:
            flat[key[len(prefix) :] if key.startswith(prefix) else key] = value
    return flat, {}, {}


def config_from_mapping(data, plugin=DEFAULT_PLUGIN, base=None):
    """Build a LoadedConfig from a parsed parameter mapping (see the module docstring)."""
    loaded = LoadedConfig(config=(base or vf.CoreConfig()).copy())
    params, server, costmap = _select(data, plugin)
    if 'controller_frequency' in server:
        params = {'controller_frequency': server['controller_frequency'], **params}
    for key in ('footprint', 'robot_radius'):
        if key in costmap and key not in params:
            params[key] = costmap[key]
    if 'footprint' in params and str(params['footprint']).strip() in ('', '[]'):
        del params['footprint']  # Nav2's "use robot_radius" placeholder
    for name, value in params.items():
        if name in ROS_ONLY:
            loaded.ignored.append(name)
        elif name in PARAMETER_GROUPS or name in ('footprint', 'robot_radius'):
            set_parameter(loaded.config, name, value)
            loaded.applied[name] = value
        else:
            loaded.unknown.append(name)
    if costmap.get('rolling_window', False) and {'width', 'height'} <= set(costmap):
        loaded.costmap = vf.CostmapInfo(
            width_m=float(costmap['width']),
            height_m=float(costmap['height']),
            resolution=float(costmap.get('resolution', 0.05)),
        )
    return loaded


def load_config(path, plugin=DEFAULT_PLUGIN, base=None):
    """Load a LoadedConfig from a YAML parameter file."""
    yaml = require('yaml')
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    try:
        return config_from_mapping(data or {}, plugin=plugin, base=base)
    except InputError as error:
        raise InputError(f'{path}: {error}') from None


def add_config_arguments(argp, positional=False):
    """Add the configuration file (``--config`` or positional), ``--plugin`` and ``--set``."""
    group = argp.add_argument_group('configuration')
    help_text = 'controller parameters (Nav2 file, node file or flat keys); default: the defaults'
    if positional:
        group.add_argument('config', nargs='?', metavar='CONFIG.yaml', help=help_text)
    else:
        group.add_argument('--config', metavar='YAML', help=help_text)
    group.add_argument('--plugin', default=DEFAULT_PLUGIN, help='controller plugin name')
    group.add_argument(
        '--set',
        action='append',
        default=[],
        metavar='NAME=VALUE',
        help='set a parameter, e.g. --set convergence_gain=3 (repeatable)',
    )


def config_from_args(args, base=None):
    """Build a LoadedConfig from ``--config``, ``--plugin`` and ``--set``."""
    if args.config:
        loaded = load_config(args.config, plugin=args.plugin, base=base)
    else:
        loaded = LoadedConfig(config=(base or vf.CoreConfig()).copy())
    for entry in args.set:
        name, value = assignment(entry)
        set_parameter(loaded.config, name, value)
        loaded.applied[name] = value
    return loaded
