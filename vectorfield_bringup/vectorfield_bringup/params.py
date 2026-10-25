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

"""Merge the bringup overlays onto nav2_bringup's parameter file (plan section 8).

The demos never ship a full Nav2 parameter file. They start from the installed
``nav2_bringup/params/nav2_params.yaml`` of the running distro, so every stock server keeps the
configuration its own launch file expects, and deep-merge small overlays onto it:

- dictionaries merge key by key; any other overlay value, lists included, replaces the base value,
  except ``bt_navigator.plugin_lib_names``, whose overlay entries are appended to the base list
  (Humble's stock file lists every Nav2 behaviour-tree plugin there; later distros list only
  custom ones);
- ``controller_server.FollowPath`` is replaced wholesale when the overlay names its ``plugin``
  (the motion-model overlays), so no key of the stock controller (DWB or MPPI) leaks into ours;
  without a ``plugin`` (a scenario's controller keys, e.g. ``reversing.yaml``) it merges key by
  key; any other block is replaced wholesale when its ``plugin`` changes (for example the
  Ackermann overlay's goal checker and planner);
- the stock placeholders of :data:`STOCK_PLACEHOLDERS` get their launch-default values;
- a behaviour-tree file name without a directory (``default_nav_to_pose_bt_xml`` and
  ``default_nav_through_poses_bt_xml``) is resolved to ``nav2_bt_navigator/behavior_trees`` or,
  failing that, to this package's ``behavior_trees``;
- on Humble, ``nav2_smac_planner::X``, ``nav2_navfn_planner::X`` and ``nav2_behaviors::X`` are
  renamed to the ``/`` form, the only names Humble's pluginlib resolves.
"""

import copy
import os
import re
import tempfile

import yaml

#: Paths (suffixes of the key path) whose dictionary an overlay replaces instead of merging.
WHOLESALE_KEYS = (('controller_server', 'ros__parameters', 'FollowPath'),)

APPEND_KEYS = (('bt_navigator', 'ros__parameters', 'plugin_lib_names'),)

BT_XML_PACKAGES = ('nav2_bt_navigator', 'vectorfield_bringup')

#: bt_navigator keys that name a behaviour-tree file.
BT_XML_KEYS = ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml')

#: Packages whose plugins Humble registers as ``package/Class`` only.
HUMBLE_SLASH_PACKAGES = ('nav2_smac_planner', 'nav2_navfn_planner', 'nav2_behaviors')

#: Placeholder values in the stock file that nav2_bringup's launch files rewrite (Lyrical's
#: costmap filters), with the values of their launch defaults. merge() substitutes them, so the
#: merged file is also valid where it is not rewritten, such as a component container's
#: parameters.
STOCK_PLACEHOLDERS = {'KEEPOUT_ZONE_ENABLED': False, 'SPEED_ZONE_ENABLED': False}

_HUMBLE_RENAME = re.compile(r'^(' + '|'.join(HUMBLE_SLASH_PACKAGES) + r')::(\w+)$')


def ros_distro():
    """Return the lower-case ``ROS_DISTRO`` of the environment ('' if unset)."""
    return os.environ.get('ROS_DISTRO', '').strip().lower()


def _share_directory(package):
    # Imported here so that the pure helpers work without an ament environment.
    from ament_index_python.packages import get_package_share_directory

    return get_package_share_directory(package)


def load_yaml(path):
    """Load a parameter file as a dictionary; an empty file gives an empty dictionary.

    Raises:
        ValueError: if the document is not a mapping.

    """
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f'{path}: a parameter file must be a mapping, got {type(data).__name__}')
    return data


def _replaces_wholesale(path, base, overlay):
    if any(path[-len(key) :] == key for key in WHOLESALE_KEYS) and 'plugin' in overlay:
        return True
    return 'plugin' in base and 'plugin' in overlay and base['plugin'] != overlay['plugin']


def deep_merge(base, overlay, _path=()):
    """Return ``base`` with ``overlay`` merged in; neither argument is modified.

    Dictionaries merge recursively. A list at an :data:`APPEND_KEYS` path is appended to the base
    list, without repeating an entry. Every other overlay value replaces the base value, and so
    does a dictionary at a :data:`WHOLESALE_KEYS` path that names a ``plugin``, or one that names
    a different ``plugin``.
    """
    merged = copy.deepcopy(base)
    for key, value in overlay.items():
        path = _path + (key,)
        current = merged.get(key)
        if path in APPEND_KEYS and isinstance(current, list) and isinstance(value, list):
            merged[key] = current + [entry for entry in value if entry not in current]
        elif (
            isinstance(current, dict)
            and isinstance(value, dict)
            and not _replaces_wholesale(path, current, value)
        ):
            merged[key] = deep_merge(current, value, path)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _map_values(node, function, key=None):
    if isinstance(node, dict):
        return {k: _map_values(v, function, k) for k, v in node.items()}
    if isinstance(node, list):
        return [_map_values(v, function, key) for v in node]
    return function(key, node)


def rename_for_humble(params):
    """Return ``params`` with ``package::Class`` plugin names in the Humble ``package/Class`` form.

    Only the packages in :data:`HUMBLE_SLASH_PACKAGES` are renamed; other type names
    (``nav2_controller::*``, our own plugin) are registered as types on Humble too.
    """

    def rename(_key, value):
        if isinstance(value, str):
            return _HUMBLE_RENAME.sub(r'\1/\2', value)
        return value

    return _map_values(params, rename)


def substitute_values(params, substitutions):
    """Return ``params`` with every string value that is a key of ``substitutions`` replaced."""

    def substitute(_key, value):
        if isinstance(value, str) and value in substitutions:
            return substitutions[value]
        return value

    return _map_values(params, substitute)


def resolve_bt_xml(params, share_directory=_share_directory):
    """Return ``params`` with bare behaviour-tree file names made absolute.

    A :data:`BT_XML_KEYS` value without a directory part is looked up in the ``behavior_trees``
    directory of each :data:`BT_XML_PACKAGES` package in turn (the stock trees first); paths and
    substitutions are left alone.

    Raises:
        FileNotFoundError: if a bare name is neither a stock tree nor one of this package's.

    """

    def resolve(key, value):
        if key not in BT_XML_KEYS or not isinstance(value, str) or not value:
            return value
        if os.path.dirname(value) or '$(' in value:
            return value
        searched = []
        for package in BT_XML_PACKAGES:
            trees = os.path.join(share_directory(package), 'behavior_trees')
            path = os.path.join(trees, value)
            if os.path.isfile(path):
                return path
            searched.append(trees)
        raise FileNotFoundError(
            f'{key}: behaviour tree {value!r} not found in {" or ".join(searched)}'
        )

    return _map_values(params, resolve)


def stock_params_file(share_directory=_share_directory):
    """Return the installed ``nav2_bringup/params/nav2_params.yaml``."""
    return os.path.join(share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')


def merge(overlays, base=None, distro=None, share_directory=_share_directory):
    """Deep-merge parameter overlays onto nav2_bringup's parameter file.

    Args:
        overlays: Overlay file paths (or already loaded dictionaries), applied in order (later
            ones win); all of them go through the substitutions and renames below.
        base: Base parameter file; defaults to :func:`stock_params_file`.
        distro: ROS distro name; defaults to :func:`ros_distro`.
        share_directory: ``package -> share directory`` lookup (ament_index by default).

    Returns:
        The merged parameters as a dictionary.

    """
    if distro is None:
        distro = ros_distro()
    params = load_yaml(base if base is not None else stock_params_file(share_directory))
    for overlay in overlays:
        params = deep_merge(params, overlay if isinstance(overlay, dict) else load_yaml(overlay))
    params = substitute_values(params, STOCK_PLACEHOLDERS)
    params = resolve_bt_xml(params, share_directory)
    if distro == 'humble':
        params = rename_for_humble(params)
    return params


def write_params(params, path=None):
    """Write ``params`` as a YAML parameter file and return its path.

    Without ``path``, a new file is created in the temporary directory; it is left there for the
    nodes that read it after the launch description has been built.
    """
    if path is None:
        handle, path = tempfile.mkstemp(prefix='vectorfield_nav2_params_', suffix='.yaml')
        os.close(handle)
    with open(path, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(params, stream, default_flow_style=False, sort_keys=False)
    return path
