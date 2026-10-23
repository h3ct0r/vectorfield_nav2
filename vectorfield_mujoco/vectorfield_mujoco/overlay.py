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

"""Draw RViz-style markers into a MuJoCo scene (the viewer's ``user_scn`` or a renderer's).

The controllers publish ``visualization_msgs/MarkerArray`` (path, field arrows, prediction).
This module has no ROS dependency: the simulator node turns each marker into a plain dict

``{'type': 'LINE_STRIP', 'points': [(x, y, z), ...], 'pose': ((x, y, z), (qx, qy, qz, qw)),
'scale': (x, y, z), 'rgba': (r, g, b, a), 'colors': [(r, g, b, a), ...]}``

(``pose`` and ``colors`` optional; the quaternion in ROS order) in the world frame, and
:func:`add_markers` appends the matching ``mjvGeom`` s. Lines become capsules of the marker's
width in metres, so they read the same at any resolution. Supported types: LINE_STRIP,
LINE_LIST, ARROW (pose or two points), SPHERE, CUBE, SPHERE_LIST, POINTS; others (TEXT, mesh
markers) are skipped. At most ``cap`` geoms are added; long strips are thinned first.
"""

import math

import numpy as np

#: Default geom budget per frame (``mjvScene`` holds a few thousand geoms in total).
DEFAULT_CAP = 1500

#: Line strips longer than this many segments are thinned to it.
MAX_STRIP_SEGMENTS = 400

#: The supported marker types (visualization_msgs/Marker names and ids).
TYPES = {
    'ARROW': 0,
    'CUBE': 1,
    'SPHERE': 2,
    'LINE_STRIP': 4,
    'LINE_LIST': 5,
    'SPHERE_LIST': 7,
    'POINTS': 8,
}
_IDS = {v: k for k, v in TYPES.items()}


def _quat_xyzw_mat(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def marker_type(marker):
    """Return the marker's type name (it may be given by name or by Marker id)."""
    kind = marker.get('type', 'LINE_STRIP')
    return _IDS.get(kind, 'UNSUPPORTED') if isinstance(kind, int) else str(kind).upper()


def _pose(marker):
    pos, quat = marker.get('pose', ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
    return np.asarray(pos, dtype=float), _quat_xyzw_mat(quat)


def primitives(marker):
    """Return the marker as a list of primitives, all in the world frame.

    Each primitive is ``('connector', kind, width, from, to, rgba)`` with kind ``'capsule'`` or
    ``'arrow'``, or ``('solid', kind, half_size, pos, mat, rgba)`` with kind ``'ellipsoid'``,
    ``'box'`` or ``'sphere'``.
    """
    kind = marker_type(marker)
    if kind not in TYPES:
        return []
    pos, rot = _pose(marker)
    scale = np.asarray(marker.get('scale', (0.05, 0.05, 0.05)), dtype=float)
    rgba = tuple(float(c) for c in marker.get('rgba', (1.0, 1.0, 1.0, 1.0)))
    colors = [tuple(float(c) for c in col) for col in marker.get('colors', ())]
    points = np.asarray(marker.get('points', ()), dtype=float).reshape(-1, 3)
    world = points @ rot.T + pos if len(points) else points

    def color(i):
        return colors[i] if i < len(colors) else rgba

    out = []
    if kind == 'LINE_STRIP':
        idx = np.arange(len(world))
        if len(world) - 1 > MAX_STRIP_SEGMENTS:
            idx = np.unique(np.linspace(0, len(world) - 1, MAX_STRIP_SEGMENTS + 1).round()).astype(
                int
            )
        for a, b in zip(idx[:-1], idx[1:]):
            out.append(('connector', 'capsule', 0.5 * scale[0], world[a], world[b], color(a)))
    elif kind == 'LINE_LIST':
        for i in range(0, len(world) - 1, 2):
            out.append(('connector', 'capsule', 0.5 * scale[0], world[i], world[i + 1], color(i)))
    elif kind == 'ARROW':
        if len(world) >= 2:
            start, end, width = world[0], world[1], 0.5 * scale[0]
        else:
            start, end, width = pos, pos + rot[:, 0] * scale[0], 0.5 * max(scale[1], scale[2])
        out.append(('connector', 'arrow', width, start, end, rgba))
    elif kind in ('SPHERE', 'CUBE'):
        solid = 'ellipsoid' if kind == 'SPHERE' else 'box'
        out.append(('solid', solid, 0.5 * scale, pos, rot, rgba))
    else:  # SPHERE_LIST, POINTS
        for i, p in enumerate(world):
            out.append(('solid', 'sphere', 0.5 * scale, p, np.eye(3), color(i)))
    return out


def add_markers(scene, markers, cap=DEFAULT_CAP):
    """Append markers to an ``mujoco.MjvScene``; return the number of geoms added.

    At most ``cap`` geoms are added (and never beyond ``scene.maxgeom``); markers later in the
    list are dropped first.
    """
    import mujoco

    kinds = {
        'capsule': mujoco.mjtGeom.mjGEOM_CAPSULE,
        'arrow': mujoco.mjtGeom.mjGEOM_ARROW,
        'ellipsoid': mujoco.mjtGeom.mjGEOM_ELLIPSOID,
        'box': mujoco.mjtGeom.mjGEOM_BOX,
        'sphere': mujoco.mjtGeom.mjGEOM_SPHERE,
    }
    limit = min(scene.maxgeom, scene.ngeom + int(cap))
    added = 0
    identity = np.eye(3).ravel()
    for marker in markers:
        for prim in primitives(marker):
            if scene.ngeom >= limit:
                return added
            geom = scene.geoms[scene.ngeom]
            rgba = np.asarray(prim[5], dtype=np.float32)
            if prim[0] == 'connector':
                _, kind, width, start, end, _ = prim
                if np.linalg.norm(np.asarray(end) - np.asarray(start)) < 1e-9:
                    continue
                mujoco.mjv_initGeom(geom, kinds[kind], np.zeros(3), np.zeros(3), identity, rgba)
                mujoco.mjv_connector(
                    geom,
                    kinds[kind],
                    max(width, 1e-4),
                    np.asarray(start, dtype=float),
                    np.asarray(end, dtype=float),
                )
            else:
                _, kind, half, pos, rot, _ = prim
                size = np.asarray(half, dtype=float)
                if kind == 'sphere':
                    size = np.full(3, size[0])
                mujoco.mjv_initGeom(
                    geom,
                    kinds[kind],
                    size,
                    np.asarray(pos, dtype=float),
                    np.asarray(rot, dtype=float).ravel(),
                    rgba,
                )
            geom.category = mujoco.mjtCatBit.mjCAT_DECOR
            scene.ngeom += 1
            added += 1
    return added
