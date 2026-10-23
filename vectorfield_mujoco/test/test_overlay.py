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

"""Unit tests of vectorfield_mujoco.overlay: marker dicts to mjvGeoms, counts and cap."""

import math

import numpy as np
import pytest

from vectorfield_mujoco import overlay

STRIP = {
    'type': 'LINE_STRIP',
    'points': [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)],
    'scale': (0.02, 0, 0),
    'rgba': (0, 1, 0, 1),
}


def test_primitive_counts():
    """Each marker type yields its number of primitives; TEXT yields none."""
    assert len(overlay.primitives(STRIP)) == 3
    line_list = dict(STRIP, type='LINE_LIST')
    assert len(overlay.primitives(line_list)) == 2
    assert len(overlay.primitives({'type': 'ARROW', 'points': [(0, 0, 0), (1, 0, 0)]})) == 1
    assert len(overlay.primitives({'type': 'SPHERE'})) == 1
    assert len(overlay.primitives({'type': 'CUBE'})) == 1
    assert len(overlay.primitives({'type': 'SPHERE_LIST', 'points': [(0, 0, 0)] * 5})) == 5
    assert overlay.primitives({'type': 'TEXT', 'text': 'hello'}) == []
    assert overlay.primitives({'type': 9}) == []  # TEXT_VIEW_FACING by id
    assert len(overlay.primitives(dict(STRIP, type=4))) == 3  # LINE_STRIP by id


def test_pose_and_scale():
    """Marker pose, scale and per-point colours are applied as RViz does."""
    yaw = math.pi / 2
    marker = {
        'type': 'LINE_STRIP',
        'points': [(1, 0, 0), (2, 0, 0)],
        'pose': ((10, 0, 0), (0, 0, math.sin(yaw / 2), math.cos(yaw / 2))),
        'scale': (0.04, 0, 0),
        'colors': [(1, 0, 0, 1), (0, 0, 1, 1)],
    }
    ((_, kind, width, start, end, rgba),) = overlay.primitives(marker)
    assert kind == 'capsule' and width == pytest.approx(0.02)
    assert start == pytest.approx([10, 1, 0]) and end == pytest.approx([10, 2, 0])
    assert rgba == (1, 0, 0, 1)
    arrow = {'type': 'ARROW', 'pose': ((0, 0, 1), (0, 0, 0, 1)), 'scale': (0.5, 0.1, 0.1)}
    ((_, kind, width, start, end, _),) = overlay.primitives(arrow)
    assert kind == 'arrow' and end == pytest.approx([0.5, 0, 1]) and width == pytest.approx(0.05)
    ((_, kind, half, pos, _, _),) = overlay.primitives(
        {'type': 'CUBE', 'pose': ((1, 2, 3), (0, 0, 0, 1)), 'scale': (0.2, 0.4, 0.6)}
    )
    assert (
        kind == 'box'
        and half == pytest.approx([0.1, 0.2, 0.3])
        and pos == pytest.approx([1, 2, 3])
    )


def test_long_strips_are_thinned():
    """Long strips are thinned to MAX_STRIP_SEGMENTS, keeping both ends."""
    points = [(0.01 * i, 0, 0) for i in range(5001)]
    prims = overlay.primitives({'type': 'LINE_STRIP', 'points': points})
    assert len(prims) == overlay.MAX_STRIP_SEGMENTS
    assert prims[0][3] == pytest.approx([0, 0, 0]) and prims[-1][4] == pytest.approx([50, 0, 0])


def test_add_markers_to_a_scene():
    """Geoms land in the scene with the right type, size and position, within the cap."""
    mujoco = pytest.importorskip('mujoco')
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody/></mujoco>')
    scn = mujoco.MjvScene(model, maxgeom=3000)
    added = overlay.add_markers(scn, [STRIP, {'type': 'SPHERE', 'scale': (0.2, 0.2, 0.2)}])
    assert added == 4 and scn.ngeom == 4
    capsule = scn.geoms[0]
    assert capsule.type == mujoco.mjtGeom.mjGEOM_CAPSULE
    assert capsule.pos == pytest.approx([0.5, 0, 0])
    assert capsule.size[0] == pytest.approx(0.01)
    assert scn.geoms[3].type == mujoco.mjtGeom.mjGEOM_ELLIPSOID
    # The cap counts from the geoms already in the scene; the scene's capacity bounds it too.
    many = {'type': 'SPHERE_LIST', 'points': np.zeros((2000, 3)), 'scale': (0.1, 0.1, 0.1)}
    assert overlay.add_markers(scn, [many], cap=1500) == 1500 and scn.ngeom == 1504
    assert overlay.add_markers(scn, [many]) == 3000 - 1504
    # Zero-length segments are skipped.
    scn.ngeom = 0
    assert overlay.add_markers(scn, [{'type': 'LINE_LIST', 'points': [(1, 1, 1)] * 4}]) == 0
