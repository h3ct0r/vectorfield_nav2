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

"""Unit tests of vectorfield_mujoco.scene: Nav2 map to walls, obstacles and world assembly."""

import math
import pathlib

import numpy as np
import pytest

from vectorfield_mujoco import scene

MAPS = pathlib.Path(__file__).resolve().parents[2] / 'vectorfield_bringup' / 'maps'


def _write_map(
    tmp_path,
    pixels,
    *,
    negate=0,
    origin=(0.0, 0.0, 0.0),
    resolution=0.1,
    fmt='P5',
    occupied_thresh=0.65,
):
    pixels = np.asarray(pixels, dtype=np.uint8)
    h, w = pixels.shape
    image = tmp_path / 'map.pgm'
    if fmt == 'P5':
        image.write_bytes(b'P5\n# a comment\n%d %d\n255\n' % (w, h) + pixels.tobytes())
    else:
        rows = '\n'.join(' '.join(str(v) for v in row) for row in pixels)
        image.write_text(f'P2\n{w} {h}\n255\n{rows}\n')
    yaml_path = tmp_path / 'map.yaml'
    yaml_path.write_text(
        f'image: map.pgm\nresolution: {resolution}\norigin: {list(origin)}\nnegate: {negate}\n'
        f'occupied_thresh: {occupied_thresh}\nfree_thresh: 0.196\nmode: trinary\n'
    )
    return yaml_path


def _cover(boxes, shape):
    grid = np.zeros(shape, dtype=int)
    for r, c, nr, nc in boxes:
        grid[r : r + nr, c : c + nc] += 1
    return grid


@pytest.mark.parametrize('fmt', ['P5', 'P2'])
def test_read_pgm(tmp_path, fmt):
    """Binary and ASCII PGM images read back with comments skipped."""
    pixels = np.array([[0, 255, 100], [254, 0, 205]])
    path = _write_map(tmp_path, pixels, fmt=fmt).parent / 'map.pgm'
    assert scene.read_pgm(path) == pytest.approx(pixels)


def test_load_map_thresholds_and_orientation(tmp_path):
    """Occupancy follows map_server: thresholds, negate, bottom row first."""
    # Top row first in the image: black (occupied), mid grey (unknown), white (free).
    pixels = [[0, 100, 255], [255, 255, 0]]
    occ = scene.load_map(_write_map(tmp_path, pixels)).occupied
    # Row 0 of the grid is the bottom image row (y = origin).
    assert occ.tolist() == [[False, False, True], [True, False, False]]
    # negate: white is occupied.
    occ = scene.load_map(_write_map(tmp_path, pixels, negate=1)).occupied
    assert occ.tolist() == [[True, True, False], [False, False, True]]
    # A lower threshold turns the grey cell (p = 0.61) into a wall.
    occ = scene.load_map(_write_map(tmp_path, pixels, occupied_thresh=0.5)).occupied
    assert occ[1].tolist() == [True, True, False]


def test_merge_boxes_exact_cover():
    """The merged boxes cover every occupied cell exactly once."""
    rng = np.random.default_rng(3)
    for _ in range(20):
        occ = rng.random((17, 23)) < 0.4
        boxes = scene.merge_boxes(occ)
        assert (_cover(boxes, occ.shape) == occ.astype(int)).all()
    l_shape = np.zeros((6, 6), dtype=bool)
    l_shape[0, :] = True
    l_shape[:, 0] = True
    assert len(scene.merge_boxes(l_shape)) == 2
    assert len(scene.merge_boxes(np.ones((5, 7), dtype=bool))) == 1


def test_map_walls_positions(tmp_path):
    """Wall boxes sit on their cells, including under a rotated origin."""
    # One occupied 2x3 block at image rows 0-1 (top), columns 1-3, in a 4x5 map.
    pixels = np.full((4, 5), 255)
    pixels[0:2, 1:4] = 0
    walls = scene.map_walls(
        scene.load_map(_write_map(tmp_path, pixels, origin=(-1.0, 2.0, 0.0))), height=0.5
    )
    assert len(walls) == 1
    assert walls[0]['pos'] == pytest.approx([-1.0 + 0.25, 2.0 + 0.3, 0.25])
    assert walls[0]['size'] == pytest.approx([0.15, 0.1, 0.25])
    # A rotated origin rotates the walls about it.
    walls = scene.map_walls(
        scene.load_map(_write_map(tmp_path, pixels, origin=(0.0, 0.0, math.pi / 2)))
    )
    assert walls[0]['pos'][:2] == pytest.approx([-0.3, 0.25])
    assert walls[0]['yaw'] == pytest.approx(math.pi / 2)


def test_open_10x10_walls():
    """open_10x10 becomes four walls covering exactly its 0.1 m border."""
    occupancy = scene.load_map(MAPS / 'open_10x10.yaml')
    walls = scene.map_walls(occupancy)
    assert len(walls) == 4
    lo = np.min([np.subtract(w['pos'], w['size']) for w in walls], axis=0)
    hi = np.max([np.add(w['pos'], w['size']) for w in walls], axis=0)
    assert lo == pytest.approx([-5.0, -5.0, 0.0])
    assert hi == pytest.approx([5.0, 5.0, scene.WALL_HEIGHT])
    # The walls cover exactly the occupied area of the map (0.1 m border).
    area = sum(4 * w['size'][0] * w['size'][1] for w in walls)
    assert area == pytest.approx(occupancy.occupied.sum() * occupancy.resolution**2)
    assert area == pytest.approx(10.0 * 10.0 - 9.8 * 9.8)


@pytest.mark.parametrize('name', ['corridor_door', 'corridor_corner'])
def test_bringup_maps_merge(name):
    """The corridor maps merge into a few dozen boxes at most."""
    occupancy = scene.load_map(MAPS / f'{name}.yaml')
    boxes = scene.merge_boxes(occupancy.occupied)
    assert (_cover(boxes, occupancy.occupied.shape) == occupancy.occupied).all()
    assert len(boxes) < 50


def test_obstacle_geom_defaults():
    """Obstacles rest on the floor unless given a height; bad ones are refused."""
    kind, size, pos, yaw = scene.obstacle_geom(
        {'type': 'cylinder', 'pos': [1, 2], 'size': [0.2, 0.4]}
    )
    assert (kind, size, pos, yaw) == ('cylinder', [0.2, 0.4, 0.0], [1.0, 2.0, 0.4], 0.0)
    assert scene.obstacle_geom({'type': 'sphere', 'pos': [0, 0], 'size': [0.3]})[2][2] == 0.3
    with pytest.raises(ValueError):
        scene.obstacle_geom({'type': 'cone', 'pos': [0, 0], 'size': [1]})
    with pytest.raises(ValueError):
        scene.obstacle_geom({'type': 'box', 'pos': [0, 0], 'size': [1, 1]})


def test_build_world_spec():
    """The world compiles with its options, groups and obstacle poses."""
    mujoco = pytest.importorskip('mujoco')
    obstacles = [
        {'type': 'box', 'pos': [2.0, 0.0], 'size': [0.3, 0.3, 0.3], 'yaw': 0.4},
        {'type': 'sphere', 'pos': [0.0, 2.0, 1.0], 'size': [0.25], 'rgba': [1, 0, 0, 1]},
    ]
    spec = scene.build(map_yaml=MAPS / 'open_10x10.yaml', obstacles=obstacles)
    model = spec.compile()
    assert model.opt.timestep == scene.TIMESTEP
    assert model.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    assert model.vis.global_.offwidth == 1920 and model.vis.global_.offheight == 1080
    groups = {model.geom(i).name: int(model.geom_group[i]) for i in range(model.ngeom)}
    assert groups.pop('floor') == 0
    assert set(groups.values()) == {scene.WORLD_GROUP}
    assert len(groups) == 4 + 2
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    sphere = model.geom('obstacle_1').id
    assert data.geom_xpos[sphere] == pytest.approx([0.0, 2.0, 1.0])
    opt = scene.visual_option()
    assert [int(g) for g in opt.geomgroup] == [1, 1, 1, 0, 1, 0]


def test_small_worlds_get_the_least_extent():
    """A small world gets MIN_EXTENT (the shadow map's size); a larger one keeps its own."""
    pytest.importorskip('mujoco')
    small = scene.make_world('scout_mini')
    assert small.model.stat.extent == pytest.approx(scene.MIN_EXTENT)
    far = {'type': 'box', 'pos': [40.0, 0.0], 'size': [0.2, 0.2, 0.2]}
    large = scene.make_world('scout_mini', obstacles=[far])
    assert large.model.stat.extent > scene.MIN_EXTENT
