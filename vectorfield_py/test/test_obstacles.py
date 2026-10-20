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

"""Occupancy grid, distance transform, footprint checker and collision predictor."""

import numpy as np
import pytest
import vectorfield as vf


def _brute_force_squared(mask):
    """Squared distance in cells from each cell centre to the nearest site centre."""
    sites = np.argwhere(mask)  # rows of [iy, ix]
    iy, ix = np.indices(mask.shape)
    cells = np.stack([iy.ravel(), ix.ravel()], axis=1)
    d2 = ((cells[:, None, :] - sites[None, :, :]) ** 2).sum(axis=2).min(axis=1)
    return d2.reshape(mask.shape).astype(float)


@pytest.mark.parametrize('seed', [0, 1, 2])
def test_edt_matches_brute_force(seed):
    """EDT_all is exact in cell units and every feature is a site at that distance."""
    rng = np.random.default_rng(seed)
    mask = rng.random((15, 21)) < 0.08
    mask[7, 3] = True
    grid = vf.OccupancyGrid(mask, 0.05, origin=(1.0, -2.0))
    edt = vf.EuclideanDistanceTransform(grid)
    expected = _brute_force_squared(mask)
    np.testing.assert_array_equal(edt.squared_distance_grid(), expected)
    np.testing.assert_allclose(edt.distance_grid(), 0.05 * np.sqrt(expected), rtol=1e-12)
    features = edt.feature_grid()
    assert features.shape == (15, 21, 2)
    fx, fy = features[..., 0], features[..., 1]
    assert mask[fy, fx].all()
    iy, ix = np.indices(mask.shape)
    np.testing.assert_array_equal((fx - ix) ** 2 + (fy - iy) ** 2, expected)
    assert edt.distance(3, 7) == pytest.approx(0.05 * np.sqrt(expected[7, 3]))


def test_edt_update_rebuilds_only_on_change_and_filters():
    """update() rebuilds only for a new version; build() with a filter uses only those sites."""
    mask = np.zeros((10, 10), dtype=bool)
    mask[2, 2] = mask[8, 8] = True
    grid = vf.OccupancyGrid(mask, 0.1)
    edt = vf.EuclideanDistanceTransform()
    assert edt.update(grid) and not edt.update(grid)
    assert edt.built_version == grid.version
    only = np.zeros_like(mask)
    only[8, 8] = True
    edt.build(grid, only)
    assert edt.feature(2, 2) == (8, 8) and edt.built_version == 0
    empty = vf.EuclideanDistanceTransform(vf.OccupancyGrid(np.zeros((4, 5)), 0.1))
    assert not empty.has_sites and np.isinf(empty.distance_grid()).all()


def test_grid_mask_round_trip_and_cells():
    """The mask comes back as (height, width) uint8; cells follow the Costmap2D layout."""
    mask = np.zeros((4, 6), dtype=bool)
    mask[1, 4] = True
    grid = vf.OccupancyGrid(mask, 0.5, origin=[-1.0, 2.0])
    assert grid.shape == (4, 6) and (grid.width, grid.height) == (6, 4)
    out = grid.mask
    assert out.dtype == np.uint8 and out.shape == (4, 6)
    np.testing.assert_array_equal(out, mask.astype(np.uint8))
    assert grid.is_site(4, 1) and not grid.is_site(1, 4)
    np.testing.assert_allclose(grid.cell_center(4, 1), [-1.0 + 4.5 * 0.5, 2.0 + 1.5 * 0.5])
    assert grid.cell_at([1.2, 2.7]) == (4, 1) and grid.cell_at([-5.0, 0.0]) is None
    np.testing.assert_allclose(grid.origin, [-1.0, 2.0])
    with pytest.raises(ValueError):
        vf.OccupancyGrid(np.zeros(5), 0.1)
    with pytest.raises(ValueError):
        vf.OccupancyGrid(mask, -0.1)


def test_query_points_and_obstacle_map():
    """Continuous queries point away from the site; ObstacleMap keeps EDT_all current."""
    mask = np.zeros((20, 20), dtype=np.uint8)
    mask[10, 10] = 1
    grid = vf.OccupancyGrid(mask, 0.1)
    obstacles = vf.ObstacleMap(grid)
    centre = grid.cell_center(10, 10)
    q = obstacles.edt.query(centre + [0.3, 0.0])
    assert q.valid and q.D_o == pytest.approx(0.3, abs=1e-9)
    np.testing.assert_allclose(q.n_o, [1.0, 0.0], atol=1e-9)
    batch = obstacles.edt.query_points(np.array([centre + [0.0, 0.4], [50.0, 50.0]]))
    np.testing.assert_allclose(batch['D_o'], [0.4, np.inf], atol=1e-9)
    np.testing.assert_array_equal(batch['valid'], [True, False])
    mask[2, 2] = 1
    grid.set_from_mask(mask, 0.1)
    assert obstacles.update() and obstacles.edt.built_version == grid.version


def test_footprint_checker():
    """A disc collides next to a site and is free far from it; a stale EDT is refused."""
    mask = np.zeros((40, 40), dtype=np.uint8)
    mask[20, 20] = 1
    grid = vf.OccupancyGrid(mask, 0.05)
    edt = vf.EuclideanDistanceTransform(grid)
    checker = vf.FootprintChecker(vf.Footprint.circle(0.2))
    site = grid.cell_center(20, 20)
    assert checker.collides((site[0] + 0.1, site[1], 0.0), grid, edt)
    assert not checker.collides((site[0] + 0.6, site[1], 0.0), grid, edt)
    square = vf.Footprint.polygon([[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]])
    assert square.w_lat == pytest.approx(0.1) and square.r_circ == pytest.approx(0.1 * 2**0.5)
    assert square.cover_radius((0.0, 0.0)) == pytest.approx(square.r_circ)
    assert square.cover_radius((0.1, 0.0)) == pytest.approx(0.05**0.5)
    assert vf.Footprint.circle(0.2).cover_radius((0.2, 0.0)) == pytest.approx(0.4)
    with pytest.raises(vf.ConfigError):
        vf.Footprint.polygon([[0.0, 0.0], [1.0, 0.0]])
    mask[5, 5] = 1
    grid.set_from_mask(mask, 0.05)
    with pytest.raises(ValueError):
        checker.collides((1.0, 1.0, 0.0), grid, edt)


def test_collision_predictor_rollout_with_python_policy():
    """A straight rollout toward a wall stops before it with a finite free arc length."""
    mask = np.zeros((40, 80), dtype=np.uint8)
    mask[:, 60] = 1
    grid = vf.OccupancyGrid(mask, 0.05)
    obstacles = vf.ObstacleMap(grid)
    predictor = vf.CollisionPredictor(
        vf.Kinematics(), vf.FootprintChecker(vf.Footprint.circle(0.2))
    )

    def policy(pose, t):
        return vf.RolloutStep(cmd=vf.Twist2(0.5, 0.0, 0.0))

    result, poses = predictor.rollout(
        vf.Pose2(1.0, 1.0, 0.0), obstacles, vf.RolloutOptions(horizon=10.0), policy
    )
    wall = 60 * 0.05
    assert result.collided and poses.shape[1] == 3
    assert result.s_free == pytest.approx(wall - 0.2 - 1.0, abs=0.06)
    assert vf.braking_cap(np.inf, 0.1, 1.0) == np.inf
