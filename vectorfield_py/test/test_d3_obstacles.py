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

"""3D obstacles and the CBF filter through vectorfield.d3 (math_spec 14.8), and vf-sim3d."""

import csv

import numpy as np
import pytest
import vectorfield as vf
from vectorfield import d3
from vectorfield_tools import curves3d, vf_sim3d


def cbf_config(**kwargs):
    """Build a TrackerConfig with the CBF filter, body radius 0.2 m and margin 0.05 m."""
    config = d3.TrackerConfig(**kwargs)
    config.obstacles.obstacle_mode = vf.ObstacleMode.CBF
    config.obstacles.body_radius = 0.2
    config.obstacles.cbf_margin = 0.05
    return config


def test_primitive_signed_distances():
    """Exact signed distances and normals of the four primitives; validation errors."""
    sphere = d3.Primitive.sphere(np.array([1.0, 2.0, 3.0]), 0.5)
    D, n = sphere.signed_distance(np.array([1.0, 2.0, 5.0]))
    assert D == pytest.approx(1.5) and np.allclose(n, [0, 0, 1])
    box = d3.Primitive.box(np.zeros(3), np.ones(3), yaw=0.0)
    D, n = box.signed_distance(np.array([[2.0, 2.0, 2.0], [0.2, 0.9, 0.0]]))
    assert np.allclose(D, [np.sqrt(3.0), -0.1]) and n.shape == (2, 3)
    pole = d3.Primitive.cylinder(np.array([1.0, 1.0, 0.0]), 0.5)
    assert pole.half_height == np.inf
    assert pole.signed_distance(np.array([1.0, 3.0, 100.0]))[0] == pytest.approx(1.5)
    ground = d3.Primitive.plane(np.zeros(3), np.array([0.0, 0.0, 2.0]))
    assert np.allclose(ground.normal, [0, 0, 1])
    assert ground.signed_distance(np.array([3.0, 1.0, -0.25]))[0] == pytest.approx(-0.25)
    assert ground.type == d3.PrimitiveType.PLANE and 'PLANE' in repr(ground)
    with pytest.raises(vf.ConfigError):
        d3.Primitive.sphere(np.zeros(3), 0.0).validate()
    with pytest.raises(vf.ConfigError):
        d3.ObstacleSet([d3.Primitive.box(np.zeros(3), np.array([1.0, 0.0, 1.0]))])


def test_voxel_edt_matches_brute_force():
    """The voxel EDT equals brute force on a random (nz, ny, nx) mask."""
    rng = np.random.default_rng(3)
    mask = (rng.random((5, 6, 7)) < 0.08).astype(np.uint8)  # (nz, ny, nx)
    mask[2, 3, 4] = 1
    grid = d3.VoxelGrid(mask, 0.1, np.array([-0.3, 0.2, 1.0]))
    assert grid.shape == (5, 6, 7) and np.array_equal(grid.mask, mask)
    edt = d3.DistanceTransform(grid)
    assert edt.has_sites and not edt.update(grid)
    sites = np.argwhere(mask)  # (iz, iy, ix)
    cells = np.argwhere(np.ones_like(mask))
    brute = ((cells[:, None, :] - sites[None, :, :]) ** 2).sum(axis=2).min(axis=1)
    assert np.allclose(edt.distances().ravel(), 0.1 * np.sqrt(brute))
    iz, iy, ix = 0, 0, 0
    fx, fy, fz = edt.feature(ix, iy, iz)
    assert mask[fz, fy, fx] == 1
    assert edt.squared_distance_cells(ix, iy, iz) == brute[0]
    D, _ = edt.query(np.array([10.0, 10.0, 10.0]))
    assert D == np.inf  # outside the grid
    with pytest.raises(ValueError):
        d3.VoxelGrid(mask[0], 0.1, np.zeros(3))
    with pytest.raises(IndexError):
        edt.squared_distance_cells(7, 0, 0)


def test_obstacle_set_sites_and_solver():
    """Sites are nearest first across primitives and voxels; solve_cbf slides along a floor."""
    obstacles = d3.ObstacleSet()
    obstacles.add(d3.Primitive.plane(np.zeros(3), np.array([0.0, 0.0, 1.0])))
    obstacles.add(d3.Primitive.sphere(np.array([2.0, 0.0, 1.0]), 0.5))
    mask = np.zeros((4, 4, 4), dtype=bool)
    mask[:, :, 0] = True
    # A voxel wall (the ix = 0 column, centres at x = -0.75); the grid spans x in [-0.875, 0.125).
    obstacles.set_voxels(d3.VoxelGrid(mask, 0.25, np.array([-0.875, -0.5, 0.5])))
    assert obstacles.has_voxels and len(obstacles.primitives) == 2
    sites = obstacles.sites(np.array([0.0, 0.0, 1.0]))
    assert [s['source'] for s in sites] == [-1, 0, 1]
    assert [s['D'] for s in sites] == pytest.approx([0.75, 1.0, 1.5])
    D, n = obstacles.nearest(np.array([[0.0, 0.0, 1.0], [2.0, 0.0, 1.6]]))
    assert D == pytest.approx([0.75, 0.1])
    F, active, relaxed, binds = d3.solve_cbf(
        np.array([0.3, 0.2, -0.1]), np.array([[0.0, 0.0, 1.0]]), np.array([0.0]), 2.0
    )
    assert active and not relaxed and binds == 1 and np.allclose(F, [0.3, 0.2, 0.0])


def test_tracker_holds_at_a_centred_sphere():
    """A sphere centred on the line: BLOCKED, at rest, 0 <= h < 2 cm."""
    line = d3.make_curve(d3.CurveSpec.polyline([[-1, 0, 1], [5, 0, 1]], closed=False))
    obstacles = d3.ObstacleSet([d3.Primitive.sphere(np.array([2.0, 0.0, 1.0]), 0.5)])
    tracker = d3.Tracker(cbf_config())
    sim = d3.sim.ClosedLoopSimulator(tracker, obstacles=obstacles)
    sim.reset(np.array([-1.0, 0.0, 1.0]))
    run = sim.run(line, 30.0)
    assert run['status'][-1] & int(vf.Status.BLOCKED)
    assert np.linalg.norm(run['v'][-1]) < 1e-6
    assert 0.0 <= run['h'][-1] < 0.02 and run['h'].min() >= 0.0
    assert run['cbf_active'].any() and not run['settled'][-1]
    out = sim.last_output
    assert out.cbf.on and out.cbf.hold and out.cbf.count == 1
    assert out.cbf.sites[0]['source'] == 0 and sim.state.blocked
    # One step through Tracker.step with the obstacles argument.
    out, state = tracker.step(line, np.array([1.2, 0.05, 1.0]), obstacles=obstacles)
    assert out.cbf.on and out.cbf.active and np.allclose(out.cbf.F_in, out.Phi)
    assert out.cbf.h == pytest.approx(np.hypot(0.8, 0.05) - 0.75)


def test_closed_loop_passes_a_sphere_off_the_helix():
    """A sphere beside the helix is passed with h >= -1e-3; the records carry h."""
    helix = d3.make_curve(d3.CurveSpec.builtin('helix'))
    obstacles = d3.ObstacleSet([d3.Primitive.sphere(np.array([1.35, 0.0, 0.5]), 0.3)])
    sim = d3.sim.ClosedLoopSimulator(d3.Tracker(cbf_config()), obstacles=obstacles)
    sim.reset(np.array([1.0, 0.0, 0.0]))
    run = sim.run(helix, 60.0)
    assert run['settled'][-1]
    D, _ = obstacles.nearest(run['position'])
    assert D.min() - 0.25 >= -1e-3
    assert np.allclose(run['h'], D - 0.25)
    # Without the mode the same obstacles are ignored (and h is inf).
    plain = d3.sim.ClosedLoopSimulator(d3.Tracker(), obstacles=obstacles)
    plain.reset(np.array([1.0, 0.0, 0.0]))
    assert np.isinf(plain.run(helix, 5.0)['h']).all()


def test_config_validation():
    """Out-of-range cbf_sites and the planar CONTOUR mode raise ConfigError."""
    config = cbf_config()
    config.obstacles.cbf_sites = 5
    with pytest.raises(vf.ConfigError):
        d3.Tracker(config)
    config = cbf_config()
    config.obstacles.obstacle_mode = vf.ObstacleMode.CONTOUR
    with pytest.raises(vf.ConfigError):
        d3.Tracker(config)


def test_vf_sim3d_obstacles(tmp_path, capsys):
    """vf-sim3d with every obstacle kind: summary, plot, CSV column h, input errors."""
    pytest.importorskip('matplotlib')  # the plot needs the [tools] extra
    png = tmp_path / 'avoid.png'
    table = tmp_path / 'avoid.csv'
    argv = [
        '--curve',
        'helix',
        '--pose',
        '1,0,0.2',
        '--sphere',
        '1.35,0,0.5,0.3',
        '--plane',
        '0,0,-0.1,0,0,1',
        '--box',
        '-1.2,0,1,0.15,0.15,0.3,0.4',
        '--cylinder',
        '0,-1.4,0.1',
        '--cylinder',
        '0,1.5,0.1,0,1',
        '-o',
        str(png),
        '--csv',
        str(table),
    ]
    assert vf_sim3d.main(argv) == 0
    out = capsys.readouterr().out
    assert 'obstacles    min h' in out and 'settled      yes' in out
    assert png.stat().st_size > 1000
    with open(table) as f:
        header = next(csv.reader(f))
    assert header[-1] == 'h'
    assert vf_sim3d.main(['--sphere', '1,2,3']) == 2
    assert vf_sim3d.main(['--cylinder', '0,0,0.1,1,0']) == 2
    assert vf_sim3d.main(['--sphere', '0,0,0,-1']) == 2
    primitives = curves3d.primitives_from_args(
        vf_sim3d.build_parser().parse_args(['--box', '0,0,0,1,1,1'])
    )
    assert len(primitives) == 1 and curves3d.obstacle_extent(primitives).shape == (8, 3)
