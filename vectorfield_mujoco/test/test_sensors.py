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

"""Unit tests of vectorfield_mujoco.sensors: lidar ranges in a box room, obstacle cloud."""

import math

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from vectorfield_mujoco import scene, sensors  # noqa: E402

HALF_X, HALF_Y, WALL = 2.0, 1.5, 0.05


def _room(sensor_pos, yaw):
    """Build a 4 m x 3 m room of world walls and a body with the lidar site and robot geoms."""
    walls = [
        {'type': 'box', 'pos': [s * (HALF_X + WALL), 0.0], 'size': [WALL, HALF_Y + 2 * WALL, 0.5]}
        for s in (-1, 1)
    ] + [
        {'type': 'box', 'pos': [0.0, s * (HALF_Y + WALL)], 'size': [HALF_X, WALL, 0.5]}
        for s in (-1, 1)
    ]
    spec = scene.build(obstacles=walls)
    body = spec.worldbody.add_body(
        name='robot', pos=sensor_pos, quat=[math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    )
    body.add_site(name='lidar')
    # Robot geoms around the sensor (visual and collision groups) must not be hit.
    body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.2, 0.2, 0.2], group=2)
    body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.3, 0, 0], group=3)
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def _expected(px, py, angle):
    c, s = math.cos(angle), math.sin(angle)
    hits = []
    if c > 1e-12:
        hits.append((HALF_X - px) / c)
    if c < -1e-12:
        hits.append((-HALF_X - px) / c)
    if s > 1e-12:
        hits.append((HALF_Y - py) / s)
    if s < -1e-12:
        hits.append((-HALF_Y - py) / s)
    return min(hits)


@pytest.mark.parametrize('yaw', [0.0, 0.6, -2.0])
def test_lidar_ranges_in_a_box_room(yaw):
    """Ranges in a box room equal the analytic distances, robot geoms ignored."""
    px, py = 0.4, -0.3
    model, data = _room([px, py, 0.25], yaw)
    lidar = sensors.Lidar2D(model, 'lidar', beams=360, range_max=10.0)
    ranges = lidar.scan(model, data)
    assert ranges.shape == (360,)
    assert lidar.angle_min == pytest.approx(-math.pi)
    assert lidar.angle_increment == pytest.approx(2 * math.pi / 360)
    angles = lidar.angle_min + lidar.angle_increment * np.arange(360) + yaw
    expected = [_expected(px, py, a) for a in angles]
    assert ranges == pytest.approx(expected, abs=1e-6)


def test_lidar_range_limits():
    """Returns beyond range_max read inf."""
    model, data = _room([0.0, 0.0, 0.25], 0.0)
    lidar = sensors.Lidar2D(model, 'lidar', beams=8, range_min=0.05, range_max=1.8)
    ranges = lidar.scan(model, data)
    # Beams along ±y hit at 1.5 m; along ±x the wall at 2.0 m is beyond range_max.
    angles = lidar.angle_min + lidar.angle_increment * np.arange(8)
    for a, r in zip(angles, ranges):
        if abs(math.sin(a)) > 0.99:
            assert r == pytest.approx(1.5)
        if abs(math.cos(a)) > 0.99:
            assert math.isinf(r)


def test_sphere_points_match_point_sim_3d():
    """Sphere sampling matches point_sim_3d: same count, on the surface."""
    pts = sensors.sphere_points(0.5, 0.05)
    # point_sim_3d: ceil(4 pi r^2 / spacing^2) + 1 points on the surface.
    assert len(pts) == math.ceil(4 * math.pi * 0.25 / 0.0025) + 1
    assert np.linalg.norm(pts, axis=1) == pytest.approx(np.full(len(pts), 0.5))


def test_world_cloud_lies_on_the_surfaces():
    """Cloud points lie on the obstacle surfaces, and none on the floor."""
    obstacles = [
        {'type': 'sphere', 'pos': [1.0, 0.0, 1.0], 'size': [0.3]},
        {'type': 'cylinder', 'pos': [-1.0, 0.5], 'size': [0.2, 0.6]},
        {'type': 'box', 'pos': [0.0, -1.0], 'size': [0.2, 0.3, 0.4], 'yaw': 0.5},
    ]
    model = scene.build(obstacles=obstacles).compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    cloud = sensors.world_cloud(model, data, spacing=0.05)
    assert len(cloud) > 500
    sphere = cloud[np.linalg.norm(cloud - [1.0, 0.0, 1.0], axis=1) < 0.35]
    assert np.linalg.norm(sphere - [1.0, 0.0, 1.0], axis=1) == pytest.approx(0.3)
    cyl = cloud[np.hypot(cloud[:, 0] + 1.0, cloud[:, 1] - 0.5) < 0.25] - [-1.0, 0.5, 0.6]
    radial = np.hypot(cyl[:, 0], cyl[:, 1])
    on_side = np.isclose(radial, 0.2) & (np.abs(cyl[:, 2]) <= 0.6 + 1e-9)
    on_cap = np.isclose(np.abs(cyl[:, 2]), 0.6) & (radial <= 0.2 + 1e-9)
    assert (on_side | on_cap).all()
    c, s = math.cos(0.5), math.sin(0.5)
    box = cloud[np.linalg.norm(cloud[:, :2] - [0.0, -1.0], axis=1) < 0.45] - [0.0, -1.0, 0.4]
    local = np.column_stack(
        (c * box[:, 0] + s * box[:, 1], -s * box[:, 0] + c * box[:, 1], box[:, 2])
    )
    assert np.max(np.abs(local) / [0.2, 0.3, 0.4], axis=1) == pytest.approx(np.ones(len(local)))
    # The floor is not an obstacle.
    assert cloud[:, 2].min() >= -1e-9
