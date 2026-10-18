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

"""NumPy round trips: dtypes, shapes, copies and implicit conversions."""

import math

import numpy as np
import pytest
import vectorfield as vf


def test_vectors_are_float64_copies():
    """Vec2 attributes are (2,) float64 copies; writes go through the setter."""
    pose = vf.Pose2(1.0, 2.0, 0.5)
    p = pose.p
    assert p.dtype == np.float64 and p.shape == (2,)
    p[0] = 9.0
    assert pose.x == 1.0
    pose.p = np.array([3, 4])  # ints are converted
    assert (pose.x, pose.y) == (3.0, 4.0)
    x, y, yaw = pose
    assert (x, y, yaw) == (3.0, 4.0, 0.5)
    np.testing.assert_array_equal(pose.to_array(), [3.0, 4.0, 0.5])
    with pytest.raises(TypeError):
        pose.p = [1.0, 2.0, 3.0]


def test_implicit_pose_and_twist_conversions():
    """Tuples, lists and arrays convert to Pose2 and Twist2 arguments."""
    goal = vf.GoalInfo(pose=(1.0, 2.0, 0.3))
    assert goal.pose.yaw == 0.3
    goal.pose = np.array([4.0, 5.0, 6.0])
    assert goal.pose.x == 4.0
    goal.pose.yaw = 1.0  # nested structs are live references
    assert goal.pose.yaw == 1.0
    assert vf.Twist2([1.0, 2.0, 3.0]).wz == 3.0
    with pytest.raises(ValueError):
        vf.Pose2([1.0, 2.0])


def test_point_lists_round_trip():
    """(N, 2) point attributes accept lists and int arrays and return float64 copies."""
    spec = vf.CurveSpec(type=vf.CurveSpec.Type.POLYLINE, points=[[0, 0], [1, 0], [1, 1]])
    pts = spec.points
    assert pts.dtype == np.float64 and pts.shape == (3, 2)
    spec.points = np.arange(8).reshape(4, 2)
    np.testing.assert_array_equal(spec.points, np.arange(8).reshape(4, 2))
    spec.points = []
    assert spec.points.shape == (0, 2)
    with pytest.raises(ValueError):
        spec.points = np.zeros((3, 3))
    curve = vf.PolylineCurve(np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]), closed=True)
    np.testing.assert_array_equal(curve.vertices, [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    np.testing.assert_allclose(curve.stations, [0.0, 1.0, 2.0, 2.0 + math.sqrt(2.0)])


def test_curve_queries_shapes():
    """Scalar stations give (2,), arrays give (N, 2); batch queries give (N,) and (N, 2)."""
    curve = vf.make_curve(vf.CurveSpec.builtin('ellipse'))
    assert curve.point_at(0.0).shape == (2,)
    assert curve.point_at(np.linspace(0.0, 1.0, 7)).shape == (7, 2)
    assert curve.point_at([0.0, 0.5]).shape == (2, 2)
    batch = curve.closest_points(np.zeros((0, 2)))
    assert batch['D'].shape == (0,) and batch['x_star'].shape == (0, 2)
    iso = vf.Isometry2(math.pi / 2, [1.0, 0.0])
    np.testing.assert_allclose(iso.apply([1.0, 0.0]), [1.0, 1.0], atol=1e-15)
    np.testing.assert_allclose(iso.apply(np.eye(2)), [[1.0, 1.0], [0.0, 0.0]], atol=1e-15)
    np.testing.assert_allclose(iso.inverse().apply(iso.apply([0.3, -0.2])), [0.3, -0.2])
    np.testing.assert_allclose(vf.wrap_angle(np.array([3 * math.pi, -math.pi])), math.pi)


def test_simulation_result_columns():
    """SimulationResult columns have one row per period and fixed dtypes."""
    tracker = vf.Tracker(vf.CoreConfig())
    simulator = vf.sim.ClosedLoopSimulator(tracker, vf.sim.SimulationOptions(period=0.1))
    simulator.reset((1.2, 0.0, math.pi / 2))
    curve = vf.make_curve(vf.CurveSpec.builtin('ellipse'))
    result = simulator.run(vf.TrackerInput(curve), 2.0)
    n = len(result)
    assert n == 20
    expected = {
        't': ((n,), np.float64),
        'pose': ((n, 3), np.float64),
        'measured': ((n, 3), np.float64),
        'cmd': ((n, 3), np.float64),
        'x_eval': ((n, 2), np.float64),
        'F': ((n, 2), np.float64),
        'D': ((n,), np.float64),
        'phase': ((n,), np.uint8),
        'status': ((n,), np.uint32),
        'settled': ((n,), np.bool_),
    }
    columns = result.as_dict()
    for name, (shape, dtype) in expected.items():
        assert columns[name].shape == shape and columns[name].dtype == dtype, name
    np.testing.assert_allclose(result.t, 0.1 * np.arange(n), atol=1e-12)
    np.testing.assert_array_equal(result.measured[1:], result.cmd[:-1])
    assert len(simulator.records()) == n
    again = simulator.run(vf.TrackerInput(curve), 1.0)
    assert len(again) == 10 and len(simulator.records()) == 30
    assert again.t[0] == pytest.approx(2.0)


def test_status_flags_match_the_message_constants():
    """Status is an IntFlag with the FieldState.msg values."""
    assert int(vf.Status.NON_FINITE) == 1 and int(vf.Status.BLOCKED) == 256
    flags = vf.Status(1 | 256)
    assert vf.Status.BLOCKED in flags and vf.Status.HAIRPIN not in flags
    assert int(vf.Phase.STOPPED) == 4 and vf.Phase.ROTATE_TO_FIELD.is_rotation
