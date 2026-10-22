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

"""The experimental 3D core through vectorfield.d3, and the vf-sim3d / vf-curve --dim 3 CLIs."""

import csv
import math

import numpy as np
import pytest
import vectorfield as vf
from vectorfield import d3
from vectorfield_tools import curves3d, vf_curve, vf_sim3d


def _needs(*modules):
    for module in modules:
        pytest.importorskip(module)


def test_submodule_imports():
    """vectorfield.d3 and vectorfield.d3.sim import like package modules."""
    import vectorfield.d3.sim as sim3

    assert sim3 is d3.sim
    assert 'd3' in vf.__all__
    names = {shape['name'] for shape in d3.builtin_shapes()}
    assert names == {'helix', 'circle', 'ellipse', 'viviani', 'lemniscate3d', 'torus_knot'}


def test_curves_and_numpy_shapes():
    """Specs, curves, sampling and closest points with (3,) and (N, 3) arrays."""
    helix = d3.make_curve(d3.CurveSpec.builtin('helix', radius=1.0, lead=0.5, turns=2))
    assert not helix.is_closed
    c = 0.5 / (2 * math.pi)
    assert helix.length() == pytest.approx(4 * math.pi * math.hypot(1.0, c), rel=1e-12)
    samples = helix.sample(ds=0.05)
    assert samples.ndim == 2 and samples.shape[1] == 3
    assert helix.point_at(0.0).shape == (3,)
    assert helix.point_at(np.array([0.0, 1.0, 2.0])).shape == (3, 3)
    cp = helix.closest_point(np.array([1.2, 0.0, 0.0]))
    assert cp.valid and cp.D == pytest.approx(0.2, abs=1e-9)
    assert abs(np.dot(cp.n, cp.T)) < 1e-12
    D = helix.distance(samples[:10])
    assert np.all(D < 1e-7)
    poly = d3.make_curve(d3.CurveSpec.polyline([[0, 0, 0], [1, 0, 0], [1, 1, 1]]))
    assert poly.length() == pytest.approx(1 + math.sqrt(2))
    assert poly.spec.points.shape == (3, 3)
    expr = d3.make_curve(d3.CurveSpec.expression('cos(s)', 'sin(s)', '0.2*sin(2*s)'))
    assert expr.is_closed and not expr.is_time_varying
    assert d3.CurveSpec.builtin('circle', radius=2.0) == d3.CurveSpec.builtin('circle', radius=2.0)


def test_curve_errors():
    """Invalid 3D curves raise vectorfield.CurveError with the result code."""
    with pytest.raises(vf.CurveError) as info:
        d3.make_curve(d3.CurveSpec.expression('cos(s)', 'sin(s)', 's > 1 && s < 2'))
    assert info.value.code == 'INVALID_CURVE'
    with pytest.raises(vf.CurveError) as info:
        d3.make_curve(d3.CurveSpec.expression('cos(s)', 'sin(s)', 't', closed=False))
    assert info.value.code == 'UNSUPPORTED'
    with pytest.raises(vf.CurveError):
        d3.make_curve(d3.CurveSpec.builtin('torus_knot', p=2, q=4))
    with pytest.raises(ValueError):
        d3.CurveSpec.polyline([[0, 0], [1, 1]])


def test_field_unit_speed_and_streamline():
    """||Phi|| = v_r on random points; a streamline approaches the curve."""
    curve = d3.make_curve(d3.CurveSpec.builtin('viviani'))
    field = d3.DistanceField(vf.GainFunction(vf.GainType.ATAN, 2.0), v_r=0.5)
    rng = np.random.default_rng(1)
    points = rng.uniform(-3, 3, size=(500, 3))
    speeds = np.linalg.norm(field(curve, points), axis=1)
    assert np.max(np.abs(speeds - 0.5)) < 1e-12
    line = field.streamline(curve, np.array([2.5, 0.5, 1.0]), step=0.05, count=300)
    assert line.shape[1] == 3
    assert curve.distance(line[-1:])[0] < 0.05


def test_tracker_step_and_closed_loop():
    """Tracker.step returns (output, state); the simulator converges on a helix and settles."""
    config = d3.TrackerConfig(desired_speed=0.5)
    config.limits.max_axis_speed = np.array([1.0, 1.0, 0.8])
    assert config.limits.max_axis_speed[2] == pytest.approx(0.8)
    tracker = d3.Tracker(config)
    curve = d3.make_curve(d3.CurveSpec.builtin('helix'))
    out, state = tracker.step(curve, np.array([1.3, 0.0, 0.0]))
    assert out.has_goal and out.phase == vf.Phase.FOLLOW
    assert np.linalg.norm(out.v) > 0.0
    out, state = tracker.step(curve, np.array([1.3, 0.0, 0.0]), state=state, t=0.05)
    simulator = d3.sim.ClosedLoopSimulator(tracker, period=0.05)
    simulator.reset(np.array([1.6, -0.4, -0.5]))
    run = simulator.run(curve, 60.0)
    assert run['position'].shape == (len(run['t']), 3)
    after = run['D'][(run['t'] > 15.0) & (run['sigma'] > 1.0)]
    assert after.size > 100 and after.max() < 0.01
    assert run['settled'][-1]
    assert simulator.last_output.settled


def test_cancel_and_plant():
    """cancel_at stops the closed loop; integrate is the exact ZOH step."""
    tracker = d3.Tracker(d3.TrackerConfig())
    curve = d3.make_curve(d3.CurveSpec.builtin('circle'))
    simulator = d3.sim.ClosedLoopSimulator(tracker)
    simulator.reset(np.array([1.0, 0.0, 0.0]))
    run = simulator.run(curve, 20.0, cancel_at=3.0)
    assert run['phase'][-1] == int(vf.Phase.STOPPED)
    assert run['t'][-1] < 4.0
    position, yaw, v, wz = d3.sim.integrate(
        np.zeros(3), 0.0, np.zeros(3), 0.0, np.array([0.3, 0.0, 0.1]), 0.5, 0.1
    )
    assert np.allclose(position, [0.03, 0.0, 0.01]) and yaw == pytest.approx(0.05)


def test_curves3d_parsing_and_files(tmp_path):
    """Built-ins, expression triples and CSV / YAML files round-trip."""
    spec = curves3d.parse_curve('helix:radius=2,turns=1')
    assert spec.parameters == {'radius': 2.0, 'turns': 1.0}
    spec = curves3d.parse_curve('cos(s); sin(s); 0.1*s', s_range='0:pi', closed=False)
    assert spec.z_expression == '0.1*s' and spec.s_max == pytest.approx(math.pi)
    path = tmp_path / 'path.csv'
    path.write_text('x,y,z\n0,0,0\n1,0,0.5\n2,1,1\n', encoding='utf-8')
    spec = curves3d.parse_curve(str(path))
    assert spec.points.shape == (3, 3) and not spec.closed
    _needs('yaml')
    yaml_path = tmp_path / 'knot.yaml'
    knot = d3.CurveSpec.builtin('torus_knot', p=3, q=2)
    curves3d.write_curve_yaml(yaml_path, knot)
    assert curves3d.read_curve_file(yaml_path) == knot


def test_vf_curve_dim3_exports(tmp_path, capsys):
    """vf-curve --dim 3 lists, describes and exports 3D curves; the 2D path is unchanged."""
    assert vf_curve.main(['--dim', '3', '--list']) == 0
    assert 'viviani' in capsys.readouterr().out
    csv_path = tmp_path / 'viviani.csv'
    assert vf_curve.main(['--dim', '3', '--curve', 'viviani', '--export', str(csv_path)]) == 0
    out = capsys.readouterr().out
    assert 'closed       yes' in out and 'z [' in out
    with open(csv_path, encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    radius = [
        math.sqrt(float(r['x']) ** 2 + float(r['y']) ** 2 + float(r['z']) ** 2) for r in rows
    ]
    assert max(abs(r - 2.0) for r in radius) < 1e-5
    assert vf_curve.main(['--dim', '3', '--curve', 'trefoil']) == 2
    capsys.readouterr()
    assert vf_curve.main(['--curve', 'ellipse:a=2,b=1']) == 0
    assert 'length       9.6884 m' in capsys.readouterr().out


def test_vf_sim3d_plot_gif_and_csv(tmp_path, capsys):
    """vf-sim3d writes a 3D plot, a GIF and the records."""
    _needs('matplotlib', 'PIL')
    png, gif, records = tmp_path / 'helix.png', tmp_path / 'helix.gif', tmp_path / 'run.csv'
    argv = [
        '--curve',
        'helix:turns=1',
        '--pose',
        '1.3,0,-0.2',
        '--streamlines',
        '4',
        '-o',
        str(png),
        '--gif',
        str(gif),
        '--gif-seconds',
        '1',
        '--fps',
        '4',
        '--csv',
        str(records),
    ]
    assert vf_sim3d.main(argv) == 0
    out = capsys.readouterr().out
    assert 'settled      yes' in out
    assert png.stat().st_size > 1000 and gif.stat().st_size > 1000
    with open(records, encoding='utf-8', newline='') as stream:
        header = stream.readline().strip().split(',')
    assert header[:4] == ['t', 'x', 'y', 'z']
    with pytest.raises(SystemExit) as info:
        vf_sim3d.main(['--help'])
    assert info.value.code == 0
    assert vf_sim3d.main(['--curve', 'cos(s); sin(s); t', '--open']) == 2


def _rotation(yaw, pitch, roll):
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _transform(yaw, pitch, roll, translation):
    T = np.eye(4)
    T[:3, :3] = _rotation(yaw, pitch, roll)
    T[:3, 3] = translation
    return T


def test_curve_frame_and_se3():
    """CurveFrame defaults, validation and extrapolation; exp_se3 / log_se3 / body_twist."""
    frame = d3.CurveFrame()
    assert not frame.moving and np.array_equal(frame.T_eval_curve, np.eye(4))
    frame = d3.CurveFrame(v=[0.1, 0.0, 0.0])
    assert frame.moving
    np.testing.assert_allclose(frame.at(2.0).T_eval_curve[:3, 3], [0.2, 0.0, 0.0], atol=1e-15)
    with pytest.raises(ValueError):
        d3.CurveFrame(T_eval_curve=np.eye(3))
    with pytest.raises(ValueError):
        frame.T_eval_curve = np.ones((4, 4))
    v, w = np.array([0.3, -0.2, 0.5]), np.array([0.4, -1.0, 2.5])
    T = d3.exp_se3(v, w)
    back_v, back_w = d3.log_se3(T)
    np.testing.assert_allclose(back_v, v, atol=1e-12)
    np.testing.assert_allclose(back_w, w, atol=1e-12)
    a = _transform(3.1, 0.2, -0.1, [1.0, 2.0, 3.0])
    tv, tw = d3.body_twist(a, a @ d3.exp_se3(v, w, 0.05), 0.05)
    np.testing.assert_allclose(tv, v, atol=1e-9)
    np.testing.assert_allclose(tw, w, atol=1e-9)
    rotating = d3.CurveFrame(w=[0.0, 0.0, 0.5])
    np.testing.assert_allclose(
        rotating.velocity(np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0]])),
        [[0.0, 0.5, 0.0], [-1.0, 0.0, 0.0]],
        atol=1e-15,
    )


def test_frame_twist_estimator_constant_twist_across_pi():
    """The estimator recovers a constant twist to 1e-6 while the heading crosses +-pi."""
    v, w = np.array([0.3, -0.1, 0.2]), np.array([0.2, -0.1, 1.2])
    T0 = _transform(math.pi - 0.05, 0.1, -0.2, [1.0, -2.0, 0.5])
    estimator = d3.FrameTwistEstimator(d3.FrameTwistConfig(cutoff_hz=2.0))
    assert estimator.add_pose(0.0, T0) == d3.FrameSampleResult.STARTED
    for k in range(1, 41):
        t = 0.05 * k
        assert estimator.add_pose(t, T0 @ d3.exp_se3(v, w, t)) == d3.FrameSampleResult.ACCEPTED
        estimate = estimator.estimate(t)
        assert estimate.ready and not estimate.stale
        np.testing.assert_allclose(estimate.v, v, atol=1e-6)
        np.testing.assert_allclose(estimate.w, w, atol=1e-6)
    jump = T0 @ d3.exp_se3(v, w, 2.05)
    jump[0, 3] += 1.0
    assert estimator.add_pose(2.05, jump) == d3.FrameSampleResult.OUTLIER
    assert estimator.outliers == 1
    assert estimator.estimate(5.0).stale and not estimator.estimate(5.0).curve_frame().moving
    assert estimator.estimate(2.0).curve_frame().moving


def test_tracker_in_a_translating_frame():
    """Phi_T = (I - T T^T) u for a circle in a translating frame, and the closed loop converges."""
    circle = d3.make_curve(d3.CurveSpec.builtin('circle', radius=2.0))
    T = _transform(0.3, 0.2, 0.0, [0.2, 0.0, 0.5])
    v_F = np.array([0.15, 0.05, 0.03])
    frame = d3.CurveFrame(T, v=v_F)
    u = T[:3, :3] @ v_F
    tracker = d3.Tracker(d3.TrackerConfig(desired_speed=0.5))
    out, _ = tracker.step(circle, (T @ [2.3, 0.4, 0.2, 1.0])[:3], frame=frame)
    # The circle runs counterclockwise in F's xy plane: T_F = z x x*_F / R, mapped by R_F.
    t_eval = T[:3, :3] @ np.cross([0.0, 0.0, 1.0], np.asarray(out.x_star_curve) / 2.0)
    np.testing.assert_allclose(out.Phi_T, u - (t_eval @ u) * t_eval, atol=1e-9)
    np.testing.assert_allclose(np.linalg.norm(out.Phi), 0.5, atol=1e-12)
    simulator = d3.sim.ClosedLoopSimulator(tracker, frame=frame)
    simulator.reset((T @ [2.5, 0.3, 0.3, 1.0])[:3])
    run = simulator.run(circle, 40.0)
    assert run['frame'].shape == (len(run['t']), 4, 4)
    assert run['D'][run['t'] >= 15.0].max() < 0.01
    assert not np.any(run['status'] & int(vf.Status.ASSUMPTION3_CLAMPED))
    np.testing.assert_allclose(
        simulator.frame_at(10.0).T_eval_curve[:3, 3], T[:3, 3] + 10.0 * u, atol=1e-12
    )


def test_simulator_frame_motion_callback_and_open_curve_hold():
    """A frame_motion callback drives the frame; an open helix in a rotating frame settles."""
    calls = []

    def motion(t):
        calls.append(t)
        return d3.CurveFrame(w=[0.0, 0.0, 0.15]).at(t)

    helix = d3.make_curve(d3.CurveSpec.builtin('helix', cx=0.8))
    tracker = d3.Tracker(d3.TrackerConfig(desired_speed=0.5))
    simulator = d3.sim.ClosedLoopSimulator(tracker, frame_motion=motion, stop_when_settled=False)
    simulator.reset([2.0, -0.2, -0.1])
    run = simulator.run(helix, 60.0)
    assert len(calls) == len(run['t'])
    settled = np.flatnonzero(run['settled'])
    assert settled.size > 0
    follow = (run['t'] >= 15.0) & (run['t'] < run['t'][settled[0]])
    assert run['D'][follow].max() < 0.02
    assert run['settled'][settled[0] :].all()  # the hold rides with the frame
    assert np.linalg.norm(simulator.last_output.frame_velocity) > 0.05

    def broken(t):
        raise RuntimeError('frame source failed')

    simulator = d3.sim.ClosedLoopSimulator(tracker, frame_motion=broken)
    simulator.reset([2.0, 0.0, 0.0])
    with pytest.raises(RuntimeError, match='frame source failed'):
        simulator.run(helix, 1.0)


def test_vf_sim3d_moving_frame(tmp_path, capsys):
    """vf-sim3d --frame-velocity / --frame-rate run in a moving frame and plot it."""
    _needs('matplotlib', 'PIL')
    png, gif = tmp_path / 'moving.png', tmp_path / 'moving.gif'
    argv = [
        '--curve',
        'helix:turns=1,cx=0.8',
        '--frame-velocity',
        '0.05,0,0',
        '--frame-rate',
        '0,0,0.15',
        '-o',
        str(png),
        '--gif',
        str(gif),
        '--gif-seconds',
        '1',
        '--fps',
        '4',
    ]
    assert vf_sim3d.main(argv) == 0
    out = capsys.readouterr().out
    assert 'frame        v = (0.05, 0, 0) m/s, w = (0, 0, 0.15) rad/s' in out
    assert 'settled      yes' in out
    assert png.stat().st_size > 1000 and gif.stat().st_size > 1000
    assert vf_sim3d.main(['--curve', 'circle', '--frame-rate', '1,2']) == 2
