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

"""The Tracker and the closed-loop simulator."""

import math

import numpy as np
import pytest
import vectorfield as vf
from vectorfield.sim import ClosedLoopSimulator, SimulationOptions


def _circle(radius=1.0):
    return vf.make_curve(vf.CurveSpec.builtin('ellipse', a=radius, b=radius))


def _config(model, law):
    config = vf.CoreConfig()
    config.motion.motion_model = model
    config.motion.tracking_law = law
    return config


@pytest.mark.parametrize(
    'model,law',
    [
        (vf.MotionModel.DIFFERENTIAL, vf.TrackingLaw.FEEDBACK_LINEARIZATION),
        (vf.MotionModel.DIFFERENTIAL, vf.TrackingLaw.HEADING_TRACKING),
        (vf.MotionModel.SKID_STEER, vf.TrackingLaw.FEEDBACK_LINEARIZATION),
        (vf.MotionModel.OMNIDIRECTIONAL, vf.TrackingLaw.HOLONOMIC),
    ],
)
def test_closed_loop_converges_on_a_circle(model, law):
    """From 0.5 m outside a unit circle, |D| < 0.05 m after a 15 s transient."""
    tracker = vf.Tracker(_config(model, law))
    simulator = ClosedLoopSimulator(tracker, SimulationOptions(stop_when_settled=False))
    simulator.reset(vf.Pose2(1.5, 0.0, math.pi / 2))
    result = simulator.run(vf.TrackerInput(_circle()), 30.0)
    assert len(result) == 600
    late = result.t >= 15.0
    assert np.abs(result.D[late]).max() < 0.05
    assert (result.phase[late] == int(vf.Phase.FOLLOW)).all()
    assert not (result.status & int(vf.Status.NON_FINITE)).any()
    # The robot actually travels around the circle at about v_r.
    speed = np.linalg.norm(np.diff(result.pose[late, :2], axis=0), axis=1) / 0.05
    assert speed.mean() == pytest.approx(0.4, rel=0.1)


def test_manual_step_loop_matches_the_simulator():
    """Tracker.step plus sim.integrate reproduces ClosedLoopSimulator.run."""
    tracker = vf.Tracker(vf.CoreConfig())
    curve = _circle(1.5)
    start = vf.Pose2(2.0, 0.3, 1.0)
    simulator = ClosedLoopSimulator(tracker, SimulationOptions(stop_when_settled=False))
    simulator.reset(start)
    result = simulator.run(vf.TrackerInput(curve), 2.0)

    state = vf.Tracker.initial_state()
    pose, twist = start, vf.Twist2()
    kinematics = tracker.kinematics
    for k in range(len(result)):
        np.testing.assert_allclose(pose.to_array(), result.pose[k], atol=1e-12)
        inp = vf.TrackerInput(curve, pose=pose, measured=twist, t=0.05 * k, dt=0.05)
        out, state = tracker.step(inp, state)
        np.testing.assert_allclose(out.cmd.to_array(), result.cmd[k], atol=1e-12)
        pose = vf.sim.integrate(kinematics, pose, out.cmd, 0.05)
        twist = out.cmd
    assert state.phase == simulator.state.phase


def test_step_returns_output_and_new_state():
    """step() leaves the input state unchanged and fills the output record."""
    tracker = vf.Tracker(vf.CoreConfig())
    state = vf.Tracker.initial_state()
    assert not state.hint.valid and not state.limiter.valid
    inp = vf.TrackerInput(_circle(), pose=(1.3, 0.0, math.pi / 2))
    out, nxt = tracker.step(inp, state)
    assert not state.hint.valid and nxt.hint.valid
    assert out.phase == vf.Phase.FOLLOW and nxt.phase == vf.Phase.FOLLOW
    assert out.field.valid and out.field.D == pytest.approx(0.3, abs=0.05)
    assert math.isinf(out.s_free) and out.predicted.shape == (0, 3)
    with pytest.raises(ValueError):
        tracker.step(vf.TrackerInput(), state)


def test_probe_points_and_streamline_after_a_step():
    """Probes see the field of the step; a streamline converges onto the circle."""
    tracker = vf.Tracker(vf.CoreConfig())
    inp = vf.TrackerInput(_circle(), pose=(1.0, 0.0, math.pi / 2))
    _, state = tracker.step(inp, vf.Tracker.initial_state())
    pts = np.array([[2.0, 0.0], [0.0, 1.5], [-0.5, 0.0]])
    probes = tracker.probe_points(pts, inp, state)
    assert probes['F'].shape == (3, 2) and probes['D'].shape == (3,)
    np.testing.assert_allclose(np.linalg.norm(probes['F'], axis=1), 0.4, rtol=1e-9)
    np.testing.assert_allclose(probes['D'], [1.0, 0.5, 0.5], atol=1e-6)
    single = tracker.probe(pts[0], inp, state)
    np.testing.assert_allclose(single.F, probes['F'][0])
    line = tracker.streamline([2.0, 0.0], inp, state, step=0.02, max_points=600)
    assert line.shape == (600, 2)
    assert abs(np.linalg.norm(line[-1]) - 1.0) < 0.02


def test_open_curve_settles_at_the_goal():
    """An open polyline with a goal ends settled near its last vertex."""
    tracker = vf.Tracker(vf.CoreConfig())
    curve = vf.make_curve(vf.CurveSpec.polyline([[0.0, 0.0], [2.0, 0.0]]))
    goal = vf.GoalInfo(pose=(2.0, 0.0, 0.0), xy_tol=0.1, has_yaw=False)
    simulator = ClosedLoopSimulator(tracker)
    simulator.reset((0.0, 0.2, 0.0))
    result = simulator.run(vf.TrackerInput(curve, goal=goal), 30.0)
    assert result.settled[-1]
    assert np.hypot(*(result.pose[-1, :2] - [2.0, 0.0])) < 0.1


def test_obstacles_stop_the_robot_before_a_wall():
    """Collision prediction keeps the footprint off a wall across the path."""
    mask = np.zeros((60, 120), dtype=np.uint8)
    mask[:, 80] = 1
    grid = vf.OccupancyGrid(mask, 0.05, origin=(-1.0, -1.5))
    curve = vf.make_curve(vf.CurveSpec.polyline([[0.0, 0.0], [4.0, 0.0]]))
    goal = vf.GoalInfo(pose=(4.0, 0.0, 0.0))
    tracker = vf.Tracker(vf.CoreConfig())
    simulator = ClosedLoopSimulator(tracker, SimulationOptions(stop_when_settled=False))
    simulator.reset((0.0, 0.0, 0.0))
    result = simulator.run(vf.TrackerInput(curve, goal=goal, obstacles=vf.ObstacleMap(grid)), 15.0)
    wall_x = -1.0 + 80 * 0.05
    assert result.pose[:, 0].max() < wall_x - 0.22
    assert (result.status & int(vf.Status.BLOCKED | vf.Status.COLLISION_AHEAD)).any()
    assert np.isfinite(result.s_free).any()
    assert simulator.scratch.rollout.shape[1] == 3


def test_update_hook_and_python_disturbance():
    """The update hook edits the input; a callable disturbance is called every period."""
    calls = []

    def disturbance(t, pose):
        calls.append(t)
        return (0.0, 0.0)

    def update(t, inp):
        if t >= 5.0:
            inp.speed_limit_scale = 0.5

    tracker = vf.Tracker(vf.CoreConfig())
    options = SimulationOptions(stop_when_settled=False, disturbance=disturbance)
    simulator = ClosedLoopSimulator(tracker, options)
    simulator.reset((1.0, 0.0, math.pi / 2))
    result = simulator.run(vf.TrackerInput(_circle()), 10.0, update)
    assert len(calls) == len(result) == 200
    assert np.abs(result.cmd[result.t >= 7.0, 0]).max() <= 0.2 + 1e-9
    assert np.abs(result.cmd[result.t < 5.0, 0]).max() > 0.3

    def broken(t, inp):
        raise RuntimeError('hook failed')

    with pytest.raises(RuntimeError, match='hook failed'):
        simulator.run(vf.TrackerInput(_circle()), 1.0, broken)


def test_native_disturbance_is_reproducible():
    """bounded_random_disturbance is seeded: two runs agree, another seed differs."""

    def run(seed):
        options = SimulationOptions(
            stop_when_settled=False, disturbance=vf.sim.bounded_random_disturbance(0.05, seed)
        )
        simulator = ClosedLoopSimulator(vf.Tracker(vf.CoreConfig()), options)
        simulator.reset((1.2, 0.0, math.pi / 2))
        return simulator.run(vf.TrackerInput(_circle()), 5.0).pose

    np.testing.assert_array_equal(run(7), run(7))
    assert not np.array_equal(run(7), run(8))
    d = vf.sim.bounded_random_disturbance(0.05, 1)
    assert np.linalg.norm(d(0.0, vf.Pose2())) <= 0.05
