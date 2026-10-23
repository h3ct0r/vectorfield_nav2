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

"""Unit tests of vectorfield_mujoco.flight on a synthetic quadrotor with the Skydio X2's numbers.

The model copies X2's masses, rotor sites, thrust/yaw gears (0.0201 N m per N) and thrust range
(0-13 N), and the air of its MJCF, without its mesh, so the tests need no Menagerie download.
"""

import math

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from vectorfield_mujoco import flight  # noqa: E402

QUAD = """
<mujoco model="x2_like">
  <option timestep="0.002" integrator="implicitfast" density="1.225" viscosity="1.8e-5"/>
  <worldbody>
    <geom type="plane" size="0 0 0.05"/>
    <body name="x2" pos="0 0 1">
      <freejoint/>
      <geom type="ellipsoid" size=".16 .04 .02" pos="0 0 0.02" mass=".325"/>
      <geom type="ellipsoid" size=".13 .13 .01" pos="-.14 -.18 .05" mass=".25"/>
      <geom type="ellipsoid" size=".13 .13 .01" pos="-.14 .18 .05" mass=".25"/>
      <geom type="ellipsoid" size=".13 .13 .01" pos=".14 .18 .08" mass=".25"/>
      <geom type="ellipsoid" size=".13 .13 .01" pos=".14 -.18 .08" mass=".25"/>
      <site name="thrust1" pos="-.14 -.18 .05"/>
      <site name="thrust2" pos="-.14 .18 .05"/>
      <site name="thrust3" pos=".14 .18 .08"/>
      <site name="thrust4" pos=".14 -.18 .08"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="thrust1" site="thrust1" gear="0 0 1 0 0 -.0201" ctrlrange="0 13"/>
    <motor name="thrust2" site="thrust2" gear="0 0 1 0 0 .0201" ctrlrange="0 13"/>
    <motor name="thrust3" site="thrust3" gear="0 0 1 0 0 -.0201" ctrlrange="0 13"/>
    <motor name="thrust4" site="thrust4" gear="0 0 1 0 0 .0201" ctrlrange="0 13"/>
  </actuator>
</mujoco>
"""


@pytest.fixture
def sim():
    """Return (model, data, MujocoQuadrotor) hovering at 1 m."""
    model = mujoco.MjModel.from_xml_string(QUAD)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data, flight.MujocoQuadrotor(model, 'x2')


def _run(sim, seconds, vel_sp, yaw_rate=0.0, record=None):
    model, data, quad = sim
    for _ in range(int(round(seconds / model.opt.timestep))):
        u = quad.apply(model, data, vel_sp, yaw_rate)
        assert np.all(u >= -1e-9) and np.all(u <= 13.0 + 1e-9)
        mujoco.mj_step(model, data)
        if record is not None:
            record.append(data.qpos.copy())


def test_model_parameters(sim):
    """Mass, mixer and hover thrust are read from the model."""
    model, _, quad = sim
    q = quad.controller.quad
    assert q.mass == pytest.approx(1.325)
    # Thrust row and yaw row of the mixer come from the gears.
    assert q.wrench_map[0] == pytest.approx([1, 1, 1, 1])
    assert q.wrench_map[3] == pytest.approx([-0.0201, 0.0201, -0.0201, 0.0201])
    # X2's keyframe "hover": 3.2495625 N per rotor.
    assert q.hover_thrusts() == pytest.approx([3.2495625] * 4, rel=1e-3)
    assert np.all(np.linalg.eigvalsh(q.inertia) > 0)


def test_hover(sim):
    """A zero velocity set-point holds the start position at the hover thrust."""
    model, data, _ = sim
    _run(sim, 3.0, [0.0, 0.0, 0.0])
    assert data.qpos[:3] == pytest.approx([0.0, 0.0, 1.0], abs=0.02)
    assert np.linalg.norm(data.qvel[:6]) < 0.02
    assert data.ctrl == pytest.approx([3.2495625] * 4, rel=0.02)


def test_velocity_step(sim):
    """A 1 m/s step is tracked quickly, without overshoot or altitude loss."""
    model, data, quad = sim
    trace = []
    _run(sim, 3.0, [1.0, 0.0, 0.0], record=trace)
    assert data.qvel[0] == pytest.approx(1.0, abs=0.05)
    assert abs(data.qvel[1]) < 0.02 and abs(data.qvel[2]) < 0.05
    assert data.qpos[2] == pytest.approx(1.0, abs=0.05)
    # The response is a well-damped step: 90 % within 1.5 s, overshoot under 15 %.
    xs = np.array(trace)[:, 0]
    speed = np.gradient(xs, model.opt.timestep)
    assert speed[int(1.5 / model.opt.timestep)] > 0.9
    assert speed.max() < 1.15
    # The tilt never exceeds the limit (35°) by more than a few degrees.
    tilts = [math.acos(max(-1.0, min(1.0, 1 - 2 * (q[4] ** 2 + q[5] ** 2)))) for q in trace]
    assert max(tilts) < quad.controller.gains.max_tilt + math.radians(5)


def test_climb_and_yaw_rate(sim):
    """Vertical velocity and yaw rate are tracked together."""
    model, data, _ = sim
    _run(sim, 2.5, [0.0, 0.0, 0.5], yaw_rate=0.5)
    assert data.qvel[2] == pytest.approx(0.5, abs=0.05)
    assert data.qvel[5] == pytest.approx(0.5, abs=0.05)  # body z rate, level flight


def test_mixer_saturation(sim):
    """Saturated commands stay within the thrust range, thrust first."""
    _, _, quad = sim
    u = quad.controller.mix(80.0, np.array([5.0, -5.0, 1.0]))
    assert np.all(u >= 0.0) and np.all(u <= 13.0)
    u = quad.controller.mix(13.0, np.array([0.0, 0.0, 0.2]))
    # Yaw is given up first: the collective thrust survives.
    assert u.sum() == pytest.approx(13.0, rel=1e-6)
