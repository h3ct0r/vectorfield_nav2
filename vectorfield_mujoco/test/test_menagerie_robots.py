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

"""Tests of the Menagerie robots: TIAGo, TidyBot and the Skydio X2 load and move sensibly.

They need the Menagerie checkout (``ros2 run vectorfield_mujoco fetch_menagerie`` or
``VF_MENAGERIE_DIR``) and skip without it; they never download it themselves.
"""

import math

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from vectorfield_mujoco import menagerie, scene  # noqa: E402

pytestmark = pytest.mark.skipif(
    not menagerie.is_available(), reason='MuJoCo Menagerie not fetched (fetch_menagerie)'
)


def _run(world, seconds, vx, vy=0.0, omega=0.0):
    for _ in range(int(round(seconds / world.model.opt.timestep))):
        world.drive(vx, vy, omega)
        mujoco.mj_step(world.model, world.data)


def test_tiago_differential_drive():
    """TIAGo loads with its upper body frozen and drives like a differential base."""
    world = scene.make_world('tiago', pose=(0.5, -0.5, 0.4))
    model = world.model
    # Upper body frozen: free joint (7) + 2 wheels + 4 casters x 2 joints; wheel servos only.
    assert model.nq == 7 + 2 + 8 and model.nu == 2
    assert world.geometry['radius'] == pytest.approx(0.098, abs=0.002)
    assert world.geometry['track'] == pytest.approx(0.403, abs=0.005)
    _run(world, 0.5, 0.0)
    x0, y0, _, yaw0 = world.base_pose()
    _run(world, 3.0, 0.4)
    x1, y1, z1, yaw1 = world.base_pose()
    assert math.hypot(x1 - x0, y1 - y0) == pytest.approx(1.2, rel=0.05)
    assert abs(yaw1 - yaw0) < 0.02 and abs(z1) < 0.01
    _run(world, 3.0, 0.0, omega=1.0)
    lin, ang = world.base_twist()
    assert ang[2] == pytest.approx(1.0, rel=0.05) and abs(lin[0]) < 0.03
    _run(world, 3.0, 0.3, omega=0.5)
    lin, ang = world.base_twist()
    assert lin[0] == pytest.approx(0.3, rel=0.05) and ang[2] == pytest.approx(0.5, rel=0.05)


def test_tidybot_holonomic():
    """TidyBot starts at the initial pose and tracks a holonomic twist."""
    world = scene.make_world('tidybot', pose=(1.0, -1.0, 0.5))
    assert world.base_pose()[:2] == pytest.approx((1.0, -1.0))
    assert world.base_pose()[3] == pytest.approx(0.5)
    _run(world, 2.0, 0.2, vy=0.3, omega=0.4)
    lin, ang = world.base_twist()
    assert lin[:2] == pytest.approx([0.2, 0.3], abs=0.01) and ang[2] == pytest.approx(
        0.4, abs=0.01
    )
    assert world.data.ncon == 0  # the base floats on its planar joints, clear of the floor
    world.reset()
    assert world.base_pose()[:2] == pytest.approx((1.0, -1.0))


def test_x2_hover_and_velocity():
    """The X2 hovers at the take-off altitude and tracks a 3D velocity."""
    world = scene.make_world('x2', pose=(0.0, 0.0, 0.3), altitude=1.0)
    quad = world.controller.controller.quad
    assert quad.mass == pytest.approx(1.325)
    assert quad.hover_thrusts() == pytest.approx([3.2495625] * 4, rel=1e-3)
    assert world.model.nlight == 1  # the world's sun: the X2's own spotlight is removed
    _run(world, 2.0, 0.0)
    x, y, z, yaw = world.base_pose()
    assert (x, y) == pytest.approx((0.0, 0.0), abs=0.02)
    assert z == pytest.approx(1.005, abs=0.02) and yaw == pytest.approx(0.3, abs=0.01)
    vel_sp = np.array([0.5, -0.3, 0.2])
    for _ in range(int(3.0 / world.model.opt.timestep)):
        world.controller.apply(world.model, world.data, vel_sp, 0.3)
        mujoco.mj_step(world.model, world.data)
    assert world.data.qvel[:3] == pytest.approx(vel_sp, abs=0.05)
    assert world.base_twist()[1][2] == pytest.approx(0.3, abs=0.05)
