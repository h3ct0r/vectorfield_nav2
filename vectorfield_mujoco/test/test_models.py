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

"""Tests of the package's own robots: the vendored Scout Mini and the Ackermann car.

They compile, the drive geometry read back from the compiled model matches the AgileX manual
(Scout Mini) and the Ackermann controller overlay (car), and they drive as their kinematics say.
No network: neither robot comes from the Menagerie.
"""

import math

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from vectorfield_mujoco import registry, scene  # noqa: E402

# SCOUT MINI User Manual v1.0.1 (AgileX), sections 1.2 and 6.1.
DATASHEET_WHEELBASE = 0.452
DATASHEET_TRACK = 0.450
DATASHEET_WHEEL_RADIUS = 0.0875
WHEEL_DROP = 0.077  # wheel centres below base_link (the xacro)

# vectorfield_bringup/config/controller/ackermann.yaml.
MIN_TURNING_RADIUS = 0.5


def _drive(world, seconds, vx, omega=0.0, vy=0.0):
    model, data = world.model, world.data
    for _ in range(int(round(seconds / model.opt.timestep))):
        world.drive(vx, vy, omega)
        mujoco.mj_step(model, data)


def test_models_dir_budget():
    """The vendored Scout Mini stays within its 3 MB budget and has its README."""
    root = registry.models_dir()
    size = sum(p.stat().st_size for p in (root / 'scout_mini').rglob('*') if p.is_file())
    assert size <= 3 * 1024 * 1024
    assert (root / 'scout_mini' / 'README.md').is_file()


def test_scout_mini_geometry():
    """Wheel radius, wheelbase and track read from the model match the manual."""
    world = scene.make_world('scout_mini')
    g = world.geometry
    assert g['radius'] == pytest.approx(DATASHEET_WHEEL_RADIUS)
    assert g['wheelbase'] == pytest.approx(DATASHEET_WHEELBASE)
    # The wheel collisions are centred on the wheels: the contact track is the track.
    assert g['track'] == pytest.approx(DATASHEET_TRACK)
    model, data = world.model, world.data
    joints = {
        side: data.xanchor[model.joint(world.name(f'front_{side}_wheel')).id]
        for side in ('left', 'right')
    }
    assert joints['left'][1] - joints['right'][1] == pytest.approx(DATASHEET_TRACK)
    base = world.base_id
    # The datasheet mass (20 kg) for the whole robot, and it rests on its wheels.
    assert model.body_subtreemass[base] == pytest.approx(20.0)
    assert world.base_pose()[2] == pytest.approx(DATASHEET_WHEEL_RADIUS + WHEEL_DROP, abs=1e-3)
    assert world.robot.scan_site and world.scan_site >= 0


def test_scout_mini_drives_straight_and_spins():
    """Equal wheel speeds drive straight; opposite ones spin it in place."""
    world = scene.make_world('scout_mini', pose=(1.0, 2.0, 0.3))
    _drive(world, 0.5, 0.0)
    x0, y0, z0, yaw0 = world.base_pose()
    # Equal wheel speeds: straight along the heading.
    _drive(world, 3.0, 0.5)
    x1, y1, _, yaw1 = world.base_pose()
    assert abs(yaw1 - yaw0) < 0.01
    travelled = math.hypot(x1 - x0, y1 - y0)
    assert travelled == pytest.approx(1.5, rel=0.05)
    assert math.atan2(y1 - y0, x1 - x0) == pytest.approx(0.3, abs=0.01)
    # Opposite wheel speeds: a spin in place, slower than commanded (skid-steer slip). With
    # wheelbase and track about equal the wheels skid hard and the rate jitters: check its mean.
    world.reset()
    _drive(world, 0.5, 0.0)
    start = np.array(world.base_pose())
    _drive(world, 3.0, 0.0, omega=1.0)
    end = np.array(world.base_pose())
    assert math.hypot(*(end[:2] - start[:2])) < 0.03
    assert 0.05 < (end[3] - start[3]) / 3.0 < 1.0
    assert world.base_pose()[2] == pytest.approx(z0, abs=2e-3)


def test_ackermann_car_geometry():
    """The car matches the Ackermann overlay: base_link on the rear axle, enough steering."""
    world = scene.make_world('ackermann_car')
    g = world.geometry
    assert g['radius'] == pytest.approx(0.05)
    assert g['wheelbase'] == pytest.approx(0.26)
    assert g['track'] == pytest.approx(0.22)
    # The steering reaches the inner-wheel angle at the controller's minimum turning radius.
    assert g['max_steer'] >= math.atan(g['wheelbase'] / (MIN_TURNING_RADIUS - g['track'] / 2))
    assert g['max_steer'] - math.atan(g['wheelbase'] / MIN_TURNING_RADIUS) < 0.2
    # base_link is the rear-axle centre (reference_point_offset: 0.0).
    rear = world.model.body(world.name('rear_left_wheel_link')).id
    assert world.data.xpos[rear][0] == pytest.approx(world.base_pose()[0])


def test_ackermann_car_turns_at_the_commanded_radius():
    """The car drives straight, then circles at the commanded radius."""
    world = scene.make_world('ackermann_car')
    _drive(world, 1.0, 0.4)
    lin, ang = world.base_twist()
    assert lin[0] == pytest.approx(0.4, abs=0.01) and abs(ang[2]) < 0.01
    _drive(world, 6.0, 0.4, omega=0.4 / 0.7)
    lin, ang = world.base_twist()
    # The rear-axle centre moves without side slip, on (nearly) the commanded circle.
    assert abs(lin[1]) < 0.01
    assert lin[0] / ang[2] == pytest.approx(0.7, rel=0.05)


@pytest.mark.parametrize('robot', ['scout_mini', 'ackermann_car'])
def test_skidding_tyres_stay_on_the_floor(robot):
    """Tight, fast turns make the tyres skid; all of them keep touching the floor (no hop)."""
    from vectorfield_mujoco import simulation

    world = scene.make_world(robot)
    sim = simulation.Simulation(world)
    model, data = world.model, world.data
    tyres = [
        g
        for g in range(model.ngeom)
        if model.geom(g).name.endswith('_wheel') and model.geom_group[g] == 3
    ]
    assert len(tyres) == 4
    ticks = touching_all = 0
    for v, omega in ((0.6, 2.0), (1.0, 3.0)):
        for _ in range(300):
            sim.set_command(v, 0.0, omega)
            sim.step()
            touching = {c.geom1 for c in data.contact[: data.ncon]}
            touching |= {c.geom2 for c in data.contact[: data.ncon]}
            ticks += 1
            touching_all += all(t in touching for t in tyres)
    assert touching_all / ticks > 0.9


def test_robot_geoms_avoid_the_world_group():
    """No robot geom is in the lidar's world group."""
    for key in ('scout_mini', 'ackermann_car'):
        model = scene.make_world(key).model
        robot = [
            g
            for g in range(model.ngeom)
            if model.body(model.geom_bodyid[g]).name.startswith('robot/')
        ]
        assert robot and all(model.geom_group[g] != scene.WORLD_GROUP for g in robot)


def test_registry_entries():
    """The registry lists the five robots, each with a source and a lidar or none."""
    assert set(registry.ROBOTS) == {'tiago', 'scout_mini', 'ackermann_car', 'tidybot', 'x2'}
    for robot in registry.ROBOTS.values():
        assert robot.source in ('menagerie', 'package')
        if robot.adapter != 'quadrotor':
            assert robot.scan_site
    with pytest.raises(KeyError):
        registry.get('hovercraft')
