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

"""Unit tests of vectorfield_mujoco.adapters: the body-twist kinematics and their inverses."""

import math

import numpy as np
import pytest

from vectorfield_mujoco import adapters

TWISTS = [(0.4, 0.0), (0.3, 0.6), (-0.2, 0.4), (0.0, 1.0), (0.25, -1.2)]


@pytest.mark.parametrize('v,omega', TWISTS)
def test_diff_drive_round_trip(v, omega):
    """Wheel speeds map back to the commanded twist."""
    left, right = adapters.diff_drive(v, omega, 0.0983, 0.4034)
    assert adapters.diff_drive_inverse(left, right, 0.0983, 0.4034) == pytest.approx((v, omega))


def test_diff_drive_values():
    """Wheel speeds are (v ∓ ω·track/2)/r."""
    # (v ∓ ω·track/2)/r
    assert adapters.diff_drive(0.5, 1.0, 0.1, 0.4) == pytest.approx((3.0, 7.0))


@pytest.mark.parametrize('v,omega', TWISTS)
def test_skid_steer_pairs(v, omega):
    """Each side turns its two wheels at the differential-drive speed of that side."""
    fl, rl, fr, rr = adapters.skid_steer(v, omega, 0.08, 0.5164)
    assert fl == rl and fr == rr
    assert adapters.diff_drive_inverse(fl, fr, 0.08, 0.5164) == pytest.approx((v, omega))


@pytest.mark.parametrize('v,omega', [(0.4, 0.8), (0.4, -0.8), (0.3, 0.2), (-0.3, 0.5)])
def test_ackermann_wheels_share_the_turn_centre(v, omega):
    """Both front wheels point at the rear-axle centre of rotation (Ackermann)."""
    wheelbase, track, radius = 0.26, 0.22, 0.05
    sl, sr, rl, rr = adapters.ackermann(v, omega, wheelbase, track, radius)
    assert adapters.ackermann_inverse(rl, rr, radius, track) == pytest.approx((v, omega))
    # Each front wheel's normal passes through the centre of rotation (0, R) of the rear axle.
    turn_radius = v / omega
    for steer, y in ((sl, 0.5 * track), (sr, -0.5 * track)):
        assert math.tan(steer) == pytest.approx(wheelbase / (turn_radius - y))
    # The inner wheel steers more than the bicycle angle, the outer one less.
    bicycle = adapters.bicycle_steer(v, omega, wheelbase)
    inner, outer = (sl, sr) if omega * v > 0 else (sr, sl)
    assert abs(inner) > abs(bicycle) > abs(outer)


def test_ackermann_straight_and_stopped():
    """Straight ahead the wheels are straight; at v = 0 nothing moves."""
    assert adapters.ackermann(0.5, 0.0, 0.26, 0.22, 0.05) == pytest.approx((0, 0, 10, 10))
    # A car cannot turn in place: with v = 0 everything stays still.
    assert adapters.ackermann(0.0, 1.0, 0.26, 0.22, 0.05) == (0.0, 0.0, 0.0, 0.0)


def test_ackermann_clips_the_steering():
    """Steering angles are clipped to max_steer, each wheel on its own."""
    sl, sr, _, _ = adapters.ackermann(0.3, 1.0, 0.26, 0.22, 0.05, max_steer=0.65)
    assert sl == pytest.approx(0.65) and sr == pytest.approx(math.atan(0.26 / 0.41))


def test_ackermann_steer_track():
    """The steering geometry uses the pivot spacing, the rear wheels the track."""
    # Pivots closer together than the rear wheels: smaller inner/outer split.
    wide = adapters.ackermann(0.4, 0.8, 0.26, 0.22, 0.05)
    narrow = adapters.ackermann(0.4, 0.8, 0.26, 0.22, 0.05, steer_track=0.1)
    assert narrow[0] - narrow[1] < wide[0] - wide[1]
    assert narrow[2:] == wide[2:]


@pytest.mark.parametrize('yaw', [0.0, 0.7, -2.5, math.pi])
def test_holonomic_round_trip(yaw):
    """Body twist to world velocities and back, at any heading."""
    twist = (0.3, -0.2, 0.5)
    world = adapters.holonomic(*twist, yaw)
    assert np.hypot(world[0], world[1]) == pytest.approx(np.hypot(0.3, -0.2))
    assert adapters.holonomic_inverse(*world, yaw) == pytest.approx(twist)


def test_holonomic_rotates_into_the_world():
    """Forward at a heading of 90° is +y in the world."""
    assert adapters.holonomic(1.0, 0.0, 0.0, math.pi / 2) == pytest.approx((0.0, 1.0, 0.0))


class _Data:
    def __init__(self, nu):
        self.ctrl = np.zeros(nu)


def test_adapter_writes_the_actuators():
    """Adapter.apply() writes each role into its actuator."""
    ids = {'front_left': 3, 'rear_left': 0, 'front_right': 1, 'rear_right': 2}
    adapter = adapters.Adapter('skid_steer', ids, {'radius': 0.1, 'track': 0.4})
    data = _Data(4)
    adapter.apply(data, 0.5, 0.0, 1.0)
    assert data.ctrl[[3, 0, 1, 2]] == pytest.approx([3.0, 3.0, 7.0, 7.0])
    holo = adapters.Adapter('holonomic', {'x': 0, 'y': 1, 'theta': 2}, {})
    data = _Data(3)
    holo.apply(data, 1.0, 0.0, 0.2, yaw=math.pi / 2)
    assert data.ctrl == pytest.approx([0.0, 1.0, 0.2])
    with pytest.raises(ValueError):
        adapters.Adapter('tank', {}, {})
