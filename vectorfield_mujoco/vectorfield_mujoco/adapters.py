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

"""Body twist to actuator commands for the ground robots.

The functions are pure kinematics (no MuJoCo): a body twist ``(v_x, v_y, omega)`` in, the
actuator set-points out, with the inverse where one exists (used by the tests). Wheel speeds are
in rad/s, positive forward; ``track`` is the lateral distance between the left and right wheel
contacts and ``radius`` the wheel radius, both read from the compiled model by
:mod:`vectorfield_mujoco.registry`.

:class:`Adapter` binds one of them to the actuators of a compiled model.
"""

import math

import numpy as np

# A speed at or below this is zero (m/s), as vectorfield_sim's kZeroVelocity: an Ackermann
# command with |v| <= ZERO_SPEED and |ω| > ZERO_SPEED is infeasible.
ZERO_SPEED = 1e-6


def diff_drive(v, omega, radius, track):
    """Return the (left, right) wheel speeds of a differential drive: (v ∓ ω·track/2)/r."""
    half = 0.5 * track
    return (v - omega * half) / radius, (v + omega * half) / radius


def diff_drive_inverse(left, right, radius, track):
    """Return the body twist (v, omega) of the wheel speeds (left, right)."""
    return 0.5 * radius * (left + right), radius * (right - left) / track


def skid_steer(v, omega, radius, track):
    """Return the (front_left, rear_left, front_right, rear_right) wheel speeds.

    Each side's pair turns at the differential-drive speed of that side. The slip of the real
    turn is the physics' business: the commanded yaw rate assumes the geometric track, and the
    controller's slip compensation (``skid_yaw_rate_efficiency``) accounts for the rest.
    """
    left, right = diff_drive(v, omega, radius, track)
    return left, left, right, right


def ackermann(v, omega, wheelbase, track, radius, max_steer=None, steer_track=None):
    """Return (steer_left, steer_right, rear_left, rear_right) of a front-steered car.

    The bicycle model at the rear axle gives the turn radius R = v/ω of the rear-axle centre;
    each front wheel at lateral offset y steers to atan(L·ω / (v − ω·y)) so that its normal
    passes through the centre of rotation (inner wheel steeper), and each rear wheel turns at
    (v − ω·y)/r (an electronic differential). Angles are clipped to ``max_steer`` if given.
    ``steer_track`` is the distance between the steering pivots (default ``track``). With
    v = 0 the car cannot turn: the wheels stay straight and still.
    """
    if abs(v) <= ZERO_SPEED:
        return 0.0, 0.0, 0.0, 0.0
    half = 0.5 * track
    pivot = 0.5 * (track if steer_track is None else steer_track)
    steer = []
    for y in (pivot, -pivot):
        denom = v - omega * y
        angle = (
            math.atan(wheelbase * omega / denom)
            if abs(denom) > 1e-9
            else math.copysign(0.5 * math.pi, omega * v)
        )
        if max_steer is not None:
            angle = min(max(angle, -max_steer), max_steer)
        steer.append(angle)
    return steer[0], steer[1], (v - omega * half) / radius, (v + omega * half) / radius


def ackermann_inverse(rear_left, rear_right, radius, track):
    """Return the rear-axle twist (v, omega) of the rear wheel speeds."""
    return diff_drive_inverse(rear_left, rear_right, radius, track)


def bicycle_steer(v, omega, wheelbase):
    """Return the bicycle-model steering angle atan(L·ω/v) (0 when v = 0)."""
    return 0.0 if abs(v) <= ZERO_SPEED else math.atan(wheelbase * omega / v)


def holonomic(vx, vy, omega, yaw):
    """Return the world-frame planar velocities (ẋ, ẏ, θ̇) of a body twist at heading ``yaw``."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx - s * vy, s * vx + c * vy, omega


def holonomic_inverse(xdot, ydot, thetadot, yaw):
    """Return the body twist (vx, vy, omega) of world-frame planar velocities."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * xdot + s * ydot, -s * xdot + c * ydot, thetadot


#: Adapter kinds and their actuator roles, in the order the kinematics function returns them.
ROLES = {
    'diff_drive': ('left', 'right'),
    'skid_steer': ('front_left', 'rear_left', 'front_right', 'rear_right'),
    'ackermann': ('steer_left', 'steer_right', 'rear_left', 'rear_right'),
    'holonomic': ('x', 'y', 'theta'),
}


class Adapter:
    """Writes a body twist into the actuators of a compiled model.

    Args:
        kind: One of :data:`ROLES`.
        actuator_ids: Actuator index per role (``ROLES[kind]``).
        geometry: The model geometry from :func:`vectorfield_mujoco.registry.read_geometry`
            (``radius``, ``track``, ``wheelbase``, ``max_steer``).
    """

    def __init__(self, kind, actuator_ids, geometry):
        if kind not in ROLES:
            raise ValueError(f'unknown adapter kind {kind!r}')
        self.kind = kind
        self.ids = np.array([actuator_ids[r] for r in ROLES[kind]], dtype=int)
        self.geometry = dict(geometry)

    def command(self, vx, vy, omega, yaw=0.0):
        """Return the actuator set-points (in ``ROLES[kind]`` order) for a body twist."""
        g = self.geometry
        if self.kind == 'diff_drive':
            return diff_drive(vx, omega, g['radius'], g['track'])
        if self.kind == 'skid_steer':
            return skid_steer(vx, omega, g['radius'], g['track'])
        if self.kind == 'ackermann':
            return ackermann(
                vx,
                omega,
                g['wheelbase'],
                g['track'],
                g['radius'],
                g.get('max_steer'),
                g.get('steer_track'),
            )
        return holonomic(vx, vy, omega, yaw)

    def apply(self, data, vx, vy, omega, yaw=0.0):
        """Set ``data.ctrl`` for a body twist (``yaw`` is used by the holonomic base only)."""
        data.ctrl[self.ids] = self.command(vx, vy, omega, yaw)
