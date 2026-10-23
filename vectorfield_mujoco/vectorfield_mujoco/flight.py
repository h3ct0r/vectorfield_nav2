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

"""A cascaded quadrotor velocity controller, standing in for PX4/ArduPilot velocity mode.

The 3D follower commands a world-frame velocity and a yaw rate, as it would to an autopilot in
offboard velocity mode. :class:`VelocityController` closes that loop on the MuJoCo model at the
physics rate:

1. velocity error (PI) → desired acceleration, clipped, plus gravity → desired force;
2. force direction (tilt-limited) and the integrated yaw set-point → desired attitude R_d;
   collective thrust = desired force projected on the current body z;
3. SO(3) PD on the attitude error (Lee, Leok and McClamroch, CDC 2010) → body torque;
4. mixer: the inverse of the actuator wrench map, built from the rotor sites and the actuator
   gears (thrust along the site z, yaw torque ``gear[5]`` per newton) read from the model;
   thrusts are clipped to the actuators' ``ctrlrange``.

Only the mixer touches MuJoCo (:meth:`QuadrotorModel.from_mujoco`); the control law is numpy.
"""

import dataclasses
import math

import numpy as np

GRAVITY = 9.81


def _vee(m):
    return np.array([m[2, 1], m[0, 2], m[1, 0]])


def _yaw(rot):
    return math.atan2(rot[1, 0], rot[0, 0])


@dataclasses.dataclass
class QuadrotorModel:
    """Rigid-body and actuator parameters of a quadrotor.

    Attributes:
        mass: Total mass (kg).
        inertia: 3x3 inertia about the centre of mass in the body frame (kg m²).
        wrench_map: 4xN map from actuator controls to [thrust_z (N), τx, τy, τz (N m)].
        ctrl_min, ctrl_max: Actuator control limits (N).
        gravity: Gravity magnitude (m/s²).
        actuator_ids: The model's actuator indices, in ``wrench_map`` column order.
    """

    mass: float
    inertia: np.ndarray
    wrench_map: np.ndarray
    ctrl_min: np.ndarray
    ctrl_max: np.ndarray
    gravity: float = GRAVITY
    actuator_ids: tuple = ()

    @classmethod
    def from_mujoco(cls, model, body, actuators=None):
        """Read the parameters of the free body ``body`` and its site actuators from a model.

        Args:
            model: The compiled ``mujoco.MjModel``.
            body: Body name or id (the one with the free joint).
            actuators: Actuator names or ids; default every site actuator on the body subtree.
        """
        import mujoco

        bid = body if isinstance(body, int) else model.body(body).id
        if actuators is None:
            actuators = [
                i
                for i in range(model.nu)
                if model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_SITE
                and _in_subtree(model, model.site_bodyid[model.actuator_trnid[i, 0]], bid)
            ]
        ids = [a if isinstance(a, int) else model.actuator(a).id for a in actuators]
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        rot_b = data.xmat[bid].reshape(3, 3)
        com = data.subtree_com[bid]
        wrench = np.zeros((4, len(ids)))
        for k, a in enumerate(ids):
            site = model.actuator_trnid[a, 0]
            rot_s = rot_b.T @ data.site_xmat[site].reshape(3, 3)  # site in body frame
            gear = model.actuator_gear[a]
            force = rot_s @ gear[:3]
            arm = rot_b.T @ (data.site_xpos[site] - com)
            torque = np.cross(arm, force) + rot_s @ gear[3:]
            wrench[:, k] = (force[2], *torque)
        # Inertia of the subtree about its centre of mass, in the body frame.
        inertia = np.zeros((3, 3))
        for b in range(model.nbody):
            if not _in_subtree(model, b, bid):
                continue
            rot_i = data.ximat[b].reshape(3, 3)
            local = rot_i @ np.diag(model.body_inertia[b]) @ rot_i.T
            r = data.xipos[b] - com
            local += model.body_mass[b] * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
            inertia += local
        inertia = rot_b.T @ inertia @ rot_b
        gravity = float(np.linalg.norm(model.opt.gravity)) or GRAVITY
        return cls(
            mass=float(model.body_subtreemass[bid]),
            inertia=inertia,
            wrench_map=wrench,
            ctrl_min=model.actuator_ctrlrange[ids, 0].copy(),
            ctrl_max=model.actuator_ctrlrange[ids, 1].copy(),
            gravity=gravity,
            actuator_ids=tuple(ids),
        )

    def hover_thrusts(self):
        """Return the actuator controls that hold the body level and still."""
        return np.linalg.pinv(self.wrench_map) @ np.array([self.mass * self.gravity, 0, 0, 0])


def _in_subtree(model, body, root):
    while body > 0:
        if body == root:
            return True
        body = model.body_parentid[body]
    return root == 0


@dataclasses.dataclass
class Gains:
    """Controller gains and limits (defaults suit a 1-2 kg quadrotor such as the Skydio X2)."""

    vel_p: float = 3.0
    vel_i: float = 0.8
    vel_i_limit: float = 2.0
    max_accel_xy: float = 5.0
    max_accel_up: float = 6.0
    max_accel_down: float = 5.0
    max_tilt: float = math.radians(35.0)
    att_freq: float = 20.0
    att_damping: float = 0.9
    yaw_freq: float = 8.0


class VelocityController:
    """World velocity + yaw-rate set-point → actuator thrusts (see the module docstring)."""

    def __init__(self, quad, gains=None):
        self.quad = quad
        self.gains = gains or Gains()
        self._mixer = np.linalg.pinv(quad.wrench_map)
        self.reset()

    def reset(self, yaw=None):
        """Clear the integrator; hold ``yaw`` (or the yaw at the next update)."""
        self._vel_int = np.zeros(3)
        self._yaw_sp = yaw

    def update(self, dt, rot, vel, omega, vel_sp, yaw_rate_sp=0.0):
        """Return the actuator controls.

        Args:
            dt: Time since the last update (s).
            rot: Body-to-world rotation matrix (3x3).
            vel: World-frame linear velocity (m/s).
            omega: Body-frame angular velocity (rad/s).
            vel_sp: World-frame velocity set-point (m/s).
            yaw_rate_sp: Yaw-rate set-point (rad/s).
        """
        g, q = self.gains, self.quad
        rot = np.asarray(rot, dtype=float).reshape(3, 3)
        omega = np.asarray(omega, dtype=float)
        err = np.asarray(vel_sp, dtype=float) - np.asarray(vel, dtype=float)
        self._vel_int = np.clip(self._vel_int + err * dt, -g.vel_i_limit, g.vel_i_limit)
        acc = g.vel_p * err + g.vel_i * self._vel_int
        horizontal = np.linalg.norm(acc[:2])
        if horizontal > g.max_accel_xy:
            acc[:2] *= g.max_accel_xy / horizontal
        acc[2] = min(max(acc[2], -g.max_accel_down), g.max_accel_up)
        force = q.mass * (acc + np.array([0.0, 0.0, q.gravity]))
        # Tilt limit: keep the vertical component, shorten the horizontal one.
        max_h = force[2] * math.tan(g.max_tilt)
        h = np.linalg.norm(force[:2])
        if force[2] <= 0.0:
            force = np.array([0.0, 0.0, 1e-3])
        elif h > max_h:
            force[:2] *= max_h / h
        b3 = force / np.linalg.norm(force)

        yaw = _yaw(rot)
        if self._yaw_sp is None:
            self._yaw_sp = yaw
        self._yaw_sp += yaw_rate_sp * dt
        # Do not wind the yaw set-point up beyond what the vehicle follows.
        lag = math.remainder(self._yaw_sp - yaw, 2.0 * math.pi)
        if abs(lag) > 0.5:
            self._yaw_sp = yaw + math.copysign(0.5, lag)
        b1c = np.array([math.cos(self._yaw_sp), math.sin(self._yaw_sp), 0.0])
        b2 = np.cross(b3, b1c)
        b2 /= np.linalg.norm(b2)
        rot_d = np.column_stack((np.cross(b2, b3), b2, b3))

        thrust = float(force @ rot[:, 2])
        e_r = 0.5 * _vee(rot_d.T @ rot - rot.T @ rot_d)
        omega_d = rot.T @ rot_d @ np.array([0.0, 0.0, yaw_rate_sp])
        e_w = omega - omega_d
        wn = np.array([g.att_freq, g.att_freq, g.yaw_freq])
        accel = -(wn**2) * e_r - 2.0 * g.att_damping * wn * e_w
        torque = q.inertia @ accel + np.cross(omega, q.inertia @ omega)
        return self.mix(thrust, torque)

    def mix(self, thrust, torque):
        """Return actuator controls for a collective thrust and a body torque, within limits.

        Priorities, as autopilot mixers do: the collective thrust first (clipped to what the
        rotors give), then as much of the roll/pitch torque as fits, then as much yaw torque as
        still fits. Each is scaled down along its own direction, so a saturated command keeps
        its sense instead of being clipped rotor by rotor.
        """
        q = self.quad
        lo, hi = q.ctrl_min, q.ctrl_max
        base = self._mixer @ np.array([thrust, 0.0, 0.0, 0.0])
        if np.any(base > hi) or np.any(base < lo):
            base = np.clip(base, lo, hi)
        roll_pitch = self._mixer @ np.array([0.0, torque[0], torque[1], 0.0])
        base = base + _fit(base, roll_pitch, lo, hi) * roll_pitch
        yaw = self._mixer @ np.array([0.0, 0.0, 0.0, torque[2]])
        u = base + _fit(base, yaw, lo, hi) * yaw
        return np.clip(u, lo, hi)


def _fit(base, step, lo, hi):
    """Return the largest s in [0, 1] keeping lo <= base + s * step <= hi."""
    s = 1.0
    for b, d, low, high in zip(base, step, lo, hi):
        if d > 1e-12:
            s = min(s, (high - b) / d)
        elif d < -1e-12:
            s = min(s, (low - b) / d)
    return max(0.0, s)


class MujocoQuadrotor:
    """:class:`VelocityController` bound to a free body and its thrust actuators in a model."""

    def __init__(self, model, body, actuators=None, gains=None):
        import mujoco

        quad = QuadrotorModel.from_mujoco(model, body, actuators)
        self.actuator_ids = np.array(quad.actuator_ids, dtype=int)
        self.body_id = body if isinstance(body, int) else model.body(body).id
        joint = model.body_jntadr[self.body_id]
        if joint < 0 or model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError('the quadrotor body needs a free joint')
        self._dof = model.jnt_dofadr[joint]
        self.controller = VelocityController(quad, gains)

    def apply(self, model, data, vel_sp, yaw_rate_sp=0.0):
        """Run one controller step and write the thrusts into ``data.ctrl``."""
        rot = data.xmat[self.body_id].reshape(3, 3)
        vel = data.qvel[self._dof : self._dof + 3]
        omega = data.qvel[self._dof + 3 : self._dof + 6]
        u = self.controller.update(model.opt.timestep, rot, vel, omega, vel_sp, yaw_rate_sp)
        data.ctrl[self.actuator_ids] = u
        return u
