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

"""The simulator node's ROS-free core: a world stepped in ticks, with vectorfield_sim's frames.

:class:`Simulation` steps a :class:`vectorfield_mujoco.scene.World` in *ticks* of a whole number
of physics steps (the node publishes ``/clock`` once per tick) and keeps the frames of
vectorfield_sim:

- the MuJoCo world frame is ``map``, and the base's pose in it is the ground truth;
- odometry starts at the identity: the *true* ``map → odom`` is the initial pose, and the odom
  pose is the ground truth seen from it, so it carries every bit of the physics' slip;
- the *published* ``map → odom`` (the localization estimate) equals the true one, except after
  :meth:`Simulation.step_map_to_odom` (a localization jump that leaves the robot in place);
- :meth:`Simulation.set_initial_pose` (``initialpose``) teleports the robot and moves both
  ``map → odom`` transforms so that the odometry stays continuous.

Poses are 2D (x, y, yaw) tuples; :func:`compose` and :func:`inverse` are their group operations.
The module also parses the node's obstacle and size parameters.
"""

import math

import numpy as np
import yaml

from vectorfield_mujoco import scene

#: Poses this close (m, rad) to the current one are not teleported to (only re-anchored), so
#: the initial pose publisher's first message does not disturb a robot that just settled.
TELEPORT_TOLERANCE = (1e-3, 1e-3)


def wrap(angle):
    """Return ``angle`` wrapped to [-π, π)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def compose(a, b):
    """Return the 2D pose ``a ∘ b`` (``b`` expressed in the frame of ``a``)."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return ax + c * bx - s * by, ay + s * bx + c * by, wrap(ayaw + byaw)


def inverse(a):
    """Return the inverse of the 2D pose ``a``."""
    x, y, yaw = a
    c, s = math.cos(yaw), math.sin(yaw)
    return -c * x - s * y, s * x - c * y, wrap(-yaw)


def steps_per_period(period, step):
    """Return the whole number of ``step`` s nearest to ``period`` s (at least 1).

    The same rounding as vectorfield_sim's ``stepsPerPeriod``.
    """
    if step <= 0.0 or period <= step:
        return 1
    return max(1, int(math.floor(period / step + 0.5)))


def parse_obstacles(value):
    """Return the obstacle dicts of the ``obstacles`` parameter, validated.

    Args:
        value: A YAML (or JSON) list of obstacles, ``[{type: box, pos: [x, y], size: [hx, hy,
            hz]}, ...]`` as :func:`vectorfield_mujoco.scene.obstacle_geom` reads them, a list
            already parsed, or ``''`` for none.

    Raises:
        ValueError: The value is not a list of valid obstacles.

    """
    obstacles = yaml.safe_load(value) if isinstance(value, str) else value
    if obstacles is None:
        return []
    if not isinstance(obstacles, list) or not all(isinstance(o, dict) for o in obstacles):
        raise ValueError(f'obstacles must be a list of mappings, not {value!r}')
    for i, obstacle in enumerate(obstacles):
        try:
            scene.obstacle_geom(obstacle)
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f'obstacle {i} ({obstacle}): {e}') from e
    return obstacles


def obstacle_span(obstacle):
    """Return the (bottom, top) heights (m) of an obstacle dict."""
    kind, size, pos, _ = scene.obstacle_geom(obstacle)
    half = {'box': size[2], 'cylinder': size[1], 'sphere': size[0]}[kind]
    return pos[2] - half, pos[2] + half


def unseen_obstacles(obstacles, scan_height):
    """Return the indices of the obstacles the lidar's plane at ``scan_height`` (m) misses."""
    unseen = []
    for i, obstacle in enumerate(obstacles):
        bottom, top = obstacle_span(obstacle)
        if not bottom < scan_height < top:
            unseen.append(i)
    return unseen


def parse_size(value):
    """Return (width, height) of a ``WIDTHxHEIGHT`` string such as ``960x540`` (even numbers)."""
    try:
        width, height = (int(v) for v in str(value).lower().split('x'))
    except ValueError:
        raise ValueError(f'size must be WIDTHxHEIGHT, not {value!r}') from None
    if width <= 0 or height <= 0 or width % 2 or height % 2:
        raise ValueError(f'size must be positive and even, not {value!r}')
    return width, height


class Simulation:
    """A world stepped in ticks, with a command timeout and vectorfield_sim's frames.

    Args:
        world: A :class:`vectorfield_mujoco.scene.World` at its initial pose.
        physics_rate: Physics steps per simulated second (sets the model's timestep).
        tick_rate: Ticks per simulated second; each tick is a whole number of physics steps.
        cmd_timeout: A command older than this (simulated s) is replaced by zero; 0 disables it.

    """

    def __init__(self, world, physics_rate=500.0, tick_rate=100.0, cmd_timeout=0.5):
        if physics_rate <= 0.0 or tick_rate <= 0.0:
            raise ValueError('physics_rate and tick_rate must be positive')
        self.world = world
        world.model.opt.timestep = 1.0 / physics_rate
        self.substeps = steps_per_period(1.0 / tick_rate, world.model.opt.timestep)
        self.tick_period = self.substeps * world.model.opt.timestep
        self.cmd_timeout = cmd_timeout
        self.ticks = 0
        self.infeasible_count = 0
        self._command = (0.0, 0.0, 0.0)
        self._command_time = None
        self._true_map_to_odom = self.truth()
        self._map_to_odom = self._true_map_to_odom

    @property
    def time(self):
        """Return the simulated time (s)."""
        return float(self.world.data.time)

    def set_command(self, vx, vy, omega):
        """Set the body twist command; return whether it is infeasible (Ackermann v = 0, ω ≠ 0).

        Non-finite commands raise ``ValueError``. An infeasible command is still applied (the
        car's wheels stay straight and still) and counted in :attr:`infeasible_count`.
        """
        if not all(math.isfinite(v) for v in (vx, vy, omega)):
            raise ValueError('non-finite command')
        self._command = (float(vx), float(vy), float(omega))
        self._command_time = self.time
        infeasible = self.world.robot.adapter == 'ackermann' and vx == 0.0 and omega != 0.0
        if infeasible:
            self.infeasible_count += 1
        return infeasible

    def command(self):
        """Return the command in effect: zero once it is older than the timeout."""
        if self._command_time is None:
            return 0.0, 0.0, 0.0
        if self.cmd_timeout > 0.0 and self.time - self._command_time > self.cmd_timeout:
            return 0.0, 0.0, 0.0
        return self._command

    def step(self, on_substep=None):
        """Advance one tick: the command is applied before every physics step.

        Args:
            on_substep: Optional callable ``on_substep(data)`` after each physics step (the
                recorder captures its frames there, in simulated time).

        """
        import mujoco

        world = self.world
        for _ in range(self.substeps):
            world.drive(*self.command())
            mujoco.mj_step(world.model, world.data)
            if on_substep is not None:
                on_substep(world.data)
        # mj_step leaves the poses and velocities of the step's start; recompute them for the
        # new state, which everything published after the tick reads.
        mujoco.mj_forward(world.model, world.data)
        self.ticks += 1

    def truth(self):
        """Return the true base pose (x, y, yaw) in the map frame."""
        x, y, _, yaw = self.world.base_pose()
        return x, y, yaw

    def odom_pose(self):
        """Return the base pose in the odom frame."""
        return compose(inverse(self._true_map_to_odom), self.truth())

    def map_to_odom(self):
        """Return the published map → odom transform."""
        return self._map_to_odom

    def body_twist(self):
        """Return the base's (vx, vy, omega) in its own frame."""
        linear, angular = self.world.base_twist()
        return float(linear[0]), float(linear[1]), float(angular[2])

    def set_initial_pose(self, x, y, yaw):
        """Teleport the robot to the map pose (x, y, yaw) at rest; the odometry stays continuous.

        Returns whether the robot was moved: a pose within :data:`TELEPORT_TOLERANCE` of the
        current one only re-anchors map → odom.
        """
        current = self.truth()
        moved = (
            math.hypot(x - current[0], y - current[1]) > TELEPORT_TOLERANCE[0]
            or abs(wrap(yaw - current[2])) > TELEPORT_TOLERANCE[1]
        )
        odom = self.odom_pose()
        if moved:
            teleport(self.world, x, y, yaw)
        self._true_map_to_odom = compose(self.truth(), inverse(odom))
        self._map_to_odom = self._true_map_to_odom
        return moved

    def step_map_to_odom(self, dx, dy, dyaw):
        """Add (dx, dy, dyaw) to the published map → odom only (a localization jump)."""
        x, y, yaw = self._map_to_odom
        self._map_to_odom = (x + dx, y + dy, wrap(yaw + dyaw))


def teleport(world, x, y, yaw):
    """Move the robot's base to the map pose (x, y, yaw), at rest, keeping its height.

    A free-floating robot is moved through the free joint of its root body (rotated about the
    base, so a root other than the base follows); a planar base through its x, y, yaw joints.
    """
    import mujoco

    model, data = world.model, world.data
    mujoco.mj_forward(model, data)
    x0, y0, _, yaw0 = world.base_pose()
    if world.robot.planar_joints:
        for joint, value, current in zip(world.robot.planar_joints, (x, y, yaw), (x0, y0, yaw0)):
            jid = model.joint(world.name(joint)).id
            delta = (
                wrap(value - current) if joint == world.robot.planar_joints[2] else value - current
            )
            data.qpos[model.jnt_qposadr[jid]] += delta
            data.qvel[model.jnt_dofadr[jid]] = 0.0
    else:
        root = model.body_rootid[world.base_id]
        jid = model.body_jntadr[root]
        if jid < 0 or model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError(f'{world.robot.key}: the root body has no free joint')
        adr = model.jnt_qposadr[jid]
        dyaw = wrap(yaw - yaw0)
        c, s = math.cos(dyaw), math.sin(dyaw)
        base = data.xpos[world.base_id]
        rel = data.qpos[adr : adr + 3] - base
        data.qpos[adr] = x + c * rel[0] - s * rel[1]
        data.qpos[adr + 1] = y + s * rel[0] + c * rel[1]
        turn = np.array([math.cos(0.5 * dyaw), 0.0, 0.0, math.sin(0.5 * dyaw)])
        quat = np.zeros(4)
        mujoco.mju_mulQuat(quat, turn, data.qpos[adr + 3 : adr + 7].copy())
        data.qpos[adr + 3 : adr + 7] = quat
        # The whole robot comes to rest (wheels and casters too).
        first = model.body_dofadr[root]
        count = sum(
            model.body_dofnum[b] for b in range(model.nbody) if model.body_rootid[b] == root
        )
        data.qvel[first : first + count] = 0.0
    mujoco.mj_forward(model, data)
