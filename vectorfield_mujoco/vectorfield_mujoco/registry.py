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

"""The example robots: where each MJCF comes from and how it is driven.

:data:`ROBOTS` maps a robot key to a :class:`Robot`. :func:`load_spec` returns its
``mujoco.MjSpec``, edited where needed (TIAGo's upper body frozen, TidyBot's position servos
swapped for velocity servos); :func:`read_geometry` reads the wheel radius, track, wheelbase
and steering limit back from the *compiled* model, so nothing about the wheels is hard-coded
here; :func:`make_controller` binds the twist adapter or the flight controller.
"""

import dataclasses
import math
import pathlib

import numpy as np

from vectorfield_mujoco import adapters, menagerie

PACKAGE = 'vectorfield_mujoco'


@dataclasses.dataclass(frozen=True)
class Robot:
    """A registry entry.

    Attributes:
        key: The registry key.
        description: One line for logs and docs.
        source: ``'menagerie'`` (fetched, :mod:`vectorfield_mujoco.menagerie`) or ``'package'``
            (``models/`` of this package).
        mjcf: MJCF path relative to the Menagerie checkout or to ``models/``.
        license: Licence of the model files.
        base_body: The body whose pose is the robot pose (``base_link`` in ROS terms).
        adapter: ``'diff_drive'``, ``'skid_steer'``, ``'ackermann'``, ``'holonomic'`` or
            ``'quadrotor'``.
        mode: The vectorfield motion model it demonstrates (``motion_model`` or ``'3d'``).
        actuators: Actuator name per adapter role (:data:`vectorfield_mujoco.adapters.ROLES`).
        wheels: Wheel body name per role, for :func:`read_geometry`.
        steer_joints: The steering hinges (Ackermann).
        planar_joints: (x, y, yaw) joints of a base without a free joint, else empty.
        spawn_height: Height of the base origin above the floor at spawn; None keeps the MJCF's.
        scan_site: Site the 2D lidar scans from, or None (no lidar).
        option: ``mjOption`` fields the world needs for this robot.
        customize: Optional callable editing the loaded ``MjSpec`` in place.
    """

    key: str
    description: str
    source: str
    mjcf: str
    license: str
    base_body: str
    adapter: str
    mode: str
    actuators: dict = dataclasses.field(default_factory=dict)
    wheels: dict = dataclasses.field(default_factory=dict)
    steer_joints: tuple = ()
    planar_joints: tuple = ()
    spawn_height: float | None = None
    scan_site: str | None = None
    option: dict = dataclasses.field(default_factory=dict)
    customize: object = None


# ----------------------------------------------------------------------------- MjSpec helpers


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def _quat_mat(q):
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _axis_quat(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    return np.concatenate(([math.cos(0.5 * angle)], math.sin(0.5 * angle) * axis))


def _z_to(axis):
    """Return the quaternion rotating +z onto ``axis``."""
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    z = np.array([0.0, 0.0, 1.0])
    cross, dot = np.cross(z, axis), float(z @ axis)
    if np.linalg.norm(cross) < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0]) if dot > 0 else np.array([0.0, 1.0, 0.0, 0.0])
    q = np.concatenate(([1.0 + dot], cross))
    return q / np.linalg.norm(q)


def freeze_joints(spec, values):
    """Weld hinge and slide joints at fixed values.

    Each joint's motion at its value is folded into its body's pose, then the joint and every
    actuator acting on it are deleted. Keyframes become invalid and are deleted too.

    Args:
        spec: The ``MjSpec`` to edit.
        values: Joint name → value (rad or m).
    """
    import mujoco

    for name, value in values.items():
        joint = spec.joint(name)
        body = joint.parent
        rot = _quat_mat(body.quat)
        axis = np.asarray(joint.axis, dtype=float) / np.linalg.norm(joint.axis)
        if joint.type == mujoco.mjtJoint.mjJNT_HINGE:
            q = _axis_quat(axis, value)
            p = np.asarray(joint.pos, dtype=float)
            body.pos = np.asarray(body.pos) + rot @ (p - _quat_mat(q) @ p)
            body.quat = _quat_mul(np.asarray(body.quat) / np.linalg.norm(body.quat), q)
        elif joint.type == mujoco.mjtJoint.mjJNT_SLIDE:
            body.pos = np.asarray(body.pos) + rot @ (axis * value)
        else:
            raise ValueError(f'cannot freeze joint {name!r} of type {joint.type}')
        for actuator in list(spec.actuators):
            if actuator.target == name:
                spec.delete(actuator)
        spec.delete(joint)
    for key in list(spec.keys):
        spec.delete(key)


def absolutize_assets(spec):
    """Make mesh and texture file paths absolute, so the spec can be attached anywhere."""
    root = pathlib.Path(spec.modelfiledir or '.')
    for elements, folder in ((spec.meshes, spec.meshdir), (spec.textures, spec.texturedir)):
        for element in elements:
            if element.file and not pathlib.Path(element.file).is_absolute():
                element.file = str((root / (folder or '') / element.file).resolve())
    spec.meshdir = ''
    spec.texturedir = ''


def _wheel_cylinder(model, data, geom, axis_world):
    """Return (centre in body frame, radius, half width) of a wheel geom about ``axis_world``."""
    body = model.geom_bodyid[geom]
    rot_g = data.geom_xmat[geom].reshape(3, 3)
    rot_b = data.xmat[body].reshape(3, 3)
    centre_w = data.geom_xpos[geom] + rot_g @ model.geom_aabb[geom, :3]
    half = model.geom_aabb[geom, 3:]
    along = np.abs(rot_g.T @ axis_world)
    k = int(np.argmax(along))
    radius = float(max(half[i] for i in range(3) if i != k))
    return rot_b.T @ (centre_w - data.xpos[body]), radius, float(half[k])


# --------------------------------------------------------------------------- customizations

# TIAGo's upper body, frozen in pal_tiago's "home" keyframe pose (arm tucked, torso at 0.15 m).
_TIAGO_FROZEN = (
    'torso_lift_joint',
    'head_1_joint',
    'head_2_joint',
    'arm_1_joint',
    'arm_2_joint',
    'arm_3_joint',
    'arm_4_joint',
    'arm_5_joint',
    'arm_6_joint',
    'arm_7_joint',
    'gripper_left_finger_joint',
    'gripper_right_finger_joint',
)


def _customize_tiago(spec):
    """Freeze the upper body, give the wheels primitive contacts, add the base laser site."""
    import mujoco

    model = spec.compile()
    data = mujoco.MjData(model)
    key = model.key('home').id
    home = {
        n: float(model.key_qpos[key, model.jnt_qposadr[model.joint(n).id]]) for n in _TIAGO_FROZEN
    }
    mujoco.mj_forward(model, data)
    for side in ('left', 'right'):
        name = f'wheel_{side}_link'
        bid = model.body(name).id
        joint = model.body_jntadr[bid]
        axis = data.xaxis[joint]
        geom = next(g for g in range(model.ngeom) if model.geom_bodyid[g] == bid)
        centre, radius, half_width = _wheel_cylinder(model, data, geom, axis)
        rot_b = data.xmat[bid].reshape(3, 3)
        body = spec.body(name)
        # The wheel mesh stays as the visual. The contact is a single-point ellipsoid of the
        # wheel's size: a cylinder's two rim contacts scrub when the base turns in place and
        # cost about 35 % of the commanded yaw rate.
        for g in body.geoms:
            g.contype, g.conaffinity = 0, 0
        body.add_geom(
            name=f'wheel_{side}_contact',
            type=mujoco.mjtGeom.mjGEOM_ELLIPSOID,
            size=[radius, radius, half_width],
            pos=centre,
            quat=_z_to(rot_b.T @ axis),
            group=3,
            contype=0,
            conaffinity=1,
            mass=0.0,
        )
    freeze_joints(spec, home)
    # The base laser (SICK TiM551) mesh origin: 0.202 m ahead, 0.0945 m above the ground.
    spec.body('base_link').add_site(name='base_laser', pos=[0.202, 0.0, 0.0945], size=[0.01] * 3)


def _customize_tidybot(spec):
    """Swap the planar position servos for velocity servos and add a scan site."""
    import mujoco

    gains = {'joint_x': 1500.0, 'joint_y': 1500.0, 'joint_th': 300.0}
    ranges = {'joint_x': 1.5, 'joint_y': 1.5, 'joint_th': 3.0}
    for actuator in list(spec.actuators):
        spec.delete(actuator)
    for joint, kv in gains.items():
        actuator = spec.add_actuator(
            name=f'{joint}_vel',
            target=joint,
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            ctrlrange=[-ranges[joint], ranges[joint]],
        )
        actuator.set_to_velocity(kv=kv)
    spec.body('base_link').add_site(name='base_laser', pos=[0.0, 0.0, 0.3], size=[0.01] * 3)


# --------------------------------------------------------------------------------- registry

ROBOTS = {
    robot.key: robot
    for robot in (
        Robot(
            key='tiago',
            description='PAL TIAGo, differential base with wheel velocity servos (arm tucked)',
            source='menagerie',
            mjcf='pal_tiago/tiago_velocity.xml',
            license='Apache-2.0',
            base_body='base_link',
            adapter='diff_drive',
            mode='differential',
            actuators={'left': 'wheel_left_joint_vel', 'right': 'wheel_right_joint_vel'},
            wheels={'left': 'wheel_left_link', 'right': 'wheel_right_link'},
            scan_site='base_laser',
            customize=_customize_tiago,
        ),
        Robot(
            key='scout_mini',
            description='AgileX Scout Mini, skid-steer with four in-wheel velocity servos',
            source='package',
            mjcf='scout_mini/scout_mini.xml',
            license='BSD (ugv_gazebo_sim scout_description package.xml)',
            base_body='base_link',
            adapter='skid_steer',
            mode='skid_steer',
            actuators={r: f'{r}_wheel' for r in adapters.ROLES['skid_steer']},
            wheels={r: f'{r}_wheel_link' for r in adapters.ROLES['skid_steer']},
            scan_site='lidar',
        ),
        Robot(
            key='ackermann_car',
            description='Front-steered, rear-driven car built from primitives',
            source='package',
            mjcf='ackermann_car/ackermann_car.xml',
            license='Apache-2.0',
            base_body='base_link',
            adapter='ackermann',
            mode='ackermann',
            actuators={
                'steer_left': 'steer_left',
                'steer_right': 'steer_right',
                'rear_left': 'rear_left_wheel',
                'rear_right': 'rear_right_wheel',
            },
            wheels={
                'rear_left': 'rear_left_wheel_link',
                'rear_right': 'rear_right_wheel_link',
                'front_left': 'front_left_wheel_link',
                'front_right': 'front_right_wheel_link',
            },
            steer_joints=('front_left_steer', 'front_right_steer'),
            scan_site='lidar',
        ),
        Robot(
            key='tidybot',
            description='Stanford TidyBot base, ideal holonomic (planar x, y, yaw joints)',
            source='menagerie',
            mjcf='stanford_tidybot/base.xml',
            license='MIT',
            base_body='base_link',
            adapter='holonomic',
            mode='omnidirectional',
            actuators={'x': 'joint_x_vel', 'y': 'joint_y_vel', 'theta': 'joint_th_vel'},
            planar_joints=('joint_x', 'joint_y', 'joint_th'),
            scan_site='base_laser',
            customize=_customize_tidybot,
        ),
        Robot(
            key='x2',
            description='Skydio X2 quadrotor, four thrust motors',
            source='menagerie',
            mjcf='skydio_x2/x2.xml',
            license='Apache-2.0',
            base_body='x2',
            adapter='quadrotor',
            mode='3d',
            spawn_height=0.005,
            option={'density': 1.225, 'viscosity': 1.8e-5},
        ),
    )
}


def get(key):
    """Return the :class:`Robot` of ``key``."""
    try:
        return ROBOTS[key]
    except KeyError:
        raise KeyError(f'unknown robot {key!r}; known: {", ".join(sorted(ROBOTS))}') from None


def models_dir():
    """Return this package's ``models/`` directory (source tree or installed share)."""
    here = pathlib.Path(__file__).resolve().parent.parent / 'models'
    if here.is_dir():
        return here
    from ament_index_python.packages import get_package_share_directory

    return pathlib.Path(get_package_share_directory(PACKAGE)) / 'models'


def mjcf_path(robot, menagerie_dir=None):
    """Return the MJCF file of a robot, fetching the Menagerie first if needed."""
    robot = get(robot) if isinstance(robot, str) else robot
    if robot.source == 'menagerie':
        return menagerie.ensure_menagerie(menagerie_dir) / robot.mjcf
    return models_dir() / robot.mjcf


def load_spec(robot, menagerie_dir=None):
    """Return the robot's ``MjSpec``: loaded, assets made absolute, customized, base placed."""
    import mujoco

    robot = get(robot) if isinstance(robot, str) else robot
    spec = mujoco.MjSpec.from_file(str(mjcf_path(robot, menagerie_dir)))
    absolutize_assets(spec)
    if robot.customize is not None:
        robot.customize(spec)
    if robot.spawn_height is not None:
        spec.body(robot.base_body).pos = [0.0, 0.0, robot.spawn_height]
    return spec


def read_geometry(model, data, robot, prefix=''):
    """Read the drive geometry of a ground robot from its compiled model.

    The wheel radius and the contact positions come from each wheel body's contact geom (the
    collision geom with the largest radius about the wheel's hinge axis), expressed in the base
    frame. Returns a dict with ``radius``, ``track`` (lateral distance between left and right
    contacts) and, where they apply, ``wheelbase`` (front axle to rear axle) and ``max_steer``
    (the steering hinges' limit). Empty for robots without wheels.
    """
    import mujoco

    robot = get(robot) if isinstance(robot, str) else robot
    if not robot.wheels:
        return {}
    mujoco.mj_kinematics(model, data)
    base = model.body(prefix + robot.base_body).id
    rot_base = data.xmat[base].reshape(3, 3)
    wheels = {}
    for role, name in robot.wheels.items():
        bid = model.body(prefix + name).id
        joint = model.body_jntadr[bid]
        axis = data.xaxis[joint]
        best = None
        for g in range(model.ngeom):
            if model.geom_bodyid[g] != bid or not (
                model.geom_contype[g] or model.geom_conaffinity[g]
            ):
                continue
            if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER:
                radius = float(model.geom_size[g, 0])
                centre_w = data.geom_xpos[g]
            else:
                centre_b, radius, _ = _wheel_cylinder(model, data, g, axis)
                centre_w = data.xpos[bid] + data.xmat[bid].reshape(3, 3) @ centre_b
            if best is None or radius > best[0]:
                best = (radius, rot_base.T @ (centre_w - data.xpos[base]))
        if best is None:
            raise ValueError(f'wheel body {name!r} has no contact geom')
        wheels[role] = best
    left = [w for r, w in wheels.items() if r.endswith('left')]
    right = [w for r, w in wheels.items() if r.endswith('right')]
    driven = [wheels[r] for r in adapters.ROLES[robot.adapter] if r in wheels]
    geometry = {
        'radius': float(np.mean([w[0] for w in driven])),
        'track': float(np.mean([w[1][1] for w in left]) - np.mean([w[1][1] for w in right])),
    }
    if robot.adapter == 'ackermann':
        geometry['track'] = float(wheels['rear_left'][1][1] - wheels['rear_right'][1][1])
        steer = [model.joint(prefix + j).id for j in robot.steer_joints]
        pivots = [rot_base.T @ (data.xanchor[j] - data.xpos[base]) for j in steer]
        rear_x = 0.5 * (wheels['rear_left'][1][0] + wheels['rear_right'][1][0])
        geometry['wheelbase'] = float(np.mean([p[0] for p in pivots]) - rear_x)
        geometry['steer_track'] = float(pivots[0][1] - pivots[1][1])
        geometry['max_steer'] = float(min(model.jnt_range[j, 1] for j in steer))
    elif any(r.startswith('front') for r in wheels):
        front = [w[1][0] for r, w in wheels.items() if r.startswith('front')]
        rear = [w[1][0] for r, w in wheels.items() if r.startswith('rear')]
        geometry['wheelbase'] = float(np.mean(front) - np.mean(rear))
    return geometry


def make_controller(model, robot, prefix='', geometry=None):
    """Return the robot's :class:`~vectorfield_mujoco.adapters.Adapter` or quadrotor controller."""
    robot = get(robot) if isinstance(robot, str) else robot
    if robot.adapter == 'quadrotor':
        from vectorfield_mujoco import flight

        return flight.MujocoQuadrotor(model, prefix + robot.base_body)
    ids = {role: model.actuator(prefix + name).id for role, name in robot.actuators.items()}
    return adapters.Adapter(robot.adapter, ids, geometry or {})
