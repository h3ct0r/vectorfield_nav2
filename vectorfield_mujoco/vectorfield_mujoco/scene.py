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

"""The MuJoCo world: floor, lights, walls from a Nav2 map, unmapped obstacles and the robot.

The MuJoCo world frame is Nav2's ``map`` frame. :func:`map_walls` extrudes the occupied cells of
the same map YAML/PGM that map_server serves (``resolution``, ``origin``, ``negate``,
``occupied_thresh``), greedily merged into boxes, so the physical walls are the cells Nav2
sees. :func:`build` assembles an ``mujoco.MjSpec``; :func:`make_world` adds a registry robot
and returns a compiled :class:`World`.

Walls and obstacles are in geom group :data:`WORLD_GROUP`, which no robot uses: the lidar's
geom-group mask (:data:`WORLD_GROUP_MASK`) hits only them. Renderers must enable that group
(:func:`visual_option`), since MuJoCo hides groups 3-5 by default.
"""

import dataclasses
import math
import pathlib

import numpy as np
import yaml

#: Geom group of the walls and the unmapped obstacles (the lidar sees only this group).
WORLD_GROUP = 4

#: ``mj_multiRay`` geomgroup mask selecting :data:`WORLD_GROUP`.
WORLD_GROUP_MASK = np.array([int(g == WORLD_GROUP) for g in range(6)], dtype=np.uint8)

#: Geom groups shown by :func:`visual_option`: the floor (0), robot visuals (1, 2) and the world.
VISIBLE_GROUPS = (0, 1, 2, WORLD_GROUP)

#: Default wall height (m): taller than every robot's lidar.
WALL_HEIGHT = 0.5

TIMESTEP = 0.002

#: Least statistic extent (m) of a compiled world, that of the open_10x10 world. MuJoCo sizes
#: the directional light's shadow map by the extent, and a small world (a drone among a few
#: obstacles, without walls) gets a speckled floor around it.
MIN_EXTENT = 20.0


@dataclasses.dataclass
class OccupancyMap:
    """An occupancy grid as map_server builds it.

    Attributes:
        occupied: Boolean cells, ``occupied[row, col]`` with row 0 at the *bottom* (y = origin).
        resolution: Cell size (m).
        origin: (x, y, yaw) of the lower-left cell's corner in the map frame.
    """

    occupied: np.ndarray
    resolution: float
    origin: tuple


def read_pgm(path):
    """Read a binary (P5) or ASCII (P2) PGM into a 2D array, top row first, scaled to 0-255."""
    data = pathlib.Path(path).read_bytes()
    tokens = []
    pos = 0
    # Header: magic, width, height, maxval, separated by whitespace and '#' comments.
    while len(tokens) < 4:
        while data[pos : pos + 1].isspace():
            pos += 1
        if data[pos : pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while end < len(data) and not data[end : end + 1].isspace():
            end += 1
        tokens.append(data[pos:end].decode('ascii'))
        pos = end
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    count = width * height
    if magic == 'P5':
        pos += 1  # the single whitespace after maxval
        dtype = np.uint8 if maxval < 256 else np.dtype('>u2')
        pixels = np.frombuffer(data, dtype=dtype, count=count, offset=pos)
    elif magic == 'P2':
        pixels = np.array(data[pos:].split()[:count], dtype=np.int64)
    else:
        raise ValueError(f'{path}: not a PGM (P5/P2) image')
    if pixels.size != count:
        raise ValueError(f'{path}: truncated image')
    return (pixels.reshape(height, width).astype(np.float64) * (255.0 / maxval)).round()


def _read_image(path):
    if pathlib.Path(path).suffix.lower() == '.pgm':
        return read_pgm(path)
    try:
        from PIL import Image
    except ImportError as e:  # pragma: no cover - depends on the host
        raise ValueError(f'{path}: only PGM maps are supported without Pillow') from e
    with Image.open(path) as image:
        return np.asarray(image.convert('L'), dtype=np.float64)


def load_map(yaml_path):
    """Load a Nav2 map YAML and its image into an :class:`OccupancyMap`.

    A cell is occupied when its occupancy probability exceeds ``occupied_thresh``, as
    map_server's ``trinary`` and ``scale`` modes decide it: p = (255 − v)/255, or v/255 with
    ``negate``. Unknown and free cells are not walls.
    """
    yaml_path = pathlib.Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text())
    image = _read_image(yaml_path.parent / meta['image'])
    shade = image / 255.0
    occ = shade if int(meta.get('negate', 0)) else 1.0 - shade
    occupied = occ > float(meta.get('occupied_thresh', 0.65))
    origin = tuple(float(v) for v in (list(meta.get('origin', [0.0, 0.0, 0.0])) + [0.0])[:3])
    return OccupancyMap(np.flipud(occupied), float(meta['resolution']), origin)


def merge_boxes(occupied):
    """Cover the occupied cells with rectangles, greedily (row-major, widest run, then down).

    Returns:
        A list of (row, col, rows, cols) rectangles that exactly cover ``occupied`` without
        overlap.
    """
    occ = np.asarray(occupied, dtype=bool)
    free = occ.copy()
    rows, cols = occ.shape
    boxes = []
    for r in range(rows):
        c = 0
        while c < cols:
            if not free[r, c]:
                c += 1
                continue
            width = 1
            while c + width < cols and free[r, c + width]:
                width += 1
            height = 1
            while r + height < rows and free[r + height, c : c + width].all():
                height += 1
            free[r : r + height, c : c + width] = False
            boxes.append((r, c, height, width))
            c += width
    return boxes


def map_walls(occupancy, height=WALL_HEIGHT):
    """Return the walls of an :class:`OccupancyMap` as obstacle dicts (boxes on the floor)."""
    res = occupancy.resolution
    ox, oy, oyaw = occupancy.origin
    c, s = math.cos(oyaw), math.sin(oyaw)
    walls = []
    for r, col, nr, nc in merge_boxes(occupancy.occupied):
        # Box centre in the map image frame, then rotated by the origin yaw.
        lx, ly = (col + 0.5 * nc) * res, (r + 0.5 * nr) * res
        walls.append(
            {
                'type': 'box',
                'pos': [ox + c * lx - s * ly, oy + s * lx + c * ly, 0.5 * height],
                'size': [0.5 * nc * res, 0.5 * nr * res, 0.5 * height],
                'yaw': oyaw,
            }
        )
    return walls


def _yaw_quat(yaw):
    return [math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)]


_GEOM_TYPES = {'box': 'mjGEOM_BOX', 'cylinder': 'mjGEOM_CYLINDER', 'sphere': 'mjGEOM_SPHERE'}


def obstacle_geom(obstacle):
    """Normalise an obstacle dict; return (geom type name, size, pos, yaw).

    ``size`` follows MuJoCo: box half-extents [x, y, z], cylinder [radius, half-height], sphere
    [radius]. ``pos`` is [x, y] (resting on the floor) or [x, y, z] (centre).
    """
    kind = obstacle.get('type', 'box')
    if kind not in _GEOM_TYPES:
        raise ValueError(f'obstacle type must be one of {sorted(_GEOM_TYPES)}, not {kind!r}')
    size = [float(v) for v in obstacle['size']]
    need = {'box': 3, 'cylinder': 2, 'sphere': 1}[kind]
    if len(size) != need or min(size) <= 0.0:
        raise ValueError(f'{kind} obstacle needs {need} positive size values, got {size}')
    pos = [float(v) for v in obstacle['pos']]
    if len(pos) == 2:
        pos.append(size[0] if kind == 'sphere' else size[-1])
    if len(pos) != 3:
        raise ValueError(f'obstacle pos must be [x, y] or [x, y, z], got {pos}')
    return kind, size + [0.0] * (3 - len(size)), pos, float(obstacle.get('yaw', 0.0))


def build(
    robot=None,
    *,
    map_yaml=None,
    obstacles=(),
    wall_height=WALL_HEIGHT,
    pose=(0.0, 0.0, 0.0, 0.0),
    prefix='robot/',
    option=None,
):
    """Return the world ``mujoco.MjSpec``.

    Args:
        robot: The robot's ``MjSpec`` (attached under a frame at ``pose``), or None.
        map_yaml: Nav2 map YAML whose occupied cells become walls, or None.
        obstacles: Unmapped obstacle dicts (``type``, ``pos``, ``size``, optional ``yaw``,
            ``rgba``, ``name``; see :func:`obstacle_geom`).
        wall_height: Height of the map walls (m).
        pose: (x, y, z, yaw) of the frame the robot is attached to.
        prefix: Name prefix of every attached robot element.
        option: Extra ``mjOption`` fields (e.g. ``density``, ``viscosity`` for a drone).
    """
    import mujoco

    spec = mujoco.MjSpec()
    spec.modelname = 'vectorfield_world'
    spec.compiler.degree = False
    spec.option.timestep = TIMESTEP
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    for key, value in (option or {}).items():
        setattr(spec.option, key, value)
    spec.visual.global_.offwidth = 1920
    spec.visual.global_.offheight = 1080
    spec.visual.quality.shadowsize = 4096
    # Dim enough that the sun's light and shadows keep the floor's checker from saturating.
    spec.visual.headlight.ambient = [0.25, 0.25, 0.25]
    spec.visual.headlight.diffuse = [0.25, 0.25, 0.25]
    spec.visual.headlight.specular = [0.0, 0.0, 0.0]
    spec.visual.map.znear = 0.01

    spec.add_texture(
        name='vf_skybox',
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=[0.55, 0.7, 0.85],
        rgb2=[0.1, 0.12, 0.15],
        width=512,
        height=3072,
    )
    spec.add_texture(
        name='vf_checker',
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        rgb1=[0.82, 0.84, 0.86],
        rgb2=[0.72, 0.74, 0.77],
        mark=mujoco.mjtMark.mjMARK_EDGE,
        markrgb=[0.6, 0.62, 0.65],
        width=512,
        height=512,
    )
    floor = spec.add_material(
        name='vf_floor', texrepeat=[0.5, 0.5], texuniform=True, reflectance=0.05
    )
    floor.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'vf_checker'  # 1 m squares
    spec.add_material(name='vf_wall', rgba=[0.55, 0.58, 0.62, 1.0])
    spec.add_material(name='vf_obstacle', rgba=[0.85, 0.45, 0.15, 1.0])

    world = spec.worldbody
    world.add_geom(
        name='floor',
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[0.0, 0.0, 0.05],
        material='vf_floor',
        group=0,
    )
    sun = world.add_light(
        name='sun',
        pos=[0.0, 0.0, 6.0],
        dir=[0.3, 0.2, -1.0],
        diffuse=[0.7, 0.7, 0.7],
        specular=[0.1, 0.1, 0.1],
        castshadow=True,
    )
    if hasattr(mujoco, 'mjtLightType'):
        sun.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    else:  # pragma: no cover - MuJoCo < 3.3
        sun.directional = True

    walls = map_walls(load_map(map_yaml), wall_height) if map_yaml else []
    for i, wall in enumerate(walls):
        _add_obstacle(spec, wall, f'wall_{i}', 'vf_wall')
    for i, obstacle in enumerate(obstacles):
        _add_obstacle(spec, obstacle, obstacle.get('name', f'obstacle_{i}'), 'vf_obstacle')

    if robot is not None:
        # The world's options win; aligning the robot's first keeps attach() from warning.
        for field in ('timestep', 'integrator', 'cone', 'impratio', 'density', 'viscosity'):
            setattr(robot.option, field, getattr(spec.option, field))
        x, y, z, yaw = pose
        frame = world.add_frame(name='robot_spawn', pos=[x, y, z], quat=_yaw_quat(yaw))
        spec.attach(robot, prefix=prefix, frame=frame)
    return spec


def _add_obstacle(spec, obstacle, name, material):
    import mujoco

    kind, size, pos, yaw = obstacle_geom(obstacle)
    geom = spec.worldbody.add_geom(
        name=name,
        type=getattr(mujoco.mjtGeom, _GEOM_TYPES[kind]),
        size=size,
        pos=pos,
        quat=_yaw_quat(yaw),
        group=WORLD_GROUP,
        material=material,
    )
    if 'rgba' in obstacle:
        geom.material = ''
        geom.rgba = [float(v) for v in obstacle['rgba']]
    return geom


def visual_option():
    """Return an ``MjvOption`` showing :data:`VISIBLE_GROUPS` (robot collision groups hidden)."""
    import mujoco

    opt = mujoco.MjvOption()
    opt.geomgroup[:] = [int(g in VISIBLE_GROUPS) for g in range(len(opt.geomgroup))]
    return opt


@dataclasses.dataclass
class World:
    """A compiled world with one robot from :mod:`vectorfield_mujoco.registry`.

    Attributes:
        model, data: The compiled model and its data, at the initial pose.
        robot: The registry entry.
        prefix: Name prefix of the robot's elements.
        base_id: Body id of the robot base.
        controller: :class:`~vectorfield_mujoco.adapters.Adapter` (ground robots) or
            :class:`~vectorfield_mujoco.flight.MujocoQuadrotor` (drones).
        geometry: Wheel geometry read from the model (empty for the drone and holonomic base).
        scan_site: Site id of the lidar, or -1.
        initial_qpos: {qpos address: value} applied by :meth:`reset` (planar joints).
    """

    model: object
    data: object
    robot: object
    prefix: str
    base_id: int
    controller: object
    geometry: dict
    scan_site: int
    initial_qpos: dict = dataclasses.field(default_factory=dict)

    def name(self, element):
        """Return the full (prefixed) name of a robot element."""
        return self.prefix + element

    def reset(self):
        """Reset the simulation to the initial pose, at rest."""
        import mujoco

        mujoco.mj_resetData(self.model, self.data)
        for address, value in self.initial_qpos.items():
            self.data.qpos[address] = value
        mujoco.mj_forward(self.model, self.data)
        if hasattr(self.controller, 'controller'):
            self.controller.controller.reset()

    def base_pose(self):
        """Return the base (x, y, z, yaw) in the world (map) frame."""
        pos = self.data.xpos[self.base_id]
        rot = self.data.xmat[self.base_id].reshape(3, 3)
        return float(pos[0]), float(pos[1]), float(pos[2]), math.atan2(rot[1, 0], rot[0, 0])

    def base_twist(self):
        """Return the (linear, angular) velocity of the base origin, in the base frame."""
        import mujoco

        vel = np.zeros(6)
        mujoco.mj_objectVelocity(
            self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY, self.base_id, vel, 1
        )
        return vel[3:].copy(), vel[:3].copy()

    def drive(self, vx, vy, omega):
        """Write a body twist command (ground robots) or a world velocity (drone) into ctrl.

        For the drone, (vx, vy, omega) is read as the world-frame velocity (vx, vy, 0) and the
        yaw rate omega; :class:`~vectorfield_mujoco.flight.MujocoQuadrotor` takes a full 3D
        set-point directly.
        """
        if self.robot.adapter == 'quadrotor':
            return self.controller.apply(self.model, self.data, [vx, vy, 0.0], omega)
        return self.controller.apply(self.data, vx, vy, omega, self.base_pose()[3])

    def fly(self, velocity, yaw_rate):
        """Write the drone's thrusts for a world-frame velocity (m/s) and a yaw rate (rad/s)."""
        return self.controller.apply(self.model, self.data, velocity, yaw_rate)


def make_world(
    robot_key,
    *,
    map_yaml=None,
    obstacles=(),
    pose=(0.0, 0.0, 0.0),
    altitude=0.0,
    wall_height=WALL_HEIGHT,
    prefix='robot/',
    menagerie_dir=None,
):
    """Build, compile and reset a world with the registry robot ``robot_key``.

    Args:
        robot_key: A key of :data:`vectorfield_mujoco.registry.ROBOTS`.
        map_yaml, obstacles, wall_height: See :func:`build`.
        pose: Initial (x, y, yaw) of the base in the map frame.
        altitude: Extra initial height of the base (the drone's take-off altitude).
        prefix: Name prefix of the robot's elements.
        menagerie_dir: Menagerie checkout (default: fetched on demand).
    """
    import mujoco

    from vectorfield_mujoco import registry

    robot = registry.get(robot_key)
    robot_spec = registry.load_spec(robot, menagerie_dir=menagerie_dir)
    x, y, yaw = pose
    if robot.planar_joints:
        # Planar joints are expressed in the attachment frame: attach at the origin and start
        # from the initial pose through the joints, so that the joint axes are the map axes.
        frame_pose = (0.0, 0.0, altitude, 0.0)
    else:
        frame_pose = (x, y, altitude, yaw)
    spec = build(
        robot_spec,
        map_yaml=map_yaml,
        obstacles=obstacles,
        wall_height=wall_height,
        pose=frame_pose,
        prefix=prefix,
        option=robot.option,
    )
    model = spec.compile()
    model.stat.extent = max(float(model.stat.extent), MIN_EXTENT)
    data = mujoco.MjData(model)
    initial = {}
    for joint, value in zip(robot.planar_joints, (x, y, yaw)):
        initial[int(model.jnt_qposadr[model.joint(prefix + joint).id])] = value
    base_id = model.body(prefix + robot.base_body).id
    mujoco.mj_forward(model, data)
    geometry = registry.read_geometry(model, data, robot, prefix)
    controller = registry.make_controller(model, robot, prefix, geometry)
    scan_site = model.site(prefix + robot.scan_site).id if robot.scan_site else -1
    world = World(model, data, robot, prefix, base_id, controller, geometry, scan_site, initial)
    world.reset()
    return world
