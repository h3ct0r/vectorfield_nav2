#!/usr/bin/env python3
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

"""Convert the AgileX Scout Mini Gazebo description into ``models/scout_mini/`` (dev-time tool).

Run once by a developer; the outputs are committed. It is not installed and needs, besides
numpy, ``trimesh``, ``fast-simplification`` and ``pycollada`` (and ``mujoco`` for the final
check)::

    python3 -m venv /tmp/scout && /tmp/scout/bin/pip install \
        numpy trimesh fast-simplification pycollada mujoco
    /tmp/scout/bin/python vectorfield_mujoco/tools/convert_scout_mini.py

Steps:

1. Fetch the pinned files of ``agilexrobotics/ugv_gazebo_sim`` ``scout/scout_description``
   (:data:`UPSTREAM_COMMIT`) and print their SHA-256.
2. Read the wheel height, wheel mass and friction from ``scout_mini.xacro``, the wheel xacros and
   ``scout_mini.gazebo``. The wheel geometry comes from the AgileX user manual instead
   (:data:`DATASHEET_WHEELBASE` and the constants after it): upstream's descriptions match
   neither the manual nor each other.
3. Decimate ``scout_mini_base_link.STL`` and ``wheel.dae`` (split by material colour into tyre,
   hub and accent parts) to OBJ.
4. Write ``scout_mini.xml``: visual meshes in group 2, collision primitives in group 3, four
   wheel hinges with velocity actuators, isotropic wheel friction, tyres soft at first touch,
   lidar and IMU sites, and the datasheet mass (see ``models/scout_mini/README.md`` for every
   change).
5. Compile it with MuJoCo and print the wheel geometry read back from the model.
"""

import argparse
import hashlib
import pathlib
import re
import sys
import urllib.request

import numpy as np

UPSTREAM_REPO = 'agilexrobotics/ugv_gazebo_sim'

#: ugv_gazebo_sim master on 2026-06-03 (the latest commit on 2026-10-23).
UPSTREAM_COMMIT = '27633a956c845903ee630538afeb17fe70afdd84'

UPSTREAM_DIR = 'scout/scout_description'

UPSTREAM_FILES = (
    'package.xml',
    'urdf/scout_mini.urdf',
    'urdf/scout_mini.xacro',
    'urdf/scout_mini_wheel_1.xacro',
    'urdf/scout_mini_wheel_2.xacro',
    'urdf/scout_mini_wheel_3.xacro',
    'urdf/scout_mini_wheel_4.xacro',
    'urdf/scout_mini.gazebo',
    'meshes/scout_mini_base_link.STL',
    'meshes/wheel.dae',
)

#: Total vehicle mass (kg): "Weight of vehicle body 20 kg", SCOUT MINI User Manual v1.0.1
#: (AgileX), section 1.2. The xacro's chassis alone is 132.39 kg.
DATASHEET_MASS = 20.0

#: Wheel geometry from the same manual: wheelbase 452 mm (section 1.2; the side view in section
#: 6.1 draws 460), track 450 mm between the wheel centres and 549 mm overall width (rear view,
#: so a 99 mm tyre), and a 175 mm wheel. Upstream's xacro (wheelbase 313 mm, joint track
#: 456 mm, 160 mm wheels, collisions 30 mm outboard) and its Gazebo plugin (separation 580 mm,
#: wheelbase 490 mm) match neither the manual nor each other.
DATASHEET_WHEELBASE = 0.452
DATASHEET_TRACK = 0.450
DATASHEET_WIDTH = 0.549
DATASHEET_WHEEL_DIAMETER = 0.175

#: Yaw (deg) of the chassis mesh on base_link. The STL lies with its long side (0.56 m) along y,
#: across the wheels: the xacro's rpy 0 is wrong (the URDF's .dae visual turns its mesh by rpy
#: 1.57 0 -1.57). -90° puts the deck rails along x with the front fence and light at +x and the
#: tail recess (voltmeter, power switch, charging port) at -x, as the manual's sections 2 and 6.1
#: show (upstream's RViz image has front and rear swapped).
BASE_YAW_DEG = -90.0

#: Face budgets of the decimated meshes (the model directory must stay under 3 MB).
BASE_FACES = 24000
WHEEL_FACES = 9000

#: Tyre contact impedance (MuJoCo solimp: dmin at first touch, dmax from width on, width in m;
#: midpoint and power stay MuJoCo's): a tyre that stiffens as it deflects. With MuJoCo's default
#: first touch (0.9) a skidding wheel hops: the convex contact model turns part of the friction's
#: slip-velocity correction into a separating normal acceleration (about mu B v_slip, B = 105/s
#: for the default solref), which a stiff contact absorbs only below a few mm/s of slip. Turning
#: in place, the wheels then touched the floor about half the time, each landing spun the light
#: wheel down and tilted the friction sideways, and the base turned at 0.15 to 0.25 of the
#: no-slip yaw rate instead of the Coulomb t^2 / (l^2 + t^2) = 0.5 (l, t: half the wheelbase and
#: half the track). With 0.05 the wheels stay on the floor up to 5 rad/s in place and the base
#: turns at 0.49 of the no-slip rate; at rest they sink 0.6 mm into the floor (0.1 mm before).
WHEEL_SOLIMP = (0.05, 0.95, 0.001)

# Wheel naming, side (+1 left), longitudinal sign (+1 front); upstream labels the left wheels
# 1 and 2 and the right wheels 3 and 4.
WHEELS = (
    ('front_left', 1, 1),
    ('rear_left', 1, -1),
    ('front_right', -1, 1),
    ('rear_right', -1, -1),
)

# wheel.dae material colours (0-255 RGB) -> part name.
WHEEL_PARTS = {
    (75, 75, 75): 'wheel_tire',
    (229, 234, 237): 'wheel_hub',
    (255, 15, 15): 'wheel_trim',
}


def fetch(dest):
    """Download the pinned upstream files into ``dest``; return {relative path: sha256}."""
    dest.mkdir(parents=True, exist_ok=True)
    hashes = {}
    for rel in UPSTREAM_FILES:
        path = dest / pathlib.Path(rel).name
        if not path.is_file():
            url = (
                f'https://raw.githubusercontent.com/{UPSTREAM_REPO}/{UPSTREAM_COMMIT}/'
                f'{UPSTREAM_DIR}/{rel}'
            )
            print(f'fetching {url}', file=sys.stderr)
            with urllib.request.urlopen(url, timeout=120) as response:
                path.write_bytes(response.read())
        hashes[rel] = hashlib.sha256(path.read_bytes()).hexdigest()
    return hashes


def _strip_comments(text):
    return re.sub(r'<!--.*?-->', '', text, flags=re.DOTALL)


def xacro_properties(text):
    """Return the numeric ``<xacro:property>`` values outside comments."""
    props = {}
    pattern = r'<xacro:property\s+name="([^"]+)"\s+value="([^"]+)"'
    for name, value in re.findall(pattern, _strip_comments(text)):
        try:
            props[name] = float(value)
        except ValueError:
            pass
    return props


def read_upstream(src):
    """Read the wheel height, wheel mass and friction the conversion keeps from upstream."""
    props = xacro_properties((src / 'scout_mini.xacro').read_text())
    wheel = _strip_comments((src / 'scout_mini_wheel_1.xacro').read_text())
    inertial = re.search(r'<inertial>.*?</inertial>', wheel, flags=re.DOTALL).group(0)
    wheel_mass = float(re.search(r'<mass value="([0-9.e-]+)"', inertial).group(1))
    gazebo = (src / 'scout_mini.gazebo').read_text()  # mu1/mu2 sit in a commented-out block
    mu1 = float(re.search(r'<mu1 value="([0-9.]+)"', gazebo).group(1))
    mu2 = float(re.search(r'<mu2 value="([0-9.]+)"', gazebo).group(1))
    return {
        'wheel_vertical_offset': props['wheel_vertical_offset'],
        'wheel_mass': wheel_mass,
        'mu1': mu1,
        'mu2': mu2,
    }


def simplify(mesh, faces):
    """Quadric-decimate a trimesh mesh to about ``faces`` faces."""
    import fast_simplification
    import trimesh

    if len(mesh.faces) <= faces:
        return mesh
    points, tris = fast_simplification.simplify(
        np.asarray(mesh.vertices, dtype=np.float32),
        np.asarray(mesh.faces, dtype=np.int64),
        target_reduction=1.0 - faces / len(mesh.faces),
    )
    out = trimesh.Trimesh(points, tris, process=True)
    out.remove_unreferenced_vertices()
    return out


def write_obj(mesh, path, header):
    """Write a compact OBJ (positions and faces only, 10 µm precision); MuJoCo makes normals."""
    lines = [f'# {line}' for line in header]
    lines += [f'v {x:.5f} {y:.5f} {z:.5f}' for x, y, z in np.asarray(mesh.vertices)]
    lines += [f'f {a + 1} {b + 1} {c + 1}' for a, b, c in np.asarray(mesh.faces)]
    path.write_text('\n'.join(lines) + '\n')


def convert_meshes(src, out_meshes, base_faces, wheel_faces):
    """Decimate the chassis STL and the wheel DAE parts into ``out_meshes``."""
    import trimesh

    out_meshes.mkdir(parents=True, exist_ok=True)
    header = [f'Decimated from {UPSTREAM_REPO}@{UPSTREAM_COMMIT[:12]} by convert_scout_mini.py']
    base = trimesh.load(src / 'scout_mini_base_link.STL', process=True)
    base = simplify(base, base_faces)
    write_obj(
        base, out_meshes / 'base_link.obj', header + ['from meshes/scout_mini_base_link.STL']
    )
    print(f'base_link.obj: {len(base.faces)} faces', file=sys.stderr)

    scene = trimesh.load(src / 'wheel.dae', force='scene')
    parts = {}
    for geometry in scene.geometry.values():
        color = tuple(int(c) for c in geometry.visual.material.main_color[:3])
        parts.setdefault(WHEEL_PARTS[color], []).append(geometry)
    total = sum(len(g.faces) for gs in parts.values() for g in gs)
    for name, geometries in sorted(parts.items()):
        mesh = trimesh.util.concatenate(
            [trimesh.Trimesh(g.vertices, g.faces, process=True) for g in geometries]
        )
        mesh = simplify(mesh, max(200, int(wheel_faces * len(mesh.faces) / total)))
        write_obj(mesh, out_meshes / f'{name}.obj', header + ['from meshes/wheel.dae'])
        print(f'{name}.obj: {len(mesh.faces)} faces', file=sys.stderr)


def _box_inertia(mass, half):
    x, y, z = (2.0 * h for h in half)
    return (
        mass * (y * y + z * z) / 12.0,
        mass * (x * x + z * z) / 12.0,
        mass * (x * x + y * y) / 12.0,
    )


def _f(*values):
    return ' '.join(f'{v:.7g}' for v in values)


def obj_vertices(path):
    """Return an OBJ file's vertices as an (n, 3) array."""
    return np.array(
        [line.split()[1:4] for line in path.read_text().splitlines() if line.startswith('v ')],
        dtype=float,
    )


def mjcf(geo, mesh_dir):
    """Return the scout_mini.xml text for the upstream geometry ``geo``.

    ``mesh_dir`` holds the decimated meshes, whose vertices place the chassis and the wheels.
    """
    half_l, half_t = DATASHEET_WHEELBASE / 2.0, DATASHEET_TRACK / 2.0
    wz = geo['wheel_vertical_offset']
    r = DATASHEET_WHEEL_DIAMETER / 2.0
    tyre = DATASHEET_WIDTH - DATASHEET_TRACK
    # base_link sits at the wheel-centre height above the ground plus the wheel radius.
    ground = -(wz - r)
    # The chassis mesh turned by BASE_YAW_DEG, with its suspension arms (the parts below
    # z = -0.06 m, which end at the wheel hubs) centred on base_link, so that they meet wheels
    # at the datasheet wheelbase and track.
    yaw = np.radians(BASE_YAW_DEG)
    rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    body = obj_vertices(mesh_dir / 'base_link.obj')
    turned = body[:, :2] @ rot.T
    arms = turned[body[:, 2] < -0.06]
    shift = -(arms.min(axis=0) + arms.max(axis=0)) / 2.0
    base_pos = (*shift, 0.0)
    base_quat = (np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0))
    # Chassis collision box: the turned body's length (fence to tail) and height (0.18 m), and
    # its width up to the tyres' inner faces.
    lo, hi = turned.min(axis=0) + shift, turned.max(axis=0) + shift
    chassis_pos = ((lo[0] + hi[0]) / 2.0, 0.0, 0.0)
    chassis_half = ((hi[0] - lo[0]) / 2.0, min((hi[1] - lo[1]) / 2.0, half_t - tyre / 2.0), 0.09)
    wheel_mass = geo['wheel_mass']
    chassis_mass = DATASHEET_MASS - 4.0 * wheel_mass
    ixx, iyy, izz = _box_inertia(chassis_mass, chassis_half)
    # Solid-cylinder inertia of the datasheet wheel (upstream's is for its 160 mm wheel); the
    # axle is body y.
    w_axle = wheel_mass * r * r / 2.0
    w_diam = wheel_mass * (3.0 * r * r + tyre * tyre) / 12.0
    # The upstream wheel meshes (axle = mesh z, the tyre between z ~ 0 and its width) scaled to
    # the datasheet wheel and centred on the hub.
    tyre_mesh = obj_vertices(mesh_dir / 'wheel_tire.obj')
    mesh_r = (tyre_mesh[:, 0].max() - tyre_mesh[:, 0].min()) / 2.0
    mesh_w = tyre_mesh[:, 2].max() - tyre_mesh[:, 2].min()
    scale = (r / mesh_r, r / mesh_r, tyre / mesh_w)
    mesh_mid = (tyre_mesh[:, 2].max() + tyre_mesh[:, 2].min()) / 2.0 * scale[2]
    friction = geo['mu2']
    wheels = []
    for name, side, sign in WHEELS:
        euler = '-1.5707963 0 0' if side > 0 else '1.5707963 0 0'
        visual = f'pos="{_f(0, -side * mesh_mid, 0)}" euler="{euler}"'
        wheels.append(f'''
      <body name="{name}_wheel_link" pos="{_f(sign * half_l, side * half_t, wz)}">
        <joint name="{name}_wheel" class="scout_wheel"/>
        <inertial pos="0 0 0" mass="{_f(wheel_mass)}" diaginertia="{_f(w_diam, w_axle, w_diam)}"/>
        <geom name="{name}_wheel" class="scout_wheel"/>
        <geom class="scout_visual" mesh="wheel_tire" material="scout_tire" {visual}/>
        <geom class="scout_visual" mesh="wheel_hub" material="scout_hub" {visual}/>
        <geom class="scout_visual" mesh="wheel_trim" material="scout_trim" {visual}/>
      </body>''')
    actuators = '\n'.join(
        f'    <velocity name="{name}_wheel" joint="{name}_wheel" class="scout_wheel"/>'
        for name, _, _ in WHEELS
    )
    return f'''<?xml version="1.0"?>
<!--
AgileX Scout Mini, converted from {UPSTREAM_REPO} {UPSTREAM_DIR}
at commit {UPSTREAM_COMMIT} (package.xml licence: BSD) by
vectorfield_mujoco/tools/convert_scout_mini.py. Do not edit by hand: change the converter and
rerun it. README.md lists the sources and every change made to them.
-->
<mujoco model="scout_mini">
  <compiler angle="radian" meshdir="meshes" autolimits="true"/>

  <default>
    <default class="scout_mini">
      <default class="scout_visual">
        <geom type="mesh" group="2" contype="0" conaffinity="0" mass="0"/>
      </default>
      <default class="scout_collision">
        <geom group="3" mass="0" material="scout_collision"/>
      </default>
      <!-- In-wheel motors: velocity servos on the wheel hinges (axle = body y, + = forward).
           Wheel friction is isotropic ({friction:g}, the Gazebo mu2) and wins over the floor's
           through priority, as does the tyre's contact impedance: soft at first touch, so
           that a skidding wheel stays on the floor (the converter's WHEEL_SOLIMP). ctrlrange
           covers the 20 km/h top speed. -->
      <default class="scout_wheel">
        <joint axis="0 1 0" damping="0.01" armature="0.002"/>
        <geom type="cylinder" size="{_f(r, tyre / 2.0)}" zaxis="0 1 0" group="3"
          mass="0" friction="{_f(friction)} 0.005 0.0001" solimp="{_f(*WHEEL_SOLIMP)}"
          priority="1" material="scout_collision"/>
        <velocity kv="20" ctrlrange="-70 70" forcerange="-20 20"/>
      </default>
    </default>
  </default>

  <asset>
    <material name="scout_body" rgba="0.22 0.23 0.25 1" specular="0.3" shininess="0.4"/>
    <material name="scout_tire" rgba="0.1 0.1 0.1 1"/>
    <material name="scout_hub" rgba="0.82 0.84 0.85 1" specular="0.5"/>
    <material name="scout_trim" rgba="0.9 0.1 0.08 1"/>
    <material name="scout_lidar" rgba="0.08 0.08 0.08 1"/>
    <material name="scout_collision" rgba="0.6 0.3 0.3 0.4"/>
    <mesh name="base_link" file="base_link.obj"/>
    <mesh name="wheel_tire" file="wheel_tire.obj" scale="{_f(*scale)}"/>
    <mesh name="wheel_hub" file="wheel_hub.obj" scale="{_f(*scale)}"/>
    <mesh name="wheel_trim" file="wheel_trim.obj" scale="{_f(*scale)}"/>
  </asset>

  <worldbody>
    <body name="base_link" pos="0 0 {ground:.7g}" childclass="scout_mini">
      <freejoint name="root"/>
      <!-- Datasheet total mass ({DATASHEET_MASS:g} kg) less the four upstream wheels; the
           inertia of a solid box of the chassis collision box, centred on base_link. -->
      <inertial pos="0 0 0" mass="{_f(chassis_mass)}" diaginertia="{_f(ixx, iyy, izz)}"/>
      <geom class="scout_visual" mesh="base_link" material="scout_body"
        pos="{_f(*base_pos)}" quat="{_f(*base_quat)}"/>
      <geom name="chassis" class="scout_collision" type="box" pos="{_f(*chassis_pos)}"
        size="{_f(*chassis_half)}"/>
      <!-- A top-mounted 2D lidar puck (upstream: laser_link 0.4 m above base_link). -->
      <geom class="scout_visual" type="cylinder" size="0.04 0.025" pos="0 0 0.115"
        material="scout_lidar"/>
      <geom name="lidar" class="scout_collision" type="cylinder" size="0.04 0.025"
        pos="0 0 0.115"/>
      <site name="lidar" pos="0 0 0.125" size="0.01"/>
      <site name="imu" pos="0 0 0" size="0.01"/>
      <site name="base_footprint" pos="0 0 {-ground:.7g}" size="0.01"/>{''.join(wheels)}
    </body>
  </worldbody>

  <actuator>
{actuators}
  </actuator>

  <sensor>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_accel" site="imu"/>
    <framequat name="imu_quat" objtype="site" objname="imu"/>
  </sensor>
</mujoco>
'''


def check(xml_path):
    """Compile the model, settle it on a floor and print the wheel geometry read back."""
    import mujoco

    spec = mujoco.MjSpec.from_file(str(xml_path))
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.1])
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ids = {
        n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f'{n}_wheel') for n, _, _ in WHEELS
    }
    pos = {n: data.geom_xpos[i] for n, i in ids.items()}
    radius = model.geom_size[ids['front_left'], 0]
    print(
        f'mass {sum(model.body_mass):.3f} kg, wheel radius {radius:.4f} m, '
        f'wheelbase {pos["front_left"][0] - pos["rear_left"][0]:.7f} m, '
        f'contact track {pos["front_left"][1] - pos["front_right"][1]:.7f} m',
        file=sys.stderr,
    )
    for _ in range(1000):
        mujoco.mj_step(model, data)
    print(f'settled base height {data.qpos[2]:.4f} m', file=sys.stderr)


def main(argv=None):
    """Run the conversion."""
    here = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--out', type=pathlib.Path, default=here.parent / 'models' / 'scout_mini')
    parser.add_argument(
        '--download-dir',
        type=pathlib.Path,
        default=pathlib.Path('/tmp/scout_mini_upstream'),
        help='where the upstream files are cached (in a directory per upstream commit)',
    )
    parser.add_argument('--base-faces', type=int, default=BASE_FACES)
    parser.add_argument('--wheel-faces', type=int, default=WHEEL_FACES)
    parser.add_argument('--no-check', action='store_true', help='skip the MuJoCo compile check')
    args = parser.parse_args(argv)

    # Per commit: a new UPSTREAM_COMMIT never reuses the files of an older one.
    cache = args.download_dir / UPSTREAM_COMMIT
    for rel, digest in fetch(cache).items():
        print(f'{digest}  {rel}')
    geo = read_upstream(cache)
    print({k: v for k, v in geo.items()}, file=sys.stderr)
    convert_meshes(cache, args.out / 'meshes', args.base_faces, args.wheel_faces)
    xml_path = args.out / 'scout_mini.xml'
    xml_path.write_text(mjcf(geo, args.out / 'meshes'))
    size = sum(p.stat().st_size for p in args.out.rglob('*') if p.is_file())
    print(f'wrote {xml_path} ({size / 1e6:.2f} MB in {args.out})', file=sys.stderr)
    if not args.no_check:
        check(xml_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
