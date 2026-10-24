# AgileX Scout Mini (MuJoCo)

`scout_mini.xml` and `meshes/*.obj` are converted from the Gazebo description of the AgileX
Scout Mini by [`tools/convert_scout_mini.py`](../../tools/convert_scout_mini.py). Do not edit
them by hand: change the converter and rerun it (it needs `numpy`, `trimesh`,
`fast-simplification`, `pycollada` and `mujoco`).

## Source

- Repository: <https://github.com/agilexrobotics/ugv_gazebo_sim>, directory
  `scout/scout_description`
- Commit: `27633a956c845903ee630538afeb17fe70afdd84` (master, 2026-06-03)
- Files used (SHA-256):

| File | SHA-256 |
| --- | --- |
| `package.xml` | `a3cc2a8941bbf01914b804c514cfaea7f6c0d007f8e2d0b08a68ebc2d253c8a8` |
| `urdf/scout_mini.xacro` | `7ae98874d711bd585fb5af637f04eae86d4b4bfc02a9b371b96266a4cdf9de38` |
| `urdf/scout_mini_wheel_1.xacro` | `b0f0a68637abfa2d095336dfb8a7327a8b0ba3958c0076fd3462448c22b4a52a` |
| `urdf/scout_mini_wheel_2.xacro` | `4ccb13e80ec64397458a069f5ef9279ffcbdf8088fdb7e69347b3d10d0a9537d` |
| `urdf/scout_mini_wheel_3.xacro` | `d223eefcc1acb4cd802819433a8c33e7af063a28fc60545f420ce3f683c25748` |
| `urdf/scout_mini_wheel_4.xacro` | `699255cff121fed79632c324e70cd8e7cb4dec61a7a2f852c0b01c3800e1e96a` |
| `urdf/scout_mini.gazebo` | `b47030ed09e6c40e2b0e55e7f3cbbdf953a73d26adb2117e18b2edefcf118eba` |
| `urdf/scout_mini.urdf` (compared, not used) | `1a2aeefeb2b53f6d00f218275cdb85f1f4808b7b8aa7f678cc0a81b01efd22de` |
| `meshes/scout_mini_base_link.STL` | `13a164aadf1b0a2c20327941fd31db091f30d16ca1b50aa74a41e3d35f8417a9` |
| `meshes/wheel.dae` | `71d7c2c16c03782d127b84fcddba7393f057566885a20c1257dcfbbf2e65439c` |

## Licence

The upstream repository has no LICENSE file. The `package.xml` of `scout_description` declares
`<license>BSD</license>`, and the wheel xacros carry the BSD (3-clause) notice of Clearpath
Robotics, Inc. (2015) and Weston Robot Pte. Ltd. (2020), from which they were adapted. These
converted files are redistributed under that declaration; the repository's
[NOTICE](../../../NOTICE) records it. If the declaration is not enough for your use, delete this
directory and run the converter locally: it produces the same files.

The notice of the wheel xacros, reproduced as its conditions require:

```text
Software License Agreement (BSD)

\file      scout_wheel.xacro (adapted from wheel.urdf.xacro in husky_description)
\authors   Paul Bovbel <pbovbel@clearpathrobotics.com>
           Ruixiang Du <ruixiang.du@westonrobot.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.
\copyright Copyright (c) 2020, Weston Robot Pte. Ltd., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## Changes made in the conversion

1. **Wheel geometry from the AgileX manual.** SCOUT MINI User Manual v1.0.1 (AgileX):
   wheelbase 452 mm (section 1.2; the side view in section 6.1 draws 460 mm), track 450 mm
   between the wheel centres, 549 mm overall width (so 99 mm tyres) and 175 mm wheels (the
   rear view in section 6.1). The wheels sit at the four corners. Upstream's descriptions match
   neither the manual nor each other: `scout_mini.xacro` has a 313 mm wheelbase, a 456 mm joint
   track, 160 mm × 84 mm wheels and wheel collisions 0.03 m outboard (a 516 mm contact track);
   the Gazebo skid-steer plugin uses `wheelSeparation 0.58` and `wheelbase 0.49`; the stale
   `scout_mini.urdf` puts the wheels at ±0.2320, ±0.2083. Only the wheel centres' height
   (0.077 m below `base_link`) comes from the xacro.
2. **Wheel contact.** One cylinder per wheel, radius 0.0875 m and length 0.099 m, centred on
   the wheel, so the contact track is the 450 mm track; `registry.read_geometry()` reports it.
3. **Mass.** The xacro's 132.39 kg chassis is replaced by the datasheet: "Weight of vehicle body
   20 kg" (section 1.2) is the total. The wheels keep the upstream 3 kg each (in-wheel motors),
   with the inertia of a solid 175 mm × 99 mm cylinder, so the chassis is 8 kg, with its
   centre of mass at `base_link` and the inertia of a solid box of the chassis collision box.
   A later manual lists 23 kg.
4. **Meshes.** The 205 k-triangle chassis STL is decimated to 24 k triangles (quadric
   simplification) and turned −90° about z: the STL lies with its long side across the
   wheels, so the xacro's visual origin (rpy 0) shows the body crosswise (the URDF's `.dae`
   visual turns its mesh by rpy 1.57 0 -1.57). Turned, the deck rails run along x with the
   front fence and light at +x and the tail recess (voltmeter, power switch, charging port) at
   −x, as the manual's sections 2 and 6.1 show (upstream's RViz image has front and rear
   swapped). The mesh is centred on its four suspension arms (the parts below z = −0.06 m,
   which end at the wheel hubs), so they meet the wheels. `wheel.dae` is split by material
   colour into tyre, hub and trim parts, decimated to 9 k triangles in total and scaled from
   upstream's 160 mm × 84 mm wheel to the datasheet's 175 mm × 99 mm. All visual meshes are in
   geom group 2, without contacts.
5. **Collision primitives** (geom group 3): a 0.56 × 0.35 × 0.18 m chassis box (fence to tail,
   up to the tyres' inner faces), the wheel cylinders, and a lidar puck.
6. **Drive.** Four wheel hinges (axle = `base_link` y, positive forward), each with a velocity
   servo (`kv` 20 N m s/rad, ±70 rad/s, ±20 N m), as the real robot's four in-wheel motors. The
   Gazebo `planar_move` plugin, which moved the body directly, is not ported.
7. **Friction.** The Gazebo `mu1`/`mu2` values (1.0 / 0.9, in a block that is commented out
   upstream) are direction-dependent; MuJoCo's geom friction is isotropic, so the wheels use
   the lateral 0.9, which governs skid-steer slip, and win over the floor's friction through
   `priority`.
8. **Sensors.** `lidar` site on a 2D lidar puck on top of the chassis, 0.29 m above the ground
   (upstream `laser_link` was 0.4 m above `base_link`, above the 0.5 m walls of the example
   worlds); `imu` site at `base_link` with gyro, accelerometer and orientation sensors;
   `base_footprint` site on the ground below `base_link`.
