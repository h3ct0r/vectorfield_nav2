# vectorfield_mujoco

MuJoCo physics for the vectorfield_nav2 examples: robots from the
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie), a vendored AgileX
Scout Mini and an own Ackermann car, in worlds built from the same Nav2 maps that map_server
serves.

**Status:** the simulation core (milestone D2). The simulator node and the example launch files
come next.

The package is source-only: it depends on MuJoCo from pip (rosdep key `python3-mujoco-pip`; on
Ubuntu Noble and later run rosdep with `PIP_BREAK_SYSTEM_PACKAGES=1`).

## Robots

| Key | Robot | Source, licence | Drive |
|---|---|---|---|
| `tiago` | PAL TIAGo, arm tucked | Menagerie `pal_tiago/tiago_velocity.xml`, Apache-2.0 | differential: two wheel velocity servos |
| `scout_mini` | AgileX Scout Mini | [`models/scout_mini`](models/scout_mini/README.md), converted from Gazebo, BSD | skid-steer: four wheel velocity servos |
| `ackermann_car` | front-steered car | [`models/ackermann_car`](models/ackermann_car/ackermann_car.xml), Apache-2.0 | Ackermann: steering position servos, rear velocity servos |
| `tidybot` | Stanford TidyBot base | Menagerie `stanford_tidybot/base.xml`, MIT | holonomic: planar x, y, yaw velocity servos |
| `x2` | Skydio X2 | Menagerie `skydio_x2/x2.xml`, Apache-2.0 | four thrust motors, quadrotor velocity controller |

The Menagerie robots are not vendored. They are fetched on first use, or ahead of time:

```bash
ros2 run vectorfield_mujoco fetch_menagerie
```

This sparse-clones `pal_tiago`, `stanford_tidybot` and `skydio_x2` at a pinned commit into
`~/.cache/vectorfield_mujoco/menagerie/<sha>`. Set `VF_MENAGERIE_DIR` to use another directory,
or an existing Menagerie checkout offline.

## Modules

- `registry`: the robots above; wheel radius, track, wheelbase and steering limit are read from
  the compiled model.
- `scene`: floor, lights, walls extruded from a Nav2 map YAML/PGM (greedily merged boxes, 0.5 m
  high, geom group 4), unmapped obstacles (box, cylinder, sphere) and the robot.
- `adapters`: body twist to wheel speeds, steering angles or planar velocities.
- `flight`: cascaded quadrotor velocity controller (velocity PI, tilt-limited attitude, SO(3)
  PD, priority mixer), standing in for an autopilot's velocity mode.
- `sensors`: 2D lidar (`mj_multiRay`, world geoms only) and the obstacle point cloud.
- `overlay`: RViz-style markers drawn into a MuJoCo scene.
- `recorder`, `gl`: offscreen chase, top or fixed camera to MP4 through ffmpeg, in simulated
  time. Unless `MUJOCO_GL` is set, headless rendering uses OSMesa (package `libosmesa6`, a
  dependency) when it is installed, else EGL; with neither, the physics runs and rendering is
  off.

```python
import mujoco
from vectorfield_mujoco import scene

world = scene.make_world('scout_mini', map_yaml='open_10x10.yaml', pose=(0.0, 0.0, 0.0))
while world.data.time < 5.0:
    world.drive(0.4, 0.0, 0.2)  # body twist: vx, vy, yaw rate
    mujoco.mj_step(world.model, world.data)
print(world.base_pose())
```

`tools/convert_scout_mini.py` regenerates `models/scout_mini` (a dev-time script, not installed).
