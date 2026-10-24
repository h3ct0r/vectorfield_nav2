# vectorfield_mujoco

MuJoCo physics for the vectorfield_nav2 examples: robots from the
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie), a vendored AgileX
Scout Mini and an own Ackermann car, in worlds built from the same Nav2 maps that map_server
serves.

**Status:** the simulator node, the example launch files (ground robots and the drone) and
their gallery clips.

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

## Examples

Each example starts the physics, the controller, RViz and its scenario
(`config/scenarios/<example>.yaml`), which `run_scenario` drives once the controller is up:

```bash
ros2 launch vectorfield_mujoco drone.launch.py
```

| Launch | Robot | Driven by | Scenario |
|---|---|---|---|
| `differential.launch.py` | `tiago` | Nav2 + vector-field controller | contours around an unmapped box |
| `skid_steer.launch.py` | `scout_mini` | Nav2 + vector-field controller | through `corridor_door` and back |
| `omnidirectional.launch.py` | `tidybot` | curve follower | rounded rectangle at a fixed heading |
| `ackermann.launch.py` | `ackermann_car` | Nav2 (Smac Hybrid) + controller, CBF | swerves around a box next to the path |
| `curve_following.launch.py` | `scout_mini` | curve follower | lemniscate laps |
| `drone.launch.py` | `x2` | 3D follower, CBF | helix around a pillar and a sphere |

`scenario:=false` keeps only the world, `exit_on_done:=true` ends the launch with the scenario,
`use_rviz:=false camera_topic:=camera` publishes an offscreen camera image instead,
`viewer:=true` opens MuJoCo's viewer and `record_path:=clip.mp4` records in simulated time, as
the scenario's `recording` says (camera, view, markers left out, the robot's trail), or with
`record_camera:=chase`, `follow`, `top` or `fixed` and that camera's default view. The generic launches
`mujoco_nav2.launch.py`, `mujoco_curve_follower.launch.py` and `mujoco_drone.launch.py` take any
robot, world, obstacles and scenario.

For the drone, `mujoco_sim` stands in for `point_sim_3d` of vectorfield_follower_3d: it takes
the 3D follower's `cmd_vel` (TwistStamped, heading frame when its `frame_id` is `base_link`,
else `map`) and publishes `odom`, the TF `map → base_link`, `/clock` and `obstacle_cloud` (the
surfaces of the world's obstacles), plus point_sim_3d's optional moving frame. Our cascaded
flight controller (`flight`) tracks the command at the physics rate, standing in for PX4's or
ArduPilot's velocity mode: the drone's attitude and velocity lag are simulated, its autopilot is
not. point_sim_3d's `obstacle_spheres` parameter has no counterpart; the cloud holds the
`obstacles` built into the world.

### Gallery clips

The clips of the README and the docs' Examples pages (`docs/_static/media/<example>.mp4`, `.gif`
and `.jpg`) are recordings of these launches, made headless by `scripts/record_gallery.sh` (run
from the source tree; it is not installed) in the dev container, with the workspace built and
sourced:

```bash
apt-get install ffmpeg fonts-dejavu-core libosmesa6  # if missing
vectorfield_mujoco/scripts/record_gallery.sh          # all six examples
vectorfield_mujoco/scripts/record_gallery.sh drone    # one
```

Each example runs to the end of its scenario (`exit_on_done:=true`), filmed as its scenario's
`recording` says. ffmpeg then writes a 960×540 H.264 MP4 of the run from the first motion (the
long runs sped up, as the caption says), a 480 px GIF of its highlight and a JPEG poster, and the
script fails if a scenario does not succeed or a file exceeds its budget (3 MiB, 2 MiB, 150 KiB).
`--work DIR` keeps the raw recordings and logs; `--encode-only --work DIR` encodes them again
without running the examples. Each example takes a few minutes with OSMesa.

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
- `recorder`, `gl`: offscreen chase, follow, top or fixed camera to MP4 through ffmpeg, in
  simulated time. Unless `MUJOCO_GL` is set, headless rendering uses OSMesa (package
  `libosmesa6`, a dependency) when it is installed, else EGL; with neither, the physics runs and
  rendering is off.

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
