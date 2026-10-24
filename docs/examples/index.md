# Examples

The MuJoCo examples show the controller in every mode on a physical robot model. Each is one
command: `ros2 launch vectorfield_mujoco <example>.launch.py` starts the MuJoCo physics, the
same Nav2 controller plugin, curve follower or 3D follower that runs on a real robot, RViz and a
scenario that drives the robot by itself. The robots are models from the
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie), an AgileX Scout Mini
converted from its Gazebo description and a car built for the Ackermann mode.

::::{grid} 1 2 3 3
:gutter: 2

:::{grid-item-card} Differential
:link: differential
:link-type: doc
:class-card: vf-example vf-compact

```{clip} differential
:alt: PAL TIAGo contours around an unmapped box and rejoins its path
```

PAL TIAGo · Nav2, obstacle contour
:::

:::{grid-item-card} Skid-steer
:link: skid_steer
:link-type: doc
:class-card: vf-example vf-compact

```{clip} skid_steer
:alt: AgileX Scout Mini drives through a 0.8 m door and back
```

AgileX Scout Mini · Nav2 through a door
:::

:::{grid-item-card} Omnidirectional
:link: omnidirectional
:link-type: doc
:class-card: vf-example vf-compact

```{clip} omnidirectional
:alt: Stanford TidyBot base strafes around a rounded rectangle at a fixed heading
```

TidyBot base · curve follower, fixed heading
:::

:::{grid-item-card} Ackermann
:link: ackermann
:link-type: doc
:class-card: vf-example vf-compact

```{clip} ackermann
:alt: A front-steered car swerves around a box next to its path
```

Car model · Smac Hybrid-A\* and CBF
:::

:::{grid-item-card} Drone
:link: drone
:link-type: doc
:class-card: vf-example vf-compact

```{clip} drone
:alt: Skydio X2 flies a helix around a pillar and a sphere
```

Skydio X2 · 3D follower, CBF
:::

:::{grid-item-card} Curve following
:link: curve_following
:link-type: doc
:class-card: vf-example vf-compact

```{clip} curve_following
:alt: AgileX Scout Mini laps a lemniscate
```

AgileX Scout Mini · lemniscate laps
:::
::::

| Example | Mode | Robot | Driven by | Scenario |
|---|---|---|---|---|
| {doc}`differential` | differential, feedback linearisation | PAL TIAGo | Nav2 + vector-field controller, `obstacle_mode: contour` | contours around an unmapped box, rejoins the path, turns to the goal heading |
| {doc}`skid_steer` | skid-steer, slip compensation | AgileX Scout Mini | Nav2 + vector-field controller | through the 0.8 m door of `corridor_door` and back |
| {doc}`omnidirectional` | omnidirectional, holonomic | Stanford TidyBot base | curve follower, `omni_heading_policy: fixed` | strafes around a rounded rectangle without turning |
| {doc}`ackermann` | Ackermann, heading tracking | own car model | Nav2 (Smac Hybrid-A\*, Dubins) + controller, `obstacle_mode: cbf` | swerves around a box next to its path |
| {doc}`drone` | 3D | Skydio X2 | 3D follower, CBF | climbs a helix around a pillar and a sphere |
| {doc}`curve_following` | skid-steer | AgileX Scout Mini | curve follower | laps of a lemniscate |

```{toctree}
:hidden:

differential
skid_steer
omnidirectional
ackermann
drone
curve_following
```

## Requirements

The examples live in the `vectorfield_mujoco` package, which is **source-only**: it depends on
MuJoCo from pip, which the ROS build farm does not accept, so it is not part of the binary
release ({doc}`../distro_matrix`). Build it in a workspace as in {doc}`../quickstart`, on
Humble, Jazzy or Lyrical. rosdep installs its dependencies:

- **MuJoCo** from pip (rosdep key `python3-mujoco-pip`). Ubuntu Noble and later (Jazzy,
  Lyrical) mark the system Python as externally managed (PEP 668), so pip refuses to install
  there unless rosdep runs with `PIP_BREAK_SYSTEM_PACKAGES=1`:

  ```bash
  PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install --from-paths src --ignore-src -y
  ```

- **OSMesa** (`libosmesa6-dev`), Mesa's software renderer, for headless rendering, and
  **ffmpeg** for recording.
- **git**, to fetch the Menagerie robots.

The devcontainers under `.devcontainer/` have everything else; run rosdep in them as above.

### First run: the Menagerie robots

The Menagerie robots (TIAGo, TidyBot and the X2) are not in the repository. The first launch
that needs one sparse-clones the three robot directories, about 20 MB, at a pinned commit into
`~/.cache/vectorfield_mujoco/menagerie/<sha>`. To fetch them ahead of time, for example before
going offline:

```bash
ros2 run vectorfield_mujoco fetch_menagerie
```

Set `VF_MENAGERIE_DIR` to keep the checkout elsewhere, or to point at an existing
`mujoco_menagerie` checkout, which is then used as it is. The Scout Mini and the car ship with
the package, so `skid_steer`, `ackermann` and `curve_following` never download anything.

### Rendering backends

MuJoCo reads `MUJOCO_GL` once, at import. Unless you set it, the simulator picks a backend
whose library is installed: `cgl` on macOS, `glfw` when there is a display (`DISPLAY` or
`WAYLAND_DISPLAY`), else `osmesa`, then `egl`. EGL needs a GPU or Mesa's EGL drivers and fails
in Docker on macOS; OSMesa works everywhere but is slow. Without any backend the physics still
runs; only the viewer, the camera image and recording are refused, with an error in the log.

## Viewing

Pick one or more:

| How | Arguments | Where it works |
|---|---|---|
| RViz with the vector-field markers | `use_rviz:=true` (default) | a desktop, or the devcontainer's noVNC desktop |
| MuJoCo's own viewer | `viewer:=true` | a Linux desktop, or the noVNC desktop (`docker compose -f .devcontainer/compose.novnc.yml up -d --build`, then `export DISPLAY=:1` in the container) |
| A camera image topic | `use_rviz:=false camera_topic:=camera` | anywhere: a `sensor_msgs/Image` (640×360, 10 Hz, chase camera) for Foxglove's Image panel through `foxglove_bridge`, or RViz's Image display; the easiest way from Docker on macOS |
| An MP4 file | `record_path:=~/clip.mp4` | anywhere: 960×540 at 30 fps, rendered in simulated time, so a slow renderer slows the run but not the clip |

The controller's markers (the path, the field, the command, the prediction, the obstacle
state) are drawn into the viewer, the camera image and the recording too. Each example page
has a legend of the colours; {doc}`../visualization` has the full one.

```bash
ros2 launch vectorfield_mujoco differential.launch.py use_rviz:=false camera_topic:=camera
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

## Launch arguments

Every example takes these arguments:

| Argument | Default | Effect |
|---|---|---|
| `scenario` | `true` | `false` starts only the world; drive it yourself with goals or curves. |
| `exit_on_done` | `false` | Shut everything down when the scenario finishes (recordings, tests). |
| `use_rviz` | `true` | Start RViz with the vector-field layout. |
| `viewer` | `false` | Open MuJoCo's passive viewer (needs a display). |
| `camera_topic` | empty | Publish an offscreen camera image on this topic. |
| `record_path` | empty | Record an MP4 here, in simulated time. |
| `record_camera` | the scenario's | `chase`, `follow`, `top` or `fixed`. |
| `obstacles` | the scenario's | `none`, a scenario name or path, or a YAML list such as `"[{type: box, pos: [2, 0], size: [0.2, 0.2, 0.4]}]"` (MuJoCo half sizes). |
| `real_time_factor` | `1.0` | Simulated seconds per wall second. |
| `initial_pose_x`, `_y`, `_yaw` | the scenario's | Start pose in the map. |

The ground examples also take `world` (a map in `vectorfield_bringup/maps` or a map YAML; its
walls are built in MuJoCo from the same image that map_server serves) and `use_composition`; the
drone takes `initial_pose_z` and `cmd_frame` (`global` or `body`).

Three generic launches take any robot, world, obstacles and scenario:
`mujoco_nav2.launch.py` (Nav2 and the controller), `mujoco_curve_follower.launch.py` and
`mujoco_drone.launch.py`. The robots are `tiago`, `scout_mini`, `tidybot`, `ackermann_car` and
`x2`:

```bash
ros2 launch vectorfield_mujoco mujoco_nav2.launch.py robot:=scout_mini world:=corridor_corner
```

## How an example is configured

The parameters are merged in this order, later files winning:

1. vectorfield_bringup's motion-model overlay (`config/controller/<model>.yaml`), as in the
   kinematic demos;
2. `vectorfield_mujoco/config/mujoco_sim.yaml`, shared by every example;
3. `config/robots/<robot>.yaml`: the robot's footprint, the controller and smoother limits
   within its actuators, the lidar;
4. the `params` of `config/scenarios/<example>.yaml`.

The scenario file also holds the start pose, the unmapped obstacles, the route (Nav2 waypoints,
or a curve and its laps) and how the gallery clip is filmed. `run_scenario` drives it once the
controller is up and exits with 0 when every goal succeeds.

The simulator node, `mujoco_sim`, has the same interface as the simulators it stands in for:
vectorfield_sim's for ground robots (`/clock`, `odom`, the `odom → base_footprint` TF, `scan`,
`initialpose`) and point_sim_3d's for the drone. Nav2 and the followers run unchanged.

## Regenerating the gallery

The clips on these pages and in the README are recordings of the examples, made headless by
`vectorfield_mujoco/scripts/record_gallery.sh` in the Jazzy devcontainer, with the workspace
built and sourced:

```bash
vectorfield_mujoco/scripts/record_gallery.sh          # all six examples
vectorfield_mujoco/scripts/record_gallery.sh drone    # one
```

Each example runs to the end of its scenario. ffmpeg then writes
`docs/_static/media/<example>.mp4` (960×540 H.264, at most 3 MiB), `.gif` (480 px, at most
2 MiB) and `.jpg` (the poster, at most 150 KiB); the script fails when a scenario does not
succeed or a file exceeds its budget. With OSMesa each example takes a few minutes.

## Licences and credits

| Model | Source | Licence |
|---|---|---|
| PAL TIAGo | MuJoCo Menagerie `pal_tiago` (fetched) | Apache-2.0 |
| Stanford TidyBot base | MuJoCo Menagerie `stanford_tidybot` (fetched) | MIT |
| Skydio X2 | MuJoCo Menagerie `skydio_x2` (fetched) | Apache-2.0 |
| AgileX Scout Mini | converted from AgileX's [ugv_gazebo_sim](https://github.com/agilexrobotics/ugv_gazebo_sim) by `tools/convert_scout_mini.py` and vendored in `vectorfield_mujoco/models/scout_mini` | BSD, as declared by its `package.xml`; the model's README lists every change |
| Car | `vectorfield_mujoco/models/ackermann_car`, built from primitives | Apache-2.0 |

Each fetched robot keeps the licence file of its Menagerie directory; the repository's `NOTICE`
records the attributions. Two models are idealised: the TidyBot base moves on planar joints
driven by velocity servos (an ideal holonomic base, no wheels), and the drone's inner loop is
this package's flight controller, standing in for an autopilot.
