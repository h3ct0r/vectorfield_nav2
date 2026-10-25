# vectorfield_nav2

[![CI](https://github.com/h3ct0r/vectorfield_nav2/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/h3ct0r/vectorfield_nav2/actions/workflows/ci.yml)
[![Docs](https://github.com/h3ct0r/vectorfield_nav2/actions/workflows/docs.yml/badge.svg?branch=main)](https://h3ct0r.github.io/vectorfield_nav2/)
[![codecov](https://codecov.io/gh/h3ct0r/vectorfield_nav2/branch/main/graph/badge.svg)](https://codecov.io/gh/h3ct0r/vectorfield_nav2)
[![License: Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

<p align="center">
  <a href="https://h3ct0r.github.io/vectorfield_nav2/examples/differential.html"><img src="docs/_static/media/differential.gif" width="480" alt="PAL TIAGo in MuJoCo follows its Nav2 plan with the vector-field controller, contours around an unmapped box and rejoins the path"></a>
</p>

Artificial vector-field navigation for ROS 2 and Nav2. vectorfield_nav2 implements the constructive time-varying vector fields of Rezende, Gonçalves and Pimenta ([IEEE T-RO 2022](https://doi.org/10.1109/TRO.2021.3093674)) as a Nav2 controller plugin and as a standalone curve-following action server, both built on a ROS-free C++ core. The field makes the robot converge to a path or curve and then circulate along it at a set speed; its time-varying feedforward term lets the robot follow curves that move. The implementation is a clean-room C++ rewrite from the equations in the paper and in Rezende's PhD thesis.

> [!NOTE]
> **Status: 0.5.0 is released.** The binary packages (`ros-<distro>-vectorfield-nav2`) wait on the ROS build farm; until they are out, build from source. The [documentation](https://h3ct0r.github.io/vectorfield_nav2/) has a quickstart, the examples, the theory, a tuning guide and the parameter reference.

## Examples

One command per mode starts MuJoCo physics, the controller, RViz and a scenario that drives the robot by itself. The clips are recordings of these launches; each links to its page in the documentation, with the configuration and how to adapt it to your robot. The examples need a source build ([requirements](https://h3ct0r.github.io/vectorfield_nav2/examples/index.html)).

<table>
<tr>
<td width="50%" valign="top">

[![PAL TIAGo contours around an unmapped box and rejoins its Nav2 path](docs/_static/media/differential.gif)](https://h3ct0r.github.io/vectorfield_nav2/examples/differential.html)

**Differential** · PAL TIAGo · Nav2 + obstacle contour

`ros2 launch vectorfield_mujoco differential.launch.py`

[launch](vectorfield_mujoco/launch/differential.launch.py) · [config](vectorfield_mujoco/config/robots/tiago.yaml) · [scenario](vectorfield_mujoco/config/scenarios/differential.yaml)

</td>
<td width="50%" valign="top">

[![AgileX Scout Mini drives through a 0.8 m door and back](docs/_static/media/skid_steer.gif)](https://h3ct0r.github.io/vectorfield_nav2/examples/skid_steer.html)

**Skid-steer** · AgileX Scout Mini · Nav2 through a door

`ros2 launch vectorfield_mujoco skid_steer.launch.py`

[launch](vectorfield_mujoco/launch/skid_steer.launch.py) · [config](vectorfield_mujoco/config/robots/scout_mini.yaml) · [scenario](vectorfield_mujoco/config/scenarios/skid_steer.yaml)

</td>
</tr>
<tr>
<td width="50%" valign="top">

[![Stanford TidyBot base strafes around a rounded rectangle at a fixed heading](docs/_static/media/omnidirectional.gif)](https://h3ct0r.github.io/vectorfield_nav2/examples/omnidirectional.html)

**Omnidirectional** · TidyBot base · curve follower, fixed heading

`ros2 launch vectorfield_mujoco omnidirectional.launch.py`

[launch](vectorfield_mujoco/launch/omnidirectional.launch.py) · [config](vectorfield_mujoco/config/robots/tidybot.yaml) · [scenario](vectorfield_mujoco/config/scenarios/omnidirectional.yaml)

</td>
<td width="50%" valign="top">

[![A front-steered car swerves around a box next to its path](docs/_static/media/ackermann.gif)](https://h3ct0r.github.io/vectorfield_nav2/examples/ackermann.html)

**Ackermann** · car model · Nav2 Smac Hybrid + CBF

`ros2 launch vectorfield_mujoco ackermann.launch.py`

[launch](vectorfield_mujoco/launch/ackermann.launch.py) · [config](vectorfield_mujoco/config/robots/ackermann_car.yaml) · [scenario](vectorfield_mujoco/config/scenarios/ackermann.yaml)

</td>
</tr>
<tr>
<td width="50%" valign="top">

[![Skydio X2 flies a helix around a pillar and a sphere](docs/_static/media/drone.gif)](https://h3ct0r.github.io/vectorfield_nav2/examples/drone.html)

**Drone** · Skydio X2 · 3D follower helix + CBF

`ros2 launch vectorfield_mujoco drone.launch.py`

[launch](vectorfield_mujoco/launch/drone.launch.py) · [config](vectorfield_mujoco/config/robots/x2.yaml) · [scenario](vectorfield_mujoco/config/scenarios/drone.yaml)

</td>
<td width="50%" valign="top">

[![AgileX Scout Mini laps a lemniscate with the curve follower](docs/_static/media/curve_following.gif)](https://h3ct0r.github.io/vectorfield_nav2/examples/curve_following.html)

**Curve following** · AgileX Scout Mini · lemniscate laps

`ros2 launch vectorfield_mujoco curve_following.launch.py`

[launch](vectorfield_mujoco/launch/curve_following.launch.py) · [config](vectorfield_mujoco/config/robots/scout_mini.yaml) · [scenario](vectorfield_mujoco/config/scenarios/curve_following.yaml)

</td>
</tr>
</table>

## Features

- **Nav2 controller plugin** `vectorfield_nav2_controller::VectorFieldController` for any Nav2 `controller_server`, with differential, skid-steer, omnidirectional and Ackermann motion models and three tracking laws (feedback linearization, heading tracking, holonomic).
- **Smooth goal handling:** a terminal field that brakes onto the goal, rotate-to-field and rotate-to-heading phases, and cancel support on Jazzy and later.
- **Obstacle handling from the costmap:** collision prediction with a speed cap, a per-cycle blocked status that Nav2's recovery can act on, and optional obstacle contouring (latched circulation around costmap obstacles).
- **More obstacle modes and plans:** a control-barrier-function obstacle mode (`obstacle_mode: cbf`), obstacle contouring for Ackermann bases, and plans that reverse (cusps) with Nav2's path constraints.
- **A `FollowCurve` behaviour-tree node** (`vectorfield_behavior_tree`) and an experimental **3D core and follower** for drones (`vectorfield_core::d3`, `vectorfield.d3`, `vf-sim3d`, `vectorfield_follower_3d`).
- **`follow_curve` action server** for closed, parametric and moving curves: polylines, muParser expressions in `s` and `t`, and built-in shapes (ellipse, lemniscates, superellipse, Lissajous, rounded rectangle, regular polygon), with laps, moving curve frames and an interlock with Nav2's own actions.
- **RViz and Foxglove visualization** that shows what the robot is doing: the current phase and active flags as colour-coded text, the field broken into the paper's convergence, circulation and feedforward components, the command sent, the predicted trajectory and first collision, the goal and braking zone, and the obstacle-contouring state. Markers are published only while someone subscribes. See the [visualization guide](https://h3ct0r.github.io/vectorfield_nav2/visualization.html).
- **Python bindings and tools** (`vectorfield`, plus `vf-plot`, `vf-sim`, `vf-tune` and `vf-curve`) for plotting fields, simulating and tuning without ROS.
- **Documentation** with the theory, a tuning guide, a generated parameter reference and a [migration guide](https://h3ct0r.github.io/vectorfield_nav2/migration.html) from `vectorfield_stack`.

## Packages

| Package | Contents |
|---|---|
| `vectorfield_core` | ROS-free C++17 library (Eigen, vendored muParser): curves, vector fields, tracking laws, command limiter, obstacle contouring, collision prediction. Also builds with plain CMake. |
| `vectorfield_msgs` | `Curve.msg`, `FieldState.msg` and `FollowCurve.action`; the experimental `Curve3.msg`, `FieldState3.msg` and `FollowCurve3.action`. |
| `vectorfield_ros` | Shared ROS 2 layer: distro compatibility, conversions, parameters and validation, RViz marker builder. |
| `vectorfield_nav2_controller` | The `nav2_core::Controller` plugin. |
| `vectorfield_curve_follower` | Lifecycle component serving the `follow_curve` action. |
| `vectorfield_follower_3d` | [experimental] Lifecycle component serving the `follow_curve_3d` action (the 3D core, for drones), and the `point_sim_3d` simulator. |
| `vectorfield_behavior_tree` | Nav2 behaviour-tree node `FollowCurve` for the `follow_curve` action, and `bt_runner`. |
| `vectorfield_py` | pybind11 bindings (`vectorfield`) and command-line tools (`vectorfield_tools`). |
| `vectorfield_sim` | Kinematic simulator for headless tests and demos on every distro, including Humble. |
| `vectorfield_bringup` | Launch files, parameter overlays, maps, RViz/Foxglove/PlotJuggler configs, example clients. |
| `vectorfield_mujoco` | The MuJoCo examples: the `mujoco_sim` simulator node, robot models and configs, one launch per mode. Source-only (MuJoCo from pip). |
| `vectorfield_system_tests` | Cross-package tests, launch tests and config validation (never released). |
| `vectorfield_nav2` | Metapackage: msgs, core, ros, controller, curve follower, 3D follower, behaviour-tree node, py and sim. |

## Architecture

Package dependencies (an arrow points to the package depended on):

```mermaid
flowchart BT
  core["vectorfield_core<br/>ROS-free C++17"]
  msgs["vectorfield_msgs"]
  ros["vectorfield_ros"]
  ctrl["vectorfield_nav2_controller"]
  follower["vectorfield_curve_follower"]
  follower3d["vectorfield_follower_3d"]
  bt["vectorfield_behavior_tree"]
  sim["vectorfield_sim"]
  py["vectorfield_py"]
  bringup["vectorfield_bringup"]
  tests["vectorfield_system_tests"]
  mujoco["vectorfield_mujoco<br/>source only"]

  ros --> core
  ros --> msgs
  ctrl --> ros
  follower --> ros
  follower3d --> follower
  bt --> msgs
  bt -.->|build only| ros
  sim --> ros
  py --> core
  bringup --> ctrl
  bringup --> follower
  bringup --> follower3d
  bringup --> bt
  bringup --> sim
  bringup --> py
  tests --> bringup
  mujoco --> bringup
```

All controller logic lives in `vectorfield_core`, so the Nav2 plugin and the curve follower run the same tracker. No package depends on a package above it; tests that need several packages live in `vectorfield_system_tests`. The `vectorfield_nav2` metapackage groups the released runtime packages and leaves out bringup, the MuJoCo examples and system tests.

## Supported ROS 2 distributions

| Distribution | Ubuntu | Nav2 controller API | CI | Status |
|---|---|---|---|---|
| Humble | 22.04 jammy | `setPlan` | every PR (amd64) | Supported |
| Jazzy | 24.04 noble | `setPlan`, plus `cancel` / `reset` | every PR (amd64 with coverage, arm64) | Supported |
| Lyrical | 26.04 resolute | path handler (`newPathReceived`) | every PR (amd64) | Supported |
| Rolling | 26.04 resolute | as Lyrical | nightly, non-gating | Best effort |
| Kilted | 24.04 noble | as Jazzy | none | Unsupported: may build, but has no CI, config or release |

Releases are made with bloom for Humble, Jazzy and Lyrical; `vectorfield_mujoco` is not released, because the build farm does not take its pip dependency (MuJoCo). Nav2 is not released into Rolling, so Rolling is tested against the Nav2 nightly image.

## Quickstart

### Devcontainers (all distros)

Each distribution has its own devcontainer under [`.devcontainer/`](.devcontainer/) (`humble`, `jazzy`, `lyrical`, `rolling`), based on the official `ros:<distro>-ros-base` images (Rolling: the Nav2 nightly image), native on Apple Silicon. Open the repository in VS Code, run **Dev Containers: Reopen in Container** and pick a distribution. Then, from the workspace root (`/ws`):

```bash
colcon build --symlink-install
colcon test && colcon test-result --verbose
```

The Rolling container is for compiling and unit tests only; run demos in the Lyrical container.

### Demo (any distro)

From a sourced workspace, the kinematic simulator demo starts Nav2 with the vector-field controller and RViz with the pre-configured markers; send a goal with **2D Goal Pose**:

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py              # every distro
ros2 launch vectorfield_bringup loopback_demo.launch.py                   # Jazzy and later
ros2 launch vectorfield_bringup curve_follower_demo.launch.py             # then:
ros2 run vectorfield_bringup send_curve.py --shape lemniscate_bernoulli --laps 3
```

The [quickstart](https://h3ct0r.github.io/vectorfield_nav2/quickstart.html) lists goals to try and what to look for in RViz. The [tutorials](https://h3ct0r.github.io/vectorfield_nav2/tutorials/getting_started.html) go from a fresh machine to every demo scenario, curve following, offline design with the Python tools, and the [controller on your own robot](https://h3ct0r.github.io/vectorfield_nav2/tutorials/your_robot.html).

### Using the controller

Configure the plugin like any Nav2 controller:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "vectorfield_nav2_controller::VectorFieldController"
      motion_model: differential
      desired_speed: 0.4
      convergence_gain: 2.0
      obstacle_mode: contour
```

Every parameter is in the [parameter reference](https://h3ct0r.github.io/vectorfield_nav2/parameters.html). Annotated templates to copy into your own `nav2_params.yaml` (differential, omnidirectional, Ackermann) are in [`vectorfield_bringup/config/examples/`](vectorfield_bringup/config/examples/).

### Native core build and Python tools (macOS)

`vectorfield_core` and the Python tools build without ROS. With the Homebrew dependencies installed (`brew install cmake ninja eigen googletest google-benchmark`):

```bash
cd vectorfield_core && cmake --preset dev && cmake --build --preset dev && ctest --preset dev
cd .. && python3 -m venv .venv && . .venv/bin/activate
pip install './vectorfield_py[tools]'
vf-sim --model differential --curve ellipse
```

Other core presets: `asan` (ASan + UBSan), `tsan`, `coverage` and `bench`.

## Citation

If you use this software, please cite the method it implements. GitHub's **Cite this repository** button reads [`CITATION.cff`](CITATION.cff), which also lists this implementation.

```bibtex
@article{rezende2022constructive,
  author  = {Rezende, Adriano M. C. and Gon{\c{c}}alves, Vinicius M. and Pimenta, Luciano C. A.},
  title   = {Constructive Time-Varying Vector Fields for Robot Navigation},
  journal = {IEEE Transactions on Robotics},
  year    = {2022},
  volume  = {38},
  number  = {2},
  pages   = {852--867},
  doi     = {10.1109/TRO.2021.3093674}
}

@phdthesis{rezende2022thesis,
  author  = {Rezende, Adriano Martins da Costa},
  title   = {Vector field based guidance and control strategies for robot navigation},
  school  = {Universidade Federal de Minas Gerais},
  address = {Belo Horizonte, Brazil},
  year    = {2022},
  url     = {https://hdl.handle.net/1843/73611}
}
```

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for the development setup, coding style, tests and the support policy. Please follow the [Code of Conduct](CODE_OF_CONDUCT.md), and report security issues as described in [SECURITY.md](SECURITY.md).

## Credits and license

The method is by Adriano M. C. Rezende, Vinicius M. Gonçalves and Luciano C. A. Pimenta. The design was inspired by [`vectorfield_stack`](https://github.com/adrianomcr/vectorfield_stack), the ROS 1 implementation that accompanies the thesis; no code from it is included. Maintained by Héctor Azpúrua.

Licensed under the [Apache License 2.0](LICENSE). See [NOTICE](NOTICE) for attributions, including the adapted and vendored third-party code (Nav2's FeasiblePathHandler, muParser).
