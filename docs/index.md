---
hide-toc: true
---

# vectorfield_nav2

::::::{grid} 1 1 2 2
:gutter: 4
:class-container: vf-hero

:::::{grid-item}
:class: vf-hero-text

Smooth, convergent path and curve following for ROS 2 robots, built on constructive
time-varying vector fields.

A Nav2 controller plugin, a curve follower and a 3D follower for differential, skid-steer,
omnidirectional and Ackermann robots and for drones, on Humble, Jazzy and Lyrical.

::::{div} vf-buttons

```{button-ref} examples/index
:ref-type: doc
:color: primary

Run an example
```

```{button-ref} quickstart
:ref-type: doc
:color: primary
:outline:

Quickstart
```

```{button-link} https://github.com/h3ct0r/vectorfield_nav2
:color: primary
:outline:

{octicon}`mark-github` GitHub
```

::::
:::::

:::::{grid-item}

```{clip} differential
:alt: PAL TIAGo in MuJoCo drives along its Nav2 plan, contours around an unmapped box and rejoins the path
```

:::::
::::::

vectorfield_nav2 brings the constructive time-varying vector fields of Rezende, Gonçalves and
Pimenta {cite:p}`rezende2022constructive` to ROS 2 Nav2 {cite:p}`macenski2020marathon`. The
field makes the robot converge to a path or curve and then circulate along it at a set speed;
its feedforward term lets the robot follow curves that move. The code is a clean-room C++
implementation of the equations in Rezende's PhD thesis {cite:p}`rezende2022thesis`, with a
ROS-free core shared by every front end.

:::{admonition} Release status
:class: note

0.5.0 is released. The binary packages (`ros-<distro>-vectorfield-nav2`) wait on the ROS build
farm; until they are out, build from source ({doc}`quickstart`). The
[releases](https://github.com/h3ct0r/vectorfield_nav2/releases) list what each one changed.
:::

## Pick your robot

Each mode has a plug-and-play MuJoCo example: one command starts the physics, the controller,
RViz and a scenario that drives the robot by itself. The clips are recordings of exactly these
launches; {doc}`examples/index` lists what they need.

::::{grid} 1 2 2 2
:gutter: 3

:::{grid-item-card} Differential
:link: examples/differential
:link-type: doc
:class-card: vf-example

```{clip} differential
:alt: PAL TIAGo contours around an unmapped box and rejoins its path
```

**PAL TIAGo** · Nav2 controller, obstacle contour

```bash
ros2 launch vectorfield_mujoco differential.launch.py
```

:::

:::{grid-item-card} Skid-steer
:link: examples/skid_steer
:link-type: doc
:class-card: vf-example

```{clip} skid_steer
:alt: AgileX Scout Mini drives through a 0.8 m door and back
```

**AgileX Scout Mini** · Nav2 controller through a door, slip compensation

```bash
ros2 launch vectorfield_mujoco skid_steer.launch.py
```

:::

:::{grid-item-card} Omnidirectional
:link: examples/omnidirectional
:link-type: doc
:class-card: vf-example

```{clip} omnidirectional
:alt: Stanford TidyBot base strafes around a rounded rectangle at a fixed heading
```

**Stanford TidyBot base** · curve follower at a fixed heading

```bash
ros2 launch vectorfield_mujoco omnidirectional.launch.py
```

:::

:::{grid-item-card} Ackermann
:link: examples/ackermann
:link-type: doc
:class-card: vf-example

```{clip} ackermann
:alt: A front-steered car swerves around a box next to its path
```

**Car model** · Nav2 Smac Hybrid-A\* and the controller's CBF filter

```bash
ros2 launch vectorfield_mujoco ackermann.launch.py
```

:::

:::{grid-item-card} Drone
:link: examples/drone
:link-type: doc
:class-card: vf-example

```{clip} drone
:alt: Skydio X2 flies a helix around a pillar and a sphere
```

**Skydio X2** · 3D follower on a helix, CBF around obstacles

```bash
ros2 launch vectorfield_mujoco drone.launch.py
```

:::

:::{grid-item-card} Curve following
:link: examples/curve_following
:link-type: doc
:class-card: vf-example

```{clip} curve_following
:alt: AgileX Scout Mini laps a lemniscate
```

**AgileX Scout Mini** · curve follower, lemniscate laps

```bash
ros2 launch vectorfield_mujoco curve_following.launch.py
```

:::
::::

## Features

::::{grid} 1 2 3 3
:gutter: 3

:::{grid-item-card} {octicon}`git-branch;1.2em;sd-mr-1` Nav2 controller plugin
:link: motion_models
:link-type: doc
:class-card: vf-feature

`VectorFieldController` for any `controller_server`: differential, skid-steer,
omnidirectional and Ackermann models, three tracking laws, smooth goal handling.
:::

:::{grid-item-card} {octicon}`infinity;1.2em;sd-mr-1` Curve follower
:link: curve_follower
:link-type: doc
:class-card: vf-feature

The `follow_curve` action: closed, parametric and moving curves, built-in shapes and laps, with
an interlock with Nav2's own actions.
:::

:::{grid-item-card} {octicon}`paper-airplane;1.2em;sd-mr-1` 3D and drones
:link: follower_3d
:link-type: doc
:class-card: vf-feature

The `follow_curve_3d` action for robots that take a 3D velocity setpoint, with moving frames
and CBF obstacle avoidance (experimental).
:::

:::{grid-item-card} {octicon}`shield-check;1.2em;sd-mr-1` Obstacle avoidance
:link: obstacle_avoidance
:link-type: doc
:class-card: vf-feature

Collision prediction on the costmap, obstacle contouring around what blocks the path, and a
control-barrier-function safety filter.
:::

:::{grid-item-card} {octicon}`eye;1.2em;sd-mr-1` RViz and Foxglove
:link: visualization
:link-type: doc
:class-card: vf-feature

Markers that show the phase, the field's components, the command, the prediction and the
obstacle state, published only while someone looks.
:::

:::{grid-item-card} {octicon}`terminal;1.2em;sd-mr-1` Python tools
:link: tutorials/offline_design
:link-type: doc
:class-card: vf-feature

The `vectorfield` bindings and `vf-plot`, `vf-sim`, `vf-tune` and `vf-curve`: plot, simulate
and tune without ROS.
:::
::::

## Install

The packages build from source on Humble, Jazzy and Lyrical ({doc}`distro_matrix`).

::::{tab-set}

:::{tab-item} Devcontainer

Each distro has a devcontainer under `.devcontainer/` (`humble`, `jazzy`, `lyrical`), native on
Apple Silicon. Open the repository in VS Code, run **Dev Containers: Reopen in Container** and
pick a distro; the container runs rosdep. Then, from `/ws`:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch vectorfield_mujoco differential.launch.py
```

For RViz or the MuJoCo viewer, start the noVNC desktop of `.devcontainer/README.md` and
`export DISPLAY=:1` first, or view the camera image in Foxglove ({doc}`examples/index`).
:::

:::{tab-item} Source build

In a ROS 2 environment (`source /opt/ros/<distro>/setup.bash`):

```bash
mkdir -p ~/vf_ws/src && cd ~/vf_ws/src
git clone https://github.com/h3ct0r/vectorfield_nav2.git
cd ~/vf_ws
PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

`PIP_BREAK_SYSTEM_PACKAGES=1` lets rosdep install MuJoCo with pip on Ubuntu Noble and later,
for the MuJoCo examples only; {doc}`quickstart` shows how to leave them out.
:::

:::{tab-item} Python tools

The bindings and tools build with pip, without ROS (a C++17 compiler, CMake ≥ 3.22 and
Eigen ≥ 3.4):

```bash
git clone https://github.com/h3ct0r/vectorfield_nav2.git && cd vectorfield_nav2
python3 -m venv .venv && . .venv/bin/activate
pip install './vectorfield_py[tools]'
vf-sim --model differential --curve ellipse
```

:::
::::

## Citation

If you use this software, please cite the method it implements. GitHub's **Cite this
repository** button reads
[`CITATION.cff`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/CITATION.cff), which also
lists this implementation.

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
```

```{toctree}
:hidden:
:caption: Get started

overview
quickstart
distro_matrix
migration
```

```{toctree}
:hidden:
:caption: Examples

examples/index
```

```{toctree}
:hidden:
:caption: User guide

motion_models
visualization
obstacle_avoidance
curve_follower
behavior_tree
core_3d
follower_3d
tuning
parameters
```

```{toctree}
:hidden:
:caption: Tutorials

tutorials/getting_started
tutorials/scenarios
tutorials/curve_following
tutorials/your_robot
tutorials/field_testing
tutorials/offline_design
tutorials/aerial_3d
```

```{toctree}
:hidden:
:caption: Theory & design

theory
architecture
design/index
```

```{toctree}
:hidden:
:caption: Project

benchmarks
controller_comparison
contributing
api
```
