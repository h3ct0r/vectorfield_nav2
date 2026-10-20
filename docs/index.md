# vectorfield_nav2

vectorfield_nav2 brings the constructive time-varying vector fields of Rezende, Gonçalves and
Pimenta {cite:p}`rezende2022constructive` to ROS 2 Nav2 {cite:p}`macenski2020marathon`. It
provides `vectorfield_nav2_controller::VectorFieldController`, a `nav2_core::Controller` plugin
for any `controller_server`, and a `follow_curve` action server for closed, parametric and
moving curves. Both share a ROS-free C++ core, which also has Python bindings and tools. The
code is a clean-room C++ implementation of the equations in Rezende's PhD thesis
{cite:p}`rezende2022thesis`. It targets differential, skid-steer, omnidirectional and Ackermann
robots on Humble, Jazzy and Lyrical.

:::{admonition} Project status: 0.1.0 in preparation
:class: note

The core library, the Nav2 controller, the simulator and demos, the Python tools and the curve
follower are complete (milestones M0 to M5). M6, this documentation and the 0.1.0 release, is
in progress; until the binary packages are released, build from source ({doc}`quickstart`). The
{doc}`roadmap` lists what each milestone delivers.
:::

```{toctree}
:caption: User guide
:maxdepth: 1

overview
quickstart
motion_models
visualization
obstacle_avoidance
curve_follower
tuning
parameters
distro_matrix
migration
```

```{toctree}
:caption: Tutorials
:maxdepth: 1

tutorials/getting_started
tutorials/scenarios
tutorials/curve_following
tutorials/your_robot
tutorials/offline_design
```

```{toctree}
:caption: Theory & design
:maxdepth: 1

theory
architecture
design/index
```

```{toctree}
:caption: Project
:maxdepth: 1

benchmarks
controller_comparison
roadmap
contributing
api
```
