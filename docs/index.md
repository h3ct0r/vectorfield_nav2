# vectorfield_nav2

vectorfield_nav2 brings the constructive time-varying vector fields of Rezende, Gonçalves and
Pimenta {cite:p}`rezende2022constructive` to ROS 2 Nav2 {cite:p}`macenski2020marathon`. It
provides `vectorfield_nav2_controller::VectorFieldController`, a `nav2_core::Controller` plugin
for any `controller_server`, and a `follow_curve` action server for closed, parametric and
moving curves. Both share a ROS-free C++ core, which also has Python bindings and tools. The
code is a clean-room C++ implementation of the equations in Rezende's PhD thesis
{cite:p}`rezende2022thesis`. It targets differential, skid-steer, omnidirectional and Ackermann
robots on Humble, Jazzy and Lyrical.

:::{admonition} Project status: milestone M0
:class: warning

The repository holds scaffolding, message definitions, design documents and this site. The core
library lands in M1, the Nav2 controller in M2 and the curve follower in M5. The
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
