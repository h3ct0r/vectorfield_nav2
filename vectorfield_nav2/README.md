# vectorfield_nav2

Metapackage for the vectorfield_nav2 stack: artificial vector-field navigation for Nav2, based on
Rezende, Gonçalves and Pimenta, *Constructive Time-Varying Vector Fields for Robot Navigation*
(IEEE T-RO 38(2), 2022).

**Status:** complete at M0. The packages it depends on are skeletons until their milestones.

## Contents

| Package | Purpose | Milestone |
|---|---|---|
| `vectorfield_msgs` | `Curve`, `FieldState` and `FollowCurve` interfaces | M0 |
| `vectorfield_core` | ROS-free C++17 library with all the field and tracking logic | M1 |
| `vectorfield_ros` | Distro compatibility, conversions, shared parameters, RViz/Foxglove visualization | M2 |
| `vectorfield_nav2_controller` | `nav2_core::Controller` plugin | M2 |
| `vectorfield_py` | Python bindings and command-line tools | M4 |
| `vectorfield_sim` | Kinematic simulator for headless tests and demos | M3 |
| `vectorfield_curve_follower` | `FollowCurve` action server for closed, parametric and moving curves | M5 |

`vectorfield_bringup` (launch files, overlays, maps, viewer configurations) and
`vectorfield_system_tests` are deliberately not included, as `navigation2` leaves out
`nav2_bringup`: installing the stack on a robot does not pull in simulators.

## Supported distros

Humble, Jazzy and Lyrical; Rolling is built nightly and is not gating. Kilted shares Jazzy's API
but is not supported. Binary packages are planned from the 0.1.0 release (M6); until then, build
from source with colcon.

Documentation: <https://h3ct0r.github.io/vectorfield_nav2/>
