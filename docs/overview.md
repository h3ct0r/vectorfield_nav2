# Overview

## What it is

vectorfield_nav2 implements the artificial vector field of Rezende, Gonçalves and Pimenta
{cite:p}`rezende2022constructive` as a set of ROS 2 packages that plug into Nav2
{cite:p}`macenski2020marathon,macenski2023desks`. At every position the field gives a velocity of
constant norm $v_r$. It blends convergence towards a curve with circulation along it, and it adds
a feedforward term when the curve itself moves. Integral curves of the field converge to the
target curve and then travel along it (see {doc}`theory`).

The project delivers two front ends on top of one shared core:

- **`vectorfield_nav2_controller::VectorFieldController`**, a `nav2_core::Controller` plugin.
  It follows the path from any Nav2 planner and can run under any `controller_server`.
- **`follow_curve`**, a `FollowCurve` action served by `vectorfield_curve_follower`. It follows
  curves that Nav2 cannot express as a path: closed curves for any number of laps, parametric
  and built-in shapes, and curves attached to a moving frame.

## Why

The ROS 1 reference implementation, `adrianomcr/vectorfield_stack`, is written in Python and
implements only the static field. It lacks the paper's key contribution, the time-varying
feedforward term $\Phi_T$ with the $\eta$ normalisation. Nav2 also requires controllers to be
C++ plugins. This project therefore re-implements the method in C++ directly from the equations
in Rezende's thesis {cite:p}`rezende2022thesis`. No upstream code is copied. The audit of the
upstream defects that motivated this decision is in the design documents
({doc}`design/index`).

## Features

Each feature lists the milestone that delivers it (see the {doc}`roadmap`).

- **Field** (M1): the static and time-varying field $\Phi = \eta\Phi_S + \Phi_T$, with `atan` or
  `rational` gain functions. A terminal field brings the robot to a stop at the end of an open
  path.
- **Motion models** (M1–M2): differential, skid-steer, omnidirectional and Ackermann. Feedback
  linearisation, heading tracking and holonomic tracking laws, with rotate-to-field,
  final-approach and rotate-to-heading phases.
- **Safety** (M1–M2): velocity limits that scale the field instead of clipping single axes, an
  acceleration limiter, and collision prediction by rollout on the local costmap with a braking
  cap.
- **Obstacle contouring** (M1–M3): the obstacle extension of the thesis (§3.4) on the costmap,
  gated so that it engages only when the path is blocked, with a latched side and a watchdog.
- **Visualization** (M2): RViz and Foxglove markers that show the phase, the field components,
  the prediction and the obstacle state. See {doc}`visualization`.
- **Curve follower** (M5): polylines, muParser expressions, built-in shapes (ellipse,
  lemniscates, superellipse, Lissajous, rounded rectangle, regular polygon) and moving frames.
- **Python** (M4): the `vectorfield` bindings and the `vf-plot`, `vf-sim`, `vf-tune` and
  `vf-curve` command-line tools.
- **Distros:** Humble, Jazzy and Lyrical, with Rolling built nightly. See {doc}`distro_matrix`.

## Packages

| Package | Contents |
|---|---|
| `vectorfield_core` | ROS-free C++17 library (Eigen 3, vendored muParser): curves, fields, tracking laws, limiter, occupancy grid, distance transform, collision prediction and the `Tracker`. Builds with plain CMake as well as ament. |
| `vectorfield_msgs` | `Curve.msg`, `FieldState.msg` and `FollowCurve.action`. |
| `vectorfield_ros` | Distro compatibility layer, conversions, shared parameters and validation, and the marker builder and visualizer. |
| `vectorfield_nav2_controller` | The `nav2_core::Controller` plugin and the legacy path handler for Humble–Kilted. |
| `vectorfield_curve_follower` | Lifecycle component serving the `follow_curve` action. |
| `vectorfield_py` | pybind11 bindings (`vectorfield`) and the `vectorfield_tools` command-line tools. |
| `vectorfield_sim` | Lightweight kinematic simulator built on the core integrator. |
| `vectorfield_bringup` | Launch files, parameter overlays, maps and RViz/Foxglove/PlotJuggler layouts. |
| `vectorfield_system_tests` | Cross-package and launch tests (never released). |
| `vectorfield_nav2` | Metapackage for the released runtime packages. |

{doc}`architecture` shows how the packages depend on each other.

## License and credits

The code is licensed under Apache-2.0. The method is due to Rezende, Gonçalves and Pimenta
{cite:p}`rezende2022constructive`, building on earlier work on vector fields for time-varying
curves {cite:p}`goncalves2010vector`. The repository's `NOTICE` file lists the attributions.
