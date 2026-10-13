# vectorfield_bringup

Launch files, Nav2 parameter overlays, maps and viewer configurations for running the
vectorfield_nav2 controller and curve follower in simulation, and for seeing in RViz what the
robot is doing.

**Status:** skeleton — the controller demos, overlays, maps and viewer configurations are
implemented in M3; the curve follower demo and scripts in M5.

## Planned contents

| Path | Contents | Milestone |
|---|---|---|
| `launch/` | `loopback_demo` (Jazzy+), `kinematic_sim_demo` (all distros), `tb3_gazebo_demo` and `tb4_gazebo_demo` (Jazzy+) | M3 |
| `launch/` | `curve_follower_demo`: the follower and its lifecycle manager | M5 |
| `config/controller/` | Motion-model overlays: `differential`, `skid_steer`, `omnidirectional`, `ackermann` | M3 |
| `config/scenario/` | Scenario overlays: `loopback`, `kinematic_sim`, `demo_obstacles`; `curve_follower` in M5 | M3 |
| `config/humble/` | `collision_monitor.yaml`, the only distro-specific file | M3 |
| `maps/` | `open_10x10` and `corridor_door` | M3 |
| `rviz/` | `vectorfield.rviz`, opened by every demo launch | M3 |
| `foxglove/` | `vectorfield.json`: 3D panel, plots and a phase/status panel | M3 |
| `plotjuggler/` | `vectorfield.xml`: the same plots | M3 |
| `vectorfield_bringup/` | Python module; `params.merge()` deep-merges the overlays onto nav2_bringup's `nav2_params.yaml` | M3 |
| scripts | `initial_pose_publisher` (M3); `send_curve.py` and `moving_frame_publisher.py` (M5) | M3, M5 |

Distro notes:

- The loopback and Gazebo demos need Jazzy or later and exit with a clear message on Humble.
  `kinematic_sim_demo` uses `vectorfield_sim` and runs everywhere.
- On Jazzy and later the command chain is Nav2's velocity smoother, then collision_monitor, then
  `cmd_vel`. On Humble the demos add collision_monitor with `config/humble/collision_monitor.yaml`
  (`cmd_vel` to `cmd_vel_safe`), and the simulator listens on `cmd_vel_safe`.

## What you will see in RViz (planned)

`rviz/vectorfield.rviz` shows the map, both costmaps, the global plan, the robot model and TF,
and the controller's markers, pre-configured with the shared colour legend:

- a status text above the robot whose colour follows the phase (FOLLOW green, ROTATE_TO_FIELD
  yellow, FINAL_APPROACH cyan, ROTATE_TO_HEADING blue, STOPPED red) and lists active flags such
  as `BLOCKED` or `CONTOUR ±1`;
- the field at the control point split into convergence (red), circulation (green), feedforward
  (blue) and the resulting field (white);
- the commanded velocity (cyan) and the predicted path, green up to the first collision;
- the goal with its tolerance and settle rings and the orange braking zone;
- during obstacle contouring, the blocking obstacle cells, the distance rings and the chosen
  side;
- optionally, the `field_grid` arrows that show the whole field around the robot.

The full legend is in `vectorfield_ros` and in the
[documentation](https://h3ct0r.github.io/vectorfield_nav2/).

## Packaging

`vectorfield_bringup` is not part of the `vectorfield_nav2` metapackage, so installing the
stack on a robot does not pull in the simulators. On Lyrical it has no installable RHEL 10 RPM,
because its Nav2 and Gazebo dependencies are not built for RHEL 10.
