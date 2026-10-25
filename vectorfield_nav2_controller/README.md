# vectorfield_nav2_controller

A Nav2 controller plugin, `vectorfield_nav2_controller::VectorFieldController`
(`nav2_core::Controller`), that tracks the global plan with the artificial vector field of
Rezende, Gonçalves and Pimenta, *Constructive Time-Varying Vector Fields for Robot Navigation*
(IEEE T-RO 38(2), 2022). The field converges to the path and circulates along it at a set
speed; near the goal a terminal field brakes the robot onto it. All control logic is in the
ROS-free `vectorfield_core`, which the curve follower and the Python tools share.

**Status:** complete, with demos and system tests. Humble, Jazzy and Lyrical;
Rolling nightly.

## Features

- Differential, skid-steer, omnidirectional and Ackermann bases (`motion_model`), with
  feedback-linearisation, heading-tracking and holonomic tracking laws.
- Phases: rotate to the field, follow, final approach, rotate to the goal heading, stopped. XY
  success is left to the goal checker, and the plugin never disagrees with it.
- A terminal field that stops on the goal instead of cutting the speed at the end of the path.
- Velocity limits that scale the field, an acceleration limiter, Nav2 speed limits
  (`setSpeedLimit`), and `cancel()` / `reset()` on Jazzy and later.
- Collision prediction on the local costmap with a braking cap. A blocked robot raises
  `NoValidControl`, so Nav2's behaviour tree replans or recovers.
- Optional obstacle contouring (`obstacle_mode: contour`), engaged only when the path ahead is
  blocked, with a latched side and a watchdog.
- Dynamic parameters, validated as a whole before they are applied.
- Works as the primary controller of `RotationShimController`.
- RViz and Foxglove markers that show the phase, the field components, the command, the
  prediction and the obstacle state.

## Usage

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "vectorfield_nav2_controller::VectorFieldController"
      motion_model: "differential"
      desired_speed: 0.4
      convergence_gain: 2.0
      max_linear_vel: 0.5
      max_angular_vel: 1.5
      obstacle_mode: "contour"  # default: none
```

Set the velocity smoother's limits at least as high as the controller's on every axis. The
demos of `vectorfield_bringup` run the plugin in simulation on every distro:

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py
```

## Parameters

Every key is prefixed with the plugin name (`FollowPath.<name>`; behind the rotation shim on
Lyrical and later, `FollowPath.primary_controller.<name>`). The common keys are defined in
[`vectorfield_ros/params/vectorfield_common_params.yaml`](../vectorfield_ros/params/vectorfield_common_params.yaml);
the legacy path-handling keys of Humble to Kilted (`prune_distance`,
`max_robot_pose_search_dist`, `transform_tolerance`, `reject_unit_path`) in
[`params/vectorfield_legacy_path_params.yaml`](params/vectorfield_legacy_path_params.yaml).
The generated [parameter reference](https://h3ct0r.github.io/vectorfield_nav2/parameters/controller.html)
lists every key; the [tuning guide](https://h3ct0r.github.io/vectorfield_nav2/tuning.html)
explains which to change.

| Group | Main keys |
|---|---|
| Field | `desired_speed`, `convergence_gain`, `gain_function` |
| Motion | `motion_model`, `tracking_law`, `control_point_offset`, `max_linear_vel`, `max_angular_vel`, accelerations, `min_turning_radius` |
| Phases | `use_rotate_to_field`, `rotate_to_field_angle`, `use_rotate_to_heading`, `terminal_braking_distance`, `min_approach_speed` |
| Obstacles | `obstacle_mode`, `obstacle_activation`, `obstacle_contour_distance`, watchdog limits |
| Safety | `use_collision_detection`, `collision_check_horizon`, `stop_margin` |
| Visualization | `visualize`, `field_grid_*`, `publish_obstacle_distance` |

## Topics

`<ns>` is the plugin name with `.` replaced by `/`, in the `controller_server` namespace
(`FollowPath` for the usual name). Every topic is published only while it has subscribers and
`visualize` is true.

| Topic | Type | Content |
|---|---|---|
| `<ns>/markers` | `visualization_msgs/MarkerArray` | status text, field components, command, prediction, goal and contour markers |
| `<ns>/field_grid` | `visualization_msgs/MarkerArray` | the field on a grid around the robot |
| `<ns>/predicted_path` | `nav_msgs/Path` | the collision-prediction rollout |
| `<ns>/window` | `nav_msgs/Path` | the plan window the field is built on |
| `<ns>/field_state` | `vectorfield_msgs/FieldState` | every per-cycle value, for plots |
| `<ns>/obstacle_distance` | `nav_msgs/OccupancyGrid` | the obstacle distance transform (with `publish_obstacle_distance`) |

The command goes out through `controller_server` (`cmd_vel`, remapped to `cmd_vel_nav` by the
stock launch). The marker legend is in the
[visualization guide](https://h3ct0r.github.io/vectorfield_nav2/visualization.html).

## Plan input per distro

| Distros | Controller API | Plan window |
|---|---|---|
| Lyrical, Rolling | `newPathReceived` + 5-argument `computeVelocityCommands` | the controller server's transformed plan and global goal |
| Humble, Jazzy | `setPlan` + 3-argument `computeVelocityCommands` | `LegacyPathHandler`, a port of Nav2's `FeasiblePathHandler` |

A parity test checks `LegacyPathHandler` against `FeasiblePathHandler` from `nav2_controller`
on Lyrical.

## Exceptions

| Exception | When |
|---|---|
| `NoValidControl` | blocked (no free path ahead, or the footprint in collision), contour watchdog, goal heading infeasible (Ackermann), non-finite command, or a configuration that failed to rebuild |
| `InvalidPath` | an empty plan, a plan with reversing segments (cusps), or a window with no usable segment |
| `ControllerTFError` | the plan cannot be transformed into the costmap frame |

Each is raised after the last command has been set to zero.

## Limitations

- Reversing segments (cusps) are rejected unless `allow_reversing: true`, which follows them
  segment by segment with a stop at each cusp (`max_reverse_vel` bounds the reverse speed);
  `drive_backward` drives a whole path backward.
- Obstacles are taken from the local costmap only, as a static snapshot per cycle.
- Ackermann: no in-place rotations; obstacle contouring uses distances that grow with
  `min_turning_radius` (and needs a 4 m local costmap at 0.5 m); recovery trees must not use Spin.
- With the rotation shim, obstacle contouring is supported only on Jazzy with
  `rotate_to_heading_once: true`.
- Planar only.
