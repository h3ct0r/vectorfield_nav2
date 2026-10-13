# vectorfield_nav2_controller

A Nav2 controller plugin, `vectorfield_nav2_controller::VectorFieldController`
(`nav2_core::Controller`), that tracks the global plan with the artificial vector field of
Rezende, Gonçalves and Pimenta, *Constructive Time-Varying Vector Fields for Robot Navigation*
(IEEE T-RO 38(2), 2022). The field converges to the path and circulates along it; near the goal a
terminal field brakes the robot onto it.

**Status:** skeleton — implemented in M2 (plugin and in-process server-harness tests). Demos
and system tests follow in M3.

> **Not loadable yet.** The package exports no pluginlib description at M0. The plugin XML is
> added in M2 together with the library it names, because a description that points to a
> missing library breaks pluginlib class loading.

## Planned features

- Motion models: differential, skid-steer, omnidirectional and Ackermann
  (`motion_model`, read-only), with feedback-linearization, heading-tracking and holonomic
  tracking laws.
- Phases: rotate to the field, follow, final approach, rotate to the goal heading, stopped. The
  plugin defers to the goal checker for XY success.
- Costmap obstacle contouring (`obstacle_mode: contour`) and collision prediction that caps the
  speed before the robot reaches an obstacle; a blocked robot raises `NoValidControl` so Nav2's
  recovery can replan.
- `setSpeedLimit`, `cancel()` and `reset()` (Jazzy and later), dynamic parameters with
  validation, and use as the primary controller of `RotationShimController`.
- Visualization in RViz and Foxglove: status text, field components, command, prediction, goal
  and contour markers under `<ns>/markers`, where `<ns>` is the plugin name with `.` replaced
  by `/` (for example `FollowPath/markers`). See `vectorfield_ros` for the topic list.

## Plan input per distro

| Distros | Controller API | Plan window |
|---|---|---|
| Lyrical, Rolling | `newPathReceived` + 5-argument `computeVelocityCommands` | The controller server's transformed plan and global goal |
| Humble, Jazzy | `setPlan` + 3-argument `computeVelocityCommands` | `LegacyPathHandler`, a port of Nav2's `FeasiblePathHandler` |

A parity test checks `LegacyPathHandler` against `FeasiblePathHandler` from `nav2_controller`
on Lyrical.

## Configuration (planned)

Parameter names follow the plan and may change before M2. Defaults target nav2_bringup's
simulated robot at 20 Hz.

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "vectorfield_nav2_controller::VectorFieldController"
      motion_model: differential
      desired_speed: 0.4
      convergence_gain: 2.0
      max_linear_vel: 0.5
      max_angular_vel: 1.5
      obstacle_mode: contour  # default: none; the demos use contour
      visualize: true
```

The full parameter reference is generated from the parameter definitions and published in the
[documentation](https://h3ct0r.github.io/vectorfield_nav2/).
