# vectorfield_ros

The ROS 2 layer shared by `vectorfield_nav2_controller`, `vectorfield_curve_follower` and
`vectorfield_sim`. It connects the ROS-free `vectorfield_core` library to Nav2 on every supported
distro (Humble, Jazzy, Lyrical; Rolling nightly).

**Status:** complete. It is a library for those packages and has no node of its own.

## Features

- **`compat.hpp`**: header probes (`VF_HAS_ROS_COMMON`, `VF_HAS_PATH_HANDLER`,
  `VF_HAS_CONTROLLER_EXCEPTIONS`), `VF_CANCEL_RESET_OVERRIDE`, `compat::LifecycleNode`, a
  `TwistPublisher` with the per-distro stamped `cmd_vel` default, TF buffer, listener and
  broadcaster factories, and goal-checker helpers. Distro-specific code lives here; the only
  other places are the controller's API signature blocks and its test server harness.
- **`SingleGoalActionServer`**: the followers' action server, on rclcpp_action with one
  long-lived worker thread. It keeps the methods of Nav2's SimpleActionServer that the followers
  use, which can accept a goal and never run it when the goal arrives as its worker thread exits.
- **Conversions**: the plan window to a polyline curve, `Curve.msg` to a core curve
  specification, costmap snapshots to an occupancy grid, and the robot-frame footprint.
- **Shared parameters**: the `vectorfield_common_params` generate_parameter_library definition,
  the parameter-to-core binding table, the cross-parameter validator and `validateFromNode()`.
- **Visualization**: `MarkerBuilder`, pure functions that turn the tracker output into
  `visualization_msgs/MarkerArray`, and `FieldVisualizer`, which publishes them at the end of
  every control cycle.

## Visualization

The goal is that anyone watching RViz or Foxglove can tell, without reading logs, which phase
the robot is in and why, what the field commands, where the robot is heading, what it predicts
and what blocks it.

Publishing is controlled by the `visualize` parameter (default `true`), and each topic is
published only while it has subscribers. All markers use the control frame and the cycle stamp;
markers of namespaces that do not apply are deleted, so nothing stale remains.

| Topic | Type | Content |
|---|---|---|
| `<ns>/markers` | `MarkerArray` | The namespaces below; each can be toggled in RViz |
| `<ns>/field_grid` | `MarkerArray` | The field on a square around the robot, as arrows or streamlines (computed only while subscribed) |
| `<ns>/predicted_path` | `nav_msgs/Path` | The collision-prediction rollout |
| `<ns>/window` | `nav_msgs/Path` | The curve window the field uses |
| `<ns>/field_state` | `vectorfield_msgs/FieldState` | Per-cycle numbers for plots |
| `<ns>/obstacle_distance` | `nav_msgs/OccupancyGrid` | Distance transform, only with `publish_obstacle_distance` |

`<ns>` is the plugin name with `.` replaced by `/` (for example `FollowPath/markers`) in the
controller, and `~/` in the curve follower.

| Namespace | Shows |
|---|---|
| `status` | Text above the robot: phase, v, ω, distance D, arc length to go σ, active flags; coloured by phase |
| `footprint` | Robot outline: green normal, orange speed-capped, red blocked |
| `curve` | Exactly the geometry the field uses, with extension rays |
| `closest_point` | Projection on the curve, its tangent, and the distance |
| `control_point` | Where the field is evaluated |
| `field` | Convergence (red), circulation (green), feedforward (blue) and the resulting field (slate) |
| `command` | The commanded velocity and a 1 s arc of the resulting motion |
| `heading` | The target heading in the rotate phases |
| `prediction` | Rollout up to the first collision, the colliding pose, and the free distance |
| `goal` | Goal (purple), tolerance and settle rings, goal yaw, braking zone |
| `contour` | Obstacle contouring: blocking samples, obstacle cells, distance rings, chosen side, watchdog |
| `curve_info` | Curve follower only: traversal direction, start, laps, moving-frame velocity |

The colours form one legend shared by the code, the RViz configuration in
`vectorfield_bringup` and the documentation.

## CMake helper: `vf_link_ament_deps()`

`ament_target_dependencies()` is deprecated in Kilted and removed in Lyrical, and on Humble and
Jazzy some Nav2 packages (for example `nav2_core`) export include directories but no CMake
target. `vf_link_ament_deps()` covers both cases. For each package it links the imported targets
in `<pkg>_TARGETS`; when there are none it uses `<pkg>_INCLUDE_DIRS` (as `SYSTEM` includes),
`<pkg>_LIBRARIES` and `<pkg>_DEFINITIONS`.

```cmake
find_package(vectorfield_ros REQUIRED)  # also defines vf_link_ament_deps()
find_package(nav2_core REQUIRED)
find_package(rclcpp REQUIRED)

add_library(my_plugin SHARED src/my_plugin.cpp)
vf_link_ament_deps(my_plugin PUBLIC rclcpp nav2_core)
```

The signature is `vf_link_ament_deps(<target> <PUBLIC|PRIVATE|INTERFACE> <pkg>...)`. Every
package must already have been found with `find_package()`. The helper is installed to
`share/vectorfield_ros/cmake` and loaded through `ament_package(CONFIG_EXTRAS ...)`.

## Parameters

[`params/vectorfield_common_params.yaml`](params/vectorfield_common_params.yaml) defines the keys
shared by the controller plugin (prefixed with its plugin name) and the curve follower
(unprefixed). The generated reference is on the
[parameters page](https://h3ct0r.github.io/vectorfield_nav2/parameters.html) of the
documentation.

## Benchmarks

`bench/bench_marker_builder.cpp` (non-gating; `-DVF_BUILD_BENCHMARKS=ON` with google-benchmark
installed) times one full `markers` array (target < 0.2 ms) and the field grid. The reference
numbers are on the [benchmarks page](https://h3ct0r.github.io/vectorfield_nav2/benchmarks.html).
