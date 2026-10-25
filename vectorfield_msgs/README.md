# vectorfield_msgs

ROS 2 interface definitions for the vectorfield_nav2 stack.

**Status:** complete; `FieldState`, `Curve` and `FollowCurve` are frozen. Once
released into a distro, the definitions evolve only by adding new types.

## Interfaces

| Interface | Used by | Purpose |
|---|---|---|
| `msg/Curve` | `FollowCurve` goal; the follower's `~/curve` topic | A curve in its own frame: a polyline, a muParser expression r(s, t), or a built-in closed shape |
| `msg/FieldState` | `<ns>/field_state`, published every control cycle by the controller plugin and the curve follower | Phase, status flags, closest point, field decomposition and commands, for plots and debugging |
| `action/FollowCurve` | the `follow_curve` action of `vectorfield_curve_follower` | Follow a closed, parametric or time-varying curve, optionally for a number of laps |
| `msg/Curve3` | `FollowCurve3` goal; the 3D follower's `~/curve` topic | [0.3.0, experimental] A curve in R^3: a 3D polyline, expressions x/y/z(s, t), or a built-in shape placed by a 6-DoF pose |
| `msg/FieldState3` | `follower_3d/field_state`, every control cycle of `vectorfield_follower_3d` | [0.3.0, experimental] Pose, closest point, field, CBF barrier and commands of the 3D follower |
| `action/FollowCurve3` | the `follow_curve_3d` action of `vectorfield_follower_3d` | [0.3.0, experimental] FollowCurve for robots that take a 3D velocity setpoint (drones) |

### Curve

- `type` selects `POLYLINE` (`points`), `EXPRESSION` (`x_expression`, `y_expression` over
  `[s_min, s_max]`) or `BUILTIN` (`builtin_shape`).
- `header.frame_id` is the curve frame. `header.stamp` is t0 of r(s, t); zero means goal
  acceptance. The header is the only frame and time reference of a goal.
- Built-in shapes: `ellipse`, `lemniscate_bernoulli`, `lemniscate_gerono`, `superellipse`,
  `rounded_rectangle`, `lissajous`, `regular_polygon`. All are closed. With `s_min = s_max = 0`
  they use their natural domain.
- `parameter_names` / `parameter_values` bind named constants for expressions and built-ins.
- `sample_spacing <= 0` uses the node default; `max_corner_deviation <= 0` disables corner
  smoothing.

### FollowCurve

Goal: `frame_motion` (`FRAME_STATIC` or `FRAME_MOVING`), `frame_odom_topic` (moving frames:
Odometry of the curve frame, `""` = TF differencing), `reverse_direction`, `desired_speed`
(`<= 0` uses the parameter), `laps` (closed curves only; 0 = until cancelled) and
`max_duration` (0 = unlimited).

Result `error_code`:

| Code | Name | Meaning |
|---|---|---|
| 0 | `NONE` | Succeeded: laps done, or an open curve's end reached and settled |
| 1 | `GOAL_REJECTED` | Client side, never in a result (Nav2's value): the follower rejected the goal because it is not active |
| 2 | `SEND_GOAL_FAILURE` | Client side, never in a result (Nav2's value): the client could not send the goal |
| 1000 | `UNKNOWN` | Non-finite command, deactivation during a goal, or a failed parameter rebuild |
| 1001 | `INVALID_CURVE` | The curve was rejected before the first command |
| 1002 | `UNSUPPORTED` | Open time-varying curve, Odometry child frame differs from the curve frame, TF mode without a reference frame, Nav2 interlock cannot see `follow_path`, or heading infeasible |
| 1003 | `TF_ERROR` | The curve frame was not transformable for `stale_abort_timeout` |
| 1004 | `STALE_INPUT` | Pose, odometry, costmap, footprint or moving-frame input stale for `stale_abort_timeout` |
| 1005 | `BLOCKED` | Blocked for `blocked_timeout`, no progress within `progress_timeout`, or the contour watchdog fired |
| 1006 | `TIMEOUT` | `max_duration` reached; a client such as the FollowCurve BT node also reports it when the follower does not acknowledge a goal within its `server_timeout` |
| 1007 | `ASSUMPTION3_VIOLATED` | The curve moves too fast to be tracked at the allowed speed for longer than `assumption3_abort_time` |
| 1008 | `NAV2_ACTIVE` | A Nav2 motion action was active at goal start or became active during the goal |
| 1009 | `INVALID_GOAL` | The goal-scoped configuration was rejected, e.g. `desired_speed > max_linear_vel` |

Feedback: `distance_to_curve`, `arc_length`, `remaining_arc_length` (+Inf when `laps` is 0),
`lap`, `speed` and `phase` (a `FieldState.PHASE_*` value).

### FieldState

- `phase`: `PHASE_ROTATE_TO_FIELD` (0), `PHASE_FOLLOW` (1), `PHASE_FINAL_APPROACH` (2),
  `PHASE_ROTATE_TO_HEADING` (3), `PHASE_STOPPED` (4).
- `status` is a bit mask: `NON_FINITE` 1, `ASSUMPTION3_CLAMPED` 2, `AMBIGUOUS_CLOSEST` 4,
  `HAIRPIN` 8, `CURVATURE_SATURATED` 16, `CONTOUR_WATCHDOG` 32, `HEADING_INFEASIBLE` 64,
  `COLLISION_AHEAD` 128, `BLOCKED` 256 (each prefixed with `STATUS_`).
- All vectors are expressed in `header.frame_id`. `distance_to_end` is NaN for closed curves;
  `obstacle_distance` and `free_arc_length` are +Inf when there is no obstacle or collision.

The phase and status constants are checked at compile time (`static_assert`) against the enums
in `vectorfield_core`.

### Curve3, FollowCurve3 and FieldState3 (experimental)

[0.3.0] The interfaces of the 3D follower (`vectorfield_follower_3d`). They may change in a
minor release while the 3D stack is experimental.

- `Curve3` is `Curve` in R^3. It has 3D `points`, a third expression `z_expression`, and the
  3D built-ins `helix` (open), `circle`, `ellipse`, `viviani`, `lemniscate3d` and
  `torus_knot`. `pose` places a built-in in the curve frame; an all-zero quaternion means the
  identity, and the pose must be the identity for polylines and expressions. A pose together
  with the `cx cy cz yaw pitch roll` parameters is rejected. It has no corner smoothing.
- `FollowCurve3` has FollowCurve's goal fields, result codes and feedback. `NAV2_ACTIVE` is
  reserved, because the 3D follower has no Nav2 interlock. `speed` is the 3D speed.
- `FieldState3` uses FieldState's phase and status values. `barrier` is the least CBF barrier
  (+Inf without obstacles), and the commands are in `header.frame_id` (the follower's
  `global_frame`).

See the [documentation](https://h3ct0r.github.io/vectorfield_nav2/) for the theory behind the
field quantities (`g`, `h`, `eta`, `feedforward`, `field`).
