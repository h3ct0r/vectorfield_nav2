# Curve follower

`vectorfield_curve_follower` serves the `follow_curve` action (`vectorfield_msgs/action/FollowCurve`).
It drives the robot along curves that a Nav2 path cannot express: closed curves for a number of
laps, parametric curves and built-in shapes, and curves attached to a moving frame. It runs the
same `Tracker` as the controller plugin, with ROTATE_TO_HEADING disabled. Nav2's planner is
bypassed: the curve itself is the reference.

The node is the lifecycle component `vectorfield_curve_follower::CurveFollower` (node name
`curve_follower`; executable `curve_follower`). Start it with a `nav2_lifecycle_manager`
(`node_names: [curve_follower]`, `autostart: true`), as `curve_follower_demo` does, or by hand
with `ros2 lifecycle set /curve_follower configure` and then `activate`. {doc}`quickstart` runs
the demo.

## The action

**Goal.**

| Field | Meaning |
|---|---|
| `curve` | the `vectorfield_msgs/Curve` (below) |
| `frame_motion` | `FRAME_STATIC` (0) or `FRAME_MOVING` (1) |
| `frame_odom_topic` | `FRAME_MOVING`: an Odometry topic of the curve frame; empty selects TF differencing |
| `reverse_direction` | traverse the curve against its parameter direction |
| `desired_speed` | reference speed for this goal; ≤ 0 uses the `desired_speed` parameter |
| `laps` | closed curves only: laps to drive; 0 runs until cancelled |
| `max_duration` | 0 is unlimited; otherwise the goal ends with `TIMEOUT` |

**Feedback:** `distance_to_curve`, `arc_length`, `remaining_arc_length` (closed curve with laps:
$(\mathrm{laps} - p)L$; `laps` = 0: $+\infty$; open curve: the arc length to the end), `lap`,
`speed` and `phase` (a `FieldState.PHASE_*` value).

**Result:** `error_code`, `error_msg`, `laps_completed`, `distance_travelled` and
`total_elapsed_time`.

**Curves.** `Curve.type` selects one of three forms:

- `POLYLINE`: `points` (float64; z is ignored) and `closed`. The follower removes duplicates,
  resamples at `sample_spacing` (≤ 0: the `default_sample_spacing` parameter, 0.05 m) and can
  fillet corners within `max_corner_deviation` (≤ 0: no smoothing). Both endpoints are kept.
- `EXPRESSION`: `x_expression` and `y_expression` in muParser syntax, in the variable `s` on
  `[s_min, s_max]`, the time `t` for time-varying curves, and the constants declared in
  `parameter_names` and `parameter_values`. Use `^` for powers and `_pi` for π.
- `BUILTIN`: `builtin_shape` with optional parameters; all built-ins are closed and use
  $s \in [0, 2\pi)$ when `s_min` = `s_max` = 0. Every shape also takes the pose parameters `cx`,
  `cy` and `yaw` (radians), which default to 0.

| `builtin_shape` | Parameters (defaults) |
|---|---|
| `ellipse` | `a` 1, `b` 1 |
| `lemniscate_bernoulli` | `a` 1 |
| `lemniscate_gerono` | `a` 1, `b` 0.5 |
| `superellipse` | `a` 1, `b` 1, `n` 4 ($n \ge 2$) |
| `lissajous` | `ax` 1, `ay` 1, `fx` 1, `fy` 2, `phase` $\pi/2$ ($f_x$, $f_y$ coprime positive integers) |
| `rounded_rectangle` | `width` 2, `height` 1, `radius` 0.2 |
| `regular_polygon` | `radius` 1 (circumradius), `sides` 6, `corner_radius` 0.1 |

The shapes' equations are in {ref}`sec-builtins`. Curves are validated before the first command:
an irregular curve (a stationary point or a tangent reversal), a curve marked closed that does not
close, an unknown shape or parameter, or more than `max_curve_points` (20000) vertices after
conditioning ends the goal with `INVALID_CURVE`.

## Frames and time

`Curve.header.frame_id` is the curve frame. The follower controls in `global_frame` (`map`) and
looks up the robot (`robot_base_frame`, `base_link`) and the curve frame there every cycle; every
marker is published in `global_frame`. `Curve.header.stamp` is $t_0$ of $r(s, t)$: $t$ in an
expression is the time since $t_0$, and a zero stamp means the time of goal acceptance.

A **moving frame** (`FRAME_MOVING`) adds the frame's motion to the field as the feedforward term
$\Phi_T$ (see {doc}`theory`). The frame's twist comes either from `frame_odom_topic`, an Odometry
whose child frame must be the curve frame, or, when that is empty, from TF differencing of the
curve frame against `moving_frame_reference`. TF differencing uses exact SE(2) log differences
of distinct stamps, rejects speeds above `frame_twist_max_speed` (2 m/s), low-pass filters the
result at `frame_twist_cutoff_hz` (2 Hz) and extrapolates the frame for at most `input_timeout`
(0.3 s). The reference frame's chain to the curve frame must not contain `map` → `odom`, whose
localization jumps would read as frame motion; the demo uses `odom`. With an empty
`moving_frame_reference` and no Odometry topic, a moving goal is rejected with `UNSUPPORTED`.

**Assumption 3** of the method requires the curve to move slower than the robot. The feedforward
is clamped to $(1 - \varepsilon_v)$ of the reference speed (`assumption3_margin` $\varepsilon_v$ =
0.1), shown as `A3-CLAMP` in the status text; a clamp that lasts `assumption3_abort_time` (1 s)
outside a block ends the goal with `ASSUMPTION3_VIOLATED`. Open time-varying curves are not
supported.

## Laps and success

The follower tracks the progress $p$ along a closed curve in laps, signed by the traversal
direction, from the robot's closest point at goal start; `laps_completed` is $\lfloor p\rfloor$.
A closed curve succeeds when `laps` > 0 and `laps_completed` ≥ `laps`; with `laps` = 0 it runs
until cancelled. An open curve succeeds when the robot has settled at its end, inside
`goal_tolerance` (0.1 m); with `reverse_direction` the end is the first point. `laps` > 0 on an
open curve is `INVALID_CURVE`.

`send_curve.py` drives the robot to the curve with NavigateToPose before it sends the goal (an
open curve: to its start), because the follower converges from anywhere but takes the shortest
way, through whatever lies between.

## Nav2 interlock

The follower and Nav2's own motion actions must never command the base at the same time. The
follower subscribes to the status topic of every action in `exclusive_actions` (by default
`follow_path`, `navigate_to_pose`, `navigate_through_poses`, `spin`, `backup`,
`drive_on_heading`, `assisted_teleop`, `dock_robot`, `undock_robot` and `follow_object`). An action
counts as active while any of its goals is accepted, executing or cancelling.

- A goal does not start while one is active: it ends with `NAV2_ACTIVE` before any command.
- When one becomes active during a goal, the follower sends one zero command, goes silent and
  ends the goal with `NAV2_ACTIVE`.
- With `require_nav2_interlock` (default `true`), a goal is refused with `UNSUPPORTED` when
  `follow_path/_action/status` has no publisher: the interlock cannot see Nav2, usually because
  of a wrong namespace or remapping.

The interlock covers actions only. Docking (Jazzy and later) and following (Lyrical and later)
publish straight to `cmd_vel`, as documented by Nav2.

## Safety contract

- The follower publishes `cmd_vel_nav` only while a goal executes. On termination it decelerates
  within the limits, sends one zero command and goes silent; at goal start, a goal that fails
  validation publishes nothing.
- `cmd_vel_nav` must go through Nav2's velocity smoother and collision_monitor, as the
  controller's does. Never remap it to the base's command topic.
- Stale inputs make the robot decelerate: a pose or odometry older than `input_timeout`, a gap of
  more than `costmap_timeout` (0.5 s) between costmaps, no costmap or footprint yet, or a stale
  moving frame. If it lasts `stale_abort_timeout` (2 s) the goal ends with `STALE_INPUT`, or
  `TF_ERROR` for an untransformable curve frame.
- A cancel decelerates to zero and then sends one zero; the goal ends canceled.

## Failures

| Condition | Response | Result code |
|---|---|---|
| Invalid curve: fewer than 2 points, non-finite values, `s_min` ≥ `s_max`, parse error, size limits, unknown shape or parameter, irregular, declared closed but not closing, `laps` > 0 on an open curve | ends before the first command | `INVALID_CURVE` (1001) |
| Goal-scoped configuration rejected, e.g. `desired_speed` > `max_linear_vel` | ends before the first command | `INVALID_GOAL` (1009) |
| Open time-varying curve; Odometry child frame ≠ curve frame; TF differencing without `moving_frame_reference`; the interlock cannot see `follow_path` | ends before the first command | `UNSUPPORTED` (1002) |
| A Nav2 action active at goal start | ends before publishing anything | `NAV2_ACTIVE` (1008) |
| A Nav2 action becomes active during the goal | one zero command, then silent | `NAV2_ACTIVE` (1008) |
| Curve frame not transformable | zero command | `TF_ERROR` (1003) after `stale_abort_timeout` |
| Stale pose, odometry, costmap, footprint or moving frame | decelerate | `STALE_INPUT` (1004) after `stale_abort_timeout` |
| Blocked, or no progress | stop and wait | `BLOCKED` (1005) after `blocked_timeout` (10 s) or `progress_timeout` (10 s) |
| Contour watchdog | decelerate | `BLOCKED` (1005) |
| Goal heading infeasible (Ackermann) | decelerate | `UNSUPPORTED` (1002) |
| Feedforward clamped for `assumption3_abort_time` | decelerate | `ASSUMPTION3_VIOLATED` (1007) |
| Non-finite command, deactivation during a goal, failed parameter rebuild | zero command | `UNKNOWN` (1000) |
| `max_duration` reached | decelerate | `TIMEOUT` (1006) |
| The follower is not active | goal rejected, no result | `GOAL_REJECTED` (1), set by the client |
| Cancel | decelerate, then one zero | canceled |
| Laps done; open curve settled at its end | one zero | `NONE` (0), succeeded |

An active follower never rejects a goal at the action level: goals are accepted and validated
in the execute callback, so every failure carries a code and a message. Only an inactive
follower (not yet activated, or deactivated) rejects goals; that has no result, and a client such
as the FollowCurve BT node ({doc}`behavior_tree`) reports it as `GOAL_REJECTED` (1). The client
side codes `GOAL_REJECTED` (1) and `SEND_GOAL_FAILURE` (2) carry Nav2's values and are never in
a result from the follower. A new goal preempts the running one; the
tracker state carries over only when the curve and the goal overrides are identical.

## Map awareness and keepout zones

The follower bypasses the planner, so it sees obstacles only through the **local costmap**. Its
collision prediction and contouring work as in {doc}`obstacle_avoidance`, but nothing checks the
curve against the global map. The `curve_follower` overlay of `vectorfield_bringup` therefore
adds the `static_layer` to the local costmap, so mapped walls stop the robot as well as sensed
obstacles. Without it, only sensed obstacles stop the robot, the same limit as sending a
user-made path straight to `follow_path`. The local costmap must also publish the full costmap
at the follower's rate: `always_send_full_costmap: true`, as in the stock `nav2_params.yaml`,
and `update_frequency` as well as `publish_frequency`, because `costmap_raw` is published from
the update loop.

Areas the robot must never enter can be added with Nav2's keepout filter on the local costmap.
The follower then treats them as lethal: the collision prediction stops the robot before the
zone, and with `obstacle_mode: contour` a zone across the curve is contoured like an obstacle. A
minimal setup, with the mask as a map image whose keepout cells are occupied:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      filters: ["keepout_filter"]
      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: true
        filter_info_topic: "costmap_filter_info"

costmap_filter_info_server:
  ros__parameters:
    type: 0  # keepout
    filter_info_topic: "costmap_filter_info"
    mask_topic: "keepout_filter_mask"
    base: 0.0
    multiplier: 1.0

filter_mask_server:
  ros__parameters:
    yaml_filename: "keepout_mask.yaml"
    topic_name: "keepout_filter_mask"
    frame_id: "map"
```

Run `nav2_map_server`'s `map_server` (named `filter_mask_server`) and its
`costmap_filter_info_server`, both under a lifecycle manager, as in Nav2's keepout-zone tutorial.

## Parameters

The follower takes the common parameter groups (field, motion, phases, obstacles, safety,
visualization) unprefixed; the rotate-to-heading keys are unused. Its own keys cover the loop rate
and frames, input timeouts, curve intake, moving-frame estimation, the progress watchdog and the
interlock. `curve_follower_demo` gives it the merged `FollowPath` block of the controller
configuration first, so both run with the same motion model, speeds, limits and obstacle mode.
See {doc}`parameters/follower`.

## Topics

All names are relative, so the node can run in a namespace.

| Kind | Name | Type |
|---|---|---|
| Action server | `follow_curve` | `vectorfield_msgs/action/FollowCurve` |
| Publisher | `cmd_vel_nav` | `geometry_msgs/Twist`, or `TwistStamped` with `enable_stamped_cmd_vel` (Jazzy and later) |
| Publisher | `~/curve` | `vectorfield_msgs/Curve`, the active curve (transient local) |
| Publishers | `~/markers`, `~/field_grid`, `~/predicted_path`, `~/window`, `~/field_state`, `~/obstacle_distance` | as for the controller ({doc}`visualization`), plus the `curve_info` marker namespace |
| Publisher | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |
| Subscriptions | `odom`, `speed_limit`, `local_costmap/costmap_raw`, `local_costmap/published_footprint` | robot state, speed limits and obstacles |
| Subscriptions | `<action>/_action/status` for each `exclusive_actions` entry | the Nav2 interlock |
| Subscription | the goal's `frame_odom_topic` | `FRAME_MOVING` goals that name one |
