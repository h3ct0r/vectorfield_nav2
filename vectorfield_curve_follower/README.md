# vectorfield_curve_follower

A lifecycle component, `vectorfield_curve_follower::CurveFollower`, with a `follow_curve`
action server (`vectorfield_msgs/action/FollowCurve`). It drives a robot along closed,
parametric and time-varying curves, including curves attached to a moving frame, with the same
vector-field tracker as the Nav2 controller plugin. Nav2's planner is bypassed: the curve itself
is the reference.

**Status:** the node is implemented (M5). `curve_follower_demo`, `send_curve.py` and
`moving_frame_publisher.py` come with `vectorfield_bringup`; the end-to-end tests are the
`test_curve_*` launch tests of `vectorfield_system_tests`.

## Running it

The component is `vectorfield_curve_follower::CurveFollower` (node name `curve_follower`); the
executable `curve_follower` runs it alone. It is a Nav2 lifecycle node: start it with a
`nav2_lifecycle_manager` (`node_names: [curve_follower]`, `autostart: true`), or by hand with
`ros2 lifecycle set /curve_follower configure` and `activate`. On the kinematic simulator, with
Nav2:

```bash
ros2 launch vectorfield_bringup curve_follower_demo.launch.py
ros2 run vectorfield_bringup send_curve.py --shape lemniscate_bernoulli --laps 3
```

From a Nav2 behaviour tree, send goals with the `FollowCurve` node of
[`vectorfield_behavior_tree`](../vectorfield_behavior_tree/README.md). Inside bt_navigator, take
the navigator's own actions (`navigate_to_pose`, `navigate_through_poses`) out of
`exclusive_actions`: they stay active while the tree runs the follower.

## Interfaces

All names are relative, so the node can run in a namespace.

| Kind | Name | Notes |
|---|---|---|
| Action server | `follow_curve` | `vectorfield_msgs/action/FollowCurve`. An active follower never rejects a goal: goals are validated in execute and invalid ones end with an error code before any command (an inactive follower rejects goals, `GOAL_REJECTED` on the client side) |
| Publisher | `cmd_vel_nav` | Feeds Nav2's velocity smoother or collision_monitor; never remap it to the base topic. `Twist`, or `TwistStamped` with `enable_stamped_cmd_vel` (Jazzy and later). Only while a goal executes |
| Publisher | `~/curve` | The active curve (transient local) |
| Publishers | `~/markers`, `~/field_grid`, `~/predicted_path`, `~/window`, `~/field_state`, `~/obstacle_distance` | Visualization, including the `curve_info` marker namespace (direction, start, laps, frame velocity) |
| Publisher | `/diagnostics` | Node health (`diagnostic_updater`) |
| Subscriptions | `odom`, `speed_limit`, `local_costmap/costmap_raw`, `local_costmap/published_footprint` | Robot state, speed limits (volatile, as controller_server subscribes) and obstacles; the local costmap must publish the full costmap (`always_send_full_costmap: true`) |
| Subscriptions | `<action>/_action/status` | Every `exclusive_actions` entry (the Nav2 interlock) |
| Subscription | the goal's `frame_odom_topic` | Only for `FRAME_MOVING` goals that give one |
| TF | `global_frame` ← `robot_base_frame`, ← the curve frame; costmap frame ← `global_frame`; `moving_frame_reference` ← curve frame | Robot pose, curve frame, obstacles, moving frames |

## Parameters

The common groups (Field, Motion, Phases, Obstacles, Safety, Visualization) come from
`vectorfield_ros/params/vectorfield_common_params.yaml`, unprefixed; the rotate-to-heading keys
are unused. The follower's own keys are in
[`params/vectorfield_follower_params.yaml`](params/vectorfield_follower_params.yaml): loop rate
and frames, input timeouts, curve intake limits, moving-frame estimation, the progress watchdog
and the Nav2 interlock. The generated
[parameter reference](https://h3ct0r.github.io/vectorfield_nav2/parameters/follower.html) lists
every key, and the [curve follower guide](https://h3ct0r.github.io/vectorfield_nav2/curve_follower.html)
documents the action, the failure table and a keepout-zone recipe.

## Behaviour

- **Goals:** polylines, muParser expressions r(s, t) and built-in shapes (ellipse, lemniscates,
  superellipse, rounded rectangle, Lissajous, regular polygon). Closed curves are followed for
  `laps` laps or until cancelled; open curves until the robot settles at their end
  (`reverse_direction` swaps the end). `desired_speed` > 0 and `reverse_direction` override the
  parameters for that goal only.
- **Moving frames:** the curve frame's twist comes from an Odometry topic (child frame = curve
  frame) or from TF differencing against `moving_frame_reference` (exact log-SE(2) differences,
  outlier rejection, low-pass filter, extrapolation for at most `input_timeout`) and enters the
  field as feedforward.
- **Mutual exclusion with Nav2:** a goal does not start (`NAV2_ACTIVE`) while any of the
  `exclusive_actions` is active, and stops with one zero command when one becomes active. With
  `require_nav2_interlock` (default) a goal is refused (`UNSUPPORTED`) when
  `follow_path/_action/status` has no publisher, i.e. the interlock cannot see Nav2.
- **Safety:** it publishes only while a goal executes, decelerates on termination, sends one
  zero command and then goes silent. It sees only the local costmap, so mapped hazards must be
  added to it (the bringup overlay adds the static layer).
- **Failures** end the goal with the `FollowCurve` result codes documented in
  `vectorfield_msgs`: stale inputs (`STALE_INPUT`) and an untransformable curve frame
  (`TF_ERROR`) after `stale_abort_timeout`, a persistent block or no progress (`BLOCKED`), a curve
  moving faster than Assumption 3 allows (`ASSUMPTION3_VIOLATED`), `max_duration` (`TIMEOUT`).

## Limitations

- No planner: nothing checks the curve against the map. Only obstacles in the local costmap stop
  the robot; add the static layer (as the bringup overlay does) or a keepout filter for mapped
  hazards.
- Open time-varying curves are not supported (`UNSUPPORTED`).
- The curve must move slower than the robot (Assumption 3), or the goal ends with
  `ASSUMPTION3_VIOLATED`.
- No goal yaw: open curves end when the robot settles at the end point.
- Ackermann bases: no obstacle contouring, as for the controller plugin.
- Planar only.
