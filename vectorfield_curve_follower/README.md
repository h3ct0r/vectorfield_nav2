# vectorfield_curve_follower

A lifecycle component, `vectorfield_curve_follower::CurveFollower`, with a `follow_curve`
action server (`vectorfield_msgs/action/FollowCurve`). It drives a robot along closed,
parametric and time-varying curves, including curves attached to a moving frame, with the same
vector-field tracker as the Nav2 controller plugin. Nav2's planner is bypassed: the curve itself
is the reference.

**Status:** skeleton — implemented in M5, together with `curve_follower_demo`, `send_curve.py`
and `moving_frame_publisher.py` in `vectorfield_bringup`.

## Planned interfaces

All names are relative, so the node can run in a namespace.

| Kind | Name | Notes |
|---|---|---|
| Action server | `follow_curve` | Goals are validated in execute; invalid goals end with an error code before any command |
| Publisher | `cmd_vel_nav` | Feeds Nav2's velocity smoother and collision_monitor; never remap it to the base topic |
| Publisher | `~/curve` | The active curve (transient local) |
| Publishers | `~/markers`, `~/field_grid`, `~/predicted_path`, `~/window`, `~/field_state`, `~/obstacle_distance` | Visualization, including the `curve_info` marker namespace (direction, start, laps) |
| Publisher | `/diagnostics` | Node health |
| Subscriptions | `odom`, `speed_limit`, `local_costmap/costmap_raw`, `local_costmap/published_footprint` | Robot state, speed limits and obstacles |
| Subscriptions | `<action>/_action/status` | Nav2 actions that must not run at the same time (see below) |

## Planned behaviour

- **Lifecycle:** managed by `lifecycle_manager_curve_follower` (autostart), or manually with
  `ros2 lifecycle set`.
- **Curves:** polylines, muParser expressions r(s, t) and built-in shapes (ellipse,
  lemniscates, superellipse, rounded rectangle, Lissajous, regular polygon). Closed curves can
  be followed for a number of laps or until cancelled.
- **Moving frames:** the curve frame's velocity comes from an Odometry topic or from TF
  differencing and enters the field as feedforward.
- **Mutual exclusion with Nav2:** the follower refuses to start, or stops with one zero command,
  while any of the `exclusive_actions` (by default `follow_path`, `navigate_to_pose`,
  `navigate_through_poses`, `spin`, `backup`, `drive_on_heading`, `assisted_teleop`,
  `dock_robot`, `undock_robot`, `follow_object`) is active.
- **Safety:** it publishes only while a goal executes, decelerates on termination, sends one
  zero command and then goes silent. It sees only the local costmap, so mapped hazards must be
  added to it (the bringup overlay adds the static layer).
- **Failures** end the goal with the `FollowCurve` result codes documented in
  `vectorfield_msgs`.
