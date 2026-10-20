# Migrating from vectorfield_stack

[`adrianomcr/vectorfield_stack`](https://github.com/adrianomcr/vectorfield_stack) is the ROS 1
Python implementation that accompanies Rezende's thesis. vectorfield_nav2 re-implements the
method in C++ from the equations; no upstream code is used. This page maps upstream concepts,
parameters and topics to vectorfield_nav2, lists the behaviour you will notice changing, and
summarises the upstream defects that the new design avoids. The complete audit, with the
evidence for every defect at upstream commit `2ab3c95`, is {doc}`design/upstream_audit`; the
identifiers U01 to U40 below refer to it.

## Concepts

| vectorfield_stack | vectorfield_nav2 |
|---|---|
| `distancefield` class: static field $\Phi = v_r(-G\nabla D + HT)$ | `vectorfield_core`: `DistanceField` with the same `atan` gain (plus `rational`), the time-varying feedforward $\Phi_T$ and the terminal field for open paths |
| Point paths on the `Path` topic, nearest sample in a ±10-sample window | Nav2 plans through the controller plugin; polylines through the `follow_curve` action. Exact segment projection with metric search windows |
| Equation curves on the `PathEq` topic, a Python string `"[x(u), y(u), z(u)]"` | `follow_curve` goals with an `EXPRESSION` curve: muParser strings in `s` and `t`, or a built-in shape |
| `differential_node`, `skidsteer_node` (feedback linearisation, wheel speeds) | `vectorfield_nav2_controller::VectorFieldController` in Nav2's `controller_server`, or the curve follower; `motion_model` `differential` or `skid_steer`, body twist output |
| `simple_node` (holonomic integrator, world-frame 3D velocity) | `motion_model: omnidirectional` with the holonomic law: planar body-frame $(v_x, v_y, \omega)$ |
| Obstacle contour from one externally published closest point | Contouring on the local costmap's distance transform, gated by a blocked path (`obstacle_mode: contour`) |
| `robotsim` simulators with circular obstacles | `vectorfield_sim`: exact-arc kinematics for four models, laser scan, circle and box obstacles settable at run time |
| `examples` launch files and `example_path.py` | `vectorfield_bringup` demos, `send_curve.py` and `moving_frame_publisher.py` |
| Quadrotor controller, drone simulator, 3D curves | Not ported: vectorfield_nav2 is planar, for Nav2 ground robots |

New in vectorfield_nav2, with no upstream counterpart: moving curves (the time-varying field),
the terminal field, collision prediction with the braking cap and the blocked status, velocity,
acceleration and curvature limits, the Ackermann model, the rotate phases, Nav2 integration on
three distros, the `FollowCurve` action with laps and result codes, configuration validation,
the RViz visualization, and the Python bindings and tools.

## Parameters

Upstream parameters are ROS 1 private parameters (`~group/name`). In vectorfield_nav2 they are
flat names, prefixed with the plugin name in `controller_server` (`FollowPath.<name>`) and
unprefixed on the `curve_follower` node.

| vectorfield_stack | vectorfield_nav2 | Default | Notes |
|---|---|---|---|
| `vector_field/vr` (YAML 0.5) | `desired_speed`; per-goal `desired_speed` on the follower | 0.4 m/s | Must not exceed `max_linear_vel` (0.5). |
| `vector_field/kf` (5.0) | `convergence_gain` | 2.0 1/m | Same `atan` gain. Upstream's 5.0 with $d$ = 0.2 m triggers the `fl_gain_offset` warning. |
| `vector_field/reverse_direction` | `FollowCurve.reverse_direction` | false | The plugin follows the plan's direction. |
| `groundrobot/d_feedback` (0.1) | `control_point_offset` | 0.2 m | |
| `groundrobot/move_backwards` | `drive_backward` | false | `allow_reversing` (false) separately forbids speed against the drive direction. |
| `groundrobot/a`, `groundrobot/b` | none | — | No wheel speeds. Use `motion_model: skid_steer` and `min_in_place_angular_vel` 0.25 to 0.3. |
| `obstacle_avoidance/flag_follow_obstacle` | `obstacle_mode` (`none`, `contour`, `cbf`) and `obstacle_activation` (`path_blocked`, `always`) | `none`; `path_blocked` | `always` reproduces the upstream activation rule. The demos use `contour`. |
| `obstacle_avoidance/epsilon` | `obstacle_contour_distance` ($\lambda$) | auto from the footprint (0.47 m for a TB3) | Measured to the costmap cells. |
| `obstacle_avoidance/switch_dist` | `obstacle_full_switch_distance` ($D_{\mathrm{in}}$) | auto: $\lambda$ + 0.15 | $\lambda < D_{\mathrm{in}} \le D'_{\mathrm{in}}$ is enforced. |
| `obstacle_avoidance/switch_dist_0` | `obstacle_blend_start_distance` ($D'_{\mathrm{in}}$) | auto: $\lambda$ + 0.35 | |
| `obstacle_avoidance/obstacle_point_topic_name` and the body-frame variant | none | — | Obstacles come from the local costmap. |
| `topics/pose_topic_name`, `pose_topic_type` | none | — | Pose from TF: the controller server's pose; the follower's `global_frame` ← `robot_base_frame`, plus `odom_topic`. |
| `topics/cmd_vel_topic_name` | plugin: the controller server's output; follower: `cmd_vel_nav` | — | Both go through the velocity smoother and collision_monitor. |
| `topics/path_topic_name`, `path_equation_topic_name` | Nav2 plans; `follow_curve` goals | — | |
| 50 Hz loop (hard-coded) | `controller_server`'s `controller_frequency`; the follower's `controller_frequency` | 20 Hz | $\Delta t$ is measured each cycle. |
| N = 200 equation samples, ±10-sample window | `default_sample_spacing`, `global_search_samples`, `search_window` | 0.05 m, 400, 1.0 m | Metric, independent of the sample density. |
| Example publisher `N_curve`, `a`, `b`, `phi`, `cx`, `cy` | `send_curve.py` shapes; built-in parameters `cx`, `cy`, `yaw` | — | Upstream `phi` is in degrees; `yaw` is in radians. |

## Messages and topics

| vectorfield_stack | vectorfield_nav2 |
|---|---|
| `distancefield/Path` (`geometry_msgs/Polygon`, float32) | `vectorfield_msgs/Curve` with `type = POLYLINE` and float64 `points`; `header.frame_id` is the curve frame, resolved through TF |
| `Path.closed_path_flag` | `Curve.closed` (verified for expressions) |
| `Path.insert_n_points` | `Curve.sample_spacing` (metres) |
| `Path.filter_path_n_average` | `Curve.max_corner_deviation`: a bounded corner fillet that keeps the endpoints, not a moving average |
| `distancefield/PathEq` (`equation`, `u_i`, `u_f`) | `Curve` with `type = EXPRESSION`: `x_expression`, `y_expression`, `s_min`, `s_max`, `parameter_names`, `parameter_values` |
| `/example_path`, `/example_path_equation` topics | the `follow_curve` action (feedback, result codes, cancel, `laps`) |
| `<robot>/cmd_vel`, `skidsteer/wheels_speeds` | body twist through Nav2's command chain; no wheel speeds |
| `/visualization_path` and the simulators' markers | `~/curve`, `<ns>/markers` (the namespaces of {doc}`visualization`), `<ns>/window`, `<ns>/predicted_path`, `<ns>/field_state` |

## Behaviour differences

1. **Open paths end with a controlled stop.** Upstream cut the speed from $v_r$ to 0 when the
   nearest sample was the last one, wherever the robot was (U10). Here the terminal field brakes
   over `terminal_braking_distance` and stops on the goal; Nav2's goal checker or the follower's
   settled rule decides completion, and ROTATE_TO_HEADING turns to the goal yaw when one is
   checked.
2. **Curves are action goals.** `follow_curve` reports progress and ends with a result code.
   Invalid curves are rejected before the robot moves (`INVALID_CURVE`). A closed curve with
   `laps` = 0 runs until cancelled, as upstream did; `laps` > 0 stops after that many laps.
3. **No Python in curve definitions.** Expressions are muParser strings, one per axis (U01).
   Python-only syntax is a parse error.
4. **Commands are bounded.** Velocity, acceleration and, for Ackermann, curvature limits apply;
   upstream published the raw field. Starts and recoveries are slower but continuous.
5. **No unintended reversing.** Differential and skid-steer robots rotate in place when the field
   points more than 90° away and resume within 45°; upstream's feedback linearisation would
   reverse. Backing up is opt-in: `allow_reversing: true` follows planner plans with cusps,
   segment by segment ({ref}`reversing`).
6. **Segment-exact tracking.** Sparse plans no longer scallop, and no densification or filtering
   is needed (U13).
7. **Re-localisation is handled.** After an AMCL jump the follower re-acquires the curve through
   its global search, and the plugin searches the fresh plan window every cycle (U14).
8. **Obstacles come from the costmap, and contouring is gated.** Every lethal cell counts, not one
   external point; by default the contour engages only when the path ahead is blocked, with a
   latched side and a watchdog (U17, U24).
9. **The robot stops before collisions.** The braking cap stops it `stop_margin` short of a
   predicted collision; a blocked plugin raises `NoValidControl` so Nav2 replans.
10. **Missing data stops the robot.** Stale pose, odometry, costmap or frame input decelerates the
    robot; upstream kept the last command (U33).
11. **Planar only.** z is ignored; 3D example curves become their planar projections (U26).
12. **Moving curves work.** `FRAME_MOVING` goals add the paper's feedforward $\Phi_T$; upstream had
    only static fields.
13. **Different defaults.** `desired_speed` 0.4 (upstream YAML 0.5), `convergence_gain` 2.0 (5.0)
    and `control_point_offset` 0.2 (0.1). Re-tune with {doc}`tuning` rather than copying the
    upstream values.

## Upstream defects avoided

The audit lists 39 confirmed defects (4 crash or security, 16 wrong behaviour, 14 robustness,
5 cosmetic) and one refuted suspicion (U15). The ones that change behaviour:

| ID | Upstream defect | vectorfield_nav2 |
|---|---|---|
| U01 | Curve strings from a topic passed to Python `eval` | vendored muParser with only declared variables, constants and deterministic functions |
| U02, U03 | Expression rewriting corrupts identifiers; one global curve shared by every instance | identifier-bound parameters; curves are values owned by the caller |
| U04, U05 | Initial closest point searched over 3 samples; samples spaced by $u_f/N$ | thesis §5.1 global stage over `global_search_samples`; sampling spans $[s_{\min}, s_{\max}]$ |
| U06 | Open equation curves never terminate and wrap around | only closed curves wrap; open curves end on the terminal field |
| U07 | Tangent divides by zero at stationary points | irregular curves rejected with `INVALID_CURVE` |
| U08, U09 | Short paths index out of range; open paths wrap the search window | segment projection with end rays; no index window; open curves never wrap |
| U10 | Instant stop half a sample before the goal, at any lateral distance | terminal field and settled rule |
| U11, U12 | Densification and filtering move the goal and drop points | endpoints kept exactly; corners filleted within a bound |
| U13 | Distance to the nearest vertex, not segment | exact point-to-segment projection |
| U14 | Path lost after fast motion, a localization jump or a detour | metric windows, global search, per-cycle plan-window search |
| U16 to U19 | Latent divisions by zero, a discontinuous obstacle switch, the wrong evaluation point, a field evaluation that mutates tracking state | latched circulation side, explicit evaluation points, a `const` step with explicit state |
| U21 to U24 | Pose callbacks that never work with the default type, a missing import, a mis-wired topic, obstacle points converted without TF or expiry | pose from TF and Nav2; parameters declared once; obstacles from the costmap |
| U25 | No validation of $d$, no limits, no goal-reached signal | validator, limiter and settled status |
| U32, U33 | Unlocked concurrent callbacks; last command persists when not ready | immutable snapshots; every failure commands zero and is reported |
| U34 | float32 path points, no frames | float64 points, `header.frame_id` resolved through TF |
| U40 | Skid-steer wheel-speed map disagrees with the thesis | no wheel speeds: a body twist, with slip modelled in `vectorfield_sim` |

The quadrotor defects (U27 to U31) have no counterpart, because the quadrotor controller is not
ported.
