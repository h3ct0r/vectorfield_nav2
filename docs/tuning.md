# Tuning guide

The defaults target a TurtleBot-sized base (TB3 footprint, 0.5 m/s, 20 Hz, the stock 3 m × 3 m
local costmap) and pass the configuration validator without warnings. This page explains which
parameters to change for another robot, what the validator's findings mean, and how to try
changes without ROS using `vf-tune`. Every parameter is listed in {doc}`parameters`.

## Order of work

1. **Motion model and limits.** Set `motion_model` and the robot's real limits:
   `max_linear_vel`, `max_angular_vel`, `max_linear_accel`, `max_linear_decel` and
   `max_angular_accel` (`max_lateral_vel` for an omnidirectional base, `min_turning_radius` for
   Ackermann). The limits scale the whole field when one binds, so the commanded direction is
   kept. Set the velocity smoother's limits at least as high as the controller's on every axis,
   otherwise the smoother clips single axes and bends the path; with `scale_velocities: true` it
   scales all axes together.
2. **Speed.** `desired_speed` $v_r$ is the field's speed and must not exceed `max_linear_vel`.
   Speed limits from Nav2, the braking cap and the clearance slowdown lower it further.
3. **Convergence.** `convergence_gain` $k_f$ sets how sharply the field turns toward the path
   ({ref}`sec-gains`). The initial slope of the gain function is $G'(0) = 2k_f/\pi$ for `atan`
   (default) and $k_f$ for `rational`. Larger values converge in a shorter distance and turn
   harder near the path; too large values overshoot in discrete time or need more yaw rate than
   the base has.
4. **Control point** (feedback linearisation only). `control_point_offset` $d$ is the distance of
   the tracked point ahead of `base_link`. The base cuts corners by about $d$; a smaller $d$
   follows corners more tightly but turns more abruptly. The thesis experiments used 0.15 to
   0.3 m.
5. **Goal approach.** `terminal_braking_distance` $\sigma_b$ (0.5 m) is the path length over which
   the terminal field brakes to `min_approach_speed` (0.05 m/s); it must be at least
   $v_r^2/a_{\mathrm{dec}}$. `final_approach_distance` (auto) is where feedback linearisation
   hands over to heading tracking at `base_link`.
6. **Obstacles.** Keep `collision_check_horizon` long enough to see the stopping distance and
   short enough to stay in the local costmap. Leave the contour distances on auto (−1) unless the
   robot needs more clearance; they follow from the footprint and $d$ ({doc}`obstacle_avoidance`).
7. **Rotations.** `rotate_to_heading_angular_vel` (1.5 rad/s) and `max_angular_accel` shape the
   in-place rotations; skid-steer bases need `min_in_place_angular_vel` of 0.25 to 0.3 rad/s.

## Validator findings

The controller validates its configuration at configure time and on every parameter change.
Errors make `configure()` fail, and a `ros2 param set` that would cause one is refused with the
reason, keeping the previous configuration. Warnings are logged with their code. If a rebuild
still fails at run time (for example after a footprint change), the controller commands nothing
and raises `NoValidControl` until the configuration is valid again. The curve follower validates the same way, and once
more for each goal's overrides. The findings, with their codes:

| Code | Kind | Condition | Fix |
|---|---|---|---|
| `desired_speed_above_max_linear_vel` | error | $v_r$ > `max_linear_vel` | lower `desired_speed` or raise `max_linear_vel` |
| `min_in_place_above_rotate_vel` | error | `min_in_place_angular_vel` > `rotate_to_heading_angular_vel` | lower the floor |
| `contour_distance_order` | error | not $\lambda < D_{\mathrm{in}} \le D'_{\mathrm{in}}$ | fix the order, or −1 (auto) for all three |
| `contour_distance_below_inscribed` | error | $\lambda$ too small for the footprint | raise `obstacle_contour_distance`, or auto |
| `invalid_tracking_law` | error | model and law do not match | `tracking_law: auto` |
| `ackermann_contour_unsupported` | error | `obstacle_mode: contour` with `ackermann` | `obstacle_mode: none` |
| `ackermann_min_turning_radius` | error | `min_turning_radius` ≤ 0 with `ackermann` | set the car's turning radius |
| `braking_distance_exceeds_window` | error | $\sigma_b + d + 0.1$ m exceeds half the local costmap | lower $\sigma_b$ or enlarge the costmap |
| `fl_control_point_offset` | error | $d$ ≤ 0 with feedback linearisation | $d$ > 0 |
| `rotate_vel_above_max_angular_vel` | warning | `rotate_to_heading_angular_vel` > `max_angular_vel` | the smaller one is used |
| `discrete_overshoot` | warning | $G'(0)\, v_r\, \Delta t \ge 1$ | lower $k_f$ or raise the control rate |
| `fl_gain_offset` | warning | $G'(0)\, d > 0.5$: the field turns faster than the heading can follow | lower $k_f$ or $d$ |
| `terminal_braking_too_short` | warning | $\sigma_b < v_r^2/a_{\mathrm{dec}}$ | raise $\sigma_b$ or `max_linear_decel` |
| `contour_distance_below_circumscribed` | warning | $\lambda - d < r_{\mathrm{circ}}$: a footprint corner can touch the obstacle | raise $\lambda$ or lower $d$ |
| `blend_zone_exceeds_window` | warning | $D'_{\mathrm{in}} + d$ ≥ half the costmap − 0.2 m | lower $D'_{\mathrm{in}}$ or enlarge the costmap |
| `horizon_too_short` | warning | horizon × `max_linear_vel` below the stopping distance plus two cycles, 0.1 s of latency and 0.1 m | raise `collision_check_horizon` |
| `horizon_exceeds_window` | warning | horizon × `max_linear_vel` > half the costmap − $r_{\mathrm{circ}}$ | lower the horizon or enlarge the costmap |
| `ackermann_curvature` | warning | $R_{\min}(c_G k_f + \kappa_{\mathrm{path}}) > 1$ | lower $k_f$, or smooth the path |
| `yaw_rate_for_curvature` | warning | $v_r(c_G k_f + \kappa_{\mathrm{path}})$ > `max_angular_vel` | lower $v_r$ or $k_f$ |
| `fl_path_curvature` | warning, per plan | $\kappa_{\mathrm{path}}\, d > 1$: the control point cuts the path's corners | lower $d$ or smooth the path |

$c_G$ is a constant of the gain function (0.186 for `atan`, 0.385 for `rational`) and
$\kappa_{\mathrm{path}}$ the largest curvature of the plan or curve. The per-plan advice runs once
per goal on the received plan. The full rules are in {ref}`sec-math-validation`.

## Symptoms and fixes

| Symptom | Likely cause | Change |
|---|---|---|
| Oscillates across the path | $k_f$ too high for $v_r$ and the control rate (`discrete_overshoot`), or $G'(0)d > 0.5$ | lower `convergence_gain`; raise `control_point_offset` slightly |
| Converges slowly, runs parallel to the path | $k_f$ too low | raise `convergence_gain` |
| Cuts corners | feedback-linearisation offset | lower `control_point_offset`, or use `tracking_law: heading_tracking` |
| Turns in place at sharp corners | the field at the control point points more than 90° away from the heading (ROTATE_TO_FIELD) | expected, at most once per corner; smooth the plan or use a planner with smoother paths. Repeated entries at one corner are a defect worth reporting |
| Stops short of the goal, then creeps | `min_approach_speed` too low, or the cap binds near a wall | raise `min_approach_speed`; check `stop_margin` and the footprint |
| Overshoots the goal | `terminal_braking_distance` below $v_r^2/a_{\mathrm{dec}}$ | raise `terminal_braking_distance` or `max_linear_decel` |
| Skid-steer stalls in rotations | static friction | `min_in_place_angular_vel` 0.25 to 0.3 rad/s |
| Skid-steer runs a steady offset outside curves | wheel slip: only part of the commanded yaw rate is realised, and the centre of rotation is off `base_link` | identify `skid_yaw_rate_efficiency` (measured over commanded yaw rate) and `skid_icr_offset` on the robot ({ref}`skid-slip`) |
| Slows down near obstacles that are not on the path | the braking cap sees them in the rollout, or the clearance slowdown | shorten `collision_check_horizon` within the warning bounds; `clearance_slowdown_distance: 0` |
| Contour engages in doorways | `obstacle_activation: always` | use `path_blocked` (default) |
| Stops in front of a blocking obstacle instead of contouring | `obstacle_mode: none`; neither side passes within the side probe; the goal is inside the obstacle; or Ackermann | expected in a dead end; otherwise enable `contour` ({doc}`obstacle_avoidance`) |
| `CONTOUR` episodes end in STOPPED | watchdog limits too low for the obstacle size | raise `obstacle_max_contour_length` or `obstacle_max_contour_time` |
| `A3-CLAMP` with a moving curve | the curve moves almost as fast as $v_r$ | raise `desired_speed` (per goal or parameter) or slow the frame |
| `CURV-SAT` on an Ackermann base | the field needs more curvature than $1/R_{\min}$ | lower `convergence_gain` (`ackermann_curvature`) |

## vf-tune

`vf-tune` runs the same validator without ROS, prints a suggested change for each finding, and
sweeps parameters in closed-loop simulation. It is part of the Python tools (`pip install
'./vectorfield_py[tools]'`, see {doc}`quickstart`). It reads a Nav2 parameter file (the
`controller_server.ros__parameters.FollowPath` block), a node parameter file or flat keys, and
takes the costmap size from the file's `local_costmap` when present:

```bash
# Validate a Nav2 parameter file, with the per-plan advice for a path
vf-tune nav2_params.yaml --curve path.csv --costmap 3,3,0.05

# Try overrides on top of the defaults
vf-tune --set desired_speed=0.6 --set max_linear_vel=0.7

# Sweep the convergence gain on an ellipse: max |D| after the transient and the settle time
vf-tune --sweep convergence_gain=1:4:0.5 --curve ellipse:a=2,b=1
```

The exit status is 0 for a valid configuration (warnings allowed), 1 when an optional dependency
is missing and 2 for an invalid configuration or input. `vf-sim` runs one configuration with a
plot or GIF, and `vf-plot --decompose X,Y` prints the field components at a point, which helps
to see why the robot turns where it does.
