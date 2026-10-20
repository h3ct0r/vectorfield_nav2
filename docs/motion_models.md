# Motion models and phases

The field gives a desired velocity $F$ at a point. A tracking law turns it into the body twist
that the base can execute, and a small phase machine handles what the field alone does not: a
robot facing away from the path, the stop at the goal and the final yaw. The theory behind the
laws is in {doc}`theory`; the exact definitions, guards and tolerances are in
{doc}`design/math_spec` ({ref}`sec-laws` and {ref}`sec-phases`).

## Motion models

`motion_model` is read-only and selects the kinematics; `tracking_law: auto` picks the law.

| `motion_model` | Command | `auto` law | Other valid laws | In-place phases | Notes |
|---|---|---|---|---|---|
| `differential` | $(v, 0, \omega)$ | `feedback_linearization` | `heading_tracking` | ROTATE_TO_FIELD, ROTATE_TO_HEADING | |
| `skid_steer` | $(v, 0, \omega)$ | `feedback_linearization` | `heading_tracking` | ROTATE_TO_FIELD, ROTATE_TO_HEADING | Set `min_in_place_angular_vel` (0.25 to 0.3 rad/s) to overcome static friction; the shipped overlay uses 0.25. |
| `omnidirectional` | $(v_x, v_y, \omega)$ | `holonomic` | — | ROTATE_TO_HEADING | `max_lateral_vel` bounds $v_y$. Nav2's `min_y_velocity_threshold` (0.5 by default) zeroes the measured lateral speed; the shipped overlay sets 0.001. |
| `ackermann` | $(v, 0, \omega)$, $v \ge 0$ | `heading_tracking` | — | none | Requires `min_turning_radius` > 0. `obstacle_mode: contour` is rejected in 0.x. See {ref}`ackermann-caveats`. |

Any other model and law pair is rejected when the plugin configures, with a message that names
the valid laws.

`drive_backward` makes $-x$ the drive direction of a differential or skid-steer base; an
Ackermann base always drives forward. `allow_reversing` (default `false`) clamps the speed of
these bases to the drive direction, so feedback linearisation never reverses at full speed when
the field points behind the robot. Plans with reversing segments (cusps) are rejected with `InvalidPath`; use a
forward-only planner.

## Tracking laws

**Feedback linearisation** (differential and skid-steer; thesis eq. 2.1). The field is evaluated
at a control point $p_c$ placed `control_point_offset` $d$ (0.2 m) ahead of `base_link`, and

$$
v = \mathbf h^\top F, \qquad \omega = \frac{(J\mathbf h)^\top F}{d},
$$

with $\mathbf h$ the heading unit vector and $J$ the 90° rotation, so $\dot p_c = F$ exactly. The
base follows the path with an offset that vanishes on straight segments and cuts corners by about
$d$. Larger $d$ gives smoother turns and larger corner cutting; $d$ must be > 0.

**Heading tracking** (differential and skid-steer on request; Ackermann always). The field is
evaluated at `base_link` (Ackermann: the rear-axle reference point, `reference_point_offset` along
body x). The law steers the heading toward $\angle F$ with feedforward of the field's turning rate
and the gain `heading_gain` $k_\psi$ (2.0 1/s):

$$
v = \lVert F \rVert \max(0, \cos e), \qquad \omega = \operatorname{clamp}(\dot\psi_d, \pm\omega_c) + k_\psi e,
$$

with $e$ the heading error. The speed drops to zero when the error reaches 90°.

**Holonomic** (omnidirectional). The base velocity is exactly $F$ in the body frame. The yaw is
independent and follows `omni_heading_policy`: `path_tangent` (face the path tangent, default),
`field` (face the field) or `fixed` (no rotation), with gain `heading_gain`.

| Law and phase | Field evaluated at |
|---|---|
| Feedback linearisation in FOLLOW and ROTATE_TO_FIELD | the control point $p_c$, $d$ ahead of `base_link` |
| Feedback linearisation in FINAL_APPROACH (switches to heading tracking) | `base_link` |
| Heading tracking (differential, skid-steer), holonomic | `base_link` |
| Ackermann, every phase | the reference point, `reference_point_offset` along body x |

The `control_point` marker shows the evaluation point in RViz ({doc}`visualization`).

## Phases

| Phase | Status colour | Entry | Behaviour | Exit |
|---|---|---|---|---|
| ROTATE_TO_FIELD | yellow | differential or skid-steer with `use_rotate_to_field`, and the heading error to the tracked field above `rotate_to_field_angle` ($\pi/2$) | rotate in place, $v = 0$, $\lvert\omega\rvert \ge$ `min_in_place_angular_vel` | error below `rotate_to_field_release_angle` ($\pi/4$) |
| FOLLOW | green | default | speed shaping, field, contour, law, kinematic limits, acceleration limiter, collision cap | — |
| FINAL_APPROACH | cyan | open path ending at the goal, and remaining path length below `final_approach_distance` (auto: $\max(2d,\ \mathrm{xy\_tol} + d)$ for feedback linearisation, xy_tol otherwise) | feedback linearisation hands over to heading tracking at `base_link` on the terminal field; the other laws keep their own | settled |
| ROTATE_TO_HEADING | blue | settled, `use_rotate_to_heading`, the goal checker checks yaw (yaw tolerance > 10⁻⁶ rad); not Ackermann, not the curve follower | rotate in place toward the goal yaw at up to `rotate_to_heading_angular_vel` | the goal checker succeeds; if the robot leaves the XY tolerance, back to FINAL_APPROACH or FOLLOW |
| STOPPED | red | cancel, the contour watchdog, or a goal heading the robot cannot reach | decelerate every axis to zero | latched until the next goal |

The heading error that starts and ends ROTATE_TO_FIELD is measured to the field the active law
tracks, at that law's own point, contour included. With these thresholds a robot enters the
phase at most once per sharp corner (60° to 170°) and once for a goal behind it; a circulation
around an obstacle never re-enters it.

**Settled** means: XY reached, and the robot is within $\sigma_{\mathrm{stop}} = \mathrm{xy\_tol}/2$
of the goal (by distance or by remaining path length), or the braking cap binds. XY reached never
disagrees with Nav2:

- on Lyrical and later, the plugin calls the goal checker's `isGoalXYReached` with the server's
  own arguments;
- on Humble and Jazzy, it latches $\lVert p - g\rVert \le \mathrm{xy\_tol}$ with the tolerance
  from `getTolerances()`, re-armed on every `setPlan()`;
- the curve follower uses its own latch with `goal_tolerance`.

Once settled with no rotation to do (`use_rotate_to_heading: false`, a goal checker that ignores
yaw, or Ackermann), the robot holds zero until it leaves the XY tolerance. `BLOCKED` is a
per-cycle status, not a phase: the phase's target becomes zero and motion resumes by itself when
the way clears ({doc}`obstacle_avoidance`).

The rotate law is RPP-style: $\lvert\omega\rvert = \min(\omega_{\mathrm{rot}}, \sqrt{2\alpha_{\max}\lvert e\rvert})$
with $\omega_{\mathrm{rot}}$ = min(`rotate_to_heading_angular_vel`, `max_angular_vel`) and
$\alpha_{\max}$ = `max_angular_accel`, so the rotation decelerates into the target. In
ROTATE_TO_HEADING the floor `min_in_place_angular_vel` applies only while the error exceeds half
the yaw tolerance.

## Rotation shim

The plugin can run as the primary controller of Nav2's `RotationShimController`. The shim then
performs the initial rotation itself, so set `use_rotate_to_field: false`. The key layout differs
by distro.

Humble and Jazzy: the primary is named by a string, and its keys sit next to the shim's under
`FollowPath`:

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "vectorfield_nav2_controller::VectorFieldController"
      angular_dist_threshold: 0.785
      # vector-field keys, unprefixed under FollowPath
      motion_model: "differential"
      use_rotate_to_field: false
```

Lyrical and later: the primary has its own namespace, and the vector-field keys move under
`FollowPath.primary_controller`:

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      angular_dist_threshold: 0.785
      primary_controller:
        plugin: "vectorfield_nav2_controller::VectorFieldController"
        motion_model: "differential"
        use_rotate_to_field: false
```

On Lyrical the marker topics follow the plugin name: `FollowPath/primary_controller/markers`.

On Humble and Jazzy the shim declares `rotate_to_heading_angular_vel` (1.8) and
`max_angular_accel` under the same prefix, so both controllers share those values; the plugin
warns when `rotate_to_heading_angular_vel` exceeds `max_angular_vel` and uses the smaller one.

:::{warning}
The shim re-aims the robot at the path whenever it re-checks the heading, which undoes an
obstacle-contour episode. The shim with `obstacle_mode: contour` is supported only on Jazzy with
`FollowPath.rotate_to_heading_once: true`; in every other combination the plugin logs a warning
once at configuration. No shim overlay is shipped.
:::

(ackermann-caveats)=

## Ackermann caveats

A car-like base cannot turn in place, which affects the controller and the rest of Nav2:

- **No in-place phases.** ROTATE_TO_FIELD and ROTATE_TO_HEADING never run. A heading error above
  90° at the reference point, a car that has passed the goal without reaching the XY tolerance,
  or a goal yaw outside the tolerance once settled raises `STATUS_HEADING_INFEASIBLE`: the controller stops and raises
  `NoValidControl`, and Nav2 replans or aborts.
- **Minimum speed.** While following, a positive speed below `ackermann_min_speed`
  $v_{\mathrm{ack}}$ (0.05 m/s) is raised to $v_{\mathrm{ack}}$, because at $v = 0$ the car cannot
  turn at all. Only the braking cap, `BLOCKED` and STOPPED command zero. The band is lifted near
  the goal.
- **Curvature.** $\lvert\omega\rvert \le \lvert v\rvert / R_{\min}$ after every limit;
  `CURV-SAT` in the status text shows when it binds.
- **Planner and goal checker.** The path must be drivable: the Ackermann overlay uses Smac
  Hybrid-A* with `DUBIN` motions and the car's turning radius (0.5 m), and
  `nav2_controller::PositionGoalChecker`, which ignores yaw.
- **Behaviour trees.** The stock recovery trees, and every stock NavigateThroughPoses tree, use
  the Spin behaviour, which a car cannot execute. The overlay runs NavigateToPose with the stock
  `navigate_w_replanning_time.xml`, which has no Spin. A custom tree for a car must also leave out
  Spin, and BackUp only where reversing is safe.
- **Obstacles.** `obstacle_mode: contour` is rejected: circulating at $\lambda$ needs a curvature
  far above $1/R_{\min}$. Obstacles stop the car through the collision cap, and Nav2 replans.
