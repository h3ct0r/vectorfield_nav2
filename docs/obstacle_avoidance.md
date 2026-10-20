# Obstacle avoidance

Two mechanisms act on obstacles. **Collision prediction** is always on (unless
`use_collision_detection` is false): it slows the robot down and stops it short of a predicted
collision. **Obstacle contouring** (`obstacle_mode: contour`, default `none`) is the obstacle
extension of the thesis (§3.4): when the path ahead is blocked, the field circulates around the
obstacle at a set distance and returns to the path. The underlying field is summarised in
{doc}`theory`; the exact rules, with their rationale, are in {doc}`design/math_spec`
({ref}`sec-contour-mods` and {ref}`sec-collision`). The `contour` and `prediction` markers are
described in {doc}`visualization`.

## Obstacle input

Obstacles are the **lethal** cells (cost 254) of the local costmap, plus unknown cells when
`unknown_is_obstacle` is true. Inflated and inscribed costs are ignored: the robot's own
footprint polygon is checked against the lethal cells. The plugin copies the
`controller_server`'s local costmap and footprint once per cycle; the curve follower subscribes
to `local_costmap/costmap_raw` and `local_costmap/published_footprint`. In each cycle the grid,
its distance transform and the obstacle component are built once and stay frozen for every
rollout and field probe of that cycle.

## Collision prediction and the braking cap

Each cycle the controller rolls itself out on the costmap for `collision_check_horizon` (1.5 s),
sub-sampled so that checked poses differ by at most one cell. The free arc length $s_{\mathrm{free}}$
is the base path length up to the last collision-free pose ($+\infty$ if none collides). The speed
is capped so that the robot can stop `stop_margin` $m_s$ (0.1 m) short of the collision:

$$
v_{\mathrm{cap}} = \sqrt{2\, a_{\mathrm{dec}} \max(0,\ s_{\mathrm{free}} - m_s)},
$$

with $a_{\mathrm{dec}}$ = `max_linear_decel`. The cap scales the field, so the robot slows down
along the same path rather than turning away. The `prediction` markers show the rollout (green up
to the first collision), the colliding pose and `s_free`; `STATUS_COLLISION_AHEAD` is set while
the cap binds. `clearance_slowdown_distance` (0, disabled) additionally slows the robot when the
clearance to the nearest obstacle is small.

## Blocked

`STATUS_BLOCKED` (`BLOCKED` in the status text, red footprint) is set when the current pose
collides; in FOLLOW or FINAL_APPROACH when the free arc is too short to keep moving
($s_{\mathrm{free}} < m_s + v_{\mathrm{stop}}^2/(2a_{\mathrm{dec}})$); or in a rotation phase when the
remaining in-place rotation would collide. It clears only $\max(2\,\mathrm{res}, 0.05)$ m above
that threshold, so sensor flicker does not toggle it. Blocked is a per-cycle status, not a phase:
the target becomes zero and the robot resumes by itself when the way clears.

- **Controller plugin:** each blocked cycle raises `NoValidControl` after commanding zero. The
  `controller_server` keeps publishing zero for `failure_tolerance` (0.3 s in the stock
  configuration), then fails `FollowPath`, and the behaviour tree replans or runs its recoveries.
  A robot that stops at the goal with the cap binding counts as settled, never as blocked.
- **Curve follower:** it waits; `blocked_timeout` (10 s) without resuming, or no progress of
  `progress_radius` within `progress_timeout`, ends the goal with `BLOCKED`.

## Obstacle contouring

With `obstacle_mode: contour`, an obstacle that blocks the path starts a **contour episode**: the
field blends from the path field into a field that circulates around the obstacle at the contour
distance $\lambda$, on a side chosen once at the start, and blends back to the path field once the
obstacle is passed.

:::{figure} _static/figures/contour_disc.svg
:alt: A robot on a straight path meets a disc, circulates below it at distance lambda, then returns to the path and stops at the goal.
:width: 100%

Contouring a disc that blocks a straight path (differential drive, `obstacle_mode: contour`,
defaults). Dashed: the contour distance $\lambda$; dotted: $D_{\mathrm{in}}$ and
$D'_{\mathrm{in}}$, between which the commanded field blends from the path field to the contour
field. Orange arrows show the contour field from the pose where the episode starts (black
outline); the circulation side is latched for the episode. Reproduce it with
`vf-sim --curve line.csv --pose -3,0,0 --circle 0,0.05,0.3 -o contour.svg`, where `line.csv` holds the two
vertices `-3,0` and `3,0`.
:::

### Distances

| Symbol | Parameter | Auto value (negative parameter) | Meaning |
|---|---|---|---|
| $\lambda$ | `obstacle_contour_distance` | $\max\big(\sqrt{(w_{\mathrm{lat}} + 0.10)^2 + d^2},\ r_{\mathrm{circ}} + d + 0.05\big)$ | distance from the obstacle cells at which the evaluation point circulates |
| $D_{\mathrm{in}}$ | `obstacle_full_switch_distance` | $\lambda + 0.15$ | below it only the contour field acts |
| $D'_{\mathrm{in}}$ | `obstacle_blend_start_distance` | $\lambda + 0.35$ | where the contour field starts to blend in |

$w_{\mathrm{lat}}$ is the footprint's half-width, $r_{\mathrm{circ}}$ its circumscribed radius and
$d$ the offset of the evaluation point (the control point for feedback linearisation, 0
otherwise). The auto $\lambda$ leaves about 10 cm of lateral clearance for the base; for the TB3
footprint and $d$ = 0.2 m it is 0.47 m. Configuration requires $\lambda < D_{\mathrm{in}} \le D'_{\mathrm{in}}$.
Between $D_{\mathrm{in}}$ and $D'_{\mathrm{in}}$ the commanded direction blends linearly; the speed
is that of the path field, so terminal braking still applies near obstacles.

### The path-blocked gate

`obstacle_activation: path_blocked` (default) lets an episode start only while the path ahead is
blocked; `always` reproduces the thesis, where the contour may engage whenever an obstacle is
close and the field points at it. Without the gate, a valid path that passes a wall, a door jamb
or furniture at a clearance between $\lambda$ and $D_{\mathrm{in}}$ would pull the robot in to
$\lambda$ and make it limit-cycle exactly where clearance matters.

- The gate samples the path one cell apart from the closest point up to `obstacle_lookahead`
  ahead (auto: $\max(1.0,\ v_r T_h)$ = 1.0 m with the defaults), never beyond the goal.
- A sample is blocked when its distance to the nearest obstacle cell is below
  $w_{\mathrm{lat}}$ − `obstacle_gate_tolerance` (0.05 m), that is, when the footprint centred on the
  path would touch the obstacle. Grazing contacts are left to the collision prediction.
- The gate is open in a cycle with a blocked sample and closes after the path has stayed clear
  for `obstacle_release_time` (0.5 s).
- **Goal veto:** when the path ends at the goal and the goal itself is blocked, no episode starts
  and an active one ends; the braking cap stops the robot instead.

### Which obstacle: the blocking set

The episode is anchored to the obstacle that actually blocks the path, not to the nearest lethal
cell, which is often an unrelated wall. The obstacle component $K$ is the 8-connected set of
obstacle cells (dilated by the inscribed radius for connectivity) that contains the first blocked
sample's nearest cell; obstacles closer than twice the inscribed radius merge into one. The
**blocking set** keeps, for every blocked sample, its station along the path and its nearest
cell in $K$. The start and exit tests measure to the part of the blocking set still ahead of the
robot, so an obstacle merged with a long wall parallel to the path does not turn the episode into
wall-following.

### Start and side probe

An episode starts in a cycle in which the gate is open and no goal veto applies, the evaluation
point is within $D'_{\mathrm{in}}$ of the blocking set and the path field points toward it, and the
side probe finds a side that gets past the blockage.

The **side probe** rolls the controller out twice, with the contour forced on at side +1 and at
side −1, for

$$
T_{\mathrm{side}} = \max\big(T_h,\ (D'_{\mathrm{in}} - \lambda + \pi\lambda + L_B)/v_r\big),
$$

long enough for the approach, half a circulation and the length $L_B$ of the blocking set (about
4.6 s for a TB3 and a point obstacle). The sides are ranked by: collision-free; reaches the
episode exit past the blockage; larger clearance (saturated at $2m_s$); shorter path to the exit;
larger free arc; the side the path field already leans to; +1. If neither side can stop in time
($s_{\mathrm{free}} \ge v_r^2/(2a_{\mathrm{dec}}) + m_s$), no episode starts.

**Dead ends.** With `path_blocked`, a side counts only if its rollout exits the episode at a path
station beyond the first blocked station; an exit reached by retreating does not count. In a
dead end, such as a corridor closed across the path, neither side passes, so no episode starts:
the cap stops the robot short of the blockage, `BLOCKED` is raised and Nav2 replans. The price is
that an obstacle the probe cannot pass within $T_{\mathrm{side}}$ (in practice, a wall across the
path wider than about 2 m) also stops the robot for a replan instead of being contoured.

### During the episode

- The circulation side $\sigma_c$ is latched for the whole episode. This removes the singularity
  of the thesis formulation and the trap basin of a head-on approach
  ({ref}`sec-contour-trap`).
- The episode ends when the path field no longer points toward the blocking set, when the
  blocking set is behind the robot, when the distance to $K$ exceeds $D'_{\mathrm{in}}$ plus a small
  hysteresis, when the gate closes, when $K$ is lost, or on the goal veto.
- **Watchdog:** `obstacle_max_contour_length` (5.0 m of base travel) or
  `obstacle_max_contour_time` (20 s) per episode. A trip sets `STATUS_CONTOUR_WATCHDOG` and the
  STOPPED phase: the plugin raises `NoValidControl`, and the curve follower ends the goal with
  `BLOCKED`.
- No episode starts while cancelling, in STOPPED, while holding the goal or in
  ROTATE_TO_HEADING.

The `contour` markers show the blocked gate samples, $K$'s cells, the closest obstacle point,
the $\lambda$, $D_{\mathrm{in}}$ and $D'_{\mathrm{in}}$ rings, the side arrow and the watchdog text
`contour +1 | 1.3/5.0 m | 4/20 s`.

## Limits

- **Local costmap only.** Contouring and prediction see the rolling local costmap. The blend zone
  and the rollout must fit in it; the configuration validator warns when $D'_{\mathrm{in}} + d$ or
  `collision_check_horizon` × `max_linear_vel` reach the window edge ({doc}`tuning`).
- **Static obstacles.** Each cycle uses a snapshot; obstacle motion is not predicted.
- **Narrow gaps.** Gaps narrower than about $2\lambda$ act as closed for contouring, and obstacles
  closer than twice the inscribed radius merge. A free passage such as a door is not affected,
  because the gate stays closed while the footprint fits along the path.
- **Ackermann:** `obstacle_mode: contour` is rejected in 0.x. Obstacles stop the car, and Nav2
  replans.
- **Rotation shim:** the shim undoes contour episodes by re-aiming at the path; see
  {doc}`motion_models`.
- **Global planning still matters.** Contouring handles obstacles the global plan does not know
  yet. With the stock global obstacle layer, the planner routes around an obstacle once the
  global costmap marks it, and the contour rarely needs to engage.
