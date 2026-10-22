# Obstacle avoidance

Three mechanisms act on obstacles. **Collision prediction** is always on (unless
`use_collision_detection` is false): it slows the robot down and stops it short of a predicted
collision. **Obstacle contouring** (`obstacle_mode: contour`, default `none`) is the obstacle
extension of the thesis (§3.4): when the path ahead is blocked, the field circulates around the
obstacle at a set distance and returns to the path. The **CBF mode** (`obstacle_mode: cbf`)
changes the field as little as possible so that a margin is kept from every obstacle
({ref}`cbf-mode`). The underlying field is summarised in {doc}`theory`. The exact rules, with
their rationale, are in {doc}`design/math_spec` ({ref}`sec-contour-mods`, {ref}`sec-cbf` and
{ref}`sec-collision`). The `contour` and `prediction` markers are described in
{doc}`visualization`.

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

(ackermann-contour)=

### Ackermann (car-like) bases

A car cannot turn tighter than `min_turning_radius` $R_{\min}$ and cannot rotate in place, so the
contour of an Ackermann base differs in four ways ({ref}`sec-contour-ackermann`,
{doc}`design/m7_ackermann_contour`):

- **Larger distances.** The field is evaluated at the rear axle (`reference_point_offset`). Around
  a convex obstacle corner the circulation is a circle of radius about $\lambda$, so the auto values
  are

  | Parameter | Ackermann auto value | TB3 footprint, $R_{\min}$ = 0.5 m |
  |---|---|---|
  | `obstacle_contour_distance` $\lambda$ | $\max(\text{the value above},\ R_{\min} + w_{\mathrm{lat}} + 0.05)$, with $d = \lvert x_r\rvert$ | 0.77 |
  | `obstacle_full_switch_distance` $D_{\mathrm{in}}$ | $\lambda + \max(0.15, R_{\min})$: room for a quarter turn | 1.27 |
  | `obstacle_blend_start_distance` $D'_{\mathrm{in}}$ | $D_{\mathrm{in}} + 0.2$ | 1.47 |
  | `obstacle_lookahead` | $\max(1.0,\ v_r T_h,\ D'_{\mathrm{in}} + 0.1)$ | 1.57 |

  An explicit $\lambda < R_{\min} + 0.05$ is rejected. The validator warns when $\lambda$, $D_{\mathrm{in}}$,
  the lookahead or the watchdog length is too small for the turning radius ({doc}`tuning`). The
  blend band must fit in the local costmap: with $R_{\min}$ = 0.5 m use at least a 4 m × 4 m
  window, as the shipped `ackermann.yaml` does.
- **Turning while the field turns.** During an episode the car keeps at least half the field
  speed and turns at $R_{\min}$ toward the field, even when the field points more than 90° away
  from its heading. Outside an episode that case is `STATUS_HEADING_INFEASIBLE`. Commands always
  satisfy $\lvert\omega\rvert \le \lvert v\rvert / R_{\min}$ (`STATUS_CURVATURE_SATURATED`
  while the turn is at the limit).
- **Late detection.** An obstacle first seen inside $D_{\mathrm{in}}$ is passed as closely as a turn
  at $R_{\min}$ allows. A 0.4 m box 0.8 m ahead of a car at rest is passed at 0.35 m from the
  rear axle, not at $\lambda$.
- **Blocked instead of contoured.** The side probe rolls out the car's real kinematics. When
  neither side can be driven, the car stops and Nav2 replans: a gap narrower than about
  $\lambda + w_{\mathrm{lat}}$, a dead end, or a turn tighter than $R_{\min}$. An episode never
  changes direction: on a reverse plan segment (0.3.0) the car contours backing up, at the rear
  axle, with the same rules ({ref}`reversing`).

(cbf-mode)=

## CBF mode

With `obstacle_mode: cbf`, a control-barrier-function filter acts on the path field every cycle
({ref}`sec-cbf`, {doc}`design/m7_cbf`). It has no episodes, no gate and no side to choose. The
barrier is $h = D_o - \delta_{\mathrm{safe}}$. $D_o$ is the obstacle distance at the point where
the law evaluates the field: the control point for feedback linearization, base_link for
holonomic and heading tracking. $\delta_{\mathrm{safe}}$ is the radius of the smallest disc
around that point that holds the footprint, plus `cbf_margin`. With the TB3 footprint that is
0.47 m at the control point and 0.27 m at base_link. The filter returns the field closest to the
path field for which $h$ shrinks no faster than $\gamma_b h$ (`cbf_gain`). The robot can then
approach an obstacle at most at speed $\gamma_b h$, and slide along it at any speed. The filter
never speeds the robot up. Inside the margin ($h < 0$) it pushes the robot out, no faster than
the path field's speed.

:::{figure} _static/figures/cbf_disc.svg
:alt: A robot on a straight path passes a disc 0.2 m off the path, keeping delta_safe from it, then returns to the path and stops at the goal.
:width: 100%

The CBF filter around a disc 0.2 m off a straight path (differential drive,
`obstacle_mode: cbf`, defaults). Dashed: $D_o = \delta_{\mathrm{safe}}$ (0.47 m at the control
point). The control point slides along that level set and the base passes below it. Orange
arrows are where the filter changes the field, seen from the pose where it first binds (black
outline); inside the margin they point out. Reproduce it with
`vf-sim --curve line.csv --pose -3,0,0 --circle 0,0.2,0.3 --obstacle-mode cbf -o cbf.svg`.
:::

- **Exact or approximate.** Feedback linearization moves its control point exactly along the
  commanded field, and holonomic bases move base_link along it. For these laws the filter is the
  CBF condition itself, up to the zero-order hold and the acceleration limits. Heading tracking
  turns toward the field, so the condition holds only once the heading has converged. The
  collision rollout runs the filtered pipeline, so it still brakes the robot before a contact.
- **Ackermann** (0.3.0). The filter runs at the rear axle, which moves along the heading without
  slipping. $\delta_{\mathrm{safe}}$ is the disc about the axle that holds the body (0.635 m for a
  0.7 × 0.4 m car with the axle 0.15 m from the rear bumper). A car cannot turn on the spot, so
  each obstacle it approaches gets a turning margin $R_{\min}(1 - \sin\beta)$, with $\beta$ the
  angle between the drive direction and the obstacle: the distance a full-lock turn covers toward
  a wall before the car is parallel to it. The filter keeps $h$ above that margin. So head-on it
  acts about `min_turning_radius` earlier, and parallel to a wall it acts as for any robot. Where
  the margin is used up, the car turns at full lock. The filter never pushes a car sideways out of
  the margin, and the speed band `ackermann_min_speed` applies until the hold stops it.
  Head-on, the car holds `min_turning_radius` + $\delta_{\mathrm{safe}}$ from the face, at the
  axle, where it can still turn away; a gap narrower than $2\delta_{\mathrm{safe}}$ also stops
  it. Reverse plan segments filter along the drive direction. The validator warns when a face
  straight ahead would enter the local costmap inside the margin (`ackermann_cbf_window`), and
  when `cbf_margin` is below the stopping distance from `ackermann_min_speed`
  (`ackermann_cbf_margin`). Design and tests: {ref}`cbf-ackermann`.
- **Several obstacles.** `cbf_sites` (3) obstacles are constrained at once: the nearest one, and
  the nearest of every other obstacle whose direction differs by more than 45°. The two sides of
  a gap or doorway are therefore both in the filter, and their lateral pushes cancel. With one
  site, the nearest obstacle switches from one side to the other in the middle of a gap, and the
  command chatters.
- **Doorways and walls.** A passage wider than $2\delta_{\mathrm{safe}}$ is passed on the path. Near
  its jambs the robot slows to about $\gamma_b\sqrt{2ec}$, with $e$ the spare width per side and
  $c$ the jamb's lateral distance (O58 (b): 0.36 m/s at $e$ = 3 cm). A wall parallel to the path
  costs no speed. The filter only removes the approaching component.
- **Stops instead of circulating.** When the field points straight at an obstacle, the filter
  stops the robot at $h \approx 0$. This happens with a face across the path, or a disc centred on
  it. The *CBF hold* then raises `STATUS_BLOCKED`: the filter binds and leaves less than 5 % of
  the field speed. The plugin raises NoValidControl, and Nav2 replans around the obstacle once
  the global costmap shows it. The follower aborts with `BLOCKED` after `blocked_timeout`. A gap
  narrower than $2\delta_{\mathrm{safe}}$ also stops the robot. The design note explains why there
  is no tie-break circulation.
- **Tuning.** A head-on approach at $v_r$ turns the filter on at $h = v_r/\gamma_b$, and then asks
  for the deceleration $\gamma_b v_r$. The validator warns when that exceeds `max_linear_decel`
  (`cbf_gain_deceleration`). A larger `cbf_gain` passes doorways faster but brakes harder.
  `cbf_margin` should be at least $\mathrm{res}/\sqrt2$, since distances are measured to cell
  centres.

The `contour` markers show the CBF instead in this mode. They draw the constrained sites, the
normals of the constraints that bind, the $\delta_{\mathrm{safe}}$ disc about the evaluation
point (red while $h < 0$), the filter's correction and the text `cbf h 0.12 m | 1/2 bind | 85 %`.
The status text shows `CBF` while the filter binds.

The experimental 3D core has the same filter for a spherical body among spheres, boxes, poles,
planes and voxels ({ref}`core-3d-obstacles`, 0.3.0).

## Contour or CBF

| | `contour` | `cbf` |
|---|---|---|
| Acts | only when the path ahead is blocked (gate), until the obstacle is passed | every cycle, wherever an obstacle is within reach of the barrier |
| Obstacle on the path | circulates around it on the side the probe chose; rejoins | stops in front of it; `STATUS_BLOCKED`, Nav2 replans |
| Obstacle beside the path, inside the footprint's reach | gate opens; an episode circulates at $\lambda$ | minimal deviation to $\delta_{\mathrm{safe}}$, then back |
| Doorways, corridors | unaffected while the footprint fits (gate closed) | passed on the path if wider than $2\delta_{\mathrm{safe}}$; may slow near the jambs |
| Guarantee | none (thesis extension, latched side, watchdog) | $h \ge 0$ for FL and holonomic up to discretisation; approximate for heading tracking and Ackermann |
| State | episode, side, $K$, watchdog | none |
| Ackermann | supported, with turning-radius distances | supported (0.3.0), at the rear axle with a turning margin |
| Use when | obstacles appear on the path that the planner does not see in time, or the follower must get around them on a fixed curve | the planner routes around obstacles (stock global obstacle layer) and the controller should keep a margin and pass narrow spaces smoothly |

## Limits

- **Local costmap only.** Contouring and prediction see the rolling local costmap. The blend zone
  and the rollout must fit in it; the configuration validator warns when $D'_{\mathrm{in}} + d$ or
  `collision_check_horizon` × `max_linear_vel` reach the window edge ({doc}`tuning`).
- **Static obstacles.** Each cycle uses a snapshot; obstacle motion is not predicted.
- **Narrow gaps.** Gaps narrower than about $2\lambda$ act as closed for contouring, and obstacles
  closer than twice the inscribed radius merge. A free passage such as a door is not affected,
  because the gate stays closed while the footprint fits along the path.
- **CBF:** no circulation; obstacles across the path and gaps narrower than
  $2\delta_{\mathrm{safe}}$ stop the robot ({ref}`cbf-mode`). With feedback linearization the
  barrier disc is centred on the control point, so lateral clearances exceed the footprint's by
  up to `control_point_offset`.
- **Ackermann:** the car contours with larger distances ({ref}`ackermann-contour`). An obstacle
  it cannot drive around without changing direction stops it, and Nav2 replans. With cbf, the
  barrier disc about the rear axle is wide beside the car, and a face across the path holds it
  `min_turning_radius` + $\delta_{\mathrm{safe}}$ short.
- **Reverse plan segments** (0.3.0): obstacles are contoured while backing up, at the law's point
  on the drive side, at most at `max_reverse_vel` ({ref}`reversing`).
- **Rotation shim:** the shim undoes contour episodes by re-aiming at the path; see
  {doc}`motion_models`.
- **Global planning still matters.** Contouring handles obstacles the global plan does not know
  yet. With the stock global obstacle layer, the planner routes around an obstacle once the
  global costmap marks it, and the contour rarely needs to engage. The `demo_obstacles` scenario
  of `vectorfield_bringup` turns the global obstacle layer off to show contouring (see
  {doc}`quickstart`).
