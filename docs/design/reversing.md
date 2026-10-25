# Reversing (plans with cusps) and Nav2 path constraints

Plans with reversing segments are followed as a sequence of segments with an intermediate stop
at each cusp. The changes to {doc}`math_spec` are tagged [Design] in §1, §2.8, §5.2, §7.1,
§7.5, §9.2, §9.4 and §13. Nav2 facts come from `navigation2` `lyrical`:
`nav2_controller/plugins/feasible_path_handler.cpp` (FPH), `nav2_util/src/path_utils.cpp`
(`findFirstPathConstraint`) and `nav2_controller/src/controller_server.cpp`.

**Enabling.** `allow_reversing: true` turns on segment following; the default `false` keeps 0.1:
a cusp throws `InvalidPath("plan contains reversing segments; use a forward-only planner")` and
plan headings are ignored. The parameter already means "speed against the drive direction is
allowed", so a planner that reverses needs no second switch, and nobody gets reversing without
asking for it. A forward plan under `allow_reversing: true` runs exactly as before (one segment).

**Segmentation** (plugin, once per raw plan, plan frame; math_spec §2.8):

1. Cusps: `findCusps` (§2.8 [Design, 0.2.1]). The chord decimation ($\ell_c$; reversals shorter
   than $\ell_c$ are still ignored) gives candidates, turns of more than 90°; the raw cusp is the
   pose between the two neighbouring chord vertices with the largest projection on the incoming
   chord. A candidate is a cusp where the headings change the drive sign or, without headings,
   where the plan doubles back for more than $2\ell_c$ (legs within 3° of opposite and a cell
   apart). A sharp forward corner is not one (0.2.0 split every candidate, so with
   `allow_reversing` the leg after a 150° NavigateThroughPoses corner was driven backward).
   Segments run from cusp to cusp.
2. Direction $q \in \{+1, -1\}$ of each segment (body $+x$ or $-x$). A plan *has headings*
   (`planHasHeadings`, 0.2.1) when yaws aligned with the motion (within 30°, either way) cover 90%
   of its length and an oriented run of them is at least $\ell_c$ long: Smac Hybrid and Lattice
   (the vehicle yaw) and smoothed Smac 2D plans (along the path) have them; NavFn, Theta\* and
   unsmoothed Smac 2D leave the poses at the identity, and the oriented waypoint poses of a
   NavigateThroughPoses plan are single steps, so these have none. With headings,
   $q = \operatorname{sgn}\sum_k (p_{k+1} - p_k)^\top \mathbf h(\psi_k)$ over the segment when
   $\lvert\sum\rvert \ge L_{\mathrm{seg}}/2$; otherwise, and without headings, the first segment
   takes the configured sign ($-1$ iff `drive_backward`), or its opposite when the plan starts
   behind the robot in that direction (RPP's `allow_reversing` rule; Smac emits exact identity
   orientations for straight motion along $\pm x$, so its plan for a goal straight behind a car
   facing $+x$ has no headings), and the sign alternates at each cusp. So a plan that is reverse
   from the start (Smac: goal behind the car) is driven in reverse.
3. *Segmented mode*: `allow_reversing`, and the plan has a cusp or a segment whose $q$ differs
   from the configured sign. Otherwise the plan is one forward segment, as in 0.1.

**Window.** In segmented mode the plugin's `LegacyPathHandler` (the parity-tested port of FPH) is
given only the active segment, on every distro: the window ends at the next cusp by construction,
and its closest-pose search never sees the next leg, which often overlaps the current one (FPH's
`min_by` over the full plan can jump onto the return leg and prune the cusp away). On Lyrical the
server's window is then ignored; its goal check still uses the full plan's end.

**The stop.** The active segment's goal is its cusp: `GoalInfo{cusp, has_yaw = false,
xy_tol = 2 cusp_tolerance}` with the core's XY latch. The terminal field brings the robot to the
cusp with the reference-point shift taken along the drive direction ($x_{\mathrm{ref}}$ is the
offset of the law's point along $q\mathbf h$: $d$ for FL, $q\,x_r$ for Ackermann; §5.2). Settled
(§9.1, $\sigma_{\mathrm{stop}}$ = `cusp_tolerance`, 0.05 m) holds zero; the next cycle switches
the segment: the window, $q$ (`TrackerInput::drive_sign`) and the core goal identity. No
ROTATE_TO_HEADING at a cusp. The new goal identity resets the goal-scoped state of §9.1
(terminal mode, XY latch, goal hold, Ackermann band, FINAL_APPROACH); a change of $q$ resets the
direction-scoped state in the core (hint, last tangent, $\psi_d$ hold, rotate target, contour
episode, kBlocked latch). The limiter is kept, so the speed passes through zero within its bounds.

**Replans** (1 Hz from the robot). The new plan is segmented again and its first segment becomes
active, except that a first segment shorter than $2\ell_c$ whose $q$ differs from the active one
is skipped: the robot is at that cusp, and a ±1 cell move of the cusp must not flip the direction
twice. The core goal identity is kept while the stop moves less than $2\ell_c$ with the same $q$.

**Motion models.**

| Model | Reverse segment |
|---|---|
| diff, skid, FL | control point $p_c = p + q\,d\,\mathbf h$ on the drive side; ROTATE_TO_FIELD on $\psi_{\mathrm{dr}} = \psi + \pi[q = -1]$; FINAL_APPROACH hands over to HT with $q$ |
| diff, skid, HT | $\psi_{\mathrm{dr}}$, $v = q\lVert F\rVert\max(0, \cos e)$ |
| Ackermann | HT at the rear axle with $q$ (0.1: always $+1$): $v < 0$, $\lvert\omega\rvert \le \lvert v\rvert/R_{\min}$ (symmetric), the $v_{\mathrm{ack}}$ band on $q v$; kHeadingInfeasible measured against $\psi_{\mathrm{dr}}$; the configured `drive_backward` stays ignored |
| omni | no drive direction ($q$ is ignored), but cusps are still stops: the legs overlap, and a closest point that jumps between them would cut the reversal short |

**Safety.** The rollout integrates the signed command, so footprint checks follow the backward
motion; $s_{\mathrm{free}}$ is path length and kBlocked is direction-free. Until 0.3.0 the
obstacle contour was off in a segment whose $q$ differed from the configured sign and reverse
segments used the forward speed; both are now handled (see *Reverse speed and contour*, below).

**Nav2 path constraints (Lyrical, FPH).** `enforce_path_inversion` truncates the window at the
first raw-pose inversion (no decimation, no headings: any raw turn of more than 90°, so a sharp
forward corner too; keep it off with headingless planners), and FPH moves past it once the robot
is within `inversion_xy_tolerance` (0.2 m) and `inversion_yaw_tolerance` (0.4 rad) of that pose.
The server checks its goal against the full plan's last pose, and `SimpleGoalChecker` also needs
its window to be shorter than `path_length_tolerance` (1.0 m), so the server never reports success
at a cusp.
In segmented mode the plugin builds its own window, stops within `cusp_tolerance` of the cusp
with the plan's heading (inside FPH's tolerances, so FPH advances and the server's window keeps
pace), and never needs the truncation. `enforce_path_rotation` truncates at in-place rotations
of the plan; on Lyrical the plugin uses its own window for any plan with an interior in-place
rotation (whatever `allow_reversing`), and the robot turns there by ROTATE_TO_FIELD, heading
tracking or the omni heading policy. Without either constraint the two windows are identical
(parity test), so nothing changes. Humble and Jazzy have no path handler: `LegacyPathHandler`
never truncates, and segment windows give the same behaviour; nothing else is mirrored.

**Parameters** (additive): `cusp_tolerance` (Phases, 0.05 m, > 0; plugin only); `max_reverse_vel`
(Motion, 0.3.0, below). `allow_reversing` documents segment following. `TrackerInput::drive_sign` (0 = configured) and
`TrackerState::drive_sign` are new core fields; the 0.x API stays source-compatible.

**Visualization.** The active cusp is the `goal` namespace (point, XY ring at 2 `cusp_tolerance`,
settle ring at `cusp_tolerance`); the status text adds `REV` while $q = -1$.

**Limits.** No reversal shorter than $\ell_c$ is a stop; a plan without headings alternates
direction blindly, and reverses only where it doubles back for more than $2\ell_c$ (an out-and-back
waypoint route on NavFn is one). A smoother that orients a headingless plan (nav2_smoother) marks the legs that leave
toward $-x$ of the identity as reverse, and they are driven so.

## Reverse speed and contour (0.3.0)

Real bases back up slower than they drive forward, and sense less behind them. 0.3.0 adds a
reverse speed bound and runs the obstacle contour (and checks the CBF filter) on reverse
segments. The changes to {doc}`math_spec` are tagged [Design, 0.3.0] in §1, §6.4, §6.6, §6.7,
§8.3, §9.4 and §11. Forward behaviour is bit-identical: every forward quantity below is the 0.2
value, and the existing oracles are unchanged.

**The reverse.** Reverse means $q = -1$, the body driving toward $-x$: a reverse plan segment, or
`drive_backward` (whose robots back up all the time). Omnidirectional bases have no drive
direction and ignore everything below.

**`max_reverse_vel`** $v_{\mathrm{rev}}$ (Motion; default $-1$ = auto = `max_linear_vel`, so nothing
changes until it is set; error unless $0 < v_{\mathrm{rev}} \le v_{\max}$; Ackermann: error when set
below $v_{\mathrm{ack}}$, where the band could not be commanded). With $q = -1$:

- the field speed is $v_r^{-} = \min(v_r, v_{\mathrm{rev}})$ (`referenceSpeed()`). Everything built on
  $v_r$ uses it: the distance field's $\lVert\Phi\rVert$, the terminal profile (so the cusp approach
  and the final approach of a reverse goal brake from $v_r^{-}$), the collision scale
  $\gamma_{\mathrm{col}} = \min(1, v_{\mathrm{cap}}/v_r^{-})$, the hold speed, the Ackermann floor
  $v_{\mathrm{ack}}/v_r^{-}$, the side probe's $T_{\mathrm{side}}$ and its $v_r^2/(2a_{\mathrm{dec}})$
  test; the external speed limit $\gamma_{\mathrm{lim}}$ scales $v_r^{-}$ like $v_r$ (an absolute
  Nav2 limit is a fraction of `desired_speed`, so it is conservative in reverse);
- the kinematic box bounds $\lvert v\rvert \le \min(v_{\max}, v_{\mathrm{rev}})$
  (`Kinematics::linearLimit()`, §8.3), on the law's command and therefore in the rollouts and the
  side probe, which run the same pipeline;
- the Ackermann band raises $q v$ to $v_{\mathrm{ack}}$, i.e. $\lvert v\rvert$ in reverse (unchanged).

Configure-time quantities stay forward: `obstacle_lookahead` (auto $\max(1, v_r T_h)$, longer than
the reverse needs) and the §11 rules on $v_r$ and $v_{\max}$ (each holds a fortiori for
$v_r^{-} \le v_r$). The rollout's pose reservation keeps $v_{\max}$, an upper bound. No rear-sensor
warning: the core does not know the sensors; {doc}`../tuning` gives the advice. The curve
follower's progress supervision uses $v_r^{-}$ for a `drive_backward` robot.

**Contour on a reverse segment.** The Tracker already flips its kinematics and law for $q = -1$
(the configured kinematics with `drive_backward` toggled, as for a `drive_backward` robot, whose
contour has always run). Each piece of §6.4 is stated in terms that carry over:

- *Evaluation point and footprint side.* The contour runs at the law's point on the drive side:
  FL's $p_c = p - d\,\mathbf h$ (behind base_link), HT's base_link, Ackermann's rear axle $p_r$ (the
  no-slip point in both directions). $\lambda$'s auto value and the inscribed rule use
  $\lvert d\rvert$ and the base_link radii, which are direction-free; the leading part of the
  footprint is the rear, and the rollout checks the real polygon.
- *Gate.* The plugin passes only the active segment, which is traversed toward increasing
  station whatever $q$; the gate samples $[s^\star, s^\star + \ell_{\mathrm{look}}]$ ahead along the
  reverse leg, i.e. behind the body. The blocking set, $B(s^\star)$ and the past-the-blockage test
  are station-based and unchanged.
- *Side probe.* It rolls out `stepNominal` with the flipped kinematics: diff and skid FL with the
  control point behind, HT on $\psi_{\mathrm{dr}} = \psi + \pi$, Ackermann HT with $v < 0$ and
  $\lvert\omega\rvert \le \lvert v\rvert/R_{\min}$, all through the box at $v_{\mathrm{rev}}$ and the plant's
  exact integration of the signed command. A side the robot cannot back around collides or never
  gets past the blockage.
- *Circulation sign.* $\sigma_c$ latches a direction of the field $T_\lambda = \sigma_c J n_o$ in the
  plane; the probe ranking and its tie-break ($\operatorname{sgn}((Jn_o)^\top\Phi)$, then $+1$) never
  refer to the body, so the side means the same in both directions.
- *Exits and watchdog.* The exit tests use $n$, $\Phi$ and $D_o$ at the evaluation point (the
  Ackermann `b_site` fallback too); the watchdog counts $\lVert(v_x, v_y)\rVert\Delta t$, a path length.
- *Dead end.* Unchanged rule: no episode without a collision-free side past the blockage; the
  cap stops the robot while backing up and kBlocked follows (NoValidControl, replan).
- *Segment switch.* A change of $q$ still ends an active episode and clears kBlocked
  (`resetDirectionScoped`): an obstacle met in reverse is re-evaluated going forward.

So the change in the core is that the episode is no longer suppressed while $q$ differs from the
configured sign. The tests are new oracles.

**CBF in reverse.** The filter is stateless: $\delta_{\mathrm{safe}}$, the sites and the QP are
evaluated each cycle at the law's point of the active drive direction (the flipped point, §6.7),
and the CBF hold and kBlocked are direction-free. It already ran on reverse segments in 0.2; 0.3.0
adds the test.

**Tests.** Core (`test_reversing.cpp`): a cusp plan whose reverse leg has a 0.15 m disc 0.1 m
off the path, for diff FL, diff HT and Ackermann ($R_{\min}$ 0.5, $x_r = -0.2$): one episode, in
reverse only, a constant $\sigma_c$, the evaluation point at least $\lambda - \mathrm{res}$ from the
disc, the free side, no collision, settled at the goal (min clearance 0.508, 0.388, 0.774 m
for $\lambda$ 0.47, 0.32, 0.75); the O56 dead end on the reverse leg (no episode, kBlocked kept,
stopped 0.10 m short); `max_reverse_vel` 0.2 (reverse $\lvert v\rvert \le 0.2$, reached; forward
commands identical); CBF with a disc 0.2 m off the reverse leg ($\min h \ge -\mathrm{res}$, no
kBlocked). Controller layer 2: a cusp plan with a lethal box on the reverse leg (differential,
Ackermann) succeeds with the contour active only in reverse; `max_reverse_vel` bounds the
published reverse command. System (`test_reversing_kinematic`, differential): Smac
Reeds-Shepp backs up 2.7 m past an unmapped box (global obstacle layer off, NavigateToPose
replanning only when the plan becomes invalid), the base at least 0.50 m from the box (bound
$\lambda - d - 0.05 = 0.22$); the reverse ground-truth speed stays within `max_reverse_vel` of the
reversing overlay (0.2 m/s).

**Limits.** The reverse lookahead and the window are the forward ones. With Nav2's 1 Hz
replanning and a global costmap blind to the obstacle, a plan restarted from the pose a contour
has displaced can pass the obstacle closer than the footprint but not within
$w_{\mathrm{lat}} - $ `obstacle_gate_tolerance` (a Reeds-Shepp plan cannot turn back to the line
at once): the gate leaves that graze to the predictor (§6.4), the robot stops with kBlocked and
Nav2 recovers. This is not specific to reversing; the system test replans only on an invalid
plan, and a global obstacle layer, which plans around the obstacle, avoids it. The 2 m `corridor_door`
is narrower than an Ackermann circulation ($2(\lambda + w_{\mathrm{lat}})$), so the system test
runs the box case for the differential base only; the core and layer-2 tests cover Ackermann.
