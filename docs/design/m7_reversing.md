# M7: reversing (plans with cusps) and Nav2 path constraints

Plans with reversing segments are followed as a sequence of segments with an intermediate stop
at each cusp. The changes to {doc}`math_spec` are tagged [Design, M7] in §1, §2.8, §5.2, §7.1,
§7.5, §9.2, §9.4 and §13. Nav2 facts come from `navigation2` `lyrical`:
`nav2_controller/plugins/feasible_path_handler.cpp` (FPH), `nav2_util/src/path_utils.cpp`
(`findFirstPathConstraint`) and `nav2_controller/src/controller_server.cpp`.

**Enabling.** `allow_reversing: true` turns on segment following; the default `false` keeps 0.1:
a cusp throws `InvalidPath("plan contains reversing segments; use a forward-only planner")` and
plan headings are ignored. The parameter already means "speed against the drive direction is
allowed", so a planner that reverses needs no second switch, and nobody gets reversing without
asking for it. A forward plan under `allow_reversing: true` runs exactly as before (one segment).

**Segmentation** (plugin, once per raw plan, plan frame; math_spec §2.8):

1. Cusps: the chord decimation of §2.8 ($\ell_c$; reversals shorter than $\ell_c$ are still
   ignored). The raw cusp is the pose between the two neighbouring chord vertices with the largest
   projection on the incoming chord. Segments run from cusp to cusp.
2. Direction $q \in \{+1, -1\}$ of each segment (body $+x$ or $-x$). A plan *has headings* when
   some interior pose has a non-identity orientation (Smac Hybrid and Lattice: the vehicle yaw;
   NavFn, Smac 2D and Theta\* leave interior poses at the identity). With headings,
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
motion; $s_{\mathrm{free}}$ is path length and kBlocked is direction-free. The obstacle contour is
off in a segment whose $q$ differs from the configured sign (0.x): its gate, side probes and
circulation assume the drive direction; an obstacle on a reverse leg stops the robot (cap,
kBlocked, NoValidControl, replan). Reverse speed uses the same $v_r$ and box; there is no
`max_reverse_vel` in 0.x (use speed-filter zones); the Ackermann band floor applies both ways.

**Nav2 path constraints (Lyrical, FPH).** `enforce_path_inversion` truncates the window at the
first raw-pose inversion (no decimation), and FPH moves past it once the robot is within
`inversion_xy_tolerance` (0.2 m) and `inversion_yaw_tolerance` (0.4 rad) of that pose. The server
checks its goal against the full plan's last pose, and `SimpleGoalChecker` also needs its window
to be shorter than `path_length_tolerance` (1.0 m), so the server never reports success at a cusp.
In segmented mode the plugin builds its own window, stops within `cusp_tolerance` of the cusp
with the plan's heading (inside FPH's tolerances, so FPH advances and the server's window keeps
pace), and never needs the truncation. `enforce_path_rotation` truncates at in-place rotations
of the plan; on Lyrical the plugin uses its own window for any plan with an interior in-place
rotation (whatever `allow_reversing`), and the robot turns there by ROTATE_TO_FIELD, heading
tracking or the omni heading policy. Without either constraint the two windows are identical
(parity test), so nothing changes. Humble and Jazzy have no path handler: `LegacyPathHandler`
never truncates, and segment windows give the same behaviour; nothing else is mirrored.

**Parameters** (additive): `cusp_tolerance` (Phases, 0.05 m, > 0; plugin only). `allow_reversing`
documents segment following. `TrackerInput::drive_sign` (0 = configured) and
`TrackerState::drive_sign` are new core fields; the 0.x API stays source-compatible.

**Visualization.** The active cusp is the `goal` namespace (point, XY ring at 2 `cusp_tolerance`,
settle ring at `cusp_tolerance`); the status text adds `REV` while $q = -1$.

**Limits.** No reversal shorter than $\ell_c$ is a stop; a plan without headings alternates
direction blindly; a reverse segment cannot contour; one speed limit for both directions.
