# M7: control-barrier-function obstacle mode

`obstacle_mode: cbf` is a third obstacle mode beside `none` and `contour`. Instead of switching to
a circulation field, it changes the path field as little as possible so that the evaluation point
keeps a safe distance from every obstacle: a CBF safety filter. The changes to {doc}`math_spec`
are tagged [Design, M7] in §6.7 (new), §8.1, §10.4, §11, §12.5 (O58) and §13. The contour, the
collision rollout, the braking cap and kBlocked are unchanged. The rollout is the safety backstop
in every mode.

**Barrier.** $h_j(x) = D_j(x) - \delta_{\mathrm{safe}}$, with
$\delta_{\mathrm{safe}} = r_e + m$, $m$ = `cbf_margin`. $r_e$ is the radius of the smallest disc
centred on the evaluation point $x$ that contains the footprint. It is the largest distance from
$x$ to a footprint vertex, or $R + \lvert e\rvert$ for a disc of radius $R$ when $x$ lies
$\lvert e\rvert$ from base_link. $D_j$ is measured at the law's evaluation point (§7.1). Omni and
heading tracking use base_link, so $r_e = r_{\mathrm{circ}}$. FL uses $p_c$, so
$r_e \le r_{\mathrm{circ}} + d$ (TC: 0.42 m). A reverse segment and FINAL_APPROACH switch to the
flipped control point or to base_link. The disc contains the footprint whatever the heading, so
$h \ge 0$ keeps every footprint point at least $m$ from every site centre. The sites are cells,
so the clearance to the cell squares is $m - \mathrm{res}/\sqrt2$. FL is conservative sideways:
a wall parallel to the path holds the base $r_e - w_{\mathrm{lat}}$ farther away than needed.
That is the price of the exact law below.

**Exact where it can be.** FL makes $\dot p_c = F$ (§7.3), and holonomic makes $\dot p = F$. So
$\dot h_j = n_j^\top F$, and the condition on $F$ is the CBF condition itself. Heading tracking
gives $\dot p = v\mathbf h$ with $v \approx \lVert F\rVert\cos e$. The condition then holds only
as $e \to 0$: approximate. The heading lag, and the CommandLimiter's acceleration bounds in every
law, can take $h$ below 0. The rollout runs the filtered pipeline, limiter included, against the
real footprint, so a predicted contact brakes through the cap (§10.3). Ackermann is rejected
(`ackermann_cbf_unsupported`). A car cannot move along an arbitrary $F^\star$, and the disc about
the rear axle is large. Cars keep the $R_{\min}$-aware contour (§6.6).

**Filter.** With $F$ the shaped path field $\Phi_\gamma$ and $\gamma_b$ = `cbf_gain`:
$F^\star = \arg\min \lVert F' - F\rVert^2$ s.t. $n_j^\top F' \ge -\gamma_b h_j$, $j \le k$.
For one constraint the closed form is $F^\star = F + \max(0,\ -\gamma_b h - n^\top F)\,n$.
`cbf_sites` $k \le 3$ constraints are used, as a small active-set QP. In 2D at most two
constraints are active, so the solver enumerates the unconstrained point, the 3 single
projections and the 3 pair intersections (KKT multipliers $\ge 0$), on the stack.
Constraint 1 is the continuous $\mathrm{EDT}_{\mathrm{all}}$ query of §6.1, which is smooth along
walls. Constraints 2–3 are the nearest sites among the features of the 7 × 7 cells around $x$
whose normals are more than 45° from those already taken. Sites of one convex obstacle merge,
and the two sides of a gap or a doorway stay separate. With one constraint, the nearest site
jumps from one jamb to the other at the bisector, and $\omega$ chatters. With both jambs in the
QP, the active set is the same on both sides.

**Speed and $h < 0$.** With every $h_j \ge 0$ the feasible set is convex and contains 0, and
projection is non-expansive. So $\lVert F^\star\rVert \le \lVert F\rVert$, and the filter never
speeds the robot up. Inside the margin, $-\gamma_b h_j > 0$ asks for a push out along $n_j$.
It is bounded: $F^\star$ is scaled to $\lVert F^\star\rVert \le \lVert F\rVert$. If the pushes
conflict (squeezed between two obstacles), the QP is solved again with
$\min(-\gamma_b h_j, 0)$, which is always feasible ($F' = 0$). The robot then holds without
pushing into the other obstacle. When the filter binds, the box is uniform (§8.3), since
$F^\star$ is no longer $\eta_1\Phi_S + \Phi_T$.

**Discrete time.** Inside one site's Voronoi cell, $D_j$ is the distance to a point, which is
convex. So under ZOH, $D_j(x + F^\star\Delta t) \ge D_j + n_j^\top F^\star\Delta t$, i.e.
$h_{k+1} \ge (1 - \gamma_b\Delta t)\,h_k$ for FL and omni. This is up to the $O(\lVert F\rVert
\omega\Delta t^2)$ arc of the held command, and it needs $\gamma_b\Delta t < 1$. A head-on
approach at $v_r$ turns the filter on at $h = v_r/\gamma_b$. From there it asks for the
deceleration $\gamma_b v_r$, so `cbf_gain_deceleration` warns when $\gamma_b v_r > a_{\mathrm{dec}}$.

**Deadlock: accepted.** When $F \parallel -n$ (a disc centred on the path, or any face across
the path, where §6.1 makes $n$ exact), $F^\star \to 0$ and the robot stops at $h \approx 0$.
The rollout then predicts no contact, so the cap alone would never set kBlocked. A *CBF hold*
therefore sets kBlocked when the filter binds at the current pose of a translation phase with
$\lVert F^\star\rVert < 0.05\lVert F\rVert$, and releases it at 0.10. The settled, goal-hold and
XY-reached exclusions of §10.4 apply. kBlocked commands zero, so the hold persists until the map
or the path changes. The plugin raises NoValidControl and Nav2 replans: once the costmap shows
the obstacle, the planner routes around it. The follower reports BLOCKED after
`blocked_timeout`. A latched tie-break $\sigma J n$ was rejected. Without the side probe, it
picks the long way or a dead end about half the time. Once latched, it follows walls and needs
the contour's exit and watchdog rules. That is the contour mode, which already exists. Near
discs, the head-on point is unstable for $G'(0)\rho < 1$ (§6.3). Off-centre discs slide off the
hold and pass, and a sufficiently symmetric approach stops.

**Interaction.** `cbf` and `contour` are exclusive modes. The gate, $K$, the blocking set, the
side probe and the watchdog are not used. Once XY is reached, the filter is off: the robot only
settles, under the rollout's protection. A goal within $\delta_{\mathrm{safe}}$ of an obstacle
would otherwise never settle. The probes of heading tracking ($\dot\psi_d$) and the
ROTATE_TO_FIELD decision filter their fields too. Reverse segments use the flipped law's point;
the filter keeps no state.

**Parameters** (additive): `cbf_gain` $\gamma_b$ (2.0 s⁻¹, $> 0$); `cbf_margin` $m$ (0.05 m,
$\ge 0$); `cbf_sites` $k$ (3, in $[1, 3]$). Warnings: `cbf_gain_deceleration`;
`cbf_gain_discrete` ($\gamma_b\Delta t \ge 1$). Error: `ackermann_cbf_unsupported`.
`FieldSample::cbf` carries $\delta_{\mathrm{safe}}$, $\min h_j$, the constraints, which of them
bind, $\lVert F^\star\rVert/\lVert F\rVert$ and the hold. The `cbf` marker namespace draws the
$\delta_{\mathrm{safe}}$ disc, coloured by the sign of $h$, the constraint normals (bold when
they bind) and a text with $h$ and the speed ratio. FieldState is unchanged:
`obstacle_distance` is $D_1$.

**Tests.** O58 (core, TC footprint, measured by brute force to the site centres). (a) A 0.15 m
disc 0.1–0.3 m off the path, FL, HT and omni: it passes, with $\min h \ge -\mathrm{res}$ (worst
−4 mm), and rejoins with $\lvert D\rvert < 0.02$. (b) A doorway of half-width
$\delta_{\mathrm{safe}} + 0.03$: the contour never engages. The CBF binds at the jambs, keeps
$\lvert D\rvert < 0.02$ and passes at $\ge 0.6\,v_r$ (0.36 m/s FL, 0.29 base_link). (c) A centred
disc on a symmetric grid: hold and kBlocked, with the robot at rest at $h = 9$ mm. (d) A wall
0.45 m from the path: the speed stays $\ge 0.9\,v_r$, and FL holds $p_c$ 2 cm off the path. (e) Two
discs, wide gap: the robot passes, with per-cycle $\Delta\omega$ or $\Delta v_y \le 0.05$. Narrow
gap ($< 2\delta_{\mathrm{safe}}$): three sites hold with $h \ge 0$, while one site lets FL squeeze
through at $h = -4$ cm with $\omega$ jumps of 1.1 rad/s per cycle. (f) ZOH at 20 Hz, FL and omni
at 0.4 and 0.5 m/s: $\min h \ge -\mathrm{res}$ (worst −5.1 mm). Layer 2 (FL, omni): a box 0.12 m beside the plan
passes with clearance $\ge m - \mathrm{res}$, and a box across the plan holds until the
patience is exceeded. The follower passes a box 0.1 m outside a 1 m circle without BLOCKED.
System test (diff, omni): a box 0.1 m beside the path succeeds with clearance
$\ge m - \mathrm{res}$. With a box across the path, the robot holds and raises NoValidControl,
FollowPath fails and Nav2 replans.
