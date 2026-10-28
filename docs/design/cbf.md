# Control-barrier-function obstacle mode

`obstacle_mode: cbf` is a third obstacle mode beside `none` and `contour`. Instead of switching to
a circulation field, it changes the path field as little as possible so that the evaluation point
keeps a safe distance from every obstacle: a CBF safety filter. The changes to {doc}`math_spec`
are tagged [Design] in §6.7 (new), §8.1, §10.4, §11, §12.5 (O58) and §13. The contour, the
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
real footprint, so a predicted contact brakes through the cap (§10.3). A car cannot move along an
arbitrary $F^\star$: until 0.3.0 Ackermann was rejected; it now filters with a turning margin
({ref}`cbf-ackermann` below).

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
`cbf_gain_discrete` ($\gamma_b\Delta t \ge 1$); for cars `ackermann_cbf_window` and
`ackermann_cbf_margin` (0.3.0, below).
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

(cbf-ackermann)=

## Ackermann [Design, 0.3.0]

Heading tracking at the rear axle $p_r$ ({doc}`math_spec` §7.1) tracks the direction of $F^\star$
with $\lvert\omega\rvert \le \lvert v\rvert/R_{\min}$, the speed band $v_{\mathrm{ack}}$ and no
rotation in place. The changes to {doc}`math_spec` are tagged [Design, 0.3.0] in §6.7, §11 and
§12.5 (O59).

**The issue.** The field filter turns $F^\star$ only once $-\gamma_b h > n^\top F$, i.e. from
$h \approx v_r/\gamma_b$ (0.2 m at TC) head-on. A car needs up to $R_{\min}$ of approach to turn
from head-on to parallel to a face, and slowing down does not help: $\omega \le v/R_{\min}$ bounds
the curvature of the path, whatever the speed. So $h$ is violated during the turn. Measured
without a correction (O59a, a 0.15 m disc 0.2 m off the path): $\min h = -0.115$ m at
$R_{\min} = 0.5$ and $-0.240$ m at 1.0. Inside the margin, the push-out $-\gamma_b h\,n$ can also
point $F^\star$ behind the car, which heading tracking reports as kHeadingInfeasible (STOPPED): a
car cannot move sideways.

**Options.** (a) A curvature-aware barrier: a turning margin on $\delta_{\mathrm{safe}}$.
(b) A lookahead or higher-order barrier at a point ahead of the rear axle. (c) A speed-only filter:
keep tracking $F$ and lower $\lvert v\rvert$. (c) never steers, so it stops in front of every
obstacle that touches the swept path, offset discs included; its lower speed leaves the path
unchanged. (b) moves the disc to a point that slips sideways ($\omega\ell$), so
$\dot h \ne v\,n^\top\mathbf h$, the disc about it must still contain the footprint, and $\ell$ is
one more tuning knob. (a) is one scalar per site in the existing QP.

**Chosen: (a), the turning margin.** The rear axle does not slip: $\dot p_r = v\mathbf h$. For
site $j$ and the drive direction $q\mathbf h$, let $c_j = -n_j^\top q\mathbf h = \cos\beta_j$
($c_j > 0$: the axle approaches the site). Turning away at full lock until parallel to a face
covers
$$
\mu_j = R_{\min}\bigl(1 - \sqrt{1 - c_j^2}\bigr) = R_{\min}(1 - \sin\beta_j) \quad (c_j > 0;\ 0 \text{ otherwise})
$$
toward it. The filter constrains $\tilde h_j = \max(0,\ h_j - \mu_j)$ in place of $h_j$:
$n_j^\top F' \ge -\gamma_b\tilde h_j$. For a face [Derived]: along a full-lock turn away,
$\mathrm dh/\mathrm ds = -\cos\beta$ and $\mathrm d\mu/\mathrm ds = -R_{\min}\cos\beta\,
\mathrm d\beta/\mathrm ds = -\cos\beta$. So $h - \mu$ is constant, and $h - \mu \ge 0$ is exactly
the set from which a full-lock turn keeps $h \ge 0$. Once parallel, $\mu = 0$. Any lesser
curvature $\kappa$ lowers $h - \mu$ at the rate $v\cos\beta\,(1 - R_{\min}\kappa)$. For a site (a
point), the arc curves away from it, so the margin is conservative. As $\tilde h \to 0$,
$F^\star$ turns tangent, the heading error grows, and the law saturates at
$\lvert v\rvert/R_{\min}$, i.e. full lock. So the car turns as hard as it can exactly when it
must. The lag of $k_\psi e$ and of the angular acceleration bound costs millimetres (O59a:
$\min h = -6$ mm). Head-on, $\mu = R_{\min}$: the filter acts about $R_{\min}$ earlier. The clamp at
0 drops the push-out ($-\gamma_b\tilde h_j \le 0$). $F' = 0$ is then always feasible, the QP is
never relaxed, and a car inside the margin drives tangentially or holds; it is never pushed out
behind itself. $\mu_j$ depends on the heading, not on $x$. So the probes of $\dot\psi_d$ use the
base heading, and the decision field re-filters with the step's margins. Diff, skid and omni are
bit-identical ($\mu = 0$, no clamp). $h_j$, $\min h$, the hold, the markers and the rollout keep
$h_j = D_j - \delta_{\mathrm{safe}}$, and `CbfSample::turn_margin` carries $\mu_j$.

**The disc.** $r_e$ about $p_r$ covers the body ahead of the axle: 0.585 m for the O57 car
(0.7 × 0.4 m, the axle 0.15 m from the rear bumper), $\delta_{\mathrm{safe}} = 0.635$ m. The car
turns about $p_r$, and the disc holds at every heading. A wall beside the path is kept
$r_e - w_{\mathrm{lat}} = 0.385$ m farther from the body than needed. That is the price of
exactness at the point that does not slip. The turning radius enters through $\mu_j$, not
$\delta_{\mathrm{safe}}$. A margin of $R_{\min}$ in $\delta_{\mathrm{safe}}$ would also hold the
car $R_{\min}$ away from walls beside the path, which it can follow.

**$v_{\mathrm{ack}}$, the hold, kBlocked.** The band raises FOLLOW speeds in $(0, v_{\mathrm{ack}})$
to $v_{\mathrm{ack}}$. Near a hold, $\lVert F^\star\rVert$ falls below $v_{\mathrm{ack}}$ before
the hold ratio 0.05. The car then closes in at $v_{\mathrm{ack}}$ over a few millimetres of
$\tilde h$ (the turn is per distance, so its geometry is unchanged). Then the hold sets kBlocked,
and the car stops from $v_{\mathrm{ack}}$ within $v_{\mathrm{ack}}^2/(2a_{\mathrm{dec}})$ (1.25 mm
at the defaults).

**A lagging heading.** A real car's steering and drive lag behind the commands, so beside an
obstacle its heading can trail the filtered field by more than 90° (the MuJoCo car did, about
1 m before a box 0.1 m off its path). The heading-tracking law commands $v = \lVert F\rVert\cos e$,
which is 0 there, and a car at rest cannot turn: with kHeadingInfeasible the plugin raised
NoValidControl and Nav2 aborted the goal. While the filter changes the field the law therefore
turns as in a contour episode: the speed factor is $\max(\cos e, \tfrac12)$ and the heading error
does not set kHeadingInfeasible, so the car keeps moving and turns at $R_{\min}$ toward
$F^\star$. The collision cap and the hold still stop it where that turn would collide (pointed at
the box from close up it only creeps under the cap). Once the filter no longer binds, a heading
more than 90° from the path's field is kHeadingInfeasible again. The hold rule is unchanged: where the filter stops a car, kBlocked follows, the
plugin raises NoValidControl and Nav2 replans. Head-on, the car stops where $\tilde h = 0$:
$R_{\min} + \delta_{\mathrm{safe}}$ from the face at the axle. There it can still turn away at
full lock, which leaves a Hybrid-A* replan room. A gap narrower than $2\delta_{\mathrm{safe}}$
holds too, before it.

**Reversing.** In a reverse segment $q = -1$: the axle leads, $c_j$ uses $-\mathbf h$, and
$R_{\min}$ and $\delta_{\mathrm{safe}}$ are those of the same point. The filter keeps no state.

**Validation.** `ackermann_cbf_unsupported` is removed. Warnings:
`ackermann_cbf_window` (with a costmap) when $x_r + \delta_{\mathrm{safe}} + R_{\min} + v_r/\gamma_b
> \tfrac12\min(W, H_c)$. A face straight ahead then enters the window inside the turning margin,
and the car holds where it could have turned. `ackermann_cbf_margin` when
$m < v_{\mathrm{ack}}^2/(2a_{\mathrm{dec}})$: the stop from the band speed at the hold can use
more than the margin. The shipped `ackermann.yaml` validates with cbf in the stock 3 m window
without warnings ($0 + 0.27 + 0.5 + 0.2 = 0.97$ m).

**Limitations.** The margin is a face's, conservative for points. The disc about the axle is wide
beside the car. A car stops $R_{\min} + \delta_{\mathrm{safe}}$ short of a face across its path,
and holds in front of gaps narrower than $2\delta_{\mathrm{safe}}$. Like heading tracking, the
barrier is not exact under the law's lag and the limiter. The rollout stays the backstop.

**Tests.** O59 (core, the O57 car at $R_{\min}$ 0.5 and 1 m, $m = 0.05$, $\gamma_b = 2$, $h$ by
brute force at the rear axle). (a) A 0.15 m disc 0.2, 0.35 and 0.5 m off the path: passed and
settled, never kBlocked or kHeadingInfeasible, $\lvert\omega\rvert \le \lvert v\rvert/R_{\min}$,
no collision, $\min h \ge -6$ mm (0.5) and $\ge +4.5$ mm (1.0), footprint clearance $\ge 0.35$ m,
$\lvert D\rvert < 0.02$ at the goal. (b) A wall 0.035 m inside $\delta_{\mathrm{safe}}$ of the
path, the car starting 0.3 m off on the other side: it levels off at $h = 0$ and follows at
$v_r$. (c) A box across the path: hold and kBlocked for 100 cycles, at rest, $h \ge 0$, stopped
0.57 m (0.5) and 1.07 m (1.0) short. (d) Two discs whose sites are $\delta_{\mathrm{safe}} - 0.05$
off the path (0.39 m of free space beside the body): hold, kBlocked, $h \ge 0$. (e) A disc 0.3 m
beside the reverse leg of a cusp plan: passed backwards with $\min h \ge -3$ mm. (f) One step:
$\mu = R_{\min}(1 - \sin\beta)$, 0 when parallel, receding or not a car. (g) A lagging heading:
the car 0.85 or 1.25 m before a box beside the path, heading 0.6 to 1.2 rad toward its side:
never kHeadingInfeasible, no collision, settled (six of its nine starts stopped with
kHeadingInfeasible before the turn rule). Layer 2: a box 0.12 m
beside an Ackermann plan passes with clearance $\ge m - \mathrm{res}$. System test (Ackermann,
kinematic): an unmapped box 0.1 m beside the path succeeds with clearance $\ge m - \mathrm{res}$
and no infeasible command at the simulator.
