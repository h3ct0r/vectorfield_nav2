(math-spec)=

# Mathematical specification

This is the authoritative mathematical specification of `vectorfield_core`. Milestone M1
implements it and tests it against the oracles in {ref}`sec-math-oracles`. When code and this
document disagree, one of them is wrong: settle the document first, then the code.
{doc}`../theory` is the readable summary; this page holds every symbol, guard and tolerance.

Results that are not plain definitions carry a tag:

| Tag | Meaning |
|---|---|
| **[Thesis n.m]** | Equation or result *n.m* of the Rezende thesis {cite:p}`rezende2022thesis`. |
| **[Derived]** | Derived in this document; the derivation is given. |
| **[Design]** | A deliberate change to the thesis method, with its rationale. |
| **[Spec]** | A detail the approved plan leaves open and this document fixes. {ref}`sec-math-spec-choices` lists them all for review. |

(sec-math-scope)=

## 0 Scope and sources

**Scope.** Planar (2D) path and curve following for differential, skid-steer, omnidirectional and
Ackermann robots: curve geometry, gains, the static and time-varying field, the terminal field,
the obstacle contour, tracking laws, speed shaping and limiting, phases, collision prediction,
configuration validation and the test oracles. ROS behaviour appears only where it defines a
core input. The 3D core is milestone M7. Units are SI; angles are in radians.

**Sources.**

| Source | Use |
|---|---|
| A. M. C. Rezende, *Vector Field Based Guidance and Control Strategies for Robot Navigation*, PhD thesis, Universidade Federal de Minas Gerais, 2022. Open access: <https://repositorio.ufmg.br/bitstream/1843/73611/3/Tese_AdrianoRezende_ABNT_PDFA.pdf> | Primary source. **All equation numbers in this document are thesis numbers.** Thesis p. 54 states that its §3.1–3.2 were published as the T-RO paper. Some PDF pages are raster images (pp. 70, 78, 80, 82, 85, 95–117, 137); content taken from them, including the obstacle method of §3.4 and the experimental values, was read by OCR. |
| A. M. C. Rezende, V. M. Gonçalves, L. C. A. Pimenta, "Constructive Time-Varying Vector Fields for Robot Navigation", IEEE T-RO 38(2):852–867, 2022, doi:10.1109/TRO.2021.3093674 {cite:p}`rezende2022constructive` | Closed access. **T-RO equation numbers are UNVERIFIED** and are not used anywhere in this project until they are checked against the published paper. The paper writes the field as $\Psi$, the thesis as $\Phi$; this document follows the thesis. |
| Approved implementation plan (revision 6) and its five design-review rounds | Source of every [Design] item. These records are not part of the repository; each rationale is restated here so this document stands alone. |
| `adrianomcr/vectorfield_stack` (ROS 1) | Read for comparison only. No code is reused. Defects are in `docs/design/upstream_audit.md`. |
| Nav2 (Humble, Jazzy, Lyrical) | Only where it defines inputs: goal checkers, path handlers, NavFn plan tails. See `docs/design/nav2_api_matrix.md`. |

**Conventions.**

- $J = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}$ is the rotation by $+\pi/2$;
  $R(\psi) = \cos\psi\, I + \sin\psi\, J$; $\mathbf h(\psi) = (\cos\psi, \sin\psi)^\top$.
- $a \times b = a_x b_y - a_y b_x$; $\operatorname{unit}(v) = v / \lVert v \rVert$.
- $\operatorname{wrap}(a) \in (-\pi, \pi]$; $\operatorname{clamp}(a, l, u) = \min(u, \max(l, a))$;
  $\operatorname{sgn}(0) = 0$.
- $(p, \psi)$ is the pose of `base_link` in the evaluation frame (plugin: the costmap global
  frame; follower: `global_frame`). "At base_link" means evaluated at $p$.
- All vectors are column vectors in the evaluation frame unless a frame is named.

(sec-math-symbols)=

## 1 Symbols

Default values are the plan's parameter defaults. The block of rows from $w_{\mathrm{lat}}$ to
$x_{\mathrm{ref}}$ repeats the plan's symbol table verbatim.

| Symbol | Meaning | Parameter, default |
|---|---|---|
| $x$ | evaluation point of the active law ({ref}`sec-laws-points`) | — |
| $p,\ \psi,\ \mathbf h$ | `base_link` position, yaw, heading unit vector | — |
| $r(s,t)$, $r(u,t)$ | curve point at arc length $s$ or parameter $u$, time $t$ | — |
| $s^\star,\ x^\star$ | closest-point arc length and point [Thesis 3.1, 3.2] | — |
| $\mathbf D = x - x^\star$, $D = \lVert \mathbf D \rVert$ | distance vector and distance [Thesis 3.3, 3.4] | — |
| $n$ | unit normal $\mathbf D / D$; $n = 0$ when $D < \varepsilon_D$ | — |
| $T$ | unit tangent at $s^\star$, in the curve's parameter direction [Thesis 3.5, 3.6] | — |
| $\mathrm{dir},\ T_d = \mathrm{dir}\,T$ | traversal direction $\pm 1$ and field tangent | `reverse_direction` (follower goal; plugin $+1$) |
| $\kappa,\ N,\ \Omega$ | curvature, principal normal, $\Omega = 1 - \kappa N^\top \mathbf D$ [Thesis 3.7, 3.8, 3.16] | — |
| $\Pi_T = I - T T^\top$ | projector onto the normal space [Thesis 3.9] | — |
| $G,\ H$ | gain functions ({ref}`sec-gains`) | `gain_function` atan |
| $k_f$ | convergence gain, 1/m | `convergence_gain` 2.0 |
| $v_r$ | reference speed | `desired_speed` 0.4 |
| $\Phi_S,\ \Phi_T,\ \eta,\ \Phi$ | unit field, feedforward, scale, path field ({ref}`sec-distance-field`) | — |
| $v_c$ | curve-point velocity $\partial r / \partial t$ at $s^\star$ | — |
| $\varepsilon_v$ | Assumption-3 margin | `assumption3_margin` 0.1 (follower) |
| $F$ | field the law tracks, after speed shaping and the contour | — |
| $\sigma,\ s_{\mathrm{goal}},\ g_c$ | remaining arc length, goal station, $g_c = r(s_{\mathrm{goal}})$ ({ref}`sec-terminal-field`) | — |
| $g$ | goal position given to the core (plugin: `global_goal`; follower: $r(s_{\mathrm{goal}})$) | — |
| $\sigma_b$ | terminal braking distance | `terminal_braking_distance` 0.5 |
| $\sigma_{\mathrm{stop}}$ | settle radius, $\mathrm{xy\_tol}/2$ (follower: `goal_tolerance`/2) | — |
| $\varphi$ | terminal speed profile | — |
| $d$ | control-point offset | `control_point_offset` 0.2 |
| $q$ | drive sign, $-1$ iff `drive_backward` | `drive_backward` false |
| $k_\psi$ | heading gain | `heading_gain` 2.0 |
| $\gamma$ | speed-shaping scale (the plan's $s$ in §3.4) | — |
| $\gamma_{\mathrm{lim}}$ | external speed-limit scale | `speed_limit_scale` input, $(0, 1]$ |
| $c$ | uniform kinematic-box scale | — |
| $v_{\max},\ v_{y,\max},\ \omega_{\max}$ | velocity limits | `max_linear_vel` 0.5, `max_lateral_vel` 0.5, `max_angular_vel` 1.5 |
| $\bar\omega = \omega_{\max}\,\gamma_{\mathrm{lim}}$ | effective angular limit | — |
| $a_{\mathrm{acc}},\ a_{\mathrm{dec}},\ \alpha_{\max}$ | acceleration limits | `max_linear_accel` 1.0, `max_linear_decel` 1.0, `max_angular_accel` 3.2 |
| $a_{\mathrm{can}}$ | cancel deceleration | `cancel_deceleration` 1.0 |
| $R_{\min}$ | minimum turning radius (Ackermann) | `min_turning_radius` 0 |
| $\lambda,\ D_{\mathrm{in}},\ D_{\mathrm{in}}'$ | contour distance, full-switch distance, blend-start distance | `obstacle_contour_distance`, `obstacle_full_switch_distance`, `obstacle_blend_start_distance` (auto) |
| $x_o^\star,\ D_o,\ n_o$ | nearest obstacle point, obstacle distance, obstacle normal ({ref}`sec-edt`) | — |
| $\theta$ | blend weight [Thesis 3.71] | — |
| $K,\ \sigma_c$ | blocking obstacle component, latched circulation side | — |
| $\mathrm{res}$ | costmap resolution | — |
| $m_s$ | stop margin | `stop_margin` 0.10 |
| $T_h$ | rollout horizon | `collision_check_horizon` 1.5 s |
| $s_{\mathrm{free}},\ v_{\mathrm{cap}}$ | free arc length and braking cap ({ref}`sec-collision`) | — |
| $w$ | follower search window | `search_window` 1.0 |
| $D_{\mathrm{trig}},\ h_g,\ M$ | global-search trigger, adoption hysteresis, sample count | `global_search_trigger` 0.5, `global_search_hysteresis` 0.10, `global_search_samples` 400 |
| $\ell_c$ | chord length $\max(0.10\ \mathrm{m},\ 2\,\mathrm{res})$ for cusp decimation and tail conditioning | — |
| $c_G$ | integral-curve curvature constant: 0.186 (atan), 0.385 (rational) ({ref}`sec-gains`) | — |
| $w_{\mathrm{lat}}$ | max $\lvert y \rvert$ over the robot-frame footprint (robot_radius if circular); computed with $r_{\mathrm{insc}}$ and $r_{\mathrm{circ}}$ in FootprintChecker | — |
| $v_{\min}$ | `min_approach_speed` | 0.05 |
| $v_{\mathrm{ack}}$ | `ackermann_min_speed` | 0.05 |
| $\varepsilon_v$ | `assumption3_margin` | 0.1 |
| $\varepsilon_{\mathrm{hp}}$ | $10^{-3}$ on $\lVert t_1 + t_2 \rVert$ (hairpin) | — |
| $\varepsilon_D$ | $10^{-9}$ m | — |
| $h_c$ | 0.05 m (contour-exit hysteresis) | — |
| $f$ | controller_frequency: the parent's in the plugin, read at configure; the follower's own. $\Delta t$ is the measured time since the previous step. | — |
| $\Delta u_b = \Delta u_f$ | $w/2$, with $w$ = `search_window` | — |
| $\kappa_{\mathrm{path}}$ | max over window vertices of turning angle / mean length of the adjacent segments, after tail conditioning | — |
| $v_{\mathrm{stop}}$ | 0.02 m/s (Ackermann: $v_{\mathrm{ack}}$) | — |
| $h_b$ | $\max(2\,\mathrm{res},\ 0.05\ \mathrm{m})$: kBlocked release band | — |
| $x_{\mathrm{ref}}$ | body-x offset of the law's reference point from base_link: $d$ for FL in FOLLOW; reference_point_offset (signed, negative = behind) for Ackermann; 0 otherwise | `reference_point_offset` 0 |

FL abbreviates the `feedback_linearization` tracking law, HT the `heading_tracking` law.

(sec-geometry)=

## 2 Curve geometry

(sec-geom-defs)=

### 2.1 Closest point and its properties

For a curve $r(s, t)$ with arc length $s$ [Thesis 3.1–3.9]:

$$
s^\star(x,t) = \arg\min_s \lVert x - r(s,t) \rVert^2, \qquad
x^\star = r(s^\star, t), \qquad
\mathbf D = x - x^\star, \qquad D = \lVert \mathbf D \rVert,
$$

$$
T(s,t) = \frac{\partial r}{\partial s}, \qquad
\kappa N = \frac{\partial T}{\partial s}, \qquad
\Pi_T = I - T T^\top .
$$

The field is defined on the set $U$ where $s^\star$ is unique. The results this specification
relies on are:

- **Lemma 2** [Thesis 3.13]: $T^\top \mathbf D = 0$ at the minimiser of a curve without
  endpoints.
- **Lemma 3** [Thesis 3.15–3.16]: $\partial s^\star / \partial x = T^\top / \Omega$ with
  $\Omega = 1 - \kappa N^\top \mathbf D$; $\Omega \to 0$ at centres of curvature.
- **Lemma 5** [Thesis 3.19]: $\nabla D = n$ for $x \notin \mathcal C(t)$.
- **Lemma 6** [Thesis 3.20–3.23]: with $P = \tfrac12 D^2$, $\nabla P = \mathbf D$ and
  $\partial P / \partial t = \mathbf D^\top \Pi_T\, \partial \mathbf D / \partial t$.

The thesis assumptions are:

- **Assumption 1**: each $\mathcal C(t)$ is unbounded and homeomorphic to a line, or bounded and
  homeomorphic to a circle. Finite curves with endpoints are excluded, because Lemma 2 needs a
  curve without borders.
- **Assumption 2**: $r$ is $C^2$ in $s$ and differentiable in $t$.
- **Assumption 3** [Thesis 3.10]: $v_m = \max_U \lVert \Pi_T\, \partial \mathbf D / \partial t \rVert < v_r$.
- **Assumption 4**: trajectories that start in $U$ stay in $U$.

**[Design] Borderless open curves.** Every open curve is extended at both ends by a
semi-infinite straight ray along its end tangent. Projections onto a ray are clamped only at the
ray's anchor ({ref}`sec-polyline`). Every open curve the core evaluates is therefore unbounded and
borderless, Lemma 2 holds at every closest point, and no "open endpoint" tangent rule is needed.
The rays also make $\sigma < 0$ well defined after an overshoot ({ref}`sec-terminal-field`).

Self-intersecting closed curves (lemniscates) violate Assumption 1. They are supported through
the hinted local search of {ref}`sec-search`, without a proof ({ref}`sec-math-limits`).

(sec-polyline)=

### 2.2 Polylines: pieces, projection and arc length

A polyline has vertices $P_0, \dots, P_{N-1}$ with $N \ge 2$ after dedupe, and a `closed` flag.
For a closed polyline $P_N \equiv P_0$ and a closing duplicate is removed by dedupe.

- Segment $k$ ($k = 0, \dots, N-2$; closed: also $k = N-1$): length
  $\ell_k = \lVert P_{k+1} - P_k \rVert$, tangent $t_k = (P_{k+1} - P_k)/\ell_k$, station
  $S_0 = 0$, $S_{k+1} = S_k + \ell_k$. Closed length $L = S_N$; open length $L = S_{N-1}$.
- Pieces and their closed parameter intervals:

| Piece | Point | Interval of $\tau$ | Arc length $s$ |
|---|---|---|---|
| segment $k$ | $P_k + \tau t_k$ | $[0, \ell_k]$ | $S_k + \tau$ |
| backward ray (open only) | $P_0 + \tau t_0$ | $(-\infty, 0]$ | $\tau$ (negative) |
| forward ray (open only) | $P_{N-1} + \tau t_{N-2}$ | $[0, \infty)$ | $S_{N-1} + \tau$ |

- **Projection onto a piece** with anchor $A$ and tangent $t$: $\hat\tau = (x - A)^\top t$,
  $\tau = \operatorname{clamp}(\hat\tau, \text{interval})$, foot $A + \tau t$, distance
  $\lVert x - A - \tau t \rVert$. A ray is clamped only at its anchor, so a point far beyond the
  goal still projects onto the forward ray.
- Arc length is measured from $P_0$ and is negative on the backward ray. For closed polylines
  $s^\star \in [0, L)$ and every arc-length difference is taken modulo $L$ into
  $(-L/2, L/2]$.

(sec-wedge)=

### 2.3 Ownership and tangent

**Ownership** [Design]. A segment or ray owns $x$ when its unclamped $\hat\tau$ lies in its closed
interval. An interior vertex $P_v$ (open: $v = 1, \dots, N-2$; closed: every vertex), between
incoming tangent $t_{v-1}$ and outgoing tangent $t_v$, owns $x$ iff

$$
a_1 = (x - P_v)^\top t_{v-1} > 0 \quad\text{and}\quad a_2 = -(x - P_v)^\top t_v > 0 ,
$$

that is, $x$ lies strictly beyond both adjacent pieces. Collinear junctions, including the two
ray joints, can never satisfy both inequalities, so they never form a wedge. Every point has at
least one owner, and the closest point over a set of pieces lies on an owner.

**Tangent on a segment or ray**: $T = t_k$ (rays: $t_0$ or $t_{N-2}$).

**Wedge tangent** [Design]. When vertex $P_v$ owns $x$ and $D \ge \varepsilon_D$:

$$
T = s_w\, J n, \qquad s_w = \operatorname{sgn}\!\big((J n)^\top (t_{v-1} + t_v)\big).
$$

*Derivation* [Derived]. The continuous blend used by the research,
$\operatorname{unit}\big((I - n n^\top)((1 - \alpha) t_{v-1} + \alpha t_v)\big)$ with
$\alpha = a_1/(a_1 + a_2)$, keeps $n^\top T = 0$. In 2D, writing $y = x - P_v = D n$,
$(J n)^\top [(1-\alpha) t_{v-1} + \alpha t_v] = D\,(t_{v-1} \times t_v)/(a_1 + a_2)$, whose sign is
constant in the wedge; the normalised blend is therefore exactly
$\operatorname{sgn}(t_{v-1} \times t_v)\, J n$. Inside the strict wedge
$\operatorname{sgn}((J n)^\top (t_{v-1} + t_v)) = \operatorname{sgn}(t_{v-1} \times t_v)$: for a
left turn by $\beta \in (0, \pi)$ with $t_{v-1} = (1, 0)$, the wedge conditions $y_x > 0$ and
$y_y < -y_x \cot\beta$ give $(J y)^\top (t_{v-1} + t_v) > y_x (1 + \cos\beta)/\sin\beta > 0$, and
right turns mirror this. The chosen form has magnitude $O(1)$ instead of $O(\beta)$, so it is
numerically robust at shallow turns. On the wedge boundaries ($a_1 = 0$ or $a_2 = 0$) it equals
$t_{v-1}$ or $t_v$, so $T$ is continuous and $n^\top T = 0$ holds exactly everywhere. Both facts
were checked numerically ({ref}`sec-math-constants`).

**Hairpin** [Design]. If $\lVert t_{v-1} + t_v \rVert < \varepsilon_{\mathrm{hp}}$ (an exact
reversal), $s_w$ is undefined. Then $T = \operatorname{unit}((I - n n^\top) T_{\mathrm{last}})$,
where $T_{\mathrm{last}}$ is `TrackerState::last_tangent` (stored in the curve's parameter
direction); if that norm is below $\varepsilon_{\mathrm{hp}}$, $T = +J n$. STATUS_HAIRPIN is set.
Reusing $T_{\mathrm{last}}$ without the projection would break $n^\top T = 0$, and with it
$\lVert \Phi_S \rVert = 1$, the $\eta$ formula and $\dot D \le 0$.

**On the curve** [Design]. If $D < \varepsilon_D$: $n := 0$ and $T :=$ the owning piece's tangent.
At a vertex point ($x = P_v$, owned by both adjacent pieces),
$T := \operatorname{unit}(t_{v-1} + t_v)$, or $t_v$ for a hairpin. Since $G(0) = 0$ and
$H(0) = 1$, the field there is $v_r T_d$. This case is constructed, not accidental: the one-pose
polyline $[p, g]$ is rebuilt from the base every cycle and evaluated at the base ({ref}`sec-conditioning`).

**Inner corners** [Derived]. On the concave side of a corner with turning angle $\theta$, the
closest point jumps across the inner bisector. A point at distance $D$ from both segments has
feet whose arc-length difference is $2D \tan(\theta/2)$. The field is discontinuous there but
crosses the bisector transversally with $\dot D < 0$, so nothing sticks.

(sec-search)=

### 2.4 Closest-point search

Each policy returns $(x^\star, s^\star, T, n, D)$, the owning piece or vertex, and flags.

**Candidate rule.** Within a set of pieces, the closest point is the owner with the smallest $D$.
Candidates whose $D$ differ by at most $10^{-9}$ m are ties [Spec]:

- with a hint, the tie goes to the candidate nearest the hint (plugin: foot nearest the hint
  position $x^\star_{\mathrm{prev}}$; follower: smallest wrapped $\lvert s - s_{\mathrm{prev}} \rvert$);
- without a hint (initialisation), the tie goes to the smallest $\mathrm{dir}\cdot s$, i.e. the
  start of traversal. This resolves the out-and-back open polyline (last point equal to the
  first) to its start instead of its end.

**WindowScan** (plugin; exact; no global stage). The candidate pieces are every piece whose
arc-length interval meets $[-(d + 0.3),\ d + 0.5]$ m, always including the backward ray. If the
winning foot lies at the far end of the last included piece, the next piece is added and the scan
repeats, up to the forward ray; the same applies backwards. A clamped edge point is never
returned, because it is not a projection and would break $n^\top T = 0$. The plugin hint is the
position $x^\star_{\mathrm{prev}}$ only; the window is re-pruned every cycle, so arc lengths and
indices are never carried across cycles.

**Hinted** (follower polylines). Candidates are the pieces meeting
$[s_{\mathrm{prev}} - w/2,\ s_{\mathrm{prev}} + w/2]$ (wrapping only when closed), extended at an
edge minimum exactly as in WindowScan. The global stage is an exact scan of every piece. It runs at
initialisation (result adopted unconditionally) and when $D_{\mathrm{loc}} > D_{\mathrm{trig}}$.

**ParametricCurve** ({ref}`sec-parametric`). Golden-section search on
$[u_{\mathrm{prev}} - \Delta u_b,\ u_{\mathrm{prev}} + \Delta u_f]$, wrapping only when closed. If
the result lies on a bracket edge that is not a domain end, the bracket is moved by $w/2$ in that
direction and the search repeats, at most one lap [Spec]. The global M-sample stage (thesis §5.1)
runs at initialisation, when the local result was on an edge, and when
$D_{\mathrm{loc}} > D_{\mathrm{trig}}$.

**Adoption rule** [Design] (Hinted and ParametricCurve). A global candidate replaces the local
one only if

$$
D_{\mathrm{glob}} < D_{\mathrm{loc}} - h_g
\quad\text{and}\quad
\operatorname{arc}(s_{\mathrm{loc}}, s_{\mathrm{glob}}) \le 3\,\lVert x^\star_{\mathrm{loc}} - x^\star_{\mathrm{glob}} \rVert + 2\,(D_{\mathrm{loc}} + D_{\mathrm{glob}}),
$$

where $\operatorname{arc}$ is the arc length between the feet (closed curves: the shorter way).
An adopted jump sets STATUS_AMBIGUOUS_CLOSEST for that cycle [Spec].

*Rationale.* A per-cycle global argmin jumps branches near a lemniscate crossing: a robot with a
small lateral error is closer to the other branch inside a cone around the crossing, and the
tangent swings by the crossing angle (106° for the Gerono built-in with $a = 4$, $b = 1.5$). The
detour test accepts inner-corner jumps (arc $2D\tan(\theta/2)$ against chord
$2D\sin(\theta/2)$; accepted while $\tan(\theta/2) \le 3\sin(\theta/2) + 2$, i.e. for turns up to
157.1° [Derived]) and rejects branch jumps, whose arc is a whole lobe. Jumps within the local
window need no adoption at all: the local candidate set already contains both pieces. For U-turn legs $w_u$
apart with a semicircular end, feet at the same station $x$ before the apex give
$\operatorname{arc} = 2x + \pi w_u/2$, chord $w_u$ and $D_{\mathrm{loc}} + D_{\mathrm{glob}} = w_u$,
so the jump is rejected only for $x > (5 - \pi/2)\, w_u/2 \approx 1.71\, w_u$ [Derived]. Nearer
the apex the configuration is geometrically an inner corner, no arc/chord test can separate the
two, and the jump is accepted. The plugin has no global stage: the server prunes the window to the
robot, which already anchors the search.

(sec-parametric)=

### 2.5 Parametric curves

Expression and built-in curves are $r(u, t)$ on a domain $[u_i, u_f]$ (`s_min`, `s_max`; built-ins
$[0, 2\pi)$). The parameter is not arc length.

- **Derivatives** [Spec]. Central differences with $h_u = 6\cdot 10^{-6}\, (u_f - u_i)$; at the
  ends of an open domain, the second-order one-sided formula
  $(-3 r_0 + 4 r_1 - r_2)/(2 h_u)$, so the expression is never evaluated outside $[u_i, u_f]$.
  Closed domains wrap. Built-ins may use analytic derivatives.
- **Tangent**: $T = \operatorname{unit}(\partial r / \partial u)$ at $u^\star$.
- **Normal** [Spec]: $n := \operatorname{unit}((I - T T^\top)(x - x^\star))$ and
  $D := \lVert x - x^\star \rVert$; if $\lVert (I - T T^\top)(x - x^\star) \rVert < \varepsilon_D$
  the on-curve rule applies. The search is accurate only to its tolerance, and this
  re-orthogonalisation keeps $n^\top T = 0$ to rounding, so $\lVert \Phi_S \rVert = 1$ and the
  $\eta$ formula stay exact.
- **Golden section** [Spec]: stop when the bracket width is at most
  $10^{-10} \max(1, u_f - u_i)$, or after 100 iterations; then polish with at most 8 secant steps on
  $(r - x)^\top r' = 0$ within $\pm 10^{-5}(u_f - u_i)$ [Spec, M1]: golden section alone locates the
  foot only to about $10^{-8}$ m at $D = 1$ m, short of O7's $10^{-9}$.
- **Global stage** (thesis §5.1): $M$ samples equally spaced in $u$ (closed: $[u_i, u_f)$; open:
  inclusive) plus the two rays; every local minimum of the sampled distance within about half a
  sample spacing of the best (at most 4) is refined by golden section on
  $[u_{k - 1}, u_{k + 1}]$ (clamped to the domain when open), and the tie rule picks among the refined
  feet [Spec, M1]. Refining only $\arg\min$ fails at self-intersections, where samples on both
  branches sit on the crossing and tie. Static curves cache the sample positions at $t_0$.
- **Adoption ties** [Spec, M1]: a global candidate within $10^{-9}$ m of the local one is not adopted
  (the tie goes to the hint).
- **Regularity slack** [Spec, M1]: adjacent tangents exactly $90°$ apart pass (with $10^{-9}$ rad
  slack), so a circle sampled at the minimum $M = 4$ is accepted.
- **Expression time** [Spec, M1]: an expression's `t` is curve time minus `CurveSpec::t0`; regularity is
  checked at expression time 0 and $\partial_t r$ is differenced around $t - t_0$.
- **Arc length, static curves**: a $u \leftrightarrow s$ table is built once at construction, with
  $s_k$ at the $M + 1$ samples by 5-point Gauss–Legendre per interval, and
  $s(u) = s_k + \int_{u_k}^{u} \lVert \partial r / \partial u \rVert\, du$ by the same rule [Spec].
  The windows use $\Delta u_{b,f} = \lvert u(s_{\mathrm{prev}} \pm w/2) - u_{\mathrm{prev}} \rvert$.
- **Arc length, time-varying curves**: no table. Windows use
  $\Delta u = w / (2 \lVert \partial r / \partial u(u_{\mathrm{prev}}, t) \rVert)$. Arc length,
  needed only by the detour test and follower feedback, is computed on demand by fixed-panel
  Gauss–Legendre (5 points, $\lceil 64\, \lvert \Delta u \rvert / (u_f - u_i) \rceil$ panels) with
  no allocation [Spec].
- **Rays** (open curves): $r(u_f) + \tau\, T(u_f)$, $\tau \ge 0$, and $r(u_i) + \tau\, T(u_i)$,
  $\tau \le 0$, with one-sided end tangents.
- **Runtime floor**: $\lVert \partial r / \partial u \rVert$ below the construction-time floor
  ({ref}`sec-regularity`) sets kNonFinite.

(sec-regularity)=

### 2.6 Regularity and closure

`makeCurve` is the single validation point. On the $M$ global-stage samples at $t_0$ it rejects
with INVALID_CURVE when [Design]:

1. $\min_k \lVert \partial r / \partial u (u_k) \rVert < 10^{-3} \cdot \operatorname{mean}_k \lVert \partial r / \partial u (u_k) \rVert$;
2. adjacent sample tangents differ by more than 90° (a stationary point or cusp between
   samples, which the speed floor can miss);
3. an expression marked closed does not close: $\lVert r(u_f) - r(u_i) \rVert > \max(10^{-9}\ \mathrm{m},\ 10^{-6} L)$, or its end tangents differ by more than 1°.

Closed polylines close implicitly. Rationale: Assumption 2 needs a regular curve; a stationary
point makes $T$ undefined and breaks $\Delta u = w/(2 \lVert \partial r/\partial u \rVert)$, and a
non-closing "closed" curve puts a jump in $s^\star$, $T$ and the field at the seam.

(sec-builtins)=

### 2.7 Built-in shapes

All built-ins are closed, with natural domain $s \in [0, 2\pi)$ when `s_min` = `s_max` = 0. Each is
written in the curve frame and mapped by $[c_x, c_y]^\top + R(\mathrm{yaw})(\cdot)$; the pose
parameters `cx`, `cy`, `yaw` default to 0. Unknown names or out-of-range parameters give
INVALID_CURVE.

| Shape | Parameters (defaults) | $r(s)$ |
|---|---|---|
| `ellipse` | `a` 1, `b` 1 | $(a \cos s,\ b \sin s)$ |
| `lemniscate_bernoulli` | `a` 1 | $a\,(\cos s,\ \sin s \cos s)/(1 + \sin^2 s)$; speed $a/\sqrt{1 + \sin^2 s} > 0$ |
| `lemniscate_gerono` | `a` 1, `b` 0.5 | $(a \cos s,\ b \sin 2s)$ |
| `superellipse` | `a` 1, `b` 1, `n` 4 ($n \ge 2$) | $\rho(s)\,(\cos s,\ \sin s)$, $\rho(s) = \big(\lvert \cos s / a \rvert^n + \lvert \sin s / b \rvert^n\big)^{-1/n}$ |
| `lissajous` | `ax` 1, `ay` 1, `fx` 1, `fy` 2, `phase` $\pi/2$ | $(a_x \sin(f_x s + \mathrm{phase}),\ a_y \sin(f_y s))$, phase on $x$; $f_x, f_y$ positive integers with $\gcd(f_x, f_y) = 1$ [Spec] |
| `rounded_rectangle` | `width` 2 (along $x$), `height` 1, `radius` 0.2, $0 < \mathrm{radius} \le \min(\mathrm{width}, \mathrm{height})/2$ | arc-length parametrisation below |
| `regular_polygon` | `radius` 1 (circumradius $R$), `sides` 6 ($n \ge 3$), `corner_radius` 0.1 ($0 < r_c \le R \cos(\pi/n)$ [Spec]) | arc-length parametrisation below |

Notes:

- **Superellipse.** The polar form has speed $\lVert r'(s) \rVert \ge \rho(s) > 0$ and is $C^2$ for
  $n \ge 2$. The common form $x = a\,\operatorname{sgn}(\cos s)\lvert \cos s \rvert^{2/n}$ has
  infinite speed at the axis points for $n > 2$ and zero speed for $n < 2$; it is not used.
  $n < 2$ is rejected (cornered shapes: `regular_polygon` or a closed POLYLINE).
- **Lissajous.** With the phase on $x$ the default is a regular figure-eight; phase $\pi/4$ gives
  $y = 2x^2 - 1$ traced back and forth with a stationary point, which the regularity check
  rejects. Non-coprime frequencies trace the curve $\gcd(f_x, f_y)$ times per period, which the
  regularity check cannot see and which breaks lap counting and Assumption 1; hence the
  coprimality rule [Spec].
- **Rounded rectangle and regular polygon.** $s$ maps linearly to arc length
  $\varsigma = s L / (2\pi)$, counter-clockwise from the midpoint of the side crossing $+x$.
  - Rounded rectangle, $W$ = width, $H_r$ = height, $\rho$ = radius:
    $L = 2(W - 2\rho) + 2(H_r - 2\rho) + 2\pi\rho$. From $(W/2, 0)$ the pieces are: up the right
    side ($H_r/2 - \rho$), quarter arc about $(W/2 - \rho,\ H_r/2 - \rho)$, top side ($W - 2\rho$),
    quarter arc about $(-W/2 + \rho,\ H_r/2 - \rho)$, left side ($H_r - 2\rho$), quarter arc about
    $(-W/2 + \rho,\ -H_r/2 + \rho)$, bottom side ($W - 2\rho$), quarter arc about
    $(W/2 - \rho,\ -H_r/2 + \rho)$, and up the right side ($H_r/2 - \rho$) back to the start.
  - Regular polygon: vertices at angles $\pi/n + 2\pi k/n$, so side $k$ has outward normal angle
    $\phi_k = 2\pi k/n$ and midpoint $a\,(\cos\phi_k, \sin\phi_k)$ with apothem
    $a = R\cos(\pi/n)$. Side length $\ell = 2R\sin(\pi/n)$; each corner is filleted inward by an
    arc of radius $r_c$, tangent to both sides at distance $t = r_c \tan(\pi/n)$ from the vertex,
    centred at $(R - r_c/\cos(\pi/n))\,(\cos(\phi_k + \pi/n), \sin(\phi_k + \pi/n))$ and spanning
    normal angles $\phi_k$ to $\phi_{k+1}$. $L = n(\ell - 2t) + 2\pi r_c$; the straight part is
    non-negative iff $r_c \le a$.
  - Both shapes are $C^1$ with curvature jumps; Assumption 2 holds piecewise, as for polylines.
- Every built-in at its defaults passes the checks of {ref}`sec-regularity` ({ref}`sec-math-constants`).

(sec-conditioning)=

### 2.8 Input conditioning

**Follower polylines** (`Curve.msg` POLYLINE). In this order [Spec on the order]:

1. Dedupe: drop consecutive vertices less than 1 mm apart (closed: also the closing duplicate).
2. Corner fillet, only when `max_corner_deviation` $\delta > 0$. At an interior vertex with turning
   angle $\theta \in (0, \pi)$ the radius is $R = \min(R_\delta, R_\ell)$ with
   $R_\delta = \delta / (1/\cos(\theta/2) - 1)$ (plan) and
   $R_\ell = \min(\ell_{v-1}, \ell_v)/(2\tan(\theta/2))$ [Spec: adjacent fillets never overlap].
   The arc is tangent at $P_v - R\tan(\theta/2)\, t_{v-1}$ and $P_v + R\tan(\theta/2)\, t_v$, centred
   at $P_v + \frac{R}{\cos(\theta/2)} \operatorname{unit}(t_v - t_{v-1})$. Its deviation from the
   vertex is $R(1/\cos(\theta/2) - 1) \le \delta$. Endpoints are preserved.
3. Densify to `sample_spacing` (or `default_sample_spacing` 0.05 m): every vertex is kept and each
   segment of length $\ell$ is split into $\lceil \ell / \mathrm{spacing} \rceil$ equal pieces, so
   endpoints and corners never move [Spec, confirmed M1]; then require $N \le$ `max_curve_points`.

**Plugin window** (per cycle, plan §5.2). Dedupe at 1 mm; no resampling or smoothing, because
neither is shift-invariant on a re-pruned window; semi-infinite backward and forward rays.

- **End tangent** $\hat T_{\mathrm{end}}$, computed once per raw plan in the plan frame: walk back
  from the last raw pose $g$ to the first pose $p$ with $\lVert g - p \rVert \ge \ell_c$ and store
  $\operatorname{unit}(g - p)$. If no pose is that far, keep the previous $\hat T_{\mathrm{end}}$
  while `goal_seq` is unchanged (near-goal replans start at the robot); otherwise use the first
  pose. A plan whose poses all coincide with $g$ has none and uses the one-pose rule.
- **Tail conditioning**, only when the window ends at the goal
  ($\lVert W_{\mathrm{back}} - \mathrm{global\_goal} \rVert < 0.05$ m): remove every window vertex
  within $\ell_c$ of $W_{\mathrm{back}}$, then insert the virtual vertex
  $W_{\mathrm{back}} - \ell_c\, R_{\mathrm{plan}\to\mathrm{costmap}}\, \hat T_{\mathrm{end}}$ before
  $W_{\mathrm{back}}$. The last segment is then at least $\ell_c$ long and fixed across cycles;
  this segment is $T_{\mathrm{end}}$. Rationale: NavFn ends plans with a sub-cell hook of up to
  $\mathrm{res}/\sqrt2$ in any direction, and the server prunes everything behind the robot, so
  near the goal the window alone cannot supply the end direction.
- **One-pose rule.** A one-pose plan, or a window with fewer than 2 vertices after dedupe, becomes
  $[p, p_{\mathrm{last}}]$; if $\lVert p - p_{\mathrm{last}} \rVert < 1$ mm it becomes
  $[p_{\mathrm{last}} - 0.01\,\mathbf h(\psi_{\mathrm{last}}),\ p_{\mathrm{last}}]$. The settled rule
  ({ref}`sec-phases`) then decides.
- **$\kappa_{\mathrm{path}}$**: over interior window vertices (rays excluded),
  $\kappa_v = \theta_v / \big((\ell_{v-1} + \ell_v)/2\big)$ with
  $\theta_v = \lvert \operatorname{wrap}(\angle t_v - \angle t_{v-1}) \rvert$;
  $\kappa_{\mathrm{path}} = \max_v \kappa_v$.

**Cusp decimation** (plugin, once per raw plan, plan frame) [Design]:

1. Walk the poses and keep a vertex only if it is at least $\ell_c$ from the last kept vertex;
   always keep the first pose.
2. Before appending the last pose, pop kept vertices from the end (never the first) while the
   last kept vertex is within $\ell_c$ of the last pose; then append the last pose.
3. Every chord is now at least $\ell_c$ long, except a final chord that starts at the first pose.
   A cusp is a negative dot product between consecutive chords and raises
   `InvalidPath("plan contains reversing segments; use a forward-only planner")` until M7.

Reversals shorter than $\ell_c$ are ignored, including NavFn's `smoothApproachToGoal` hook at the
goal and its start-cell hook at the robot. Without step 2 the final chord can be the sub-cell hook
itself; a Monte Carlo of NavFn tails flagged about 6% of goals that way.

**Detection guarantee** [Derived, M1]. A reversal longer than $2\ell_c$ is always flagged. One
between $\ell_c$ and $2\ell_c$ is flagged or not depending on where the chord grid falls (measured:
a 0.3 m reversal is flagged in 64/64 alignments at res 0.05, $\ell_c = 0.10$, but in 8/64 at res
0.10, $\ell_c = 0.20$). An unflagged short reversal is followed forward with a deviation below
$2\ell_c$; planner cusps (Smac Reeds–Shepp, Lattice reverse primitives) are far longer.

(sec-gains)=

## 3 Gains

A **G-type** function [Thesis Def. 10] is Lipschitz and strictly increasing on $[0, \infty)$ with
$G(0) = 0$ and $G < 1$; $H = \sqrt{1 - G^2}$ [Thesis Def. 11]. $G^2 + H^2 = 1$ is what makes
$\lVert \Phi_S \rVert = 1$.

| | `atan` (thesis experiments) | `rational` [Design] |
|---|---|---|
| $G(D)$ | $\frac{2}{\pi}\arctan(k_f D)$ | $k_f D / \sqrt{1 + (k_f D)^2}$ |
| $H(D)$ | $\sqrt{(1 - G)(1 + G)}$ (this form avoids cancellation) | $1/\sqrt{1 + (k_f D)^2}$ |
| $G'(D)$ | $\frac{2 k_f}{\pi}\,\frac{1}{1 + (k_f D)^2}$ | $k_f\,(1 + (k_f D)^2)^{-3/2}$ |
| $G'(0)$ | $2k_f/\pi \approx 0.6366\, k_f$ | $k_f$ |
| $G^{-1}(y)$ | $\tan(\pi y/2)/k_f$ | $y / (k_f \sqrt{1 - y^2})$ |
| $c_G$ | $0.1856$ (at $k_f D \approx 0.879$) | $2/(3\sqrt3) \approx 0.3849$ (at $k_f D = 1/\sqrt2$) |

**Rational normalisation** [Design]. The thesis lists $G = D/\sqrt{D^2 + k'}$, whose $k'$ has units
of m². The normalised form above equals it with $k' = 1/k_f^2$, so `convergence_gain` keeps its
meaning (1/m) when `gain_function` changes, and every rule written with $G'(0)$ applies to both.

**Integral-curve curvature** [Derived]. On a straight path with $T = (1, 0)$, the static field at
height $D$ has heading $\vartheta = -\arcsin G(D)$ relative to $T$, and $\dot D = -v_r G$ along
its integral curves. The curvature of an integral curve is
$\lvert d\vartheta/ds \rvert = \lvert d\vartheta/dD \rvert \cdot \lvert dD/ds \rvert = \frac{G'}{H}\, G$.
Its maximum over $D$ is $\kappa_F = c_G k_f$ with the constants above (computed numerically for
atan, analytically for rational; {ref}`sec-math-constants`). Near a curve of curvature $\kappa$ the
required curvature is at most about $c_G k_f + \kappa$. This drives the advisor rules of
{ref}`sec-math-validation`. Also $H' = -G G'/H$.

**Discrete convergence** [Derived]. Near the curve, ZOH at period $\Delta t$ gives
$D_{k+1} \approx (1 - G'(0)\, v_r \Delta t)\, D_k$. The approach is monotone, with no overshoot
across the curve, iff $G'(0)\, v_r \Delta t < 1$; the validator warns otherwise.

(sec-distance-field)=

## 4 Distance field

(sec-static-field)=

### 4.1 Static field

$$
\Phi_S = -G(D)\, n + H(D)\, T_d, \qquad \Phi = v_r\, \Phi_S \quad (\Phi_T = 0,\ \eta = v_r).
$$

$\Phi_G = -G n$ converges [Thesis 3.24]; $\Phi_H = H T_d$ circulates [Thesis 3.25]. Because
$n^\top T = 0$ (Lemma 2, and exactly by construction in {ref}`sec-wedge` and {ref}`sec-parametric`)
and $G^2 + H^2 = 1$, $\lVert \Phi_S \rVert = 1$ [Thesis Lemma 7] and $\lVert \Phi \rVert = v_r$:
the field has no zeros and no equilibria. At $D < \varepsilon_D$, $\Phi = v_r T_d$. All Nav2 plugin
curves are static.

(sec-feedforward)=

### 4.2 Time-varying feedforward

[Thesis 3.12, 3.26, 3.27]:

$$
\Phi = \eta\, \Phi_S + \Phi_T, \qquad \Phi_T = -\Pi_T\, \frac{\partial \mathbf D}{\partial t}.
$$

**[Derived] Only the curve velocity at the projection is needed.** With $x^\star = r(s^\star, t)$,
$\partial x^\star / \partial t = T\, \partial s^\star / \partial t + v_c$ with
$v_c = \partial r / \partial t \rvert_{s^\star}$. $\Pi_T$ removes the tangential term, so

$$
\Phi_T = \Pi_T\, v_c = v_c - (T^\top v_c)\, T ,
$$

the normal velocity of the curve at the closest point. It is invariant to time-varying
reparametrisation and needs no second closest-point solve.

**Sources of $v_c$ in the core.** A curve is evaluated in its own frame $\mathcal F$, given by
`CurveFrame` $\{T_{\mathrm{eval}\leftarrow\mathcal F} = (R_{\mathcal F}, p_{\mathcal F}),\ (v_{\mathcal F}, \omega_{\mathcal F})\ \text{in}\ \mathcal F\}$.
The point is mapped into $\mathcal F$, the closest point is found there, and $x^\star$, $T$, $n$ and
$v_c$ are mapped back:

$$
v_c = R_{\mathcal F}\big(v_{\mathcal F} + \omega_{\mathcal F}\, J\, x^\star_{\mathcal F} + \partial_t r_{\mathcal F}(u^\star, t)\big).
$$

- $\partial_t r_{\mathcal F}$ for an expression that uses `t`: central difference in $t$ at fixed
  $u^\star$ with $\delta_t = 10^{-4}$ s [Spec]; 0 otherwise.
- The frame twist is supplied by the follower. From Odometry, the twist is expressed in
  `child_frame_id`, which must equal $\mathcal F$. From TF, each accepted sample pair gives the body
  twist $(v_{\mathcal F}, \omega_{\mathcal F}) = \log_{SE(2)}(T_{k-1}^{-1} T_k)/\Delta t$ with
  $\Delta\theta = \operatorname{wrap}(\theta_k - \theta_{k-1})$: for the relative transform
  $(\Delta p, \Delta\theta)$ in $\mathcal F_{k-1}$, $\omega_{\mathcal F} = \Delta\theta/\Delta t$ and
  $v_{\mathcal F} = V(\Delta\theta)^{-1} \Delta p / \Delta t$ with
  $V(\vartheta) = \frac{\sin\vartheta}{\vartheta} I + \frac{1 - \cos\vartheta}{\vartheta} J$
  ($V = I$ as $\vartheta \to 0$). This is exact for a constant twist; low-pass filters act on the
  twist, never on angles.
- Stale frame input: the follower holds the last frame pose and passes zero twist ($\Phi_T = 0$).

**Assumption-3 clamp** [Design]. If $\lVert \Phi_T \rVert > (1 - \varepsilon_v)\, v_r$, $\Phi_T$ is
scaled to norm $(1 - \varepsilon_v)\, v_r$ and kAssumption3Clamped is set. Convergence is then not
guaranteed.

**Open time-varying curves are rejected** (UNSUPPORTED): the terminal field below is proved only for
static curves. Time-varying means an expression whose `GetUsedVar()` contains `t`, or a goal with
`frame_motion` MOVING.

(sec-eta)=

### 4.3 The scale $\eta$

$\lVert \eta \Phi_S + \Phi_T \rVert^2 = v_r^2$ with $\lVert \Phi_S \rVert = 1$ gives [Thesis 3.28]

$$
\eta^2 + 2 b\, \eta + \lVert \Phi_T \rVert^2 - v_r^2 = 0, \qquad
b = \Phi_S^\top \Phi_T = -G\, n^\top \Phi_T ,
$$

and the larger root is used [Thesis 3.29]:

$$
\eta = -b + \sqrt{b^2 + c_\eta}, \qquad c_\eta = v_r^2 - \lVert \Phi_T \rVert^2 > 0 .
$$

For $b > 0$ the algebraically equal $\eta = c_\eta / (b + \sqrt{b^2 + c_\eta})$ avoids
cancellation; either form meets the $10^{-12}$ oracles, because the error is absolute.

**Lemma 8** [Thesis 3.31–3.32]. With $p_T = \lVert \Phi_T \rVert$, $\lvert b \rvert \le G p_T \le p_T$
and $\eta$ decreases in $b$, so pointwise $v_r - p_T \le \eta \le v_r + p_T$, hence
$0 < v_r - v_m \le \eta \le v_r + v_m$. After the clamp, $\eta \ge \varepsilon_v v_r > 0$.

**Why only $\Phi_S$ is scaled.** Scaling $\Phi_T$ as well would break the cancellation of
$\partial P/\partial t$ in the Lyapunov derivative below (thesis p. 65). This is also why the
limiter acts on $\eta$ for time-varying fields ({ref}`sec-limiter`).

(sec-convergence)=

### 4.4 Convergence and ultimate bound

**Proposition 2** [Thesis 3.35–3.37]. Under Assumption 4, along $\dot x = \Phi$:
$\dot P = \mathbf D^\top \Phi + \partial P/\partial t$. The $H$ term vanishes by Lemma 2, and
$\mathbf D^\top \Phi_T = -\mathbf D^\top \Pi_T \partial\mathbf D/\partial t$ cancels
$\partial P/\partial t$ (Lemma 6). Hence $\dot P = -\eta G D$, i.e.

$$
\dot D = -\eta\, G(D) \le -(v_r - v_m)\, G(D), \qquad \text{static: } \dot D = -v_r\, G(D) .
$$

Equivalently $\dot D = n^\top(\dot x - v_c)$, which is how the oracles evaluate it. No curvature
bound is needed for this first-order argument; only uniqueness of the closest point along the
trajectory (the reach of the curve) matters.

**Proposition 3, ultimate bound** [Thesis 3.38, 5.4]. With $\dot x = \Phi + \delta_v$ and
$\lVert \delta_v \rVert \le \Delta_v < v_r - v_m$, trajectories enter and stay in
$D < G^{-1}\big(\Delta_v / (v_r - v_m)\big)$; for atan,

$$
D < \frac{1}{k_f} \tan\!\Big(\frac{\pi}{2}\, \frac{\Delta_v}{v_r - v_m}\Big).
$$

(sec-terminal-field)=

## 5 Terminal field

(sec-terminal-def)=

### 5.1 Definition

Nav2 paths end at a goal, which Assumption 1 excludes. For a static open curve extended by rays
[Design]:

$$
\Phi = v_r \big(-G(D)\, n + H(D)\, \varphi(\sigma)\, T_d\big), \qquad
\sigma = \mathrm{dir}\,(s_{\mathrm{goal}} - s^\star),
$$

with $s_{\mathrm{goal}}$ the end station for $\mathrm{dir} = +1$ and the start station for
$\mathrm{dir} = -1$ (plugin: the station of $W_{\mathrm{back}}$, $\mathrm{dir} = +1$). The speed
profile is

$$
\varphi(\sigma) = \operatorname{sgn}(\sigma)\, \min\!\Big(1,\ \max\!\Big(\frac{\lvert\sigma\rvert}{\sigma_b},\ \frac{v_{\min}}{v_r} \min\!\Big(1,\ \frac{\lvert\sigma\rvert}{\sigma_{\mathrm{stop}}}\Big)\Big)\Big),
\qquad \sigma_{\mathrm{stop}} = \mathrm{xy\_tol}/2 .
$$

Properties: $\varphi$ is odd, $\lvert \varphi \rvert \le 1$, $\varphi = \operatorname{sgn}(\sigma)$
for $\lvert \sigma \rvert \ge \sigma_b$, and it is Lipschitz with constant
$L_\varphi = \max(1/\sigma_b,\ v_{\min}/(v_r \sigma_{\mathrm{stop}}))$. For
$\lvert \sigma \rvert \ge \sigma_{\mathrm{stop}}$, $\lvert \varphi \rvert \ge v_{\min}/v_r$: this
is the `min_approach_speed` floor. The floor binds somewhere iff
$v_{\min} \sigma_b > v_r \sigma_{\mathrm{stop}}$ (not at the defaults with xy_tol 0.25; yes with
xy_tol 0.05). The speed is $\lVert \Phi \rVert = v_r \sqrt{G^2 + H^2 \varphi^2}$, which tends to 0 at
the goal: the unit-norm property is deliberately given up here.

(sec-terminal-shift)=

### 5.2 Reference-point shift and base-link quantities

A law whose reference point sits at $x_{\mathrm{ref}}$ ahead of base_link, along the drive
direction, evaluates the field there with [Design]

$$
s_{\mathrm{goal,ref}} = s_{\mathrm{goal}} + \mathrm{dir}\, x_{\mathrm{ref}}, \qquad
g_{\mathrm{ref}} = r(s_{\mathrm{goal,ref}}),
$$

so $\sigma_{\mathrm{ref}} = \mathrm{dir}\,(s_{\mathrm{goal,ref}} - s^\star(x_{\mathrm{ref}}\ \text{point}))$.
$g_{\mathrm{ref}} = g_c + x_{\mathrm{ref}} T_{\mathrm{end}}$ whenever $s_{\mathrm{goal,ref}}$ lies on
the forward ray ($x_{\mathrm{ref}} \ge 0$) or on the last segment
($\lvert x_{\mathrm{ref}} \rvert \le$ its length). Values: $x_{\mathrm{ref}} = d$ for FL in FOLLOW
(the control point is $d$ ahead in the drive direction, also with `drive_backward`);
$x_{\mathrm{ref}}$ = `reference_point_offset` for Ackermann in every phase; 0 otherwise.

*Why* [Derived]. On an aligned approach the reference point is at station
$s^\star_{\mathrm{base}} + \mathrm{dir}\,x_{\mathrm{ref}}$, so
$\sigma_{\mathrm{ref}} = \mathrm{dir}(s_{\mathrm{goal}} - s^\star_{\mathrm{base}}) = \sigma_{\mathrm{base}}$.
Base_link, not the reference point, therefore stops at the goal, and the speed demand is continuous
when FL hands over to HT at base_link in FINAL_APPROACH. Without the shift, FL brakes against
$\sigma_{\mathrm{base}} - d$, and at the switch the demand jumps (defaults: 0.20 to 0.36 m/s),
producing a brake–accelerate–brake. FL must still not stay in charge to the end: a control point
driven to $g_{\mathrm{ref}}$ leaves the base $2d\sin(e/2)$ from the goal at arrival heading error
$e$, which exceeds xy_tol for $e > 2\arcsin(\mathrm{xy\_tol}/(2d))$ (14° for $d = 0.2$ and
xy_tol 0.05), while $\lVert F \rVert \to 0$ stops all motion. FINAL_APPROACH therefore hands FL over
to HT at base_link on the unshifted field.

**Measured at base_link against the unshifted $s_{\mathrm{goal}}$, for every law:**
$\sigma_{\mathrm{base}}$, settled, the FINAL_APPROACH entry and exit, $\sigma_{\mathrm{end}}$, the
Ackermann $v_{\mathrm{ack}}$ band and the Ackermann "$\sigma < 0$" test, and the follower's
`xy_latched`. Nav2 checks base_link, so these must too. When the law's point is not base_link,
$\sigma_{\mathrm{base}}$ comes from a second closest-point search at $p$ with the same policy.

(sec-terminal-convergence)=

### 5.3 Convergence [Derived]

Consider the static terminal field on a borderless extended curve, $\dot x = \Phi$.

1. **Distance.** $T \perp n$ still holds, so $\dot D = n^\top \Phi = -v_r G(D)$, exactly as in the
   static case: $D \to 0$ monotonically.
2. **Remaining arc length.** By Lemma 3, $\dot s^\star = T^\top \dot x / \Omega$, so
   $\dot\sigma = -\mathrm{dir}\, \dot s^\star = -T_d^\top \Phi / \Omega = -v_r H(D)\, \varphi(\sigma)/\Omega$,
   with $\Omega = 1$ on segments and rays and $\Omega > 0$ for $D$ below the reach on curved pieces.
3. **No crossing.** $\varphi$ is odd and Lipschitz, so $\sigma = 0$ is an equilibrium of
   $\dot\sigma = -\mu(t)\varphi(\sigma)$ with $\mu > 0$; by uniqueness $\sigma$ keeps its sign and
   $\lvert\sigma\rvert$ is non-increasing.
4. **Arrival.** For $\lvert\sigma\rvert \ge \sigma_{\mathrm{stop}}$,
   $\lvert\dot\sigma\rvert \ge v_{\min} H/\Omega \to v_{\min}$ as $D \to 0$, so $\sigma_{\mathrm{stop}}$
   is reached in finite time. Below it, $\varphi$ is linear and $\sigma$ decays exponentially and
   never reaches 0. Hence $x \to g_c$ (or $g_{\mathrm{ref}}$ for a shifted law), and "$\sigma \le 0$"
   is never a usable success test; settled ({ref}`sec-phases`) is.
5. **Vertex wedges.** In a wedge, $s^\star$ is constant, $\dot\sigma = 0$ and $\dot D = -v_r G$ still
   hold, and the point turns about $P_v$ at rate $v_r H \varphi / D > 0$; the wedge angle is below
   $\pi$, so it leaves the wedge in finite time. The goal vertex is a collinear ray junction and
   never a wedge.

(sec-terminal-discrete)=

### 5.4 Discrete time and deceleration [Derived]

- **ZOH never crosses the goal.** On the curve, $\sigma_{k+1} = \sigma_k - \Delta t\, \gamma v_r \varphi(\sigma_k)$,
  so the sign is kept iff $\gamma v_r \Delta t L_\varphi < 1$, i.e. $v_r \Delta t < \sigma_b$ and
  $v_{\min} \Delta t < \sigma_{\mathrm{stop}}$. Defaults (20 Hz): the factor is
  $1 - v_r\Delta t/\sigma_b = 0.96$ per cycle.
- **Feasible braking.** In the linear region $v = v_r \sigma/\sigma_b$ demands deceleration
  $v_r^2 \sigma/\sigma_b^2 \le v_r^2/\sigma_b$ (0.32 m/s² at the defaults). The limiter can follow
  it iff $\sigma_b \ge v_r^2/a_{\mathrm{dec}}$, which the advisor warns about. In the floor region the
  demand is at most $v_{\min}^2/\sigma_{\mathrm{stop}}$.

(sec-terminal-mode)=

### 5.5 Terminal mode and the window-end cap

- **Terminal mode** (plugin; in core). Enter when $\lVert g - g_c \rVert < 0.05$ m; leave on a new
  `goal_seq` or when that distance exceeds 0.25 m. Only in terminal mode is the terminal field used
  and FINAL_APPROACH possible; otherwise the window carries the static field and
  $\sigma_{\mathrm{base}} := +\infty$. Leaving terminal mode returns FINAL_APPROACH to FOLLOW
  [Spec, M1]. The follower passes $g = r(s_{\mathrm{goal}})$, so its open curves are always terminal.
- **Window-end cap** (`cap_at_curve_end`, plugin true): a speed-shaping term
  $\gamma_{\mathrm{ap}} = \min\big(1,\ \sqrt{v_{\min}^2 + 2 a_{\mathrm{dec}}\, \sigma_{\mathrm{end}}}\,/v_r\big)$,
  with $\sigma_{\mathrm{end}} = \max(0,\ S_{N-1} - s^\star_{\mathrm{base}})$ to the window's last real
  vertex. Truncated or late windows therefore slow down before their end.

(sec-contour)=

## 6 Obstacle contour

(sec-edt)=

### 6.1 Obstacle distance

- **Mask.** Sites are lethal cells, plus unknown cells when `unknown_is_obstacle`. Distances are
  to site *centres*.
- **EDT.** Felzenszwalb–Huttenlocher squared EDT with a feature transform $f(c)$ = the nearest
  site centre of cell $c$; rebuilt only when the grid changes. No sites: $D_o = +\infty$.
- **Continuous query** [Design]. For a point $x$, take the four cell centres $c_{ij}$ around $x$
  (indices clamped to the grid) with bilinear weights $w_{ij}$, and $v_{ij} = x - f(c_{ij})$. If
  every pair satisfies $v_{ij}^\top v_{kl} \ge 0$ (no two differ by more than 90°) and none is
  zero, $v = \sum w_{ij} v_{ij}$; otherwise $v = x - f(c_{\mathrm{nearest}})$. Then
  $D_o = \lVert v \rVert$, $n_o = v/D_o$, $x_o^\star = x - v$. Outside the grid, $D_o = +\infty$.
  *Rationale*: along a straight wall the raw feature switches every cell, so $n_o$ sweeps
  $\pm\arctan(\mathrm{res}/(2 D_o))$ (±4° at $D_o = 0.36$, res 0.05) and jumps back, which gives
  FL a 0.28 rad/s $\omega$ sawtooth. Bilinear interpolation reproduces linear functions, so along
  an axis-aligned wall the tangential component is exactly zero.
- **Two EDTs.** $\mathrm{EDT}_{\mathrm{all}}$ over the full mask serves the gate, the side-probe
  clearance, the clearance slowdown and FootprintChecker. $\mathrm{EDT}_K$ over the sites in $K$
  ({ref}`sec-contour-mods`) serves the contour field and the start and exit tests.

(sec-contour-thesis)=

### 6.2 The thesis formulation in 2D

[Thesis 3.61–3.72] With $\mathbf D_o = x - x_o^\star$:

$$
\mathbf D_\lambda = \mathbf D_o - \lambda \frac{\mathbf D_o}{D_o} = (D_o - \lambda)\, n_o, \qquad
D_\lambda = \lvert D_o - \lambda \rvert, \qquad
\Psi_G = -G(D_\lambda)\, \operatorname{sgn}(D_o - \lambda)\, n_o ,
$$

$$
T_\lambda = \frac{\Pi_{n_o} A}{\lVert \Pi_{n_o} A \rVert}\ \ (A = \Phi)
= \operatorname{sgn}\!\big((J n_o)^\top \Phi\big)\, J n_o, \qquad
\Psi_H = H(D_\lambda)\, T_\lambda, \qquad
\Psi = \eta\,(\Psi_G + \Psi_H) + \Psi_T ,
$$

$$
\theta = \frac{D_o - D_{\mathrm{in}}}{D_{\mathrm{in}}' - D_{\mathrm{in}}}, \qquad
F = \begin{cases}
\Phi & n_o^\top \Phi \ge 0 \ \text{or}\ D_o > D_{\mathrm{in}}' \\
v_r \operatorname{unit}(\theta \Phi + (1 - \theta)\Psi) & n_o^\top \Phi < 0,\ D_{\mathrm{in}} \le D_o \le D_{\mathrm{in}}' \\
\Psi & n_o^\top \Phi < 0,\ D_o < D_{\mathrm{in}}
\end{cases}
$$

with $\lambda < D_{\mathrm{in}} < D_{\mathrm{in}}'$. $\Psi_T$ [Thesis 3.69] is the analogue of
$\Phi_T$ for a moving obstacle. Costmap obstacles carry no velocity, so $\Psi_T = 0$ here; moving
obstacles are collision_monitor's job. Where the nearest point is unique and $F = \Psi$,
$\dot D_o = n_o^\top \Psi = -\lVert\Psi\rVert G(D_\lambda)\operatorname{sgn}(D_o - \lambda)$, so
$\Psi$ attracts to the level set $D_o = \lambda$ [Derived]. The thesis gives no convergence
proof for this extension.

(sec-contour-trap)=

### 6.3 Why the circulation side is latched: the trap basin [Derived]

Take the path $y = 0$ traversed in $+x$ and a disc of radius $R_o$ centred at the origin. Put the
evaluation point on the circle $\rho = R_o + \lambda$ at angle $\alpha$ from the head-on point:
$x = (-\rho\cos\alpha,\ \rho\sin\alpha)$, $n_o = (-\cos\alpha, \sin\alpha)$,
$J n_o = (-\sin\alpha, -\cos\alpha)$, and $dx/d\alpha = -\rho\, J n_o$, so motion along $+J n_o$
*decreases* $\alpha$. At height $y = \rho \sin\alpha > 0$ the static field is
$\Phi = v_r(H(y), -G(y))$, and

$$
(J n_o)^\top \Phi = v_r\big(G(\rho\sin\alpha)\cos\alpha - H(\rho\sin\alpha)\sin\alpha\big)
\approx v_r\, \alpha\, (G'(0)\rho - 1) \quad (\alpha \to 0).
$$

If $G'(0)\rho > 1$, then (3.67) picks $T_\lambda = +J n_o$ for $0 < \alpha < \alpha^\star$, which
drives the point back toward $\alpha = 0$; by symmetry the same holds from below. The head-on point
is an attracting sliding equilibrium, and $n_o^\top\Phi < 0$ keeps the contour active forever. The
condition is $k_f \rho > \pi/2$ for atan and $k_f \rho > 1$ for rational. $\alpha^\star$ solves
$G(\rho\sin\alpha)\cos\alpha = H(\rho\sin\alpha)\sin\alpha$:

| $k_f\rho$ (atan) | ≤ 1.571 | 1.6 | 1.8 | 2.5 | 3.0 | 4.0 |
|---|---|---|---|---|---|---|
| basin half-width $\alpha^\star$ | none | 8.6° | 22.9° | 40.4° | 46.6° | 54.0° |

| $k_f\rho$ (rational) | ≤ 1 | 1.1 | 2.5 | 4.0 |
|---|---|---|---|---|
| basin half-width $\alpha^\star$ | none | 24.6° | 66.4° | 75.5° |

A wall perpendicular to the path always traps, whatever $G$: on the offset line $x = -\lambda$,
$(J n_o)^\top \Phi = v_r\, G(\lvert y \rvert) \operatorname{sgn}(y)$ always points back to the
crossing. A head-on approach starts inside the basin, so the thesis's fallback (keep the previous
sign where $\Pi A = 0$) covers only the measure-zero singular point. Realistic settings meet the
condition ($k_f = 2$ and $\rho \ge 0.79$ m). Hence the latch below.

(sec-contour-mods)=

### 6.4 Modifications [Design]

The contour runs at the active law's evaluation point $x$ ({ref}`sec-laws-points`).

**Gate.** `obstacle_activation: always` reproduces the paper. `path_blocked` (default):

- samples $r(s)$ one cell apart for $s \in [s^\star,\ \min(s^\star + \ell_{\mathrm{look}},\ s_{\mathrm{end}})]$,
  with $s^\star$ the cycle's closest point, $\ell_{\mathrm{look}}$ = `obstacle_lookahead` (auto
  $\max(1.0,\ v_r T_h)$) and $s_{\mathrm{end}}$ the goal for open curves or the last window vertex;
  closed curves wrap; rays are never sampled; for $\mathrm{dir} = -1$ the samples run toward
  decreasing $s$ and are kept in traversal order [Spec, M1];
- a sample is blocked iff $D_o^{\mathrm{all}}(r(s)) < w_{\mathrm{lat}} - $ `obstacle_gate_tolerance`
  (0.05). Grazing contacts are left to the predictor;
- the gate is open in a cycle with a blocked sample; it closes after staying clear for
  `obstacle_release_time` (0.5 s): a clear timer resets on any blocked sample and accumulates
  $\Delta t$ otherwise.

*Rationale*: $\Psi$ attracts to $D_o = \lambda$. Without the gate, (3.72) fires whenever a valid
path passes a wall, door jamb or furniture at clearance $\lambda < c < D_{\mathrm{in}}$ and $\Phi$
leans toward it; $\Psi$ then pulls the robot in to $\lambda$, $\Phi$ takes over, and the robot
limit-cycles, losing clearance exactly where it matters.

**Goal veto.** When the curve end is the goal (follower open curves; plugin in terminal mode) and
the sample at $r(s_{\mathrm{end}})$ is blocked, no episode starts and an active one ends. Circling
cannot reach a goal inside the obstacle; the braking cap stops the robot instead.

**Identity $K$.** The dilated mask is $\{c : \mathrm{EDT}_{\mathrm{all}}(c) \le r_{\mathrm{insc}}\}$.
$K$ is the 8-connected component of it that contains the seed cell. $\mathrm{EDT}_K$'s sites are
the sites of the full mask that lie in $K$; the dilated mask is used only for connectivity, so
$D_o$ is measured to real cells, not to the dilation.

- Seed when no episode is active: the feature cell of the first blocked gate sample. With
  `obstacle_activation: always` there is no gate: the seed is the site nearest to the evaluation
  point, and no flood fill runs while that site is farther than $D_{\mathrm{in}}'$ [Spec, M1].
- Seed during an episode: $k_{\mathrm{seed}}$ = the last $x_o^\star$ in the grid frame (never a cell
  index, never the evaluation frame), mapped each cycle through the current grid origin. If that
  cell is not in the dilated mask, seed from the nearest dilated cell within
  $r_{\mathrm{insc}} + 2\,\mathrm{res}$; if there is none, $K$ is lost and the episode ends.
- $K$, $\mathrm{EDT}_K$ and the gate decision are computed once per `step` and frozen for the
  rollouts, the side probe, the final step and the $\dot\psi_d$ probes. This is exact: the map is
  fixed within the call and every $x_o^\star$ lies in $K$.

*Rationale*: the gate is opened by one obstacle, but the nearest lethal cell is often an unrelated
wall; anchoring the contour to the blocking component stops it from wall-following. Components
closer than $2 r_{\mathrm{insc}}$ merge, and gaps narrower than about $2\lambda$ act as closed.

**Start.** An episode starts only in a cycle where all hold (thesis 3.72's activation region):

1. contour is on, the gate is open and no goal veto applies;
2. measured at $x$ to the active blocking set $B(s^\star)$ (`path_blocked`) or to $K$ (`always`):
   $D \le D_{\mathrm{in}}'$ and $n^\top \Phi < 0$, with $\Phi$ the nominal (unshaped) path field
   [Spec]; no episode starts while $B(s^\star)$ is empty;
3. the side probe returns a side (`path_blocked`: a side that gets past the blockage).

**Blocking set** (`path_blocked`) [Design, M1]. $B = \{b_j = (s_j, f_j)\}$: the station $s_j$ of each
blocked gate sample and its nearest lethal cell $f_j$ (the full-mask feature), kept when $f_j \in K$.
The active set is $B(s^\star) = \{b_j : \mathrm{dir}\,(s_j - s^\star) \ge 0\}$ (station difference
wrapped on closed curves); $D_B$ and $n_B$ are the distance and unit normal from the nearest
$f_j \in B(s^\star)$ to $x$ (below res/2 the normal is taken from base_link). $L_B$ is the station
extent of $B$ (0 with fewer than two samples or with `always`). *Rationale*: $K$ can merge the
blocking obstacle with a long wall parallel to the path; measured to $K$, the start and exit tests
follow that wall for its whole length (U17, O35). The blocking set keeps them on the part of $K$
that actually blocks the path, and releases the episode once that part is behind the robot.

Episodes start only in the once-per-call obstacle preparation, which also runs the side probe;
rollouts never start one. No episode starts while cancelling, in STOPPED, while holding the goal or
in ROTATE_TO_HEADING [Spec, M1]. A consequence: when $v_r T_h$ exceeds $D_{\mathrm{in}}'$, the rollout
sees the blocking obstacle before an episode can start and the cap brakes until it does.

**Side probe**, once per episode start. For $\sigma_c = \pm 1$, roll out ({ref}`sec-collision`) with
the contour forced on at that side, the gate frozen and $\gamma_{\mathrm{col}} = 1$, against the full
mask. Each rollout stops at the episode-exit test, the first collision or

$$
T_{\mathrm{side}} = \max\!\big(T_h,\ (D_{\mathrm{in}}' - \lambda + \pi\lambda + L_B)/v_r\big)
\quad (\approx 4.6\ \mathrm{s}\ \text{for TB3 and a point obstacle}),
$$

long enough for the radial approach, half a circulation and the length of the blocking set, so an
extended obstacle is passed within the probe. Rank the sides by, in order:

1. collision-free;
2. exit reached;
3. larger saturated clearance $\min(c_{\mathrm{cl}},\ 2 m_s)$, where
   $c_{\mathrm{cl}} = \min_{\text{rollout}} \mathrm{EDT}_{\mathrm{all}}(p) - w_{\mathrm{lat}}$ over the
   rollout's base_link positions $p$; values within $\mathrm{res}/2$ are equal;
4. shorter base path to the exit;
5. larger $s_{\mathrm{free}}$;
6. $\operatorname{sgn}((J n_o)^\top \Phi)$, if $\lvert (J n_o)^\top \Phi \rvert \ge 0.1 \lVert \Phi \rVert$;
7. $+1$.

If neither side has $s_{\mathrm{free}} \ge v_r^2/(2 a_{\mathrm{dec}}) + m_s$, no episode starts.
*Rationale*: a 1.5 s horizon ends before the obstacle's flank, so both sides look free and the
tie-break, i.e. tracking noise, picks the side; clearance ranks above path length so a side that
grazes a wall by 1 cm loses to an open one.

*Past the blockage* (`path_blocked`) [Design, M1]. A side reaches its exit (rank 2) only if its
rollout gets past the blockage: it ends on an exit test at a closest-point station with
$\mathrm{dir}\,(s^\star - s_{B,1}) \ge 0$, $s_{B,1}$ the first station of $B$ in traversal order
(station difference wrapped on closed curves). This includes $B(s^\star)$ empty; an exit reached
after $s^\star$ has moved back before $s_{B,1}$, i.e. by retreating, does not count. No episode
starts unless a collision-free side reaches its exit; otherwise the cap brakes the robot short of
the blockage and kBlocked follows ({ref}`sec-collision-blocked`). `always` keeps rank 2 as the plain
exit test and starts an episode on any side. *Rationale*: in a dead end (a corridor closed across
the path, its side walls in $K$ with the closing wall), neither side can pass, yet an episode
started (the ranking required no exit) and ended on a retreat: the evaluation point drops back along
the path, and $B$, rebuilt every cycle from the gate's lookahead, empties once the closing wall is
beyond it, as if passed (in a wider corridor the probe itself met an exit test while retreating
along a side wall). The episode restarted on the way back, so the per-episode watchdog never tripped
and the robot never stopped or set kBlocked (no NoValidControl, no replan). The price: an obstacle
the probe cannot pass within $T_{\mathrm{side}}$ (TC: a wall across the path wider than about 2 m)
stops the robot for a replan instead of being contoured; the watchdog still bounds episodes that
start, and every `always` episode.

**Latched circulation.** $T_\lambda = \sigma_c\, J n_o$ for the whole episode. This is (3.67) with
the sign frozen; it removes the $\Pi A = 0$ singularity and the trap basin of {ref}`sec-contour-trap`.

**Exit**, ending the episode and clearing $\sigma_c$, on any of: $n^\top \Phi \ge 0.05\lVert\Phi\rVert$,
with $n = n_B$ (`path_blocked`) or $n_o$ (`always`); $B(s^\star)$ empty (`path_blocked`);
$D_o > D_{\mathrm{in}}' + h_c$ ($D_o$ measured to $K$); the gate closes; $K$ is lost; the goal veto.
With `path_blocked` the episode usually ends on $B(s^\star)$ empty or the $n_B$ test, before the
gate's release time has passed.

**Field during an episode.**

- $\theta := \operatorname{clamp}\big((D_o - D_{\mathrm{in}})/(D_{\mathrm{in}}' - D_{\mathrm{in}}),\ 0,\ 1\big)$;
  if $D_{\mathrm{in}}' = D_{\mathrm{in}}$, $\theta = 0$ for $D_o < D_{\mathrm{in}}$ and 1 otherwise
  (the thesis cone experiment used that hard switch).
- Speed uses $\lVert\Phi\rVert$, not $v_r$: $\Psi = \lVert\Phi\rVert(\Psi_G + \Psi_H)$ and
  $F = \lVert\Phi\rVert \operatorname{unit}(\theta\Phi + (1 - \theta)\Psi)$, so terminal braking
  survives near obstacles. If $\lVert\theta\Phi + (1 - \theta)\Psi\rVert < 10^{-9}\lVert\Phi\rVert$,
  $F = \Psi$ [Spec]; this cannot happen while $n_o^\top\Phi < 0$ and $D_o > \lambda$, since then both
  terms have a negative $n_o$ component.
- In the band $(D_{\mathrm{in}}',\ D_{\mathrm{in}}' + h_c]$, $\theta = 1$, so $F = \Phi$ while the
  episode and $\sigma_c$ stay latched.
- Guard: if $D_o < \mathrm{res}/2$ at $x$, take $n_o$ from the $\mathrm{EDT}_K$ query at base_link.
- Without an active episode, $F = \Phi$.

**Watchdog.** `travelled_m` accumulates base_link path length and `elapsed_s` accumulates $\Delta t$
from the episode start. `travelled_m` > `obstacle_max_contour_length` (5.0) or `elapsed_s` >
`obstacle_max_contour_time` (20) sets kContourWatchdog (→ STOPPED). The path length is
$\lVert (v_x, v_y) \rVert \Delta t$ of the measured twist, counted from the cycle after the start; a trip
ends the episode, reports its final counters in that cycle, and STOPPED keeps the kContourWatchdog
bit [Spec, M1].

(sec-contour-notes)=

### 6.5 Consequences

- **Steady FL circulation** [Derived]. With $p_c$ moving on a circle of radius $\rho$, the base
  follows the circle of radius $\sqrt{\rho^2 - d^2}$ and the heading error to $F(p_c)$ is
  $\arcsin(d/\rho) \le \arcsin(d/\lambda)$ (25.2° for TB3), below the $\pi/4$ rotate-release angle,
  so a circulation never re-enters ROTATE_TO_FIELD.
- **Auto $\lambda$** [Derived]. $\sqrt{(w_{\mathrm{lat}} + 0.10)^2 + d^2}$ puts the base circle of a
  point obstacle at $w_{\mathrm{lat}} + 0.10$, i.e. 10 cm of lateral clearance;
  $r_{\mathrm{circ}} + d + 0.05$ guarantees $\lambda - d \ge r_{\mathrm{circ}} + 0.05$ by the
  triangle inequality, whatever the heading.
- **Ackermann + contour is rejected** in 0.x: the $\lambda$-offset needs curvature about $1/\lambda$
  (≈ 3 m⁻¹), far above $1/R_{\min}$, and a car turning at $R_{\min} = 1$ m from $D_{\mathrm{in}}'$
  passes the head-on point at $\sqrt{D_{\mathrm{in}}'^2 + 1} - 1 \approx 0.20$ m, inside its footprint.

(sec-laws)=

## 7 Tracking laws

Signature: `Twist2 command(const Pose2&, const FieldSample&, FieldProbe, const Kinematics&, double dt)`.

(sec-laws-points)=

### 7.1 Evaluation points

| Law and phase | Evaluation point $x$ | $x_{\mathrm{ref}}$ |
|---|---|---|
| FL in FOLLOW and ROTATE_TO_FIELD | $p_c = p + q\, d\, \mathbf h(\psi)$ | $d$ |
| FL in FINAL_APPROACH (switches to HT) | $p$ | 0 |
| HT (diff, skid) | $p$ | 0 |
| holonomic | $p$ | 0 |
| Ackermann (HT), every phase | $p_r = p + x_r\, \mathbf h(\psi)$, $x_r$ = `reference_point_offset` | $x_r$ |

Valid combinations: diff and skid with FL (auto) or HT; omni with holonomic; Ackermann with HT.
Anything else is a ConfigError.

(sec-laws-holo)=

### 7.2 Holonomic (omni)

The field is evaluated at base_link: $(v_x, v_y)^\top = R(\psi)^\top F$. The angular rate comes
from `omni_heading_policy`, as feedback only [Spec on the form]:
$\omega = \operatorname{clamp}\big(k_\psi \operatorname{wrap}(\psi_{\mathrm{tgt}} - \psi),\ \pm\bar\omega\big)$
with $\psi_{\mathrm{tgt}} = \angle T_d$ (`path_tangent`) or $\angle F$ (`field`, held while
$\lVert F \rVert < 0.05 v_r$); `fixed` commands $\omega = 0$. The base velocity is exactly $F$
whatever $\omega$, so $\omega$ is limited on its own ({ref}`sec-limiter`).

(sec-laws-fl)=

### 7.3 Feedback linearisation (diff, skid) [Thesis 2.1]

$$
v = \mathbf h^\top F, \qquad \omega = q\, \frac{(J\mathbf h)^\top F}{d}, \qquad
\text{i.e.}\ \begin{bmatrix} v \\ \omega \end{bmatrix} = M(\psi) F,\quad
M(\psi) = \begin{bmatrix} \mathbf h^\top \\ q\,(J\mathbf h)^\top / d \end{bmatrix},\quad
M^{-1} = [\,\mathbf h,\ q\,d\,J\mathbf h\,].
$$

With $q = +1$ this is thesis eq. 2.1: $v = \cos\psi F_x + \sin\psi F_y$,
$\omega = (-\sin\psi F_x + \cos\psi F_y)/d$. *Proof that the control point follows $F$*:
$\dot p_c = v\,\mathbf h + q\, d\,\omega\, J\mathbf h = \mathbf h\mathbf h^\top F + J\mathbf h (J\mathbf h)^\top F = F$.
$d > 0$ is required. $q\,v = \lVert F \rVert \cos e_{c}$ and $\omega = \lVert F \rVert \sin e_{c}/d$, with
$e_c$ the angle from the drive direction $q\,\mathbf h$ to $F$: FL advances only while $\lvert e_c \rvert < \pi/2$,
and $e_c \approx \pi$ is an unstable equilibrium where FL would reverse at full speed; hence
ROTATE_TO_FIELD. The `allow_reversing: false` backstop clamps $q\, v \ge 0$.

(sec-laws-ht)=

### 7.4 Heading tracking (diff, skid; Ackermann's law)

With $\psi_{\mathrm{dr}} = \psi + \pi\,[q = -1]$ and $\psi_d = \angle F$:

$$
e = \operatorname{wrap}(\psi_d - \psi_{\mathrm{dr}}), \qquad
v = q\, \lVert F \rVert \max(0, \cos e), \qquad
\omega = \operatorname{clamp}(\dot\psi_d,\ \pm\omega_c) + k_\psi\, e ,
$$

with $\omega_c = \bar\omega$ (Ackermann: $\lvert v \rvert / R_{\min}$).

- **Desired heading.** $\psi_d$ is held at `last_psi_d` while $\lVert F \rVert < 0.05\, v_r\, \gamma_{\mathrm{lim}}$
  (the external speed-limit scale only, so a low Nav2 speed limit never freezes $\psi_d$); while held,
  $\dot\psi_d = 0$ [Spec, threshold confirmed M1].
- **Feedforward along the actual motion** [Design]:

  $$
  \dot\psi_d = v\,\frac{\operatorname{wrap}\big(\psi_d(x + \delta\mathbf h) - \psi_d(x - \delta\mathbf h)\big)}{2\delta} + \partial_t\psi_d,
  \qquad \delta = \max\big(0.02\ \mathrm{m},\ \lvert v \rvert \Delta t,\ \min(\bar\ell, 0.10\ \mathrm{m})\big),
  $$

  $\bar\ell$ is the polyline's mean vertex spacing (`Curve::vertexSpacing()`; parametric and expression
  curves report none and the term drops out) [Design, M1]. *Rationale*: with $2\delta$ below the
  vertex spacing, the number of vertices between the probes alternates between 0 and 1, so the
  finite difference alternates between 0 and the full vertex turn; on 5 cm polyline circles
  heading tracking then settled 2.7–4.5 cm off the curve (O40). A span of whole vertex spacings
  keeps the count constant.

  The probes use the `FieldProbe`, which returns `ProbeSample{psi_d, s_star, D, contour_active, sigma_c}`
  at the cycle's $t$ with the frozen hint, $K$ and gate. A probe whose field norm is below
  $0.05 v_r$ returns `psi_d` = NaN, and a NaN zeroes $\dot\psi_d$ [Spec].
  $\partial_t\psi_d = \operatorname{wrap}(\psi_d(x, t + \Delta t/2) - \psi_d(x, t - \Delta t/2))/\Delta t$
  for time-varying fields, else 0 [Spec].
- **Jump-only zeroing** [Design]. $\dot\psi_d := 0$ only when the probes see a closest-point jump,
  $\lvert s^\star_+ - s^\star_- \rvert > 1.05\,\big(2\delta + 2\max(D_+, D_-)\big)$ (arc difference
  wrapped on closed curves), or differ in `contour_active` or `sigma_c`. A change of vertex or segment
  is not a jump. The 5% slack covers the polyline arc between feet on different segments, which
  exceeds their chord by up to $1/\cos(\Theta/2)$, $\Theta$ the turn between them [Design, M1].
  *Rationale*: probes $2\delta \ge 4$ cm apart straddle a vertex on most cycles of raw Nav2 plans
  (NavFn vertices every 2.5 cm), and zeroing on every segment change lost the feedforward on
  81–100% of cycles, leaving a steady offset of up to 0.22 m on a 0.5 m circle. On smooth pieces
  $\lvert \Delta s^\star \rvert \le 2\delta R/(R - D)$, which exceeds the threshold only within
  $\delta$ of a centre of curvature; an inner-corner crossing jumps $2D\tan(\theta/2)$, flagged only
  for turns sharper than about 90°; U-turn and lemniscate jumps exceed it by construction. At
  shallow inner bisectors the one-cycle spike integrates to the corner angle and is bounded by the
  $\pm\omega_c$ clamp.
- **Why along $v\mathbf h$.** The heading error obeys $\dot e = \nabla\psi_d^\top \dot x + \partial_t\psi_d - \omega$
  with $\dot x = v\,\mathbf h$. Differentiating along $v\mathbf h$ gives $\dot e = -k_\psi e$
  exactly when unsaturated; differentiating along $F$ would leave a residual
  $\nabla\psi_d^\top(v\mathbf h - F)$, a full-speed feedforward even when $v = 0$. Uniform scaling of
  $(v, \omega)$ preserves the property, since $\dot\psi_d$ is linear in $v$.

(sec-laws-ack)=

### 7.5 Ackermann

HT at the rear-axle reference point $p_r$ (forward only, $q = +1$), on the field shifted by
$x_{\mathrm{ref}} = x_r$ ({ref}`sec-terminal-shift`).

- **Speed band.** In FOLLOW, a positive law speed below $v_{\mathrm{ack}}$ is raised to
  $v_{\mathrm{ack}}$. Only the braking cap (including kBlocked) and STOPPED (including cancel)
  command 0. The band is lifted once $\sigma_{\mathrm{base}} < \mathrm{xy\_tol} + v_{\mathrm{ack}}^2/(2 a_{\mathrm{dec}})$.
  It applies before the kinematic box, and not in a cycle where the cap binds [Spec, M1].
  Rounding down instead would deadlock: at $v = 0$, $\lvert\omega\rvert \le \lvert v \rvert / R_{\min} = 0$,
  so the heading error between about 83° and 90° could never shrink.
- **Speed limit.** $v_r\gamma_{\mathrm{lim}}$ is floored at $v_{\mathrm{ack}}$, with a warning; never in
  STOPPED.
- **Curvature.** $\lvert\omega\rvert \le \lvert v \rvert/R_{\min}$ after scaling and limiting
  (kCurvatureSaturated).
- **kHeadingInfeasible** when $\lvert e \rvert > \pi/2$ at $p_r$; or $\sigma_{\mathrm{base}} < 0$ while XY
  is not reached; or, at settled and only if $\mathrm{yaw\_tol} > 10^{-6}$,
  $\lvert \operatorname{wrap}(\psi_{\mathrm{goal}} - \psi) \rvert > \mathrm{yaw\_tol}$. It leads to
  STOPPED; Nav2 replans or aborts.
- **Command semantics.** $v$ is the longitudinal speed, equal at every point of the body x-axis,
  and $\omega$ the yaw rate. The published twist is $(v, 0, \omega)$; the no-slip point is $p_r$, and
  base_link's lateral velocity $-x_r\omega$ follows from the rigid-body constraint.

(sec-laws-rotate)=

### 7.6 Rotate laws [Spec on the stopping term]

Translation target $(0, 0)$; with $e_r$ the angle to go (ROTATE_TO_FIELD: $e$ of {ref}`sec-phases`;
ROTATE_TO_HEADING: $\operatorname{wrap}(\psi_{\mathrm{goal}} - \psi)$),

$$
\omega^\ast = \operatorname{sgn}(e_r)\,\max\!\big(\omega_{\mathrm{floor}},\ \min(\omega_{\mathrm{rot}},\ \sqrt{2\alpha_{\max}\lvert e_r\rvert})\big),
\qquad \omega_{\mathrm{rot}} = \min(\omega_{\mathrm{rth}},\ \bar\omega),
$$

with $\omega_{\mathrm{rth}}$ = `rotate_to_heading_angular_vel`, then the limiter's angular
acceleration bound (RPP-style). $\omega_{\mathrm{floor}}$ =
`min_in_place_angular_vel` in ROTATE_TO_FIELD. In ROTATE_TO_HEADING the floor applies only while
$\lvert e_r \rvert > \mathrm{yaw\_tol}/2$; inside that band $\omega^\ast = 0$, so $\omega$
decelerates to 0 within the limits.

(sec-laws-monotone)=

### 7.7 Distance monotonicity

- FL (with $D$ at $p_c$) and holonomic, static fields, no active contour episode: after speed
  shaping and the kinematic box, the law's point moves with $c\,\gamma\,\Phi$, so
  $\dot D = -c\,\gamma\, v_r G(D) \le 0$ in continuous time.
- FL and holonomic, time-varying fields under the $\eta$-box: $\dot D = -\eta^\ast G(D) \le 0$
  because $\eta^\ast \ge 0$. Under the uniform-$c$ fallback,
  $\dot D = -c\eta G - (1 - c)\, n^\top v_c$, which can be positive (kAssumption3Clamped is set).
- HT and Ackermann: $\dot D = v\, n^\top \mathbf h$, so $D$ is non-increasing only as $e \to 0$.
- The acceleration stage makes every statement approximate.

(sec-limiter)=

## 8 Speed shaping, kinematic box and limiter

(sec-limiter-order)=

### 8.1 The `stepNominal` pipeline

For the current phase:

1. Field sample at $x$: closest point, $\Phi_S$, $\Phi_T$, $\eta$, nominal $\Phi$ (distance or
   terminal field).
2. Contour state at $x$: gate, start and exit tests, $\theta$ ({ref}`sec-contour-mods`); decisions use
   the nominal $\Phi$.
3. Speed shaping → shaped path field $\Phi_\gamma$ ({ref}`sec-limiter-shaping`).
4. Contour → $F$, with speed $\lVert\Phi_\gamma\rVert$.
5. Law, or the phase target (rotate law; zero for kBlocked, STOPPED and goal hold), then the
   FOLLOW-only Ackermann $v_{\mathrm{ack}}$ band and the rotate floors.
6. Kinematic box ({ref}`sec-limiter-box`).
7. `CommandLimiter::limit` (twist only): acceleration, Ackermann curvature clamp, NaN guard.

Feedback-only angular terms ($k_\psi e$, the omni heading policy, the rotate laws) are not
multiplied by $\gamma$; they are bounded by $\bar\omega$.

(sec-limiter-shaping)=

### 8.2 Speed shaping

$$
\gamma = \min(1,\ \gamma_{\mathrm{lim}},\ \gamma_{\mathrm{cl}},\ \gamma_{\mathrm{ap}},\ \gamma_{\mathrm{col}})
$$

| Term | Definition |
|---|---|
| $\gamma_{\mathrm{lim}}$ | `speed_limit_scale` in $(0, 1]$ (plugin: `setSpeedLimit`; follower: `speed_limit`). Ackermann: $\max(\gamma_{\mathrm{lim}},\ v_{\mathrm{ack}}/v_r)$. |
| $\gamma_{\mathrm{cl}}$ [Spec] | 1 if `clearance_slowdown_distance` $d_{\mathrm{cl}} = 0$ (default); else $\min\big(1,\ k_{\mathrm{cl}} \max(0,\ \mathrm{EDT}_{\mathrm{all}}(p) - w_{\mathrm{lat}})/d_{\mathrm{cl}}\big)$ with $k_{\mathrm{cl}}$ = `clearance_slowdown_gain` |
| $\gamma_{\mathrm{ap}}$ | window-end cap ({ref}`sec-terminal-mode`); 1 when `cap_at_curve_end` is false |
| $\gamma_{\mathrm{col}}$ | $\min(1,\ v_{\mathrm{cap}}/v_r)$ ({ref}`sec-collision`); 1 inside rollouts |

Application:

- **Static fields, or an active episode with $\theta < 1$**: $\Phi_\gamma = \gamma\,\Phi$ (uniform).
- **Time-varying fields otherwise**: re-solve $\eta_1$ with $\gamma v_r$ in place of $v_r$ and keep
  $\Phi_T$: $\Phi_\gamma = \eta_1\Phi_S + \Phi_T$, valid when
  $\lVert\Phi_T\rVert < (1 - \varepsilon_v)\gamma v_r$. Otherwise the safety limit wins:
  $\Phi_\gamma = \gamma\,\Phi$.
- **kAssumption3Clamped** is set only if $\lVert\Phi_T\rVert \ge (1 - \varepsilon_v)\, v_r\min(1, \gamma_{\mathrm{lim}})$,
  i.e. the curve cannot be tracked at the externally limited speed. When the binding term is the
  collision cap or clearance, the state is reported by STATUS_COLLISION_AHEAD or kBlocked instead.
  *Rationale*: otherwise every block on a moving curve would raise the flag and the follower would
  abort with ASSUMPTION3_VIOLATED after 1 s instead of waiting out the block.
- Never clamp one axis alone, except holonomic $\omega$ ({ref}`sec-limiter-box`).

(sec-limiter-box)=

### 8.3 Kinematic box

Limits: $\lvert v_x \rvert \le v_{\max}$, $\lvert v_y \rvert \le v_{y,\max}$, $\lvert\omega\rvert \le \bar\omega$;
omni also $\lVert v_{xy} \rVert \le v_{\max}$. Terms with a zero denominator are dropped.

- **Holonomic $\omega$**, static and time-varying alike: clamped to $\pm\bar\omega$ on its own; it never
  enters $c$ or the $\eta$-box. *Rationale*: $\omega$ does not move the base, and including it would
  turn a heading saturation into a scaled feedforward plus a false Assumption-3 abort.
- **Uniform $c$** (static fields; HT; an active episode with $\theta < 1$; or step 3 fell back to
  $\gamma\Phi$): diff, skid and Ackermann
  $c = \min(1,\ v_{\max}/\lvert v\rvert,\ \bar\omega/\lvert\omega\rvert)$; omni
  $c = \min(1,\ v_{\max}/\lVert v_{xy}\rVert,\ v_{y,\max}/\lvert v_y\rvert)$; the command (translation and
  the law's $\omega$; omni translation only) is multiplied by $c$. No Assumption-3 flag. Under HT with
  a time-varying field the feedforward cancellation is only approximate.
- **$\eta$-box** (time-varying field, FL or holonomic, no episode with $\theta < 1$). The law is linear
  in $F$: $u(\eta) = a + \eta\, b$ with $a = M\Phi_T$, $b = M\Phi_S$ (FL: $M = M(\psi)$,
  $u = (v, \omega)$; holonomic: $M = R(\psi)^\top$, $u = (v_x, v_y)$). Each constraint gives an interval:
  - $\lvert a_i + \eta b_i\rvert \le L_i$: if $b_i \ne 0$, $[\min(e_1, e_2), \max(e_1, e_2)]$ with
    $e_{1,2} = (\pm L_i - a_i)/b_i$; if $b_i = 0$, all of $\mathbb R$ when $\lvert a_i\rvert \le L_i$, else empty;
  - $\lVert a + \eta b\rVert \le v_{\max}$ (omni): with $B = \lVert b\rVert^2$, $A = a^\top b$,
    $C = \lVert a\rVert^2 - v_{\max}^2$, $\Delta = A^2 - BC$: empty if $\Delta < 0$, else
    $[(-A - \sqrt\Delta)/B,\ (-A + \sqrt\Delta)/B]$ ($B = 0$: all of $\mathbb R$ iff $C \le 0$).

  Intersect them with $[0, \eta_1]$. If the set is non-empty, $\eta^\ast$ = its maximum and the law is
  re-run on $\eta^\ast\Phi_S + \Phi_T$ (no new field evaluation). If it is empty, which happens only
  when $M\Phi_T$ itself lies outside the box, fall back to uniform $c$ on the step-3 command and set
  kAssumption3Clamped. A negative $\eta$ is never commanded. *Rationale*: without the lower bound,
  a curve receding at a normal speed above $d\,\bar\omega$ yields a non-empty set below 0; the robot
  then backs away with no flag ($\eta = -0.206$, $v = -0.200$ m/s, $\dot D = +0.05$ m/s in the
  example of oracle O44).

(sec-limiter-accel)=

### 8.4 Acceleration, curvature and NaN guard

**Acceleration** [Design]. With target $u^\ast$, previous output $u_{\mathrm{prev}}$ and
$\Delta_i = u^\ast_i - u_{\mathrm{prev},i}$ on each axis ($x$, $y$, $\omega$):

$$
u = u_{\mathrm{prev}} + \alpha\,(u^\ast - u_{\mathrm{prev}}), \qquad
\alpha = \min\!\Big(1,\ \min_{i:\Delta_i \ne 0} \frac{A_i\,\Delta t}{\lvert\Delta_i\rvert}\Big).
$$

$A_i$ is $a_{\mathrm{dec}}$ when $u_{\mathrm{prev},i}\Delta_i < 0$ and $a_{\mathrm{acc}}$ otherwise for the
linear axes (omni lateral uses the same values) [Spec on the sign rule], and $\alpha_{\max}$ for
$\omega$. Outside translation braking (below), all axes move together along a straight line in
command space, so the FL control-point direction is kept; for FL this is a first-order lag of
$\dot p_c$. In STOPPED: for a cancel, the
linear bound is $a_{\mathrm{can}}$, raised to $\max(a_{\mathrm{can}}, a_{\mathrm{dec}})$ for the rest
of the cancel once a rollout finds a collision, and the angular bound is $\alpha_{\max}$; otherwise
the maximum decelerations. The cycle in which a cancel rollout first finds a collision is re-run with
the raised bound, STATUS_COLLISION_AHEAD is set while that bound applies, and it clears when the
cancel ends [Spec, M1].

**Translation braking** [Design, 0.1.1]. In ROTATE_TO_FIELD, ROTATE_TO_HEADING and STOPPED the
translational target is zero, and the translational axes ($x$, omni $y$) and $\omega$ each take
their own scale:

$$
\alpha_{\mathrm{lin}} = \min\!\Big(1,\ \min_{i \in \{x, y\}:\Delta_i \ne 0} \frac{A_i\,\Delta t}{\lvert\Delta_i\rvert}\Big),
\qquad
\alpha_\omega = \min\!\Big(1,\ \frac{\alpha_{\max}\Delta t}{\lvert\Delta_\omega\rvert}\Big),
$$

$v_{x,y} = (1 - \alpha_{\mathrm{lin}})\,v_{\mathrm{prev},x,y}$ and
$\omega = \omega_{\mathrm{prev}} + \alpha_\omega\Delta_\omega$, with the same $A_i$ as above
($a_{\mathrm{dec}}$; $a_{\mathrm{can}}$ or $\max(a_{\mathrm{can}}, a_{\mathrm{dec}})$ in a cancel).
The translation still brakes along its own direction, so a residual speed $v_0$ stops within
$\lceil v_0/(A\,\Delta t)\rceil$ cycles after at most $v_0^2/(2A) + v_0\Delta t$ of travel.
`limiter_scale` reports $\min(\alpha_{\mathrm{lin}}, \alpha_\omega)$. In FOLLOW and FINAL_APPROACH
the single $\alpha$ stays. *Rationale*: with one $\alpha$ the $\omega$ ramp from 0 to the rotate
speed (about $\omega_{\mathrm{rot}}/(\alpha_{\max}\Delta t)$ cycles) also sets the pace of the
linear stop, so a rotation phase entered at 0.1 m/s crept about 2 cm in about ten cycles instead
of turning in place. Keeping the command direction matters in translation phases, where it is
the FL control-point direction; with a zero translational target there is no direction to keep.

**Ackermann curvature clamp**: if $\lvert\omega\rvert > \lvert v\rvert/R_{\min}$, set
$\omega := \operatorname{sgn}(\omega)\lvert v\rvert/R_{\min}$ and kCurvatureSaturated.

**NaN guard**: any non-finite component gives $u = 0$ and kNonFinite.

**Skid slip compensation** [Design, 0.1.1]. For `skid_steer`, the limited command $u$ is the twist
the robot should realise; the command sent is
$\omega_{\mathrm{cmd}} = \operatorname{clamp}(\lambda\,\omega/\hat\eta_\psi,\ \pm\omega_{\max})$
(kCurvatureSaturated when the clamp acts), with $\hat\eta_\psi$ = `skid_yaw_rate_efficiency`
$\in (0, 1]$ and, in FOLLOW under FL, $\lambda = s/(s - \hat x_{\mathrm{ICR}})$, $s = q\,d$,
$\hat x_{\mathrm{ICR}}$ = `skid_icr_offset` ($\lvert\hat x_{\mathrm{ICR}}\rvert < d/2$); elsewhere
$\lambda = 1$. For the slip model of `sim::integrate` ($\omega' = \eta_\psi\omega_{\mathrm{cmd}}$,
$v_y' = -x_{\mathrm{ICR}}\omega'$) the lateral velocity of the point at body $x = s$ is
$(s - x_{\mathrm{ICR}})\,\omega'$, so the identified values give back $\omega$ in rotations and
$\dot p_c$ under FL. $u_{\mathrm{prev}}$, the rollouts and `cmd_target` stay in realised terms; the
defaults ($1$, $0$) are the identity. *Rationale*: under that slip a skid-steer robot on a
$R = 1$ m circle settles about 8 cm off with no compensation and 4 cm with $\hat\eta_\psi$ alone.

**$u_{\mathrm{prev}}$ re-seeding**: from the measured twist after any throw, on reset or cancel
completion, on a parameter rebuild, and when $\Delta t > 2/f$. A `goal_seq` change does not re-seed.

(sec-phases)=

## 9 Phases and predicates

(sec-phases-predicates)=

### 9.1 Predicates

- **XY reached.** Plugin Gen C: the server's goal checker each cycle, called with the server's own
  arguments, passed in `GoalInfo::xy_reached`. Plugin Gen A/B: a latch
  $\lVert p - g\rVert \le \mathrm{xy\_tol}$, re-armed on every `setPlan`. Follower: the core latch
  `xy_latched`, set when $\lVert p - g\rVert \le$ `goal_tolerance` and cleared only by a `goal_seq`
  change or reset. XY reached never disagrees with Nav2. Inside a rollout, step 0 uses the caller's
  value and later steps emulate the Gen A/B latch $\lVert p - g\rVert \le \mathrm{xy\_tol}$ [Spec, M1].
- **Settled.**

  $$
  \text{settled} := \text{XY reached} \wedge \big(\lVert p - g\rVert \le \sigma_{\mathrm{stop}} \ \vee\ \text{cap binds} \ \vee\ (\text{Ackermann} \wedge \sigma_{\mathrm{base}} \le \sigma_{\mathrm{stop}})\big),
  \qquad \sigma_{\mathrm{stop}} = \mathrm{xy\_tol}/2 .
  $$

  It is `TrackerOutput::settled`, the only entry to ROTATE_TO_HEADING and the follower's open-curve
  success test. "Cap binds" means $\gamma_{\mathrm{col}} < \min(1, \gamma_{\mathrm{lim}}, \gamma_{\mathrm{cl}}, \gamma_{\mathrm{ap}})$
  [Spec]. The along-track clause $\sigma_{\mathrm{base}} \le \sigma_{\mathrm{stop}}$ applies to
  Ackermann only [Design, M1]: the other laws converge laterally under the terminal field, so they
  settle within $\sigma_{\mathrm{stop}}$ of the goal itself, whereas a car-like base cannot remove a
  lateral offset without reversing. *Rationale*: a robot that rotates as soon as it enters xy_tol stops at the tolerance edge,
  where one outward localisation correction undoes XY reached; Nav2 re-arms its checker on every
  replan, and the robot dances between FOLLOW and ROTATE_TO_HEADING. Settling inside
  $\sigma_{\mathrm{stop}}$ leaves half the tolerance as margin. Nav2 still ends a goal on entering
  xy_tol when the yaw already matches; settled is not required then.
- **Yaw tolerance.** yaw_tol is the yaw of `getTolerances().orientation`; $\le 10^{-6}$ means
  unchecked (PositionGoalChecker reports identity).
- **Goal identity.** The ROS layer compares the raw plan's last pose in the plan frame
  (1 cm / 0.01 rad) and increments `goal_seq`. A change resets only goal-scoped state:
  `terminal_mode`, the XY latch, and FINAL_APPROACH / ROTATE_TO_HEADING / STOPPED → FOLLOW. The
  contour episode, rotate hysteresis, hint and limiter persist.

(sec-phases-table)=

### 9.2 Phases

| Phase | Entry | Behaviour | Exit |
|---|---|---|---|
| ROTATE_TO_FIELD | diff/skid, `use_rotate_to_field`, $\lvert e\rvert >$ `rotate_to_field_angle` ($\pi/2$), with $e = \operatorname{wrap}(\angle F_{\mathrm{trk}} - \psi_{\mathrm{dr}})$ where $F_{\mathrm{trk}}$ is the post-contour field the active law tracks, at that law's own point ($p_c$ for FL; base_link for HT and in FINAL_APPROACH) | rotate law ({ref}`sec-laws-rotate`), $v = 0$ | $\lvert e\rvert <$ `rotate_to_field_release_angle` ($\pi/4$), same $e$; then FINAL_APPROACH if its entry holds, else FOLLOW |
| FOLLOW | default | `stepNominal` | — |
| FINAL_APPROACH | open curve (plugin: terminal mode) and $\sigma_{\mathrm{base}} <$ `final_approach_distance` (auto: $\max(2d,\ \mathrm{xy\_tol} + d)$ for FL, xy_tol otherwise) | FL switches to HT at base_link on the unshifted terminal field, honouring `drive_backward`; holonomic, HT and Ackermann keep their own law and point | settled |
| ROTATE_TO_HEADING | settled, `use_rotate_to_heading`, yaw_tol $> 10^{-6}$ (diff/skid/omni; not Ackermann; not the follower) | rotate law toward the goal yaw | the checker succeeds; if XY is no longer reached: FINAL_APPROACH, or FOLLOW if $\sigma_{\mathrm{base}} \ge$ `final_approach_distance` |
| STOPPED | kHeadingInfeasible, kContourWatchdog, or `input.cancel` | target 0 on every axis, bypassing the $v_{\mathrm{ack}}$ band, the rotate floors and goal hold; decelerations per {ref}`sec-limiter-accel`; a cancel collision sets STATUS_COLLISION_AHEAD and never throws | latched until `goal_seq` changes or reset; a cancel ends when the output is below 0.01 m/s and 0.01 rad/s |

$F_{\mathrm{trk}}$ is evaluated only when $\lVert F_{\mathrm{trk}}\rVert \ge 0.05 v_r$; otherwise there is
no entry and an active rotation keeps its last target [Spec]. The norm is taken before speed shaping,
so a binding collision cap never changes the decision. Rotation phases and STOPPED evaluate
$F_{\mathrm{trk}}$ for the phase the robot will track next (FINAL_APPROACH if its entry holds, else
FOLLOW), so entry and exit measure the same angle [Spec, M1].

**Why $\pi/2$ and $\pi/4$, measured at the tracked point** [Design]. Under FL, $v = \mathbf h^\top F$
reaches 0 at $\lvert e\rvert = \pi/2$; below that FL turns while advancing, so the phase is needed only
to avoid the unstable equilibrium at $e \approx \pi$. Closed-loop simulation of these laws showed:
measuring $e$ at base_link with 0.785/0.39 livelocks FL at polyline turns of 120° or more (16
entries and no progress), because FL turns toward $F(p_c)$ before the base reaches the corner;
$F(p_c)$ with $\pi/2$ / $\pi/4$ gives exactly one entry per corner from 60° to 170° and on a 0.15 m
U-turn, while keeping goal-behind starts at one entry with no $v < 0$. Measuring the uncontoured path
field instead would turn the robot back into the obstacle during every contour episode.

**Goal hold.** Once settled with no rotate phase (`use_rotate_to_heading` false, yaw unchecked, or
Ackermann), the target is zero and the phase is kept until XY is no longer reached. This needs a
latch, $\mathrm{hold}_k = \text{settled}_k \vee (\mathrm{hold}_{k-1} \wedge \text{XY reached}_k)$,
stored in `TrackerState::holding` ({ref}`sec-math-spec-choices`, item 13).

**kBlocked** is a per-cycle status, not a phase ({ref}`sec-collision-blocked`). While it is set, the
phase's target becomes 0 and the limiter decelerates; the next rollout re-runs the same phase, so
motion resumes by itself.

**Transition order within a cycle** [Spec]:

1. a `goal_seq` change resets goal-scoped state;
2. `input.cancel`, or a STOPPED-entry status raised in this cycle, enters STOPPED; a latched STOPPED
   stays;
3. settled → ROTATE_TO_HEADING when it applies, else goal hold;
4. in ROTATE_TO_HEADING, the "XY no longer reached" exit;
5. FINAL_APPROACH entry;
6. ROTATE_TO_FIELD entry and exit;
7. otherwise the phase is kept.

**Precedence.** In a translation phase with XY reached, a binding cap makes the cycle settled and
never kBlocked; this cycle's target is 0, and ROTATE_TO_HEADING or goal hold starts next cycle, whose
rollout checks the swept rotation.

**Backstop.** `allow_reversing` (false) clamps the drive-direction speed to $q\,v \ge 0$, after the
kinematic box, for unicycle and Ackermann drives (an omni body $v_x < 0$ is legitimate) [Spec, M1].

(sec-phases-follower)=

### 9.3 Follower progress and success

- Success: closed curves when `laps` > 0 and `laps_completed` ≥ `laps`; open curves when
  `output.settled`. The follower has no goal yaw, so ROTATE_TO_HEADING is skipped.
- Laps: at goal start $\vartheta_0 = s^\star/L$ and $\pi_\ell = 0$; each cycle
  $\pi_\ell \mathrel{+}= \mathrm{dir}\cdot\operatorname{wrap}_{1}(s^\star/L - \vartheta_{\mathrm{prev}})$ with
  $\operatorname{wrap}_1$ into $(-\tfrac12, \tfrac12]$; `laps_completed` = $\lfloor \pi_\ell \rfloor$.
- `remaining_arc_length`: closed with laps > 0: $(\mathrm{laps} - \pi_\ell)L$; laps = 0: $+\infty$;
  open: $\sigma$.

(sec-collision)=

## 10 Collision prediction

(sec-footprint)=

### 10.1 Footprint

From the robot-frame polygon: $w_{\mathrm{lat}} = \max\lvert y\rvert$ over its vertices (robot_radius
if circular), $r_{\mathrm{insc}}$ = distance from base_link to the nearest edge, $r_{\mathrm{circ}}$ =
distance to the farthest vertex.

**Ground truth** [Spec]: a pose collides iff some site cell's closed square intersects the filled
footprint polygon.

**FootprintChecker** [Design]: with $c_p$ the cell containing $p$,
$D_{\mathrm{lo}} = \mathrm{EDT}_{\mathrm{all}}(c_p) - \lVert p - c_p\rVert$ (a lower bound on the
distance to every site centre, by the triangle inequality) and
$D_{\mathrm{hi}} = \lVert p - f(c_p)\rVert$ (the distance to an actual site):

- free if $D_{\mathrm{lo}} > r_{\mathrm{circ}} + \mathrm{res}$;
- collision if $D_{\mathrm{hi}} < r_{\mathrm{insc}}$;
- otherwise rasterise the filled polygon and test the ground truth.

Both shortcuts are sound: a site square can reach the circumscribed disc only if its centre is
within $r_{\mathrm{circ}} + \mathrm{res}/\sqrt2$; a site centre inside the inscribed disc lies inside
the polygon. Using the cell-centre EDT directly against $r_{\mathrm{circ}} + \mathrm{res}$ would not
be sound (it can be up to $\mathrm{res}/\sqrt2$ optimistic). A pose is **off-grid** when the disc of
radius $r_{\mathrm{circ}} + \mathrm{res}$ leaves the grid.

(sec-rollout)=

### 10.2 Rollout and free arc length

`CollisionPredictor` rolls out the private `stepNominal` on copies of the state with
$\gamma_{\mathrm{col}} = 1$, zero-order hold at the control period, time advancing by $\Delta t$ per
step, and moving curve frames extrapolated with the input frame twist. The first step uses the
cycle's measured $\Delta t$, so reusing it as the final command is exact; later steps hold each
command for the nominal period $1/f$ [Spec, M1]. Each step integrates the
nominal plant exactly (no slip) [Derived]:

$$
p(\tau) = p + R(\psi)\Big(\frac{\sin\omega\tau}{\omega} I + \frac{1 - \cos\omega\tau}{\omega} J\Big)\begin{bmatrix} v_x \\ v_y\end{bmatrix},
\qquad \psi(\tau) = \psi + \omega\tau ,
$$

with the limit $\tau I$ as $\omega \to 0$; unicycle and bicycle are $v_y = 0$, the bicycle integrated at
the rear axle and mapped to base_link by the fixed offset. Each step is sub-sampled so that checked
poses differ by at most one cell in translation and by at most $2\arcsin(\mathrm{res}/(2 r_{\mathrm{circ}}))$
in yaw (the chord of a point at $r_{\mathrm{circ}}$ is then at most res).

The rollout stops at the first colliding pose, at the horizon $T_h$, at settled, when the footprint
goes off-grid, or at STOPPED, except during a cancel: while `input.cancel` is set it continues through
the STOPPED ramp until the command is below 0.01 m/s and 0.01 rad/s. A settled step is integrated
before the rollout ends; settled ends a rollout only outside rotation phases, so ROTATE_TO_HEADING
still checks its swept rotation; a cancel ramp may run past the horizon, by at most 10 s
[Spec, M1].

$$
s_{\mathrm{free}} = \begin{cases}
+\infty & \text{no checked pose collides} \\
\text{base arc length to the last collision-free checked pose before the first colliding one} & \text{otherwise}
\end{cases}
$$

It is 0 if the first collision is at the start pose or during rotation only; if the rollout goes
off-grid, it is the arc to the last on-grid pose. *Rationale*: measuring to the first colliding pose
over-estimates the contact distance by up to one step, so the robot would brake inside the margin;
a finite $s_{\mathrm{free}}$ for collision-free rollouts would throw in free space near every goal
and on every long rotation.

(sec-collision-cap)=

### 10.3 Braking cap and final command

$$
v_{\mathrm{cap}} = \sqrt{2 a_{\mathrm{dec}}\, \max(0,\ s_{\mathrm{free}} - m_s)}\ \ (+\infty\ \text{if}\ s_{\mathrm{free}} = +\infty),
\qquad \gamma_{\mathrm{col}} = \min(1,\ v_{\mathrm{cap}}/v_r).
$$

If the cap binds, `stepNominal` is re-run once with $\gamma_{\mathrm{col}}$ in $\gamma$; otherwise the
rollout's first step is reused. The cap never feeds the next cycle's rollout, so it cannot oscillate.
STATUS_COLLISION_AHEAD is set when $\gamma_{\mathrm{col}}$ or $\gamma_{\mathrm{cl}}$ is the binding term
of $\gamma$ [Spec], and on a cancel collision. `use_collision_detection: false` skips the rollout,
the cap and kBlocked except the current-pose test.

(sec-collision-blocked)=

### 10.4 kBlocked

- **Set** when the current pose collides; or, in a translation phase (FOLLOW, FINAL_APPROACH),
  when $s_{\mathrm{free}} < m_s + v_{\mathrm{stop}}^2/(2 a_{\mathrm{dec}})$ (equivalently
  $v_{\mathrm{cap}} < v_{\mathrm{stop}}$); or, in a rotation phase, when the rollout collides before
  leaving the rotation phase it started in (the remaining swept rotation collides). Never set in a
  translation phase while settled, holding the goal, or with XY reached and the cap binding
  [Spec, M1].
- **Clear** (translation) only when $s_{\mathrm{free}} \ge m_s + v_{\mathrm{stop}}^2/(2 a_{\mathrm{dec}}) + h_b$,
  i.e. $h_b$ above the set threshold [Design, M1]; (rotation) when the swept remaining rotation is
  free. For Ackermann $v_{\mathrm{stop}} = v_{\mathrm{ack}}$, and $v_{\mathrm{ack}}^2/(2a_{\mathrm{dec}})$ can
  exceed $h_b$ (0.09 m at $v_{\mathrm{ack}} = 0.3$, $a_{\mathrm{dec}} = 0.5$); a release at $m_s + h_b$
  would then sit below the set threshold and the flag would toggle every cycle.
- *Rationale*: the band between $v_{\mathrm{cap}} < 0.02$ and a velocity-based release is under 1 mm
  ($v_{\mathrm{stop}}^2/(2a_{\mathrm{dec}}) = 0.2$ mm), so one cell of sensor flicker would toggle the
  flag, refresh the server's patience timer every other cycle and let the robot creep;
  $h_b = \max(2\,\mathrm{res}, 0.05)$ is a real band.
- Settled precedence ({ref}`sec-phases-table`) applies. kBlocked is not evaluated in STOPPED.

(sec-math-validation)=

## 11 Stability advisor and validation rules

`validateConfig(CoreConfig, optional<CostmapInfo>)` returns errors and warnings. Rules involving
$\lambda$, $D_{\mathrm{in}}$, $D_{\mathrm{in}}'$ or the lookahead apply only with `obstacle_mode: contour`.
$\Delta t = 1/f$, $W, H_c$ = costmap width and height in metres.

**Auto values.**

| Parameter | Auto value | TB3 (robot_radius 0.22, $d$ 0.2, $v_r$ 0.4) |
|---|---|---|
| $\lambda$ | $\max\big(\sqrt{(w_{\mathrm{lat}} + 0.10)^2 + d^2},\ r_{\mathrm{circ}} + d + 0.05\big)$ | $\max(0.377, 0.47) = 0.47$ |
| $D_{\mathrm{in}}$ | $\lambda + 0.15$ | 0.62 |
| $D_{\mathrm{in}}'$ | $\lambda + 0.35$ | 0.82 |
| `obstacle_lookahead` | $\max(1.0,\ v_r T_h)$ | 1.0 |
| `final_approach_distance` | FL: $\max(2d,\ \mathrm{xy\_tol} + d)$; otherwise xy_tol | 0.45 (xy_tol 0.25) |

The auto values pass every rule: $\lambda - d \ge r_{\mathrm{circ}} + 0.05 > r_{\mathrm{circ}} \ge r_{\mathrm{insc}}$.

**Reject (errors).**

| Rule | Reason |
|---|---|
| $v_r > v_{\max}$ | the field speed must be commandable |
| `min_in_place_angular_vel` > `rotate_to_heading_angular_vel` | the floor would exceed the rotate speed |
| not $\lambda < D_{\mathrm{in}} \le D_{\mathrm{in}}'$ | blend order; equality is the hard switch |
| $\lambda - d < r_{\mathrm{insc}}$ | with $p_c$ on the $\lambda$-set the base is at least $\lambda - d$ away; below $r_{\mathrm{insc}}$ the circulation itself collides |
| invalid (type, law) pair; Ackermann with contour | {ref}`sec-laws-points`, {ref}`sec-contour-notes` |
| Ackermann with $R_{\min} \le 0$ | the curvature clamp needs $R_{\min}$ |
| $\sigma_b + d + 0.1 > \tfrac12\min(W, H_c)$, or $>$ `window_limit` when set (Gen A/B prune_distance) | the goal must enter the window before braking must start |
| FL with $d \le 0$ [Spec] | $M(\psi)$ divides by $d$; the plan omits this rule (`upstream_audit.md`, item O3) |

**Warn.**

| Rule | Reason |
|---|---|
| `rotate_to_heading_angular_vel` > $\omega_{\max}$ | the effective rate is the minimum (the rotation shim declares 1.8) |
| $G'(0)\, v_r\, \Delta t \ge 1$ | discrete overshoot across the curve ({ref}`sec-gains`) |
| FL: $G'(0)\, d > 0.5$ | the field turns faster than FL's heading can follow (time constant $d/v_r$) |
| $\sigma_b < v_r^2/a_{\mathrm{dec}}$ | terminal braking exceeds $a_{\mathrm{dec}}$ ({ref}`sec-terminal-discrete`) |
| $\lambda - d < r_{\mathrm{circ}}$ | a corner of the footprint can reach the obstacle while $p_c$ is on the $\lambda$-set |
| $D_{\mathrm{in}}' + d \ge \tfrac12\min(W, H_c) - 0.2$ | the blend zone does not fit the rolling window |
| $T_h v_{\max} < v_{\max}^2/(2 a_{\mathrm{dec}}) + v_{\max}(2/f + 0.1) + 0.1$, or $T_h v_{\max} > \tfrac12\min(W, H_c) - r_{\mathrm{circ}}$ | the horizon must cover the stopping distance plus two cycles and 0.1 s of latency plus 0.1 m, and stay in the window |
| Ackermann: $R_{\min}(c_G k_f + \kappa_{\mathrm{path}}) > 1$ | the field's integral curves need more curvature than the car has |
| $v_r (c_G k_f + \kappa_{\mathrm{path}}) > \omega_{\max}$ | yaw rate needed to follow the integral curves at $v_r$ |

At configure, $\kappa_{\mathrm{path}} := 0$; the per-plan checks below use the window's value [Spec].

**Runtime, once per plan.** An unsmoothed plan (criterion defined with the controller in M2);
FL with $\kappa_{\mathrm{path}}\, d > 1$; the goal entering the window with $\sigma_{\mathrm{end}} < \sigma_b$.

**Footprint- and costmap-dependent rules** reject at configure (plugin) using the geometry captured
then, and only warn after a later change; the follower warns once.

(sec-math-oracles)=

## 12 Test oracles

These become the gtests of M1. Items marked (M2) or (M5) belong to the controller or follower
packages; they are listed because they test math defined here.

**Default test configuration (TC)**, unless an oracle says otherwise: atan gain, $k_f = 2$, $v_r = 0.4$,
$d = 0.2$, $f = 20$ Hz, $v_{\max} = v_{y,\max} = 0.5$, $\omega_{\max} = 1.5$,
$a_{\mathrm{acc}} = a_{\mathrm{dec}} = 1.0$, $\alpha_{\max} = 3.2$, $k_\psi = 2$, $\sigma_b = 0.5$,
$v_{\min} = 0.05$, xy_tol 0.25, yaw_tol 0.25, $\varepsilon_v = 0.1$; footprint a 16-gon of
circumradius 0.22 ($r_{\mathrm{insc}} = 0.2158$, $w_{\mathrm{lat}} = r_{\mathrm{circ}} = 0.22$), res 0.05,
auto $\lambda, D_{\mathrm{in}}, D_{\mathrm{in}}'$ = 0.47, 0.62, 0.82, $m_s = 0.10$, $T_h = 1.5$ s.
"Random" means at least $10^4$ seeded samples. $\dot D$ means the instantaneous
$n^\top(\dot x - v_c)$ computed from the command mapped back through $M^{-1}$ (FL) or $R(\psi)$
(holonomic) before the acceleration stage, unless stated as a closed-loop measurement.

### 12.1 Geometry

- **O1 Orthogonality.** Random $x$ around random polylines (including wedges), built-ins and
  expressions: $\lvert n^\top T\rvert \le 10^{-12}$ and $\lvert\lVert T\rVert - 1\rvert \le 10^{-12}$.
- **O2 Degenerate points.** $x$ exactly at $P_0$, at an interior vertex and at $W_{\mathrm{back}}$; the
  one-pose polyline $[p, g]$ evaluated at $p$: $\lVert T\rVert = 1$ ($10^{-12}$), $n = 0$,
  $\lVert\Phi\rVert = v_r$ ($10^{-12}$).
- **O3 Hairpins.** 179° and 180° turns: everywhere in the wedge $\lVert\Phi\rVert = v_r$ ($10^{-12}$),
  $n^\top\Phi \le 10^{-12}$; STATUS_HAIRPIN set only for the 180° case.
- **O4 Wedge identity and continuity.** Random strict wedges:
  $\operatorname{sgn}((J n)^\top(t_{v-1} + t_v)) = \operatorname{sgn}(t_{v-1}\times t_v)$ (exact). Points
  $10^{-7}$ m either side of a wedge boundary: $\lVert T_{\mathrm{in}} - T_{\mathrm{out}}\rVert \le 10^{-6}$.
- **O5 Semi-infinite rays.** A point 100 m beyond the goal along the forward ray and 100 m before
  $P_0$: the foot is on the ray (not clamped), $\sigma = -100$ m resp. the arc is $-100$ m
  ($10^{-9}$).
- **O6 Inner-corner jump.** Turns of 60°, 90°, 120°, 150° with a point on the inner bisector at
  $D = 0.5$: the two feet differ in arc length by $2D\tan(\theta/2)$ ($10^{-12}$); with a hint on the
  incoming segment, a point $10^{-6}$ m past the bisector moves $s^\star$ to the outgoing segment.
- **O7 Circle.** `ellipse` $a = b = 1.3$ centred at $(0.4, -0.2)$: $x^\star = c + R\operatorname{unit}(x - c)$
  and $D = \lvert\lVert x - c\rVert - R\rvert$ within $10^{-9}$; $n^\top\Phi = -v_r G(D)$ and
  $T_d^\top\Phi = v_r H(D)$ within $10^{-9}$.
- **O8 Tie at start.** Open polyline whose last point equals its first, no hint, robot at $P_0$:
  $s^\star = 0$ for $\mathrm{dir} = +1$ and $s^\star = L$ for $\mathrm{dir} = -1$.
- **O9 Detour test.** U-turn, legs $w_u = 0.8$ m apart, semicircular end, follower Hinted search: robot
  $2.5\,w_u$ before the apex, 0.45 m off leg 1 toward leg 2: $s^\star$ stays on leg 1. Robot $0.5\,w_u$
  before the apex, more than $w_u/2 + h_g$ off leg 1: the jump is adopted and STATUS_AMBIGUOUS_CLOSEST
  set. With feet at the same station $x$ before the apex, the detour inequality alone holds iff
  $x \le (5 - \pi/2)w_u/2$.
- **O10 Lemniscates.** Gerono ($a = 4$, $b = 1.5$) and Bernoulli ($a = 2$) under HT and FL, with 1–5 cm
  lateral noise injected near the crossing: over 5 laps the visited lobe alternates every
  half-period (no repeated lobe).
- **O11 Regularity rejects.** Lissajous 1:2 with phase $\pi/4$; $x = s^2$, $y = s^3$ on $[-1, 1]$; an
  ellipse expression on $[0, \pi]$ marked closed; superellipse $n = 1.5$; lissajous $f_x = 2$, $f_y = 4$
  [Spec]: all INVALID_CURVE. Every built-in at its defaults passes.
- **O12 Expression rejects** (also libFuzzer seeds): `cos(s)+rnd()`, `a=a+1e-4, a*cos(s)`,
  `cos(s), sin(s)`: INVALID_CURVE.
- **O13 Built-in formulas.** For each built-in at its defaults, $r(0)$ and $r(\pi/2)$ match the
  formulas of {ref}`sec-builtins` within $10^{-12}$; `rounded_rectangle` and `regular_polygon` start at
  the $+x$ side midpoint, are continuous (max step over $2\cdot10^5$ samples below $2L/(2\cdot10^5)$)
  and have the stated $L$ ($10^{-9}$). Superellipse $n \in \{2, 4, 8\}$: speed $\ge \rho$ ($10^{-9}$).
- **O14 Cusp decimation (M2).** NavFn-style tails (half-cell spacing toward the rounded goal cell, then
  `smoothApproachToGoal`'s replace-or-append rule; 8 goal offsets of ±0.5 cell × 16 headings ×
  approach lengths 1.0, 1.0125, 1.025, 1.0375 m; res 0.05 and 0.10): no cusp. A real reversal longer
  than $2\ell_c$ is flagged in every alignment (0.3 m at res 0.05; 0.45 m at res 0.10).

### 12.2 Gains

- **O15 Gain identities.** Both gains on $k_f D \in [0, 50]$: $G(0) = 0$, $G$ strictly increasing,
  $\lvert G^2 + H^2 - 1\rvert \le 10^{-15}$, $G'(0)$ as tabulated ($10^{-12}$), $G(G^{-1}(y)) = y$
  ($10^{-12}$).
- **O16 $c_G$.** The maximum of $GG'/H$ over a $10^6$-point grid of $k_f D \in (0, 20]$ equals
  $0.18557\,k_f$ (atan) and $0.38490\,k_f$ (rational) within $10^{-4} k_f$. The curvature of a static
  field's integral curve near a straight path, integrated by RK4, never exceeds $c_G k_f + 10^{-3}$.

### 12.3 Distance field

- **O17 Unit norm.** Random $x$, static fields, both gains, all curve types: $\lvert\lVert\Phi\rVert - v_r\rvert \le 10^{-12}$.
  Time-varying fields with $\lVert\Phi_T\rVert$ up to $0.999(1 - \varepsilon_v)v_r$: the same.
- **O18 Lemma 8.** Random time-varying samples: $v_r - \lVert\Phi_T\rVert - 10^{-12} \le \eta \le v_r + \lVert\Phi_T\rVert + 10^{-12}$
  and $\dot D = -\eta G$ ($10^{-12}$).
- **O19 Assumption-3 clamp.** $\lVert\Phi_T\rVert_{\mathrm{in}} = 1.2\,v_r$: clamped to $(1 - \varepsilon_v)v_r$
  ($10^{-12}$), kAssumption3Clamped set, $\eta \ge \varepsilon_v v_r$.
- **O20 Translating circle.** A circle of radius 1.5 m translating at $u = (0.1, 0.05)$ m/s:
  $\Phi_T = (I - TT^\top)u$ ($10^{-9}$); single integrator $\dot x = \Phi$ with no limits, step
  $10^{-3}$ s, from $D_0 = 0.5$: $D < 10^{-3}$ after 20 s (about $8\cdot10^{-5}$ in the reference
  computation; ZOH at 20 Hz leaves about 3 mm, so the step matters).
- **O21 Rigid moving frame.** $R_{\mathcal F} = R(\pi/2)$, $p_{\mathcal F} = (1, 2)$,
  $v_{\mathcal F} = (0.1, 0)$ in $\mathcal F$, $\omega_{\mathcal F} = 0.3$: $v_c$ equals the central
  difference ($\delta = 10^{-5}$ s) of the world point $p_{\mathcal F}(t) + R_{\mathcal F}(t)\, r_{\mathcal F}(u^\star)$
  within $10^{-6}$.
- **O22 Ultimate bound (5.4).** Single integrator $\dot x = \Phi + \delta$ on a static circle of
  radius 1 m with $\delta = 0.1\, n$ (worst case, away from the curve), integrated with step
  $10^{-3}$ s: after 20 s,
  $D \le \tan(\pi/2 \cdot 0.25)/k_f + 10^{-3} = 0.2081$ m.
- **O23 FrameTwistEstimator (M5).** A constant twist $(0.1, 0)$ m/s, 0.3 rad/s sampled at 20 Hz with
  yaw starting at 3.1 rad: the estimate equals the twist within $10^{-9}$ after settling, including
  across the $\pm\pi$ crossing, with no spike.

### 12.4 Terminal field

- **O24 ZOH never crosses.** TC, FL, straight polyline, start 2 m before the goal with lateral offsets
  0 and ±0.3 m: $\sigma_{\mathrm{base}} > 0$ on every cycle and settled fires within 15 s.
- **O25 Monotone speed.** Same runs: once $\sigma_{\mathrm{base}} \le \sigma_b$, the commanded $\lvert v\rvert$
  never increases by more than $10^{-9}$ from one cycle to the next, apart from the $v_{\min}$ floor and
  one cycle of acceleration lag at the FINAL_APPROACH switch.
- **O26 Floor.** $\varphi$ is odd; $\lvert\varphi(\sigma)\rvert \ge v_{\min}/v_r$ for
  $\lvert\sigma\rvert \ge \sigma_{\mathrm{stop}}$; Lipschitz constant $L_\varphi$ ($10^{-12}$ on a
  $10^5$-point grid), for xy_tol 0.25 and 0.05.
- **O27 Reference shift.** Ackermann with `reference_point_offset` ∈ {−0.3, 0, +0.3} on a straight
  approach: speed non-increasing over the last $\sigma_b$ (allowing the $v_{\mathrm{ack}}$ floor), final
  $\lVert p - g\rVert \le 0.6\,\mathrm{xy\_tol}$, settled.
- **O28 Final approach.** Goals at 60° and 90° from the arrival heading, and a lateral 0.3 m goal, for
  $d \in \{0.1, 0.3\}$ and every law: ROTATE_TO_HEADING starts with $\lVert p - g\rVert \le 0.6\,\mathrm{xy\_tol}$;
  no $v < 0$. Omni with `fixed` keeps its heading ($\lvert\Delta\psi\rvert < 10^{-9}$); diff with
  `drive_backward` arrives without a 180° turn.

### 12.5 Contour

- **O29 EDT.** Random grids up to 60×60: squared distances equal brute force exactly (integer cell
  units); each feature is a site at that distance.
- **O30 Bilinear ripple.** Contour along a straight axis-aligned wall: $T_\lambda$ direction ripple
  < 0.5°.
- **O31 Trap.** Disc centred on a straight path with $k_f(R_o + \lambda) \in \{1, 2.5, 4\}$: the unlatched
  rule (3.67) traps in the last two (base_link never passes the disc's far side within 20 s); the
  latched rule passes all three and rejoins the path ($D < 0.05$), with at most one ROTATE_TO_FIELD
  entry per episode.
- **O32 Start predicate.** No episode starts in a cycle with $n_o^\top\Phi \ge 0$; in the band
  $(D_{\mathrm{in}}', D_{\mathrm{in}}' + h_c]$ the command equals the no-contour command ($10^{-12}$).
- **O33 Clearance.** Disc on the path: distance from the evaluation point to the disc's cells stays
  $\ge \lambda - \mathrm{res}$, with a minimum $\le \lambda + 2\,\mathrm{res}$; steady circulation radius
  $R_o + \lambda \pm \mathrm{res}$.
- **O34 Identity.** Box with a parallel wall 0.45 m from the path: $K$ is the box; wall clearance
  $\ge$ path clearance − 0.05.
- **O35 Side probe.** Box centred on the path, wall gaps 0.3, 0.6, 0.71 (about 0.02 m wall-side
  clearance) and 0.8 m, wall on either side, ±1 cm initial error: the open side is chosen in every
  run, one episode, no kBlocked, no $n_o$ reversal.
- **O36 K tracking.** Rolling-origin shifts of 1–10 cells per cycle keep $K$ and $\sigma_c$; a box
  removed mid-episode ends the episode on the first grid without it.
- **O37 Goal veto.** A 0.3 m disc on an open curve's end: no episode, stop
  $\ge m_s - \mathrm{res}$ short. A free goal 0.3 m behind a 0.2 m box: one episode, released by the
  gate, goal reached. A cap stopping the robot inside xy_tol: settled, never kBlocked.
- **O38 Watchdog.** With `always`, a perpendicular wall longer than the limit trips kContourWatchdog
  at the same travelled distance (±1 cell) on a fixed curve and on re-pruned windows.
- **O56 Dead end** (`path_blocked`). A straight path into a 1.2 m corridor closed by a wall across it,
  and the O38 wall: no episode starts, the robot stops $\ge m_s - \mathrm{res}$ short of the wall
  (and within $3 m_s$), never collides, and kBlocked, once set, stays set.

### 12.6 Laws

- **O39 FL identity.** Random $\psi$, $F$, $q$, $d > 0$: $\lVert v\mathbf h + q d\omega J\mathbf h - F\rVert \le 10^{-12}$.
- **O40 HT feedforward.** HT on circles of $R \in \{0.5, 1, 2\}$ discretised at 2.5 and 5 cm: steady
  $\lvert D\rvert < 0.01$ m, and $\dot\psi_d$ is zeroed on 0% of cycles. On a smooth expression circle,
  unsaturated: $\lvert\dot\psi_d - \nabla\psi_d^\top(v\mathbf h)\rvert \le 10^{-3}$ rad/s.
- **O41 Jump zeroing.** Probes straddling U-turn legs or lemniscate branches: $\dot\psi_d = 0$.
- **O42 Rotate to field.** FL corners of 60°, 90°, 120°, 150°, 170°, and closed triangle, square and
  pentagon with 2 m sides: at most one ROTATE_TO_FIELD entry per corner and transit time at most 1.2×
  the run without the phase; a U-turn with $R = 0.15$ m completes. Goals 120° and 180° behind: no
  $v < 0$.
- **O43 Ackermann.** $R_{\min} = 0.5$, start at $e = 85°$ on a straight path: converges with no
  zero-speed stall.

### 12.7 Limiter

- **O44 $\eta$ lower bound.** Pure CommandLimiter, FL, $d = 0.2$, $v_{\max} = 0.5$, $\omega_{\max} = 1.5$,
  $v_r = 0.4$, $\psi = 0$, $\Phi_S = (0.97022, 0.24224)$ (atan, $k_f = 2$, $D = 0.2$), $\Phi_T = (0, 0.35)$:
  $\eta_1 = 0.126613$, the $\omega$-interval is $[-2.68331, -0.20641]$, the set is empty, uniform
  $c = 0.788083$, box output (before the acceleration stage) $v = 0.096809 \ge 0$, $\omega = 1.5$,
  kAssumption3Clamped set (all $10^{-6}$).
- **O45 $\eta$-box keeps $\dot D \le 0$.** Translating circle (0.1 m/s) under FL with $\omega_{\max}$
  lowered until $\omega$ binds: $\dot D \le 10^{-12}$ and no kAssumption3Clamped while bound.
- **O46 Omni.** Translating circle, `path_tangent`, 180° initial heading error:
  $\lvert\omega\rvert \le \omega_{\max}$, no kAssumption3Clamped while only $\omega$ saturates,
  $\dot D \le 10^{-12}$.
- **O47 Contour with a moving curve.** Translating circle over a static box, FL and holonomic: during the
  episode the command stays in the box, $\lVert v_{xy}\rVert \le \gamma v_r + 10^{-12}$, no
  kAssumption3Clamped.
- **O48 Acceleration stage.** Random $u^\ast$, $u_{\mathrm{prev}}$: every axis change is within its
  bound ($10^{-12}$) and $u - u_{\mathrm{prev}}$ is parallel to $u^\ast - u_{\mathrm{prev}}$ ($10^{-12}$).
  With translation braking (target $(0, 0, \omega^\ast)$): every axis within its bound, the
  translation is $(1 - \alpha_{\mathrm{lin}})$ times its previous value with the largest admissible
  $\alpha_{\mathrm{lin}}$, and $\omega$ takes the largest admissible step on its own [0.1.1].

### 12.8 Phases, cancel and collision

- **O49 Cancel.** Ackermann at 0.3 m/s with speed_limit_scale 0.1 reaches $\lvert v\rvert < 0.01$ within
  $v/a_{\mathrm{can}} + 2$ cycles; skid in ROTATE_TO_FIELD with floor 0.3 reaches $\lvert\omega\rvert < 0.01$
  within $\omega/\alpha_{\max} + 2$ cycles.
- **O50 Cancel with an obstacle.** $a_{\mathrm{can}} = 0.3$, $a_{\mathrm{dec}} = 1.0$, cancel at 0.4 m/s on a
  straight; one cycle later a box appears 0.2 m ahead of the footprint front (inside the 0.267 m gentle
  stopping distance, outside the hard one plus a cycle): the robot stops with the footprint free,
  STATUS_COLLISION_AHEAD set, $\lvert v\rvert < 0.01$ reached, no throw.
- **O51 kBlocked hysteresis.** A lethal edge flickering ±1 cell every cycle in front of a blocked robot:
  kBlocked stays set and the commanded speed stays 0.
- **O52 Free arc length.** Straight approach to a box at 0.4 m/s: $s_{\mathrm{free}}$ is within one
  sub-step (≤ res) below the analytic last-free arc, never above it; the robot stops
  $\ge m_s - \mathrm{res}$ short. Collision-free rollouts give $+\infty$; a rotation that sweeps into a
  wall gives 0.
- **O53 Footprint shortcut soundness.** Random grids, footprints and poses: shortcut-free ⇒ ground truth
  free; shortcut-collision ⇒ ground truth collides.
- **O54 Settled.** Inside xy_tol: settled iff one of its three clauses holds; ROTATE_TO_HEADING is never
  entered without settled.
- **O55 SteadyStateNoAlloc.** Separate executable with counting `operator new`/`delete`, not under
  ASan/TSan: after 10 warm-up cycles, 1000 steps with polyline and built-in curves, a fixed grid,
  contour active and the rollout on, including HT (FieldProbe path): zero allocations.

**Scenario tests** of plan §3.5 not repeated above (perpendicular wall, superellipse runs, skid slip,
upstream regressions `UpstreamRegression.U08_ShortPathIndexError` etc.) use the assertions stated
there and the predicates defined in this document.

(sec-math-limits)=

## 13 Known limits

- **Self-intersecting curves** violate Assumption 1. Lemniscates work through the hinted local search
  and the adoption rule, with no proof; near the apex of a U-turn a jump to the other leg is accepted,
  as at any inner corner.
- **Open curves** are made borderless by rays. The terminal-field argument ({ref}`sec-terminal-convergence`)
  is ours, for static curves only; open time-varying curves are rejected.
- **Reach.** Convergence needs a unique closest point along the trajectory. Polyline inner corners
  have zero reach at the vertex; the field crosses the bisector transversally, but the closest point
  jumps.
- **Polylines are only $C^0$**, rounded rectangles and regular polygons only $C^1$: the wedge tangent
  and piecewise $\Omega$ handle them, but $\dot\psi_d$ has one-cycle spikes at inner bisectors, bounded
  by the clamp.
- **The obstacle contour has no proof.** The latch removes the trap basin of {ref}`sec-contour-trap`, but
  gaps narrower than about $2\lambda$ act as closed, obstacles with no velocity information are treated
  as static, and the watchdog bounds every episode. Ackermann has no contour in 0.x.
- **Saturation.** Static fields under uniform scaling keep $\dot D \le 0$; time-varying fields keep it
  under the $\eta$-box but not under its fallback; HT and Ackermann only as $e \to 0$; the acceleration
  stage makes all of these approximate.
- **Assumption 3.** Once $\Phi_T$ is clamped, convergence is not guaranteed; the follower aborts after
  `assumption3_abort_time`.
- **Heading tracking under saturation.** With a time-varying field and uniform $c$, the feedforward
  cancellation is approximate.
- **Discretisation.** Costmap distances are to cell centres (errors up to $\mathrm{res}/\sqrt2$, covered
  by the shortcut margins); the rollout uses the nominal plant with no slip; ZOH adds
  $O(\Delta t^2)$ errors to every continuous-time statement.
- **Cusps and reversing** are rejected until M7; reversals shorter than $\ell_c$ are followed forward
  with a deviation below $\ell_c$.

(sec-math-viz)=

## Appendix A Quantities exported for visualization

Every RViz/Foxglove marker in {doc}`../visualization` is drawn from `FieldSample`, `TrackerState` and
`StepScratch` after `step()`, in the control frame. The quantities are:

| Quantity | Definition |
|---|---|
| $x_{\mathrm{eval}}$ | evaluation point $x$ ({ref}`sec-laws-points`); the `control_point` marker |
| $x^\star$, $T_d$, $n$, $D$, $\sigma$ | closest point, field tangent, normal, distance, remaining arc ($\sigma_{\mathrm{base}}$; NaN for closed curves) |
| $G$, $H$, $\eta$ | gains at $D$ and the scale actually used ($\gamma v_r$ for static fields, $\eta_1$ or $\eta^\ast$ for time-varying) |
| field arrows | convergence $-\eta G\, n$; circulation $\eta H\, T_d$ (terminal field: $\gamma v_r H\varphi\, T_d$); feedforward $\Phi_T$; result $F$ (post-contour, before the kinematic box). The first three sum to the shaped path field $\Phi_\gamma$. |
| $g$, $g_{\mathrm{ref}}$ | goal and shifted goal ({ref}`sec-terminal-shift`); rings of radius xy_tol and $\sigma_{\mathrm{stop}}$ about $g$ |
| braking zone | the curve stations with $0 \le \sigma \le \sigma_b$, i.e. $s$ between $s_{\mathrm{goal}} - \mathrm{dir}\,\sigma_b$ and $s_{\mathrm{goal}}$ |
| command arc | poses $p(\tau), \psi(\tau)$ of {ref}`sec-rollout` for $\tau \in [0, 1]$ s under the published $(v_x, v_y, \omega)$ |
| heading target | ROTATE_TO_FIELD: $\angle F_{\mathrm{trk}}$ (+π if `drive_backward`); ROTATE_TO_HEADING: the goal yaw; HT: $\psi_d$; the arc spans $\operatorname{wrap}(\psi_{\mathrm{tgt}} - \psi)$ |
| prediction | rollout poses from `StepScratch`, the first colliding pose, $s_{\mathrm{free}}$ |
| contour | gate open, episode active, $\sigma_c$, $x_o^\star$, $D_o$, $\theta$, `travelled_m`, `elapsed_s`, the blocked gate samples, $K$'s cells; the $\lambda$, $D_{\mathrm{in}}$, $D_{\mathrm{in}}'$ rings are centred at $x_o^\star$ [Spec]; the side arrow is $\sigma_c J n_o$ at $x_{\mathrm{eval}}$ |
| `field_grid` arrows | $F$ at each grid node through the `FieldProbe` (frozen hint, $K$ and gate), length $\propto\lVert F\rVert$, colour by $D$. Nodes far from the robot show the field of this cycle's search state, which is what the controller would use from there only approximately. `Tracker::probe()` shapes with the speed limit $\gamma_{\mathrm{lim}}$ only; the cycle's collision cap is not stored [Spec, M1]. |
| streamlines | RK4 integration of $\dot x = F(x)$ at the frozen $t$ with step `field_grid_resolution`/2 [Spec] |

(sec-math-constants)=

## Appendix B Numerical verification of stated constants

Computed with `python3 -I` (pure Python, double precision) while writing this document:

| Quantity | Result |
|---|---|
| $c_G$ atan: $\max_u \frac{2}{\pi}\arctan u \cdot \frac{2}{\pi}\frac{1}{1+u^2} / H$ | 0.185575 at $u = 0.8787$ |
| $c_G$ rational | 0.384900 numerically at $u = 0.70711$; $2/(3\sqrt3) = 0.384900$ |
| $G'(0)$ atan coefficient | $2/\pi = 0.636620$ |
| trap half-widths, atan, $k_f\rho$ = 1.571 / 1.6 / 1.8 / 2.5 / 3.0 / 4.0 | 0 / 8.55° / 22.86° / 40.38° / 46.55° / 54.02° |
| trap half-widths, rational, $k_f\rho$ = 0.9 / 1.1 / 2.5 / 4.0 | 0 / 24.62° / 66.42° / 75.52° |
| wedge identity, $2\cdot10^5$ random strict wedges | 0 sign mismatches; $T = t_{v-1}$, $t_v$ on the boundaries to 0 |
| inner-corner jump at 60/90/120/150°, $D = 0.5$ | 0.57735 / 1.0 / 1.73205 / 3.73205 $= 2D\tan(\theta/2)$ |
| U-turn acceptance limit | $(5 - \pi/2)/2 = 1.7146$ |
| inner-corner detour limit, $\tan(\theta/2) = 3\sin(\theta/2) + 2$ | $\theta = 157.11°$ |
| O20 reference run (step $10^{-3}$ s / ZOH at 20 Hz) | $D = 8.3\cdot10^{-5}$ / $3.0\cdot10^{-3}$ m after 20 s |
| O22 reference runs (Euler, $10^{-3}$ s, three starts) | $D$ = 0.20692–0.20731 m after 20 s; bound 0.20711 m |
| TB3 auto values | $\sqrt{0.32^2 + 0.2^2} = 0.3774$; $\lambda = 0.47$; $D_{\mathrm{in}} = 0.62$; $D_{\mathrm{in}}' = 0.82$; $T_{\mathrm{side}} = 4.566$ s; $\arcsin(d/\lambda) = 25.18°$ |
| O44 example | $G = 0.242238$, $H = 0.970217$, $\eta_1 = 0.126613$; $\omega$-interval $[-2.683313, -0.206409]$; $c = 0.788083$; $v = 0.096809$; the unbounded rule gives $\eta = -0.206409$, $v = -0.200261$, $\dot D = +0.05$ |
| Lemma 8 and $\dot D = -\eta G$, $2\cdot10^5$ random samples | bounds hold; max $\lvert\lVert\Phi\rVert - v_r\rvert = 3.3\cdot10^{-16}$ |
| terminal ZOH factor, required deceleration | 0.96; 0.32 m/s² |
| floor binding, xy_tol 0.25 / 0.05 | no ($L_\varphi = 2.0$) / yes ($L_\varphi = 5.0$) |
| closed-loop FL terminal approach (TC, offsets 0 and ±0.3 m) | $\min\sigma_{\mathrm{base}} = 0.122$ m > 0; settled at 5.65–5.7 s; no speed increase after $\sigma_b$ |
| regularity of built-in defaults ($M = 400$) | all pass; lissajous phase $\pi/4$, $x = s^2, y = s^3$ and the closed half-ellipse are rejected; lissajous 2:4 passes (hence the gcd rule) |
| Bernoulli speed | $a/\sqrt{1 + \sin^2 s}$ (matches to $4\cdot10^{-12}$) |
| Gerono $a = 4$, $b = 1.5$ crossing angle | 106.3° |
| exact SE(2) step vs. $2\cdot10^5$-step quadrature | agreement to $10^{-14}$ |
| yaw sub-step bound | chord $2 r_{\mathrm{circ}}\sin(\Delta\psi/2) = \mathrm{res}$ exactly |

(sec-math-spec-choices)=

## Appendix C Details fixed by this document

The plan leaves these open; they are [Spec] above. All seventeen were reviewed and **confirmed on
2026-10-13** before M1; item 13 is implemented as `TrackerState::holding`. Change them only together
with the code and tests that depend on them.

1. Closest-point tie tolerance $10^{-9}$ m, tie-breaking toward the hint, and toward the smallest
   $\mathrm{dir}\cdot s$ at initialisation (answers `upstream_audit.md` item O2).
2. Parametric search numerics: $h_u$, one-sided end differences, golden-section tolerance, Gauss–Legendre
   order and panels, the edge-continuation of the bracket, and the re-orthogonalisation of $n$.
3. `lissajous` requires $\gcd(f_x, f_y) = 1$; `regular_polygon` requires $r_c \le R\cos(\pi/n)$.
4. Follower polyline conditioning order (dedupe, fillet, resample), its 1 mm dedupe and the fillet bound
   $R_\ell$.
5. $\partial_t r$ by central difference with $\delta_t = 10^{-4}$ s; $\partial_t\psi_d$ over $\pm\Delta t/2$;
   NaN probes and held $\psi_d$ zero $\dot\psi_d$.
6. Holonomic heading policies as pure feedback $k_\psi\operatorname{wrap}(\psi_{\mathrm{tgt}} - \psi)$;
   `fixed` means $\omega = 0$.
7. The rotate law's stopping term $\sqrt{2\alpha_{\max}\lvert e_r\rvert}$.
8. Contour start and exit tests use the nominal (unshaped) $\Phi$; the blend's degenerate fallback.
9. The clearance slowdown term $\gamma_{\mathrm{cl}}$.
10. "Cap binds" and STATUS_COLLISION_AHEAD defined through the binding term of $\gamma$;
    STATUS_AMBIGUOUS_CLOSEST on an adopted jump.
11. Acceleration bound selection by the sign of $u_{\mathrm{prev},i}\Delta_i$.
12. ROTATE_TO_FIELD needs $\lVert F_{\mathrm{trk}}\rVert \ge 0.05 v_r$; the transition order of
    {ref}`sec-phases-table`.
13. Goal hold uses the latch bit `TrackerState::holding`.
14. FootprintChecker's $D_{\mathrm{lo}}$/$D_{\mathrm{hi}}$ form and the ground-truth collision predicate.
15. Reject FL with $d \le 0$.
16. At configure, the curvature advisor rules use $\kappa_{\mathrm{path}} = 0$.
17. Visualization: contour rings centred at $x_o^\star$; streamline step.
