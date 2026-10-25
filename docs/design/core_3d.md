# The 3D core (experimental)

The thesis field is $n$-dimensional. The planar library uses $J$ (the 90° rotation) only in the
wedge and hairpin tangent rules. Every other equation, from $\Phi_S$ through $\eta$, $\Phi_T$
and the terminal field to the Lyapunov argument, holds in $\mathbb R^n$ unchanged. The 3D core
implements those equations for curves in $\mathbb R^3$. It also adds a holonomic
single-integrator law for multirotors, a limiter, a minimal tracker and a simulator. The
equations are in {doc}`math_spec` §14. The module is **experimental in 0.x**: its API may change
in a minor release.

**Packaging.** The 3D core ships in the same `vectorfield_core` library, in namespace
`vectorfield_core::d3`, with headers under `include/vectorfield_core/d3/` and sources under
`src/d3/`. We chose this over a sibling `vectorfield_core_3d` target for three reasons.

- The muParser objects are folded into the library with hidden visibility, and the expression
  hardening is internal (`detail`). A second library would need its own muParser copy, or an
  exported internal ABI.
- There is still one exported CMake target. The ROS packages, the wheel and `ament_export_*`
  are unchanged.
- The only cost is unused code in the shared library (about 70 kB of 615 kB in a macOS
  Release build).

The 2D umbrella `vectorfield_core.hpp` does not include the 3D headers. Code opts in with
`#include "vectorfield_core/d3.hpp"`. Type names carry a `3` suffix (`Vec3`, `Curve3`,
`Tracker3`, ...), so `using namespace` on both namespaces can never be ambiguous. Python exposes
the module as `vectorfield.d3`, where the suffix is dropped.

**Carried over unchanged** (shared, not copied):

- `GainFunction` ($G$, $H$, $G'$, $c_G$) and `TerminalProfile` ($\varphi$, $L_\varphi$);
- `Status` bits and `Phase`;
- `CurveOptions` (search window, global trigger, hysteresis, $M$) and `validateCurveOptions`;
- `CurveError` and its codes, `adoptGlobalCandidate`, `isTie`, `PieceKind`, `BuiltinShapeInfo`;
- `boxAxisInterval`, `OmniHeadingPolicy` (the yaw policy) and the numeric constants
  ($\varepsilon_D$, $\varepsilon_{\mathrm{hp}}$, $h_u$, $\delta_t$, the golden-section and
  regularity tolerances).

The muParser hardening moved from `src/expression_curve.cpp` to the internal
`src/expression_parser.{hpp,cpp}`, which both ExpressionCurve and ExpressionCurve3 use. The 2D
behaviour is identical, and its tests are untouched.

**New in 3D.**

- *Polyline wedge.* A vertex owns $x$ on a dihedral region, $a_1 > 0$ and $a_2 > 0$ as in 2D.
  The normal $n = (x - P_v)/D$ then ranges over a spherical lune, not an arc. In 3D the tangent
  is fixed only up to a rotation about $n$, so $s_w J n$ has no analogue. We use the blend
  $T = \operatorname{unit}((1-\alpha)t_{v-1} + \alpha t_v)$ with $\alpha = a_1/(a_1 + a_2)$. It is
  orthogonal to $n$ exactly in exact arithmetic, and it is re-projected anyway. It is continuous
  at the wedge boundaries, where it equals $t_{v-1}$ or $t_v$, and its norm is at least
  $\lVert t_{v-1} + t_v\rVert/2$. In a plane it reduces to the 2D rule (§14.2).
- *Hairpin.* When $\lVert t_{v-1} + t_v\rVert < \varepsilon_{\mathrm{hp}}$, the tangent is chosen
  from a fallback chain: $\Pi_n T_{\mathrm{last}}$, then $\Pi_n t_{v-1}$, then $\Pi_n e_k$ with
  $k = \arg\min_i \lvert n_i\rvert$. The search sets kHairpin.
- *Parametric search.* It has the 2D two-stage structure: a hinted golden-section bracket and an
  $M$-sample global stage with the same adoption rule. A Newton polish on
  $g(u) = r_u^\top(r - x)$ follows, at most 3 steps, each accepted only if $D$ grows by at
  most $10^{-12}$ m (rounding). The foot is therefore accurate to rounding, not only to the
  golden-section noise floor of about $\sqrt{\epsilon}$.
- *Built-ins.* `helix` (open), `circle` and `ellipse` (closed, in an oriented plane), `viviani`,
  `lemniscate3d` (Gerono lifted by $z = h\sin u$, which removes the double point) and
  `torus_knot` $(p, q)$ with $\gcd(p, q) = 1$. Every shape takes the pose parameters
  `cx cy cz yaw pitch roll`, applied as $R_z(\psi)R_y(\theta)R_x(\phi)$.
- *Law.* $v = F$ (holonomic, world frame) with box limits per axis and on $\lVert v\rVert$. Static
  and terminal fields use a uniform scale $c$, so the direction is kept. Time-varying fields use
  the $\eta$-box: the largest $\eta \in [0, \eta_1]$ inside every bound, with $\Phi_T$ kept, or
  the uniform fallback plus kAssumption3Clamped. Yaw follows the policy, with
  $\omega = \operatorname{clamp}(k_\psi \operatorname{wrap}(\psi_d - \psi), \pm\omega_{\max})$;
  $\psi_d$ is held while the horizontal part of its source vector is below $10^{-3}$.
- *Limiter.* One $\alpha$ over $(v_x, v_y, v_z, \omega)$, with the 2D sign rule per axis. STOPPED
  brakes the translation and the yaw rate independently. A non-finite command gives 0.
- *Tracker3.* FOLLOW; FINAL_APPROACH on open curves when $\sigma <$ the final-approach distance;
  settled (XY latch, then $\lVert p - g\rVert \le \sigma_{\mathrm{stop}}$) followed by goal hold;
  a cancel enters STOPPED. There are **no obstacles in this item**; 0.3.0 adds them (below,
  "Obstacles").
- *Simulator.* A point plant (ZOH, exact) with an optional first-order velocity lag $\tau$, the
  usual multirotor approximation, discretised exactly. `ClosedLoopSimulator3` records every
  step.

**Numerical edge cases.**

- *Exactly on the curve* ($D < \varepsilon_D$): $n := 0$, $T$ is the owning piece's tangent, or
  $\operatorname{unit}(t_{v-1} + t_v)$ at a vertex.
- *Medial set.* The axis of a circle, or the focal set of a helix where $\Omega \to 0$, has
  non-unique closest points. The search returns one sample. The field keeps unit norm there but
  is discontinuous. The tests probe inside the reach $1/\kappa_{\max}$.
- *Viviani's curve* has a double point at $(R, 0, 0)$. The hinted local stage keeps the branch,
  as it does for the planar lemniscates.

**Tests.** The 3D tests are the O1, O3 and O7-style oracles:

- $\lVert\Phi\rVert = v_r$ and $n^\top T = 0$ to $10^{-12}$;
- the helix's analytic foot;
- $\dot D = -v_r G$ on an inclined circle;
- $\Phi_T = \Pi_T u$ on a translating circle;
- the bounds $v_r - p_T \le \eta \le v_r + p_T$.

The closed-loop tests run a helix and Viviani's curve ($D < 0.01$ m after the transient, at
0.5 m/s and 20 Hz ZOH), and an open polyline that must settle at its goal. Further tests cover
the expression hardening (including `&&` and `||`), the regularity of every built-in at its
defaults, and a counted no-allocation run of `Tracker3::step`.

**Limitations.** Open curves must be static in their frame (moving frames below, 0.3.0).
Polylines are not resampled or filleted (the 3D projection is exact
on segments, so neither is needed). Tracker3 has no rotate phases, no collision rollout and no
ROS node.

## Moving frames [Design, 0.3.0]

The planar follower places a curve in a frame $\mathcal F$ that may move (`CurveFrame`,
{doc}`math_spec` §4.2). 0.3.0 adds the same to the 3D core. The equations are in
{doc}`math_spec` §14.7.

**Types.** `CurveFrame3{Isometry3 T_eval_curve; FrameTwist3 twist; bool moving}` in
`d3/frame.hpp`. `T_eval_curve` $= (R, c)$ maps curve-frame points to the evaluation frame. The
twist is the frame's *body* twist $(v_{\mathcal F}, \omega_{\mathcal F})$ in $\mathcal F$'s
axes, so $\dot R = R[\omega_{\mathcal F}]_\times$ and $\dot c = R v_{\mathcal F}$. The 3D
`Twist3` is the holonomic command $(v, \omega_z)$, so the six-component frame twist gets its own
type, `FrameTwist3{v, w}`. A point fixed in $\mathcal F$ moves at
$R(v_{\mathcal F} + \omega_{\mathcal F} \times r_{\mathcal F})$. A curve point also moves
with the curve's own time variation:
$\partial_t r = R(v_{\mathcal F} + \omega_{\mathcal F} \times r_{\mathcal F} +
\partial_t r_{\mathcal F})$.

**Where each quantity lives.**

- The *search* runs in $\mathcal F$, on $x_{\mathcal F} = R^\top(x - c)$. The hint (foot,
  curve parameter, segment) and $T_{\mathrm{last}}$ are stored in $\mathcal F$. A moving frame
  therefore never stales the hint: the curve does not move in its own frame. The planar tracker
  stores the hint in the evaluation frame and maps it each cycle. That is equivalent for a
  static frame, but in 3D the hint would lag by the frame's displacement over one period.
- The *field* is evaluated in the evaluation frame: $x^\star$, $T$ and $n$ are mapped by
  $(R, c)$; $D$, $s^\star$ and $\sigma$ are invariant. Commands, yaw, limits and the goal are
  in the evaluation frame.

**Closed curves** keep the planar design: $\Phi_T = \Pi_T v_c$ with $v_c = \partial_t r$ at the
foot, then the Assumption-3 clamp and $\eta$ of §4.3, so $\lVert\Phi\rVert = v_r$. Only the
normal motion of the curve is compensated. The tangential part only changes the progress along
the curve, which a closed curve does not constrain.

**Open curves** are rejected in the planar follower when the frame moves, because the terminal
field is proved only for static curves. In 3D an open curve must still be static *in*
$\mathcal F$, but the frame may move. The goal pins a station of $\mathcal F$, so the frame's
tangential motion matters near the goal. A tracker that compensated only the normal part would
settle behind or ahead of the goal. The tracker therefore *transports* the static terminal field:

$$
\Phi = v_r\big(-G n + H\varphi(\sigma) T_d\big) + v_{\mathrm{tr}}(x), \qquad
v_{\mathrm{tr}}(x) = R\,(v_{\mathcal F} + \omega_{\mathcal F} \times x_{\mathcal F}).
$$

$v_{\mathrm{tr}}$ is the velocity of the frame at the *body*, not at the foot. Then
$\dot x_{\mathcal F} = R^\top(\dot x - v_{\mathrm{tr}}(x))$ is exactly the static terminal
field in $\mathcal F$. For a single integrator without limits, every guarantee of §5 (convergence,
settling at the goal) holds relative to the frame. $\lVert\Phi\rVert$ is no longer $v_r$: the
relative speed is. The law uses the $\eta$-box with $a = \Phi_T = v_{\mathrm{tr}}$,
$b = \Phi_S$ and $\eta \in [0, \gamma v_r]$. A frame too fast for the box falls back to the
uniform scale and sets kAssumption3Clamped, as a fast time-varying curve does.

**Goal hold.** A static hold commands zero. In a moving frame that would drop the body off the
goal, so the hold commands the chord of the body's frame-fixed point over the next period:
$v = (T\exp(\Delta t\,\hat\xi)\,x_{\mathcal F} - x)/\Delta t$, with $\Delta t$ the
input's period. Under ZOH and a constant twist this is exact: the body ends each period where its
point of $\mathcal F$ is, so the hold does not drift. The yaw rate target is zero.

**Extrapolation.** `expSE3` is the closed form (Rodrigues for $R$, the left Jacobian $V$ for the
translation), with Taylor series below $10^{-2}$ rad. `extrapolate(frame, tau)` returns
$T\exp(\tau\hat\xi)$, exact for a constant twist. It is used by the goal hold above, by the
simulator, which places the frame at $T(t_0)\exp((t - t_0)\hat\xi)$ (no accumulated drift), and
by `FrameTwistEstimator3::estimate()`. Tracker3 has no rollout. A future rollout would sample the
frame with the same function, as the planar tracker does (`curveFrameAt`).

**Assumption-3 clamp with the frame speed.** The clamp acts on $\Pi_T v_c$ (closed curves) or
on $v_{\mathrm{tr}}(x)$ (open curves), and sets kAssumption3Clamped. For an open curve the
clamp is not needed for $\eta$ to exist. It bounds the command by $(2 - \varepsilon_v)v_r$ and
flags a frame that is too fast for the reference speed; the body then lags the frame. A
sufficient condition for no clamp anywhere on a curve is
$\lVert v_{\mathcal F}\rVert + \lVert\omega_{\mathcal F}\rVert\max_s\lVert
r_{\mathcal F}(s)\rVert < (1 - \varepsilon_v)v_r$: a rotating frame clamps first far from its
axis.

**Diagnostics.**

- `FieldSample3::v_c`: the curve velocity at the foot, frame motion included. For an open curve
  in a moving frame it holds $v_{\mathrm{tr}}(x)$ before the clamp.
- `FieldSample3::Phi_T` and Status::kAssumption3Clamped.
- `Tracker3Output::frame_velocity` (frame velocity at the body) and `x_star_curve` (the foot in
  $\mathcal F$).
- `SimulationRecord3::T_eval_curve` (the frame of each step; `frame` in the Python records).
- `FrameEstimate3::{ready, stale, age}` and `FrameTwistEstimator3::outliers()`.

**Static frames.** The default frame (identity, not moving) skips every mapping. Its results
are bit-identical to 0.2. This was checked by comparing 6017 records of five closed loops (helix,
Viviani's curve, a torus knot with lag, a moving expression and an open polyline) between the
0.2 and 0.3 builds. In the test suite, a moving identity frame with a zero twist reproduces the
frame-free records exactly up to the goal hold (whose limiter braking differs). A static
non-identity frame maps through $(R, c)$. The per-axis box and the limiter are not
rotation-invariant, so a rigidly displaced problem gives the same field on the first step but not
the same trajectory. A transform whose linear part is not a rotation to $10^{-6}$ is non-finite
input: zero target and kNonFinite.

**FrameTwistEstimator3** mirrors the follower's planar estimator in the core. It differences
consecutive pose samples with $\xi = \log_{SE(3)}(T_{k-1}^{-1}T_k)/\Delta t$, which is exact
for a constant twist with $\lVert\omega\rVert\Delta t < \pi$. Only the relative rotation is
logged, so an orientation crossing $\pm\pi$ needs no unwrapping. The rotation angle comes from
a quaternion, which is robust near $\pi$. It has the same `min_dt`, `max_dt`, duplicate and
old-stamp rules, an outlier test on $\lVert v\rVert$ and $\lVert\omega\rVert$, a first-order
low-pass on the six twist components (never on angles), and staleness after `input_timeout`. A
twist source (Odometry) takes the twist as given. `FrameEstimate3::curveFrame()` is moving iff
not stale.

**ZOH residual.** §14.3's $D_{\mathrm{zoh}}$ depends on the motion *relative to the curve*.
For a translating frame the relative speed is up to $v_r + \lvert T^\top u\rvert$, so
$D_{\mathrm{zoh}} \approx v_{\mathrm{rel}}^2\Delta t\,\kappa/(2\eta G'(0))$. On a 2 m circle
translating at 0.16 m/s we measure 8.0 mm, against 4.9 mm static. In a rotating frame the held
command turns at $-\omega$ relative to $\mathcal F$ during each period. This adds an apparent
curvature of about $\lVert\omega\rVert/v_r$. On the helix (0.8 m off the axis, 0.15 rad/s) we
measure 16.9 mm about a vertical axis and 16.1 mm about a tilted one, against 9.7 mm static.

**Tests** (`test_d3_frame`, oracles O3D-10 to O3D-14 of §14.6):

- O3D-10: a 1.5 m circle in a translating, rotated frame gives $\Phi_T = (I - TT^\top)u$ to
  $10^{-9}$, $v_c = u$ and $\lVert\Phi\rVert = v_r$.
- O3D-11: $v_c$ equals the central difference ($10^{-5}$ s) of the world foot under the exact
  frame motion, for a rotating, translating frame and a moving expression, to $10^{-6}$.
- O3D-12: closed loop on a translating circle, $D < 0.01$ m after 15 s.
- O3D-13: closed loop on a helix in a frame rotating about a vertical and a tilted axis,
  $D < 0.02$ m until the goal, no Assumption-3 flag, settled, and the hold stays settled.
- O3D-14: FrameTwistEstimator3 recovers a constant twist to $10^{-6}$ while the heading crosses
  $\pm\pi$, rejects outliers, duplicates, old stamps and non-rigid poses, and restarts after
  gaps.

Further tests cover the SE(3) round trip ($10^{-12}$ from $10^{-9}$ rad to 3.1 rad), continuity
across the series switch, the flag of a fast frame, the static-frame equality, a frame callback,
and the no-allocation run (O3D-9) of a circle and the helix in moving frames, goal hold included.

## Obstacles [Design, 0.3.0]

**Decision.** The 3D tracker avoids static obstacles with the CBF safety filter of the planar
`obstacle_mode: cbf` (math_spec §6.7), generalised to $\mathbb R^3$. The equations are in
math_spec §14.8. The 3D law is a single integrator ($\dot p = v$ after the limits), so a barrier
on the commanded velocity is exact there. In the plane the filter needs the flat-output point of
feedback linearisation or the Ackermann turning margin; in 3D it needs neither. Both obstacle
representations land in this item.

- *Analytic primitives* cover hand-built scenes and tests: spheres, boxes (axis-aligned, or
  oriented by yaw, pitch and roll), vertical cylinders (finite, or infinite poles) and solid
  half-spaces (ground, ceiling, walls). Each is convex with an exact signed distance and outward
  normal, so a primitive is exactly one barrier.
- *A voxel grid* covers perception: a uint8 occupancy mask with a resolution and an origin.
  `DistanceTransform3` is the 2D EDT with one more axis. It runs the Felzenszwalb–Huttenlocher
  lower envelope along z, y and x, carrying the flat index of the nearest occupied voxel, so the
  feature needs no recombination between passes. It is exact in integer voxel units and is
  rebuilt only when the grid's content stamp changes. Its continuous query is the trilinear
  analogue of the planar bilinear query (math_spec §6.1). Normals come from the feature vectors,
  not from a finite difference of the distance field, which is not differentiable on the medial
  set.

`ObstacleSet3` owns the primitives, a copy of the grid and its transform, so one pointer in
`Tracker3Input` (or one value in `SimulationOptions3`) carries a scene. Without obstacles, or
with `obstacle_mode: none`, Tracker3 is bit-identical to 0.2.

**Body.** The body is a sphere of radius `body_radius`; `cbf_margin` is added to it. Multirotors
are close to rotation-invariant, and a sphere makes $h$ independent of the yaw. The yaw loop
therefore never moves the body into an obstacle, and the filter acts on $v$ alone. A
non-spherical body would be a union of spheres at body offsets $e_i$. Each sphere has its own
sites with $\dot p_i = v + \omega \times R e_i$, and the QP runs over $(v, \omega)$. The active-set
solver extends to four unknowns, and the site budget grows with the number of spheres.

**Sites and solver.** Each primitive is one site. The voxels add the continuous query and
window features of distinct directions (the planar 45° rule, $5^3$ voxels). The $k \le 4$ nearest
sites are constrained. Three constraints bind independently in $\mathbb R^3$, and a fourth covers
corners such as two walls, a floor and a pole. `solveCbf3` enumerates every active set of at
most three independent constraints (15 candidates for $k = 4$). Each candidate is a 1×1, 2×2 or
3×3 Gram solve on the stack, and the least-cost feasible candidate is the optimum. Infeasible
push-outs relax as in 2D. Nothing allocates. Sorting by distance keeps the selection
deterministic. Two spheres close together are both in the filter, so $F^\star$ changes
continuously and does not chatter between them.

**Where the filter acts.** It acts on the law's command *after* its kinematic box. A filtered
command is boxed again uniformly. A uniform scale $c \le 1$ keeps $n^\top cF^\star \ge -\gamma_b h$
whenever $h \ge 0$, while the $\eta$-box of time-varying fields changes the direction of the
command. Filtering after it is therefore the order that keeps the final target inside every
constraint. When the filter is inactive the command is the law's, bit for bit. The transported
goal hold of a moving frame is filtered as well, because the obstacles are static in the
evaluation frame while the goal rides with the curve frame. Zero targets are never filtered. The
filter stays on after the XY latch because, unlike the planar tracker, Tracker3 has no rollout
behind it. A goal within $\delta_{\mathrm{safe}}$ of an obstacle is therefore never reached; the
robot reports kBlocked instead.

**Hold.** The planar rule carries over. In FOLLOW or FINAL_APPROACH, a filter that leaves less
than 5 % of the field speed sets kBlocked and zeroes the target, so the limiter brakes. It
releases at 10 %. kBlocked is a per-cycle status, not a phase: Tracker3 stays in FOLLOW and
resumes when the obstacle clears. STOPPED and the goal hold are unchanged. A cancel overrides
the hold, and a new `goal_seq` clears it. An exactly head-on obstacle (a sphere centred on a
straight path) holds the robot at $h \approx 0.05\,v_r/\gamma_b$. There is no tie-break
circulation, for the reason the planar note gives.

**Contour is out of scope.** The contour mode circulates around a blocking obstacle in a
direction chosen once per episode (left or right). In 3D the escape directions around an
obstacle form a circle, not a pair. A circulation needs a plane (for example the plane of $T$
and $n_o$, or a horizontal plane for ground robots in 3D), a rule to keep that plane over the
episode, and a 3D blocking set and gate. That is a design of its own, left as future work.
`obstacle_mode: contour` is a ConfigError in `Tracker3Config`.

**Limits.**

- Obstacles are static in the evaluation frame. Moving obstacles would add $\partial_t D$ to
  $\dot h$ and need its estimate.
- Voxel distances are measured to occupied voxel centres; the filter subtracts half a voxel
  diagonal, $\tfrac{\sqrt3}{2}\mathrm{res}$, so $h \ge 0$ keeps the body $m$ from the voxel
  cubes. The cubes over-approximate the obstacle by up to that much.
  Outside the grid there are no voxel sites.
- The CommandLimiter can delay the deceleration a head-on approach needs ($\gamma_b v_r$). Keep
  $\gamma_b v_r \le a_{\mathrm{dec}}$. `cbf_gain < control_frequency` is enforced ($\gamma_b\Delta t < 1$).
- Only the $k$ nearest sites are constrained. A farther site can bind only while
  $\gamma_b h < \lVert F\rVert$, so scenes with more than four obstacles within
  $v_r/\gamma_b + \delta_{\mathrm{safe}}$ of the body can miss one.

**Tests** (`test_d3_obstacles`, `test_oracle_d3_cbf`, oracles O3D-15 to O3D-24 of §14.6):

- Primitive distances are checked against sampled surfaces and finite differences.
- The voxel EDT equals brute force on random grids.
- `solveCbf3` is checked against its KKT conditions on 4000 random problems.
- Closed loop: spheres off a line and off the helix are passed with
  $\min h \ge -10^{-3}$ (measured 0.8 mm and 6.9 mm) and the curve is rejoined.
- A sphere centred on the line holds the robot with kBlocked at $h = 12$ mm, at rest.
- A ground plane under a descending helix keeps $z \ge \delta_{\mathrm{safe}}$.
- A corridor narrower than the helix is followed inside.
- Between two close spheres both constraints bind, with no chattering.
- A $64^3$ voxelised sphere is passed with a true barrier of $-18$ mm (res 50 mm).
- Without obstacles the tracker is bit-identical, and `Tracker3::step` and the voxel EDT
  rebuild do not allocate.
