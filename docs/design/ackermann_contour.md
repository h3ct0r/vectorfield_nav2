# Obstacle contouring for Ackermann bases

`obstacle_mode: contour` is accepted with `motion_model: ackermann`. Before 0.2.0 it was a ConfigError
(`ackermann_contour_unsupported`), because the auto distances of §11 ask a car for curvature
$1/\lambda \approx 2$–3 m⁻¹ and for a quarter turn within 0.2 m. The changes to {doc}`math_spec`
are tagged [Design] in §6.5, §6.6 (new), §7.5, §11 and §12.5 (O57). Every other motion model
is unchanged.

**Feasibility and λ.** The contour holds the evaluation point on the level set $D_o = \lambda$ of
$\mathrm{EDT}_K$. Around a convex feature (a point, a disc, a box corner) that set is a circle of
radius $R_o + \lambda \ge \lambda$; along a face it is straight. The circulation is feasible iff
$\lambda \ge R_{\min}$. The Ackermann auto value is
$\lambda = \max(\lambda_{\mathrm{auto}},\ R_{\min} + w_{\mathrm{lat}} + 0.05)$, with
$\lambda_{\mathrm{auto}}$ the footprint rule of §11 at $d = \lvert x_r\rvert$. The steady
circulation then uses at most $R_{\min}/\lambda$ of the car's curvature (TB3: 65 % at
$R_{\min} = 0.5$ m, 80 % at 1 m), and the rest corrects tracking errors. Validation: an error when
$\lambda < R_{\min} + 0.05$ (`ackermann_contour_distance_below_turning_radius`), and the warning
`ackermann_contour_curvature` when $\lambda < R_{\min} + w_{\mathrm{lat}} + 0.05$.

**Turn-in band.** A car heading straight at a face across its approach is tangent to the level
set only if it starts a quarter turn at $R_{\min}$ by $D_o = \lambda + R_{\min}$. A point obstacle
needs only $\sqrt{\lambda^2 + 2\lambda R_{\min}}$. So the auto values are
$D_{\mathrm{in}} = \lambda + \max(0.15, R_{\min})$ and $D_{\mathrm{in}}' = D_{\mathrm{in}} + 0.2$.
The blend band keeps its 0.2 m, and the field starts to turn there. The gate has to open before
the car reaches the band, so the auto lookahead is
$\ell_{\mathrm{look}} = \max(1.0,\ v_r T_h,\ D_{\mathrm{in}}' + 0.1)$. Warnings:
`ackermann_contour_turn_in` ($D_{\mathrm{in}} < \lambda + R_{\min}$), `ackermann_contour_lookahead`
($\ell_{\mathrm{look}} < D_{\mathrm{in}}'$ with `path_blocked`) and `ackermann_contour_watchdog`
(`obstacle_max_contour_length` $< D_{\mathrm{in}}' - \lambda + \pi\lambda$, the side probe's
approach plus half a circulation). For TB3 with $R_{\min} = 0.5$ the values are $\lambda = 0.77$,
$D_{\mathrm{in}} = 1.27$, $D_{\mathrm{in}}' = 1.47$ and $\ell_{\mathrm{look}} = 1.57$ m. The
existing blend-zone rule ($D_{\mathrm{in}}' + d < \tfrac12 \min(W, H_c) - 0.2$) then needs a 4 m
local costmap, so the shipped overlay `config/controller/ackermann.yaml` sets a 4 × 4 m window.
It also enables contour, like the other overlays; the core default stays `none`. There are no new
parameters, only Ackermann auto values and issue codes.

**Evaluation point and footprint.** The field is evaluated at the rear axle $p_r$, the point where
the law evaluates it (§7.1), so the field and the law agree. $p_r$ is the no-slip point, so on
the level set the car's heading is tangent to it. $n_o$ is the $\mathrm{EDT}_K$ normal at $p_r$,
with the base_link guard of §6.4. The footprint rules of §11 use $d = \lvert x_r\rvert$: since
$\lambda - d \ge r_{\mathrm{circ}} + 0.05$, every corner stays at least 0.05 m from a point
obstacle whatever the heading. During a circulation the body's inner side is $\lambda - w_{\mathrm{lat}}$
from the feature.

**Law in an episode.** The field direction can turn faster than a car: in the blend band, when
$n_o$ switches at a corner, or when an episode starts inside the band (an obstacle seen late).
Heading tracking already clamps $\lvert\omega\rvert \le \lvert v\rvert/R_{\min}$
(kCurvatureSaturated). Inside an episode only, two more rules apply. The speed factor is
$\max(\cos e, \tfrac12)$ instead of $\max(0, \cos e)$, and $\lvert e\rvert > \pi/2$ does not set
kHeadingInfeasible. Instead of stopping (a car cannot rotate in place), the car keeps at least half
the field speed and turns at $R_{\min}$ toward $F$. Inside $\lambda$, $F$ has a positive
$\sigma_c J n_o$ component, so it turns toward the circulation. The collision rollout and the side
probe run the same law, so an arc that would collide brakes the car through the cap.

**Exit.** With a wide circulation, $s^\star$ passes the blocking set about 20° past the side
point. At that moment $\Phi$ still points at the obstacle, and the episode ended on "$B(s^\star)$
empty". The car's turn back to the path then cut 6 cm inside $\lambda$ ($R_{\min} = 1$, a 0.3 m
disc). For Ackermann, once $B(s^\star)$ is empty, the direction exit is measured to the episode's
last blocking site, kept in `ContourState::b_site` (grid frame, fixed size).

**Side probe, dead ends, no rotation in place.** The probe already rolls out the real pipeline:
the speed band, the law above and the curvature clamp. A side the car cannot drive around therefore
collides or never gets past the blockage: a gap narrower than about $\lambda + w_{\mathrm{lat}}$, a
dead end, or a wall wider than $v_r T_{\mathrm{side}}$. Then no episode starts, the cap stops the
car at least $m_s$ short, kBlocked raises NoValidControl and Nav2 replans. $T_{\mathrm{side}}$
grows with $\lambda$ and $D_{\mathrm{in}}'$ by itself. An episode never changes the drive direction:
forward it never commands $v < 0$, and on a reverse segment (0.3.0, §9.4) it backs up throughout,
with the same probe, law and curvature clamp at the rear axle. A manoeuvre that
needs a turn tighter than $R_{\min}$ ends in a stop and kBlocked. A planner with reversing
(Smac Hybrid-A* Reeds-Shepp, `allow_reversing`) can then plan a three-point turn.

**Tests.** O57 (core, $R_{\min}$ 0.5 and 1 m, a 0.7 × 0.4 m body with base_link 0.2 m ahead of
the axle):

- discs of radius 0.15 and 0.3 m, centred and 0.1 m off the path: closest approach of the rear
  axle 0.757–0.776 m ($\lambda = 0.75$) and 1.217–1.248 m ($\lambda = 1.25$), no collision, no
  kHeadingInfeasible, free side, settled on the path;
- a box closed on one side, with a gap of $R_{\min} + 0.3$ m on the other, and the O56 dead end:
  no episode, stopped 0.095 m short, kBlocked kept. TC drives through the same gap;
- a 0.4 m box 0.8 m ahead of a car at rest: the rear axle passes the corner at 0.354 m (centred)
  and 0.394 m (offset), exactly the R_min-turn geometry
  $\sqrt{a^2 + (R_{\min} - b)^2} - R_{\min}$.

The kinematic system test (`test_open_kinematic_ackermann`) runs that box case with this
geometric bound minus 0.05 m in place of $\lambda - d - 0.05$, makes a U-turn between the
centred and the offset case, and checks that no infeasible command reaches the simulator.

**Limits.** Below $D_{\mathrm{in}}$ the clearance is set by geometry, not by $\lambda$. At a
concave corner of $K$ the level set has a kink that the car cuts while saturated. Narrow passages
that a differential robot contours through stop the car.
