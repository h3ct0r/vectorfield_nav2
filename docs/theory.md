# Theory

This page summarises the constructive vector field of {cite:t}`rezende2022constructive`, which
extends earlier work on fields for time-varying curves {cite:p}`goncalves2010vector`. It also
covers the additions this project needs for Nav2. The implementation follows the open-access
PhD thesis {cite:p}`rezende2022thesis`. The authoritative specification, with every symbol,
guard and tolerance, is {doc}`design/math_spec`.

:::{note}
Equation numbers are **thesis** numbers. T-RO equation numbers will be added only after they
have been checked against the paper. The thesis writes the field as $\Phi$; the T-RO paper
writes it as $\Psi$.
:::

## Problem

The target is a curve $\mathcal{C}(t) \subset \mathbb{R}^n$ parametrised as $r(s, t)$. It is
either unbounded and homeomorphic to a line, or bounded and homeomorphic to a circle
(Assumption 1), and $r$ is $C^2$ in $s$ and differentiable in $t$ (Assumption 2). With the robot
modelled as $\dot{x} = \Phi(x, t)$, the goal is a field with $\lVert\Phi\rVert = v_r$ whose
trajectories converge to $\mathcal{C}(t)$ and then travel along it. This project works in 2D
($n = 2$).

## Closest point and distance

$$
s^*(x, t) = \arg\min_s \lVert x - r(s, t) \rVert^2 \tag{3.1}
$$

The closest point is $x^* = r(s^*, t)$ (3.2), the distance vector is $\mathbf{D} = x - x^*$ and
the distance is $D = \lVert\mathbf{D}\rVert$ (3.3–3.4). With $n = \mathbf{D}/D = \nabla D$
(3.19), $T$ the unit tangent at $s^*$ and $\Pi_T = I - TT^\top$ (3.9), the first-order
optimality condition gives $n^\top T = 0$ (Lemma 2).

## Gain functions

$G(D)$ is zero at $D = 0$, strictly increasing and below 1; $H = \sqrt{1 - G^2}$ (Defs. 10–11).
The thesis experiments all use

$$
G(D) = \frac{2}{\pi}\arctan(k_f D)
$$

with $k_f$ (`convergence_gain`, in 1/m) setting how sharply the field turns towards the curve.
This project also offers the rational gain $G = k_f D / \sqrt{1 + (k_f D)^2}$.

## The field

$$
\Phi_G = -G(D)\, n \tag{3.24}
$$

$$
\Phi_H = H(D)\, T \tag{3.25}
$$

$$
\Phi_T = -\Pi_T\, \frac{\partial \mathbf{D}}{\partial t} \tag{3.26}
$$

$$
\Phi = \eta\, \Phi_S + \Phi_T, \qquad \Phi_S = \Phi_G + \Phi_H \tag{3.27}
$$

$\Phi_G$ converges, $\Phi_H$ circulates and $\Phi_T$ is the feedforward for a moving curve.
Because $n^\top T = 0$ and $G^2 + H^2 = 1$, $\lVert\Phi_S\rVert = 1$ (Lemma 7). Requiring
$\lVert\Phi\rVert = v_r$ gives a quadratic in $\eta$, whose larger root is used:

$$
\eta^2 + 2\,(\Phi_S^\top \Phi_T)\,\eta + \lVert\Phi_T\rVert^2 - v_r^2 = 0 \tag{3.28}
$$

$$
\eta = -\Phi_S^\top \Phi_T + \sqrt{(\Phi_S^\top \Phi_T)^2 + v_r^2 - \lVert\Phi_T\rVert^2} \tag{3.29}
$$

The field has norm $v_r$ everywhere it is defined, so it has no equilibria. For a static curve
$\Phi_T = 0$ and $\eta = v_r$:

$$
\Phi = v_r\,(-G\, n + H\, T)
$$

:::{figure} _static/figures/field_ellipse.svg
:alt: Unit arrows and streamlines of the field around an ellipse; every streamline joins the ellipse and then circulates along it.
:width: 85%

The static field around the ellipse $a = 2$, $b = 1$ ($k_f = 2$, atan gain). Arrows show the
direction of $\Phi$, whose norm is $v_r$ everywhere; streamlines join the curve and circulate
along it.
:::

**Feedforward.** Since $x^* = r(s^*, t)$ and $\Pi_T$ removes the tangential part,
$\Phi_T = \Pi_T\, \partial r / \partial t\,\rvert_{s = s^*}$: the normal velocity of the curve at
the closest point. The implementation therefore needs only the curve velocity at $x^*$, for
example from a moving frame's twist.

:::{figure} _static/figures/field_decomposition.svg
:alt: At a point outside a circle that translates to the right, arrows for the convergence, circulation and feedforward terms and their sum F.
:width: 70%

The decomposition (3.27) at $x = (2.3, 0.8)$ for a circle of radius 1.5 m translating at
0.1 m/s along $+x$ (dashed: the circle 10 s later), with $v_r = 0.4$ m/s. $\Phi_T$ is the
curve's normal velocity at $x^*$, and $\eta = 0.46 > v_r$ scales $-G\,n$ and $H\,T$ so that
$\lVert F \rVert = v_r$. The arrows are drawn 2 s long.
:::

**Assumption 3.** The curve's normal speed must stay below the field speed,
$v_m = \max \lVert \Pi_T\, \partial\mathbf{D}/\partial t \rVert < v_r$ (3.10); otherwise $\eta$
is not real. The implementation clamps $\lVert\Phi_T\rVert$ to $(1 - \varepsilon_v)\, v_r$
(`assumption3_margin`) and reports `STATUS_ASSUMPTION3_CLAMPED`.

## Convergence

If the closest point stays unique along the trajectory (Assumption 4), the distance obeys

$$
\dot{D} = -\eta\, G(D) \le -(v_r - v_m)\, G(D)
$$

with $v_r - v_m \le \eta \le v_r + v_m$ (Lemma 8), so $D \to 0$ (Proposition 2). For a static
curve $\dot{D} = -v_r G(D)$ exactly. Under a velocity disturbance with
$\lVert\delta_v\rVert < \Delta_v < v_r - v_m$ the distance is ultimately bounded (Proposition 3);
with the arctangent gain,

$$
D < \frac{1}{k_f} \tan\!\left(\frac{\pi}{2}\, \frac{\Delta_v}{v_r - v_m}\right) \tag{5.4}
$$

## Ground robots

Differential and skid-steer robots use feedback linearisation about a control point
$p_c = p + d\,[\cos\psi,\ \sin\psi]^\top$ ahead of the base (`control_point_offset` $d$), with
$F = \Phi(p_c)$:

$$
v = \cos\psi\, F_x + \sin\psi\, F_y, \qquad \omega = \frac{-\sin\psi\, F_x + \cos\psi\, F_y}{d} \tag{2.1}
$$

Unsaturated, the control point moves with velocity exactly $F$. The heading-tracking,
holonomic and Ackermann laws, and the rotate and final-approach phases, are specific to this
project; {doc}`motion_models` and {doc}`design/math_spec` describe them. The same field also
drives aerial robots through an acceleration-level controller {cite:p}`rezende2020quadcopter`;
this project targets ground robots.

:::{figure} _static/figures/models_convergence.svg
:alt: Distance to a circle over 40 seconds on a log scale for four motion models; three decay exponentially, skid-steer levels off near 6 cm.
:width: 85%

Closed-loop $\lvert D \rvert$ of the four motion models on a circle of radius 1.5 m, starting
1 m outside it at $v_r = 0.4$ m/s with the default limits (`vf-sim`). Near the curve the decay
rate is about $G'(0)\, v_r$. The skid-steer plant has `vectorfield_sim`'s slip
($\eta_\psi = 0.75$, $x_{\mathrm{ICR}} = 0.05$), which feedback linearisation does not model;
it settles about 6 cm off the curve.
:::

## Open paths: the terminal field

Assumption 1 excludes curves with endpoints, but a Nav2 path ends at the goal. This project
extends the path with straight rays and replaces $H\,T$ by $H\,\varphi(\sigma)\,T$, where
$\sigma$ is the arc length remaining to the goal:

$$
\Phi = v_r\,\bigl(-G\, n + H\, \varphi(\sigma)\, T\bigr)
$$

$$
\varphi(\sigma) = \operatorname{sign}(\sigma)\, \min\!\left(1,\ \max\!\left(
  \frac{|\sigma|}{\sigma_b},\ \frac{v_{\min}}{v_r} \min\!\left(1,
  \frac{|\sigma|}{\sigma_{\mathrm{stop}}}\right)\right)\right)
$$

Here $\sigma_b$ is `terminal_braking_distance`, $v_{\min}$ is `min_approach_speed` and
$\sigma_{\mathrm{stop}}$ is half the goal tolerance. $\varphi$ is odd and Lipschitz, so $\sigma$
never changes sign while $D \to 0$: the robot converges onto the path and slows down over the
last $\sigma_b$ before the goal. This construction is not part of the paper.

**Derivation.** The rays make the extended path a borderless curve, so the closest point and
$n^\top T = 0$ still hold, and

$$
\dot D = n^\top \Phi = -v_r\, G(D),
$$

exactly as for a static closed curve: $D$ decreases monotonically to 0. By Lemma 3
(3.15–3.16), $\dot s^* = T^\top \dot x / \Omega$, with $\Omega = 1$ on segments and rays and
$\Omega > 0$ within the reach of curved pieces, so the remaining arc length obeys

$$
\dot\sigma = -\dot s^* = -\frac{v_r\, H(D)}{\Omega}\, \varphi(\sigma).
$$

1. **No overshoot.** $\varphi(0) = 0$ and $\varphi$ is Lipschitz, so $\sigma = 0$ is an
   equilibrium of this scalar equation; by uniqueness of solutions $\sigma$ keeps its sign and
   $\lvert\sigma\rvert$ never increases.
2. **Arrival.** For $\lvert\sigma\rvert \ge \sigma_{\mathrm{stop}}$,
   $\lvert\varphi\rvert \ge v_{\min}/v_r$, so $\lvert\dot\sigma\rvert \ge v_{\min} H / \Omega$,
   which tends to $v_{\min}$ as $D \to 0$: $\sigma_{\mathrm{stop}}$ is reached in finite time.
   Below it $\varphi$ is linear and $\sigma$ decays exponentially, so the robot reaches the goal
   tolerance but never the goal itself; success is decided by the settled test, not by
   $\sigma \le 0$.
3. **Speed.** $\lVert\Phi\rVert = v_r \sqrt{G^2 + H^2 \varphi^2}$ tends to 0 at the goal: the
   unit-norm property is given up on purpose.
4. **Discrete time.** On the path, a zero-order hold gives
   $\sigma_{k+1} = \sigma_k - \Delta t\, v_r\, \varphi(\sigma_k)$, which keeps the sign of
   $\sigma$ when $v_r \Delta t < \sigma_b$ and $v_{\min} \Delta t < \sigma_{\mathrm{stop}}$.
   In the linear part the demanded deceleration is at most $v_r^2 / \sigma_b$; the validator
   warns when it exceeds `max_linear_decel`.

{doc}`design/math_spec` (section 5) adds the reference-point shift of the feedback-linearisation
control point and the terminal-mode hysteresis.

:::{figure} _static/figures/terminal_profile.svg
:alt: Left, phi of sigma rising linearly to 1 over the braking zone, with a small floor near zero for the tighter tolerance. Right, commanded speed against remaining arc length following v_r times phi.
:width: 100%

Left: $\varphi(\sigma)$ at the defaults ($v_r = 0.4$ m/s, $v_{\min} = 0.05$ m/s,
$\sigma_b = 0.5$ m) for goal tolerances of 0.25 m and 0.05 m; the `min_approach_speed` floor
binds only when $v_{\min} \sigma_b > v_r \sigma_{\mathrm{stop}}$, here for 0.05 m. Right: the
commanded speed of a differential robot on a 4 m straight path follows $v_r\,\varphi(\sigma)$
through the braking zone until it settles inside the goal tolerance.
:::

## Obstacles

The thesis (§3.4, not in the T-RO paper) adds a field that circulates around an obstacle at
distance $\lambda$. With $x^*_o$ the closest obstacle point and $\mathbf{D}_o = x - x^*_o$:

$$
\mathbf{D}_\lambda = \mathbf{D}_o - \lambda\, \frac{\mathbf{D}_o}{D_o} \tag{3.65}
$$

$$
\Psi = \eta\, \Psi_G + \eta\, \Psi_H + \Psi_T \tag{3.70}
$$

$\Psi$ is built from $\mathbf{D}_\lambda$ as $\Phi$ is built from $\mathbf{D}$ (3.66–3.69). When
the robot heads into the obstacle, the commanded field blends from $\Phi$ to $\Psi$ between
$D'_{\mathrm{in}}$ and $D_{\mathrm{in}}$, with $\lambda < D_{\mathrm{in}} < D'_{\mathrm{in}}$
(3.71–3.72):

$$
F = \begin{cases}
\Phi & \text{if } \mathbf{D}_o^\top \Phi \ge 0 \text{ or } D_o > D'_{\mathrm{in}} \\
v_r\, \dfrac{\theta\Phi + (1 - \theta)\Psi}{\lVert\theta\Phi + (1 - \theta)\Psi\rVert}
  & \text{if } D_{\mathrm{in}} \le D_o \le D'_{\mathrm{in}} \\
\Psi & \text{if } D_o < D_{\mathrm{in}}
\end{cases}
\qquad
\theta = \frac{D_o - D_{\mathrm{in}}}{D'_{\mathrm{in}} - D_{\mathrm{in}}}
$$

The thesis gives no convergence proof for this extension. On a costmap, this project adds a gate
that engages only when the path is blocked, a circulation side latched per episode, hysteresis
and a watchdog. {doc}`obstacle_avoidance` covers its use.

:::{figure} _static/figures/contour_disc.svg
:alt: A robot on a straight path meets a disc, circulates below it at distance lambda, then returns to the path and stops at the goal.
:width: 100%

A disc of radius 0.3 m blocks a straight path (`obstacle_mode: contour`, differential drive,
defaults). Dashed: the $\lambda$ level set of the obstacle distance; dotted: $D_{\mathrm{in}}$
and $D'_{\mathrm{in}}$. Orange arrows are the commanded field where the contour is active, as
seen when the episode starts (black outline); the side is latched for the whole episode,
because a head-on approach would otherwise sit in the trap basin of {doc}`design/math_spec`
(section 6.3). The orange stretch of the trajectory is the contour episode.
:::

## Assumptions and limits

- **Unique closest point.** Points with several closest points (the medial axis) form a set of
  measure zero, but self-intersecting curves such as lemniscates cross it at every crossing.
  The curve follower uses a local search around the previous closest point (thesis §5.1) with
  hysteresis.
- **Curve speed.** Moving curves must satisfy Assumption 3.
- **Discrete time.** Near the curve the convergence rate is about $G'(0)\, v_r$; the validator
  warns when $G'(0)\, v_r\, \Delta t \ge 1$ at the control period $\Delta t$.
- **Saturation.** Uniform scaling of a static field keeps $\dot{D} \le 0$. For a moving curve it
  would break the feedforward cancellation, so the velocity limits act on $\eta$ instead
  where possible.
- **Obstacles.** The obstacle extension has no proof and can fail in traps; the watchdog bounds
  how long an episode may last.

The figures on this page come from the `vectorfield` bindings and are generated by
`docs/gen_figures.py`; `vf-plot` and `vf-sim` draw the same views for other curves and settings.

## References

```{bibliography}
```
