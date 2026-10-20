(core-3d)=

# 3D core (experimental)

:::{warning}
The 3D core is **experimental in 0.x**. Its C++ and Python APIs may change in a minor release.
It has no ROS node yet: use it from C++ or Python.
:::

The field of {cite:t}`rezende2022constructive` is defined in any dimension ({doc}`theory`).
`vectorfield_core` ships a 3D version for aerial and underwater vehicles that can move in every
direction. It provides curves in $\mathbb R^3$, the field $\Phi = \eta\Phi_S + \Phi_T$ with the
terminal field for open curves, a holonomic single-integrator law with a yaw policy for
multirotors {cite:p}`rezende2020quadcopter`, a minimal tracker and a kinematic simulator. The
equations are in {doc}`design/math_spec` §14, and the design rationale is in
{doc}`design/m7_3d`.

## What it contains

| C++ (`vectorfield_core::d3`) | Python (`vectorfield.d3`) | Purpose |
|---|---|---|
| `Vec3`, `Pose3`, `Twist3` | NumPy `(3,)` / `(N, 3)` | Points, position and yaw, linear velocity and yaw rate |
| `CurveSpec3`, `makeCurve3()` | `CurveSpec`, `make_curve()` | Polylines, expression triples, built-ins; the single validation point |
| `Curve3`, `PolylineCurve3`, `ParametricCurve3` | `Curve` | Closest point, stations, rays, sampling |
| `DistanceField3`, `FieldSample3` | `DistanceField` | $\Phi$, the terminal field, streamlines |
| `VelocityLaw3`, `CommandLimiter3`, `Limits3`, `YawConfig3` | `Limits`, `YawConfig` | $v = F$ with the box, the yaw policy, uniform-$\alpha$ limiting |
| `Tracker3`, `Tracker3Config` | `Tracker`, `TrackerConfig` | FOLLOW, FINAL_APPROACH, settled and goal hold, cancel → STOPPED |
| `sim::integrate`, `sim::ClosedLoopSimulator3` | `d3.sim.integrate`, `d3.sim.ClosedLoopSimulator` | Point plant with an optional velocity lag $\tau$, closed loop |

The 3D core is compiled into the same `vectorfield_core` library as the planar core. Include
`vectorfield_core/d3.hpp`; the planar umbrella header does not include it. It reuses the planar
`GainFunction`, `TerminalProfile`, `Status`, `Phase`, `CurveOptions` and `CurveError`.

**Built-in shapes.** Every shape also takes the pose parameters `cx cy cz yaw pitch roll`.

| Shape | Parameters (defaults) | Closed |
|---|---|---|
| `helix` | `radius` 1, `lead` 0.5 (rise per turn), `turns` 3 | no |
| `circle` | `radius` 1 | yes |
| `ellipse` | `a` 2, `b` 1 | yes |
| `viviani` | `radius` 2 (of the sphere) | yes, crosses itself at $(R, 0, 0)$ |
| `lemniscate3d` | `a` 2, `h` 0.5: Gerono's figure eight lifted by $z = h\sin s$ | yes |
| `torus_knot` | `p` 2, `q` 3 (coprime), `major_radius` 2, `minor_radius` 0.5 | yes |

Expression curves take three hardened muParser expressions in `s` and `t`. The rules are those
of the planar curves: parameters are constants, `rnd`, `&&` and `||` are rejected, and the
length and parameter count are limited. An open curve that uses `t` is rejected as UNSUPPORTED.

## Python

```python
import numpy as np
import vectorfield as vf
from vectorfield import d3

curve = d3.make_curve(d3.CurveSpec.builtin('helix', radius=1.0, lead=0.5, turns=3))
field = d3.DistanceField(vf.GainFunction(vf.GainType.ATAN, 2.0), v_r=0.5)
print(field(curve, np.array([1.3, 0.0, 0.2])))       # Phi at a point, norm 0.5

config = d3.TrackerConfig(desired_speed=0.5)
config.limits.max_axis_speed = np.array([1.0, 1.0, 0.5])
config.yaw.policy = vf.OmniHeadingPolicy.PATH_TANGENT
simulator = d3.sim.ClosedLoopSimulator(d3.Tracker(config), period=0.05)
simulator.reset(np.array([1.6, -0.4, -0.5]))
run = simulator.run(curve, 60.0)                        # dict of arrays
print(run['D'][-1], run['settled'][-1])
```

`Tracker.step(curve, position, yaw, state, t, dt, ...)` returns `(TrackerOutput, TrackerState)`
for use in your own loop. The command `out.v` is in the curve frame (world, ENU), not the body
frame. Rotate it by $-\psi$ for a body-frame velocity setpoint.

## Command line

```bash
vf-sim3d --curve helix -o helix.png
vf-sim3d --curve viviani --pose 2.5,0.5,0.6 --streamlines 10 --gif viviani.gif
vf-sim3d --curve 'cos(s); sin(s); 0.3*sin(2*s)' --lag 0.1 --csv run.csv
vf-curve --dim 3 --list
vf-curve --dim 3 --curve torus_knot:p=3,q=2 --export knot.csv --export knot.yaml --plot -o knot.png
```

`vf-curve --dim 3 --export FILE.yaml` writes the 3D spec as a mapping. It looks like `Curve.msg`
with `z_expression` and 3D points, but it is not a ROS message. `--curve` reads it back, as it
does an `x,y,z` CSV. The 2D tools are unchanged.

## Behaviour

- **Field.** $\lVert\Phi\rVert = v_r$ for static curves and $\dot D = -\eta G(D)$ hold to
  rounding, at polyline corners too. The 3D wedge tangent blends the two segment tangents
  (math_spec §14.2).
- **Law.** $v = F$, saturated by a uniform scale, so the direction is kept. The bounds are
  $\lVert v\rVert \le$ `max_speed` and $\lvert v_i\rvert \le$ `max_axis_speed[i]`. A moving
  curve lowers $\eta$ instead and keeps the feedforward (the $\eta$-box). The yaw tracks the
  horizontal path tangent, the field, or a fixed yaw. It is held while that direction is
  vertical.
- **Limiter.** One acceleration scale is shared by $v_x, v_y, v_z$ and $\omega$, so the
  velocity changes along a straight line. A cancel decelerates at `cancel_decel`.
- **Open curves** end at a goal. FINAL_APPROACH begins within the final-approach distance (auto:
  the larger of `terminal_braking_distance` and `goal_tolerance`). The tracker settles within
  `goal_tolerance / 2` of the goal and holds there.
- **Discrete time.** Under ZOH the distance settles at about
  $v_r\Delta t\,\kappa/(2G'(0))$: 9.7 mm on the default helix at 0.5 m/s and 20 Hz. Raise
  the rate or `convergence_gain` to reduce it.

Benchmarks (Apple M3 Pro, Release; {doc}`benchmarks`): closest point and field on the helix,
with the hint carried, take 1.0 µs, and one `Tracker3::step` takes 1.1 µs. `Tracker3::step`
does not allocate.

## Limitations

- No obstacles. A spherical-obstacle CBF filter (the closed-form one-constraint case of
  {ref}`cbf-mode`) is future work.
- No moving frames (`CurveFrame`). A moving curve is an expression in `t`, and it must be closed.
- Polylines are deduplicated but not resampled or filleted.
- No rotate phases, collision rollout or goal yaw. The yaw only follows the policy.
- No ROS node or messages: `Curve.msg` and FollowCurve stay planar.
