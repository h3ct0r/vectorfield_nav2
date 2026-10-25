(core-3d)=

# 3D core (experimental)

:::{warning}
The 3D core is **experimental in 0.x**. Its C++ and Python APIs may change in a minor release.
Use it from C++, from Python, or through the experimental ROS 2 node {doc}`follower_3d`.
:::

The field of {cite:t}`rezende2022constructive` is defined in any dimension ({doc}`theory`).
`vectorfield_core` ships a 3D version for aerial and underwater vehicles that can move in every
direction. It provides curves in $\mathbb R^3$, the field $\Phi = \eta\Phi_S + \Phi_T$ with the
terminal field for open curves, a holonomic single-integrator law with a yaw policy for
multirotors {cite:p}`rezende2020quadcopter`, a minimal tracker and a kinematic simulator. The
equations are in {doc}`design/math_spec` §14, and the design rationale is in
{doc}`design/core_3d`.

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
| `CurveFrame3`, `FrameTwistEstimator3`, `expSE3()`, `logSE3()` | `CurveFrame`, `FrameTwistEstimator`, `exp_se3()`, `log_se3()` | Curves in a moving frame, the frame twist from sampled poses [0.3.0] |
| `Primitive3`, `VoxelGrid3`, `DistanceTransform3`, `ObstacleSet3`, `solveCbf3()` | `Primitive`, `VoxelGrid`, `DistanceTransform`, `ObstacleSet`, `solve_cbf()` | Static obstacles and the 3D CBF safety filter [0.3.0] |

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
vf-sim3d --curve helix:cx=0.8 --frame-rate 0,0,0.15 --gif rotating.gif
vf-sim3d --curve circle:radius=2 --frame-velocity 0.15,0.05,0 -o translating.png
vf-sim3d --curve helix --pose 1,0,0 --sphere 1.35,0,0.5,0.3 --cylinder 0,-1.4,0.1 -o avoid.png
vf-curve --dim 3 --list
vf-curve --dim 3 --curve torus_knot:p=3,q=2 --export knot.csv --export knot.yaml --plot -o knot.png
```

`vf-curve --dim 3 --export FILE.yaml` writes the 3D spec as a mapping. It looks like `Curve.msg`
with `z_expression` and 3D points, but it is not a ROS message. `--curve` reads it back, as it
does an `x,y,z` CSV. The 2D tools are unchanged.

## Moving frames

*New in 0.3.0, experimental.* A curve can live in a frame $\mathcal F$ that moves rigidly, for
example a path attached to a moving platform. `Tracker3Input::frame` is a `CurveFrame3`: the
transform $T_{\mathrm{eval}\leftarrow\mathcal F}$ at the cycle's time, the frame's body twist
$(v_{\mathcal F}, \omega_{\mathcal F})$ expressed in $\mathcal F$, and `moving`. The tracker
maps the body into $\mathcal F$, searches there (the hint stays in $\mathcal F$), and maps the
foot, tangent and normal back. A curve point moves at
$\partial_t r = R\,(v_{\mathcal F} + \omega_{\mathcal F} \times r_{\mathcal F} +
\partial_t r_{\mathcal F})$.

- **Closed curves** get the feedforward $\Phi_T = \Pi_T\,\partial_t r$ at the foot, as for a
  curve that moves through `t`: $\lVert\Phi\rVert = v_r$, and only the normal motion of the
  curve is compensated.
- **Open curves** must be static in $\mathcal F$. The body follows the static terminal field
  *relative to the frame* and adds the frame's velocity at its own position, so it settles at the
  goal as the goal moves. The goal hold then rides with the frame instead of commanding zero.
- **Speed.** When the feedforward exceeds $(1-\varepsilon_v)v_r$ it is clamped and
  kAssumption3Clamped is set. No clamp occurs anywhere on the curve if
  $\lVert v_{\mathcal F}\rVert + \lVert\omega_{\mathcal F}\rVert \max_s\lVert
  r_{\mathcal F}(s)\rVert < (1-\varepsilon_v)v_r$.
- **A static frame** (the default: identity, not moving) gives results bit-identical to 0.2.

```python
import numpy as np
from vectorfield import d3

helix = d3.make_curve(d3.CurveSpec.builtin('helix', cx=0.8))  # 0.8 m off the frame's z axis
frame = d3.CurveFrame(w=[0.0, 0.0, 0.15])                     # turning at 0.15 rad/s about z
tracker = d3.Tracker(d3.TrackerConfig(desired_speed=0.5))
simulator = d3.sim.ClosedLoopSimulator(tracker, frame=frame, stop_when_settled=False)
simulator.reset(np.array([2.0, -0.2, -0.1]))
run = simulator.run(helix, 60.0)          # run['frame'] holds the (4, 4) frame of every step
print(run['D'][run['t'] > 15].max())       # about 0.017 m, until the goal

# The frame twist from sampled poses (TF), 20 Hz:
estimator = d3.FrameTwistEstimator()
for k in range(21):
    estimator.add_pose(0.05 * k, d3.exp_se3([0.1, 0.0, 0.0], [0.0, 0.0, 0.3], 0.05 * k))
estimate = estimator.estimate(1.0)         # pose extrapolated with the exact SE(3) exponential
cycle_frame = estimate.curve_frame()      # pass to Tracker.step(..., frame=cycle_frame)
```

:::{figure} _static/figures/moving_frame_3d.svg
:alt: A helix drawn at three times as its frame turns about the vertical axis, with an orange trajectory that joins the turning helix and follows it to its end.
:width: 70%

A helix 0.8 m off the axis of a frame turning at 0.1 rad/s, drawn at three times, and a
`Tracker3` run at 0.5 m/s and 20 Hz (orange) that joins it and settles at the moving goal (×).
:::

The simulator moves the frame with its constant twist, exactly
($T(t) = T(t_0)\exp((t - t_0)\hat\xi)$), or calls `frame_motion(t)` for any other motion.
`FrameTwistEstimator` mirrors the follower's planar estimator: it differences consecutive
poses with $\log_{SE(3)}$ (exact for a constant twist, also when the orientation crosses
$\pm\pi$), drops outliers, low-pass filters the twist and marks old input stale. Under ZOH a
moving frame adds to the residual distance: 8 mm on a 2 m circle translating at 0.16 m/s, and
17 mm on the helix above (math_spec §14.7).

(core-3d-obstacles)=

## Obstacles

[0.3.0] With `obstacles.obstacle_mode = CBF`, `Tracker3` keeps a spherical body away from static
obstacles. It uses a control-barrier-function safety filter, the 3D form of the planar
{ref}`cbf-mode`. The equations are in math_spec §14.8, and the design rationale is in
{doc}`design/core_3d`, "Obstacles".

- **Obstacles** live in the evaluation frame. They are analytic primitives: spheres, boxes
  (axis-aligned, or rotated by yaw, pitch and roll), vertical cylinders (finite, or infinite
  poles) and planes (the solid half-space behind the normal: ground, ceiling, walls). A voxel
  grid can be added, given as a `(nz, ny, nx)` uint8 mask with a resolution and the origin of
  its corner. Its exact 3D distance transform is rebuilt only when the grid changes.
- **Body.** A sphere of radius `body_radius`. With `cbf_margin` $m$, the barrier of each nearby
  obstacle is $h = D - (r_b + m)$.
- **Filter.** The velocity command becomes the closest velocity that satisfies
  $n_j^\top v \ge -\gamma_b h_j$ for the `cbf_sites` ($\le 4$) nearest obstacles, with
  $\gamma_b$ = `cbf_gain`. The body slows down as it approaches an obstacle and slides along it.
  A single integrator follows this filtered command exactly, so $h$ stays non-negative up to the
  limiter's delay.
- **Hold.** When the filter leaves less than 5 % of the field speed, for example with an
  obstacle centred on the path, the tracker sets `Status.BLOCKED` and brakes to rest. It stays
  in FOLLOW and resumes when the obstacle clears (released at 10 %).

```python
obstacles = d3.ObstacleSet([
    d3.Primitive.sphere(np.array([1.35, 0.0, 0.5]), 0.3),
    d3.Primitive.plane(np.array([0.0, 0.0, -0.3]), np.array([0.0, 0.0, 1.0])),  # the ground
    d3.Primitive.cylinder(np.array([0.0, -1.4, 0.0]), 0.1),                     # a pole
])
mask = np.zeros((32, 32, 32), dtype=np.uint8)   # mask[iz, iy, ix], nonzero = occupied
mask[:, 12:20, 2] = 1
obstacles.set_voxels(d3.VoxelGrid(mask, 0.1, np.array([-0.2, 0.0, -0.5])))

config = d3.TrackerConfig(desired_speed=0.5)
config.obstacles.obstacle_mode = vf.ObstacleMode.CBF
config.obstacles.body_radius = 0.2              # cbf_margin 0.05, cbf_gain 2, cbf_sites 4
sim = d3.sim.ClosedLoopSimulator(d3.Tracker(config), obstacles=obstacles)
sim.reset(np.array([1.0, 0.0, 0.0]))
run = sim.run(curve, 60.0)
print(run['h'].min(), run['cbf_active'].sum())  # least barrier (m), steps the filter acted
```

In C++ the same scene is an `ObstacleSet3`, passed as `Tracker3Input::obstacles` or
`SimulationOptions3::obstacles`. `Tracker3Output::cbf` reports the sites, the barriers and
$F^\star$.

:::{figure} _static/figures/cbf_sphere_3d.svg
:alt: A three-turn helix with a grey sphere just outside its first turn and another just inside its second turn; the orange trajectory leaves the helix to pass each sphere and rejoins it.
:width: 70%

The default helix past two spheres (body radius 0.2 m, margin 0.05 m, 0.5 m/s, 20 Hz). The
trajectory (orange) bends inward around the outer sphere and outward around the inner one, then
rejoins the helix.
:::

In the tests (math_spec O3D-17 to O3D-24) the least barrier is 0.8 mm past a sphere beside a
line and 6.9 mm past one beside the helix. A centred sphere holds the robot at
$h \approx 12$ mm. A floor under a descending helix and two walls narrower than the helix are
followed at $h = 0$. A voxelised sphere (0.05 m voxels) is passed 26 mm outside its true
surface: voxel distances are measured to voxel centres, and the filter counts each voxel site
from its cube (half a voxel diagonal closer).

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
with the hint carried, take 1.0 µs, and one `Tracker3::step` takes 1.1 µs. With the CBF filter
a step takes 1.3 µs with three primitive sites and 3.1 µs with a voxel grid. The filter's QP
alone takes 0.14 µs, and the EDT of a $64^3$ grid takes 6.6 ms. `Tracker3::step` does not
allocate.

## Limitations

- Obstacles must be static in the evaluation frame. The body is a sphere. There is no contour
  mode in 3D (a circulation around an obstacle needs a plane) and no collision rollout behind
  the CBF filter: only the `cbf_sites` nearest obstacles are constrained, and the limiter can
  delay a deceleration, so keep `cbf_gain * desired_speed <= max_decel`. Voxels are measured to
  their centres, and there are no voxel sites outside the grid.
- An open curve must be static in its frame: an expression in `t` must be closed. The ROS node
  ({doc}`follower_3d`) feeds the frame estimator from Odometry or TF.
- Polylines are deduplicated but not resampled or filleted.
- No rotate phases, collision rollout or goal yaw. The yaw only follows the policy.
- `Curve.msg` and FollowCurve stay planar. The 3D node uses the separate `Curve3`,
  `FollowCurve3` and `FieldState3` ({doc}`follower_3d`).
