# Designing a setup offline

The Python tools run the controller's own C++ core without ROS: the same validator, the same
field and the same control loop, closed around a kinematic model. With them you can check a
parameter file, look at the field around a path or an obstacle, and simulate and sweep
parameters before the robot moves. This tutorial takes a parameter file and a representative
path through the four tools: `vf-tune`, `vf-curve`, `vf-plot` and `vf-sim`. Every tool has
`--help`; {doc}`../tuning` explains the validator's findings.

## 1. Install the tools

Without ROS (for example on macOS), install them with pip from a checkout. They need a C++17
compiler, CMake 3.22 or newer and Eigen 3.4 or newer (macOS: `brew install cmake eigen`):

```bash
git clone https://github.com/h3ct0r/vectorfield_nav2.git ~/vectorfield_nav2
```

```bash
python3 -m venv ~/vf_venv && . ~/vf_venv/bin/activate
```

```bash
pip install "$HOME/vectorfield_nav2/vectorfield_py[tools]"
```

In a ROS workspace built as in {doc}`getting_started`, the same tools run as
`ros2 run vectorfield_py vf-tune` (and so on), with the same arguments.

Work in a scratch directory, starting from the differential template of {doc}`your_robot`:

```bash
mkdir -p ~/vf_design && cd ~/vf_design
```

```bash
cp ~/vectorfield_nav2/vectorfield_bringup/config/examples/nav2_params_vectorfield_differential.yaml nav2_params.yaml
```

With ROS, copy it from
`$(ros2 pkg prefix vectorfield_bringup)/share/vectorfield_bringup/config/examples/` instead. Use
your own `nav2_params.yaml` the same way once it has the controller in it.

## 2. Validate the parameter file

```bash
vf-tune nav2_params.yaml --costmap 3,3,0.05
```

`vf-tune` reads `controller_server.ros__parameters.FollowPath` and `controller_frequency`. The
costmap rules (braking distance, contour distances and collision horizon against half the local
costmap) need the local costmap's width, height and resolution: from `local_costmap` when the
file has it, from `--costmap` otherwise. For the template:

```text
configuration: nav2_params.yaml (FollowPath)
  parameters set: collision_check_horizon, control_point_offset, controller_frequency, ...
  ROS-only, not checked here: plugin
  costmap rules for a 3 x 3 m window at 0.05 m
validate_config: 0 errors, 0 warnings
```

Try a change on top of the file with `--set`. A gain of 5 passes with a warning:

```bash
vf-tune nav2_params.yaml --costmap 3,3,0.05 --set convergence_gain=5.0
```

```text
validate_config: 0 errors, 1 warnings
  warning [fl_gain_offset]: G'(0) d = 0.63662 > 0.5: the field turns faster than feedback linearisation's heading can follow
      suggestion: lower convergence_gain below 3.93 or control_point_offset below 0.157 m
```

The exit status is 0 for a valid configuration (warnings allowed) and 2 for an invalid one, so
`vf-tune` can gate a CI job on your robot's parameter file.

## 3. Describe a representative path

The controller follows whatever the planner returns, so check it against the kind of path your
robot will see. Here: 4 m along +x, then a sharp left turn and 3 m along +y, as an `x,y` CSV:

```bash
printf 'x,y\n0,0\n4,0\n4,3\n' > route.csv
```

```bash
vf-curve --curve route.csv
```

```text
curve        polyline of 3 vertices
closed       no (open: extended by rays)
moving       no
length       7.0000 m
extent       x [0.000, 4.000], y [0.000, 3.000] m
regular      yes (make_curve validation passed)
kappa_path   61.820 1/m (max turning angle per mean segment length)
samples      401, spacing 0.0125 to 0.0175 m
vertices     3 in, mean spacing 0.0500 m
```

A curve can also be a built-in shape with parameters (`ellipse:a=2,b=1`), an expression pair
(`'2*cos(s); sin(s)'`, with `--s-range`), or a `vectorfield_msgs/Curve` YAML. `vf-curve --list`
lists the shapes; `--plot -o curve.svg` draws the curve.

## 4. Per-plan advice

With `--curve`, `vf-tune` also runs the per-plan advice that the controller logs once per goal:

```bash
vf-tune nav2_params.yaml --curve route.csv
```

```text
curve: polyline of 3 vertices, kappa_path 61.820 1/m
advise_for_plan: 0 errors, 2 warnings
  warning [fl_path_curvature]: kappa_path 61.8197 * control_point_offset 0.2 > 1: the control point cuts the path's corners
      suggestion: lower control_point_offset below 0.0162 m or smooth the path
  warning [yaw_rate_for_curvature]: desired_speed * (c_G k_f + kappa_path) = 24.8763 > max_angular_vel 1.5 for this plan
      suggestion: lower desired_speed below 0.0238 m/s, or raise max_angular_vel
```

A polyline corner has an unbounded curvature, so both warnings fire whatever the parameters:
the robot cannot turn through a sharp corner at speed. With such plans it cuts the corner by
about `control_point_offset` and may turn in place at the corner (ROTATE_TO_FIELD). Plans from
Nav2's planners are smoother; the advice is useful for the tightest curvature they produce.

## 5. Look at the field

`vf-plot` draws the field around the path as arrows and streamlines. With `--pose`, an open path
or obstacles, it draws the field the controller commands from that pose, the terminal field and
the obstacle contour included. Here with a 0.4 m box on the path, 2 m from the start:

```bash
vf-plot --curve route.csv --config nav2_params.yaml --box 2,0,0.4,0.4 --pose 0.5,0,0 --extent -1,5,-2,4 --grid 15 --seeds 16 -o route_field.svg
```

:::{figure} ../_static/tutorials/route_field.svg
:alt: The field around an L-shaped route with a box on its first leg: far from the route the streamlines converge onto it; around the box they circulate past it and rejoin the route.
:width: 60%

The field of `nav2_params.yaml` around `route.csv`, with a 0.4 m box on the first leg and the robot at (0.5, 0).
:::

The dotted and dashed rings around the box are the contour distances $D'_{\mathrm{in}}$,
$D_{\mathrm{in}}$ and $\lambda$ ({doc}`../obstacle_avoidance`). To see why the robot turns where
it does, decompose the field at a point:

```bash
vf-plot --curve route.csv --config nav2_params.yaml --decompose 1,0.5
```

```text
decomposition at (1, 0.5), t = 0 s
  closest point (1.000, 0.000), D 0.500 m, G 0.500, H 0.866, eta 0.400
  -eta G n   ( 0.000, -0.200)  |0.200| m/s
  eta H T    ( 0.346,  0.000)  |0.346| m/s
  Phi_T      ( 0.000,  0.000)  |0.000| m/s
  F          ( 0.346, -0.200)  |0.400| m/s
```

At 0.5 m from the path the convergence term $-\eta G n$ pulls toward it at 0.2 m/s, the
circulation term $\eta H T$ moves along it at 0.346 m/s, and their sum $F$ has the desired speed,
0.4 m/s ({doc}`../theory`).

## 6. Simulate

`vf-sim` closes the loop: the controller's tracker on a kinematic model of the base, at the
control rate of the file. From 0.5 m to the right of the start:

```bash
vf-sim --config nav2_params.yaml --curve route.csv --pose 0,-0.5,0 -o route_sim.svg
```

```text
differential (feedback_linearization) on polyline of 3 vertices, v_r 0.4 m/s, k_f 2 1/m, 32.5 s at 20 Hz
  simulated            18.90 s
  final |D|            7.6e-04 m
  max |D| after 9.5 s  0.0861 m
  settle time          10.55 s (|D| <= 0.05 m)
  phases               ROTATE_TO_FIELD 0.70 s, FOLLOW 16.55 s, FINAL_APPROACH 1.60 s, ROTATE_TO_HEADING 0.05 s
  status bits          none
  final pose           x 3.999 m, y 2.879 m, yaw 89.8 deg
  goal distance        0.121 m (settled)
```

:::{figure} ../_static/tutorials/route_sim.svg
:alt: Left, the robot's path converging from 0.5 m right of the route onto its first leg, turning at the corner and stopping at the goal. Right, the distance to the route on a log scale, the speed and the yaw rate over time.
:width: 100%

`vf-sim` on the route: the path (left) and the distance to the route, the speed and the yaw rate (right). The speed drops to zero at the corner while the robot turns in place, and at the goal.
:::

The summary confirms the per-plan advice: the robot turns in place at the corner
(ROTATE_TO_FIELD for 0.7 s, the speed drop at about 10 s) and stops within the goal tolerance.
The same run with the box on the path:

```bash
vf-sim --config nav2_params.yaml --curve route.csv --pose 0,-0.5,0 --box 2,0,0.4,0.4 -o route_sim_box.svg
```

:::{figure} ../_static/tutorials/route_sim_box.svg
:alt: The same route with a box on its first leg: the robot leaves the route before the box, passes it on the right at about the contour distance, and rejoins the route before the corner.
:width: 100%

The same run with the box: the contour episode is drawn in orange.
:::

The summary adds `closest obstacle 0.469 m from base_link` and `contour active 3.95 s`. With
`--obstacle-mode none` the robot stops in front of the box instead: the summary reads
`status bits COLLISION_AHEAD, BLOCKED` and `goal distance 3.996 m (not settled)`, the situation in
which the Nav2 controller raises `NoValidControl` and Nav2 replans. `--model` simulates another
base (`skid_steer` adds the simulator's slip), `--gif` writes an animation and `--csv` the
recorded columns.

## 7. Sweep a parameter

`--sweep NAME=START:STOP:STEP` (or `NAME=V1,V2,...`) simulates each value on the curve and
reports the worst distance to the curve after the transient, the final distance and the settle
time. The convergence gain on a 2 m × 1 m ellipse:

```bash
vf-tune nav2_params.yaml --sweep convergence_gain=1:5:1 --curve ellipse:a=2,b=1
```

```text
sweep convergence_gain (now 2.0) on builtin ellipse (a=2, b=1)
       value   max |D| (m)  final |D| (m)  settle (s)
           1        0.0034         0.0001       12.60
           2        0.0018         0.0002        7.10
           3        0.0015         0.0003        5.40
           4        0.0012         0.0002        4.60
           5        0.0011         0.0002        4.15
```

Higher gains converge faster. The table does not show the validator's findings, though: 4 and 5
are above the 3.93 limit of the `fl_gain_offset` warning of step 2, so 3 is the best choice here
without also lowering `control_point_offset`. Repeat `--sweep` to
sweep several parameters one after the other (not as a grid), for example the control point on
the route:

```bash
vf-tune nav2_params.yaml --sweep control_point_offset=0.1,0.2,0.3 --curve route.csv --pose 0,-0.5,0
```

```text
sweep control_point_offset (now 0.2) on polyline of 3 vertices
       value   max |D| (m)  final |D| (m)  settle (s)
         0.1        0.0879         0.0003       11.10
         0.2        0.0861         0.0008       10.55
         0.3        0.0815         0.0018       11.05
```

Write the values you pick into `nav2_params.yaml`, run step 2 again, and deploy the file as in
{doc}`your_robot`.

## 8. Curves for the follower

A curve checked with `vf-curve` can be exported as a `vectorfield_msgs/Curve` YAML, the `curve`
field of a `FollowCurve` goal ({doc}`curve_following`):

```bash
vf-curve --curve lemniscate_gerono:a=2,b=1 --export gerono.yaml
```

`--export` with a `.csv` name writes the samples instead, and `--polyline` exports the samples
as a polyline curve.

## Reproducing the figures

The figures on this page were made with the commands of steps 5 and 6, run in `~/vf_design` with
the differential template as `nav2_params.yaml`, and copied to `docs/_static/tutorials/`.
