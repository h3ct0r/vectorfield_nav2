# Your robot

This tutorial puts the vector-field controller into an existing Nav2 setup: a robot that already
navigates with Nav2 (map, localization, costmaps, planner, behaviour trees) and another
controller. Only the controller and the settings around it change. It assumes you have run the
demos of {doc}`getting_started`.

## 1. Build the packages for the robot

Until the binary packages are released, build from source in the robot's workspace
({doc}`getting_started`, step 2). The robot needs only the packages of the `vectorfield_nav2`
metapackage (the core, the controller, the curve follower, the messages, the simulator library
and the Python tools); `vectorfield_bringup` and its simulators are for development machines.
Install their dependencies only:

```bash
cd ~/vf_ws && rosdep install --ignore-src -y --from-paths src/vectorfield_nav2/{vectorfield_core,vectorfield_msgs,vectorfield_ros,vectorfield_nav2_controller,vectorfield_curve_follower,vectorfield_py,vectorfield_sim,vectorfield_nav2}
```

```bash
cd ~/vf_ws && colcon build --packages-up-to vectorfield_nav2
```

The examples below are in `vectorfield_bringup/config/examples/`, in the repository or, once
`vectorfield_bringup` is built, under
`$(ros2 pkg prefix vectorfield_bringup)/share/vectorfield_bringup/config/examples/`.

| File | For |
|---|---|
| `nav2_params_vectorfield_differential.yaml` | differential bases; skid-steer bases with `motion_model: "skid_steer"` and `min_in_place_angular_vel: 0.25` |
| `nav2_params_vectorfield_omnidirectional.yaml` | omnidirectional (holonomic) bases |
| `nav2_params_vectorfield_ackermann.yaml` | car-like bases, with the planner, goal checker and behaviour tree they need |
| `rotation_shim_humble_jazzy.yaml`, `rotation_shim_lyrical.yaml` | the controller behind Nav2's rotation shim |

Each template holds the blocks to merge into your own `nav2_params.yaml`: `controller_server`
(the goal checker and the complete `FollowPath` block), `velocity_smoother` and
`collision_monitor`, with comments. Every value in them passes the configuration validator
without warnings with the stock 3 m × 3 m local costmap and a TurtleBot-sized footprint; a CI
test keeps it that way.

## 2. Replace the FollowPath plugin

In your `nav2_params.yaml`, replace the `FollowPath` block of `controller_server` (whatever
controller it runs now) with the template's, and keep the plugin name: Nav2's behaviour trees
call the controller by its id, `FollowPath`. Remove every key of the previous controller from the
block, since they mean nothing to this one. The differential template:

```{literalinclude} ../../vectorfield_bringup/config/examples/nav2_params_vectorfield_differential.yaml
:language: yaml
:start-at: "controller_server:"
:end-before: "# The controller's command goes through the smoother."
```

Parameters that you do not list keep their defaults ({doc}`../parameters`). They are prefixed
with the plugin name on the node: `FollowPath.desired_speed` for `ros2 param`.

## 3. Choose the motion model and the tracking law

`motion_model` is read-only: set it in the file. `tracking_law: auto` picks the law that matches
it, and every other combination is rejected at configure time.

| Base | `motion_model` | `auto` law | Notes |
|---|---|---|---|
| Differential drive | `differential` | feedback linearisation at a control point `control_point_offset` (0.2 m) ahead | `heading_tracking` evaluates the field at `base_link` instead |
| Skid-steer (tracks, four wheels) | `skid_steer` | as differential | set `min_in_place_angular_vel` 0.25 to 0.3 rad/s |
| Omnidirectional (mecanum, omni wheels) | `omnidirectional` | holonomic: the base velocity is the field | `max_lateral_vel`; yaw from `omni_heading_policy` |
| Car-like | `ackermann` | heading tracking at the rear axle | `min_turning_radius` > 0; no in-place rotation, no contour |

A robot that drives backwards by design (the sensor head at the rear) sets `drive_backward: true`.
{doc}`../motion_models` explains the laws and the phases.

## 4. Set the minimum parameters

1. **Limits.** `max_linear_vel`, `max_angular_vel`, `max_linear_accel`, `max_linear_decel` and
   `max_angular_accel` are the base's real limits (`max_lateral_vel` for an omnidirectional
   base, `min_turning_radius` for a car). When one binds, the controller scales the whole command,
   so the direction, and the path's curvature, is kept.
2. **Speed.** `desired_speed` is the speed along the path; it must not exceed `max_linear_vel`.
   Start at about half of it on a new robot.
3. **Footprint.** The controller takes the footprint from the local costmap (`footprint` or
   `robot_radius` in `local_costmap`); do not set it anywhere else. The contour distances follow
   from it when left on auto (−1).
4. **Costmap size.** The validator checks the braking distance, the contour distances and the
   collision horizon against half the local costmap's width and height. The defaults fit the
   stock 3 m × 3 m; a faster robot needs a larger local costmap.
5. **Control rate.** The defaults are tuned at `controller_frequency: 20.0`. The
   `discrete_overshoot` warning appears if the gain is too high for the rate.

{doc}`../tuning` gives the order in which to tune the rest.

## 5. Velocity smoother and collision monitor

Nav2 sends the controller's command through the velocity smoother, and on Jazzy and later
through collision_monitor, before it reaches the base:
`controller_server` (`cmd_vel_nav`) → `velocity_smoother` (`cmd_vel_smoothed`) →
`collision_monitor` → `cmd_vel`.

**Velocity smoother.** Its limits must be at least the controller's on every axis, decelerations
negative. Lower limits clip single axes, which changes the commanded curvature and bends the
path. `scale_velocities: true` scales all axes together when a limit binds anyway. The stock
Humble file limits the smoother to 0.26 m/s and 1.0 rad/s, below the controller's defaults, so
raise them there. For an omnidirectional base, open the y axis.

```{literalinclude} ../../vectorfield_bringup/config/examples/nav2_params_vectorfield_differential.yaml
:language: yaml
:start-at: "velocity_smoother:"
:end-before: "# Jazzy and later"
```

**Collision monitor.** Keep your distro's stock `collision_monitor` section. The stock approach
polygon slows the robot when its footprint would reach a scan point within
`time_before_collision` (1.2 s); the controller has its own braking cap on the local costmap, so
the monitor is a second line of defence. If you add a `limit` polygon, give it
`linear_limit` ≥ `max_linear_vel` and `angular_limit` ≥ `max_angular_vel`, unless the cap is
intended. Humble's `navigation_launch.py` has no collision_monitor: add one between the smoother
and the base as the demos do (`config/humble/collision_monitor.yaml`, `cmd_vel` → `cmd_vel_safe`,
with the base listening on `cmd_vel_safe`).

## 6. Choose the goal checker

| Goal checker | Use it for | What the controller does at the goal |
|---|---|---|
| `nav2_controller::SimpleGoalChecker` (position and yaw) | differential, skid-steer and omnidirectional bases | stops on the goal, then turns in place to the goal yaw (ROTATE_TO_HEADING) |
| `nav2_controller::PositionGoalChecker` (position only) | car-like bases, and any robot whose final heading does not matter | stops on the goal and holds |

Keep `stateful: true`. The controller stops within half the XY tolerance of the goal, and its
XY test never disagrees with the goal checker's ({doc}`../motion_models`, "Settled"). The
Ackermann template replaces the goal checker block:

```{literalinclude} ../../vectorfield_bringup/config/examples/nav2_params_vectorfield_ackermann.yaml
:language: yaml
:start-at: "    general_goal_checker:"
:end-before: "    FollowPath:"
```

A car also needs a planner that returns drivable, forward-only paths and a behaviour tree without
Spin; the Ackermann template sets both (`planner_server`, `bt_navigator`), and
{ref}`ackermann-caveats` explains why.

## 7. Behind the rotation shim

The controller can run as the primary controller of Nav2's `RotationShimController`. The shim
then performs the initial rotation, so set `use_rotate_to_field: false`. The key layout depends
on the distro:

| Distro | Primary controller | Vector-field keys | Example |
|---|---|---|---|
| Humble, Jazzy | `FollowPath.primary_controller: "vectorfield_nav2_controller::VectorFieldController"` | `FollowPath.<key>`, next to the shim's | `rotation_shim_humble_jazzy.yaml` |
| Lyrical and later | `FollowPath.primary_controller.plugin: "vectorfield_nav2_controller::VectorFieldController"` | `FollowPath.primary_controller.<key>` | `rotation_shim_lyrical.yaml` |

On Humble and Jazzy the shim and the controller share `rotate_to_heading_angular_vel` (the shim's
default is 1.8 rad/s) and `max_angular_accel`; keep the first at or below `max_angular_vel`. On
Lyrical, set the shim's `rotate_to_goal_heading: true`: its shim re-checks the heading every
cycle and otherwise fights the controller's final rotation to the goal yaw ({doc}`scenarios`). The
shim undoes obstacle-contour episodes, so `obstacle_mode: contour` behind it is supported only on
Jazzy with `FollowPath.rotate_to_heading_once: true`. {doc}`scenarios` runs both examples in
the demo, and {doc}`../motion_models` has the details.

## 8. Validate the configuration

Before starting the robot, check the file with `vf-tune` (part of the Python tools; with ROS it
also runs as `ros2 run vectorfield_py vf-tune`):

```bash
vf-tune nav2_params.yaml
```

It reads `controller_server.ros__parameters.FollowPath` and `controller_frequency`, takes the
costmap size from the file's `local_costmap`, runs the same validator as the controller and
prints each finding with its code and a suggested change, for example:

```text
validate_config: 1 errors, 0 warnings
  error [desired_speed_above_max_linear_vel]: desired_speed 0.7 > max_linear_vel 0.5: the field speed must be commandable
      suggestion: lower desired_speed to 0.5 or raise max_linear_vel to 0.7
```

The exit status is 0 for a valid configuration (warnings allowed) and 2 for an invalid one.
`--curve` adds the per-plan advice for a representative path ({doc}`offline_design`).

At run time the controller validates again when it is configured, with the real costmap and
footprint, and logs:

- `FollowPath: vector-field controller configured (…, differential, feedback_linearization)`
  when the configuration is valid;
- `FollowPath: configuration: <message> [<code>]` for each warning;
- `FollowPath: invalid configuration: …` with the errors, and controller_server fails to
  configure, so the lifecycle manager does not bring Nav2 up.

The codes are explained in {doc}`../tuning` ("Validator findings").

## 9. Bring-up checklist

1. `vf-tune nav2_params.yaml` exits with 0 and no warnings you have not accepted.
2. controller_server logs `vector-field controller configured` with the motion model you
   expect, and `ros2 lifecycle get /controller_server` prints `active [3]`.
3. TF has `map` → `odom` → the costmap's `robot_base_frame`, and the odometry twist has the
   right signs: drive forward and turn left by hand, and watch
   `ros2 topic echo /odom --field twist.twist`.
4. The local costmap's footprint matches the robot (RViz: the published footprint).
5. The velocity smoother's limits cover the controller's, and the command chain ends at the base
   (`ros2 topic info -v /cmd_vel`: collision_monitor publishes it on Jazzy and later).
6. The goal checker fits the base (section 6).
7. In RViz, add a MarkerArray display on `/FollowPath/markers` (or open
   `vectorfield_bringup`'s `rviz/vectorfield.rviz`): the status text shows the phase.
8. First goals: at a low `desired_speed`, in open space, a few metres ahead, then behind the
   robot, then through doors and corridors.

## 10. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| The robot turns in place at the start of a goal | The plan starts behind it: ROTATE_TO_FIELD runs until the field is within 45° of the heading | Expected, once per goal or sharp corner. With the rotation shim, set `use_rotate_to_field: false` so only one of them rotates |
| It keeps turning and never leaves ROTATE_TO_FIELD | The heading the controller sees is wrong: odometry or TF yaw with the wrong sign, or a `base_link` that does not face forward | Check TF and `odom` (checklist step 3) |
| A skid-steer base stalls in in-place rotations | Static friction of the wheels | `min_in_place_angular_vel` 0.25 to 0.3 rad/s |
| It oscillates across the path | `convergence_gain` too high for the speed and the control rate (`discrete_overshoot`, `fl_gain_offset` warnings), or a control rate below `controller_frequency` | Lower `convergence_gain`; check the measured rate of controller_server |
| It is slower than `desired_speed`, or bends the path in turns | The velocity smoother's limits are below the controller's (the stock Humble file: 0.26 m/s) | Raise the smoother limits on every axis; `scale_velocities: true` |
| An omnidirectional base does not move sideways | The smoother's y limits are 0, or `min_y_velocity_threshold` is 0.5 | Use the omnidirectional template's smoother limits and `min_y_velocity_threshold: 0.001` |
| It stops near obstacles that are not on the path | The braking cap sees them in the rollout, the clearance slowdown, or the collision monitor's approach polygon | Shorten `collision_check_horizon` within the validator's bounds; `clearance_slowdown_distance: 0`; check the monitor's polygons |
| It stops in front of an obstacle on the path, status `BLOCKED` | `obstacle_mode: none`, or no side passes the obstacle; the controller raises `NoValidControl` and Nav2 replans | Expected with `none`; set `obstacle_mode: contour` to drive around obstacles ({doc}`../obstacle_avoidance`) |
| Nav2 logs `NoValidControl` with `vector field: …` | The reason follows the prefix: `no free path ahead` (blocked), `the robot footprint is in collision` (localization or footprint), `the goal heading cannot be reached` (Ackermann), `parameters rejected: …` (an invalid configuration after a change) | Fix the cause named; for parameters, `vf-tune` on the current values |
| `InvalidPath`: `plan contains reversing segments; use a forward-only planner` | The planner returned a path with cusps (for example Smac with Reeds-Shepp motions); the controller never reverses along a plan | Use a forward-only planner: NavFn, Smac 2D, or Smac Hybrid-A* with `DUBIN` |
| Slow, creeping approach at the goal | `min_approach_speed` too low, a long `terminal_braking_distance`, or the braking cap binds near a wall at the goal | Raise `min_approach_speed`; keep `terminal_braking_distance` near `desired_speed²/max_linear_decel`; check `stop_margin` |
| It overshoots the goal | `terminal_braking_distance` below `desired_speed²/max_linear_decel` (`terminal_braking_too_short`) | Raise `terminal_braking_distance` or `max_linear_decel` |
| A car stops with `the goal heading cannot be reached` | The goal yaw or the plan needs a turn tighter than `min_turning_radius` | Use `PositionGoalChecker`, a `DUBIN` planner with the same radius, and goals the car can reach |
| controller_server fails to configure | A validator error (`invalid configuration: …` in its log) | Fix the parameter named by the code; `vf-tune` prints a suggestion |

The {doc}`../tuning` has more symptoms, and {doc}`offline_design` shows how to reproduce a
problem without the robot.
