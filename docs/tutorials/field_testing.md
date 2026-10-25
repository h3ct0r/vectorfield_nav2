# Field testing

Everything in this repository has been tested in simulation: the kinematic simulator, Nav2's
loopback simulator and the TurtleBot Gazebo worlds. This tutorial is the procedure for the first
runs on a real robot: a safety checklist, a configuration check before the robot moves, then nine
short stages from turning in place to localisation jumps. Each stage says what to run, what the
robot and RViz should show, which numbers to compute and what to change when they are off. The
recordings and the reports are what to send back for analysis.

It assumes the robot already runs Nav2 with the vector-field controller ({doc}`your_robot`) and,
for stage 5, the curve follower ({doc}`curve_following`).

## The kit

Three scripts of `vectorfield_bringup` do the bookkeeping:

| Script | What it does |
|---|---|
| `record_field_test.py` | Records the topics the metrics need into a rosbag2 bag named `field_test_<date>_<time>_<label>`, plus a dump of the controller, follower, smoother, collision-monitor and costmap parameters |
| `field_test_report.py` | Reads a bag (sqlite3 or mcap) and writes `report.md`, `metrics.json` and one plot per goal: tracking errors, times, jerk, phases, status flags, clearance, speed limits, localisation jumps, each with a pass or fail against the thresholds below |
| `identify_skid_slip.py` | Drives a guarded sequence of turns and estimates the skid-steer slip parameters (stage 6); also runs on a recorded bag |

Two clients send the goals: `navigate_waypoints.py` (NavigateToPose per pose; with `--relative`
the poses are in the robot's frame at the start, x ahead and y to the left; with `--through`
the route is one NavigateThroughPoses goal) and `send_curve.py` (the curve follower). All five
print their options with `--help`.

## Safety checklist

Go through it at the start of every session, not once.

1. **E-stop.** Test the hardware e-stop before the first goal: the robot must stop, and Nav2 must
   not resume driving by itself when it is released. The spotter holds the (wireless) e-stop for
   the whole session. Ctrl-C in `navigate_waypoints.py` or `send_curve.py` cancels the goal, and
   `ros2 action send_goal` can be cancelled with Ctrl-C too, but a software cancel is not an
   e-stop.
2. **Speed caps, three layers.**
   - The controller: start with `desired_speed: 0.25` and `max_linear_vel: 0.3` (the defaults are
     0.4 and 0.5), `rotate_to_heading_angular_vel: 0.8`. Raise them stage by stage.
   - The velocity smoother is the hard cap on everything Nav2 and the follower send:
     `max_velocity`, `min_velocity`, `max_accel` and `max_decel` at what the robot can safely do,
     at least as high as the controller's limits on every axis ({doc}`your_robot`, section 5).
   - `/speed_limit` lowers the speed at run time; the controller, the follower and
     `identify_skid_slip.py` all obey it (stage 8). Publish it a few times (`-t 3`, one per
     second): `--once` sends a single message as soon as one subscriber (the recorder, say) has
     matched, and the others can miss it.

     ```bash
     ros2 topic pub -t 3 /speed_limit nav2_msgs/msg/SpeedLimit "{percentage: false, speed_limit: 0.2}"
     ros2 topic pub -t 3 /speed_limit nav2_msgs/msg/SpeedLimit "{percentage: false, speed_limit: 0.0}"  # no limit
     ```

3. **Collision monitor.** Its polygons must fit the robot: the stock `FootprintApproach`
   (`time_before_collision` 1.2 s on the published footprint), or a `Stop` polygon around the
   footprint plus 0.1 to 0.2 m and a `Slowdown` polygon ahead. Check it before the first goal:
   teleoperate slowly towards a box through the same command chain; the robot must stop. Check
   that it publishes the final command: `ros2 topic info -v /cmd_vel` lists `collision_monitor`
   as the only publisher (Jazzy and later; on Humble, wherever your chain ends).
4. **Clear area.** At least 6 m × 6 m of flat floor for stages 1 to 6 (8 m × 8 m at 0.5 m/s), no
   glass or black surfaces at scan height, no people inside. Mark the start pose with tape: the
   goals of the stages are relative to it.
5. **A spotter.** One person on the e-stop with the robot in view, one at the laptop. Agree on the
   stop word. Nobody walks into the area while a goal runs, except on purpose in stage 7.
6. **The robot.** Battery charged (low voltage changes the motor response), tyres and tracks in
   the state they will be used in, the floor surface noted for each bag.

## Pre-flight: configuration

Before the robot moves:

1. **Validate the parameter file offline** with `vf-tune` ({doc}`../tuning`, "vf-tune"):

   ```bash
   vf-tune robot_nav2_params.yaml
   vf-tune robot_nav2_params.yaml --curve ellipse:a=1.5,b=1   # the curve of stage 5
   ```

   The exit status must be 0, and every warning must be one you have read and accepted
   ({doc}`../tuning`, "Validator findings").
2. **Read the configure-time validation.** Start Nav2 and the follower and look for these lines
   in the `controller_server` and `curve_follower` output:
   - `FollowPath: vector-field controller configured (…, <motion model>, <tracking law>)` and
     `curve follower configured (<motion model>, …)`, with the motion model you expect;
   - `FollowPath: configuration: <message> [<code>]`: each warning, as `vf-tune` printed it;
   - `invalid configuration: …`: an error; Nav2 does not come up ({doc}`your_robot`, section 8).
3. **Check the footprint.** The controller takes the footprint from the local costmap: the
   collision rollout, the contour distance $\lambda$ and the CBF margin all depend on it. In RViz,
   the costmap's `published_footprint` and the controller's `footprint` marker (green outline) must
   match the real robot, bumpers included. `ros2 param get /local_costmap/local_costmap footprint`
   (or `robot_radius`) shows the configured values; `record_field_test.py` stores them.
4. **TF, odometry and the command chain**, as in the bring-up checklist of {doc}`your_robot`
   (section 9): `map` → `odom` → `base_link`, twist signs (drive forward and turn left by hand,
   `ros2 topic echo /odom --field twist.twist`), smoother limits at least the controller's, the
   chain ending at the base.
5. **RViz.** Open `vectorfield_bringup`'s `rviz/vectorfield.rviz` (or add MarkerArray displays
   on `/FollowPath/markers` and `/curve_follower/markers`). {doc}`../visualization` explains
   every marker; the status text above the robot shows the phase and the active flags.

## Recording and the report

Record each stage into its own bag, on the robot or on a laptop whose clock is synchronised
with it. Start the recorder in a second terminal before the stage and stop it with Ctrl-C after
it (it closes the bag and adds the parameter dumps), then write the report:

```bash
ros2 run vectorfield_bringup record_field_test.py --label stage2   # terminal 2; Ctrl-C at the end
ros2 run vectorfield_bringup navigate_waypoints.py --relative --poses "3,0,0"   # terminal 1
ros2 run vectorfield_bringup field_test_report.py field_test_20261021_101500_stage2
```

`record_field_test.py` records `/tf`, `/tf_static`, `odom`, `/plan`, `/FollowPath/field_state`,
`/curve_follower/field_state` (and the follower's `curve`), `cmd_vel_nav`, `cmd_vel_smoothed`,
`cmd_vel`, `cmd_vel_safe`, `speed_limit`, `local_costmap/costmap` and `published_footprint`,
`scan`, `amcl_pose`, `initialpose`, `goal_pose`, `/behavior_tree_log`, and the status of the
NavigateToPose, NavigateThroughPoses, FollowPath, ComputePathToPose, FollowCurve and recovery
actions (plus the FollowCurve feedback). Topics that do not exist are skipped. `--markers` adds
the marker topics (stage 7, about 0.5 MB/s more); `--duration` stops after a time; `kill -INT`
or `kill -TERM` stop it like Ctrl-C; `--namespace`, `--odom-topic`, `--scan-topic`
and `--costmap-topic` adapt the names; `--storage mcap` chooses the format. The `field_state`
topics are only published while someone subscribes, which the recorder does, and only with
`visualize: true` (the default).

`field_test_report.py` splits the bag into episodes, one per NavigateToPose, NavigateThroughPoses
or FollowCurve goal, and writes into `<bag>_report/`:

| Metric | Definition |
|---|---|
| Cross-track error (XTE), RMS and max | Distance from `base_link` (`map` frame, from TF) to the first `/plan` of the goal, over the goal; as in {doc}`../controller_comparison` |
| max \|D\| after convergence | The controller's own distance to the path (`field_state.distance`), in FOLLOW and FINAL_APPROACH, from the first sample with \|D\| ≤ 0.05 m on: the criterion of the launch tests |
| \|D\| after lap 1 | The follower's `field_state.distance` after the first lap (FollowCurve feedback): the curve follower's criterion |
| Time | Goal acceptance to result |
| Goal miss, yaw error | Pose 0.5 s after the result against the last pose of the last `/plan` |
| Stop latency | Result (success, cancel or abort) until the odometry twist stays below 0.02 m/s and 0.05 rad/s for 0.3 s |
| RMS jerk | Second differences of the commanded v and w, one sample per control update, per command topic (`cmd_vel_nav` is the controller, `cmd_vel` the final command) |
| Phases | The phase timeline and the time per phase (ROTATE_TO_FIELD, FOLLOW, FINAL_APPROACH, ROTATE_TO_HEADING, STOPPED) |
| ROTATE_TO_FIELD entries | At the start, per corner of the first plan (heading change over 1 m above 45°), elsewhere |
| Status flags | Samples and occurrences of BLOCKED, CONTOUR, ASSUMPTION3_CLAMPED, CURVATURE_SATURATED, COLLISION_AHEAD and the rest; CBF binds and holds when the markers were recorded |
| Clearance | Minimum distance from `base_link` to the lethal cells of the recorded local costmap; minimum scan range; minimum `field_state.obstacle_distance` |
| Speed limits | Per `speed_limit` period, the largest commanded speed on each command topic, 1 s after the change |
| Localisation corrections | `map` → `odom` changes that moved the robot by 5 cm or 2° or more, and the controller's reaction: \|D\| peak and recovery time, ROTATE_TO_FIELD |

Its exit status is 1 when a check failed. The thresholds are options (`--tracking-limit`,
`--lap-limit`, `--xy-tolerance`, `--yaw-tolerance`, `--max-stop-latency`; `--max-linear-vel`
converts percentage speed limits); the defaults are the ones in the tables below.

### Reference numbers

The thresholds start from the simulation results and allow for a real robot's latency, noise and
slip:

| Quantity | Simulation | Field threshold (default) |
|---|---|---|
| max \|D\| after convergence | below 0.10 m on every distro and motion model; 0.25 m for Ackermann | 0.10 m (`--tracking-limit`; 0.25 m for Ackermann) |
| XTE RMS / max, 3-goal route at 0.4 m/s | 0.048 m / 0.097 m (loopback, {doc}`../controller_comparison`) | RMS 0.05 m, max 0.15 m |
| Route time, 3 goals | 24.2 s (loopback) | within 30 % of `distance / desired_speed` plus 2 s per goal |
| \|D\| after lap 1, ellipse and lemniscate at 0.4 m/s | below 0.05 m | 0.10 m (`--lap-limit`) |
| Goal miss / yaw error | `xy_goal_tolerance` + 0.05 m | 0.30 m / 0.25 rad (the stock goal checker) |
| RMS jerk, `cmd_vel_nav` / `cmd_vel` | 12 / 8 m/s³, 48 / 25 rad/s³ (loopback route) | at most twice the simulation |
| ROTATE_TO_FIELD entries | at most one per sharp corner, plus one at the start | the same |
| Stop latency | 0.04 to 0.09 s | 1.0 s |

The reference to compare with is `field_test_report.py` on a kinematic-simulator session
(differential, `curve_follower_demo`, Jazzy, the commands of stages 1 to 3, 8 and 9 and two
ellipse laps): on the 3-goal route XTE RMS at most 0.040 m and max 0.064 m per goal, max \|D\|
after convergence 0.045 m, 24.0 s in total; the 90° turn in place 1.2 s with a yaw error of
0.10 rad; the ellipse \|D\| at most 0.005 m after the first lap; stop latencies 0.05 to 0.4 s.

Record every stage into its own bag as above, with the stage as the label (`--label stage1`,
`--label stage7_cbf`, ...).

## Stage 1: turn in place and stop

**Run.** Goals at the start position with another heading: the robot only turns.

```bash
ros2 run vectorfield_bringup navigate_waypoints.py --relative --poses "0,0,1.57 0,0,0 0,0,3.14 0,0,0"
```

Then once more, and press Ctrl-C during the 180° turn: the goal is cancelled and the robot must
stop.

**Expect.** The status text shows `ROTATE_TO_HEADING` (blue) with the target-heading arrow and the
arc from the current heading, then `STOPPED` (red). The robot turns at up to
`rotate_to_heading_angular_vel`, slows down near the target and does not drift.

**Measure.** Yaw error at the result, stop latency (also after the cancel), goal miss (here: the
drift of the base during the turn), RMS angular jerk.

**Accept.** Yaw error ≤ `yaw_goal_tolerance` (0.25 rad); stop latency ≤ 1.0 s; drift ≤ 0.05 m
(skid-steer: up to twice the ICR offset, see stage 6); no reversal of the turn direction near the
target.

**If it fails.** Stalls or jerky start on a skid-steer base: `min_in_place_angular_vel` 0.25 to
0.3 rad/s. Overshoot and oscillation around the target: lower `rotate_to_heading_angular_vel`,
check `max_angular_accel` against the smoother's angular `max_decel` and the command latency. It
turns the wrong way or never settles: the odometry yaw sign ({doc}`your_robot`, section 10).
Slow stop after a cancel: the velocity smoother's `max_decel`, and on Humble the cancel
behaviour of the controller server ({doc}`../tuning`).

## Stage 2: straight line of 3 m

**Run.**

```bash
ros2 run vectorfield_bringup navigate_waypoints.py --relative --poses "3,0,0"
ros2 run vectorfield_bringup navigate_waypoints.py --relative --poses "-3,0,3.14"   # back
```

**Expect.** `FOLLOW` (green), then `FINAL_APPROACH` (cyan) over the last
`terminal_braking_distance`, `STOPPED`. The return goal starts with one ROTATE_TO_FIELD (amber)
because the plan starts behind the robot. In RViz the teal window lies on the grey plan, the
magenta control point stays on it, the green rollout of the `prediction` namespace reaches ahead
without a red collision cube, and the command arrow points along the path.

**Measure.** XTE RMS and max, max \|D\| after convergence, time, goal miss, jerk of `cmd_vel_nav`
and `cmd_vel`, the phase timeline.

**Accept.** max \|D\| after convergence < 0.10 m; XTE RMS ≤ 0.05 m; goal miss ≤ 0.30 m; time
within 30 % of 3 m / `desired_speed` + 2 s; no NoValidControl, no FollowPath abort.

**If it fails.** Weaving across the line: `convergence_gain` too high for the speed and the
control rate, or a measured control rate below `controller_frequency` (the report lists the
command period); lower the gain ({doc}`../tuning`, "Symptoms and fixes"). A steady offset to one
side: odometry calibration (wheel radius, track width), or slip on a skid-steer base (stage 6).
Creeping at the goal: `min_approach_speed`. Overshoot: `terminal_braking_distance` ≥
`desired_speed²/max_linear_decel`. Jerk on `cmd_vel` much higher than on `cmd_vel_nav`: the
smoother limits clip single axes; `scale_velocities: true`.

## Stage 3: corners of 90° and 150°

**Run.** A sharp 90° left corner and then a 150° corner, each at a NavigateThroughPoses waypoint
2 m ahead (the second leg of the second route leaves the waypoint at 150° from the first):

```bash
ros2 run vectorfield_bringup navigate_waypoints.py --relative --through \
    --poses "2,0,0 2,2,1.57"
ros2 run vectorfield_bringup navigate_waypoints.py --relative --through \
    --poses "2,0,0 0.27,1,2.62"
```

Both are forward corners, not cusps: the controller follows them with `allow_reversing: false`
and never backs up ({ref}`reversing`; before 0.2.1 a corner sharper than 90° after chord
decimation, which the grid planner's few degrees at a waypoint can make of a 90° one, was rejected
as `plan contains reversing segments`). The stock NavigateThroughPoses trees drop a waypoint
once the robot is within 0.7 m of it, and the replanned path then leaves from the robot.

**Expect.** At the 90° corner the robot cuts the corner by about `control_point_offset`
(feedback linearisation) and keeps driving; a sharp corner can need one rotation in place,
briefly. At the 150° corner the field at the control point points more than
`rotate_to_field_angle` away from the heading, so it turns in place once (amber ROTATE_TO_FIELD
with the heading arc) and then follows the second leg. A second ROTATE_TO_FIELD at the same corner
is a defect worth reporting.

**Measure.** ROTATE_TO_FIELD entries at the start and per plan corner (the report lists the
corners it found in the first `/plan`), XTE max at the 90° corner, max \|D\| after convergence,
time, jerk.

**Accept.** At most one ROTATE_TO_FIELD at each corner; XTE max ≤ `control_point_offset` +
0.05 m; all goals SUCCEEDED, with no `InvalidPath`. \|D\| is measured at the control point, which
swings off the path at a corner while `base_link` stays on it: the report's max \|D\| check can
fail at the 90° corner by design, so compare it with the simulator rather than with 0.10 m.

**If it fails.** Repeated entries at one corner: lower `rotate_to_field_release_angle` or check
the angular limits ({doc}`../motion_models`, phases). Corners cut too much: lower
`control_point_offset` or use `tracking_law: heading_tracking`. Overshoot after the corner: lower
`convergence_gain`.

## Stage 4: final approach and goal yaw

**Run.** Goals whose heading differs from the approach direction:

```bash
ros2 run vectorfield_bringup navigate_waypoints.py --relative --poses "2,0,1.57 2,1,3.14 0,0,0"
```

**Expect.** `FINAL_APPROACH` (cyan, with the goal sphere and the goal-yaw arrow), the speed
dropping to `min_approach_speed` over the last `terminal_braking_distance`, then
`ROTATE_TO_HEADING` (blue) at the goal and `STOPPED`. With a `PositionGoalChecker` there is no
final turn.

**Measure.** Goal miss, yaw error, time in FINAL_APPROACH and ROTATE_TO_HEADING (time per phase),
stop latency.

**Accept.** Goal miss ≤ `xy_goal_tolerance` + 0.05 m; yaw error ≤ `yaw_goal_tolerance`;
FINAL_APPROACH shorter than about 2 × `terminal_braking_distance` / `min_approach_speed`; stop
latency ≤ 1.0 s.

**If it fails.** Overshoot, then a turn back: `terminal_braking_distance` too short
(`terminal_braking_too_short`) or `max_linear_decel` above what the base does. Long creep: raise
`min_approach_speed`. Oscillating final turn: lower `rotate_to_heading_angular_vel`. A goal that
never succeeds although the robot is still: the goal checker's tolerances against the
localisation noise.

## Stage 5: curve follower, circle and lemniscate

**Run.** The curves are in `map`; place them in the open area with the `cx` and `cy` shape
parameters. Read the robot's position (`ros2 run tf2_ros tf2_echo map base_link`); for a robot
at (2.0, -1.0), a circle of radius 1 m through it and a lemniscate centred on it:

```bash
ros2 run vectorfield_bringup follow_curve_client.py --shape ellipse --param a=1.0 --param b=1.0 \
    --param cx=3.0 --param cy=-1.0 --laps 3 --speed 0.3
ros2 run vectorfield_bringup follow_curve_client.py --shape lemniscate_bernoulli --param a=1.5 \
    --param cx=2.0 --param cy=-1.0 --laps 2 --speed 0.3
```

`send_curve.py` sends the same shapes centred on the map origin and drives to the curve first.

**Expect.** The `curve` marker (the whole curve) with the `curve_info` arrows and `lap n/3`
text; the robot converges within the first metres and then stays on the curve; the lemniscate's
crossing is passed straight.

**Measure.** \|D\| after lap 1 (RMS and max), laps completed, time per lap, status flags
(ASSUMPTION3_CLAMPED and CURVATURE_SATURATED must not appear on static curves), jerk, stop
latency after the result or after Ctrl-C (`--laps 0`).

**Accept.** \|D\| after lap 1 < 0.10 m (simulation: below 0.05 m at 0.4 m/s); every requested
lap completed; no NoValidControl.

**If it fails.** A steady offset outside the curve on a skid-steer base: slip, stage 6. Inside
the curve on a differential base: the odometry's track width is too large. A loop that grows
with speed: control latency; lower `convergence_gain` or the speed. CURVATURE_SATURATED on an
Ackermann base: a curve tighter than `min_turning_radius` allows, or lower `convergence_gain`.

## Stage 6: skid-steer slip

Skid-steer bases (tracks, four fixed wheels) realise only part of the commanded yaw rate and turn
about a point that is not `base_link`: the controller then runs a steady offset outside curves
({ref}`skid-slip`). `identify_skid_slip.py` measures both effects.

**Run.** In the open area, with a pose source that sees the slip: plain wheel odometry computes
the motion from the wheel speeds and cannot. Use the localisation pose (`--pose-topic
/amcl_pose`, best with a good map), motion capture, or odometry fused with a gyro
(`robot_localization`'s `/odometry/filtered`):

```bash
ros2 run vectorfield_bringup identify_skid_slip.py --v 0.2 --w 0.6 --pose-topic /odometry/filtered
```

It prints the four segments (turn in place left and right, arcs of radius `v/w` left and right,
4 s each), the farthest excursion and the stop conditions, and asks for `yes`. It stops with a
zero command on Ctrl-C, when the pose stops arriving, beyond `--max-distance` (2 m) from the
start or after `--max-time` (90 s), and scales its commands by `/speed_limit`. The commands go to
`cmd_vel_nav` (through the smoother and the collision monitor); the commanded value is read back
from `cmd_vel`. Repeat with `--repeats 2` and at the speeds the robot will use (`--v 0.4 --w
1.0`): the slip grows with the speed on most surfaces.

**Expect.** Per segment the realised over commanded yaw rate $\eta_\psi$ and the ICR offset
$x_{\mathrm{ICR}} = -v_y/\omega$, then the pooled values and the lines to paste:

```text
segment  v_cmd   w_cmd  w_real  eta_psi   x_icr   eta_v  pairs  note
      1  0.000   0.600   0.450    0.750   0.050     nan     26
      2  0.000  -0.600  -0.450    0.750   0.050     nan     27
      3  0.200   0.600   0.450    0.750   0.050   1.000     27
      4  0.200  -0.600  -0.450    0.750   0.050   1.000     27

eta_psi (skid_yaw_rate_efficiency) = 0.750 (spread 0.000)
x_icr   (skid_icr_offset)          = 0.050 m (spread 0.000)
eta_v   (forward speed, diagnostic) = 1.000

Parameters (FollowPath and the curve follower):

  # controller_server.ros__parameters.FollowPath and the curve follower:
  skid_yaw_rate_efficiency: 0.750
  skid_icr_offset: 0.050
```

That is `kinematic_sim_demo` with `motion_model:=skid_steer`, whose plant slips with 0.75 and
0.05 m; the offline estimate on the bag recorded during the run gives the same values, and the
launch test `test_skid_slip_kinematic` checks them within 10 %. `--bag <bag>` runs the same
estimate on a recorded bag: on the identification run above, or on any bag with segments of
constant command of at least `--min-segment` (2 s).

**Accept.** The spread across segments below 0.05 for $\eta_\psi$ and 0.02 m for
$x_{\mathrm{ICR}}$ (larger: the surface or the speed matters, identify at the operating speed);
$\lvert x_{\mathrm{ICR}}\rvert$ below `control_point_offset` / 2 (the controller rejects larger
values). Then set both parameters for the controller and the follower and repeat the stage-5
circle: the steady offset must drop (in simulation from about 8 cm to below 1 cm on a 1 m
circle).

**If it fails.** $\eta_\psi$ close to 1 and $x_{\mathrm{ICR}}$ close to 0 on a base that clearly
slips: the pose source is wheel odometry. A large spread between left and right turns: an
asymmetric drivetrain or a slope. $\eta_\psi > 1$: the commanded reference is not what the base
received (check `--final-topic`) or the odometry yaw is scaled.

## Stage 7: obstacles, contour and CBF

**Setup.** A cardboard box of about 0.4 m × 0.4 m, tall enough for the scan and not black, on the
stage-2 line 2 m from the start. The global planner must not route around it, or the controller
never meets it: for this stage remove the obstacle layer from the global costmap
(`global_costmap.plugins: ["static_layer", "inflation_layer"]`, as the `demo_obstacles`
scenario does), and restore it afterwards. The box must still appear in the local costmap.

**Run.** With `obstacle_mode: contour`, then with `obstacle_mode: cbf` ({doc}`../obstacle_avoidance`),
each recorded with `record_field_test.py --markers` (the CBF counts come from the markers):

```bash
ros2 run vectorfield_bringup navigate_waypoints.py --relative --poses "4,0,0"
# back at the start, switch the mode (or restart with it in the file), new recording
ros2 param set /controller_server FollowPath.obstacle_mode cbf
ros2 run vectorfield_bringup navigate_waypoints.py --relative --poses "4,0,0"
```

Then move the box 0.15 m off the line and repeat, and once walk slowly into the path (the
spotter): the collision monitor and the controller's braking must stop the robot.

**Expect.** Contour: the gate opens when the path ahead is blocked, `CONTOUR ±1` in the status
text, the `contour` markers (obstacle cells, the λ circle, the blend), the robot circles the box
at about λ and rejoins the line. CBF: no circulation; the `contour` namespace shows the
constrained sites, the δ<sub>safe</sub> ring and `cbf h … | n/m bind | %`; the robot slides past
a box beside the path and stops (`BLOCKED`, `| hold`) in front of a box squarely on it, which is
the accepted CBF deadlock: Nav2 then replans or aborts.

**Measure.** Contour episodes (CONTOUR occurrences), CBF binds and holds (needs `--markers`),
minimum clearance to the lethal costmap cells, minimum scan range, BLOCKED samples, time,
FollowPath aborts and recoveries.

**Accept.** Contour: one contour episode per box; minimum clearance (from `base_link`) at least
λ − `control_point_offset` − 0.05 m (with the defaults, about the footprint radius: the
footprint never touches); goal SUCCEEDED; no watchdog (CONTOUR_WATCHDOG). CBF: footprint
clearance at least `cbf_margin` minus one costmap cell (minimum clearance ≥ footprint radius +
`cbf_margin` − resolution); a hold only with the box on the path.

**If it fails.** No contour: `obstacle_mode` is still `none`, or the global plan goes around the
box. The robot comes too close: the footprint in the costmap is too small, or the obstacle is seen
late (scan range, costmap update rate). Contour episodes ending in STOPPED: raise
`obstacle_max_contour_length` / `obstacle_max_contour_time`. CBF stops in doorways it could pass:
raise `cbf_gain` or lower `cbf_margin` ({doc}`../tuning`).

## Stage 8: speed limits and cancel

**Run.** A long straight goal, and speed limits while it runs:

```bash
ros2 run vectorfield_bringup navigate_waypoints.py --relative --poses "5,0,0" &
sleep 3; ros2 topic pub -t 3 /speed_limit nav2_msgs/msg/SpeedLimit "{percentage: false, speed_limit: 0.15}"
sleep 5; ros2 topic pub -t 3 /speed_limit nav2_msgs/msg/SpeedLimit "{percentage: true, speed_limit: 50.0}"
sleep 5; ros2 topic pub -t 3 /speed_limit nav2_msgs/msg/SpeedLimit "{percentage: false, speed_limit: 0.0}"
```

Then a goal cancelled half way (Ctrl-C in `navigate_waypoints.py`), and a follower run with
`--laps 0` stopped with Ctrl-C.

**Expect.** The `v` of the status text drops within about a second of each limit and returns
after the last; the robot keeps following the path at the lower speed. After a cancel it stops
and the status text shows `STOPPED`.

**Measure.** The speed-limit table of the report (largest commanded speed per period, 1 s after
the change; pass `--max-linear-vel` for the percentage limit), the stop latency of the cancelled
episodes.

**Accept.** Excess over every limit ≤ 0.01 m/s on `cmd_vel_nav` and `cmd_vel`; stop latency
≤ 1.0 s; after the limit is lifted the speed returns to `desired_speed`.

**If it fails.** No reaction to the limit: the topic name or namespace (the controller server and
the follower subscribe `speed_limit` relative to their namespace). Slow stop: the smoother's
`max_decel`, the collision monitor's timeout.

## Stage 9: localisation jumps

AMCL corrects `map` → `odom` in steps. Small steps are normal; large ones move the plan under the
robot. The controller must absorb them without oscillating.

**Run.** Watch the report's localisation section during every stage. To provoke a jump, during a
straight goal publish an initial pose 0.2 to 0.3 m to the side of the true pose (AMCL resets to
it, then converges back: two jumps):

```bash
ros2 run vectorfield_bringup navigate_waypoints.py --relative --poses "5,0,0" &
sleep 5; ros2 topic pub -t 2 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: X, y: Y}, orientation: {z: QZ, w: QW}}}}"
```

with X, Y the current map position (from `tf2_echo`) shifted by 0.25 m to the left of the robot,
and QZ, QW its orientation. In the kinematic simulator the service
`/vectorfield_sim/step_map_to_odom` does the same without AMCL.

**Expect.** \|D\| jumps by about the shift, the window and the control point move with the plan,
and the robot converges back without overshoot; Nav2 replans from the new pose, which shortens
the recovery (in the kinematic simulator a 0.25 m jump gives a \|D\| peak of 0.22 m, back below
0.05 m after 0.3 to 1.0 s). A large yaw jump can cause one ROTATE_TO_FIELD. The report's max \|D\|
check leaves out the recovery after each jump; the jump rows show it.

**Measure.** Per jump during a goal: the shift and yaw, the \|D\| peak in the next 3 s, the time
until \|D\| is back below 0.05 m, whether ROTATE_TO_FIELD was entered; FollowPath aborts.

**Accept.** \|D\| back below 0.05 m within about 2 m of travel (`shift` / `desired_speed` plus
the convergence distance of {ref}`sec-gains`); no NoValidControl, no abort, at most one
ROTATE_TO_FIELD per jump.

**If it fails.** Overshoot after the jump: lower `convergence_gain`. Frequent large corrections in
normal driving: AMCL's `update_min_d`, `update_min_a` and the laser model, or the odometry quality,
not the controller. `the robot footprint is in collision` after a jump: the jump put the
footprint into an obstacle of the costmap; the localisation is wrong.

## What to send back

For each session, pack the bags and the reports:

```bash
for bag in field_test_*/; do ros2 run vectorfield_bringup field_test_report.py "$bag"; done
tar czf field_tests_$(date +%Y%m%d).tgz field_test_*
```

and send, with the archive:

1. **The bags** (each directory has `metadata.yaml`, the data, `field_test.yaml` with the command
   line and the topics, and the `params_*.yaml` dumps of the controller server, the follower, the
   smoother, the collision monitor, the local costmap and the BT navigator).
2. **The reports** (`<bag>_report/`: `report.md`, `metrics.json`, `episode_NN.png`).
3. **The robot**: drive type, mass, wheel base or track width, wheel radius, footprint, sensors,
   localisation (AMCL, SLAM, motion capture), maximum speeds; the URDF if possible.
4. **The configuration**: the Nav2 parameter file as used, the `vf-tune` output, the
   `controller_server` and `curve_follower` log lines from start-up, the distro and the package
   versions (`ros2 pkg xml vectorfield_nav2_controller | grep version`).
5. **Notes per stage**: the floor surface, what you saw, anything the robot did that the report
   does not show, and a phone video of stages 3, 5 and 7 if you can.

A minute of recording is a few MB without `--markers`. If a bag is too large, the report and
`metrics.json` are still useful on their own; the identification output of stage 6 is a few
lines to paste.
