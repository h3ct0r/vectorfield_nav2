# Demo scenarios

Each scenario below is one launch of `vectorfield_bringup` with a goal to send, the overlay it
uses, what to look for in RViz and what to change next. Start with {doc}`getting_started` if
the workspace is not built yet. In every scenario:

- the robot starts at (0, 0) facing +x;
- `use_rviz:=false` runs it without RViz, and the goals can be sent from a terminal once
  `ros2 lifecycle get /bt_navigator` prints `active [3]` (an earlier goal is rejected);
- `scenarios:=` takes extra overlays, comma separated: a name in `config/scenario/` or a YAML
  path, merged after the motion-model overlay (`config/controller/<motion_model>.yaml`).

The `vectorfield_bringup` README lists every launch argument; {doc}`../visualization` explains
the markers.

## Differential base in open space

**Goal.** See the phases of a differential-drive robot on a plain Nav2 plan.

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py
```

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: -3.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

**Overlay.** `config/controller/differential.yaml` replaces `FollowPath` and raises the velocity
smoother's limits to cover the controller's:

```{literalinclude} ../../vectorfield_bringup/config/controller/differential.yaml
:language: yaml
:start-at: "controller_server:"
:end-before: "# kinematic_sim_demo"
```

**In RViz.** The goal is behind the robot: it turns in place (ROTATE_TO_FIELD, amber) until the
slate field arrow is within 45° of its heading, follows the plan (FOLLOW, green), brakes inside
the orange zone (FINAL_APPROACH, cyan) and stops on the goal. A goal with another heading ends
with ROTATE_TO_HEADING (blue).

**Tweak.** Try `desired_speed` and `convergence_gain` at run time (see
[run-time tuning](#run-time-tuning)), or `control_point_offset` to see the corner cutting change.

## Corridor and doorway

**Goal.** Pass a 0.8 m door in a 2 m corridor without the obstacle contour engaging: the door is
free, so the path is not blocked.

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py map:=corridor_door
```

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 8.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

**Overlay.** The differential overlay above; `corridor_door` is a 2 m × 12 m corridor along +x,
from x = −1 m to 11 m, with the door at x = 5 m.

**In RViz.** The status stays FOLLOW through the door, with no `CONTOUR` flag: the contour gate
opens only when the path ahead is blocked, and the door is narrower than twice the contour
distance but free. The green predicted path passes the door without a collision mark.

**Tweak.** `ros2 param set /controller_server FollowPath.obstacle_activation always` reproduces
the upstream rule (contour whenever an obstacle is near), and the contour then engages at the
door frame, a symptom listed in the {doc}`../tuning`. Set it back to `path_blocked`.

## Sharp corner

**Goal.** Take a 90° corner in a 1.2 m wide L-shaped corridor without touching the walls.

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py map:=corridor_corner
```

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 6.0, y: 6.0}, orientation: {z: 0.7071, w: 0.7071}}}}"
```

**Overlay.** The differential overlay. The corridor runs along +x from (0, 0) to the corner at
(6, 0), then along +y to (6, 7).

**In RViz.** The robot follows the plan to the corner; when the field at the control point turns
more than 90° away from the heading, it turns in place once (ROTATE_TO_FIELD) and continues
along +y. The control point (0.2 m ahead) cuts the corner by about its offset.

**Tweak.** A smaller `control_point_offset` follows the corner more tightly but turns more
abruptly; `tracking_law: heading_tracking` evaluates the field at `base_link` instead
(`tracking_law` can be changed at run time). {doc}`../tuning` lists the trade-offs.

## Unmapped obstacles and contouring

**Goal.** Drive around an obstacle that is not in the map, on the side where it is free, and
rejoin the path.

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py scenarios:=demo_obstacles
```

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

**Overlay.** The differential overlay sets `obstacle_mode: contour`. `demo_obstacles` adds
circles and boxes that only the simulated laser sees, among them a 0.4 m box 2 m ahead, and turns
the global costmap's obstacle layer off so that the plan runs through the box:

```{literalinclude} ../../vectorfield_bringup/config/scenario/demo_obstacles.yaml
:language: yaml
:start-at: "global_costmap:"
```

**In RViz.** The local costmap shows the box; the predicted path turns red at it, the contour
gate opens and the status reads `CONTOUR +1` or `CONTOUR -1` (the side). The robot passes the box
at about the contour distance λ (0.47 m from the costmap cells for the TB3 footprint), rejoins
the plan and stops on the goal. The contour markers show the blocking cells, the λ,
$D_{\mathrm{in}}$ and $D'_{\mathrm{in}}$ rings and the watchdog progress
({ref}`viz-contour`).

**Add an obstacle at run time.** The obstacle lists are parameters of the simulator. Each box is
`x, y, size_x, size_y, yaw` in the map frame; setting the list replaces it, so repeat the boxes
to keep. This keeps the two boxes of the overlay and adds a third at (3, −1.5):

```bash
ros2 param set /vectorfield_sim box_obstacles "[2.0, 0.0, 0.4, 0.4, 0.0, -2.0, 2.0, 1.5, 0.2, 0.785, 3.0, -1.5, 0.5, 0.5, 0.0]"
```

`circle_obstacles` takes `x, y, radius` triples the same way.

**Tweak.** With `obstacle_mode: none` the robot stops short of the box instead (status
`BLOCKED`), the controller raises `NoValidControl`, and Nav2's behaviour tree replans or
recovers:

```bash
ros2 param set /controller_server FollowPath.obstacle_mode none
```

Send the robot back to (0, 0) through the box to see it. Because this overlay keeps the box out of
the global costmap, every new plan runs through it again and the goal ends ABORTED after the
recoveries; with the stock global obstacle layer, the new plan goes around the box.

{doc}`../obstacle_avoidance` explains the gate, the side choice, the distances and the
watchdog.

## Omnidirectional base

**Goal.** A holonomic base that tracks the field directly, sideways included.

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py motion_model:=omnidirectional
```

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 2.0}, orientation: {w: 1.0}}}}"
```

**Overlay.** `config/controller/omnidirectional.yaml`. Besides the motion model, it lowers
Nav2's `min_y_velocity_threshold` (the stock 0.5 m/s hides every slower lateral speed) and opens
the smoother's y axis:

```{literalinclude} ../../vectorfield_bringup/config/controller/omnidirectional.yaml
:language: yaml
:start-at: "controller_server:"
:end-before: "# kinematic_sim_demo"
```

**In RViz.** There is no ROTATE_TO_FIELD: the base moves along the field from the start while
it turns to face the path tangent (`omni_heading_policy: path_tangent`). It ends with
ROTATE_TO_HEADING to the goal yaw.

**Tweak.** `ros2 param set /controller_server FollowPath.omni_heading_policy fixed` keeps the
heading while the base slides along the path; `field` faces the field.

## Skid-steer base

**Goal.** A skid-steer base whose wheels slip, as the simulator models it.

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py motion_model:=skid_steer
```

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: -3.0, y: 1.0}, orientation: {w: 1.0}}}}"
```

**Overlay.** `config/controller/skid_steer.yaml`: the differential settings plus
`min_in_place_angular_vel: 0.25`, a floor for in-place rotations that overcomes the static
friction of skidding wheels. The overlay leaves the slip compensation off
(`skid_yaw_rate_efficiency: 1.0`, `skid_icr_offset: 0.0`). The simulator adds its slip model
(`slip_eta_psi` 0.75, `slip_x_icr` 0.05), and this demo sets the controller's compensation to
those same values ({ref}`skid-slip`).

**In RViz.** The same phases as the differential base; the rotations in place never creep below
0.25 rad/s.

**Tweak.** Real skid-steer bases usually need 0.25 to 0.3 rad/s for `min_in_place_angular_vel`;
it must not exceed `rotate_to_heading_angular_vel`. On a real base, identify the two slip values
before setting them in your copy of the overlay ({ref}`skid-slip`); the demo's values are the
simulator's.

## Ackermann (car-like) base

**Goal.** A car that cannot turn in place: drivable plans, a position-only goal check and a
behaviour tree without Spin.

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py motion_model:=ackermann
```

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.5}, orientation: {w: 1.0}}}}"
```

**Overlay.** `config/controller/ackermann.yaml` sets the turning radius (0.5 m) for the
controller, the planner and the simulator, switches the planner to Smac Hybrid-A* with
forward-only `DUBIN` motions, uses `PositionGoalChecker`, runs NavigateToPose with
`navigate_w_replanning_time.xml`, and enables contouring with a 4 m × 4 m local costmap:

```{literalinclude} ../../vectorfield_bringup/config/controller/ackermann.yaml
:language: yaml
:start-at: "controller_server:"
:end-before: "# Only the motion-model overlays"
```

**In RViz.** The plan is made of arcs no tighter than the 0.5 m turning radius. The car follows
it at a positive speed and stops on the goal without turning to the goal yaw; the status text
shows `CURV-SAT` whenever the curvature limit binds. A goal behind the car is reached through a
loop, never a turn in place.

**Contour an unmapped obstacle.** With `demo_obstacles` the 0.4 m box 2 m ahead is on the plan:

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py motion_model:=ackermann \
  scenarios:=demo_obstacles
```

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

The contour rings are wider than for the differential robot: λ = 0.77 m, and the turn-in band
reaches $D'_{\mathrm{in}}$ = 1.47 m, one turning radius further out. The gate opens about 0.3 m after
the start, the car turns at the 0.5 m radius (`CURV-SAT`) until it runs at λ around the box,
and it rejoins the plan behind it. Move the box closer with
`ros2 param set /vectorfield_sim box_obstacles "[0.9, 0.0, 0.4, 0.4, 0.0]"` and send the goal
again: the car starts inside the band, turns at once and passes the box's corner at about 0.35 m,
as close as the turning radius allows ({ref}`ackermann-contour`).

**Tweak.** With `obstacle_mode: none`, or where the car cannot drive around the obstacle (a gap
narrower than about λ + the half-width, a dead end), the obstacle stops the car (`BLOCKED`), the
controller raises `NoValidControl` and Nav2 replans. Do not use NavigateThroughPoses or the stock
recovery trees with a car: they contain Spin. {ref}`ackermann-caveats` lists the rest.

## Reversing

**Goal.** Plans that back up: a goal behind the robot, a three-point turn in a corridor. Smac
Hybrid-A* searches Reeds–Shepp motions, and the controller follows the plan segment by segment
with a stop at every cusp ({ref}`reversing`). For a differential, skid-steer or Ackermann base:

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py motion_model:=ackermann \
  map:=corridor_door scenarios:=reversing initial_pose_x:=3.5
```

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

**Overlay.** `config/scenario/reversing.yaml` turns on `allow_reversing` and switches the planner
to Smac Hybrid-A* with `REEDS_SHEPP` motions; reversing costs twice as much as driving forward
(`reverse_penalty`):

```{literalinclude} ../../vectorfield_bringup/config/scenario/reversing.yaml
:language: yaml
:start-at: "controller_server:"
```

**In RViz.** The goal 1.5 m behind the car is reached by backing up in a straight line, since a
turn in the corridor costs more: the status text shows `REV`, and the car never turns around.
When a plan has a cusp, the `goal` marker sits on the cusp with its two rings (the stop's XY
tolerance and the `cusp_tolerance` settle radius); the robot stops inside the inner ring, then
the marker moves to the next cusp or the goal. `vectorfield_system_tests` runs this scenario for
the differential and the Ackermann base.

**Tweak.** `cusp_tolerance` (0.05 m) sets how close to a cusp the robot stops, and
`max_reverse_vel` (0.2 m/s in the overlay; remove it to back up at `desired_speed`) how fast it
backs up. An obstacle on a reverse leg is contoured as a forward one; a dead end stops the robot
and Nav2 replans. A goal checker succeeds as soon as the robot is within its tolerance of the plan's last
pose, even before the plan's end: avoid goals that the plan passes earlier. On Lyrical the path
handler's `enforce_path_inversion: true` with `SimpleGoalChecker`'s `path_length_tolerance`
prevents it; the controller builds its own window either way.

## Rotation shim

**Goal.** Run the controller as the primary controller of Nav2's `RotationShimController`, which
turns the robot toward the path before handing over.

The overlay depends on the distro's key layout. Humble and Jazzy:

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py scenarios:=$(ros2 pkg prefix vectorfield_bringup)/share/vectorfield_bringup/config/examples/rotation_shim_humble_jazzy.yaml
```

Lyrical:

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py scenarios:=$(ros2 pkg prefix vectorfield_bringup)/share/vectorfield_bringup/config/examples/rotation_shim_lyrical.yaml
```

Then send a goal behind the robot:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: -3.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

**Overlay.** The example replaces the motion-model overlay's `FollowPath` (its plugin changes),
turns the controller's own ROTATE_TO_FIELD off and keeps `rotate_to_heading_angular_vel` within
`max_angular_vel`. The Humble and Jazzy layout:

```{literalinclude} ../../vectorfield_bringup/config/examples/rotation_shim_humble_jazzy.yaml
:language: yaml
:start-at: "controller_server:"
```

The Lyrical layout moves the vector-field keys under `FollowPath.primary_controller`. The
Lyrical shim also re-checks the heading on every control cycle; near the goal it would turn the
robot back toward the last path pose while the controller turns it to the goal yaw, and the goal
would never succeed. The example therefore sets the shim's `rotate_to_goal_heading: true`: the
shim makes the final turn itself as soon as the robot is inside the XY tolerance, so the robot
stops at the edge of the tolerance instead of on the goal.

```{literalinclude} ../../vectorfield_bringup/config/examples/rotation_shim_lyrical.yaml
:language: yaml
:start-at: "controller_server:"
```

**In RViz.** The shim turns the robot in place toward the path, then hands over to the
vector-field controller, which follows the plan as in the first scenario. On Lyrical the markers
are published on `FollowPath/primary_controller/markers`, so point the RViz marker displays
there.

**Tweak.** The shim re-aims the robot at the path whenever it re-checks the heading, which undoes
an obstacle-contour episode. Contour behind the shim is supported only on Jazzy with
`FollowPath.rotate_to_heading_once: true` ({doc}`../motion_models`).

## TurtleBot 3 and 4 in Gazebo (Jazzy and later)

**Goal.** The controller on a physics simulation with AMCL, in Nav2's own TurtleBot worlds.

```bash
ros2 launch vectorfield_bringup tb3_gazebo_demo.launch.py
```

```bash
ros2 launch vectorfield_bringup tb4_gazebo_demo.launch.py
```

**Overlay.** The differential overlay (or `motion_model:=skid_steer`), merged onto the stock
parameters as in the kinematic demo. The launch files include nav2_bringup's
`tb3_simulation_launch.py` and `tb4_simulation_launch.py` with their maps (`tb3_sandbox`,
`depot`), and publish the initial pose at the spawn pose: (−2, −0.5) for the TB3; the TB4 spawns
at (−8, 0) in its world, which is the origin of the depot map, so its initial pose is (0, 0).
Both have been run headless on Jazzy: a goal 1 m (TB3) and 2 m (TB4) ahead of the spawn pose
succeeded.

**In RViz.** The particle cloud of AMCL around the robot, then the same markers as in the
kinematic demo. Send goals with **2D Goal Pose** in free space of the map. Gazebo runs without
its GUI; `headless:=False` shows it.

**Tweak.** `x_pose`, `y_pose` and `yaw` move the spawn pose in the world, and the initial pose
with it; `map:=` takes a map YAML that matches the world.

## Nav2 loopback simulator (Jazzy and later)

**Goal.** The same controller configuration on Nav2's own loopback simulator, which integrates
the command kinematically.

```bash
ros2 launch vectorfield_bringup loopback_demo.launch.py
```

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"
```

**Overlay.** The motion-model overlay and `config/scenario/loopback.yaml`
(`scale_velocities: true`). The demo publishes the initial pose itself, because the loopback
simulator publishes no TF before it receives one.

**In RViz.** The same phases as the kinematic demo. `map:=corridor_door` or
`map:=corridor_corner` load the other maps. The unmapped obstacles of `demo_obstacles` exist
only in `vectorfield_sim`, so they do not appear here.

## Speed limits

**Goal.** Slow the robot down from outside, as Nav2's speed filter does in a speed zone.

Start the kinematic demo and send a goal several metres away:

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py
```

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 3.0}, orientation: {w: 1.0}}}}"
```

While the robot drives, publish a limit of 50 % (the speed drops from 0.4 to 0.2 m/s):

```bash
ros2 topic pub --times 3 /speed_limit nav2_msgs/msg/SpeedLimit "{percentage: true, speed_limit: 50.0}"
```

An absolute limit in m/s:

```bash
ros2 topic pub --times 3 /speed_limit nav2_msgs/msg/SpeedLimit "{percentage: false, speed_limit: 0.15}"
```

`speed_limit: 0.0` removes the limit:

```bash
ros2 topic pub --times 3 /speed_limit nav2_msgs/msg/SpeedLimit "{percentage: true, speed_limit: 0.0}"
```

**What happens.** controller_server passes the limit to the controller (`setSpeedLimit`), which
scales the whole field, so the robot slows down without leaving the path. The angular limit
scales with it. To see the commanded speed:

```bash
ros2 topic echo /FollowPath/field_state --field cmd.linear.x
```

**Nav2's speed filter.** The speed filter (a costmap filter with a speed mask) publishes the same
`nav2_msgs/SpeedLimit` messages on `speed_limit` while the robot is in a zone of the mask. The
controller needs no configuration for it: set up the filter as in Nav2's
[speed filter tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_speed_filter.html).
The curve follower subscribes to `speed_limit` too.

(run-time-tuning)=

## Run-time tuning

**Goal.** Change parameters while the robot drives, and see the validator refuse a configuration
that would be invalid.

Every controller parameter except `motion_model` can be set at run time. A change is validated
together with the rest of the configuration before it is applied. Start any demo, then:

```bash
ros2 param set /controller_server FollowPath.desired_speed 0.3
```

The command prints `Set parameter successful`, and the next goals run at 0.3 m/s. A value that
would make the configuration invalid is refused, with the reason, and the previous configuration
stays in force:

```bash
ros2 param set /controller_server FollowPath.desired_speed 0.7
```

```text
Setting parameter failed: rejected FollowPath.desired_speed: [desired_speed_above_max_linear_vel] desired_speed 0.7 > max_linear_vel 0.5: the field speed must be commandable
```

controller_server logs the same reason after `Parameter change rejected:`. Raise
`max_linear_vel` first (and the velocity smoother's limits with it), then `desired_speed`.
`motion_model` is read-only:

```bash
ros2 param set /controller_server FollowPath.motion_model omnidirectional
```

```text
Setting parameter failed: parameter 'FollowPath.motion_model' cannot be set because it is read-only
```

Changes that pass with a warning (for example `convergence_gain` 5, the `fl_gain_offset`
warning) are applied and logged with their code. Read a value back with:

```bash
ros2 param get /controller_server FollowPath.desired_speed
```

{doc}`../tuning` explains each validator finding and the order in which to tune.
