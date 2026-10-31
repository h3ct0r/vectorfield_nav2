# vectorfield_bringup

Launch files, Nav2 parameter overlays, maps and viewer configurations for running the
vectorfield_nav2 controller and curve follower in simulation, and for seeing in RViz what the
robot is doing.

**Status:** the controller demos, overlays, maps and viewer configurations and the curve
follower demo with its scripts are implemented.

## Demos

| Launch file | Distros | Simulation |
|---|---|---|
| `kinematic_sim_demo.launch.py` | all | `vectorfield_sim`, with map_server and Nav2's `navigation_launch.py` |
| `curve_follower_demo.launch.py` | all | `kinematic_sim_demo` with the `curve_follower` scenario, plus the curve follower and `lifecycle_manager_curve_follower` |
| `follower3d_demo.launch.py` | all | [experimental] `point_sim_3d` (a kinematic point robot in 3D), the 3D follower and its lifecycle manager; no Nav2 |
| `loopback_demo.launch.py` | Jazzy and later | Nav2's loopback simulator with the TB3 model, Nav2 as in `kinematic_sim_demo` |
| `tb3_gazebo_demo.launch.py`, `tb4_gazebo_demo.launch.py` | Jazzy and later | nav2_bringup's Gazebo TurtleBot simulations, with AMCL |

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py motion_model:=omnidirectional
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py scenarios:=demo_obstacles
ros2 launch vectorfield_bringup loopback_demo.launch.py map:=corridor_door
```

Arguments:

| Argument | Default | Meaning |
|---|---|---|
| `motion_model` | `differential` | `config/controller/<motion_model>.yaml`: `differential`, `skid_steer`, `omnidirectional` or `ackermann` (Gazebo: the first two) |
| `map` | `open_10x10` | a name in `maps/` (`open_10x10`, `corridor_door`, `corridor_corner`) or a map YAML path; Gazebo: empty keeps the world's map |
| `scenarios` | empty | extra overlays, comma separated: names in `config/scenario` (for example `demo_obstacles`) or YAML paths |
| `use_rviz` | `true` | start RViz with `rviz/vectorfield.rviz` (the wrappers' own RViz is always disabled) |
| `initial_pose_x`, `initial_pose_y`, `initial_pose_yaw` | 0 | start pose in the map (Gazebo: `x_pose`, `y_pose`, `yaw`, the spawn pose in the world; the initial pose follows it) |
| `use_composition` | `true` | kinematic, loopback and curve follower demos: run map_server, the Nav2 servers and their lifecycle managers in one component container |
| `headless` | `True` | Gazebo demos: run Gazebo without its GUI |

Each demo launch file exits with an error that names the supported distros when it cannot run on
the current one.

On Ctrl+C the demos shut down within a few seconds. The simulators stop after Nav2's nodes,
which wait on simulated time while they shut down: ours keep running while `/clock` has readers,
and Nav2's loopback simulator and Gazebo run under `vectorfield_sim`'s `shutdown_last`, which
holds the signal back from them. The demos' containers are this package's `isolated_container`:
at the signal it has Nav2's lifecycle managers shut their nodes down before the context shuts
down, so each node cleans up on its own executor (Nav2's own shutdown path cleans a node up
while its callbacks run, a crash at exit now and then). They also preload the lifecycle
manager's library, which a container would otherwise unload under the manager's own shutdown
callback (a crash at every exit); in the Gazebo demos, whose container nav2_bringup starts,
`LD_PRELOAD` is set for the included launch file. That container keeps Nav2's own shutdown
races: in Lyrical's Gazebo demos a server's cleanup can crash now and then, and a recovery
behaviour (Wait, Spin, BackUp) under way at Ctrl+C runs to its end, past the 4 s the simulator
waits, so launch kills the container.

Both Gazebo demos have been run headless on Jazzy (Gazebo Harmonic, software rendering) with the
differential overlay: one NavigateToPose goal 1 m (TB3) and 2 m (TB4) ahead of the spawn pose
SUCCEEDED. The TB4's depot map has its origin at the spawn point (−8, 0) of the world, so the
initial pose is (0, 0) in the map. Neither demo runs in CI.

## Parameters

The demos never ship a full Nav2 parameter file. `vectorfield_bringup.params.merge()` deep-merges
the overlays onto the installed `nav2_bringup/params/nav2_params.yaml` of the running distro:
dictionaries merge key by key, other values (lists included) replace, except that
`bt_navigator.plugin_lib_names` entries are appended to the stock list, `FollowPath` and any
block whose `plugin` changes are replaced wholesale, a bare behaviour-tree name is resolved in
`nav2_bt_navigator/behavior_trees`, then in this package's `behavior_trees`, and on Humble
`nav2_smac_planner::X`, `nav2_navfn_planner::X` and `nav2_behaviors::X` are renamed to the `/`
form.

| Path | Contents |
|---|---|
| `config/controller/` | Motion-model overlays: `differential`, `skid_steer`, `omnidirectional`, `ackermann` |
| `config/scenario/` | Scenario overlays: `kinematic_sim`, `loopback`, `demo_obstacles` (unmapped obstacles for `vectorfield_sim`, global obstacle layer off), `curve_follower` (local costmap for the curve follower), `follow_curve_bt` (bt_navigator runs `behavior_trees/follow_curve.xml`; with `curve_follower_demo`) |
| `config/humble/` | `collision_monitor.yaml`, the only distro-specific file |
| `config/examples/` | Templates to copy into your own `nav2_params.yaml` (`nav2_params_vectorfield_differential`, `_omnidirectional`, `_ackermann`), and the rotation-shim overlays `rotation_shim_humble_jazzy` and `rotation_shim_lyrical`; see the "Your robot" tutorial |
| `behavior_trees/` | Trees with the `FollowCurve` node of `vectorfield_behavior_tree`: `follow_curve.xml` (approach with Nav2, then one lap of an ellipse) and `follow_curve_patrol.xml` (a patrol with a recovery); docs/behavior_tree.md |
| `maps/` | `open_10x10`, `corridor_door` and `corridor_corner` |
| `rviz/`, `foxglove/`, `plotjuggler/` | `vectorfield.rviz`, `vectorfield.json`, `vectorfield.xml` |
| `scripts/` | `initial_pose_publisher`: publishes `initialpose` at 1 Hz until map → base_footprint exists; `send_curve.py`, `send_curve3d.py` and `moving_frame_publisher.py` (see below); the field-test kit (below) |
| `examples/` | The tutorials' minimal clients, installed like the scripts: `follow_curve_client.py` (a `follow_curve` action client) and `navigate_waypoints.py` (a route of NavigateToPose goals with Nav2's simple commander; `--relative` poses in the robot's start frame, `--through` one NavigateThroughPoses goal) |

Overlay rules:

- Only the motion-model overlays set the velocity smoother's `max_velocity`, `min_velocity`,
  `max_accel` and `max_decel`, as full `[x, y, θ]` lists at least the controller limits; y is 0
  except for the omnidirectional overlay, which also sets `min_y_velocity_threshold: 0.001`.
  Scenario overlays set only `scale_velocities: true`.
- The demo overlays use `obstacle_mode: contour`. The Ackermann overlay also enlarges the local
  costmap to 4 m × 4 m, because the car's contour distances grow with its turning radius.
- The Ackermann overlay switches the planner to Smac Hybrid-A* (`DUBIN`, turning radius 0.5 m,
  as the controller and the simulator), uses `PositionGoalChecker` and runs NavigateToPose with
  `navigate_w_replanning_time.xml`, a stock tree without Spin. The stock recovery trees, and every
  stock NavigateThroughPoses tree, use Spin, which a car cannot do.
- No overlay sets `enable_stamped_cmd_vel`.

Distro notes:

- The loopback and Gazebo demos need Jazzy or later. `kinematic_sim_demo` uses `vectorfield_sim`
  and runs everywhere. On Lyrical, whose `navigation_launch.py` has no lifecycle manager, it
  starts `lifecycle_manager_navigation` itself.
- On Jazzy and later the command chain is Nav2's velocity smoother, then collision_monitor, then
  `cmd_vel`. On Humble the demos add collision_monitor with `config/humble/collision_monitor.yaml`
  (`cmd_vel` to `cmd_vel_safe`), and the simulator listens on `cmd_vel_safe`.

## Curve follower

`curve_follower_demo.launch.py` takes the kinematic demo's arguments. It keeps Nav2 running: the
local costmap is the follower's obstacle input (the `curve_follower` overlay adds the static
layer), Nav2's action statuses drive the follower's interlock, and the follower's `cmd_vel_nav`
goes through the velocity smoother and collision_monitor like the controller's. The follower
gets the merged `FollowPath` block (motion model, speeds, limits, obstacle mode) and then the
overlays' `curve_follower` section, which sets `moving_frame_reference: odom`. To manage the
follower by hand instead of through `lifecycle_manager_curve_follower`, run it alone and use
`ros2 lifecycle set /curve_follower configure`, then `activate`.

```bash
ros2 launch vectorfield_bringup curve_follower_demo.launch.py
ros2 run vectorfield_bringup send_curve.py --shape lemniscate_bernoulli --laps 3
ros2 run vectorfield_bringup send_curve.py --shape sine --speed 0.2 --reverse
```

`send_curve.py` sends one of the example curves of `vectorfield_bringup.curves`, written from
their math: `ellipse`, `lemniscate_bernoulli`, `lemniscate_gerono`, `superellipse`, the open
`sine`, a time-varying `expression` (`--x`, `--y`, `--s-min`, `--s-max` and `--open` replace it)
and `moving`. It approaches the curve first, client-side: `vectorfield.make_curve` and
`closest_point` give the joining point (an open curve: its start), and `BasicNavigator.goToPose`
drives there facing along the tangent. It then sends the goal, prints the feedback and exits with
0 on success. Options: `--laps` (0: until Ctrl-C, which cancels), `--speed`, `--reverse`,
`--frame`, `--no-approach`, `--max-duration`.

`moving` is a 1.5 m circle in the frame `moving_frame`, sent without an approach.
`moving_frame_publisher.py` publishes such a frame: TF `parent_frame` → `child_frame`
(`odom` → `moving_frame`) and `<child_frame>/odom`, for the constant twist `vx`, `vy`, `wz` in the
child frame from the pose `x`, `y`, `yaw`. The follower reads the frame's twist from that
Odometry, or with `--tf` by TF differencing against `odom`:

```bash
ros2 run vectorfield_bringup moving_frame_publisher.py --ros-args -p use_sim_time:=true \
  -p vx:=0.1 -p y:=1.5
ros2 run vectorfield_bringup send_curve.py --shape moving --laps 0
ros2 run vectorfield_bringup send_curve.py --shape moving --tf --laps 0
```

Start the frame so that the circle passes near the robot (here, the demo's start pose). A NavigateToPose sent while the
follower runs (RViz's 2D Goal Pose) stops it with `NAV2_ACTIVE`.

## 3D follower (experimental)

`follower3d_demo.launch.py` runs `vectorfield_follower_3d` on `point_sim_3d` and opens RViz with
`rviz/follower_3d.rviz` (an orbit view). The follower runs the CBF filter against the
simulator's point cloud of `obstacle_spheres`, voxelised at 0.1 m. `send_curve3d.py` sends the
examples of `vectorfield_bringup.curves3d`: `helix` (open), `viviani`, `torus_knot`,
`lemniscate3d`, a time-varying `expression` and `moving`:

```bash
ros2 launch vectorfield_bringup follower3d_demo.launch.py
ros2 run vectorfield_bringup send_curve3d.py --shape helix
ros2 run vectorfield_bringup send_curve3d.py --shape viviani --laps 2
ros2 param set /point_sim_3d obstacle_spheres "[0.6, 0.0, 1.3, 0.25]"   # a sphere to avoid
ros2 launch vectorfield_bringup follower3d_demo.launch.py moving_frame:=platform
ros2 run vectorfield_bringup send_curve3d.py --shape moving --laps 0    # Ctrl-C to stop
```

Its arguments: `use_rviz`, `cmd_frame` (`global` or `body`), `params_file` (an extra YAML for
`follower_3d`), `real_time_factor`, `moving_frame` (a frame moving at 0.1 m/s and 0.1 rad/s),
`velocity_time_constant` (a first-order velocity lag), and `initial_x`, `initial_y`,
`initial_z`, `initial_yaw`. See docs/follower_3d.md.

## Field tests

The field-test kit for a real robot (docs/tutorials/field_testing.md):

```bash
ros2 run vectorfield_bringup record_field_test.py --label stage2          # Ctrl-C to stop
ros2 run vectorfield_bringup field_test_report.py field_test_<date>_<time>_stage2
ros2 run vectorfield_bringup identify_skid_slip.py --v 0.2 --w 0.6 --pose-topic /odometry/filtered
```

`record_field_test.py` records the topics the metrics need (TF, odometry, `/plan`, both
`field_state` topics, the command chain, `speed_limit`, the local costmap, the scan and the
action statuses) into a timestamped rosbag2 bag and dumps the node parameters next to it.
`field_test_report.py` reads a bag (sqlite3 or mcap) and writes `report.md`, `metrics.json` and
plots per goal: cross-track error, time to goal, jerk, phases, status flags, clearance, speed
limits and localisation jumps, with pass/fail checks. `identify_skid_slip.py` drives a guarded
turn sequence (or reads a bag) and estimates `skid_yaw_rate_efficiency` and `skid_icr_offset`.
The metrics live in `vectorfield_bringup.field_metrics` (shared with
`vectorfield_system_tests`), `.slip`, `.bag` and `.field_report`.

## What you will see in RViz

`rviz/vectorfield.rviz` shows the map, both costmaps, the global plan, the robot model and TF,
and the controller's markers, pre-configured with the shared colour legend:

- a status text above the robot whose colour follows the phase (FOLLOW green, ROTATE_TO_FIELD
  amber, FINAL_APPROACH cyan, ROTATE_TO_HEADING blue, STOPPED red) and lists active flags such
  as `BLOCKED` or `CONTOUR ±1`;
- the field at the control point split into convergence (red), circulation (green), feedforward
  (blue) and the resulting field (slate);
- the commanded velocity (cyan) and the predicted path, green up to the first collision;
- the goal with its tolerance and settle rings and the orange braking zone;
- during obstacle contouring, the blocking obstacle cells, the distance rings and the chosen
  side;
- the `field_grid` arrows that show the whole field around the robot (the "Field grid"
  display, on by default; untick it to save the grid's CPU, which is spent only while
  subscribed).

The "Curve follower" group shows the same markers, window and predicted path for
`curve_follower`. The global plan is grey, the controller's window teal and the
`predicted_path` green. RViz's 2D Goal Pose tool sends `goal_pose` to bt_navigator.
`foxglove/vectorfield.json` (Foxglove, through `foxglove_bridge`) shows the same 3D scene plus
plots of D, σ, v, ω and the limiter scale and a phase/status timeline from
`FollowPath/field_state`; `plotjuggler/vectorfield.xml` loads the same plots in PlotJuggler.

The full legend is in `vectorfield_ros` and in the
[documentation](https://h3ct0r.github.io/vectorfield_nav2/).

## Packaging

`vectorfield_bringup` is not part of the `vectorfield_nav2` metapackage, so installing the
stack on a robot does not pull in the simulators. On Lyrical it has no installable RHEL 10 RPM,
because its Nav2 and Gazebo dependencies are not built for RHEL 10.
