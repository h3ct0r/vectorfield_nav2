# vectorfield_bringup

Launch files, Nav2 parameter overlays, maps and viewer configurations for running the
vectorfield_nav2 controller and curve follower in simulation, and for seeing in RViz what the
robot is doing.

**Status:** the controller demos, overlays, maps and viewer configurations (M3) and the curve
follower demo with its scripts (M5) are implemented.

## Demos

| Launch file | Distros | Simulation |
|---|---|---|
| `kinematic_sim_demo.launch.py` | all | `vectorfield_sim`, with map_server and Nav2's `navigation_launch.py` |
| `curve_follower_demo.launch.py` | all | `kinematic_sim_demo` with the `curve_follower` scenario, plus the curve follower and `lifecycle_manager_curve_follower` |
| `loopback_demo.launch.py` | Jazzy and later | nav2_bringup's TB3 loopback wrapper |
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
| `initial_pose_x`, `initial_pose_y`, `initial_pose_yaw` | 0 | start pose in the map (Gazebo: `x_pose`, `y_pose`, `yaw`, the spawn pose) |
| `use_composition` | `true` | kinematic and curve follower demos: run map_server, the Nav2 servers and their lifecycle managers in one component container |
| `headless` | `True` | Gazebo demos: run Gazebo without its GUI |

Each demo launch file exits with an error that names the supported distros when it cannot run on
the current one.

## Parameters

The demos never ship a full Nav2 parameter file. `vectorfield_bringup.params.merge()` deep-merges
the overlays onto the installed `nav2_bringup/params/nav2_params.yaml` of the running distro:
dictionaries merge key by key, other values (lists included) replace, `FollowPath` and any block
whose `plugin` changes are replaced wholesale, a bare behaviour-tree name is resolved in
`nav2_bt_navigator/behavior_trees`, and on Humble `nav2_smac_planner::X`, `nav2_navfn_planner::X`
and `nav2_behaviors::X` are renamed to the `/` form.

| Path | Contents |
|---|---|
| `config/controller/` | Motion-model overlays: `differential`, `skid_steer`, `omnidirectional`, `ackermann` |
| `config/scenario/` | Scenario overlays: `kinematic_sim`, `loopback`, `demo_obstacles` (unmapped obstacles for `vectorfield_sim`, global obstacle layer off), `curve_follower` (local costmap for the M5 follower) |
| `config/humble/` | `collision_monitor.yaml`, the only distro-specific file |
| `maps/` | `open_10x10`, `corridor_door` and `corridor_corner` |
| `rviz/`, `foxglove/`, `plotjuggler/` | `vectorfield.rviz`, `vectorfield.json`, `vectorfield.xml` |
| `scripts/` | `initial_pose_publisher`: publishes `initialpose` at 1 Hz until map → base_footprint exists; `send_curve.py` and `moving_frame_publisher.py` (see below) |

Overlay rules:

- Only the motion-model overlays set the velocity smoother's `max_velocity`, `min_velocity`,
  `max_accel` and `max_decel`, as full `[x, y, θ]` lists at least the controller limits; y is 0
  except for the omnidirectional overlay, which also sets `min_y_velocity_threshold: 0.001`.
  Scenario overlays set only `scale_velocities: true`.
- The demo overlays use `obstacle_mode: contour`, except Ackermann (`none`; contouring is not
  supported for car-like bases in 0.x).
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

## What you will see in RViz

`rviz/vectorfield.rviz` shows the map, both costmaps, the global plan, the robot model and TF,
and the controller's markers, pre-configured with the shared colour legend:

- a status text above the robot whose colour follows the phase (FOLLOW green, ROTATE_TO_FIELD
  amber, FINAL_APPROACH cyan, ROTATE_TO_HEADING blue, STOPPED red) and lists active flags such
  as `BLOCKED` or `CONTOUR ±1`;
- the field at the control point split into convergence (red), circulation (green), feedforward
  (blue) and the resulting field (white);
- the commanded velocity (cyan) and the predicted path, green up to the first collision;
- the goal with its tolerance and settle rings and the orange braking zone;
- during obstacle contouring, the blocking obstacle cells, the distance rings and the chosen
  side;
- optionally, the `field_grid` arrows that show the whole field around the robot (the
  "Field grid" display, off by default because the grid costs CPU while subscribed).

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
