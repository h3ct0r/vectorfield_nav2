# M7: the 3D follower node (experimental)

The 3D core ({doc}`m7_3d`, math_spec §14) gets a ROS 2 node: a FollowCurve-style action server
for drones and other robots that take a 3D velocity setpoint. It mirrors the planar curve
follower ({doc}`../curve_follower`), and it is **experimental in 0.x** like the core: its
interfaces may change in a minor release.

**Package.** A new package, `vectorfield_follower_3d`, holds the lifecycle component
`vectorfield_follower_3d::Follower3D` (node `follower_3d`) and a kinematic point simulator
(`point_sim_3d`). We did not put it inside `vectorfield_curve_follower` for three reasons. The
planar follower is coupled to Nav2 (costmap, footprint, interlock), and none of that applies to
an aerial robot. The 3D node has its own parameters, and a separate package keeps them out of the
planar follower's tables and generated reference. An experimental package can change without
touching a stable one. The dependency direction is kept: core ← ros ← curve_follower ←
follower_3d. The node reuses the planar follower's pure helpers (`GoalSupervisor`, `LapCounter`
and the result codes) instead of copying them. The one change to them is additive: the
supervisor's progress watchdog takes an optional `position_z`, and planar callers leave it at 0.

## Interfaces (additive; no existing definition changes)

- **`Curve3.msg`**: POLYLINE (3D points), EXPRESSION (`x/y/z_expression` in `s`, `t`), or
  BUILTIN (`builtin_shape` with parameters). It also carries `closed`, `s_min`/`s_max`, a
  6-DoF `pose` that places a built-in in the curve frame (identity for other types) and a
  `sample_spacing`. The pose becomes the built-in's `cx cy cz yaw pitch roll` parameters. A
  pose given together with any of those parameters is INVALID_CURVE.
- **`FollowCurve3.action`**: the FollowCurve goal with a `Curve3` (`frame_motion`,
  `frame_odom_topic`, `reverse_direction`, `desired_speed`, `laps`, `max_duration`). Its
  result codes have FollowCurve's values, GOAL_REJECTED and SEND_GOAL_FAILURE included.
  NAV2_ACTIVE is reserved, because the 3D node has no Nav2 interlock. The feedback carries
  the distance, arc length, remaining arc length, lap, speed and phase.
- **`FieldState3.msg`**: one sample per cycle on `~/field_state`, for plots and tests. It
  carries the pose, foot, tangent, $D$, the station, lap and progress, $G$, $\eta$, $\Phi$,
  $\Phi_T$, the frame velocity, the CBF barrier and flags, the target and limited commands,
  and $\alpha$.

## Node

- **Inputs.** `odom` (`nav_msgs/Odometry`): the pose in `global_frame` (or a frame TF maps to
  it), and the twist in the child frame (it seeds the limiter). There is no costmap and no
  footprint. The body is the core's sphere of radius `body_radius`.
- **Output.** `cmd_vel` (`geometry_msgs/TwistStamped`). With `cmd_frame: global` it is the ENU
  velocity in `global_frame`. With `cmd_frame: body` it is that velocity rotated by $-\psi$
  about $z$ (the heading frame, FLU) with `frame_id` = `robot_base_frame`. `angular.z` is the
  yaw rate in both cases. A remap feeds MAVROS `setpoint_velocity/cmd_vel` (ENU, stamped)
  directly. PX4 (`TrajectorySetpoint`, NED) and ArduPilot (`/ap/cmd_vel`) need a few lines of
  adapter, which is documented. The package takes no autopilot dependency.
- **Execution model, safety contract and failure table**: as the planar follower. An active
  server never rejects a goal. It validates in execute (`prepareGoal3()`) and terminates an
  invalid goal before publishing. It publishes only while a goal executes: a decelerating ramp,
  then exactly one zero, then silence. A cancel decelerates at `cancel_decel` and then sends
  one zero. The supervisor maps deactivation and kNonFinite to UNKNOWN, `max_duration` to
  TIMEOUT, an untransformable curve frame to TF_ERROR, a stale odometry, moving-frame input or
  voxel cloud to STALE_INPUT, kBlocked (the CBF hold) or no progress to BLOCKED, and a long
  Assumption-3 clamp to ASSUMPTION3_VIOLATED.
- **Goal validation** (pure): unknown `frame_motion`, a negative `max_duration` or a
  non-finite speed is INVALID_GOAL. `makeCurve3` failures, a bad pose and `laps` on an open
  curve are INVALID_CURVE. An open time-varying curve, or FRAME_MOVING without a source, is
  UNSUPPORTED. A goal-scoped configuration that fails `validateTracker3Config` (for example
  `desired_speed < min_approach_speed`) is INVALID_GOAL.
- **Moving frames.** `FrameTwistEstimator3` is fed by the goal's `frame_odom_topic` (Odometry:
  the pose of the curve frame in its parent and the body twist) or by TF differencing of
  `moving_frame_reference` ← curve frame. Its `curveFrame()` goes into `Tracker3Input::frame`.
- **Obstacles.** `obstacle_mode: cbf` uses primitives from the `obstacles` string list
  (`sphere x y z r`, `box x y z hx hy hz [yaw pitch roll]`, `cylinder x y z r [half_height]`,
  `plane px py pz nx ny nz`, all in `global_frame`). An optional `voxel_cloud_topic`
  (`sensor_msgs/PointCloud2`) is voxelised into a cube of `voxel_grid_size`³ cells centred on
  the robot. Its EDT is rebuilt once per new cloud. An older cloud than `voxel_timeout` is a
  stale input. Points outside the cube are dropped. The parameters are dynamic. An update that
  `validateTracker3Config` rejects aborts the goal UNKNOWN, as in the planar follower.
- **Parameters**: one generate_parameter_library YAML (`vectorfield_follower_3d_params`). The
  built-in validators check each key. The core validates the combination at configure (an
  invalid set fails the transition) and at every rebuild.
- **Visualization** (`~/markers`, every `visualization_period`): the curve, the foot and
  tangent, the command, a lattice of field arrows around the robot, the obstacles and
  occupied voxels, the goal and a status text. `~/curve` is the goal's `Curve3` (transient
  local).

## Simulator

`point_sim_3d` integrates the commanded velocity with `d3::sim::integrate` (optional lag
$\tau$). It reads `cmd_vel` in world or body frame from the `frame_id`. It publishes `odom`,
TF `global_frame → base_frame` and, with `publish_clock`, `/clock` at `real_time_factor`.
Optional extras are a moving frame with a constant twist (Odometry and TF) and a
`PointCloud2` of the surfaces of `obstacle_spheres`, so tests and the demo can exercise the
voxel path.

## Tests

- **Pure** (gtest): curve conversion and goal validation, obstacle parsing, the voxeliser,
  output-frame rotation, and the supervisor's 3D watchdog.
- **In process** (simulated time, lock step, as the planar harness): an invalid goal ends
  INVALID_CURVE without a command, an open helix ends SUCCEEDED, laps on a closed curve, a
  cancel gives one zero and then silence, a frame moving through Odometry, and a centred
  sphere gives BLOCKED after `blocked_timeout`.
- **Launch** (`test_follower_3d.py`, all distros): `follower3d_demo.launch.py` runs headless.
  An open helix ends SUCCEEDED near its end. A Viviani curve runs 2 laps, with max $|D|$ after
  lap 1 below 0.05 m. A voxelised sphere beside a circle is passed with $h \ge -$resolution. A
  cancel mid-run gives CANCELED and then silence.

**Not in 0.3.0:** no goal yaw or rotate phases, no collision rollout behind the CBF filter, a
spherical body, static obstacles only, no Nav2 interlock and no diagnostics topic.
