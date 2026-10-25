(follower-3d)=

# 3D follower (experimental)

:::{warning}
The 3D follower is **experimental in 0.x**, like the {doc}`3D core <core_3d>` it runs. Its
interfaces (`Curve3`, `FollowCurve3`, `FieldState3`, the parameters) may change in a minor
release. Test it in simulation and with a safety pilot before you fly it.
:::

`vectorfield_follower_3d` is the ROS 2 node of the 3D core. It is for drones, underwater
vehicles and other robots that take a 3D velocity setpoint. It serves a `follow_curve_3d` action
(`vectorfield_msgs/action/FollowCurve3`), reads the robot's `nav_msgs/Odometry`, and publishes
`geometry_msgs/TwistStamped` velocity setpoints while a goal executes. Inside it runs
`Tracker3`: the field $\Phi = \eta\Phi_S + \Phi_T$ in $\mathbb R^3$, the yaw policy, the
limiter, moving frames and the CBF obstacle filter. The node mirrors the planar
{doc}`curve_follower`: the same execution model, safety contract, failure table and result codes.
It has no Nav2 dependency at run time and no costmap. The design is in {doc}`design/m7_3d_ros`.

## Try it

```bash
ros2 launch vectorfield_bringup follower3d_demo.launch.py
ros2 run vectorfield_bringup send_curve3d.py --shape helix
ros2 run vectorfield_bringup send_curve3d.py --shape viviani --laps 2
ros2 run vectorfield_bringup send_curve3d.py --shape torus_knot --laps 1 --speed 0.6
ros2 param set /point_sim_3d obstacle_spheres "[0.6, 0.0, 1.3, 0.25]"   # a sphere to avoid
```

The demo starts `point_sim_3d`, a kinematic point robot that publishes `/odom`, TF `map →
base_link` and `/clock`. It also starts `follower_3d` under a Nav2 lifecycle manager, and RViz
with `rviz/follower_3d.rviz`. The {doc}`tutorial <tutorials/aerial_3d>` goes through the demo
and the autopilot setup.

## The action

A `FollowCurve3` goal has a `Curve3`, the planar goal fields (`frame_motion`,
`frame_odom_topic`, `reverse_direction`, `desired_speed`, `laps`, `max_duration`) and the
planar result codes. Its feedback carries the distance to the curve, the arc length, the
remaining arc length, the lap, the 3D speed and the phase.

| `Curve3` field | Meaning |
|---|---|
| `type` | `POLYLINE` (`points`, 3D), `EXPRESSION` (`x_expression`, `y_expression`, `z_expression` over `[s_min, s_max]`) or `BUILTIN` (`builtin_shape`) |
| `header` | `frame_id` is the curve frame (`""` = `global_frame`). `stamp` is $t_0$ of $r(s, t)$ (zero = goal acceptance) |
| `closed` | Polylines and expressions. Built-ins decide (the `helix` is open, the others closed) |
| `parameter_names`, `parameter_values` | Constants of an expression, or a built-in's parameters ({ref}`core-3d`) |
| `pose` | Places a built-in in the curve frame: it becomes `cx cy cz yaw pitch roll` (ZYX). An all-zero quaternion is the identity. It must be the identity for the other types, and must not be combined with those six parameters |
| `sample_spacing` | Spacing of the drawn curve (m; <= 0: `default_sample_spacing`). The 3D curves are evaluated exactly |

**Success.** A closed curve succeeds after `laps` laps (0 runs until cancelled). An open curve
succeeds when the robot has settled within `goal_tolerance / 2` of its end (its start with
`reverse_direction`). `laps > 0` on an open curve is INVALID_CURVE.

## Inputs, output and autopilots

- **Odometry** (`odom_topic`). The pose is in `global_frame`, or in a frame that TF maps into
  it. The twist is in the child frame. Its age is checked against `input_timeout`. The yaw is
  the heading of the pose's rotation.
- **Velocity setpoints** (`cmd_vel_topic`, `TwistStamped`). They are published only while a
  goal executes. With `cmd_frame: global` they carry the ENU velocity in `global_frame`
  (`frame_id` = `global_frame`). With `cmd_frame: body` they carry the same velocity rotated by
  $-\psi$ about $z$: the heading frame, x forward, y left, z up, with `frame_id` =
  `robot_base_frame`. Roll and pitch are ignored. `angular.z` is the yaw rate in both cases.

The package depends on no autopilot stack. Feed it with remaps or a few lines of adapter:

- **MAVROS** (PX4 or ArduPilot over MAVLink). Remap `cmd_vel` to
  `/mavros/setpoint_velocity/cmd_vel` (TwistStamped, ENU, `cmd_frame: global`) and `odom` to
  `/mavros/local_position/odom`, with `global_frame` set to the frame of that odometry (`map`
  in MAVROS's defaults).
- **PX4 over uXRCE-DDS** (`px4_msgs`). PX4 takes `TrajectorySetpoint` in NED plus an
  `OffboardControlMode` heartbeat. An adapter node maps a global-frame command
  $(v_x, v_y, v_z, \omega)$ to `velocity = [v_y, v_x, -v_z]` and `yawspeed = -ω`, and the
  vehicle odometry back to ENU.
- **ArduPilot DDS** (`/ap/cmd_vel`, TwistStamped). Check which `frame_id` your firmware
  accepts. Recent releases take `base_link` (body, FLU: `cmd_frame: body`,
  `robot_base_frame: base_link`), and some also take the ENU map frame. Feed `/ap/odom` or
  your state estimate as `odom`.

:::{important}
By default the follower is silent between goals (the safety contract below), and PX4's
offboard mode fails safe when setpoints stop. Set `hold_between_goals: true` and, while active
and without a goal, the follower publishes a zero setpoint every control period
(`controller_frequency`), which keeps PX4 in offboard and holds ArduPilot in guided mode. No
other node may publish on the same topic then. Arm, take off and switch modes outside the
follower.
:::

## Moving frames

`frame_motion: FRAME_MOVING` follows a curve in a frame that moves rigidly, for example a
helix around a moving platform (math_spec §14.7). The node feeds `FrameTwistEstimator3` from one
of two sources.

- **Odometry**: the goal's `frame_odom_topic`, whose `child_frame_id` must be the curve frame.
  Its pose is the frame in `header.frame_id`, and its twist is the frame's body twist.
- **TF**: when `frame_odom_topic` is empty, the node differences `moving_frame_reference` ←
  curve frame.

A stale source makes the robot decelerate, and after `stale_abort_timeout` the goal aborts
STALE_INPUT. Open curves ride with the frame, so the goal hold moves with it.
`point_sim_3d` can move a frame (`moving_frame:=platform` in the demo), and `send_curve3d.py
--shape moving` follows a circle in it.

## Obstacles

With `obstacle_mode: cbf` the CBF filter of the 3D core keeps a sphere of `body_radius` at
`cbf_margin` from every obstacle ({ref}`core-3d-obstacles`). Obstacles are static in
`global_frame` and come from two sources.

- **Primitives** in the `obstacles` parameter, one string each: `sphere x y z r`,
  `box x y z hx hy hz [yaw pitch roll]`, `cylinder x y z r [half_height]` (vertical; infinite
  without `half_height`), and `plane px py pz nx ny nz` (the solid half-space behind the
  normal, for a floor or a wall). The parameter is dynamic, and an update with an invalid
  entry is refused.
- **A point cloud** (`voxel_cloud_topic`, `sensor_msgs/PointCloud2` with float x, y, z). The
  node voxelises it into a cube of `voxel_grid_size`³ voxels of `voxel_resolution`, centred on
  the robot and snapped to the voxel lattice. It rebuilds the cube and its EDT once per cloud,
  and when the robot leaves the central half of the cube. The cloud is transformed into
  `global_frame` with the latest TF. Points outside the cube are dropped. With `cbf`, a
  missing cloud, or one older than `voxel_timeout`, is a stale input. Set `voxel_timeout <= 0`
  for a map that is republished rarely. Voxel distances are measured to voxel centres; the
  filter counts each voxel site from its cube, half a voxel diagonal closer, so the margin holds
  to the voxel cubes, which over-approximate the obstacle by up to that much.

When the filter leaves less than 5 % of the field speed, for example with an obstacle centred on
the curve, the robot holds with status BLOCKED. After `blocked_timeout` the goal aborts BLOCKED.
Keep `cbf_gain * desired_speed <= max_decel`.

## Safety contract and failures

As in the planar follower, an active node never rejects a goal: it validates it first and
terminates an invalid one before any command. A goal ends in one of three ways. A deceleration
ramp at `max_decel` and `max_yaw_accel` ends with exactly one zero. A cancel decelerates at
`cancel_decel` and then sends one zero. An immediate abort sends one zero at once. After that
the node is silent until the next goal, or, with `hold_between_goals`, publishes zeros at the
control rate.

| Result | Cause | Ending |
|---|---|---|
| `INVALID_GOAL` | Unknown `frame_motion`, negative `max_duration`, non-finite `desired_speed`, or a goal configuration that the core refuses (e.g. `desired_speed < min_approach_speed`) | Before any command |
| `INVALID_CURVE` | `makeCurve3` refuses the curve, a bad `pose`, or `laps` on an open curve | Before any command |
| `UNSUPPORTED` | An open time-varying curve, FRAME_MOVING without a source, or a `frame_odom_topic` whose child is not the curve frame | Before any command / one zero |
| `TF_ERROR` | The curve frame stayed untransformable for `stale_abort_timeout` | Zero, then abort |
| `STALE_INPUT` | The odometry, the moving frame or the voxel cloud stayed stale for `stale_abort_timeout` | Decelerate, then abort |
| `BLOCKED` | The CBF hold for `blocked_timeout`, or no 3D progress within `progress_timeout` | One zero |
| `TIMEOUT` | `max_duration` | Decelerate |
| `ASSUMPTION3_VIOLATED` | The feedforward clamp of a moving curve held for `assumption3_abort_time` | Decelerate |
| `UNKNOWN` | Deactivation, a non-finite command, or a parameter update the core refuses | One zero |

## Topics

| Topic | Type | Notes |
|---|---|---|
| `follow_curve_3d` | action `FollowCurve3` | |
| `odom` | `nav_msgs/Odometry` (sub) | `odom_topic`, best-effort QoS (matches reliable publishers) |
| `cmd_vel` | `geometry_msgs/TwistStamped` (pub) | `cmd_vel_topic`, only while a goal executes |
| `~/curve` | `vectorfield_msgs/Curve3` | The goal's curve, transient local |
| `~/field_state` | `vectorfield_msgs/FieldState3` | Every cycle: pose, foot, $D$, lap, $\Phi$, CBF barrier, commands |
| `~/markers` | `visualization_msgs/MarkerArray` | Every `visualization_period` while subscribed: `curve`, `closest_point`, `tangent`, `command`, `field` (arrows of $\Phi$ with three-stroke heads on a lattice of `field_arrows_step` within `field_arrows_extent`, at most 7 per axis, coloured viridis by the distance to the curve, like the 2D field grid), `goal`, `obstacles`, `voxels`, `body`, `status` |
| `voxel_cloud_topic` | `sensor_msgs/PointCloud2` (sub) | Optional |
| `<frame_odom_topic>` | `nav_msgs/Odometry` (sub) | Moving-frame goals |

The parameters are listed in {doc}`parameters/follower_3d`.

## Limitations

- **Experimental.** The interfaces may change in 0.x.
- **Obstacles.** The body is a sphere, and obstacles are static. There is no collision rollout
  behind the CBF filter and no contour mode in 3D. Points outside the voxel cube are not seen.
- **Yaw.** There is no goal yaw and no rotate phase. The yaw follows the policy
  (`path_tangent`, `field` or `fixed`).
- **Interlock.** There is no Nav2 interlock and no diagnostics topic. Two nodes must not drive
  the same vehicle.
- **Parameters.** A parameter update that the core refuses aborts the running goal UNKNOWN. Only
  `obstacles` is checked before it is set.
- **Transport.** `cmd_frame: body` uses the heading frame only. Autopilots that expect a full
  body frame (with roll and pitch) need an adapter.
