# vectorfield_sim

A lightweight kinematic simulator for headless integration tests and demos. It runs on every
supported distro, including Humble, which has no Nav2 loopback simulator, and it adds what the
loopback simulator lacks: unmapped obstacles and skid-steer and Ackermann realism.

## Node

`kinematic_sim` (executable) or the component `vectorfield_sim::KinematicSimNode`; the node is
named `vectorfield_sim`. Launch it through `kinematic_sim_demo.launch.py` in
`vectorfield_bringup`.

- **Motion:** integrates the commanded twist with `vectorfield_core`'s exact-arc
  `sim::integrate` for `motion_model` `differential`, `skid_steer`, `omnidirectional` or
  `ackermann`, at a fixed `step_size` (0.01 s). Skid-steer adds slip (`slip_eta_psi` 0.75,
  `slip_x_icr` 0.05 m). Ackermann uses `min_turning_radius` (0.5 m, must be > 0) and
  `reference_point_offset` (0 m), and counts infeasible commands (v = 0 with ω ≠ 0) on
  `~/infeasible_command_count` (`std_msgs/UInt64`, transient local).
- **Input:** `cmd_vel_topic` (`cmd_vel`; Humble demos use `cmd_vel_safe`, after
  collision_monitor). `Twist` or `TwistStamped` follows `enable_stamped_cmd_vel`, whose default
  matches the distro's Nav2 default; overlays never set it. A command older than
  `cmd_vel_timeout` (0.5 s of simulated time; 0 disables it) is replaced by zero.
- **Time:** the node is the time source. Each step publishes `/clock`, and every stamp is
  simulated time. Steps run at `real_time_factor` (1.0) times real time; 0 stops the timer.
- **Outputs:** `odom` (with the executed body twist), the TF `odom → base_footprint`, a static
  `map → odom` (from `initial_pose_x`, `initial_pose_y`, `initial_pose_yaw`) and
  `~/ground_truth` (the true pose in the map frame).
- **Laser scan:** `scan` in `base_scan` (`scan_beams` 360 over a full turn from −π, `scan_rate`
  10 Hz, `scan_range_max` 30 m, no return = +inf unless `scan_use_inf` is false), ray-cast
  exactly from base_footprint against `/map` (`map_topic`, transient local) and the unmapped
  obstacles.
- **Unmapped obstacles** (map frame, settable at run time, validated as a whole):
  `circle_obstacles` = `[x, y, radius, ...]` and `box_obstacles` = `[x, y, size_x, size_y, yaw,
  ...]`. A malformed list is rejected with an error, at start-up and at run time.
- **Localization:** `initialpose` (map frame) teleports the robot by moving `map → odom`.
  `~/step_map_to_odom` (`nav2_msgs/srv/SetInitialPose`) adds the request pose's (x, y, yaw) to
  the published `map → odom`, component by component, while the true pose (and the laser) stays
  put: a localization jump.

Every parameter except the two obstacle lists is read-only. The pure parts (`KinematicPlant`,
`RayCaster`, the obstacle parsers) are in the ROS-free library `vectorfield_sim`.
