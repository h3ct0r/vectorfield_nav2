# vectorfield_follower_3d

**Experimental (0.x).** The ROS 2 node of the vectorfield_core 3D core, for drones, underwater
vehicles and other robots that take a 3D velocity setpoint. The interfaces may change in a minor
release.

- **`vectorfield_follower_3d::Follower3D`** (component, executable `follower_3d`, node
  `follower_3d`): a lifecycle node with the `follow_curve_3d` action
  (`vectorfield_msgs/action/FollowCurve3`). It reads `nav_msgs/Odometry` (`odom`) and publishes
  `geometry_msgs/TwistStamped` (`cmd_vel`) in `global_frame` (ENU) or the heading frame
  (`cmd_frame: body`), only while a goal executes. It also publishes `~/curve`,
  `~/field_state` (FieldState3) and `~/markers`. Inside runs `Tracker3`: the 3D field, the yaw
  policy, the limiter, moving frames from Odometry or TF, and CBF obstacle avoidance against
  primitives (`obstacles`) and an optional voxelised `sensor_msgs/PointCloud2`.
- **`point_sim_3d`**: a kinematic point robot for tests and demos. It integrates `cmd_vel` and
  publishes `odom`, TF and `/clock`. It can also move a frame and publish a point cloud of
  spheres.

It mirrors `vectorfield_curve_follower`, whose pure goal helpers (`GoalSupervisor`, `LapCounter`
and the result codes) it reuses: the same execution model, safety contract (a deceleration,
exactly one zero, then silence) and failure table. It has no Nav2 interlock and no costmap.
With `hold_between_goals: true` it streams zero setpoints between goals instead of going
silent, for an autopilot's offboard or guided mode.

```bash
ros2 launch vectorfield_bringup follower3d_demo.launch.py
ros2 run vectorfield_bringup send_curve3d.py --shape viviani --laps 2
```

MAVROS takes the global-frame output on `/mavros/setpoint_velocity/cmd_vel`. PX4 (uXRCE-DDS,
NED) and ArduPilot (`/ap/cmd_vel`) need a remap or a few lines of adapter. See the documentation:
`docs/follower_3d.md` (reference), `docs/tutorials/aerial_3d.md` (tutorial),
`docs/design/m7_3d_ros.md` (design) and the generated parameter reference
(`params/vectorfield_follower_3d_params.yaml`).

## Tests

- `test_follower_3d_helpers`: goal validation (INVALID_CURVE, UNSUPPORTED, INVALID_GOAL), the
  Curve3 conversion and built-in poses, the obstacle entries, the voxeliser, the output frame and
  the markers.
- `test_follower_3d_node`: the node in process on simulated time, in lock step with a point
  robot. It covers an invalid goal (no command), an open helix (SUCCEEDED at its end, one zero,
  silence), two laps of a tilted circle (max |D| after lap 1 < 0.05 m), a cancel (monotone
  deceleration, one zero, silence), a frame moving through Odometry, a centred sphere (hold, then
  BLOCKED), zero setpoints between goals with `hold_between_goals`, configuration checks and
  body-frame output.
- `vectorfield_system_tests/test/test_follower_3d.py`: `follower3d_demo` launched headless.
