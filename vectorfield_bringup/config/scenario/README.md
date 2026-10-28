# config/scenario

Scenario overlays, merged after the motion-model overlay:

- `kinematic_sim.yaml` and `loopback.yaml`: applied by their demos; both give bt_navigator a
  1 s `default_server_timeout` (Nav2's is 20 ms) to accept goals on a loaded machine;
- `demo_obstacles.yaml`: unmapped circles and boxes for `vectorfield_sim` on `open_10x10`, and
  no obstacle layer in the global costmap, so the plan runs through them and the controller
  contours them;
- `curve_follower.yaml`: applied by `curve_follower_demo`: the static layer and a 10 Hz update
  rate for the local costmap, and the follower's `moving_frame_reference` (`odom`);
- `reversing.yaml`: Smac Hybrid-A* with Reeds-Shepp primitives and the controller's
  `allow_reversing`, so plans that back up (a goal behind the robot, a three-point turn) are
  followed segment by segment, with a stop at every cusp (differential, skid-steer, Ackermann),
  backing up at `max_reverse_vel` 0.2 m/s;
- `follow_curve_bt.yaml`: with `curve_follower_demo`, bt_navigator runs
  `behavior_trees/follow_curve.xml` for NavigateToPose (approach the pose, then follow a curve
  with the `FollowCurve` node), and the follower's interlock leaves out the navigator actions.

Scenario overlays never set velocity smoother velocity or acceleration limits, nor any controller
limit but `max_reverse_vel`, which can only lower `max_linear_vel`; those belong to the
motion-model overlays in `config/controller/`. A scenario may set the controller's behaviour keys
`allow_reversing` and `cusp_tolerance`, and `max_reverse_vel`, which merge into the
motion-model overlay's `FollowPath` (an overlay without a `plugin` key never replaces it).
