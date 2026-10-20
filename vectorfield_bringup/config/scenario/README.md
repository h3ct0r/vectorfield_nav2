# config/scenario

Scenario overlays, merged after the motion-model overlay:

- `kinematic_sim.yaml` and `loopback.yaml`: applied by their demos;
- `demo_obstacles.yaml`: unmapped circles and boxes for `vectorfield_sim` on `open_10x10`, and
  no obstacle layer in the global costmap, so the plan runs through them and the controller
  contours them;
- `curve_follower.yaml`: applied by `curve_follower_demo`: the static layer and a 10 Hz update
  rate for the local costmap, and the follower's `moving_frame_reference` (`odom`);
- `follow_curve_bt.yaml`: with `curve_follower_demo`, bt_navigator runs
  `behavior_trees/follow_curve.xml` for NavigateToPose (approach the pose, then follow a curve
  with the `FollowCurve` node), and the follower's interlock leaves out the navigator actions.

Scenario overlays never set velocity smoother velocity or acceleration limits; those belong to
the motion-model overlays in `config/controller/`.
