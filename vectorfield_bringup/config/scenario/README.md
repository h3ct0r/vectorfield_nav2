# config/scenario

Scenario overlays, merged after the motion-model overlay:

- `kinematic_sim.yaml` and `loopback.yaml`: applied by their demos;
- `demo_obstacles.yaml`: unmapped circles and boxes for `vectorfield_sim` on `open_10x10`;
- `curve_follower.yaml`: applied by `curve_follower_demo`: the static layer and a 10 Hz update
  rate for the local costmap, and the follower's `moving_frame_reference` (`odom`).

Scenario overlays never set velocity smoother velocity or acceleration limits; those belong to
the motion-model overlays in `config/controller/`.
