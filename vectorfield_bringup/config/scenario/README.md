# config/scenario

Scenario overlays, merged after the motion-model overlay:

- `kinematic_sim.yaml` and `loopback.yaml`: applied by their demos;
- `demo_obstacles.yaml`: unmapped circles and boxes for `vectorfield_sim` on `open_10x10`;
- `curve_follower.yaml`: the static layer and a 10 Hz update rate for the local costmap, for the
  curve follower (M5).

Scenario overlays never set velocity smoother velocity or acceleration limits; those belong to
the motion-model overlays in `config/controller/`.
