# config/scenario

Scenario overlays arrive in later milestones:

- M3: `loopback.yaml`, `kinematic_sim.yaml` and `demo_obstacles.yaml`;
- M5: `curve_follower.yaml`, which adds the static layer to the local costmap so the curve
  follower sees mapped hazards.

Scenario overlays never set velocity smoother velocity or acceleration limits; those belong to
the motion-model overlays in `config/controller/`.
