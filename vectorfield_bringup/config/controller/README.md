# config/controller

Motion-model overlays: `differential.yaml`, `skid_steer.yaml`, `omnidirectional.yaml` and
`ackermann.yaml`.

Each overlay replaces the `FollowPath` controller with `VectorFieldController`, sets the
simulator's motion model and sets the velocity smoother limits as full three-element lists that
are at least the controller limits. The omnidirectional overlay also enables lateral velocity;
the Ackermann overlay switches the planner to a Dubins Hybrid-A* search, uses
`PositionGoalChecker` and a behaviour tree without Spin, and enlarges the local costmap to
4 m × 4 m so that the car's contour band fits the rolling window. Every overlay enables
`obstacle_mode: contour`.
