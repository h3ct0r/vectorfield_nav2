# config/controller

Motion-model overlays arrive in M3: `differential.yaml`, `skid_steer.yaml`,
`omnidirectional.yaml` and `ackermann.yaml`.

Each overlay replaces the `FollowPath` controller with `VectorFieldController` and sets the
velocity smoother limits as full three-element lists that are at least the controller limits.
The omnidirectional overlay also enables lateral velocity; the Ackermann overlay switches the
planner to a Dubins Hybrid-A* search, uses `PositionGoalChecker` and a behaviour tree without
Spin.
