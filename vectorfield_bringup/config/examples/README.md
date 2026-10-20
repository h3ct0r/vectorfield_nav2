# config/examples

Files to start your own configuration from (see the "Your robot" tutorial):

- `nav2_params_vectorfield_differential.yaml`, `nav2_params_vectorfield_omnidirectional.yaml`,
  `nav2_params_vectorfield_ackermann.yaml`: annotated blocks to merge into your own
  `nav2_params.yaml` (the goal checker and `FollowPath` of `controller_server`, the velocity
  smoother and collision monitor settings; for Ackermann also the planner and the behaviour
  tree). They also work as demo overlays (`scenarios:=<path>`).
- `rotation_shim_humble_jazzy.yaml`, `rotation_shim_lyrical.yaml`: the controller behind Nav2's
  `RotationShimController`, in each distro's key layout, as demo overlays.

`vectorfield_system_tests` validates every file here with the controller's validator, and
`vectorfield_bringup`'s `test_overlays` checks the overlay rules (smoother limits, goal checker,
planner and behaviour tree).
