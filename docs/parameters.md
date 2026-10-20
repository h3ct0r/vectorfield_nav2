# Parameters

Both nodes declare their parameters with
[generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library).
Its YAML definitions are the single source of truth: the pages below are generated from them by
`docs/gen_params.py` at every documentation build.

- **Controller plugin:** the common keys (`vectorfield_ros/params/vectorfield_common_params.yaml`)
  and the legacy path-handling keys
  (`vectorfield_nav2_controller/params/vectorfield_legacy_path_params.yaml`). Every key is
  prefixed with the plugin name, for example `FollowPath.desired_speed` under
  `controller_server`; behind the rotation shim on Lyrical and later the prefix is
  `FollowPath.primary_controller.` ({doc}`motion_models`). The legacy path-handling keys exist on
  Humble to Kilted only.
- **Curve follower:** the same common keys plus the follower's own
  (`vectorfield_curve_follower/params/vectorfield_follower_params.yaml`), declared directly on
  the `curve_follower` node, without a prefix. The rotate-to-heading keys are declared but unused.

Most keys can be changed at run time with `ros2 param set`. The read-only ones are
`motion_model`, the legacy path-handling keys, and the follower's `global_frame`,
`robot_base_frame`, `odom_topic` and `exclusive_actions`; each entry below says so. A change is
validated together with the rest of the configuration and refused, with the reason, when it would
make the configuration invalid ({doc}`tuning`).

```{toctree}
:maxdepth: 2

parameters/controller
parameters/follower
```

## Example

The motion-model overlay that the demos merge onto Nav2's stock parameters, for a differential
base (`vectorfield_bringup/config/controller/differential.yaml`). Keys not listed keep their
defaults.

```{literalinclude} ../vectorfield_bringup/config/controller/differential.yaml
:language: yaml
:start-at: "controller_server:"
:end-before: "# Only the motion-model overlays"
```

The other overlays in the same directory cover `skid_steer`, `omnidirectional` and `ackermann`;
the Ackermann overlay also replaces the planner, the goal checker and the behaviour tree
({doc}`motion_models`).
