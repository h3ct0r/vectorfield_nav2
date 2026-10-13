# Parameters

Both nodes declare their parameters with
[generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library).
Its YAML definitions are the single source of truth: the pages below are generated from them by
`docs/gen_params.py` at every documentation build. The controller's definitions arrive in M2,
and the curve follower's own keys in M5; until then the pages say so.

- **Controller plugin:** every key is prefixed with the plugin name, for example
  `FollowPath.desired_speed` under `controller_server`. The legacy path-handling keys apply on
  Humble to Kilted only.
- **Curve follower:** keys are declared directly on the `curve_follower` node, without a prefix.

```{toctree}
:maxdepth: 2

parameters/controller
parameters/follower
```
