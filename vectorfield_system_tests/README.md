# vectorfield_system_tests

Cross-package integration and launch tests for vectorfield_nav2. Tests that need several
packages live here, so no package has to test-depend on a package above it. This package is
never released.

**Status:** the controller tests (M3) and the curve follower tests (M5) are implemented.

## Tests

Per pull request, on every gating distro (`colcon test`; launch tests keep the default
`launch_test` label):

| Test | Distros | Checks |
|---|---|---|
| `test_shipped_configs` (gtest) | all | every installed `config/controller/*.yaml`, loaded with `--params-file`, passes `validateFromNode` with the stock 3 × 3 m / 0.05 m costmap and the TB3 footprint in both obstacle modes, without errors or warnings; `ackermann.yaml` with contour reports only the Ackermann+contour error |
| `test_merged_configs` (pytest) | all | for every motion model × scenario merge, the smoother limits cover the controller's on every commanded axis; scenario overlays never set smoother limits; the curve_follower merge lists `static_layer` |
| `test_open_kinematic_<model>` | all | `vectorfield_sim` on `open_10x10`: the 3-goal route (each goal SUCCEEDED within 120 s, max \|D\| after convergence < 0.10 m, 0.25 m Ackermann, final position within the goal tolerance, no NoValidControl, no FollowPath failure; Ackermann: no infeasible command); the final-command publishers; an unmapped box 0.8 m ahead, centred and offset 0.1 m (contour engages, clearance ≥ λ − d − 0.05, one cell less for omnidirectional, rejoin, goal reached); Ackermann: a blocked area (stop ≥ stop_margin − res short, NavigateToPose aborts) |
| `test_open_loopback_<model>` | Jazzy and later | the same route and final-command publishers on Nav2's loopback simulator |
| `test_curve_laps_differential` | all | `curve_follower_demo` on `open_10x10`: 3 laps each of an ellipse 2 × 1 m and a Bernoulli lemniscate (a = 2 m) at 0.4 m/s, each approached with NavigateToPose as `send_curve.py` does: SUCCEEDED with 3 laps, max \|D\| after lap 1 < 0.05 m |
| `test_curve_moving_differential` | all | `moving_frame_publisher.py` frames: a circle (r 1.5 m) translating at 0.1 m/s and an ellipse rotating at 0.05 rad/s about a point 1 m from its centre, each with the Odometry and the TF source: max \|D\| after 10 s < 0.10 m over 30 s, no `STATUS_ASSUMPTION3_CLAMPED`; the publisher killed mid-run: `STALE_INPUT` |
| `test_curve_open_differential` | all | an open polyline at 0.4 m/s: SUCCEEDED within length/v_r + 10 s; an open curve at 0.04 m/s: SUCCEEDED, never `BLOCKED`; a goal while NavigateToPose executes: `NAV2_ACTIVE`, no follower cycle |
| `test_corridor_kinematic_<model>` | all; differential, skid-steer, omnidirectional | `corridor_door` with contour on: through the door before and after a 0.05 m / 2° map → odom step (no contour activation, clearance ≥ 0.05 m); a goal 0.4 m from a wall; a blocked corridor, with the contour on (differential only: a dead end, no side probe passes) and with `obstacle_mode: none` (stop ≥ stop_margin − res short, NoValidControl, FollowPath fails, Nav2 replans) |

The final command (`cmd_vel` on Jazzy and later, `cmd_vel_safe` on Humble) may be published only by
collision_monitor and the allow-listed docking_server (Jazzy and later) and following_server
(Lyrical and later). The launch tests share the helpers of the `vectorfield_system_tests` Python
package (`launch`, `harness`, `geometry`, `checks`, and `follower` for the curve follower),
imported from the source tree, and the test overlay `test/config/global_obstacle_layer_off.yaml`.

Nightly and non-gating, registered only with `-DVF_NIGHTLY_TESTS=ON`: Gazebo TB3 (label
`gazebo`) and noise, latency and slip injection (label `nightly`). No nightly tests exist yet.

## Running

```bash
colcon build --packages-up-to vectorfield_system_tests
colcon test --packages-select vectorfield_system_tests
colcon test-result --verbose
```

`colcon test` exits with 0 even when tests fail; `colcon test-result --verbose` reports the
failures. Tests run serially, without retries, isolated through `ament_cmake_ros`, and set
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (Jazzy and later) or `ROS_LOCALHOST_ONLY=1` (Humble)
themselves. Never set `ROS_DOMAIN_ID`. On Jazzy a controller launch test takes about one minute,
a curve follower test from one and a half to three and a half minutes (600 s timeout), and the
whole package about fifteen; to run one:

```bash
colcon test --packages-select vectorfield_system_tests --ctest-args -R test_open_kinematic_differential
```

Nightly tests:

```bash
colcon build --packages-up-to vectorfield_system_tests --cmake-args -DVF_NIGHTLY_TESTS=ON
colcon test --packages-select vectorfield_system_tests --ctest-args -L 'gazebo|nightly'
colcon test-result --verbose
```
