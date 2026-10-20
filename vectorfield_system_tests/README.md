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
| `test_shipped_configs` (gtest) | all | every installed `config/controller/*.yaml`, loaded with `--params-file`, passes `validateFromNode` with the stock 3 × 3 m / 0.05 m costmap and the TB3 footprint in both obstacle modes, (or the local costmap the file sets: 4 × 4 m for the Ackermann files) without errors or warnings; the Ackermann files with contour in the stock 3 m window report only the blend-zone warning |
| `test_merged_configs` (pytest) | all | for every motion model × scenario merge, the smoother limits cover the controller's on every commanded axis; scenario overlays never set smoother limits; the curve_follower merge lists `static_layer` |
| `test_open_kinematic_<model>` | all | `vectorfield_sim` on `open_10x10`: the 3-goal route (each goal SUCCEEDED within 120 s, max \|D\| after convergence < 0.10 m, 0.25 m Ackermann, final position within the goal tolerance, no NoValidControl, no FollowPath failure; Ackermann: no infeasible command); the final-command publishers; an unmapped box 0.8 m ahead, centred and offset 0.1 m (contour engages, clearance ≥ λ − d − 0.05, one cell less for omnidirectional, for Ackermann the closest approach of a turn at min_turning_radius from rest minus 0.05; rejoin, goal reached; Ackermann: a U-turn between the two and no infeasible command); Ackermann: a blocked area (stop ≥ stop_margin − res short, NavigateToPose aborts) |
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

## Known issues

**Jazzy: Nav2's loopback simulator can crash at start-up** (`nav2_loopback_sim` 1.3.13, still
the same on the `jazzy` and `kilted` branches; Lyrical's C++ loopback simulator logs `Map server
returned empty/invalid map ..., will retry` and is not affected).
In about one `test_open_loopback_<model>` run in fifteen, mostly under load, the launch log shows

```text
[map_server]: Received GetMap request but not in ACTIVE state, ignoring!
[loopback_simulator]: Laser scan will be populated using map data
[loopback_simulator]: Received initial pose!
utils.py:71: RuntimeWarning: divide by zero encountered in scalar divide
OverflowError: cannot convert float infinity to integer
```

and the test fails with `no /map or no true pose`. The robot pose is never infinite; the map is
empty:

1. nav2_bringup's `tb3_loopback_simulation.launch.py` starts map_server and the loopback simulator
   together (lines 219-220).
2. The simulator asks for the map once, in its constructor (`loopback_simulator.py:154`,
   `getMap` at 306-324). `wait_for_service` (line 308) returns as soon as map_server is
   configured, because map_server creates the service in `on_configure` (`map_server.cpp:111`).
3. Until map_server is active, `getMapCallback` (`map_server.cpp:180-193`) logs the warning above
   and returns an empty map, resolution 0, which the simulator keeps (line 313).
4. The first `initialpose` (from `initial_pose_publisher`, as the simulator requires) starts the
   scan timer (line 214); `getLaserScan` (line 358) calls `worldToMap`, which divides by the
   resolution (`utils.py:71`), and the process exits.

Nothing in this repository triggers it: the race is between two nodes of the stock wrapper. It
reproduces every time with stock Jazzy packages alone, by holding map_server in the inactive
state:

```bash
ros2 run tf2_ros static_transform_publisher --frame-id base_footprint --child-frame-id base_scan &
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/opt/ros/jazzy/share/nav2_bringup/maps/tb3_sandbox.yaml &
ros2 lifecycle set /map_server configure   # configured, not activated
ros2 run nav2_loopback_sim loopback_simulator &
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 1.0}, orientation: {w: 1.0}}}}'
```

Possible upstream fixes: retry `getMap` until the map has a non-zero resolution, or fail the
GetMap service while map_server is inactive. Until then the coverage build (CI Jazzy amd64), the
most loaded one, does not register the loopback tests (see `CMakeLists.txt`).

**Lyrical: the test client's participant sometimes discovers nothing.** In about one Lyrical
launch test in fifteen, after the previous test's container was killed (SIGKILL), the test
client's own participant gets no /clock, or /clock but no service answer, while the launched
nodes see each other. The client then starts over with a new node and participant
(`no /clock ...; new node` or `map_server never answered get_state ...; new node` in the log)
and leaves the old one undestroyed: destroying it has blocked in rclpy's `destroy_publisher`
until the CTest timeout. Should start-up hang anyway, the client dumps every thread's stack to
the log 60 s after its own time-out.

## Controller comparison

`benchmarks/compare_controllers.py` (Jazzy and later; installed, not a test) drives the 3-goal
route on `loopback_demo` with this controller, Regulated Pure Pursuit, MPPI and Graceful, under
the same limits, goal checker, smoother and collision monitor, and writes
[docs/controller_comparison.md](../docs/controller_comparison.md) and a per-goal CSV. Each run is
its own launch, about one minute on Jazzy:

```bash
ros2 run vectorfield_system_tests compare_controllers.py --runs 5 \
  --out-md docs/controller_comparison.md --out-csv controller_comparison.csv
```

`--controllers vf,rpp,mppi,graceful` selects the controllers, `--log-dir` keeps the launch logs
and `--from-csv` regenerates the page from a recorded CSV (and the JSON next to it). A run that
fails for a Nav2 reason outside the controller plugin (a goal acknowledgement time-out in
bt_navigator, a controller server that stops publishing) is repeated up to `--retries` times
and listed on the page.
