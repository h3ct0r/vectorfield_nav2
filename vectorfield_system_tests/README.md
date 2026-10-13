# vectorfield_system_tests

Cross-package integration and launch tests for vectorfield_nav2. Tests that need several
packages live here, so no package has to test-depend on a package above it. This package is
never released.

**Status:** skeleton — tests are implemented in M3 (controller) and M5 (curve follower). No
tests are registered yet.

## Planned tests

Per pull request, on every gating distro:

- navigation per distro and motion model on the `open_10x10` map, with Nav2's loopback
  simulator (Jazzy and later) and with `vectorfield_sim`;
- obstacle scenarios: a corridor with a door (no contour activation), an unmapped box ahead
  (contour engages and the robot rejoins the path), a blocked corridor (stop, then replan), and
  a goal close to a wall;
- the final `cmd_vel` publishers: only collision_monitor and the allow-listed Nav2 servers;
- validation of every shipped configuration and of the merged overlays;
- from M5: curve following laps, `NAV2_ACTIVE` interlock, stale inputs and open curves.

Nightly and non-gating, registered only with `-DVF_NIGHTLY_TESTS=ON`: Gazebo TB3 (label
`gazebo`) and noise, latency and slip injection (label `nightly`). No nightly tests exist yet.

## Running

```bash
colcon build --packages-up-to vectorfield_system_tests
colcon test --packages-select vectorfield_system_tests
colcon test-result --verbose
```

`colcon test` exits with 0 even when tests fail; `colcon test-result --verbose` reports the
failures. Tests run serially, isolated through `ament_cmake_ros`. Never set `ROS_DOMAIN_ID`;
restrict discovery with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (Jazzy and later) or
`ROS_LOCALHOST_ONLY=1` (Humble).

Nightly tests:

```bash
colcon build --packages-up-to vectorfield_system_tests --cmake-args -DVF_NIGHTLY_TESTS=ON
colcon test --packages-select vectorfield_system_tests --ctest-args -L 'gazebo|nightly'
colcon test-result --verbose
```
