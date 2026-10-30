# Distro matrix

vectorfield_nav2 supports Humble, Jazzy and Lyrical. Rolling is built and tested nightly; Kilted
is not supported. The per-distro details of the Nav2 interfaces, with their sources, are in
{doc}`design/nav2_api_matrix`.

## Support

| ROS 2 distro | Ubuntu | Nav2 controller API | CI | Status |
|---|---|---|---|---|
| Humble | 22.04 (jammy) | Gen A: `setPlan`, 3-argument `computeVelocityCommands`; the plugin's own `LegacyPathHandler` | every pull request (amd64) | Supported |
| Jazzy | 24.04 (noble) | Gen B: as Humble, plus `cancel()` and `reset()` | every pull request (amd64 with coverage, arm64) | Supported |
| Kilted | 24.04 (noble) | Gen B | none | Unsupported: shares Jazzy's API and may build, but has no CI, configuration or release |
| Lyrical | 26.04 (resolute) | Gen C: `newPathReceived`, 5-argument `computeVelocityCommands`; the server's path handler | every pull request (amd64) | Supported |
| Rolling | 26.04 (resolute) | Gen C | nightly, against the Nav2 nightly image, without launch tests | Built and tested; failures do not gate |

A release requires green CI on Humble, Jazzy (amd64 and arm64) and Lyrical.
Rolling failures become issues. Nav2 is not released into Rolling, so vectorfield_nav2 has no
Rolling binaries.

## Features per distro

| Feature | Humble | Jazzy | Lyrical (and Rolling) |
|---|---|---|---|
| Controller plugin, all four motion models | yes | yes | yes |
| Plan window | `LegacyPathHandler` (a port of Nav2's `FeasiblePathHandler`; keys `prune_distance`, `max_robot_pose_search_dist`, `transform_tolerance`, `reject_unit_path`) | as Humble | the controller server's path handler; the legacy keys do not exist |
| XY-reached test | latched $\lVert p - g\rVert \le \mathrm{xy\_tol}$, re-armed on every `setPlan()` | as Humble | the goal checker's `isGoalXYReached` |
| `cancel()` (decelerate to a stop, then end the goal) | not called by the server | yes | yes |
| `reset()` | emulated: a new `setPlan()` after more than 3 idle control periods resets | on every goal exit | on every goal exit |
| `controller_server` output | `Twist` | `Twist`, or `TwistStamped` with `enable_stamped_cmd_vel` | `TwistStamped` by default |
| Rotation shim key layout | primary keys under `FollowPath` | as Humble | primary keys under `FollowPath.primary_controller` |
| Shim with `obstacle_mode: contour` | warned, not supported | with `rotate_to_heading_once: true` | warned, not supported |
| Curve follower | yes | yes | yes |
| Docking and following servers seen by the follower's interlock | none exist | `dock_robot`, `undock_robot` | also `follow_object` |
| generate_parameter_library | 0.7.7 | 0.7.7 | 1.3.1 |

The command chain is Nav2's: the controller's `cmd_vel` is remapped to `cmd_vel_nav`, then the
velocity smoother, then collision_monitor, then `cmd_vel`. Humble's stock launch has no
collision_monitor between the smoother and the base, so the Humble demos add one
(`config/humble/collision_monitor.yaml`, from `cmd_vel` to `cmd_vel_safe`).

## Demos per distro

| Launch file | Humble | Jazzy | Lyrical |
|---|---|---|---|
| `kinematic_sim_demo.launch.py` | yes, with collision_monitor added and the simulator on `cmd_vel_safe` | yes | yes; starts `lifecycle_manager_navigation` itself, because Lyrical's `navigation_launch.py` has none |
| `curve_follower_demo.launch.py` | yes | yes | yes |
| `loopback_demo.launch.py` | no (no `nav2_loopback_sim`) | yes | yes |
| `tb3_gazebo_demo.launch.py`, `tb4_gazebo_demo.launch.py` | no | yes (Gazebo Harmonic) | yes (Gazebo Jetty) |
| `vectorfield_mujoco` examples ({doc}`examples/index`) | yes | yes | yes |

Each demo exits with an error that names the supported distros when it cannot run on the current
one.

## Binary packages

Releases go through bloom for Humble, Jazzy and Lyrical, on the platforms the ROS build farm
builds each distro for:

| Distro | Ubuntu (amd64, arm64) | Other platforms of the distro | vectorfield_nav2 binaries |
|---|---|---|---|
| Humble | 22.04 jammy | RHEL 8 | Ubuntu only: Nav2 has no RHEL packages for Humble, so the RHEL jobs are skipped |
| Jazzy | 24.04 noble | Debian bookworm, RHEL 9 | Ubuntu only, for the same reason |
| Lyrical | 26.04 resolute | Debian trixie, Fedora 43, RHEL 10 | every released package; on RHEL 10 every package except `vectorfield_bringup` |

On Lyrical, `vectorfield_bringup` has no installable RHEL 10 RPM, because its dependencies
(`nav2_bringup`, the TurtleBot simulations, `ros_gz`) are not built for RHEL 10; those build-farm
failures are expected. `vectorfield_bringup` is not part of the `vectorfield_nav2` metapackage,
so installing the stack on a robot does not pull in the simulators. `vectorfield_system_tests` is
never released.

`vectorfield_mujoco` is not released either: it depends on MuJoCo through the pip rosdep key
`python3-mujoco-pip`, which the build farm does not accept. It is source-only; build it in a
workspace with rosdep run as `PIP_BREAK_SYSTEM_PACKAGES=1` on Noble and later
({doc}`examples/index`).

0.6.0 is released; until its binaries are available, build from source ({doc}`quickstart`).
