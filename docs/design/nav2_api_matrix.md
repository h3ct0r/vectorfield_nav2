# Nav2 API matrix

This is the per-distro Nav2 integration reference. `vectorfield_ros` (`compat.hpp`), `vectorfield_nav2_controller`, `vectorfield_sim`, `vectorfield_bringup`, `vectorfield_system_tests` and `vectorfield_curve_follower` are implemented against it. For each supported Nav2 release it records what Nav2 does, where the source says so, and the project rule that follows.

## How to read this page

**Scope.** Humble, Jazzy and Lyrical gate CI. Rolling (`navigation2` branch `main`) runs nightly and does not gate. Kilted is unsupported. It appears only where it differs from Jazzy, and only as a reference.

**Verification.** All facts were read on 2026-10-13 from the branch heads of `ros-navigation/navigation2` (`humble`, `jazzy`, `kilted`, `lyrical`, `main`) and of the other repositories listed below. Line numbers refer to those heads. Branches move, so re-check a line before relying on it in review.

**Markers.**

- No marker: re-read for this page.
- `(R)`: checked against the cited primary source during the M0 research and review rounds (2026-10-12/13) but not re-read for this page.
- `UNVERIFIED`: not checked against a primary source.
- **Rule**: a project decision that follows from the facts. Rules bind milestones M2 to M5.

**Citation forms.**

| Form | Repository |
|---|---|
| `<branch>:<path>#L<a>-L<b>`, where branch is one of `humble`, `jazzy`, `kilted`, `lyrical`, `main` | <https://github.com/ros-navigation/navigation2>, file `blob/<branch>/<path>#L<a>-L<b>` |
| `geometry2/<branch>:<path>` | <https://github.com/ros2/geometry2> |
| `rclcpp/<branch>:<path>` | <https://github.com/ros2/rclcpp> |
| `rcl/<branch>:<path>` | <https://github.com/ros2/rcl> |
| `common_interfaces/<branch>:<path>` | <https://github.com/ros2/common_interfaces> |
| `ament_cmake/<branch>:<path>` | <https://github.com/ament/ament_cmake> |
| `ament_cmake_ros/<branch>:<path>` | <https://github.com/ros2/ament_cmake_ros> |
| `gpl/<tag>:<path>` (tags `0.7.7`, `1.3.1`) | <https://github.com/PickNikRobotics/generate_parameter_library> |
| `nav2_docker/main:<path>` | <https://github.com/ros-navigation/nav2_docker> |
| `rosdistro:<distro>` | `<distro>/distribution.yaml` in <https://github.com/ros/rosdistro> (`master`) |
| `ros2_documentation/rolling:<path>` | <https://github.com/ros2/ros2_documentation> |
| `gazebosim/docs:<path>` | <https://github.com/gazebosim/docs> |

**Path aliases** (navigation2 paths used in citations).

| Alias | Path |
|---|---|
| `CTRL` | `nav2_core/include/nav2_core/controller.hpp` |
| `GC` | `nav2_core/include/nav2_core/goal_checker.hpp` |
| `PH` | `nav2_core/include/nav2_core/path_handler.hpp` |
| `CS` | `nav2_controller/src/controller_server.cpp` |
| `CSH` | `nav2_controller/include/nav2_controller/controller_server.hpp` |
| `FPH` | `nav2_controller/plugins/feasible_path_handler.cpp` |
| `SGC` | `nav2_controller/plugins/simple_goal_checker.cpp` |
| `PGC` | `nav2_controller/plugins/position_goal_checker.cpp` |
| `SHIM` | `nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp` |
| `NAVL` | `nav2_bringup/launch/navigation_launch.py` |
| `PARAMS` | `nav2_bringup/params/nav2_params.yaml` |
| `LN` | `nav2_ros_common/include/nav2_ros_common/lifecycle_node.hpp` (Lyrical and later) |
| `TF2F` | `nav2_ros_common/include/nav2_ros_common/tf2_factories.hpp` (Lyrical and later) |

## 1. Distro matrix

### 1.1 Versions and platforms

| | Humble | Jazzy | Kilted (reference) | Lyrical | Rolling | Source |
|---|---|---|---|---|---|---|
| Project status | gating | gating (amd64 and arm64) | unsupported | gating | nightly, non-gating | project decision |
| navigation2 branch | `humble` | `jazzy` | `kilted` | `lyrical` | `main` | GitHub branch list |
| navigation2 release | 1.1.20 | 1.3.13 | 1.4.2 | 1.5.2 | not released; `main` declares 1.5.0 | `rosdistro:<distro>`; `main:nav2_core/package.xml` |
| rclcpp | 16.0.22 | 28.1.22 | 29.5.11 | 32.0.3 | 33.1.0 | `rosdistro:<distro>` |
| geometry2 | 0.25.24 | 0.36.23 | 0.41.10 | 0.45.10 | 0.46.4 | `rosdistro:<distro>` |
| generate_parameter_library (gpl) | 0.7.7 | 0.7.7 | 0.7.7 | 1.3.1 | 1.3.1; apt only from `ros2-testing` (R) | `rosdistro:<distro>` |
| Ubuntu | 22.04 jammy | 24.04 noble | 24.04 noble | 26.04 resolute | 26.04 resolute | `rosdistro:<distro>` (`release_platforms`) |
| Other release platforms | RHEL 8 | Debian bookworm, RHEL 9 | Debian bookworm, RHEL 9 | Debian trixie, Fedora 43, RHEL 10 | Debian trixie, Fedora 44, RHEL 10 | `rosdistro:<distro>` |
| C++ standard used by Nav2 | 17 | 17 | 17 | 20 | 20 | `humble:nav2_common/cmake/nav2_package.cmake#L31-L36`, `lyrical:nav2_common/cmake/nav2_package.cmake#L31-L40` |
| C++ standard pushed onto dependents | C++17 (R) | C++17 (R) | C++17 (R) | C++20: rclcpp links `ament_ros_defaults`, which sets `cxx_std_20` | as Lyrical (R) | `ament_cmake_ros/lyrical:ament_cmake_ros_core/cmake/ament_ros_defaults.cmake#L16`, `rclcpp/lyrical:rclcpp/CMakeLists.txt#L220` |
| Gazebo pairing | Fortress; Nav2's own Humble TB3 demo uses Gazebo Classic (`gzserver`) | Harmonic | Ionic | Jetty | Jetty | `gazebosim/docs:common/ros_installation.md#L39-L45`, `humble:nav2_bringup/launch/tb3_simulation_launch.py#L162` |
| EOL | May 2027 | May 2029 | December 2026 | May 2031 | n/a | `ros2_documentation/rolling:source/Releases.rst#L135-L165` |
| `ros-<distro>-navigation2` from apt | yes | yes | yes | yes | no: build from source or use `nav2_docker` | `rosdistro:<distro>` |
| `nav2_ros_common` | no | no | no | yes | source only | `rosdistro:lyrical` |
| `nav2_loopback_sim` | no | yes | yes | yes | source only | `rosdistro:<distro>` |
| `nav2_minimal_tb3_sim`, `nav2_minimal_tb4_sim` | no | 1.0.1 | 1.2.0 | 1.3.0 | 1.3.0 (released although Nav2 is not) | `rosdistro:<distro>` (`nav2_minimal_turtlebot_simulation`) |
| `opennav_docking` | no | yes, started by stock launch | yes | yes, started by stock launch | source only | `rosdistro:<distro>`, `jazzy:NAVL#L43-L54`, `lyrical:NAVL#L28-L40` |
| `opennav_following` | no | released, but stock launch does not start it | no | yes, started by stock launch | source only | `rosdistro:<distro>`, `jazzy:NAVL#L43-L54`, `lyrical:NAVL#L28-L40` |

### 1.2 Container images

| Image | Arch | Contents | Source |
|---|---|---|---|
| `ros:<distro>-ros-base-<codename>` | amd64, arm64 | Official image with colcon, rosdep and vcstool (R). Used by the gating CI jobs and the Humble, Jazzy and Lyrical devcontainers. | `osrf/docker_images` `ros/<distro>/ubuntu/<codename>/ros-base/Dockerfile` (R) |
| `osrf/ros:<distro>-desktop*` | amd64 only (R) | Not used by this project. | registry API (R) |
| `ghcr.io/ros-navigation/nav2_docker:<distro>-nightly-{devel,standard,production}` | amd64 | Rebuilt nightly for humble, jazzy, lyrical and rolling. Kilted tags exist but are no longer rebuilt (R). | `nav2_docker/main:.github/workflows/docker-release.yml#L34-L36`, `#L52` |
| `…:<distro>-nightly-standard` | amd64 | `osrf/ros:<distro>-desktop-full`, plus `/opt/ros/<distro>` and `/root/nav2_ws/install` copied from the builder. The final stage runs no rosdep, so the `/usr` libraries the builder installed are missing. One example is GraphicsMagick, needed by map_server (R). | `nav2_docker/main:docker/Dockerfile#L163-L173` |
| `…:<distro>-nightly-standard-arm64` | arm64 | `ros:<distro>-ros-base` plus the copied install tree only. It skips `nav2_bringup`, `nav2_rviz_plugins`, `nav2_system_tests` and the minimal TB sims, and lacks Nav2's own apt dependencies (angles, bondcpp, behaviortree_cpp and others) (R). | `nav2_docker/main:docker/Dockerfile.arm64#L72`, `#L91-L98` |
| every `nav2_docker` image | | The Nav2 overlay is sourced only by `/ros_entrypoint.sh`. GitHub Actions `run:` steps and JavaScript actions never run that entrypoint. | `nav2_docker/main:docker/Dockerfile#L173`, `docker/Dockerfile.arm64#L98` |

**Rules.**

- The Rolling CI job runs in `nav2_docker:rolling-nightly-standard`, in one bash step that sources `/opt/ros/rolling` and `/root/nav2_ws/install`. It does not use `action-ros-ci`, which sources only `/opt/ros/<distro>`. It runs `colcon test --ctest-args -LE launch_test`, because map_server cannot start in that image.
- The Rolling devcontainer (`rolling-nightly-standard-arm64`) runs `rosdep install --from-paths /root/nav2_ws/install/*/share …`. colcon writes `COLCON_IGNORE` into the install root, so rosdep would skip `install` itself (R). The container is for compile and unit tests only. Demos run in the Lyrical container, which has the same Nav2 API.

## 2. `nav2_core::Controller` per API generation

### 2.1 Generations

| Gen | Distros | `configure()` node type | Plan input | `computeVelocityCommands` | `cancel()`, `reset()` | Exception header | Source |
|---|---|---|---|---|---|---|---|
| A | Humble | `rclcpp_lifecycle::LifecycleNode::WeakPtr` | `setPlan(path)` | 3 arguments | absent | `nav2_core/exceptions.hpp` (`PlannerException` only) | `humble:CTRL#L75-L126`, `humble:nav2_core/include/nav2_core/exceptions.hpp#L46-L49` |
| B | Jazzy (and Kilted) | `rclcpp_lifecycle::LifecycleNode::WeakPtr` | `setPlan(path)` | 3 arguments | virtual with defaults: `cancel()` returns true, `reset()` does nothing | `nav2_core/controller_exceptions.hpp` | `jazzy:CTRL#L75-L141`, `kilted:CTRL#L75-L141` |
| C | Lyrical, Rolling | `nav2::LifecycleNode::WeakPtr` | `newPathReceived(raw_global_path)` | 5 arguments | as Gen B | `nav2_core/controller_exceptions.hpp` | `lyrical:CTRL#L74-L147`, `main:CTRL#L74-L147` |

The TF-buffer parameter has the same type on every distro: Gen A/B take `std::shared_ptr<tf2_ros::Buffer>`, and Gen C's `nav2::TransformBuffer::SharedPtr` resolves to it because of `using TransformBuffer = tf2_ros::Buffer;` (`lyrical:TF2F#L37`).

### 2.2 Exact signatures

Gen A (Humble, `humble:CTRL#L75-L126`):

```cpp
virtual void configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
  std::string name, std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) = 0;
virtual void cleanup() = 0;
virtual void activate() = 0;
virtual void deactivate() = 0;
virtual void setPlan(const nav_msgs::msg::Path & path) = 0;
virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker) = 0;
virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
```

Gen B (Jazzy, Kilted) adds two virtuals with defaults (`jazzy:CTRL#L124-L141`):

```cpp
virtual bool cancel() {return true;}  // the server keeps calling compute until this returns true
virtual void reset() {}
```

Gen C (Lyrical, Rolling, `lyrical:CTRL#L74-L147`):

```cpp
virtual void configure(
  const nav2::LifecycleNode::WeakPtr &,
  std::string name, nav2::TransformBuffer::SharedPtr,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) = 0;
virtual void newPathReceived(const nav_msgs::msg::Path & raw_global_path) = 0;  // replaces setPlan
virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker,
  const nav_msgs::msg::Path & transformed_global_plan,
  const geometry_msgs::msg::PoseStamped & global_goal) = 0;
// cleanup, activate, deactivate, cancel, setSpeedLimit and reset are unchanged from Gen B.
```

### 2.3 Detecting the generation

| Probe macro | Header probed with `__has_include` | True on | Implies | Source |
|---|---|---|---|---|
| `VF_HAS_ROS_COMMON` | `nav2_ros_common/lifecycle_node.hpp` | Lyrical, Rolling | `nav2::LifecycleNode`; `nav2_util` lifecycle, node-utils and action-server headers are gone | `lyrical:LN#L53-L57`; `nav2_util/include/nav2_util` listing on `lyrical` (no `lifecycle_node.hpp`, `node_utils.hpp` or `simple_action_server.hpp`) |
| `VF_HAS_PATH_HANDLER` | `nav2_core/path_handler.hpp` | Lyrical, Rolling | `newPathReceived` and the 5-argument compute (same upstream commit) | `lyrical:PH#L31-L98`; absent on `humble`, `jazzy`, `kilted` |
| `VF_HAS_CONTROLLER_EXCEPTIONS` | `nav2_core/controller_exceptions.hpp` | Jazzy and later | `cancel()` and `reset()` virtuals exist | `jazzy:nav2_core/include/nav2_core/controller_exceptions.hpp#L23-L75`; absent on `humble` |

- **No compile-time Nav2 version.** No Nav2 package calls `ament_generate_version_header`: none of the `nav2_core` or `nav2_ros_common` CMakeLists do. `nav2_core_VERSION` from CMake is unreliable for Rolling source builds, because `main` stayed at 1.4.0 from the Kilted branch-off until 2026-07-27, after the Gen C API had landed (R). `main:nav2_core/package.xml` declares 1.5.0 today.
- **rclcpp version.** `RCLCPP_VERSION_GTE(maj, min, pat)` comes from `rclcpp/version.h`, generated by `ament_generate_version_header` (`rclcpp/humble:rclcpp/CMakeLists.txt#L262`, `rclcpp/jazzy:rclcpp/CMakeLists.txt#L312`). It identifies the ROS distro, not the Nav2 API. Include `rclcpp/version.h` explicitly, because `rclcpp.hpp` does not pull it in on Jazzy (R). This project uses it for exactly two switches: `kStampedCmdVelDefault` (29.0.0) and the TF constructor form (30.0.0).
- **Probes depend on the link line.** From Kilted on, `nav2_core` installs its headers under `include/nav2_core/` (`kilted:nav2_core/CMakeLists.txt#L45-L55`), and Lyrical's `nav2_ros_common` does the same (`lyrical:nav2_ros_common/CMakeLists.txt#L32`, `#L51`). A `__has_include` probe is therefore true only in translation units whose target links those packages.

**Rules.**

- `vectorfield_ros` links `nav2_core`, `nav2_util`, `nav2_costmap_2d` and, when found, `nav2_ros_common` as PUBLIC dependencies and exports them. Every translation unit that includes `compat.hpp` then sees the same probe results.
- CMake sets `VF_CMAKE_HAS_ROS_COMMON` from `find_package(nav2_ros_common QUIET)`. A compile-only test asserts it equals `VF_HAS_ROS_COMMON`, and logs the detected generation in every CI job.

### 2.4 `compat.hpp` contents (`vectorfield_ros`)

| Symbol | Humble | Jazzy | Lyrical, Rolling | Reason and source |
|---|---|---|---|---|
| `VF_CANCEL_RESET_OVERRIDE` | empty | `override` | `override` | Humble has no `cancel()` or `reset()`, so `override` does not compile there (`humble:CTRL#L75-L126`). Leaving `override` off on Jazzy+ trips `-Winconsistent-missing-override` (R). The plugin always declares both methods. |
| `compat::LifecycleNode` | `nav2_util::LifecycleNode` | `nav2_util::LifecycleNode` | `nav2::LifecycleNode` | `humble:nav2_util/include/nav2_util/lifecycle_node.hpp#L37`, `lyrical:LN#L53` |
| `compat::NodeWeakPtr` = `LifecycleNode::WeakPtr` | `std::weak_ptr<rclcpp_lifecycle::LifecycleNode>` | same | `std::weak_ptr<nav2::LifecycleNode>` | `nav2_util::LifecycleNode` declares no pointer aliases (`humble:nav2_util/include/nav2_util/lifecycle_node.hpp#L37`), so it inherits rclcpp_lifecycle's. `nav2::LifecycleNode` declares its own (`lyrical:LN#L56-L57`). On every distro the alias equals the `configure()` parameter type, and a `static_assert` checks this. Never spell it `std::weak_ptr<LifecycleNode>`. |
| Consequence of the row above | `LifecycleNode::SharedPtr` points to the rclcpp_lifecycle base and does not expose `createBond()` | same | own `SharedPtr` | The follower calls `createBond()` in `on_activate` and `destroyBond()` in `on_deactivate`, on `this`. No distro does this automatically (`humble:nav2_util/include/nav2_util/lifecycle_node.hpp#L180`, `lyrical:LN#L377-L404`). |
| `compat::SimpleActionServer<A>` | `nav2_util::SimpleActionServer<A>` | `nav2_util::SimpleActionServer<A>` | `nav2::SimpleActionServer<A>` | The namespace moved (`humble:nav2_util/include/nav2_util/simple_action_server.hpp#L29`, `lyrical:nav2_ros_common/include/nav2_ros_common/simple_action_server.hpp#L39`). |
| `compat::makeActionServer(node, name, execute, completion, timeout)` | `(node, name, execute, completion, timeout, spin_thread, options)` | adds a trailing `realtime` | `(node, name, execute, goal_received, completion, timeout, spin_thread, realtime)`: `GoalReceivedCallback` is inserted before `CompletionCallback` and `options` is dropped | `humble:nav2_util/include/nav2_util/simple_action_server.hpp#L61-L69`, `jazzy:nav2_util/include/nav2_util/simple_action_server.hpp#L65-L73`, `lyrical:nav2_ros_common/include/nav2_ros_common/simple_action_server.hpp#L79-L88`. **Rule:** no `GoalReceivedCallback` is ever passed. Humble and Jazzy accept every goal while the server is active (`humble:nav2_util/include/nav2_util/simple_action_server.hpp#L144-L148`). |
| `compat::TwistPublisher` | plain `Twist` lifecycle publisher; no `twist_publisher.hpp` on Humble | `nav2_util::TwistPublisher(nav2_util::LifecycleNode::SharedPtr, topic, qos)` | `nav2_util::TwistPublisher(nav2::LifecycleNode::SharedPtr, topic, qos)` | `jazzy:nav2_util/include/nav2_util/twist_publisher.hpp#L55-L65`, `lyrical:nav2_util/include/nav2_util/twist_publisher.hpp#L53-L59` |
| `compat::kStampedCmdVelDefault` | `false` | `false` | `true` (Kilted: `true`) | Selected with `RCLCPP_VERSION_GTE(29,0,0)`. It mirrors the `enable_stamped_cmd_vel` default, which is false on Jazzy (`jazzy:nav2_util/include/nav2_util/twist_publisher.hpp#L60-L65`) and true on Kilted (`kilted:nav2_util/include/nav2_util/twist_publisher.hpp#L60-L65`) and Lyrical (`lyrical:nav2_util/include/nav2_util/twist_publisher.hpp#L59`). Humble always publishes `Twist`. |
| `compat::subscribe<M>(node, topic, qos, cb)` | base overload | base overload | upcast to `rclcpp_lifecycle::LifecycleNode` | `nav2::LifecycleNode::create_subscription(topic, callback, qos, group)` hides the base `(topic, qos, callback)` overload (`lyrical:LN#L158-L171`). |
| `makeTfBuffer`, `makeTfListener`, `makeTfBroadcaster`, `makeStaticTfBroadcaster` | pass `node` | pass `node` | pass `*node` (`RCLCPP_VERSION_GTE(30,0,0)`) | The node-pointer constructors are `[[deprecated]]` on Lyrical, which breaks `-Werror` (`geometry2/lyrical:tf2_ros/include/tf2_ros/transform_listener.hpp#L139-L183`), and removed on Rolling (`geometry2/rolling:tf2_ros/include/tf2_ros/transform_listener.hpp#L119-L147`). Nav2 uses the same switch (`lyrical:TF2F#L83-L145`). The buffer needs a `CreateTimerROS` (`lyrical:TF2F#L88-L92`). |
| `get_goal_tolerances(gc, pose_tol, vel_tol)` | `getTolerances(pose, vel)` | same | `getTolerances(pose, vel, path_length_tolerance)` | `humble:GC#L100-L102`, `lyrical:GC#L104-L107` |
| `goal_xy_reached(…)` | emulated latch | emulated latch | forwards to `isGoalXYReached(pose.pose, global_goal.pose, velocity, transformed_global_plan)` | `lyrical:GC#L119-L123`; rules in section 3.2 |
| Exceptions `NoValidControl`, `InvalidPath`, `ControllerTFError` | compat types derived from `nav2_core::PlannerException` | aliases of the `nav2_core` types | aliases | Humble's server applies patience to any `PlannerException` (`humble:CS#L489-L506`). |
| tf2 headers | `.hpp` with a `.h` fallback | same | `.hpp` only | `tf2/utils.h` is absent on Lyrical and Rolling and emits a GCC warning on Kilted (`geometry2/kilted:tf2/include/tf2/utils.h#L20`). `tf2_ros/*.h` emits `#warning` on Lyrical (`geometry2/lyrical:tf2_ros/include/tf2_ros/buffer.h#L40`). Humble 0.25.24 ships the `.hpp` headers; older Humble and Jazzy snapshots did not (R). |
| 2D message types | none used | none used | none used | `geometry_msgs/msg/Pose2D` is absent from `common_interfaces/lyrical:geometry_msgs/msg` and `common_interfaces/rolling:geometry_msgs/msg`. `nav_2d_*` stays internal to the Humble to Kilted servers. |

### 2.5 Plugin header layout

This is the only place in the plugin with distro conditionals.

```cpp
class VectorFieldController : public nav2_core::Controller
{
public:
  void configure(
    const compat::NodeWeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
#if VF_HAS_PATH_HANDLER  // Gen C
  void newPathReceived(const nav_msgs::msg::Path & raw_global_path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker, const nav_msgs::msg::Path & transformed_global_plan,
    const geometry_msgs::msg::PoseStamped & global_goal) override;
#else  // Gen A/B: LegacyPathHandler builds the same window
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
#endif
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  bool cancel() VF_CANCEL_RESET_OVERRIDE;  // also called by the Humble reset emulation
  void reset() VF_CANCEL_RESET_OVERRIDE;
  // Both compute overloads delegate to
  // computeCommandImpl(pose, velocity, goal_checker, window, goal).
};
```

## 3. GoalChecker, PathHandler, exceptions and FollowPath result codes

### 3.1 `nav2_core::GoalChecker`

| Method | Humble, Jazzy, Kilted | Lyrical, Rolling | Source |
|---|---|---|---|
| `initialize` | `(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string & plugin_name, std::shared_ptr<Costmap2DROS>)` | `(const nav2::LifecycleNode::WeakPtr &, …)` | `humble:GC#L72-L74`, `lyrical:GC#L72-L74` |
| `reset()` | yes | yes | `humble:GC#L77`, `lyrical:GC#L77` |
| `isGoalReached` | `(query_pose, goal_pose, velocity)` | adds `const nav_msgs::msg::Path & transformed_global_plan` | `humble:GC#L86-L88`, `lyrical:GC#L87-L91` |
| `getTolerances` | `(Pose &, Twist &)` | adds `double & path_length_tolerance` | `humble:GC#L100-L102`, `lyrical:GC#L104-L107` |
| `isGoalXYReached` | absent | `(query_pose, goal_pose, velocity, transformed_global_plan)`, pure virtual | `lyrical:GC#L119-L123` |

### 3.2 Goal-checker plugins and the semantics this project relies on

| Plugin | Distros | Behaviour that matters here | Source |
|---|---|---|---|
| `nav2_controller::SimpleGoalChecker` | all | Stateful XY latch (`stateful` defaults to true), re-armed by `reset()`. `getTolerances` returns `position.x` and `position.y` = `xy_goal_tolerance` and `orientation` = the quaternion of `yaw_goal_tolerance`. | `humble:SGC#L58-L59`, `#L100`, `#L142-L148` |
| `SimpleGoalChecker` on Lyrical+ | Lyrical, Rolling | `isGoalXYReached` returns false while `calculate_path_length(transformed_global_plan) > path_length_tolerance` (default 1.0). It latches with `xy_goal_tolerance_buffer` hysteresis (default 0.0). | `lyrical:SGC#L88-L100`, `#L113-L115`, `#L146-L174` |
| `nav2_controller::StoppedGoalChecker` | all | Adds stopped-velocity checks (`rot_stopped_velocity`, `trans_stopped_velocity`, default 0.25) (R). | `jazzy:nav2_controller/plugins/stopped_goal_checker.cpp` (R) |
| `nav2_controller::PositionGoalChecker` | all (present in Humble's `plugins.xml`) | `getTolerances` returns an identity orientation: "Return zero orientation tolerance as we don't check it". | `humble:nav2_controller/plugins.xml`, `humble:PGC#L89-L103`, `jazzy:PGC#L89-L103`, `lyrical:PGC#L134-L150` |
| `AdaptiveToleranceGoalChecker`, `AxisGoalChecker` | Lyrical, Rolling | `AxisGoalChecker` derives the end-of-path direction from `transformed_global_plan` and gates on its length (R). | `lyrical:nav2_controller/plugins.xml`, `lyrical:nav2_controller/plugins/axis_goal_checker.cpp` (R) |
| Initialization time | Humble to Kilted: `on_configure` | | `humble:CS#L151`, `jazzy:CS#L178` |
| Initialization time | | Lyrical+: `on_activate`, after the controllers' `activate()` | `lyrical:CS#L232-L250` |

**Rules.**

- **XY reached never disagrees with the server.**
  - Gen C: call `goal_checker->isGoalXYReached(pose.pose, global_goal.pose, velocity, transformed_global_plan)` every cycle with the server's arguments unchanged. Never pass `W.back()`, the conditioned window or a base-frame copy.
  - Gen A/B: emulate it with a latch on ‖base − g‖ ≤ xy_tol from `getTolerances()`, re-armed in every `setPlan()`. The server resets its checker on every plan (section 4.2).
- `yaw_tol` = the yaw of `getTolerances().orientation`. Values ≤ 1e-6 mean the yaw is unchecked, which is what PositionGoalChecker reports.
- The plugin never reads goal-checker parameters. On Lyrical they are not declared yet at controller `configure()` time.

### 3.3 `nav2_core::PathHandler` (Gen C only)

```cpp
using PathIterator = std::vector<geometry_msgs::msg::PoseStamped>::iterator;
using PathSegment = std::pair<PathIterator, PathIterator>;
virtual void initialize(
  const nav2::LifecycleNode::WeakPtr & parent, const rclcpp::Logger & logger,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  nav2::TransformBuffer::SharedPtr tf) = 0;
virtual void setPlan(const nav_msgs::msg::Path & path) = 0;
virtual PathSegment findPlanSegment(const geometry_msgs::msg::PoseStamped & pose) = 0;
virtual nav_msgs::msg::Path transformLocalPlan(
  const PathIterator & closest_point, const PathIterator & pruned_plan_end) = 0;
virtual geometry_msgs::msg::PoseStamped getTransformedGoal(
  const builtin_interfaces::msg::Time & stamp) = 0;
```

Source: `lyrical:PH#L31-L98` (identical on `main`).

**`nav2_controller::FeasiblePathHandler` parameters** (namespace `<handler id>.`; default id `PathHandler`):

| Parameter | Default | Source |
|---|---|---|
| `reject_unit_path` | false | `lyrical:FPH#L61-L62` |
| `max_robot_pose_search_dist` | `max(costmap size x, size y) / 2`, evaluated at `initialize()`; a negative value means unbounded | `lyrical:FPH#L63-L64`, `#L77-L82`, `#L99-L105` |
| `prune_distance` | 2.0 | `lyrical:FPH#L65-L66` |
| `enforce_path_inversion`, `enforce_path_rotation` | false, false | `lyrical:FPH#L67-L70` |
| `inversion_xy_tolerance`, `inversion_yaw_tolerance`, `minimum_rotation_angle` | 0.2, 0.4, 0.785 | `lyrical:FPH#L71-L76` |
| server `path_handler_plugins` | `["PathHandler"]` mapped to `nav2_controller::FeasiblePathHandler` | `lyrical:nav2_controller/include/nav2_controller/parameter_handler.hpp#L99-L101`, `lyrical:PARAMS#L103`, `#L125` |

**What it does each cycle** (Lyrical):

1. It transforms the robot pose into the plan frame with the `tf` it was given. An empty plan, or a unit plan when `reject_unit_path` is set, throws `InvalidPath`. A transform failure throws `ControllerTFError` (`lyrical:FPH#L140-L160`).
2. It finds the closest pose with `min_by`, searching up to `first_after_integrated_distance(max_robot_pose_search_dist)`. It never prunes below 2 poses: if the closest pose is the last one, it steps back one. The segment end is `first_after_integrated_distance(closest, end, prune_distance)` (`lyrical:FPH#L163-L198`).
3. It transforms the segment into the **costmap global frame**, stamped with the robot-pose stamp. It stops at the first pose outside the costmap. Lyrical transforms pose by pose and ignores per-pose failures. It then erases the stored plan up to the closest pose, so the search is monotonic. An empty result throws `InvalidPath` (`lyrical:FPH#L201-L252`).
4. `getTransformedGoal(stamp)` returns the last pose of the **full** plan, transformed with `costmap_ros->getTfBuffer()` rather than `tf`. A transform failure throws `ControllerTFError` (`lyrical:FPH#L256-L270`).

**`main` differs in one place:** it does one `lookupTransform` per cycle at the robot-pose stamp and throws `ControllerTFError` on failure (`main:FPH#L205-L240`).

**Lifecycle.** Path handlers are initialized in the server's `on_activate`, after the controllers' `activate()` (`lyrical:CS#L252-L255`). The handler is chosen per goal through `FollowPath.Goal.path_handler_id` (`lyrical:nav2_msgs/action/FollowPath.action`).

**Rules.**

- The plugin never reads path-handler parameters.
- `LegacyPathHandler` (Gen A/B) ports **`main`'s** FeasiblePathHandler, with attribution. It takes a plain `LegacyPathConfig{prune_distance 2.0, max_robot_pose_search_dist 0.0 (auto = max(W, H)/2; negative = unbounded), transform_tolerance 0.1, reject_unit_path false}` and is compiled on every distro.
- The parity gtest gates on Lyrical and is informational on Rolling. It uses `costmap_ros->getTfBuffer()` as its only buffer. `getTransformedGoal` reads that buffer, so a separate buffer makes every Gen C cycle throw.

### 3.4 Exceptions

| Header | Distros | Classes | Source |
|---|---|---|---|
| `nav2_core/exceptions.hpp` | Humble | `PlannerException : std::runtime_error`, `explicit PlannerException(const std::string)` | `humble:nav2_core/include/nav2_core/exceptions.hpp#L46-L49` |
| `nav2_core/controller_exceptions.hpp` | Jazzy and later | `ControllerException : std::runtime_error`. Derived from it: `InvalidController`, `ControllerTFError`, `FailedToMakeProgress`, `PatienceExceeded`, `InvalidPath`, `NoValidControl`, `ControllerTimedOut`, each `explicit X(const std::string &)` | `jazzy:nav2_core/include/nav2_core/controller_exceptions.hpp#L23-L75` |

### 3.5 `nav2_msgs/action/FollowPath`

| Part | Humble | Jazzy, Kilted | Lyrical, Rolling | Source |
|---|---|---|---|---|
| Goal | `path`, `controller_id`, `goal_checker_id` | adds `progress_checker_id` | adds `path_handler_id` | `<branch>:nav2_msgs/action/FollowPath.action` |
| Result codes | none (`std_msgs/Empty result`) | `NONE=0`, `UNKNOWN=100`, `INVALID_CONTROLLER=101`, `TF_ERROR=102`, `INVALID_PATH=103`, `PATIENCE_EXCEEDED=104`, `FAILED_TO_MAKE_PROGRESS=105`, `NO_VALID_CONTROL=106`, `CONTROLLER_TIMED_OUT=107` | adds `GOAL_REJECTED=1`, `SEND_GOAL_FAILURE=2`, `TIMEOUT=108` | same files |
| `error_msg` | absent | field present. Jazzy's server never sets it; Kilted sets `e.what()` | `e.what()` | `kilted:CS#L514-L583`, `lyrical:CS#L593-L662`; no `error_msg` assignment in `jazzy:CS` |
| Feedback | `distance_to_goal`, `speed` | same | `nav2_msgs/TrackingFeedback tracking_feedback` | same files |

### 3.6 What the server does with an exception from `computeVelocityCommands`

| Thrown | Humble | Jazzy and later | Source |
|---|---|---|---|
| `NoValidControl` | (compat type, a `PlannerException`) patience, see below | patience, see below; afterwards the result is `NO_VALID_CONTROL` or `PATIENCE_EXCEEDED` | `humble:CS#L489-L506`, `jazzy:CS#L675-L692`, `lyrical:CS#L776-L793` |
| `InvalidPath` | patience, like any `PlannerException` | immediate abort, `INVALID_PATH` | `humble:CS#L489-L506`; `jazzy:CS#L587-L593` |
| `ControllerTFError` | patience | immediate abort, `TF_ERROR` | `humble:CS#L489-L506`; `jazzy:CS#L559-L565` |
| any other `std::exception` | immediate abort, empty result | immediate abort, `UNKNOWN` | `humble:CS#L427-L432`; `jazzy:CS#L601-L614` |

**Patience** (`failure_tolerance`, default 0.0; −1 means infinite): while the tolerance has not elapsed, the server publishes a zero twist. It does not call `reset()`. After `failure_tolerance` seconds since the last valid command it throws `PatienceExceeded`; on Humble it throws `PlannerException("Controller patience exceeded")`. The stock `nav2_params.yaml` sets 0.3 on every distro (`humble:PARAMS#L119`, `jazzy:PARAMS#L75`, `lyrical:PARAMS#L99`).

**Rules.**

- Transient conditions throw `NoValidControl`: kBlocked, kNonFinite, kContourWatchdog, kHeadingInfeasible, a colliding footprint, and a parameter-rebuild failure. State is committed before throwing.
- An empty plan or window, or a cusp before M7, throws `InvalidPath`. TF failures throw `ControllerTFError`.
- While cancelling, no status throws. With `failure_tolerance` 0, a throw would turn a cancel into an abort.

## 4. `controller_server` behaviour per distro

### 4.1 Order of calls inside one control cycle

| Step | Humble | Jazzy | Kilted (reference) | Lyrical | Rolling (`main`) |
|---|---|---|---|---|---|
| Cancel requested | `terminate_all()` plus zero velocity; the plugin is not told (`humble:CS#L393-L396`) | poll `cancel()`; on true, `terminate_all()` plus `publishZeroVelocity()`, which also calls `reset()` (`jazzy:CS#L512-L516`) | poll `cancel()`; on true, `onGoalExit(true)` (`kilted:CS#L475-L479`) | as Kilted (`lyrical:CS#L554-L563`) | as Kilted (`main:CS#L555-L559`) |
| Costmap freshness | no wait | `ControllerTimedOut` if not current within `costmap_update_timeout` (0.30 s) (`jazzy:CS#L524-L531`) | as Jazzy (`kilted:CS#L491`) | `waitUntilCurrent()`, failure → `ControllerTimedOut` (`lyrical:CS#L675-L688`) | as Lyrical |
| Plan update (preempt or replan) | `updateGlobalPath()` (`humble:CS#L406`) | `jazzy:CS#L534` | `kilted:CS#L497` | `lyrical:CS#L569` | `main:CS#L570` |
| Window and goal | built by the plugin | built by the plugin | built by the plugin | `transformedPlanAndGoal()` at robot-pose stamp t1 (`lyrical:CS#L572`, `#L715-L740`) | one `getCurrentRobotPose()` reused for the whole cycle (`main:CS#L574-L577`) |
| Goal check | **after** compute (`humble:CS#L410`) | **before** compute (`jazzy:CS#L536`) | **after** compute (`kilted:CS#L501`) | before compute, against `transformed_end_pose_` from t1 (`lyrical:CS#L574`, `#L966-L979`) | before compute (`main:CS#L579`) |
| Compute | `humble:CS#L408` | `jazzy:CS#L541` | `kilted:CS#L499` | `lyrical:CS#L579`; `global_goal = getTransformedGoal(t3)` after a second `getRobotPose()` (`#L742-L759`) | `main:CS#L584`, same stamp as the window |

Consequences:

- On Humble the plugin is called once more on the cycle in which the goal is reached. It must return a sane command inside the tolerance and must not throw on arrival. Example: Ackermann with PositionGoalChecker, whose yaw tolerance reads as 0 and so must mean "unchecked".
- On Lyrical, `W.back()` and `global_goal` can differ by millimetres to centimetres after map→odom corrections. Only `main` and our `LegacyPathHandler` transform both at one stamp.

**Rule.** The goal is "in the window" when ‖W.back() − global_goal‖ < 0.05 m, latched until the next plan. The terminal point is `W.back()`.

### 4.2 Plan delivery and goal-checker re-arm

| Distro | `setPlannerPath()` | Source |
|---|---|---|
| Humble | an empty path throws `PlannerException`; `setPlan(path)`; `end_pose_ = path.poses.back()`; `goal_checker->reset()` | `humble:CS#L445-L457` |
| Jazzy | an empty path throws `InvalidPath`; `setPlan(path)`; `goal_checker->reset()` | `jazzy:CS#L625-L637` |
| Kilted | as Jazzy | `kilted:CS#L596-L608` |
| Lyrical, Rolling | an empty path throws `InvalidPath`; `newPathReceived(path)`; `path_handlers_[id]->setPlan(path)`; `goal_checker->reset()` | `lyrical:CS#L692-L705` |

`setPlannerPath()` runs at goal start and from `updateGlobalPath()` on every preemption, i.e. on every BT replan (about 1 Hz with the stock trees) (`humble:CS#L538`, `jazzy:CS#L724`, `lyrical:CS#L862`). `reset()` is **not** called on replans.

**Rules.**

- `setPlan` and `newPathReceived` are cheap and keep tracking state.
- Goal identity is decided only there, on the raw plan's last pose in the plan frame (1 cm / 0.01 rad).
- Gen A/B re-arms the emulated XY latch in every `setPlan()`.

### 4.3 `cancel()` and `reset()`

| | Humble | Jazzy | Kilted, Lyrical, Rolling | Source |
|---|---|---|---|---|
| `cancel()` contract | not called | called every cycle while a cancel is pending; the server keeps calling compute until it returns true | same | `jazzy:CS#L512-L520`, `lyrical:CS#L554-L563` |
| `reset()` called | never | inside `publishZeroVelocity()`, i.e. on every goal exit (success, cancel, error) | in `onGoalExit(force_stop)` on every exit; zero velocity only if `publish_zero_velocity` or `force_stop` | `jazzy:CS#L779-L797`, `kilted:CS#L778-L787`, `lyrical:CS#L954-L963` |

**Rules.**

- `cancel()` sets `cancelling_`, which compute forwards as `TrackerInput::cancel`. It returns true once the last command has \|v\|, \|vy\| < 0.01 m/s and \|ω\| < 0.01 rad/s.
- `reset()` clears the flag and `TrackerState`.
- Humble reset emulation: a full reset happens in `setPlan()` only when more than 3 control periods have passed since the last compute.

### 4.4 Speed limit

| Item | Value | Source |
|---|---|---|
| Topic parameter | `speed_limit_topic`, default `speed_limit`, message `nav2_msgs/SpeedLimit` | `humble:CS#L60`, `lyrical:nav2_controller/src/parameter_handler.cpp#L42-L43` |
| Delivery | `setSpeedLimit(msg->speed_limit, msg->percentage)` on **every** loaded controller, from the subscription callback (not the control thread) | `humble:CS#L621-L625`, `lyrical:CS#L991-L995` |
| Sentinel | `nav2_costmap_2d::NO_SPEED_LIMIT = 0.0` restores the default; percentages are 0 to 100 | `humble:nav2_costmap_2d/include/nav2_costmap_2d/costmap_filters/filter_values.hpp#L59`, `lyrical:…/filter_values.hpp#L60` |

**Rule.** `setSpeedLimit` takes the params mutex and scales v_r, and ω_max proportionally, through `speed_limit_scale`.

### 4.5 Output command

| | Humble | Jazzy | Kilted | Lyrical, Rolling | Source |
|---|---|---|---|---|---|
| Publisher | `create_publisher<Twist>("cmd_vel", 1)`; the returned header is discarded | `nav2_util::TwistPublisher(node, "cmd_vel", 1)` | same | `nav2_util::TwistPublisher(node, "cmd_vel")` | `humble:CS#L198`, `#L567-L572`; `jazzy:CS#L227`; `lyrical:CS#L196` |
| Message type | `Twist` | `Twist` unless `enable_stamped_cmd_vel` (default false) | `TwistStamped` (default true) | `TwistStamped` (default true) | section 2.4 |
| Header | overwritten: `frame_id` = base frame, stamp = now | same | same | same | `humble:CS#L498`, `jazzy:CS#L671`, `kilted:CS#L640`, `lyrical:CS#L772` |
| NaN or Inf command | published | published | dropped with an error | dropped with an error | `kilted:CS#L752-L755`, `lyrical:CS#L928-L934` |
| Published when | the publisher is activated and has subscribers | same | same | same | `humble:CS#L567-L572`, `lyrical:CS#L928-L937` |

Stock `navigation_launch.py` remaps the controller's `cmd_vel` to `cmd_vel_nav` (`humble:NAVL#L122`, `jazzy:NAVL#L137`, `lyrical:NAVL#L173`). Section 8.3 has the rest of the chain.

### 4.6 Robot pose and measured velocity

| Input | Humble, Jazzy, Kilted | Lyrical | Rolling (`main`) | Source |
|---|---|---|---|---|
| `pose` | `costmap_ros_->getRobotPose()`: costmap global frame (`odom` for the local costmap), latest TF | same, but obtained separately for the window, the goal check and compute | `nav2_util::getFreshPose(…, transform_staleness_threshold)`, once per cycle | `humble:CS#L611-L618`, `lyrical:CS#L981-L988`, `main:CS#L973-L976` |
| `velocity` | `nav_2d_utils::OdomSubscriber` on `odom`, then `getThresholdedTwist` | `nav2_util::OdomSmoother(odom_topic, odom_duration 0.3)`, `getRawTwist()`, then `getThresholdedTwist` | as Lyrical | `humble:CS#L197`, `jazzy:CS#L226`, `lyrical:CS#L194`, `lyrical:nav2_controller/src/parameter_handler.cpp#L50-L51` |
| Thresholds | `min_x/y/theta_velocity_threshold`, default 0.0001; each axis is zeroed below its threshold | same | same | `humble:CS#L56-L58`, `lyrical:CSH#L244-L254` |

The stock `nav2_params.yaml` sets `min_y_velocity_threshold: 0.5` on every distro (`humble:PARAMS#L117`, `jazzy:PARAMS#L73`, `lyrical:PARAMS#L97`), so the measured vy below 0.5 m/s is reported as 0.

**Rule.** The omni overlay sets `controller_server.min_y_velocity_threshold: 0.001`.

### 4.7 Frames and stamps of the plan the plugin sees

| Item | Gen A/B (`LegacyPathHandler`) | Lyrical | Rolling (`main`) | Source |
|---|---|---|---|---|
| Raw plan in `setPlan` / `newPathReceived` | planner frame (`map` with stock params) | same | same | `jazzy:nav2_navfn_planner/src/navfn_planner.cpp#L175-L176`; global costmap `global_frame: map` (`jazzy:PARAMS#L235`) |
| Window | costmap global frame, robot-pose stamp, ≥ 2 poses, clipped at the costmap and at `prune_distance` | `transformed_global_plan`: costmap global frame, stamp t1 | same, one stamp | `lyrical:FPH#L201-L252`, `main:FPH#L205-L240` |
| Goal | last raw pose, same transform as the window | `global_goal`: last raw pose, stamp t3 (a second robot-pose read) | same stamp as the window | `lyrical:CS#L742-L759`, `main:CS#L754` |
| Server's own goal check | Humble and Jazzy transform `end_pose_` inside `isGoalReached`. Humble ignores a failure; Jazzy throws `ControllerTFError`. | `transformed_end_pose_` from t1, bit-identical to `W.back()` when the window reaches the goal | same | `humble:CS#L589-L609`, `jazzy:CS#L801-L822`, `lyrical:CS#L966-L979` |

### 4.8 Server parameters per distro

Unknown keys are ignored, so one YAML can carry both progress-checker keys (R).

| Parameter | Humble | Jazzy | Kilted | Lyrical | main | Source |
|---|---|---|---|---|---|---|
| `controller_frequency` | 20.0 | 20.0 | 20.0 | 20.0 | 20.0 | `humble:CS#L51`, `jazzy:CS#L52`, `lyrical:nav2_controller/src/parameter_handler.cpp#L35` |
| progress checker key | `progress_checker_plugin` (string) | `progress_checker_plugins` (list) | list | list | list | `humble:CS#L53`, `jazzy:CS#L56`, `lyrical:nav2_controller/src/parameter_handler.cpp#L60` |
| `failure_tolerance` | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | `humble:CS#L62`, `jazzy:CS#L65`, `lyrical:nav2_controller/src/parameter_handler.cpp#L44` |
| `publish_zero_velocity` | true | true | true | true | true | `humble:CS#L63`, `jazzy:CS#L67` |
| `costmap_update_timeout` | none | 0.30 | 0.30 | 0.30 | 0.30 | `jazzy:CS#L68`, `lyrical:nav2_controller/src/parameter_handler.cpp#L47-L49` |
| `use_realtime_priority` | none | false | false | false | false | `jazzy:CS#L66` |
| `odom_topic`, `odom_duration` | `OdomSubscriber` default (R) | same (R) | same (R) | `odom`, 0.3 | same | `lyrical:nav2_controller/src/parameter_handler.cpp#L50-L51` |
| `path_handler_plugins` | none | none | none | `["PathHandler"]` | same | `lyrical:nav2_controller/src/parameter_handler.cpp#L93-L100` |
| `search_window` (tracking feedback) | none | none | none | 2.0 | 2.0 | `lyrical:nav2_controller/src/parameter_handler.cpp#L52` |
| `transform_staleness_threshold` | none | none | none | none | yes | `main:nav2_controller/src/parameter_handler.cpp` (key list compared with `lyrical`) |

### 4.9 Lifecycle facts the plugin depends on

| Fact | Source |
|---|---|
| The controller's TF buffer is `costmap_ros_->getTfBuffer()`. | `humble:CS#L177-L179`, `lyrical:CS#L173-L175` |
| The local costmap is configured, and so sized, before the controllers' `configure()`, so W, H and the resolution can be read in `configure()`. | `humble:CS#L125` before `#L177`; `lyrical:CS#L70` before `#L173` |
| The server's `on_activate` activates only its own publishers. Plugin lifecycle publishers must call `on_activate()` in `activate()` and `on_deactivate()` in `deactivate()`, and be reset in `cleanup()`. | `lyrical:CS#L222-L260` |
| On Lyrical+, `nav2::LifecycleNode::create_publisher` auto-activates only when the node is already ACTIVE, which it is not during `configure()`. Its default QoS is `nav2::qos::StandardTopicQoS` (reliable, volatile, depth 10). | `lyrical:LN#L183-L198`, `lyrical:nav2_ros_common/include/nav2_ros_common/qos_profiles.hpp#L30-L42` |
| `create_publisher<T>(topic, rclcpp::QoS(n))` compiles with both node types; `nav2::Publisher<T>` is `rclcpp_lifecycle::LifecyclePublisher<T>`. | `lyrical:nav2_ros_common/include/nav2_ros_common/publisher.hpp#L29`; (R) for the call form |

### 4.10 Topics for visualization (FieldVisualizer, RViz)

| Fact | Consequence | Source |
|---|---|---|
| Lyrical+ servers publish `transformed_global_plan` and `tracking_feedback`, only when subscribed. Humble and Jazzy publish neither; plugins publish their own (RPP: `received_global_plan`, `lookahead_point`, `lookahead_collision_arc`) (R). | Our `<ns>/window` is the conditioned window that the field actually uses, which differs from the server's topic, so it is published on every distro. The RViz config shows `<ns>/window`, not the server topic. | `lyrical:CS#L197-L198`, `#L736-L738`, `#L851-L853` |
| Window and pose are in the costmap global frame. | Markers use the costmap global frame and the cycle stamp. RViz resolves them to `map` through TF. | sections 4.6 and 4.7 |
| The server and Nav2 plugins skip publishing when there are no subscribers. | Gate on `visualize` **and** `get_subscription_count()`. | `lyrical:CS#L736-L738`, `#L931-L936` |
| `visualization_msgs/Marker` provides `ARROW`, `LINE_STRIP`, `CUBE_LIST`, `TEXT_VIEW_FACING` and `DELETEALL` on every supported distro. | The `MarkerBuilder` namespaces of the visualization design work unchanged on Humble. | `common_interfaces/humble:visualization_msgs/msg/Marker.msg#L7-L23` |
| ROS names cannot contain `.`, but Nav2 plugin names may (`FollowPath.primary_controller` on Lyrical). | Topic namespace `<ns>` = plugin name with `.` replaced by `/`. | `lyrical:SHIM#L74` |

## 5. Parameters and dynamic parameters

### 5.1 Declaration helpers

| API | Humble | Jazzy, Kilted | Lyrical, Rolling | Source |
|---|---|---|---|---|
| `nav2_util::declare_parameter_if_not_declared(node, name, value or type)` | yes | yes | header removed | `humble:nav2_util/include/nav2_util/node_utils.hpp#L92-L113`, `jazzy:…/node_utils.hpp#L95-L115` |
| `nav2_util::declare_or_get_parameter`, honouring `warn_on_missing_params` and `strict_param_loading` | no | yes | moved | `jazzy:nav2_util/include/nav2_util/node_utils.hpp#L139-L237` |
| `nav2::declare_parameter_if_not_declared`, `nav2::declare_or_get_parameter` | no | no | yes, in `nav2_ros_common/node_utils.hpp` | `lyrical:nav2_ros_common/include/nav2_ros_common/node_utils.hpp#L187-L323` |
| member `node->declare_or_get_parameter(name, default)` | no | no | yes | `lyrical:LN#L122-L145` |

**Rule.** This project declares its own parameters only through gpl. `compat.hpp` has no parameter helpers.

### 5.2 rclcpp parameter callbacks

| Fact | Humble (rclcpp 16) | Jazzy and later | Source |
|---|---|---|---|
| `add_on_set_parameters_callback` | yes | yes | `rclcpp/humble:rclcpp/include/rclcpp/node.hpp#L934`, `rclcpp/jazzy:rclcpp/include/rclcpp/node.hpp#L1159` |
| `add_pre_set_parameters_callback`, `add_post_set_parameters_callback` | no | yes (since rclcpp 17 (R)) | `rclcpp/jazzy:rclcpp/include/rclcpp/node.hpp#L1085`, `#L1220` |
| Order of on-set callbacks | the newest is registered with `emplace_front` and runs first; the first failing result stops the chain (R) | same | `rclcpp/humble:rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp#L994-L995` |

Nav2's own idiom on Lyrical is a validate-only on-set callback plus an apply-only post-set callback, both registered in `activate()`. Example: `lyrical:FPH#L87-L95`.

### 5.3 generate_parameter_library: 0.7.7 (Humble to Kilted) and 1.3.1 (Lyrical, Rolling)

| Aspect | 0.7.7 | 1.3.1 | Source |
|---|---|---|---|
| C++ namespace of `Params` and `ParamListener` | the YAML's single root key | same | `gpl/0.7.7:generate_parameter_library_py/generate_parameter_library_py/jinja_templates/cpp/parameter_library_header#L37`, `gpl/1.3.1:…#L32` |
| Constructor | `template<NodeT> ParamListener(NodeT node, std::string const & prefix = "")`; a missing trailing `.` is appended | same | `gpl/0.7.7:…/parameter_library_header#L99-L113`, `gpl/1.3.1:…#L91-L105` |
| Callback registration | one `add_on_set_parameters_callback`, which validates **and commits** | same | `gpl/0.7.7:…#L119`, `gpl/1.3.1:…#L111` |
| Read API | `get_params()`, `try_get_params()`, `try_update_params()`, `is_old()`; private mutex | same | `gpl/0.7.7:…#L123-L165`, `gpl/1.3.1:…#L115-L157` |
| `setUserCallback` | runs after the commit and cannot reject | same | `gpl/0.7.7:…#L211-L213`, `#L243`; `gpl/1.3.1:…#L203-L205`, `#L235` |
| Declaration of an existing name | skipped; the existing value is adopted | same | `gpl/0.7.7:…/jinja_templates/cpp/declare_parameter#L1`, `gpl/1.3.1:…#L1` |
| `parameter_traits` | included | removed in 1.0.0 | `gpl/0.7.7:…/parameter_library_header#L26`; `gpl/1.3.1:generate_parameter_library/CHANGELOG.rst#L24-L28` |
| Expected type | links `tl::expected` | links `tl::expected` from libexpected-dev; the vendored `tl_expected` was removed in 1.1.0 | `gpl/0.7.7:generate_parameter_library/cmake/generate_parameter_library.cmake#L92-L103`, `gpl/1.3.1:…#L77-L83`, `gpl/1.3.1:generate_parameter_library/CHANGELOG.rst#L14-L16` |
| CMake | `generate_parameter_library(LIB YAML)` creates an INTERFACE target; header at `<build>/include/<project>/<LIB>.hpp` | same | `gpl/0.7.7:…/generate_parameter_library.cmake#L30-L105`, `gpl/1.3.1:…#L30-L85` |
| Bare `#include "<LIB>.hpp"` | works with a deprecation `#pragma message` | removed in 1.0.0 | `gpl/0.7.7:…/generate_parameter_library.cmake#L71-L81`; `gpl/1.3.1:generate_parameter_library/CHANGELOG.rst#L28` |
| Validators | `rsl/parameter_validators.hpp` | same | `gpl/0.7.7:…/parameter_library_header#L31`, `gpl/1.3.1:…#L26` |
| Markdown generator | one YAML at a time, no prefix option; its "Default Config" block uses the root key as node name (R) | same | `gpl/1.3.1:generate_parameter_library_py/generate_parameter_library_py/generate_markdown.py` (R) |

**Gotchas and rules.**

- Always `#include "<package>/<LIB>.hpp"`, and use only built-in validators (`gt<>`, `bounds<>`, `one_of<>` and the like). Both versions then behave the same.
- Root keys are unique repo-wide, because a shared key gives a silent ODR clash between inline `ParamListener` definitions: `vectorfield_common_params` (vectorfield_ros, exported), `vectorfield_legacy_path_params` and `vectorfield_follower_params`.
- gpl cannot declare conditionally, so the legacy path-handler keys live in their own YAML. That listener is constructed only under `#if !VF_HAS_PATH_HANDLER`.
- Under the Humble/Jazzy rotation shim, a key the shim declared first (`rotate_to_heading_angular_vel` 1.8) is adopted silently (section 10).
- Cross-parameter rules cannot live in gpl. One extra `add_on_set_parameters_callback` is registered **after** the listeners, so it runs first and can reject before gpl commits.
- A `SetParametersAtomically` batch that spans two listeners can be committed by one and rejected by the other (R).
- Give every parameter a default: a required parameter without one throws inside `configure()` (R).
- The listener's callback outlives `deactivate()`, so reset listeners and the validator handle in `cleanup()` (R).
- The parameter reference is generated by `docs/gen_params.py`, which applies the plugin prefix. gpl's "Default Config" block is not used.
- Install gpl's Python package from git with `#subdirectory=generate_parameter_library_py` (R).

## 6. Costmap and TF utilities

### 6.1 APIs that are stable on every supported distro

| API | Notes | Source |
|---|---|---|
| `Costmap2DROS::getRobotPose`, `getGlobalFrameID`, `getBaseFrameID`, `getTransformTolerance`, `getCostmap`, `getLayeredCostmap`, `getRobotFootprint`, `getUseRadius`, `getRobotRadius`, `getTfBuffer`, `isCurrent` | `getRobotFootprint()` is the padded footprint in the robot frame | `humble:nav2_costmap_2d/src/costmap_2d_ros.cpp#L428-L431`; rest (R) |
| `Costmap2D::getMutex()` (`std::recursive_mutex`), `worldToMap`, `mapToWorld`, `getCost`, `getCharMap`, `getSizeInCells{X,Y}`, `getSizeInMeters{X,Y}`, `getOrigin{X,Y}`, `getResolution` | `worldToMapContinuous` is not available on Humble | (R) |
| `LayeredCostmap::getInscribedRadius`, `getCircumscribedRadius`, `isTrackingUnknown`, `isRolling` | | (R) |
| cost values `FREE_SPACE=0`, `MAX_NON_OBSTACLE=252`, `INSCRIBED_INFLATED_OBSTACLE=253`, `LETHAL_OBSTACLE=254`, `NO_INFORMATION=255` | | (R) |
| `nav2_util::geometry_utils`: `euclidean_distance`, `first_after_integrated_distance`, `calculate_path_length`, `orientationAroundZAxis` | | (R) |
| `std::make_shared<Costmap2DROS>("name")` | works for tests everywhere; the other constructor overloads differ per distro | (R) |

### 6.2 Signature and behaviour splits

| API | Humble | Jazzy (Kilted) | Lyrical, Rolling | Source |
|---|---|---|---|---|
| `calculateMinAndMaxDistances(footprint, …)` | `void (const std::vector<Point> &, double & min, double & max)` | `std::pair<double, double> (const std::vector<Point> &)` | same as Jazzy | `humble:nav2_costmap_2d/include/nav2_costmap_2d/footprint.hpp#L61-L63`, `jazzy:…/footprint.hpp#L62-L63`, `lyrical:…/footprint.hpp#L61-L62` |
| `InflationLayer::getInflationLayer`, `getCostScalingFactor`, `getInflationRadius` | absent | present | through `InflationLayerInterface` (overrides) | `jazzy:nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp#L166-L201`, `lyrical:…/inflation_layer.hpp#L146-L151` |
| `nav2_costmap_2d::DistanceTransform` | absent | absent | present; distance only, no nearest-point output (R) | file present only on `lyrical` and `main` |
| `CostmapSubscriber` constructor | `(nav2_util::LifecycleNode::WeakPtr or rclcpp::Node::WeakPtr, topic)` | same; adds `isCostmapReceived()` | template `NodeT`; also subscribes to `<topic>_updates`; exposes the frame id | `humble:nav2_costmap_2d/include/nav2_costmap_2d/costmap_subscriber.hpp#L38-L50`, `jazzy:…#L42-L72`, `lyrical:…#L44-L94` |
| `FootprintSubscriber` constructor | `(WeakPtr, topic, tf2_ros::Buffer &, …)` | same | template `NodeT`, `nav2::TransformBuffer &` | `humble:nav2_costmap_2d/include/nav2_costmap_2d/footprint_subscriber.hpp#L39-L52`, `lyrical:…#L41-L45` |
| `nav2_util::geometry_utils::min_by` tie-break | `<`: first minimum wins | `<=`: last minimum wins | `<=` | `humble:nav2_util/include/nav2_util/geometry_utils.hpp#L135`, `jazzy:…#L135`, `lyrical:…#L120` |
| `nav2_util` path and controller utils (`path_utils.hpp`, `controller_utils.hpp`, `parameter_handler.hpp`) | absent | absent | present | header listing of `nav2_util/include/nav2_util` per branch |

**Rule.** The core computes the footprint radii from the robot-frame polygon, so `calculateMinAndMaxDistances` is never called. The Lyrical-only utilities are not used.

### 6.3 Footprint and collision checking

| Fact | Source |
|---|---|
| `local_costmap/published_footprint` is the padded footprint transformed to the robot pose, in the costmap global frame. Consumers must transform it back into the base frame at the message stamp, as `FootprintSubscriber::getFootprintInRobotFrame` does. | `lyrical:nav2_costmap_2d/src/costmap_2d_ros.cpp#L637-L640` |
| `FootprintCollisionChecker::footprintCost` returns `LETHAL_OBSTACLE` as soon as any vertex is off the map, rasterizes only the outline (`lineCost`), and takes a max, so `NO_INFORMATION` (255) can mask `LETHAL` (254). | `humble:nav2_costmap_2d/src/footprint_collision_checker.cpp#L48-L84`, `lyrical:…#L48-L84` |
| `InflationLayer::computeCost(d)`: `LETHAL` at d = 0, `INSCRIBED` within the inscribed radius, else `252·exp(−k·(d·res − r_ins))`. | `jazzy:nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp#L150-L160`, `lyrical:…#L122-L132` |
| Stock local inflation: `inflation_radius` 0.55 (Humble) or 0.70 (Jazzy, Lyrical); `cost_scaling_factor` 3.0. | `humble:PARAMS#L199-L200`, `jazzy:PARAMS#L203-L204`, `lyrical:PARAMS#L261-L262` |

**Rule.** The core uses its own lethal-cell mask plus EDT for collision and contouring, never `footprintCost` or inflation costs.

### 6.4 `costmap_raw` publishing (the follower's obstacle input)

| Fact | Humble | Jazzy and later | Source |
|---|---|---|---|
| Where publishing happens | inside `mapUpdateLoop`, which runs at `update_frequency`, so the effective publish rate is ≤ `update_frequency` | same | `humble:nav2_costmap_2d/src/costmap_2d_ros.cpp#L467-L493`, `lyrical:nav2_costmap_2d/src/costmap_2d_ros.cpp#L555`, `#L571-L586` |
| Full `costmap_raw` | on every publish cycle when subscribed | only when `always_send_full_costmap`, or when size, resolution or origin changes; otherwise `costmap_raw_updates` (`nav2_msgs/CostmapUpdate`) when the update bounds are non-empty, and nothing at all when they are empty | `humble:nav2_costmap_2d/src/costmap_2d_publisher.cpp#L184-L188`, `jazzy:nav2_costmap_2d/src/costmap_2d_publisher.cpp#L241-L266`, `lyrical:…#L241-L266` |
| Header stamp | `clock->now()` at publish time, not the sensor time | same | `humble:nav2_costmap_2d/src/costmap_2d_publisher.cpp#L161`, `jazzy:…#L170`, `lyrical:…#L163` |
| Stock local costmap | `update_frequency` 5.0, `publish_frequency` 2.0, 3 × 3 m at 0.05 m, `global_frame` odom, `robot_base_frame` base_link, `robot_radius` 0.22, `always_send_full_costmap` true | same | `humble:PARAMS#L186-L224`, `jazzy:PARAMS#L191-L228`, `lyrical:PARAMS#L242-L292` |

**Rules.**

- Costmap staleness is detected only as a stalled costmap node, using receive time. A dead sensor is collision_monitor's job.
- The follower overlay raises **both** `update_frequency` and `publish_frequency` to the follower rate (section 11, open item 1).

### 6.5 TF

| Fact | Source |
|---|---|
| `nav2::TransformBuffer` is `tf2_ros::Buffer`; `nav2::TransformListener`, `TransformBroadcaster` and `StaticTransformBroadcaster` are thin wrappers. | `lyrical:TF2F#L37-L63` |
| Node-pointer constructors of the TF classes are deprecated on Lyrical and removed on Rolling. The replacement takes `NodeInterfaces`, which Kilted lacks, so no single call form works everywhere. | `geometry2/lyrical:tf2_ros/include/tf2_ros/transform_listener.hpp#L139-L183`, `geometry2/rolling:…#L119-L147`; Kilted lacks the NodeInterfaces constructor (R) |
| `nav2_util::transformPoseInTargetFrame(in, out, tf2_ros::Buffer &, frame, timeout)` keeps the input stamp and returns false on failure. | `lyrical:FPH#L153-L157`, `#L266-L269`; (R) for the stamp behaviour |

## 7. Build system

### 7.1 `ament_target_dependencies`

| Humble, Jazzy | Kilted | Lyrical, Rolling | Source |
|---|---|---|---|
| available | emits `message(DEPRECATION …)` | removed: `ament_cmake_target_dependencies/cmake/ament_target_dependencies.cmake` no longer exists | `ament_cmake/kilted:ament_cmake_target_dependencies/cmake/ament_target_dependencies.cmake#L87`; file absent on `ament_cmake/lyrical` and `ament_cmake/rolling` |

**Rule.** Never call it. `vf_link_ament_deps(<target> <scope> <pkgs…>)` (`vectorfield_ros/cmake/vf_link_ament_deps.cmake`, exported through `ament_package(CONFIG_EXTRAS cmake/vectorfield_ros-extras.cmake)`) links `<pkg>_TARGETS` when it is non-empty, and otherwise `<pkg>_INCLUDE_DIRS` and `<pkg>_LIBRARIES`.

### 7.2 Exported CMake targets per package

| Package | Humble | Jazzy | Kilted | Lyrical, Rolling | Source |
|---|---|---|---|---|---|
| `nav2_core` | no target; headers in the merged `include/` | same | INTERFACE `nav2_core::nav2_core`; headers in `include/nav2_core/` | same | `humble:nav2_core/CMakeLists.txt#L31-L42`, `jazzy:nav2_core/CMakeLists.txt#L33-L44`, `kilted:nav2_core/CMakeLists.txt#L38-L67`, `lyrical:nav2_core/CMakeLists.txt#L39-L73` |
| `nav2_util` | no target: `nav2_util_LIBRARIES` and `nav2_util_INCLUDE_DIRS`; headers in `include/` | `nav2_util::nav2_util_core`; headers in `include/nav2_util/` | same | same | `humble:nav2_util/CMakeLists.txt#L42-L60`, `jazzy:nav2_util/CMakeLists.txt#L24-L60`, `lyrical:nav2_util/CMakeLists.txt#L23-L56` |
| `nav2_costmap_2d` | `nav2_costmap_2d::nav2_costmap_2d_core`, `::layers`, `::filters`, `::nav2_costmap_2d_client` | same | same | same | `humble:nav2_costmap_2d/CMakeLists.txt#L155-L191`, `lyrical:nav2_costmap_2d/CMakeLists.txt#L195-L226` |
| `nav2_ros_common` | absent | absent | absent | INTERFACE `nav2_ros_common::nav2_ros_common` | `lyrical:nav2_ros_common/CMakeLists.txt#L32-L92` |
| `nav2_controller` (parity test) | libraries only | libraries only | targets | `nav2_controller::feasible_path_handler` in the export set | `humble:nav2_controller/CMakeLists.txt#L110-L111`, `lyrical:nav2_controller/CMakeLists.txt#L250-L288` |
| message packages | `${<pkg>_TARGETS}` | same | same | same; an aggregate `<pkg>::<pkg>` target exists only from rosidl 5.1.3 (R) | (R) |
| gpl-generated libraries | an INTERFACE target per YAML; the user installs and exports it | same | same | same | `gpl/0.7.7:generate_parameter_library/cmake/generate_parameter_library.cmake#L86-L105` |

### 7.3 Conditional dependencies (REP 149)

Each side of `and` must be a full comparison:

```xml
<depend condition="$ROS_DISTRO != humble and $ROS_DISTRO != jazzy and $ROS_DISTRO != kilted">nav2_ros_common</depend>
<exec_depend condition="$ROS_DISTRO != humble">nav2_loopback_sim</exec_depend>
```

- Naming the old distros means future distros pick up `nav2_ros_common` automatically.
- `nav2_loopback_sim`, `nav2_minimal_tb3_sim` and `nav2_minimal_tb4_sim` do not exist in Humble's rosdistro (`rosdistro:humble`). They also need conditions in `vectorfield_bringup` and in the test depends of `vectorfield_system_tests`.
- `ros_gz_sim` and `ros_gz_bridge` exist on Humble (R). The project still conditions them, because the Humble demo does not use them.
- Jazzy's `nav2_bringup` does not exec-depend on `nav2_loopback_sim`; Lyrical's does. `vectorfield_bringup` therefore declares it itself (`jazzy:nav2_bringup/package.xml#L21-L28`, `lyrical:nav2_bringup/package.xml#L24-L32`).
- CI jobs set `rosdep-check: true`, so an unresolvable key fails the pull request instead of the bloom release (R).

### 7.4 Detecting `nav2_ros_common` in CMake

```cmake
find_package(nav2_ros_common QUIET)
if(nav2_ros_common_FOUND)
  vf_link_ament_deps(${target} PUBLIC nav2_ros_common)
  target_compile_definitions(${target} PUBLIC VF_CMAKE_HAS_ROS_COMMON=1)
endif()
```

A compile-only test `static_assert`s that `VF_CMAKE_HAS_ROS_COMMON` equals the header probe `VF_HAS_ROS_COMMON`.

### 7.5 Language standard, warnings and test helpers

| Fact | Source |
|---|---|
| `nav2_package()` forces C++17 (Humble) or C++20 (Lyrical) and adds `-Werror`. **Rule:** never call it. | `humble:nav2_common/cmake/nav2_package.cmake#L31-L36`, `lyrical:nav2_common/cmake/nav2_package.cmake#L31-L40` |
| On Lyrical every target linking rclcpp compiles as C++20. **Rule:** code is valid C++17 and C++20, e.g. no implicit `this` capture via `[=]` and no `std::result_of`. | `ament_cmake_ros/lyrical:ament_cmake_ros_core/cmake/ament_ros_defaults.cmake#L16` |
| Lyrical Nav2 packages use `cmake_minimum_required(VERSION 3.20)`. CMake 4 rejects compatibility below 3.5 (R). The project uses 3.22, Ubuntu 22.04's CMake (R). | `lyrical:nav2_core/CMakeLists.txt#L1` |
| `PLUGINLIB_EXPORT_CLASS(…, nav2_core::Controller)` and `pluginlib_export_plugin_description_file(nav2_core <xml>)` are unchanged on every branch (R). | (R) |
| `ament_add_ros_isolated_gtest`/`pytest` and `run_test_isolated.py` exist from Humble. `add_launch_test(… RUNNER …)` forwards `RUNNER` to `ament_add_test` and keeps the default `launch_test` label. `nav2_add_gtest` exists only on `main` (R). | (R) |
| `run_test_isolated.py` keeps an existing `ROS_DOMAIN_ID` and then skips isolation. **Rule:** never set `ROS_DOMAIN_ID` in CI or devcontainers. `ROS_AUTOMATIC_DISCOVERY_RANGE` does not exist on Humble; use `ROS_LOCALHOST_ONLY=1` there (R). The first rclcpp release with the variable is UNVERIFIED. | (R) |

## 8. Bringup and simulation per distro

### 8.1 Launch files

| File | Humble | Jazzy | Kilted | Lyrical, Rolling | Source |
|---|---|---|---|---|---|
| `nav2_bringup/launch/navigation_launch.py`, `bringup_launch.py`, `localization_launch.py`, `rviz_launch.py` | yes | yes | yes | yes | `nav2_bringup/launch` listing per branch |
| `tb3_simulation_launch.py` | yes (Gazebo Classic `gzserver`) | yes (gz sim with `nav2_minimal_tb3_sim`) | yes | yes | `humble:nav2_bringup/launch/tb3_simulation_launch.py#L162`, `jazzy:nav2_bringup/launch/tb3_simulation_launch.py` |
| `tb4_simulation_launch.py` | no | yes | yes | yes | listing |
| TB3 loopback wrapper | none | `tb3_loopback_simulation.launch.py` (dot) | `tb3_loopback_simulation_launch.py` (underscore) | underscore | listing |
| `nav2_loopback_sim/launch/loopback_simulation.launch.py` | none | yes | yes | yes | `jazzy:nav2_loopback_sim/launch/loopback_simulation.launch.py`, `lyrical:…` |
| `nav2_collision_monitor/launch/collision_monitor_node.launch.py` | yes, with its own autostart `lifecycle_manager` | yes | yes | yes | `humble:nav2_collision_monitor/launch/collision_monitor_node.launch.py#L35-L79` |

The TB3 loopback wrapper provides `robot_state_publisher` with the TB3 URDF, `map_server` with `lifecycle_manager_map_server`, the loopback simulator and `use_localization: False`. It takes `map`, `params_file` and `use_rviz` (default true) (`jazzy:nav2_bringup/launch/tb3_loopback_simulation.launch.py#L119-L196`, `lyrical:nav2_bringup/launch/tb3_loopback_simulation_launch.py#L38-L121`).

**Rules.**

- `loopback_demo` (Jazzy+) includes the wrapper, chosen by an existence check, with `use_rviz:=False`, and starts `rviz2 -d rviz/vectorfield.rviz` itself.
- Never assemble `loopback_simulation.launch.py` plus `navigation_launch.py` by hand: nothing would publish base_footprint→base_link→base_scan.

### 8.2 Loopback simulator facts

| Fact | Source |
|---|---|
| It publishes no TF and no odometry until it receives `initialpose`. | `lyrical:nav2_loopback_sim/src/loopback_simulator.cpp#L76`, `#L260-L283` |
| Frames: `base_frame_id` defaults to `base_footprint` and `scan_frame_id` to `base_scan`. It publishes map→odom and odom→base_footprint. | `lyrical:nav2_loopback_sim/src/loopback_simulator.cpp#L47-L50`, `#L69` |
| `publish_clock` defaults to true. Lyrical's launch forces `use_sim_time: True`, so every other node must use sim time. | `lyrical:nav2_loopback_sim/src/loopback_simulator.cpp#L62`, `lyrical:nav2_loopback_sim/launch/loopback_simulation.launch.py#L69` |
| It integrates `linear.y`, so omni runs on it. | `lyrical:nav2_loopback_sim/src/loopback_simulator.cpp#L344` |
| It is Python on Jazzy and C++ on Lyrical. | `jazzy:nav2_loopback_sim/nav2_loopback_sim/loopback_simulator.py`, `lyrical:nav2_loopback_sim/src/loopback_simulator.cpp` |
| `BasicNavigator.waitUntilNav2Active()` defaults to `localizer='amcl'` and hangs without AMCL. **Rule:** tests call `setInitialPose()`, then `waitUntilNav2Active(localizer='controller_server')`. | `nav2_simple_commander/nav2_simple_commander/robot_navigator.py` (R) |

### 8.3 Command chain per distro

| Distro | Who publishes what | Source |
|---|---|---|
| Humble | `controller_server`: `cmd_vel` remapped to `cmd_vel_nav`. `velocity_smoother`: `cmd_vel_nav` in, `cmd_vel_smoothed` remapped to **`cmd_vel`** out. `behavior_server` is **not** remapped, so behaviors publish `cmd_vel` directly. No collision_monitor in the lifecycle list, and no `collision_monitor` block in `nav2_params.yaml`. | `humble:NAVL#L43-L48`, `#L122`, `#L143-L152`, `#L183`; `humble:nav2_behaviors/include/nav2_behaviors/timed_behavior.hpp#L130` |
| Humble, collision_monitor defaults | `cmd_vel_in_topic` `cmd_vel_raw`, `cmd_vel_out_topic` `cmd_vel`, `base_frame_id` `base_footprint`. The Humble node ignores the result of a source's `getData()`. | `humble:nav2_collision_monitor/src/collision_monitor_node.cpp#L194-L202`, `#L359` |
| Jazzy | controller and behaviors publish to `cmd_vel_nav`; the smoother turns `cmd_vel_nav` into `cmd_vel_smoothed`; collision_monitor turns `cmd_vel_smoothed` into `cmd_vel`. `docking_server` publishes `cmd_vel` directly (no remap). | `jazzy:NAVL#L137`, `#L181`, `#L215`, `#L218-L238`; `jazzy:PARAMS#L376-L382`; `jazzy:nav2_docking/opennav_docking/src/docking_server.cpp#L64` |
| Lyrical, Rolling | as Jazzy, plus `following_server` publishing `cmd_vel` directly | `lyrical:NAVL#L253-L284`, `lyrical:nav2_following/opennav_following/src/following_server.cpp#L47`, `#L56` |

**Rules.**

- On Humble, bringup includes `collision_monitor_node.launch.py` with `config/humble/collision_monitor.yaml`: `cmd_vel` in, `cmd_vel_safe` out, `base_footprint`, polygons and a scan source. This captures both the smoother's and the behaviors' output. `vectorfield_sim` listens on `cmd_vel_safe`.
- The final-topic launch test allows collision_monitor plus the documented bypass list (docking_server on Jazzy+, following_server on Lyrical+). It fails on any other publisher, and on any publisher from this repository.

### 8.4 Stock `nav2_params.yaml` values that matter

| Key | Humble | Jazzy | Lyrical | Source |
|---|---|---|---|---|
| Default `FollowPath` plugin | `dwb_core::DWBLocalPlanner` | `nav2_mppi_controller::MPPIController` | MPPI | `humble:PARAMS#L142`, `jazzy:PARAMS#L98`, `lyrical:PARAMS#L134` |
| `general_goal_checker` | Simple, 0.25 m / 0.25 rad | same | same | `humble:PARAMS#L137-L139`, `jazzy:PARAMS#L94-L96`, `lyrical:PARAMS#L120-L122` |
| progress checker | Simple, 0.5 m within 10 s | same | same | `humble:PARAMS#L126-L128`, `jazzy:PARAMS#L83-L85`, `lyrical:PARAMS#L109-L111` |
| `velocity_smoother` `max_velocity` | `[0.26, 0.0, 1.0]` | `[0.5, 0.0, 2.0]` | `[0.5, 0.0, 2.0]` | `humble:PARAMS#L343`, `jazzy:PARAMS#L365`, `lyrical:PARAMS#L497` |
| `velocity_smoother` `max_accel` | `[2.5, 0.0, 3.2]` | `[2.5, 0.0, 3.2]` | `[3.0, 0.0, 3.5]` | `humble:PARAMS#L345`, `jazzy:PARAMS#L367`, `lyrical:PARAMS#L499` |
| `scale_velocities` | False | False | false | `humble:PARAMS#L341`, `jazzy:PARAMS#L363`, `lyrical:PARAMS#L495` |
| `enable_stamped_cmd_vel` | not set | not set | not set | absent from all three files |
| `collision_monitor` | absent | `cmd_vel_smoothed` → `cmd_vel`, `base_footprint`, `source_timeout` 1.0 | same | `jazzy:PARAMS#L376-L382`, `lyrical:PARAMS#L509-L515` |
| `loopback_simulator` | absent | `base_footprint`, `base_scan` | same | `jazzy:PARAMS#L460-L463`, `lyrical:PARAMS#L594-L597` |

**Velocity smoother.** It clamps `linear.y` to `[min_velocity[1], max_velocity[1]]` on every distro (`humble:nav2_velocity_smoother/src/velocity_smoother.cpp#L278`, `jazzy:…#L355-L357`, `lyrical:…#L354-L356`). With the stock y = 0, every lateral command is zeroed. From Jazzy on it subscribes through `nav2_util::TwistSubscriber`, which honours `enable_stamped_cmd_vel` (`jazzy:nav2_velocity_smoother/src/velocity_smoother.cpp#L156`).

**Rules.**

- Only the model overlays set the smoother vectors, always as full 3-element lists with x and θ ≥ the controller limits. The omni overlay sets y ≥ `max_lateral_vel`. Scenario overlays set only `scale_velocities: true`.
- Overlays never set `enable_stamped_cmd_vel`, so the sim, collision_monitor and the follower agree through `compat::kStampedCmdVelDefault`.

### 8.5 Plugin type strings

| Plugin | Humble | Jazzy and later | Source |
|---|---|---|---|
| Smac (2D, Hybrid, Lattice) | `nav2_smac_planner/SmacPlanner2D` (registered `name`; pluginlib resolves only that name) | `nav2_smac_planner::SmacPlanner2D` (type only) | `humble:nav2_smac_planner/smac_plugin_2d.xml`, `humble:nav2_smac_planner/smac_plugin_hybrid.xml#L2`, `jazzy:nav2_smac_planner/smac_plugin_hybrid.xml` |
| NavFn | `nav2_navfn_planner/NavfnPlanner` | `nav2_navfn_planner::NavfnPlanner` | `humble:nav2_navfn_planner/global_planner_plugin.xml`, `humble:PARAMS#L282`, `jazzy:PARAMS#L284` |
| Behaviors | `nav2_behaviors/Spin` and so on | `nav2_behaviors::Spin` and so on | `humble:nav2_behaviors/behavior_plugin.xml`, `humble:PARAMS#L304-L312` |
| Goal checkers, `nav2_controller::*` | type names | type names | `humble:nav2_controller/plugins.xml` |

**Rule.** `vectorfield_bringup.params.merge()` rewrites `nav2_smac_planner::X`, `nav2_navfn_planner::X` and `nav2_behaviors::X` to the `/` form on Humble. The Humble collision-monitor file is the only other distro-specific config.

### 8.6 Behaviour trees and action status

| Fact | Source |
|---|---|
| The root formats differ: Humble uses `main_tree_to_execute="MainTree"` (BT.CPP v3); Jazzy uses `BTCPP_format="4"` with `MainTree`; Lyrical uses `BTCPP_format="4"` with `NavigateToPoseWReplanningAndRecovery`. No stock tree contains `SmoothPath` (R). | `<branch>:nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml` (R) |
| `navigate_w_replanning_time.xml` (1 Hz replanning, no recovery, no Spin) exists on Humble, Jazzy and Lyrical. The stock recovery trees and the only stock NavigateThroughPoses tree use Spin (R). **Rule:** the Ackermann overlay selects `navigate_w_replanning_time.xml`. | `nav2_bt_navigator/behavior_trees` listing per branch; contents (R) |
| `follow_point.xml` (GoalUpdater, then ComputePathToPose, then `TruncatePath distance=1.0`, at 1 Hz) moves the plan's last pose on every replan (R). | `jazzy:nav2_bt_navigator/behavior_trees/follow_point.xml` (R) |
| The action status topics `<action>/_action/status` use KEEP_LAST 1, RELIABLE, TRANSIENT_LOCAL. **Rule:** the follower's interlock subscribes with this QoS. | `rcl/humble:rcl_action/include/rcl_action/default_qos.h#L24-L35` |

## 9. Planner output quirks relevant to the controller

| Quirk | Effect on the plugin | Source |
|---|---|---|
| NavFn with start XY = goal XY returns a **one-pose** plan: start XY with the goal yaw. | A one-pose plan is valid input. | `jazzy:nav2_navfn_planner/src/navfn_planner.cpp#L171-L188`, `lyrical:…#L170-L188` |
| NavFn's `smoothApproachToGoal` replaces the last pose if the goal is closer to the second-to-last pose; otherwise it appends the exact goal after the goal-cell pose. The result is a sub-cell hook of up to res/√2 in any direction. Paths step half a cell (`pathStep = 0.5`). | Cusp detection uses chord decimation that pops trailing vertices within ℓ_c of the goal. The end tangent comes from the raw plan, never from the window tail. | `jazzy:nav2_navfn_planner/src/navfn_planner.cpp#L311`, `#L350-L376`; `jazzy:nav2_navfn_planner/src/navfn.cpp#L141` |
| Smac 2D and Hybrid return one pose when start and goal share a cell. Humble's Hybrid also requires the same orientation bin. | Same as NavFn. | `jazzy:nav2_smac_planner/src/smac_planner_2d.cpp#L261-L262`, `jazzy:nav2_smac_planner/src/smac_planner_hybrid.cpp#L427-L429`, `humble:nav2_smac_planner/src/smac_planner_hybrid.cpp#L338-L341`, `lyrical:nav2_smac_planner/include/nav2_smac_planner/smac_planner_hybrid_impl.hpp#L469` |
| Smac Hybrid `motion_model_for_search` defaults to `DUBIN` (forward only). `REEDS_SHEPP` produces cusps. | The Ackermann overlay pins `DUBIN`. Cusps throw `InvalidPath` until M7. | `humble:nav2_smac_planner/src/smac_planner_hybrid.cpp#L137`, `jazzy:…#L168`, `lyrical:nav2_smac_planner/include/nav2_smac_planner/smac_planner_hybrid_impl.hpp#L131` |
| Near the goal, the path handler leaves only the last 1 or 2 raw poses in the window (≥ 2 poses, erased behind the closest pose). | Tail conditioning inserts a virtual vertex ℓ_c behind the goal along the cached end tangent. | `lyrical:FPH#L185-L191`, `#L238` |
| The window ends at the first pose outside the local costmap (3 × 3 m by default, so about 1.5 m ahead), before `prune_distance` 2.0. | `cap_at_curve_end` always brakes before the window end. | `lyrical:FPH#L226-L229`; `jazzy:PARAMS#L196-L198` |
| `enforce_path_inversion` and `enforce_path_rotation` truncate the window at a constraint while the goal stays the full plan's last pose. | Unsupported until M7, as documented. | `lyrical:FPH#L67-L76`, `#L238-L245` |
| Stock trees replan at about 1 Hz from the current robot pose. | Plan intake is cheap. A near-goal replan whose poses are all within ℓ_c keeps the previous end tangent. | (R) for the replan rate |
| User paths sent straight to FollowPath can be sparse. | Warn once when a raw segment exceeds 0.25 m. | project rule |

## 10. Rotation shim interaction

| Aspect | Humble | Jazzy (Kilted) | Lyrical, Rolling | Source |
|---|---|---|---|---|
| Primary type key | `FollowPath.primary_controller` (string) | same | `FollowPath.primary_controller.plugin` | `humble:SHIM#L68`, `jazzy:SHIM#L70`, `lyrical:nav2_rotation_shim_controller/src/parameter_handler.cpp#L54-L55` |
| Primary configured as | `configure(parent, name, …)`: our keys are `FollowPath.*`, next to the shim's | same | `configure(parent, name + ".primary_controller", …)`: our keys are `FollowPath.primary_controller.*` | `humble:SHIM#L102`, `jazzy:SHIM#L111`, `lyrical:SHIM#L74` |
| Keys the shim declares first, under our prefix | `rotate_to_heading_angular_vel` 1.8 and `max_angular_accel` 3.2. gpl adopts them, so they are shared with ours. | same | not shared (different namespace) | `humble:SHIM#L62-L64`, `jazzy:SHIM#L61-L63`; gpl `declare_parameter#L1` |
| `angular_dist_threshold`, `angular_disengage_threshold`, `forward_sampling_distance` | 0.785, 0.785, 0.5 | 0.785, 0.3925, 0.5 | 0.785, 0.3925, 0.5 | `humble:SHIM#L56-L60`, `jazzy:SHIM#L55-L59`, `lyrical:nav2_rotation_shim_controller/src/parameter_handler.cpp#L39-L44` |
| `rotate_to_heading_once` | absent | false | false | `jazzy:SHIM#L74`, `lyrical:nav2_rotation_shim_controller/src/parameter_handler.cpp#L71-L72` |
| When the shim re-checks the heading | every `setPlan()` | every `setPlan()`, or only on a changed goal with `rotate_to_heading_once: true` | every control cycle: the flag is set in `computeVelocityCommands()`. With `rotate_to_heading_once: true`, `isGoalChanged()` compares the full stamped `global_goal`, whose stamp changes every cycle, so it never latches. | `humble:SHIM#L357`, `jazzy:SHIM#L382-L395`, `lyrical:SHIM#L127`, `#L328-L336`, `lyrical:FPH#L256-L261` |
| What a re-check does | If the bearing to the plan pose `forward_sampling_distance` ahead exceeds `angular_dist_threshold`, the shim rotates in place itself and does not call the primary (R). | same | same | `jazzy:SHIM#L197-L251` (R) |

**Rules.**

- Under the shim, set `use_rotate_to_field: false`. The docs show both YAML layouts, and no shim overlay is shipped.
- `rotate_to_heading_angular_vel > max_angular_vel` is a warning, not a rejection: the effective rate is the minimum. The message names the shim's 1.8 default.
- The cross-validator ignores names under our prefix that are not ours: `<plugin>.plugin`, and the shim's keys on Humble and Jazzy.
- Shim plus `obstacle_mode: contour` is supported only on Jazzy (and Kilted) with `FollowPath.rotate_to_heading_once: true`, because a re-check aims the robot back at a path that runs through the obstacle. `configure()` detects the shim and warns once in every other combination. On Humble and Jazzy it detects `<name>.primary_controller`; on Lyrical+ it detects a plugin name ending in `.primary_controller`.

## 11. Open items

1. **Follower costmap rate.** `costmap_raw` is published from the update loop, so `publish_frequency` above `update_frequency` has no effect (section 6.4). The follower overlay must set the local costmap's `update_frequency` as well as `publish_frequency: 10.0`. The current design names only the publish rate.
2. **UNVERIFIED**: the first rclcpp release that supports `ROS_AUTOMATIC_DISCOVERY_RANGE`. Only "absent on Humble, present on Jazzy" is checked (R).
3. **UNVERIFIED**: whether gz sim renders the TB3 `gpu_lidar` (ogre2) in a display-less container without `xvfb-run`. The nightly Gazebo job wraps the tests in `xvfb-run` regardless.
4. **UNVERIFIED**: when Rolling packages on Ubuntu 26.04 sync to the main apt repository. Until then Rolling CI installs from `ros2-testing` or uses `nav2_docker`.
5. Kilted rows are reference only. Kilted is not built in CI, and its EOL is December 2026 (section 1.1).
