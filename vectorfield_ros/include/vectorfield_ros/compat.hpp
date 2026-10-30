// Copyright 2026 Héctor Azpúrua
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file
/// @brief Nav2 distro compatibility: API-generation probes, node and action-server aliases, the
/// `cmd_vel` publisher, TF factories, goal-checker helpers, controller exceptions and tf2 header
/// fallbacks (docs/design/nav2_api_matrix.md §2.3–§2.4).
///
/// **The only distro switch of the ROS layer.** Besides this header, distro conditionals appear
/// only in the controller's Gen C / Gen A–B signature blocks, behind `VF_CANCEL_RESET_OVERRIDE`,
/// and in the controller's test server harness. Code elsewhere uses the aliases and
/// constants below instead of `#if`.
///
/// **API generations** (nav2_api_matrix.md §2.1):
///
/// | Gen | Distros | VF_HAS_ROS_COMMON | VF_HAS_PATH_HANDLER | VF_HAS_CONTROLLER_EXCEPTIONS |
/// |---|---|---|---|---|
/// | A | Humble | 0 | 0 | 0 |
/// | B | Jazzy (Kilted, unsupported) | 0 | 0 | 1 |
/// | C | Lyrical, Rolling | 1 | 1 | 1 |
///
/// **Detection** uses `__has_include`, never a Nav2 version (Nav2 generates no version header,
/// and `nav2_core_VERSION` is unreliable for Rolling source builds). Probes depend on the link
/// line: from Kilted on, nav2_core and nav2_ros_common install their headers under
/// `include/<pkg>/`, so a probe is true only in translation units whose target links them.
/// vectorfield_ros links nav2_core, nav2_util, nav2_costmap_2d and, when found, nav2_ros_common
/// PUBLIC, so every target that links `vectorfield_ros::vectorfield_ros` sees the same results.
/// CMake also defines `VF_CMAKE_HAS_ROS_COMMON` (0 or 1, from `find_package(nav2_ros_common
/// QUIET)`); the API-generation test asserts that it equals VF_HAS_ROS_COMMON.
///
/// **rclcpp version switches.** `RCLCPP_VERSION_GTE` (rclcpp/version.h, included explicitly
/// because rclcpp.hpp does not pull it in on Jazzy) selects exactly two things:
/// `kStampedCmdVelDefault` (29.0.0) and the TF constructor form (30.0.0).
///
/// **Not used anywhere:** `geometry_msgs/Pose2D` (absent on Lyrical and Rolling) and `nav_2d_*`
/// (internal to the Humble–Kilted servers).
///
/// Header-only. Everything here compiles on every supported distro; the static_asserts at the end
/// pin the aliases to the `nav2_core` interfaces.

#ifndef VECTORFIELD_ROS__COMPAT_HPP_
#define VECTORFIELD_ROS__COMPAT_HPP_

/// @def VF_HAS_ROS_COMMON
/// @brief 1 when `nav2_ros_common/lifecycle_node.hpp` exists (Lyrical, Rolling), else 0.
///
/// Implies `nav2::LifecycleNode`; the nav2_util lifecycle node and node utilities are gone on
/// those distros.
#if __has_include(<nav2_ros_common/lifecycle_node.hpp>)
#define VF_HAS_ROS_COMMON 1
#else
#define VF_HAS_ROS_COMMON 0
#endif

/// @def VF_HAS_PATH_HANDLER
/// @brief 1 when `nav2_core/path_handler.hpp` exists (Gen C: Lyrical, Rolling), else 0.
///
/// Implies `Controller::newPathReceived()`, the 5-argument `computeVelocityCommands()`, the
/// 3-argument `GoalChecker::getTolerances()` and `GoalChecker::isGoalXYReached()`.
#if __has_include(<nav2_core/path_handler.hpp>)
#define VF_HAS_PATH_HANDLER 1
#else
#define VF_HAS_PATH_HANDLER 0
#endif

/// @def VF_HAS_CONTROLLER_EXCEPTIONS
/// @brief 1 when `nav2_core/controller_exceptions.hpp` exists (Jazzy and later), else 0.
///
/// Implies the `Controller::cancel()` and `Controller::reset()` virtuals, and that the
/// controller_server polls cancel() and calls reset() on every goal exit.
#if __has_include(<nav2_core/controller_exceptions.hpp>)
#define VF_HAS_CONTROLLER_EXCEPTIONS 1
#else
#define VF_HAS_CONTROLLER_EXCEPTIONS 0
#endif

/// @def VF_CANCEL_RESET_OVERRIDE
/// @brief `override` on Jazzy and later, empty on Humble.
///
/// The plugin always declares `cancel()` and `reset()`. Humble has neither virtual, so `override`
/// does not compile there; leaving it off on Jazzy and later trips
/// `-Winconsistent-missing-override`.
#if VF_HAS_CONTROLLER_EXCEPTIONS
#define VF_CANCEL_RESET_OVERRIDE override
#else
#define VF_CANCEL_RESET_OVERRIDE
#endif

/// @def VF_COMPAT_HAS_TWIST_PUBLISHER
/// @brief Internal: 1 when `nav2_util/twist_publisher.hpp` exists (Jazzy and later), else 0.
/// Use compat::TwistPublisher instead of testing it.
#if __has_include(<nav2_util/twist_publisher.hpp>)
#define VF_COMPAT_HAS_TWIST_PUBLISHER 1
#else
#define VF_COMPAT_HAS_TWIST_PUBLISHER 0
#endif

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>
#include <utility>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#if VF_HAS_ROS_COMMON
#include "nav2_ros_common/lifecycle_node.hpp"
#else
#include "nav2_util/lifecycle_node.hpp"
#endif

#if VF_HAS_CONTROLLER_EXCEPTIONS
#include "nav2_core/controller_exceptions.hpp"
#else
#include "nav2_core/exceptions.hpp"
#endif

#if VF_COMPAT_HAS_TWIST_PUBLISHER
#include "nav2_util/twist_publisher.hpp"
#endif

// tf2 headers: prefer .hpp. The .h forms are absent (tf2/utils.h) or emit #warning (tf2_ros/*.h)
// on Lyrical and later; older Humble and Jazzy snapshots shipped only .h.
#if __has_include(<tf2/utils.hpp>)
#include "tf2/utils.hpp"
#else
#include "tf2/utils.h"
#endif
#if __has_include(<tf2_ros/buffer.hpp>)
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/create_timer_ros.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"
#else
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#endif
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif

/// @brief The ROS 2 layer shared by the vectorfield_nav2 nodes.
namespace vectorfield_ros
{

/// @brief Per-distro aliases and helpers. Nothing outside this namespace tests a distro macro.
namespace compat
{

/// @name API generation
/// @{

/// @brief 'A' (Humble), 'B' (Jazzy, Kilted) or 'C' (Lyrical, Rolling).
inline constexpr char kApiGeneration = VF_HAS_PATH_HANDLER            ? 'C'
                                       : VF_HAS_CONTROLLER_EXCEPTIONS ? 'B'
                                                                      : 'A';

/// @brief Human-readable generation, for the configure() log line and the API-generation test.
inline constexpr const char * kApiGenerationName =
  VF_HAS_PATH_HANDLER ? "Gen C (Lyrical, Rolling: newPathReceived + 5-argument compute)"
  : VF_HAS_CONTROLLER_EXCEPTIONS
    ? "Gen B (Jazzy, Kilted: setPlan + 3-argument compute, cancel/reset)"
    : "Gen A (Humble: setPlan + 3-argument compute)";

/// @brief The controller_server calls `Controller::reset()` on every goal exit (Jazzy and later).
/// Humble never does; the plugin emulates it in setPlan() (Humble reset emulation).
inline constexpr bool kServerCallsReset = VF_HAS_CONTROLLER_EXCEPTIONS != 0;

/// @brief The controller_server keeps calling compute and polls `Controller::cancel()` until it
/// returns true (Jazzy and later). Humble terminates a cancelled goal at once and never tells the
/// plugin (nav2_api_matrix.md §4.3).
inline constexpr bool kServerPollsCancel = VF_HAS_CONTROLLER_EXCEPTIONS != 0;

/// @brief Default of Nav2's `enable_stamped_cmd_vel`: false on Humble and Jazzy, true on Kilted,
/// Lyrical and later (`RCLCPP_VERSION_GTE(29, 0, 0)`). Humble always publishes `Twist`.
/// vectorfield_sim uses it as its own default so the whole chain agrees without overlays.
#if RCLCPP_VERSION_GTE(29, 0, 0)
inline constexpr bool kStampedCmdVelDefault = true;
#else
inline constexpr bool kStampedCmdVelDefault = false;
#endif
/// @}

/// @name Nodes
/// @{

#if VF_HAS_ROS_COMMON
/// @brief The Nav2 lifecycle node: `nav2::LifecycleNode` on Lyrical and later,
/// `nav2_util::LifecycleNode` before.
using LifecycleNode = nav2::LifecycleNode;
#else
using LifecycleNode = nav2_util::LifecycleNode;
#endif

/// @brief The parent-node type of `Controller::configure()` and `GoalChecker::initialize()`.
///
/// `std::weak_ptr<rclcpp_lifecycle::LifecycleNode>` on Humble to Kilted (nav2_util::LifecycleNode
/// declares no pointer aliases and inherits rclcpp_lifecycle's), `std::weak_ptr<nav2::LifecycleNode>`
/// on Lyrical and later. Never spell it `std::weak_ptr<LifecycleNode>`. Checked against the
/// nav2_core signatures by the static_asserts at the end of this header.
using NodeWeakPtr = LifecycleNode::WeakPtr;

/// @brief The shared-pointer form of the parent node (same caveat as NodeWeakPtr: on Humble to
/// Kilted it points to the rclcpp_lifecycle base and does not expose `createBond()`).
using NodeSharedPtr = LifecycleNode::SharedPtr;

/// @brief Subscribes through the `rclcpp_lifecycle::LifecycleNode` base on every distro.
///
/// `nav2::LifecycleNode::create_subscription(topic, callback, qos, group)` (Lyrical and later)
/// hides the base `(topic, qos, callback)` overload; upcasting first gives one call form.
/// @tparam MessageT Message type.
/// @param node Pointer-like node (shared_ptr or raw pointer).
/// @param topic Topic name.
/// @param qos Subscription QoS.
/// @param callback Any callback rclcpp accepts for MessageT.
template <typename MessageT, typename NodeT, typename CallbackT>
typename rclcpp::Subscription<MessageT>::SharedPtr subscribe(
  const NodeT & node, const std::string & topic, const rclcpp::QoS & qos, CallbackT && callback)
{
  rclcpp_lifecycle::LifecycleNode & base = *node;
  return base.template create_subscription<MessageT>(topic, qos, std::forward<CallbackT>(callback));
}
/// @}

/// @name The cmd_vel publisher
/// @{

#if VF_COMPAT_HAS_TWIST_PUBLISHER
/// @brief Nav2's `Twist`/`TwistStamped` publisher (Jazzy and later: `nav2_util::TwistPublisher`,
/// honouring the node's `enable_stamped_cmd_vel` parameter, whose default is
/// kStampedCmdVelDefault).
///
/// On Humble, a minimal class with the same interface that always publishes `Twist` (Humble has
/// no twist_publisher.hpp and no `enable_stamped_cmd_vel`). Construct it as
/// `TwistPublisher(node, topic, qos)` on every distro; activate it explicitly.
using TwistPublisher = nav2_util::TwistPublisher;
#else
class TwistPublisher
{
public:
  /// @brief A lifecycle `Twist` publisher on `topic` (Humble only).
  /// @param node The node (any rclcpp_lifecycle node).
  /// @param topic Topic name.
  /// @param qos Publisher QoS.
  TwistPublisher(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & topic,
    const rclcpp::QoS & qos)
  : pub_(node->create_publisher<geometry_msgs::msg::Twist>(topic, qos))
  {
  }

  /// @brief Activates the lifecycle publisher.
  void on_activate() { pub_->on_activate(); }
  /// @brief Deactivates the lifecycle publisher.
  void on_deactivate() { pub_->on_deactivate(); }
  /// @brief True while activated.
  [[nodiscard]] bool is_activated() const { return pub_->is_activated(); }
  /// @brief Publishes the twist part of `velocity` (the header is dropped).
  void publish(std::unique_ptr<geometry_msgs::msg::TwistStamped> velocity)
  {
    pub_->publish(std::make_unique<geometry_msgs::msg::Twist>(velocity->twist));
  }
  /// @brief Number of matched subscriptions.
  [[nodiscard]] std::size_t get_subscription_count() const
  {
    return pub_->get_subscription_count();
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};
#endif
/// @}

/// @name TF factories
/// The node-pointer constructors of the tf2_ros classes are `[[deprecated]]` on Lyrical (which
/// breaks -Werror) and removed on Rolling; their replacement takes `NodeInterfaces`, which Kilted
/// lacks. These factories pass `*node` when `RCLCPP_VERSION_GTE(30, 0, 0)` and the pointer (or
/// its interfaces) before, exactly like nav2_ros_common/tf2_factories.hpp.
/// @{

/// @brief A TF buffer on the node's clock with a `CreateTimerROS` interface (needed for
/// asynchronous `waitForTransform`).
/// @param node Pointer-like node.
/// @param callback_group Callback group of the buffer's timers; null for the default.
template <typename NodeT>
std::shared_ptr<tf2_ros::Buffer> makeTfBuffer(
  const NodeT & node, rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
{
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
#if RCLCPP_VERSION_GTE(30, 0, 0)
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(*node, callback_group);
#else
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(), node->get_node_timers_interface(), callback_group);
#endif
  buffer->setCreateTimerInterface(timer_interface);
  return buffer;
}

/// @brief A callback group spun by a thread of its own, which stops at destruction however soon
/// that comes.
///
/// rclcpp's Executor::cancel() is lost when it comes before spin() starts, so the dedicated
/// threads of tf2_ros's TransformListener and Nav2's NodeThread never stop if they are destroyed
/// right after their creation; this one spins in bounded slices and checks a flag of its own.
class GroupSpinner
{
public:
  GroupSpinner(
    rclcpp::CallbackGroup::SharedPtr group,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
  : context_(node_base->get_context()), executor_(optionsFor(context_))
  {
    executor_.add_callback_group(std::move(group), std::move(node_base));
    thread_ = std::thread([this]() {
      while (!stop_.load() && rclcpp::ok(context_)) {
        executor_.spin_once(std::chrono::milliseconds(100));
      }
    });
  }

  ~GroupSpinner()
  {
    stop_.store(true);
    try {
      executor_.cancel();  // only wakes the thread early: it sees the flag within a slice
    } catch (const std::exception &) {
    }
    thread_.join();
  }

  GroupSpinner(const GroupSpinner &) = delete;
  GroupSpinner & operator=(const GroupSpinner &) = delete;
  GroupSpinner(GroupSpinner &&) = delete;
  GroupSpinner & operator=(GroupSpinner &&) = delete;

private:
  static rclcpp::ExecutorOptions optionsFor(const rclcpp::Context::SharedPtr & context)
  {
    rclcpp::ExecutorOptions options;
    options.context = context;
    return options;
  }

  rclcpp::Context::SharedPtr context_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

/// @brief A TF listener that fills `buffer` from the node's `/tf` and `/tf_static`
/// subscriptions.
/// @param buffer Buffer to fill; must outlive the listener.
/// @param node Pointer-like node.
/// @param spin_thread Spin the listener's subscriptions in a dedicated thread (a GroupSpinner,
/// not tf2_ros's own, so that the listener can be destroyed at any time).
template <typename NodeT>
std::shared_ptr<tf2_ros::TransformListener> makeTfListener(
  tf2_ros::Buffer & buffer, const NodeT & node, bool spin_thread = true)
{
#if RCLCPP_VERSION_GTE(30, 0, 0)
  auto && node_arg = *node;
#else
  const NodeT & node_arg = node;
#endif
  if (!spin_thread) {
    return std::make_shared<tf2_ros::TransformListener>(buffer, node_arg, false);
  }
  const auto node_base = node->get_node_base_interface();
  auto group =
    node_base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto options =
    tf2_ros::detail::get_default_transform_listener_sub_options<std::allocator<void>>();
  options.callback_group = group;
  auto static_options =
    tf2_ros::detail::get_default_transform_listener_static_sub_options<std::allocator<void>>();
  static_options.callback_group = group;
  /// The listener and its thread; the thread stops first (reverse declaration order).
  struct Spun
  {
    std::shared_ptr<tf2_ros::TransformListener> listener;
    std::unique_ptr<GroupSpinner> spinner;
  };
  auto spun = std::make_shared<Spun>();
  spun->listener = std::make_shared<tf2_ros::TransformListener>(
    buffer, node_arg, false, tf2_ros::DynamicListenerQoS(), tf2_ros::StaticListenerQoS(), options,
    static_options);
  spun->spinner = std::make_unique<GroupSpinner>(group, node_base);
  buffer.setUsingDedicatedThread(true);
  return std::shared_ptr<tf2_ros::TransformListener>(spun, spun->listener.get());
}

/// @brief A `/tf` broadcaster.
/// @param node Pointer-like node.
template <typename NodeT>
std::shared_ptr<tf2_ros::TransformBroadcaster> makeTfBroadcaster(const NodeT & node)
{
#if RCLCPP_VERSION_GTE(30, 0, 0)
  return std::make_shared<tf2_ros::TransformBroadcaster>(*node);
#else
  return std::make_shared<tf2_ros::TransformBroadcaster>(node);
#endif
}

/// @brief A `/tf_static` broadcaster.
/// @param node Pointer-like node.
template <typename NodeT>
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> makeStaticTfBroadcaster(const NodeT & node)
{
#if RCLCPP_VERSION_GTE(30, 0, 0)
  return std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node);
#else
  return std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
#endif
}
/// @}

/// @name Goal checker
/// @{

/// @brief `GoalChecker::getTolerances()` with the same arguments on every distro.
///
/// Gen C adds a `path_length_tolerance` out-argument, which is read and dropped here. Per the
/// nav2_core contract, unmeasured fields come back as `std::numeric_limits<double>::lowest()`;
/// decode them with toGoalTolerances() (conversions.hpp).
/// @param goal_checker The server's checker; null returns false.
/// @param pose_tolerance Receives the pose tolerance (position: XY radius; orientation: yaw
/// tolerance as a quaternion, identity for PositionGoalChecker).
/// @param vel_tolerance Receives the velocity tolerance.
/// @return The checker's result: true if the tolerances are valid; false for a null checker.
inline bool get_goal_tolerances(
  nav2_core::GoalChecker * goal_checker, geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  if (goal_checker == nullptr) {
    return false;
  }
#if VF_HAS_PATH_HANDLER
  double path_length_tolerance = 0.0;
  return goal_checker->getTolerances(pose_tolerance, vel_tolerance, path_length_tolerance);
#else
  return goal_checker->getTolerances(pose_tolerance, vel_tolerance);
#endif
}

/// @brief The Gen A/B emulation of the server's stateful XY latch (math_spec §9.1).
///
/// Latches once \f$\lVert p - g\rVert \le\f$ xy_tol and stays latched until rearm(), which the
/// plugin calls in every `setPlan()` because the server resets its goal checker on every plan
/// (nav2_api_matrix.md §4.2). Compiled on every distro (Gen C also uses it when the server passes
/// no checker). A plain value type; not thread-safe (control thread only).
class EmulatedXYLatch
{
public:
  /// @brief Clears the latch (every setPlan()).
  void rearm() noexcept { latched_ = false; }

  /// @brief Latches when `distance <= xy_tol`.
  /// @param distance \f$\lVert p - g\rVert\f$ (m); NaN never latches.
  /// @param xy_tol XY tolerance (m).
  /// @return Whether XY is reached (latched) after this update.
  bool update(double distance, double xy_tol) noexcept
  {
    if (distance <= xy_tol) {
      latched_ = true;
    }
    return latched_;
  }

  /// @brief Whether XY is reached (latched).
  bool latched() const noexcept { return latched_; }

private:
  bool latched_{false};
};

/// @brief "XY reached" as the server sees it (math_spec §9.1): one call form for every
/// distro, so the shared compute path has no `#if`.
///
/// - **Gen C:** forwards `goal_checker->isGoalXYReached(pose.pose, global_goal.pose, velocity,
///   transformed_global_plan)` with the server's own arguments, unchanged. Never pass the
///   conditioned window, `W.back()` or a base-frame copy: the checkers read plan geometry.
///   `latch` and `xy_tol` are unused (except with a null checker, see below).
/// - **Gen A/B:** the emulated latch \f$\lVert p - g\rVert \le\f$ xy_tol, with `pose` and
///   `global_goal` in the same (costmap) frame; `goal_checker`, `velocity` and
///   `transformed_global_plan` are unused.
/// - A null `goal_checker` uses the latch on every distro.
/// @param goal_checker The server's checker (may be null).
/// @param latch The plugin's latch, re-armed in every setPlan() (Gen A/B).
/// @param pose Robot pose as passed by the server (costmap global frame).
/// @param global_goal Gen C: the server's `global_goal`; Gen A/B: LegacyPathHandler's goal.
/// @param velocity Measured velocity as passed by the server.
/// @param transformed_global_plan Gen C: the server's `transformed_global_plan`.
/// @param xy_tol XY tolerance from get_goal_tolerances() (Gen A/B and null checker).
inline bool goal_xy_reached(
  nav2_core::GoalChecker * goal_checker, EmulatedXYLatch & latch,
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::PoseStamped & global_goal,
  const geometry_msgs::msg::Twist & velocity, const nav_msgs::msg::Path & transformed_global_plan,
  double xy_tol)
{
#if VF_HAS_PATH_HANDLER
  if (goal_checker != nullptr) {
    return goal_checker->isGoalXYReached(
      pose.pose, global_goal.pose, velocity, transformed_global_plan);
  }
#else
  (void)goal_checker;
  (void)velocity;
  (void)transformed_global_plan;
#endif
  const double dx = pose.pose.position.x - global_goal.pose.position.x;
  const double dy = pose.pose.position.y - global_goal.pose.position.y;
  return latch.update(std::hypot(dx, dy), xy_tol);
}
/// @}

/// @name Controller exceptions
/// What the plugin throws (nav2_api_matrix.md §3.6). On Humble these derive
/// from `nav2_core::PlannerException`, to which Humble's server applies `failure_tolerance`
/// patience; on Jazzy and later they are the nav2_core types (NoValidControl: patience;
/// InvalidPath and ControllerTFError: immediate abort).
/// @{
#if VF_HAS_CONTROLLER_EXCEPTIONS
/// @brief Base class of the controller exceptions (`nav2_core::PlannerException` on Humble).
using ControllerException = nav2_core::ControllerException;
/// @brief No valid command this cycle (transient: kBlocked, kNonFinite, kContourWatchdog,
/// kHeadingInfeasible, a colliding footprint, a rejected parameter rebuild).
using NoValidControl = nav2_core::NoValidControl;
/// @brief The plan cannot be followed (empty plan or window, a cusp without `allow_reversing`).
using InvalidPath = nav2_core::InvalidPath;
/// @brief A transform needed this cycle is unavailable.
using ControllerTFError = nav2_core::ControllerTFError;
#else
using ControllerException = nav2_core::PlannerException;

class NoValidControl : public nav2_core::PlannerException
{
public:
  /// @brief An exception with a human-readable description.
  explicit NoValidControl(const std::string & description)
  : nav2_core::PlannerException(description)
  {
  }
};

class InvalidPath : public nav2_core::PlannerException
{
public:
  /// @brief An exception with a human-readable description.
  explicit InvalidPath(const std::string & description) : nav2_core::PlannerException(description)
  {
  }
};

class ControllerTFError : public nav2_core::PlannerException
{
public:
  /// @brief An exception with a human-readable description.
  explicit ControllerTFError(const std::string & description)
  : nav2_core::PlannerException(description)
  {
  }
};
#endif
/// @}

/// @cond VF_INTERNAL
namespace detail
{

template <typename F>
struct MemberFunctionTraits;

template <typename C, typename R, typename... A>
struct MemberFunctionTraits<R (C::*)(A...)>
{
  using Parameters = std::tuple<std::decay_t<A>...>;
};

template <std::size_t I, typename F>
using ParameterAt = std::tuple_element_t<I, typename MemberFunctionTraits<F>::Parameters>;

}  // namespace detail
/// @endcond

static_assert(
  std::is_same_v<NodeWeakPtr, detail::ParameterAt<0, decltype(&nav2_core::Controller::configure)>>,
  "compat::NodeWeakPtr must equal the parent type of nav2_core::Controller::configure() "
  "(nav2_api_matrix.md §2.4)");
static_assert(
  std::is_same_v<
    std::shared_ptr<tf2_ros::Buffer>,
    detail::ParameterAt<2, decltype(&nav2_core::Controller::configure)>>,
  "Controller::configure() must take std::shared_ptr<tf2_ros::Buffer> on every distro "
  "(nav2::TransformBuffer is tf2_ros::Buffer)");
static_assert(
  std::is_same_v<
    NodeWeakPtr, detail::ParameterAt<0, decltype(&nav2_core::GoalChecker::initialize)>>,
  "compat::NodeWeakPtr must equal the parent type of nav2_core::GoalChecker::initialize()");
static_assert(
  std::is_base_of_v<rclcpp_lifecycle::LifecycleNode, LifecycleNode>,
  "compat::LifecycleNode must derive from rclcpp_lifecycle::LifecycleNode");
static_assert(
  VF_HAS_PATH_HANDLER == 0 || (VF_HAS_ROS_COMMON == 1 && VF_HAS_CONTROLLER_EXCEPTIONS == 1),
  "Gen C (path handler) implies nav2_ros_common and the controller exceptions");

}  // namespace compat
}  // namespace vectorfield_ros

#endif  // VECTORFIELD_ROS__COMPAT_HPP_
