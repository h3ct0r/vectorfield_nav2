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

#include "server_harness.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <typeinfo>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rcl/time.h"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_ros/conversions.hpp"
#if VF_HAS_PATH_HANDLER
#include "nav2_core/path_handler.hpp"
#endif

namespace vectorfield_nav2_controller
{
namespace test
{

namespace vf = vectorfield_core;
namespace vr = vectorfield_ros;
namespace compat = vectorfield_ros::compat;

namespace
{

/// Harness time at construction (s).
constexpr double kStartTime = 1000.0;
/// Our plugin and the rotation shim (pluginlib type strings).
constexpr const char * kControllerType = "vectorfield_nav2_controller::VectorFieldController";
constexpr const char * kShimType = "nav2_rotation_shim_controller::RotationShimController";
/// The server's goal-checker id (stock nav2_params.yaml).
constexpr const char * kGoalCheckerId = "general_goal_checker";
/// The server's default path-handler id (Gen C); path-handler parameter overrides go under it.
constexpr const char * kPathHandlerId = "PathHandler";
#if VF_HAS_PATH_HANDLER
/// Gen C: the server's default path-handler type.
constexpr const char * kPathHandlerType = "nav2_controller::FeasiblePathHandler";
#endif
/// The server's `min_{x,y,theta}_velocity_threshold` defaults.
constexpr double kVelocityThreshold = 1e-4;
/// Every harness transform is injected under this authority.
constexpr const char * kAuthority = "harness";

builtin_interfaces::msg::Time toStamp(double t)
{
  return rclcpp::Time(static_cast<std::int64_t>(std::llround(t * 1e9)), RCL_ROS_TIME);
}

geometry_msgs::msg::TransformStamped toTransform(
  const vf::Pose2 & pose, const std::string & parent, const std::string & child, double t)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = parent;
  transform.header.stamp = toStamp(t);
  transform.child_frame_id = child;
  transform.transform.translation.x = pose.p.x();
  transform.transform.translation.y = pose.p.y();
  transform.transform.rotation = vr::yawToQuaternion(pose.yaw);
  return transform;
}

/// Declares a parameter, or sets it when the node already declared it (Costmap2DROS declares
/// most of its parameters in its constructor, some distros only in on_configure).
void setOrDeclare(rclcpp_lifecycle::LifecycleNode & node, const rclcpp::Parameter & parameter)
{
  if (node.has_parameter(parameter.get_name())) {
    const rcl_interfaces::msg::SetParametersResult result = node.set_parameter(parameter);
    if (!result.successful) {
      throw std::runtime_error(
        "ServerHarness: cannot set " + parameter.get_name() + ": " + result.reason);
    }
  } else {
    node.declare_parameter(parameter.get_name(), parameter.get_parameter_value());
  }
}

/// The type name the record shows for an exception.
std::string exceptionName(const std::exception & e)
{
  if (dynamic_cast<const compat::NoValidControl *>(&e) != nullptr) {
    return "NoValidControl";
  }
  if (dynamic_cast<const compat::InvalidPath *>(&e) != nullptr) {
    return "InvalidPath";
  }
  if (dynamic_cast<const compat::ControllerTFError *>(&e) != nullptr) {
    return "ControllerTFError";
  }
  return typeid(e).name();
}

/// The distro-independent abort reason of an exception.
std::string abortReason(const std::exception & e)
{
  if (dynamic_cast<const compat::NoValidControl *>(&e) != nullptr) {
    return "NO_VALID_CONTROL";
  }
  if (dynamic_cast<const compat::InvalidPath *>(&e) != nullptr) {
    return "INVALID_PATH";
  }
  if (dynamic_cast<const compat::ControllerTFError *>(&e) != nullptr) {
    return "TF_ERROR";
  }
  return "UNKNOWN";
}

/// Exceptions that go through the server's patience (nav2_api_matrix.md §3.6): any
/// PlannerException on Humble, only NoValidControl on Jazzy and later.
bool patienceApplies(const std::exception & e)
{
#if VF_HAS_CONTROLLER_EXCEPTIONS
  return dynamic_cast<const nav2_core::NoValidControl *>(&e) != nullptr;
#else
  return dynamic_cast<const nav2_core::PlannerException *>(&e) != nullptr;
#endif
}

double threshold(double value) { return std::abs(value) < kVelocityThreshold ? 0.0 : value; }

/// The local costmap, with a view of its executor. Nav2's NodeThread loses a cancel() that comes
/// before its spin() starts, so cleaning up a costmap within microseconds of configuring it can
/// hang in join(); the harness waits until the costmap's executor spins.
class HarnessCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  using nav2_costmap_2d::Costmap2DROS::Costmap2DROS;

  /// True once the costmap's executor thread spins (or the costmap has none).
  bool executorReady() const { return executor_ == nullptr || executor_->is_spinning(); }
};

}  // namespace

struct ServerHarness::Impl
{
  explicit Impl(HarnessOptions harness_options)
  : options(std::move(harness_options)),
    controller_loader("nav2_core", "nav2_core::Controller"),
    checker_loader("nav2_core", "nav2_core::GoalChecker")
#if VF_HAS_PATH_HANDLER
    ,
    handler_loader("nav2_core", "nav2_core::PathHandler")
#endif
  {
  }

  HarnessOptions options;
  double period{0.05};
  double t{kStartTime};
  compat::NodeSharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::shared_ptr<tf2_ros::Buffer> tf;
  rclcpp::executors::SingleThreadedExecutor executor;

  // Loaders before the instances they create (destroyed after them).
  pluginlib::ClassLoader<nav2_core::Controller> controller_loader;
  pluginlib::ClassLoader<nav2_core::GoalChecker> checker_loader;
#if VF_HAS_PATH_HANDLER
  pluginlib::ClassLoader<nav2_core::PathHandler> handler_loader;
  std::shared_ptr<nav2_core::PathHandler> handler;
#endif
  std::shared_ptr<nav2_core::Controller> controller;
  std::shared_ptr<nav2_core::GoalChecker> checker;
  bool configured{false};
  bool active{false};

  // World.
  vf::Pose2 robot;
  vf::Twist2 twist;
  vf::Pose2 map_to_odom;
  std::optional<vf::Pose2> goal_skew;

  // Goal.
  GoalStatus status{GoalStatus::kNone};
  GoalResult result;
  double last_valid{kStartTime};
  bool cancel_requested{false};
  std::optional<nav_msgs::msg::Path> pending;
  geometry_msgs::msg::PoseStamped end_pose;
  geometry_msgs::msg::Twist last_published;
  std::vector<CycleRecord> history;

  void setTime()
  {
    rcl_clock_t * clock = node->get_clock()->get_clock_handle();
    const auto ns = static_cast<rcl_time_point_value_t>(std::llround(t * 1e9));
    if (rcl_set_ros_time_override(clock, ns) != RCL_RET_OK) {
      throw std::runtime_error("ServerHarness: cannot set the ROS time");
    }
  }

  void publishTf(const vf::Pose2 & map_odom)
  {
    tf->setTransform(
      toTransform(map_odom, options.map_frame, options.odom_frame, t), kAuthority, true);
    tf->setTransform(
      toTransform(robot, options.odom_frame, options.base_frame, t), kAuthority, true);
  }

  geometry_msgs::msg::PoseStamped robotPose() const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = options.odom_frame;
    pose.header.stamp = toStamp(t);
    pose.pose = vr::toPoseMsg(robot);
    return pose;
  }

  geometry_msgs::msg::Twist measuredTwist() const
  {
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = threshold(twist.vx);
    velocity.linear.y = threshold(twist.vy);
    velocity.angular.z = threshold(twist.wz);
    return velocity;
  }

  void initGoalChecker()
  {
    checker = checker_loader.createSharedInstance(options.goal_checker);
    checker->initialize(compat::NodeWeakPtr(node), kGoalCheckerId, costmap);
  }

  /// The goal exit of the distro (nav2_api_matrix.md §4.3): zero, and reset() on Jazzy and later.
  void finish(
    GoalStatus final_status, const std::string & reason, const std::string & message,
    CycleRecord * record)
  {
    status = final_status;
    result = GoalResult{final_status, reason, message};
    pending.reset();
    cancel_requested = false;
    if (record != nullptr) {
      record->published = geometry_msgs::msg::Twist();
    }
#if VF_HAS_CONTROLLER_EXCEPTIONS
    if (controller) {
      controller->reset();
    }
#endif
  }

  void abortWith(const std::exception & e, CycleRecord * record)
  {
    finish(GoalStatus::kAborted, abortReason(e), e.what(), record);
  }

  /// setPlannerPath() of the distro. False if the goal was aborted.
  bool setPlannerPath(nav_msgs::msg::Path path, CycleRecord * record)
  {
    if (path.header.frame_id.empty()) {
      path.header.frame_id = options.map_frame;
    }
    if (path.poses.empty()) {
      finish(GoalStatus::kAborted, "INVALID_PATH", "Path is empty.", record);
      return false;
    }
    try {
#if VF_HAS_PATH_HANDLER
      controller->newPathReceived(path);
      handler->setPlan(path);
#else
      controller->setPlan(path);
#endif
    } catch (const std::exception & e) {
      abortWith(e, record);
      return false;
    }
    end_pose = path.poses.back();
    end_pose.header.frame_id = path.header.frame_id;
    checker->reset();
    return true;
  }

#if !VF_HAS_PATH_HANDLER
  /// Gen A/B isGoalReached(): end_pose_ transformed into the costmap frame (Humble ignores a
  /// transform failure, Jazzy throws ControllerTFError).
  bool isGoalReached(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity)
  {
    geometry_msgs::msg::PoseStamped end_in_costmap;
    try {
      const geometry_msgs::msg::TransformStamped transform = tf->lookupTransform(
        costmap->getGlobalFrameID(), end_pose.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(end_pose, end_in_costmap, transform);
    } catch (const tf2::TransformException & e) {
#if VF_HAS_CONTROLLER_EXCEPTIONS
      throw compat::ControllerTFError(e.what());
#else
      (void)e;
      return false;
#endif
    }
    return checker->isGoalReached(pose.pose, end_in_costmap.pose, velocity);
  }
#endif

  /// computeAndPublishVelocity(): the call, then the server's exception handling and patience.
  template <typename Compute>
  void computeAndPublish(CycleRecord * record, Compute && compute)
  {
    record->computed = true;
    try {
      const geometry_msgs::msg::TwistStamped command = compute();
      last_valid = t;
      const geometry_msgs::msg::Twist & twist_out = command.twist;
      const bool finite = std::isfinite(twist_out.linear.x) && std::isfinite(twist_out.linear.y) &&
                          std::isfinite(twist_out.angular.z);
      // Lyrical and later drop a non-finite command (the robot keeps the last one).
      record->published = (finite || !VF_HAS_PATH_HANDLER) ? twist_out : last_published;
    } catch (const std::exception & e) {
      record->exception = exceptionName(e);
      record->message = e.what();
      const double tolerance = options.failure_tolerance;
      if (patienceApplies(e) && (tolerance > 0.0 || tolerance == -1.0)) {
        record->published = geometry_msgs::msg::Twist();
        if (t - last_valid > tolerance && tolerance != -1.0) {
          finish(GoalStatus::kAborted, "PATIENCE_EXCEEDED", "Controller patience exceeded", record);
        }
      } else {
        abortWith(e, record);
      }
    }
  }

  /// One control cycle in the distro's order (see the file comment of the header).
  void runCycle(CycleRecord * record)
  {
    const geometry_msgs::msg::PoseStamped pose = robotPose();
    const geometry_msgs::msg::Twist velocity = measuredTwist();

    // 1. Cancel.
    if (cancel_requested) {
#if VF_HAS_CONTROLLER_EXCEPTIONS
      record->cancel_called = true;
      record->cancel_result = controller->cancel();
      if (record->cancel_result) {
        finish(GoalStatus::kCanceled, "", "", record);
        return;
      }
#else
      finish(GoalStatus::kCanceled, "", "", record);
      return;
#endif
    }

    // 2. updateGlobalPath().
    if (pending) {
      nav_msgs::msg::Path path = std::move(*pending);
      pending.reset();
      if (!setPlannerPath(std::move(path), record)) {
        return;
      }
    }

#if VF_HAS_PATH_HANDLER
    // 3. Gen C: the window at the robot-pose stamp t1, the goal check against its end, then the
    // goal at t3 (a second robot-pose read, possibly after a map->odom change).
    nav_msgs::msg::Path window;
    try {
      const nav2_core::PathSegment segment = handler->findPlanSegment(pose);
      window = handler->transformLocalPlan(segment.first, segment.second);
    } catch (const std::exception & e) {
      abortWith(e, record);
      return;
    }
    // The server checks transformed_end_pose_ (the full plan's last pose at t1), which is
    // W.back() unless a path constraint truncated the window (lyrical:CS#L966-L979).
    geometry_msgs::msg::PoseStamped end_in_costmap = window.poses.back();
    try {
      const geometry_msgs::msg::TransformStamped transform = tf->lookupTransform(
        costmap->getGlobalFrameID(), end_pose.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(end_pose, end_in_costmap, transform);
    } catch (const tf2::TransformException & e) {
      abortWith(compat::ControllerTFError(e.what()), record);
      return;
    }
    if (checker->isGoalReached(pose.pose, end_in_costmap.pose, velocity, window)) {
      record->goal_reached = true;
      finish(GoalStatus::kSucceeded, "", "", record);
      return;
    }
    geometry_msgs::msg::PoseStamped global_goal;
    try {
      if (goal_skew) {
        vf::Pose2 skewed = map_to_odom;
        skewed.p += goal_skew->p;
        skewed.yaw = vf::wrapAngle(skewed.yaw + goal_skew->yaw);
        publishTf(skewed);
      }
      global_goal = handler->getTransformedGoal(pose.header.stamp);
      publishTf(map_to_odom);
    } catch (const std::exception & e) {
      publishTf(map_to_odom);
      abortWith(e, record);
      return;
    }
    computeAndPublish(record, [&]() {
      return controller->computeVelocityCommands(
        pose, velocity, checker.get(), window, global_goal);
    });
#else
#if VF_HAS_CONTROLLER_EXCEPTIONS
    // 3. Gen B: the goal check before compute.
    try {
      if (isGoalReached(pose, velocity)) {
        record->goal_reached = true;
        finish(GoalStatus::kSucceeded, "", "", record);
        return;
      }
    } catch (const std::exception & e) {
      abortWith(e, record);
      return;
    }
#endif
    computeAndPublish(
      record, [&]() { return controller->computeVelocityCommands(pose, velocity, checker.get()); });
#if !VF_HAS_CONTROLLER_EXCEPTIONS
    // 4. Gen A: the goal check after compute.
    if (status == GoalStatus::kRunning && isGoalReached(pose, velocity)) {
      record->goal_reached = true;
      finish(GoalStatus::kSucceeded, "", "", record);
    }
#endif
#endif
  }
};

ServerHarness::ServerHarness(HarnessOptions options)
: impl_(std::make_unique<Impl>(std::move(options)))
{
  Impl & s = *impl_;
  const HarnessOptions & o = s.options;
  s.period = 1.0 / o.controller_frequency;
  s.robot = o.initial_pose;
  s.map_to_odom = o.map_to_odom;

  // The server node; every parameter is an override, as from a params file.
  std::vector<rclcpp::Parameter> overrides = {
    rclcpp::Parameter("use_sim_time", true),
    rclcpp::Parameter("controller_frequency", o.controller_frequency),
    rclcpp::Parameter("failure_tolerance", o.failure_tolerance),
    rclcpp::Parameter(
      o.plugin_name + ".plugin", std::string(o.rotation_shim ? kShimType : kControllerType)),
    rclcpp::Parameter("goal_checker_plugins", std::vector<std::string>{kGoalCheckerId}),
    rclcpp::Parameter(std::string(kGoalCheckerId) + ".plugin", o.goal_checker),
    rclcpp::Parameter(std::string(kGoalCheckerId) + ".xy_goal_tolerance", o.xy_goal_tolerance),
    rclcpp::Parameter(std::string(kGoalCheckerId) + ".yaw_goal_tolerance", o.yaw_goal_tolerance),
  };
  for (const rclcpp::Parameter & parameter : o.goal_checker_parameters) {
    overrides.emplace_back(
      std::string(kGoalCheckerId) + "." + parameter.get_name(), parameter.get_parameter_value());
  }
  for (const rclcpp::Parameter & parameter : o.path_handler_parameters) {
    overrides.emplace_back(
      std::string(kPathHandlerId) + "." + parameter.get_name(), parameter.get_parameter_value());
  }
  if (o.rotation_shim) {
    // Humble/Jazzy: a string; Lyrical and later: a sub-plugin with its own `.plugin`.
    overrides.emplace_back(
      o.plugin_name + (VF_HAS_PATH_HANDLER ? ".primary_controller.plugin" : ".primary_controller"),
      std::string(kControllerType));
  }
  overrides.insert(overrides.end(), o.parameters.begin(), o.parameters.end());
  s.node = std::make_shared<compat::LifecycleNode>(
    "controller_server", "", rclcpp::NodeOptions().parameter_overrides(overrides));
  if (rcl_enable_ros_time_override(s.node->get_clock()->get_clock_handle()) != RCL_RET_OK) {
    throw std::runtime_error("ServerHarness: cannot enable the ROS time override");
  }
  s.setTime();
  s.executor.add_node(s.node->get_node_base_interface());

  // The local costmap: fixed origin, not rolling, no layers, system time (a TF buffer on a
  // frozen sim clock would wait forever on a failed lookup). Configured, never activated.
#if VF_HAS_ROS_COMMON
  const auto costmap = std::make_shared<HarnessCostmap>("local_costmap", "/", false);
#elif VF_HAS_CONTROLLER_EXCEPTIONS
  const auto costmap = std::make_shared<HarnessCostmap>("local_costmap", false);
#else
  const auto costmap = std::make_shared<HarnessCostmap>("local_costmap");
#endif
  s.costmap = costmap;
  // Nav2 declares width and height as integers (metres).
  const auto side = static_cast<int>(std::lround(o.costmap_size));
  std::vector<rclcpp::Parameter> costmap_parameters = {
    rclcpp::Parameter("global_frame", o.odom_frame),
    rclcpp::Parameter("robot_base_frame", o.base_frame),
    rclcpp::Parameter("rolling_window", false),
    rclcpp::Parameter("width", side),
    rclcpp::Parameter("height", side),
    rclcpp::Parameter("resolution", o.costmap_resolution),
    rclcpp::Parameter("origin_x", o.costmap_center.x() - 0.5 * side),
    rclcpp::Parameter("origin_y", o.costmap_center.y() - 0.5 * side),
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("track_unknown_space", false),
    rclcpp::Parameter("robot_radius", o.robot_radius),
  };
  if (!o.footprint.empty()) {
    std::ostringstream footprint;
    footprint << '[';
    for (std::size_t i = 0; i < o.footprint.size(); ++i) {
      footprint << (i == 0 ? "" : ", ") << '[' << o.footprint[i].x << ", " << o.footprint[i].y
                << ']';
    }
    footprint << ']';
    costmap_parameters.emplace_back("footprint", footprint.str());
  }
  for (const rclcpp::Parameter & parameter : costmap_parameters) {
    setOrDeclare(*s.costmap, parameter);
  }
  if (s.costmap->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    throw std::runtime_error("ServerHarness: the local costmap failed to configure");
  }
  // Not a control-time wait: only until the costmap's executor thread has entered spin().
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!costmap->executorReady() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  s.tf = s.costmap->getTfBuffer();
  s.publishTf(s.map_to_odom);
  s.history.reserve(4096);
}

ServerHarness::~ServerHarness()
{
  Impl & s = *impl_;
  try {
    if (s.active) {
      deactivate();
    }
    if (s.configured) {
      cleanup();
    }
    s.executor.remove_node(s.node->get_node_base_interface());
    s.costmap->cleanup();
    s.costmap->shutdown();
    s.node->shutdown();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(s.node->get_logger(), "ServerHarness teardown: %s", e.what());
  }
}

void ServerHarness::configure()
{
  Impl & s = *impl_;
  if (!s.node->has_parameter("controller_frequency")) {
    s.node->declare_parameter("controller_frequency", 20.0);
    s.node->declare_parameter("failure_tolerance", 0.0);
  }
  const std::string type = s.options.rotation_shim ? kShimType : kControllerType;
  if (!s.node->has_parameter(s.options.plugin_name + ".plugin")) {
    s.node->declare_parameter(s.options.plugin_name + ".plugin", type);
  }
  try {
    s.controller = s.controller_loader.createSharedInstance(type);
    if (!VF_HAS_PATH_HANDLER) {
      s.initGoalChecker();  // Gen A/B: in on_configure
    }
    s.controller->configure(compat::NodeWeakPtr(s.node), s.options.plugin_name, s.tf, s.costmap);
  } catch (...) {
    s.controller.reset();
    s.checker.reset();
    throw;
  }
  s.configured = true;
}

void ServerHarness::activate()
{
  Impl & s = *impl_;
  s.controller->activate();
#if VF_HAS_PATH_HANDLER
  // Gen C: the goal checker and the path handler after the controller's activate().
  s.initGoalChecker();
  s.handler = s.handler_loader.createSharedInstance(kPathHandlerType);
  s.handler->initialize(
    compat::NodeWeakPtr(s.node), s.node->get_logger(), kPathHandlerId, s.costmap, s.tf);
#endif
  s.active = true;
}

void ServerHarness::deactivate()
{
  Impl & s = *impl_;
  s.controller->deactivate();
  s.active = false;
}

void ServerHarness::cleanup()
{
  Impl & s = *impl_;
  s.controller->cleanup();
  s.controller.reset();
  s.checker.reset();
#if VF_HAS_PATH_HANDLER
  s.handler.reset();
#endif
  s.configured = false;
  s.status = GoalStatus::kNone;
}

void ServerHarness::followPath(const nav_msgs::msg::Path & plan)
{
  Impl & s = *impl_;
  s.status = GoalStatus::kRunning;
  s.result = GoalResult{GoalStatus::kRunning, "", ""};
  s.last_valid = s.t;
  s.cancel_requested = false;
  s.pending.reset();
  s.setPlannerPath(plan, nullptr);
}

void ServerHarness::replan(const nav_msgs::msg::Path & plan) { impl_->pending = plan; }

void ServerHarness::requestCancel()
{
  if (impl_->status == GoalStatus::kRunning) {
    impl_->cancel_requested = true;
  }
}

GoalStatus ServerHarness::status() const noexcept { return impl_->status; }

const GoalResult & ServerHarness::result() const noexcept { return impl_->result; }

const CycleRecord & ServerHarness::spinOnce()
{
  Impl & s = *impl_;
  CycleRecord record;
  record.t = s.t;
  record.pose_odom = s.robot;
  record.pose_map = vf::Isometry2::fromPose(s.map_to_odom).apply(s.robot);
  if (s.status == GoalStatus::kRunning) {
    s.runCycle(&record);
  }
  VectorFieldController * plugin = controller();
  if (plugin != nullptr) {
    record.diagnostics = plugin->lastCycle();
  }
  s.last_published = record.published;

  // The plant: one period with the published command.
  const vf::Twist2 command = vr::toTwist2(record.published);
  s.robot = vf::sim::integrate(s.options.plant, s.robot, command, s.period);
  s.twist = vf::sim::feasibleTwist(s.options.plant, command);
  s.t += s.period;
  s.setTime();
  s.publishTf(s.map_to_odom);
  s.history.push_back(std::move(record));
  return s.history.back();
}

GoalStatus ServerHarness::runUntilDone(
  double timeout, const std::function<void(const CycleRecord &)> & on_cycle)
{
  Impl & s = *impl_;
  const double end = s.t + timeout - 1e-9;
  while (s.status == GoalStatus::kRunning && s.t < end) {
    const CycleRecord & record = spinOnce();
    if (on_cycle) {
      on_cycle(record);
    }
  }
  return s.status;
}

const std::vector<CycleRecord> & ServerHarness::history() const noexcept { return impl_->history; }

double ServerHarness::now() const noexcept { return impl_->t; }

void ServerHarness::spinSome() { impl_->executor.spin_some(std::chrono::milliseconds(10)); }

void ServerHarness::setMapToOdom(const vf::Pose2 & map_to_odom)
{
  impl_->map_to_odom = map_to_odom;
  impl_->publishTf(map_to_odom);
}

void ServerHarness::setGoalSkew(const vf::Pose2 & delta) { impl_->goal_skew = delta; }

void ServerHarness::setRobotPose(const vf::Pose2 & pose_odom)
{
  impl_->robot = pose_odom;
  impl_->twist = vf::Twist2{};
  impl_->publishTf(impl_->map_to_odom);
}

namespace
{

/// Sets every cell whose centre satisfies `inside` (odom frame) to `cost`.
template <typename Inside>
void paintCells(nav2_costmap_2d::Costmap2DROS & costmap_ros, unsigned char cost, Inside && inside)
{
  nav2_costmap_2d::Costmap2D & costmap = *costmap_ros.getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());
  for (unsigned int my = 0; my < costmap.getSizeInCellsY(); ++my) {
    for (unsigned int mx = 0; mx < costmap.getSizeInCellsX(); ++mx) {
      double wx = 0.0;
      double wy = 0.0;
      costmap.mapToWorld(mx, my, wx, wy);
      if (inside(vf::Vec2(wx, wy))) {
        costmap.setCost(mx, my, cost);
      }
    }
  }
}

}  // namespace

void ServerHarness::setLethalBox(const vf::Vec2 & min_corner, const vf::Vec2 & max_corner)
{
  paintCells(*impl_->costmap, nav2_costmap_2d::LETHAL_OBSTACLE, [&](const vf::Vec2 & c) {
    return c.x() >= min_corner.x() && c.x() <= max_corner.x() && c.y() >= min_corner.y() &&
           c.y() <= max_corner.y();
  });
}

void ServerHarness::setLethalDisc(const vf::Vec2 & center, double radius)
{
  paintCells(*impl_->costmap, nav2_costmap_2d::LETHAL_OBSTACLE, [&](const vf::Vec2 & c) {
    return (c - center).norm() <= radius;
  });
}

void ServerHarness::clearObstacles()
{
  paintCells(*impl_->costmap, nav2_costmap_2d::FREE_SPACE, [](const vf::Vec2 &) { return true; });
}

void ServerHarness::setFootprint(const std::vector<geometry_msgs::msg::Point> & polygon)
{
  geometry_msgs::msg::Polygon footprint;
  for (const geometry_msgs::msg::Point & point : polygon) {
    geometry_msgs::msg::Point32 vertex;
    vertex.x = static_cast<float>(point.x);
    vertex.y = static_cast<float>(point.y);
    footprint.points.push_back(vertex);
  }
#if VF_HAS_ROS_COMMON
  impl_->costmap->setRobotFootprintPolygon(footprint);
#else
  impl_->costmap->setRobotFootprintPolygon(
    std::make_shared<geometry_msgs::msg::Polygon>(footprint));
#endif
}

void ServerHarness::setSpeedLimit(double speed_limit, bool percentage)
{
  activeController()->setSpeedLimit(speed_limit, percentage);
}

rcl_interfaces::msg::SetParametersResult ServerHarness::setParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  return impl_->node->set_parameters_atomically(parameters);
}

VectorFieldController * ServerHarness::controller() noexcept
{
  if (impl_->options.rotation_shim) {
    return nullptr;
  }
  return dynamic_cast<VectorFieldController *>(impl_->controller.get());
}

nav2_core::Controller * ServerHarness::activeController() noexcept
{
  return impl_->controller.get();
}

compat::NodeSharedPtr ServerHarness::node() const { return impl_->node; }

std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ServerHarness::costmap() const
{
  return impl_->costmap;
}

std::shared_ptr<tf2_ros::Buffer> ServerHarness::tf() const { return impl_->tf; }

nav2_core::GoalChecker * ServerHarness::goalChecker() noexcept { return impl_->checker.get(); }

const HarnessOptions & ServerHarness::options() const noexcept { return impl_->options; }

}  // namespace test
}  // namespace vectorfield_nav2_controller
