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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"
#include "vectorfield_core/conditioning.hpp"
#include "vectorfield_core/footprint.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/polyline_curve.hpp"
#include "vectorfield_nav2_controller/legacy_path_handler.hpp"
#include "vectorfield_nav2_controller/vectorfield_controller.hpp"
#include "vectorfield_nav2_controller/vectorfield_legacy_path_params.hpp"
#include "vectorfield_ros/conversions.hpp"
#include "vectorfield_ros/core_params.hpp"
#include "vectorfield_ros/field_visualizer.hpp"
#include "vectorfield_ros/marker_builder.hpp"
#include "vectorfield_ros/topics.hpp"

namespace vectorfield_nav2_controller
{

namespace vf = vectorfield_core;
namespace vr = vectorfield_ros;
namespace compat = vectorfield_ros::compat;

namespace
{

/// A footprint vertex moving more than this is a footprint change (plan §4).
constexpr double kFootprintChangeTolerance = 1e-3;
/// WindowScan margins behind and ahead of the evaluation offset d (plan §5.2 step 2).
constexpr double kWindowScanBehind = 0.3;
constexpr double kWindowScanAhead = 0.5;
/// Suffix of the primary controller's name under a Lyrical rotation shim.
constexpr const char * kShimPrimarySuffix = ".primary_controller";

bool endsWith(const std::string & text, const std::string & suffix)
{
  return text.size() >= suffix.size() &&
         text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

/// The zero command in the base frame.
geometry_msgs::msg::TwistStamped zeroCommand(
  const std::string & base_frame, const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = base_frame;
  command.header.stamp = stamp;
  return command;
}

}  // namespace

struct VectorFieldController::Impl
{
  /// @name Node-side handles (set in configure, released in cleanup)
  /// @{
  compat::NodeWeakPtr parent;
  std::string name;
  rclcpp::Logger logger{rclcpp::get_logger("VectorFieldController")};
  rclcpp::Clock::SharedPtr clock;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros;
  std::string base_frame;
  std::shared_ptr<vr::CommonParamListener> listener;
  std::unique_ptr<vr::CrossValidator> cross_validator;
  std::unique_ptr<vr::FieldVisualizer> visualizer;
  std::shared_ptr<vectorfield_legacy_path_params::ParamListener> legacy_listener;
  std::unique_ptr<LegacyPathHandler> legacy;
  bool active{false};
  /// @}

  /// @name Configuration
  /// @{
  vr::CommonParams params;
  double controller_frequency{20.0};
  double control_period{0.05};
  vf::CostmapInfo costmap_info;
  std::unique_ptr<vf::Tracker> tracker;
  std::optional<vf::FootprintChecker> checker;
  bool config_invalid{false};
  std::string config_error;
  /// Footprint last seen on the costmap (a change triggers a rebuild).
  std::vector<geometry_msgs::msg::Point> seen_footprint;
  std::vector<vf::Vec2> seen_vertices;
  std::vector<vf::Vec2> footprint_work;
  /// @}

  /// @name Shared with setSpeedLimit() and the cross-validator (guarded by `mutex`)
  /// @{
  mutable std::mutex mutex;
  double speed_limit_scale{1.0};
  double desired_speed{0.4};
  std::vector<geometry_msgs::msg::Point> applied_footprint;
  /// @}

  /// @name Control-thread state
  /// @{
  vf::TrackerState state{vf::Tracker::initialState()};
  vf::StepScratch scratch;
  vf::PolylineCurve curve;
  vf::OccupancyGrid grid;
  vf::EuclideanDistanceTransform edt;
  std::vector<std::uint8_t> mask;
  PlanIntakeState intake;
  std::string raw_frame;
  compat::EmulatedXYLatch xy_latch;
  bool cancelling{false};
  std::optional<double> last_compute_time;
  /// Gen A/B: the yaw of LegacyPathHandler's transform for this cycle (plan → costmap frame).
  std::optional<double> plan_to_window_yaw;
  LegacyPathOutput legacy_out;
  geometry_msgs::msg::TwistStamped last_command;
  CycleDiagnostics diagnostics;
  /// Goal identities of the once-per-goal checks (0: none since the last reset).
  std::uint32_t computed_goal{0};
  std::uint32_t advised_goal{0};
  std::uint32_t entered_goal{0};
  /// @}

  /// @name Plan segments (math_spec §9.4 [Design, M7]; control thread)
  /// @{
  /// The last raw plan; segment windows are cut from it (`raw_poses` holds its poses).
  nav_msgs::msg::Path raw_plan;
  /// The active segment's poses, given to `legacy` in segmented mode.
  nav_msgs::msg::Path segment_plan;
  std::vector<vf::PlanSegment> segments;
  std::vector<std::size_t> segment_work;
  /// `allow_reversing` and the plan has a cusp or a segment against the configured direction.
  bool segmented{false};
  /// Gen C: the window comes from `legacy` (segmented, or an in-place rotation in the plan).
  bool own_window{false};
  std::size_t active_segment{0};
  /// Drive sign of the active segment (0 before the first plan).
  int drive_sign{0};
  /// Goal identity passed to the core: a new goal, a segment switch or a stop that moved.
  std::uint32_t core_goal_seq{0};
  /// The active stop (plan frame) and whether it is the plan's goal.
  vf::Vec2 stop{vf::Vec2::Zero()};
  bool stop_final{true};
  /// End tangent of an intermediate stop (plan frame).
  bool stop_has_tangent{false};
  vf::Vec2 stop_tangent{vf::Vec2::UnitX()};
  /// @}

  /// @name Work buffers (no allocation once their capacity suffices)
  /// @{
  std::vector<vf::Pose2> raw_poses;
  std::vector<vf::Pose2> window_poses;
  std::vector<vf::Vec2> positions;
  std::vector<vf::Vec2> decimated;
  std::vector<vf::Vec2> work;
  std::vector<vf::Vec2> vertices;
  /// @}

  double chordLength() const
  {
    return vf::chordLength(costmap_ros ? costmap_ros->getCostmap()->getResolution() : 0.05);
  }

  /// The configuration currently in force for the cross-validator (parameter-service thread).
  vr::CoreParamTarget committedTarget() const
  {
    vr::CoreParamTarget target;
    vr::applyRows(vr::kCommonCoreParams, listener->get_params(), &target);
    std::lock_guard<std::mutex> lock(mutex);
    vr::setNodeDerivedFields(controller_frequency, applied_footprint, &target.config);
    return target;
  }

  void logReport(const vf::ValidationReport & report, const char * context) const
  {
    for (const vf::ValidationIssue & warning : report.warnings) {
      RCLCPP_WARN(
        logger, "%s: %s: %s [%s]", name.c_str(), context, warning.message.c_str(),
        warning.code.c_str());
    }
  }

  void failRebuild(const std::string & reason)
  {
    if (!config_invalid || reason != config_error) {
      RCLCPP_ERROR(
        logger, "%s: configuration rejected, no command until a valid one: %s", name.c_str(),
        reason.c_str());
    }
    config_invalid = true;
    config_error = reason;
  }

  /// Installs a Tracker built from `config` (already validated) and the dependent state.
  void install(std::unique_ptr<vf::Tracker> next, const std::vector<geometry_msgs::msg::Point> & fp)
  {
    if (tracker) {
      const vf::CoreConfig & before = tracker->config();
      const vf::CoreConfig & after = next->config();
      // An episode ends when the mode or λ changes; a law change restarts in FOLLOW (plan §4).
      if (
        before.obstacles.obstacle_mode != after.obstacles.obstacle_mode ||
        before.obstacles.obstacle_contour_distance != after.obstacles.obstacle_contour_distance) {
        state.contour = vf::ContourState{};
      }
      if (before.motion.tracking_law != after.motion.tracking_law) {
        state.phase = vf::Phase::kFollow;
      }
      vf::requestLimiterReseed(&state);
    }
    tracker = std::move(next);
    checker.emplace(tracker->footprint());
    const double d = vf::evaluationOffset(tracker->config());
    curve.setSearchPolicy(
      vf::SearchPolicy::windowScan(-(d + kWindowScanBehind), d + kWindowScanAhead));
    {
      std::lock_guard<std::mutex> lock(mutex);
      applied_footprint = fp;
      desired_speed = tracker->config().field.desired_speed;
    }
    config_invalid = false;
    config_error.clear();
  }

  /// The transactional rebuild (plan §4): builds into a temporary, swaps only on success.
  void rebuild(const std::vector<geometry_msgs::msg::Point> & footprint, const char * cause)
  {
    vf::CoreConfig config;
    const vf::ValidationReport report =
      vr::validateParams(params, controller_frequency, costmap_info, footprint, &config);
    if (!report.ok()) {
      failRebuild(std::string(cause) + ": " + report.summary());
      return;
    }
    std::unique_ptr<vf::Tracker> next;
    try {
      next = std::make_unique<vf::Tracker>(config);
    } catch (const std::exception & e) {
      failRebuild(std::string(cause) + ": " + e.what());
      return;
    }
    logReport(report, cause);
    install(std::move(next), footprint);
    if (visualizer) {
      try {
        visualizer->setConfig(vr::toVizConfig(params));
      } catch (const std::exception & e) {
        RCLCPP_WARN(logger, "%s: visualization settings kept: %s", name.c_str(), e.what());
      }
    }
    RCLCPP_INFO(logger, "%s: %s applied", name.c_str(), cause);
  }

  /// Pending parameter or footprint changes (control thread, every cycle).
  void applyPendingChanges()
  {
    bool changed = false;
    const char * cause = "parameter update";
    if (listener && listener->try_update_params(params)) {
      changed = true;
    }
    if (costmap_ros) {
      std::vector<geometry_msgs::msg::Point> footprint = costmap_ros->getRobotFootprint();
      vr::toVertices(footprint, &footprint_work);
      if (vr::footprintChanged(seen_vertices, footprint_work, kFootprintChangeTolerance)) {
        seen_vertices.swap(footprint_work);
        seen_footprint = std::move(footprint);
        cause = changed ? "parameter and footprint update" : "footprint update";
        changed = true;
      }
    }
    if (changed) {
      rebuild(seen_footprint, cause);
    }
  }

  void markThrown()
  {
    last_command = zeroCommand(base_frame, last_command.header.stamp);
    diagnostics.command = last_command;
    diagnostics.threw = true;
    diagnostics.cancelling = cancelling;
    diagnostics.config_invalid = config_invalid;
  }

  void clearMarkers()
  {
    if (visualizer && active && clock && costmap_ros) {
      visualizer->clear(clock->now(), costmap_ros->getGlobalFrameID());
    }
  }

  /// The sign a plan without headings starts with (math_spec §9.4): the configured one, or its
  /// opposite when the plan starts behind the robot in that direction (RPP's allow_reversing
  /// rule; Smac also emits identity orientations along +-x). Too short a plan keeps the active
  /// sign; no robot pose keeps the configured one.
  int startSign(int configured)
  {
    const vf::Vec2 & start = raw_poses.front().p;
    const vf::Vec2 * ahead = nullptr;
    for (const vf::Pose2 & pose : raw_poses) {
      if ((pose.p - start).norm() >= chordLength()) {
        ahead = &pose.p;
        break;
      }
    }
    if (ahead == nullptr) {
      return drive_sign != 0 ? drive_sign : configured;
    }
    geometry_msgs::msg::PoseStamped robot;
    if (!costmap_ros || !costmap_ros->getRobotPose(robot)) {
      return configured;
    }
    vf::Isometry2 plan_from_robot_frame = vf::Isometry2::identity();
    if (!raw_frame.empty() && raw_frame != robot.header.frame_id) {
      try {
        plan_from_robot_frame = vr::toIsometry2(
          tf->lookupTransform(raw_frame, robot.header.frame_id, tf2::TimePointZero).transform);
      } catch (const tf2::TransformException &) {
        return configured;
      }
    }
    const double yaw = plan_from_robot_frame.apply(vr::toPose2(robot.pose)).yaw;
    return (*ahead - start).dot(configured * vf::heading(yaw)) >= 0.0 ? configured : -configured;
  }

  /// The active segment ends at a cusp (an intermediate stop), not at the plan's goal.
  bool cuspStop() const { return segmented && active_segment + 1 < segments.size(); }

  /// Makes `segments[active_segment]` active: the stop and its identity, the drive sign, the
  /// stop's end tangent and the window source (math_spec §9.4).
  void activateSegment(bool same_goal)
  {
    const vf::PlanSegment & segment = segments[active_segment];
    const bool final = active_segment + 1 == segments.size();
    const vf::Vec2 & next_stop = raw_poses[segment.last].p;
    const double hysteresis = 2.0 * chordLength();
    const bool same_stop = same_goal && segment.drive_sign == drive_sign && final == stop_final &&
                           (final || (next_stop - stop).norm() < hysteresis);
    if (!same_stop) {
      ++core_goal_seq;
    }
    stop = next_stop;
    stop_final = final;
    drive_sign = segment.drive_sign;
    stop_has_tangent = false;
    if (!final) {
      positions.clear();
      for (std::size_t k = segment.first; k <= segment.last; ++k) {
        positions.push_back(raw_poses[k].p);
      }
      const vf::EndTangent tangent = vf::endTangent(positions, chordLength());
      stop_has_tangent = tangent.valid;
      stop_tangent = tangent.valid ? tangent.tangent : vf::Vec2::UnitX();
    }
    if (!segmented) {
      if (!VF_HAS_PATH_HANDLER || own_window) {
        legacy->setPlan(raw_plan);
      }
      return;
    }
    segment_plan.header = raw_plan.header;
    const auto first = raw_plan.poses.begin() + static_cast<std::ptrdiff_t>(segment.first);
    const auto last = raw_plan.poses.begin() + static_cast<std::ptrdiff_t>(segment.last) + 1;
    segment_plan.poses.assign(first, last);
    legacy->setPlan(segment_plan);
  }

  /// The next segment after the robot settled at a cusp.
  void switchSegment()
  {
    ++active_segment;
    intake.goal_in_window = false;
    xy_latch.rearm();
    activateSegment(false);
  }

  void resetTracking()
  {
    cancelling = false;
    segments.clear();
    segmented = false;
    own_window = false;
    active_segment = 0;
    drive_sign = 0;
    core_goal_seq = 0;
    stop_final = true;
    state = vf::Tracker::initialState();
    intake = PlanIntakeState{};
    xy_latch.rearm();
    last_compute_time.reset();
    last_command = zeroCommand(base_frame, last_command.header.stamp);
    computed_goal = 0;
    advised_goal = 0;
    entered_goal = 0;
  }
};

VectorFieldController::VectorFieldController() : impl_(std::make_unique<Impl>()) {}

VectorFieldController::~VectorFieldController() = default;

void VectorFieldController::configure(
  const compat::NodeWeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  Impl & s = *impl_;
  const compat::NodeSharedPtr node = parent.lock();
  if (!node) {
    throw std::runtime_error("VectorFieldController::configure: the parent node is gone");
  }
  s.parent = parent;
  s.name = std::move(name);
  s.logger = node->get_logger();
  s.clock = node->get_clock();
  s.tf = std::move(tf);
  s.costmap_ros = std::move(costmap_ros);
  s.base_frame = s.costmap_ros->getBaseFrameID();
  const auto parameters = node->get_node_parameters_interface();

  // Window limit of the costmap rules: LegacyPathHandler's prune_distance on Gen A/B.
  std::optional<double> window_limit;
  if (!VF_HAS_PATH_HANDLER) {
    s.legacy_listener =
      std::make_shared<vectorfield_legacy_path_params::ParamListener>(parameters, s.logger, s.name);
    const vectorfield_legacy_path_params::Params legacy = s.legacy_listener->get_params();
    LegacyPathConfig legacy_config;
    legacy_config.prune_distance = legacy.prune_distance;
    legacy_config.max_robot_pose_search_dist = legacy.max_robot_pose_search_dist;
    legacy_config.transform_tolerance = legacy.transform_tolerance;
    legacy_config.reject_unit_path = legacy.reject_unit_path;
    s.legacy = std::make_unique<LegacyPathHandler>(legacy_config, s.tf, s.costmap_ros);
    window_limit = legacy_config.prune_distance;
  } else {
    // Gen C: the plugin's own window for segmented plans and in-place rotations (math_spec
    // §9.4), with FeasiblePathHandler's defaults.
    s.legacy = std::make_unique<LegacyPathHandler>(LegacyPathConfig{}, s.tf, s.costmap_ros);
  }

  s.listener = std::make_shared<vr::CommonParamListener>(parameters, s.logger, s.name);
  s.params = s.listener->get_params();
  s.controller_frequency = vr::controllerFrequencyOf(*parameters);
  s.control_period = 1.0 / s.controller_frequency;
  s.costmap_info = vr::toCostmapInfo(*s.costmap_ros->getCostmap(), window_limit);
  s.seen_footprint = s.costmap_ros->getRobotFootprint();
  vr::toVertices(s.seen_footprint, &s.seen_vertices);

  vf::CoreConfig config;
  const vf::ValidationReport report =
    vr::validateFromNode(parameters, s.logger, s.name, s.costmap_info, s.seen_footprint, &config);
  if (!report.ok()) {
    throw vf::ConfigError(s.name + ": invalid configuration: " + report.summary());
  }
  s.logReport(report, "configuration");
  s.tracker.reset();
  s.install(std::make_unique<vf::Tracker>(config), s.seen_footprint);
  s.state = vf::Tracker::initialState();

  // Registered after the listeners: rclcpp runs it first (plan §4).
  std::vector<vr::ParamSetter> setters;
  vr::appendSetters(vr::kCommonCoreParams, &setters);
  Impl * impl = impl_.get();
  s.cross_validator = std::make_unique<vr::CrossValidator>(
    parameters, s.name, std::move(setters), s.costmap_info,
    [impl]() { return impl->committedTarget(); }, s.logger);

  s.visualizer = std::make_unique<vr::FieldVisualizer>(
    node, vr::topics::pluginNamespace(s.name), vr::toVizConfig(s.params));

  // Work buffers for the costmap's size.
  const nav2_costmap_2d::Costmap2D & costmap = *s.costmap_ros->getCostmap();
  const auto width = static_cast<int>(costmap.getSizeInCellsX());
  const auto height = static_cast<int>(costmap.getSizeInCellsY());
  s.grid.reserve(width, height);
  s.edt.reserve(width, height);
  s.mask.reserve(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));

  // Rotation shim with contour (plan §5.3): supported only on Jazzy/Kilted with
  // rotate_to_heading_once.
  const bool shim =
    endsWith(s.name, kShimPrimarySuffix) || node->has_parameter(s.name + kShimPrimarySuffix);
  if (shim && s.tracker->config().obstacles.obstacle_mode == vf::ObstacleMode::kContour) {
    bool once = false;
    const std::string once_key = s.name + ".rotate_to_heading_once";
    if (compat::kApiGeneration == 'B' && node->has_parameter(once_key)) {
      const rclcpp::Parameter value = node->get_parameter(once_key);
      once = value.get_type() == rclcpp::ParameterType::PARAMETER_BOOL && value.as_bool();
    }
    if (!once) {
      RCLCPP_WARN(
        s.logger,
        "%s: the rotation shim re-aims at the path whenever it re-arms, which undoes obstacle "
        "contour episodes; use it with obstacle_mode contour only on Jazzy/Kilted with "
        "rotate_to_heading_once: true",
        s.name.c_str());
    }
  }

  s.resetTracking();
  RCLCPP_INFO(
    s.logger, "%s: vector-field controller configured (%s, %s, %s)", s.name.c_str(),
    compat::kApiGenerationName, vf::toString(s.tracker->config().motion.motion_model),
    vf::toString(s.tracker->config().motion.tracking_law));
}

void VectorFieldController::cleanup()
{
  Impl & s = *impl_;
  s.cross_validator.reset();
  s.listener.reset();
  s.legacy_listener.reset();
  s.legacy.reset();
  s.visualizer.reset();
  s.checker.reset();
  s.tracker.reset();
  s.active = false;
  s.resetTracking();
}

void VectorFieldController::activate()
{
  Impl & s = *impl_;
  if (s.visualizer) {
    s.visualizer->on_activate();
  }
  s.active = true;
}

void VectorFieldController::deactivate()
{
  Impl & s = *impl_;
  if (s.visualizer && s.active) {
    s.visualizer->on_deactivate(s.clock->now(), s.costmap_ros->getGlobalFrameID());
  }
  s.active = false;
}

#if VF_HAS_PATH_HANDLER  // Gen C

void VectorFieldController::newPathReceived(const nav_msgs::msg::Path & raw_global_path)
{
  processRawPlan(raw_global_path);
}

geometry_msgs::msg::TwistStamped VectorFieldController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker, const nav_msgs::msg::Path & transformed_global_plan,
  const geometry_msgs::msg::PoseStamped & global_goal)
{
  Impl & s = *impl_;
  try {
    if (s.own_window) {
      // A segmented plan or an in-place rotation: the plugin's window (math_spec §9.4), never
      // the server's truncated or overlapping one.
      s.legacy->computeWindow(pose, &s.legacy_out);
      s.plan_to_window_yaw = vr::toIsometry2(s.legacy_out.costmap_from_plan.transform).yaw();
      return computeCommandImpl(
        pose, velocity, goal_checker, s.legacy_out.window, s.legacy_out.goal);
    }
    s.plan_to_window_yaw.reset();
    return computeCommandImpl(pose, velocity, goal_checker, transformed_global_plan, global_goal);
  } catch (...) {
    impl_->markThrown();
    throw;
  }
}

#else  // Gen A/B

void VectorFieldController::setPlan(const nav_msgs::msg::Path & path)
{
  Impl & s = *impl_;
  // Humble: the server never calls reset(); a goal that starts after an idle gap gets one here.
  if (!compat::kServerCallsReset) {
    const double since =
      s.last_compute_time && s.clock ? s.clock->now().seconds() - *s.last_compute_time : vf::kNaN;
    if (resetDue(since, s.control_period)) {
      s.resetTracking();
    }
  }
  processRawPlan(path);  // also gives LegacyPathHandler the plan or the active segment
  s.xy_latch.rearm();
}

geometry_msgs::msg::TwistStamped VectorFieldController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  Impl & s = *impl_;
  try {
    s.legacy->computeWindow(pose, &s.legacy_out);
    s.plan_to_window_yaw = vr::toIsometry2(s.legacy_out.costmap_from_plan.transform).yaw();
    return computeCommandImpl(pose, velocity, goal_checker, s.legacy_out.window, s.legacy_out.goal);
  } catch (...) {
    s.markThrown();
    throw;
  }
}

#endif

void VectorFieldController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  Impl & s = *impl_;
  std::lock_guard<std::mutex> lock(s.mutex);
  s.speed_limit_scale = speedLimitScale(speed_limit, percentage, s.desired_speed);
}

bool VectorFieldController::cancel()
{
  Impl & s = *impl_;
  s.cancelling = true;
  const geometry_msgs::msg::Twist & last = s.last_command.twist;
  return std::abs(last.linear.x) < vf::kStopLinearSpeed &&
         std::abs(last.linear.y) < vf::kStopLinearSpeed &&
         std::abs(last.angular.z) < vf::kStopAngularSpeed;
}

void VectorFieldController::reset()
{
  Impl & s = *impl_;
  s.resetTracking();
  s.clearMarkers();
}

CycleDiagnostics VectorFieldController::lastCycle() const { return impl_->diagnostics; }

vf::CoreConfig VectorFieldController::appliedConfig() const
{
  return impl_->tracker ? impl_->tracker->config() : vf::CoreConfig{};
}

geometry_msgs::msg::TwistStamped VectorFieldController::computeCommandImpl(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker, const nav_msgs::msg::Path & window,
  const geometry_msgs::msg::PoseStamped & goal)
{
  Impl & s = *impl_;
  const auto wall_start = std::chrono::steady_clock::now();
  if (!s.tracker) {
    throw std::logic_error("VectorFieldController: computeVelocityCommands before configure()");
  }
  CycleDiagnostics & diag = s.diagnostics;
  const bool previous_threw = diag.threw;
  diag.threw = false;
  s.last_command.header.stamp = pose.header.stamp;
  if (window.poses.empty()) {
    throw compat::InvalidPath("vector field: the plan window is empty");
  }

  // Time: the node clock; the nominal period after a gap or at the first cycle.
  const double now = s.clock->now().seconds();
  if (s.last_compute_time && now == *s.last_compute_time && !previous_threw) {
    // No time passed since the last cycle (a simulated /clock slower than the controller, e.g.
    // Nav2's 10 Hz loopback clock at 20 Hz): the robot and the world are unchanged, so return
    // the committed command instead of stepping the limiter by a period that did not elapse.
    return s.last_command;
  }
  double dt = s.control_period;
  if (s.last_compute_time) {
    const double elapsed = now - *s.last_compute_time;
    if (elapsed > 0.0 && elapsed <= kResetIdlePeriods * s.control_period) {
      dt = elapsed;
    }
  }
  s.last_compute_time = now;

  // Costmap snapshot → OccupancyGrid → EDT (rebuilt only when a site changed).
  std::optional<vf::ObstacleMap> obstacles;
  if (
    vr::toOccupancyGrid(
      *s.costmap_ros->getCostmap(), s.tracker->config().obstacles.unknown_is_obstacle, &s.mask,
      &s.grid)) {
    s.edt.update(s.grid);
    obstacles = vf::ObstacleMap{&s.grid, &s.edt, vf::Isometry2::identity()};
  }

  // Pending parameter or footprint changes: transactional rebuild.
  s.applyPendingChanges();
  if (s.config_invalid && !s.cancelling) {
    throw compat::NoValidControl("vector field: parameters rejected: " + s.config_error);
  }
  const vf::Tracker & tracker = *s.tracker;
  const vf::CoreConfig & config = tracker.config();

  // Window conditioning (plan §5.2 step 1).
  vr::toPoses(window, &s.window_poses);
  const vf::Pose2 base = vr::toPose2(pose.pose);
  const vf::Pose2 goal_pose = vr::toPose2(goal.pose);
  WindowInput window_input;
  window_input.window = s.window_poses;
  window_input.goal = goal_pose;
  window_input.base = base.p;
  // An intermediate stop ends with its segment's tangent (math_spec §9.4).
  const bool cusp_stop = s.cuspStop();
  window_input.has_end_tangent = cusp_stop ? s.stop_has_tangent : s.intake.has_end_tangent;
  window_input.chord_length = s.chordLength();
  const bool tail_needed =
    window_input.has_end_tangent &&
    (s.intake.goal_in_window ||
     (s.window_poses.back().p - goal_pose.p).norm() < vf::kWindowAtGoalDistance);
  if (tail_needed) {
    double yaw = 0.0;
    if (s.plan_to_window_yaw) {
      yaw = *s.plan_to_window_yaw;
    } else if (!s.raw_frame.empty() && s.raw_frame != window.header.frame_id) {
      try {
        const geometry_msgs::msg::TransformStamped transform = s.tf->lookupTransform(
          window.header.frame_id, s.raw_frame, rclcpp::Time(window.header.stamp));
        yaw = vr::toIsometry2(transform.transform).yaw();
      } catch (const tf2::TransformException & e) {
        throw compat::ControllerTFError(
          std::string("vector field: no transform from the plan frame to the window frame: ") +
          e.what());
      }
    }
    window_input.end_tangent = vf::rotate(yaw, cusp_stop ? s.stop_tangent : s.intake.end_tangent);
  }
  s.plan_to_window_yaw.reset();
  const WindowBuild build = buildWindow(window_input, &s.intake, &s.work, &s.vertices);
  if (s.curve.assign(s.vertices, false) != vf::AssignStatus::kOk) {
    throw compat::InvalidPath("vector field: the plan window has no usable segment");
  }

  // Once per goal: the plan advisories of the stability advisor. kappa_path is measured on the
  // chord-decimated window: raw NavFn vertices half a cell apart turn by up to 45° each, a grid
  // staircase that reads as tens of 1/m the robot never follows. `decimated` is intake scratch,
  // refilled on every plan.
  if (s.advised_goal != s.intake.goal_seq) {
    s.advised_goal = s.intake.goal_seq;
    vf::decimateChords(s.vertices, s.chordLength(), &s.decimated);
    s.logReport(vf::adviseForPlan(config, vf::kappaPath(s.decimated, false)), "plan");
  }

  // Goal, tolerances and XY reached as Nav2 sees it.
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  const vr::GoalTolerances tolerances =
    compat::get_goal_tolerances(goal_checker, pose_tolerance, velocity_tolerance)
      ? vr::toGoalTolerances(pose_tolerance)
      : vr::GoalTolerances{};
  vf::GoalInfo goal_info;
  goal_info.pose = goal_pose;
  goal_info.has_yaw = true;
  goal_info.xy_tol = tolerances.xy;
  goal_info.yaw_tol = tolerances.yaw;
  if (cusp_stop) {
    // A cusp: no yaw, the cusp tolerance, the core's XY latch (math_spec §9.4).
    goal_info.has_yaw = false;
    goal_info.xy_tol = 2.0 * config.phases.cusp_tolerance;
  } else {
    goal_info.xy_reached = compat::goal_xy_reached(
      goal_checker, s.xy_latch, pose, goal, velocity, window, tolerances.xy);
  }

  // A footprint that already collides gets no command (plan §5.2 step 5).
  if (
    obstacles && config.safety.use_collision_detection && !s.cancelling &&
    s.checker->collides(base, s.grid, s.edt)) {
    throw compat::NoValidControl("vector field: the robot footprint is in collision");
  }

  vf::TrackerInput input;
  input.pose = base;
  input.measured = vr::toTwist2(velocity);
  input.t = now;
  input.dt = dt;
  input.curve = &s.curve;
  input.goal_seq = s.core_goal_seq;
  input.drive_sign = static_cast<std::int8_t>(s.segmented ? s.drive_sign : 0);
  input.goal = goal_info;
  input.obstacles = obstacles;
  {
    std::lock_guard<std::mutex> lock(s.mutex);
    input.speed_limit_scale = s.speed_limit_scale;
  }
  input.cancel = s.cancelling;
  input.cap_at_curve_end = true;

  const vf::TrackerOutput output = tracker.step(input, s.state, &s.state, s.scratch);

  // Commit before acting (plan §3.1).
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = s.base_frame;
  command.header.stamp = pose.header.stamp;
  command.twist = vr::toTwistMsg(output.cmd);
  s.last_command = command;
  diag.output = output;
  diag.state = s.state;
  diag.goal_seq = s.intake.goal_seq;
  diag.command = command;
  diag.cancelling = s.cancelling;
  diag.config_invalid = s.config_invalid;
  diag.window_kind = build.kind;
  diag.virtual_vertex = build.virtual_vertex;
  diag.window_vertices = s.vertices.size();
  diag.window_end_tangent =
    (s.vertices[s.vertices.size() - 1] - s.vertices[s.vertices.size() - 2]).normalized();
  diag.speed_limit_scale = input.speed_limit_scale;
  diag.segment = s.active_segment;
  diag.segment_count = std::max<std::size_t>(1, s.segments.size());
  diag.drive_sign = input.drive_sign;
  diag.own_window = !VF_HAS_PATH_HANDLER || s.own_window;
  diag.cusp_stop = cusp_stop;

  // The goal entered the window too late to brake on the terminal field: checked once per goal,
  // when the goal first enters a window after the goal's first cycle (a goal that starts within
  // the braking distance is no truncated window).
  const bool first_cycle_of_goal = s.computed_goal != s.intake.goal_seq;
  s.computed_goal = s.intake.goal_seq;
  if (build.goal_entered && s.entered_goal != s.intake.goal_seq) {
    s.entered_goal = s.intake.goal_seq;
    if (
      !first_cycle_of_goal && std::isfinite(output.field.sigma_end) &&
      output.field.sigma_end < config.phases.terminal_braking_distance) {
      RCLCPP_WARN(
        s.logger,
        "%s: the goal entered the window %.2f m ahead, less than terminal_braking_distance %.2f m "
        "(a larger costmap or prune_distance lets the robot brake on the terminal field)",
        s.name.c_str(), output.field.sigma_end, config.phases.terminal_braking_distance);
    }
  }

  // Visualization (built only for subscribed topics).
  if (s.visualizer && s.active) {
    vr::MarkerContext context;
    context.frame_id = window.header.frame_id;
    context.stamp = pose.header.stamp;
    context.tracker = &tracker;
    context.input = &input;
    context.output = &output;
    context.state = &s.state;
    context.scratch = &s.scratch;
    context.window = s.vertices;
    context.virtual_vertex = build.virtual_vertex;
    context.compute_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall_start)
        .count();
    s.visualizer->publish(context);
    diag.visualizer_builder_calls = s.visualizer->builderCalls();
  }

  // Settled at a cusp: the next cycle drives the next segment (math_spec §9.4).
  if (cusp_stop && output.settled && !s.cancelling) {
    s.switchSegment();
  }

  // Map the status (never while cancelling).
  const std::string reason = noValidControlReason(output.status, s.cancelling);
  if (!reason.empty()) {
    throw compat::NoValidControl(reason);
  }
  return command;
}

void VectorFieldController::processRawPlan(const nav_msgs::msg::Path & raw_plan)
{
  Impl & s = *impl_;
  vr::toPoses(raw_plan, &s.raw_poses);
  const PlanIntake intake =
    intakePlan(s.raw_poses, s.chordLength(), &s.intake, &s.positions, &s.decimated);
  if (intake.empty) {
    throw compat::InvalidPath("vector field: received an empty plan");
  }
  s.raw_frame = raw_plan.header.frame_id;
  const vf::CoreConfig & config = s.tracker->config();
  if (intake.cusp && !config.motion.allow_reversing) {
    throw compat::InvalidPath("plan contains reversing segments; use a forward-only planner");
  }

  // Plan segments (math_spec §9.4): split at cusps, each with its drive sign; one forward
  // segment unless allow_reversing and the plan has a cusp or a segment against the configured
  // direction (omni: no drive direction, only cusps).
  const vf::Kinematics & kinematics = s.tracker->kinematics();
  const int configured = kinematics.isHolonomic() ? 1 : static_cast<int>(kinematics.driveSign());
  const int start_sign = config.motion.allow_reversing && !kinematics.isHolonomic()
                           ? s.startSign(configured)
                           : configured;
  vf::segmentPlan(s.raw_poses, s.chordLength(), start_sign, &s.segment_work, &s.segments);
  const bool against = std::any_of(
    s.segments.begin(), s.segments.end(),
    [configured](const vf::PlanSegment & segment) { return segment.drive_sign != configured; });
  const bool was_segmented = s.segmented;
  s.segmented = config.motion.allow_reversing &&
                (s.segments.size() > 1 || (against && !kinematics.isHolonomic()));
  if (!s.segmented) {
    vf::PlanSegment whole;
    whole.last = s.raw_poses.size() - 1;
    whole.drive_sign = static_cast<std::int8_t>(configured);
    s.segments.assign(1, whole);
  }
  s.own_window = s.segmented || vf::hasInPlaceRotation(s.raw_poses);
  s.raw_plan = raw_plan;
  // The replan hysteresis applies to a moved goal too (a preemption while the robot moves).
  s.active_segment = was_segmented && s.segmented
                       ? vf::firstActiveSegment(s.segments, s.drive_sign, 2.0 * s.chordLength())
                       : 0;
  s.activateSegment(!intake.new_goal);
  if (intake.new_goal) {
    // The once-per-goal advisories of the raw plan.
    if (intake.sparse) {
      RCLCPP_WARN(
        s.logger,
        "%s: sparse plan: a segment is %.2f m long (more than %.2f m); the window follows the "
        "planner's poses, so the path between them is a straight chord",
        s.name.c_str(), intake.max_segment, kSparseSegmentLength);
    }
    if (intake.unsmoothed && !s.segmented) {  // a cusp is no corner
      RCLCPP_WARN(
        s.logger,
        "%s: unsmoothed plan: a corner turns by %.0f deg; a smoother (or a smoothing planner) "
        "gives a steadier field",
        s.name.c_str(), intake.max_chord_turn * 180.0 / vf::kPi);
    }
  }
}

}  // namespace vectorfield_nav2_controller

PLUGINLIB_EXPORT_CLASS(vectorfield_nav2_controller::VectorFieldController, nav2_core::Controller)
