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

#include "vectorfield_curve_follower/curve_follower.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/tracker.hpp"
#include "vectorfield_curve_follower/follower_params.hpp"
#include "vectorfield_curve_follower/frame_twist_estimator.hpp"
#include "vectorfield_curve_follower/goal_setup.hpp"
#include "vectorfield_curve_follower/lap_counter.hpp"
#include "vectorfield_msgs/msg/curve.hpp"
#include "vectorfield_ros/conversions.hpp"
#include "vectorfield_ros/core_params.hpp"
#include "vectorfield_ros/field_visualizer.hpp"
#include "vectorfield_ros/marker_builder.hpp"
#include "vectorfield_ros/topics.hpp"

namespace vectorfield_curve_follower
{

namespace vf = vectorfield_core;
namespace vr = vectorfield_ros;
namespace compat = vectorfield_ros::compat;

using FollowCurve = vectorfield_msgs::action::FollowCurve;

static_assert(result_code::kUnknown == FollowCurve::Result::UNKNOWN, "FollowCurve.action");
static_assert(
  result_code::kInvalidCurve == FollowCurve::Result::INVALID_CURVE, "FollowCurve.action");
static_assert(result_code::kUnsupported == FollowCurve::Result::UNSUPPORTED, "FollowCurve.action");
static_assert(result_code::kTfError == FollowCurve::Result::TF_ERROR, "FollowCurve.action");
static_assert(result_code::kStaleInput == FollowCurve::Result::STALE_INPUT, "FollowCurve.action");
static_assert(result_code::kBlocked == FollowCurve::Result::BLOCKED, "FollowCurve.action");
static_assert(result_code::kTimeout == FollowCurve::Result::TIMEOUT, "FollowCurve.action");
static_assert(
  result_code::kAssumption3Violated == FollowCurve::Result::ASSUMPTION3_VIOLATED,
  "FollowCurve.action");
static_assert(result_code::kNav2Active == FollowCurve::Result::NAV2_ACTIVE, "FollowCurve.action");
static_assert(result_code::kInvalidGoal == FollowCurve::Result::INVALID_GOAL, "FollowCurve.action");

namespace
{

/// A published_footprint vertex moving more than this is a footprint change (plan §4).
constexpr double kFootprintChangeTolerance = 0.01;
/// The action whose status publisher proves that the interlock sees Nav2 (plan §6).
constexpr const char * kInterlockAction = "follow_path";
/// Suffix of an action's status topic.
constexpr const char * kStatusSuffix = "/_action/status";
/// Grace of SimpleActionServer::deactivate() before it terminates the goals itself.
constexpr std::chrono::milliseconds kServerTimeout{500};
/// A measured Δt longer than this many periods (a stall) is replaced by the nominal period.
constexpr double kMaxDtPeriods = 5.0;
/// Minimum Δt of a TF differencing pair (s).
constexpr double kFrameMinDt = 0.01;
/// Lower bound of the speed-limit scale (as the controller plugin).
constexpr double kMinSpeedLimitScale = 0.01;
/// Polling slice of the loop's wait under simulated time.
constexpr std::chrono::milliseconds kSimTimePoll{1};

double toSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

builtin_interfaces::msg::Time toTimeMsg(double seconds)
{
  return rclcpp::Time(static_cast<std::int64_t>(std::llround(seconds * 1e9)), RCL_ROS_TIME);
}

/// `TrackerInput::speed_limit_scale` of a `nav2_msgs/SpeedLimit` (plugin semantics: 0, negative
/// and non-finite mean no limit; percentage → limit/100; absolute → limit/desired_speed).
double speedLimitScale(double speed_limit, bool percentage, double desired_speed)
{
  if (!std::isfinite(speed_limit) || speed_limit <= 0.0) {
    return 1.0;
  }
  double scale = 1.0;
  if (percentage) {
    scale = speed_limit / 100.0;
  } else if (desired_speed > 0.0 && std::isfinite(desired_speed)) {
    scale = speed_limit / desired_speed;
  }
  return std::clamp(scale, kMinSpeedLimitScale, 1.0);
}

/// An action is active if any goal is ACCEPTED, EXECUTING or CANCELING.
bool anyGoalActive(const action_msgs::msg::GoalStatusArray & msg)
{
  using action_msgs::msg::GoalStatus;
  return std::any_of(msg.status_list.begin(), msg.status_list.end(), [](const GoalStatus & g) {
    return g.status == GoalStatus::STATUS_ACCEPTED || g.status == GoalStatus::STATUS_EXECUTING ||
           g.status == GoalStatus::STATUS_CANCELING;
  });
}

bool isZero(const vf::Twist2 & twist) noexcept
{
  return twist.vx == 0.0 && twist.vy == 0.0 && twist.wz == 0.0;
}

/// A message with the node-clock time it was received.
template <typename MessageT>
struct Snapshot
{
  std::shared_ptr<const MessageT> msg;
  double received{0.0};
};

/// Per-goal state (goal context; the TrackerState is in here too).
struct GoalRun
{
  PreparedGoal goal;
  /// Serial number (CycleSnapshot::goal).
  std::uint64_t serial{0};
  std::uint32_t goal_seq{0};
  double start_time{0.0};
  std::unique_ptr<vf::Tracker> tracker;
  vf::TrackerState state{vf::Tracker::initialState()};
  bool rebuild{true};
  bool config_invalid{false};
  std::string config_error;
  LapCounter laps;
  GoalSupervisor supervisor;
  FrameTwistEstimator frame;
  bool has_published{false};
  std::size_t published{0};
  vf::Twist2 last_cmd;
  std::optional<double> last_cycle_time;
  std::optional<vf::Vec2> last_position;
  double distance{0.0};
  std::optional<SupervisorDecision> terminating;
  std::shared_ptr<FollowCurve::Feedback> feedback{std::make_shared<FollowCurve::Feedback>()};
};

}  // namespace

struct CurveFollower::Impl
{
  explicit Impl(CurveFollower & owner) : node(owner), logger(owner.get_logger()) {}

  CurveFollower & node;
  rclcpp::Logger logger;
  rclcpp::Clock::SharedPtr clock;

  /// @name Configure-time handles (released in cleanup)
  /// @{
  std::shared_ptr<vr::CommonParamListener> common_listener;
  std::shared_ptr<FollowerParamListener> follower_listener;
  std::unique_ptr<vr::CrossValidator> cross_validator;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<compat::TwistPublisher> cmd_pub;
  rclcpp_lifecycle::LifecyclePublisher<vectorfield_msgs::msg::Curve>::SharedPtr curve_pub;
  std::unique_ptr<vr::FieldVisualizer> visualizer;
  std::shared_ptr<compat::SimpleActionServer<FollowCurve>> action_server;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub;
  std::vector<rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> status_subs;
  std::vector<std::string> status_actions;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frame_odom_sub;
  std::string frame_odom_topic;
  /// @}

  /// The diagnostics (created with the node, so they report every lifecycle state).
  std::unique_ptr<diagnostic_updater::Updater> diagnostics;

  /// @name Parameters (control thread; the cross-validator reads the listeners)
  /// @{
  vr::CommonParams common_params;
  FollowerParams follower_params;
  /// @}

  /// @name Shared with the subscription callbacks and the cross-validator (guarded by `mutex`)
  /// @{
  mutable std::mutex mutex;
  Snapshot<nav_msgs::msg::Odometry> odom;
  Snapshot<nav2_msgs::msg::SpeedLimit> speed_limit;
  Snapshot<nav2_msgs::msg::Costmap> costmap;
  Snapshot<geometry_msgs::msg::PolygonStamped> footprint;
  Snapshot<nav_msgs::msg::Odometry> frame_odom;
  std::vector<std::uint8_t> action_active;
  std::vector<geometry_msgs::msg::Point> applied_footprint;
  /// @}

  /// @name Loop wake-up
  /// @{
  std::mutex wait_mutex;
  std::condition_variable wake;
  std::atomic<bool> deactivating{false};
  /// @}

  /// @name Control-thread state that outlives goals (work buffers)
  /// @{
  std::uint32_t goal_counter{0};
  std::uint64_t goals_taken{0};
  vf::StepScratch scratch;
  vf::OccupancyGrid grid;
  vf::EuclideanDistanceTransform edt;
  std::vector<std::uint8_t> mask;
  std::shared_ptr<const nav2_msgs::msg::Costmap> used_costmap;
  bool grid_valid{false};
  std::string costmap_frame;
  vf::CostmapInfo costmap_info;
  std::optional<vf::CostmapInfo> warned_costmap;
  std::shared_ptr<const geometry_msgs::msg::PolygonStamped> used_footprint;
  std::vector<geometry_msgs::msg::Point> robot_footprint;
  std::vector<vf::Vec2> footprint_vertices;
  std::vector<vf::Vec2> footprint_work;
  bool has_footprint{false};
  /// @}

  /// @name Snapshot of the last cycle (guarded by `diag_mutex`)
  /// @{
  mutable std::mutex diag_mutex;
  CycleSnapshot snapshot;
  /// @}

  // --- Configuration -----------------------------------------------------------------------

  /// The configuration in force for the cross-validator (parameter-service thread).
  vr::CoreParamTarget committedTarget() const
  {
    std::vector<geometry_msgs::msg::Point> footprint_copy;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      footprint_copy = applied_footprint;
    }
    vr::CoreParamTarget target;
    toFollowerTarget(
      common_listener->get_params(), follower_listener->get_params(), footprint_copy, &target);
    return target;
  }

  SupervisorConfig supervisorConfig(double max_duration) const
  {
    SupervisorConfig config;
    config.stale_abort_timeout = follower_params.stale_abort_timeout;
    config.blocked_timeout = follower_params.blocked_timeout;
    config.progress_radius = follower_params.progress_radius;
    config.progress_timeout = follower_params.progress_timeout;
    config.assumption3_abort_time = follower_params.assumption3_abort_time;
    config.max_duration = max_duration;
    return config;
  }

  FrameTwistConfig frameConfig() const
  {
    FrameTwistConfig config;
    config.min_dt = kFrameMinDt;
    config.max_dt = follower_params.input_timeout;
    config.cutoff_hz = follower_params.frame_twist_cutoff_hz;
    config.max_speed = follower_params.frame_twist_max_speed;
    config.input_timeout = follower_params.input_timeout;
    return config;
  }

  double period() const { return 1.0 / follower_params.controller_frequency; }

  /// Pending parameter changes (control thread). Returns true if any value changed.
  bool updateParameters()
  {
    bool changed = false;
    if (common_listener && common_listener->try_update_params(common_params)) {
      changed = true;
      used_costmap.reset();  // unknown_is_obstacle may have changed
      if (visualizer) {
        try {
          visualizer->setConfig(vr::toVizConfig(common_params));
        } catch (const std::exception & e) {
          RCLCPP_WARN(logger, "visualization settings kept: %s", e.what());
        }
      }
    }
    if (follower_listener && follower_listener->try_update_params(follower_params)) {
      changed = true;
    }
    return changed;
  }

  /// The goal's configuration from the current parameters, footprint and the goal's overrides.
  bool goalConfig(const GoalRun & run, vf::CoreConfig * config, std::string * error) const
  {
    vr::CoreParamTarget target;
    std::string failed;
    if (!toFollowerTarget(common_params, follower_params, robot_footprint, &target, &failed)) {
      *error = "parameter '" + failed + "' has a value vectorfield_core cannot use";
      return false;
    }
    *config = goalScopedConfig(target.config, run.goal.desired_speed, run.goal.reverse_direction);
    return true;
  }

  /// The transactional rebuild (plan §4): builds into a temporary and installs it only on
  /// success. A failure marks the goal's configuration invalid (the goal aborts UNKNOWN).
  void buildTracker(GoalRun & run, const char * cause)
  {
    run.rebuild = false;
    vf::CoreConfig config;
    std::string error;
    std::unique_ptr<vf::Tracker> next;
    if (goalConfig(run, &config, &error)) {
      const vf::ValidationReport report = vf::validateConfig(config, std::nullopt);
      if (report.ok()) {
        try {
          next = std::make_unique<vf::Tracker>(config);
        } catch (const std::exception & e) {
          error = e.what();
        }
      } else {
        error = report.summary();
      }
    }
    if (!next) {
      if (!run.config_invalid || error != run.config_error) {
        RCLCPP_ERROR(logger, "%s: configuration rejected: %s", cause, error.c_str());
      }
      run.config_invalid = true;
      run.config_error = error;
      return;
    }
    // Costmap rules: warn once per costmap geometry (plan §4).
    if (
      grid_valid && (!warned_costmap || warned_costmap->width_m != costmap_info.width_m ||
                     warned_costmap->height_m != costmap_info.height_m ||
                     warned_costmap->resolution != costmap_info.resolution)) {
      warned_costmap = costmap_info;
      const vf::ValidationReport with_costmap = vf::validateConfig(config, costmap_info);
      for (const vf::ValidationIssue & issue : with_costmap.errors) {
        RCLCPP_WARN(
          logger, "local costmap %.2f x %.2f m: %s [%s]", costmap_info.width_m,
          costmap_info.height_m, issue.message.c_str(), issue.code.c_str());
      }
      for (const vf::ValidationIssue & issue : with_costmap.warnings) {
        RCLCPP_WARN(logger, "%s [%s]", issue.message.c_str(), issue.code.c_str());
      }
    }
    if (run.tracker) {
      const vf::CoreConfig & before = run.tracker->config();
      const vf::CoreConfig & after = next->config();
      // An episode ends when the mode or λ changes (the configs are resolved, so a footprint
      // change that moves the auto λ counts); a law change restarts in FOLLOW (plan §4).
      if (
        before.obstacles.obstacle_mode != after.obstacles.obstacle_mode ||
        before.obstacles.obstacle_contour_distance != after.obstacles.obstacle_contour_distance) {
        run.state.contour = vf::ContourState{};
      }
      if (before.motion.tracking_law != after.motion.tracking_law) {
        run.state.phase = vf::Phase::kFollow;
      }
    }
    vf::requestLimiterReseed(&run.state);
    run.tracker = std::move(next);
    run.config_invalid = false;
    run.config_error.clear();
    {
      const std::lock_guard<std::mutex> lock(mutex);
      applied_footprint = robot_footprint;
    }
    RCLCPP_DEBUG(logger, "%s: tracker built", cause);
  }

  // --- Inputs ------------------------------------------------------------------------------

  /// Latest transform target ← source as an isometry; false if unavailable.
  bool lookup(
    const std::string & target, const std::string & source, vf::Isometry2 * out,
    double * stamp = nullptr) const
  {
    if (target == source) {
      *out = vf::Isometry2::identity();
      if (stamp != nullptr) {
        *stamp = 0.0;
      }
      return true;
    }
    try {
      const geometry_msgs::msg::TransformStamped transform =
        tf->lookupTransform(target, source, tf2::TimePointZero);
      *out = vr::toIsometry2(transform.transform);
      if (stamp != nullptr) {
        *stamp = toSeconds(transform.header.stamp);
      }
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  /// A new costmap_raw → OccupancyGrid → EDT (rebuilt only when a site changed).
  void refreshCostmap()
  {
    std::shared_ptr<const nav2_msgs::msg::Costmap> msg;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      msg = costmap.msg;
    }
    if (!msg || msg == used_costmap) {
      return;
    }
    used_costmap = msg;
    if (vr::toOccupancyGrid(*msg, common_params.unknown_is_obstacle, &mask, &grid)) {
      edt.update(grid);
      grid_valid = true;
      costmap_frame = msg->header.frame_id;
      costmap_info = vr::toCostmapInfo(msg->metadata, std::nullopt);
    } else {
      RCLCPP_WARN_THROTTLE(
        logger, *clock, 1000, "local_costmap/costmap_raw rejected (inconsistent or rotated)");
    }
  }

  /// A new published_footprint, transformed into the base frame at its stamp. Returns true if
  /// the robot-frame footprint changed (by more than 1 cm).
  bool refreshFootprint()
  {
    std::shared_ptr<const geometry_msgs::msg::PolygonStamped> msg;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      msg = footprint.msg;
    }
    if (!msg || msg == used_footprint) {
      return false;
    }
    const std::string & base = follower_params.robot_base_frame;
    const std::string & frame = msg->header.frame_id;
    vf::Isometry2 T_base_fp = vf::Isometry2::identity();
    if (!frame.empty() && frame != base) {
      try {
        T_base_fp = vr::toIsometry2(
          tf->lookupTransform(base, frame, rclcpp::Time(msg->header.stamp)).transform);
      } catch (const tf2::TransformException &) {
        // The newest transform, if it is within transform_tolerance of the stamp.
        double stamp = 0.0;
        if (
          !lookup(base, frame, &T_base_fp, &stamp) ||
          (stamp != 0.0 &&
           std::abs(stamp - toSeconds(msg->header.stamp)) > follower_params.transform_tolerance)) {
          return false;  // retried with the next message
        }
      }
    }
    used_footprint = msg;
    std::vector<geometry_msgs::msg::Point> polygon;
    polygon.reserve(msg->polygon.points.size());
    for (const geometry_msgs::msg::Point32 & p : msg->polygon.points) {
      polygon.push_back(vr::toPointMsg(T_base_fp.apply(vr::toVec2(p))));
    }
    vr::toVertices(polygon, &footprint_work);
    if (
      has_footprint &&
      !vr::footprintChanged(footprint_vertices, footprint_work, kFootprintChangeTolerance)) {
      return false;
    }
    footprint_vertices.swap(footprint_work);
    robot_footprint = std::move(polygon);
    has_footprint = true;
    return true;
  }

  /// Subscribes to a moving frame's Odometry topic (or drops the subscription for "").
  void setFrameOdomTopic(const std::string & topic)
  {
    if (topic == frame_odom_topic && (topic.empty() || frame_odom_sub)) {
      return;
    }
    frame_odom_sub.reset();
    frame_odom_topic = topic;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      frame_odom = Snapshot<nav_msgs::msg::Odometry>{};
    }
    if (topic.empty()) {
      return;
    }
    frame_odom_sub = compat::subscribe<nav_msgs::msg::Odometry>(
      &node, topic, rclcpp::SystemDefaultsQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const double received = clock->now().seconds();
        const std::lock_guard<std::mutex> lock(mutex);
        frame_odom = Snapshot<nav_msgs::msg::Odometry>{std::move(msg), received};
      });
  }

  bool nav2Active(std::string * which = nullptr) const
  {
    const std::lock_guard<std::mutex> lock(mutex);
    for (std::size_t i = 0; i < action_active.size(); ++i) {
      if (action_active[i] != 0U) {
        if (which != nullptr) {
          *which = status_actions[i];
        }
        return true;
      }
    }
    return false;
  }

  // --- Commands ----------------------------------------------------------------------------

  void publish(GoalRun & run, const vf::Twist2 & command)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.frame_id = follower_params.robot_base_frame;
    msg->header.stamp = clock->now();
    msg->twist = vr::toTwistMsg(command);
    cmd_pub->publish(std::move(msg));
    run.has_published = true;
    ++run.published;
    run.last_cmd = command;
    const std::lock_guard<std::mutex> lock(diag_mutex);
    ++snapshot.commands_sent;
  }

  /// One zero if this goal (or the goal it preempted) published anything.
  void stopNow(GoalRun & run)
  {
    if (run.has_published) {
      publish(run, vf::Twist2{});
    }
  }

  /// One step of the deceleration ramp; publishes nothing if nothing was published.
  vf::Twist2 decelerate(GoalRun & run, double dt)
  {
    if (!run.has_published) {
      return vf::Twist2{};
    }
    double linear = common_params.max_linear_decel;
    double angular = common_params.max_angular_accel;
    if (run.tracker) {
      linear = run.tracker->config().motion.max_linear_decel;
      angular = run.tracker->config().motion.max_angular_accel;
    }
    const vf::Twist2 command = rampDown(run.last_cmd, dt, linear, angular);
    publish(run, command);
    return command;
  }

  // --- Goals -------------------------------------------------------------------------------

  /// Validates a goal and sets up its run (plan §6): nullopt, or why it must be terminated
  /// before any command.
  std::optional<GoalRejection> startGoal(
    const FollowCurve::Goal & goal, const GoalRun * previous, GoalRun * run)
  {
    const double now = clock->now().seconds();
    updateParameters();
    GoalSetupContext context;
    vr::CoreParamTarget target;
    std::string failed;
    if (!toFollowerTarget(common_params, follower_params, robot_footprint, &target, &failed)) {
      return GoalRejection{
        result_code::kUnknown,
        "parameter '" + failed + "' has a value vectorfield_core cannot use"};
    }
    context.config = target.config;
    context.curve_options = target.curve_options;
    context.global_frame = follower_params.global_frame;
    context.moving_frame_reference = follower_params.moving_frame_reference;
    context.now = now;
    if (auto rejection = prepareGoal(goal, context, &run->goal)) {
      return rejection;
    }

    // Interlock liveness and mutual exclusion (plan §6).
    if (follower_params.require_nav2_interlock) {
      for (std::size_t i = 0; i < status_actions.size(); ++i) {
        if (status_actions[i] == kInterlockAction) {
          const std::string topic = status_subs[i]->get_topic_name();
          if (node.count_publishers(topic) == 0) {
            return GoalRejection{
              result_code::kUnsupported,
              "Nav2 interlock cannot see " + topic + "; check namespace/remaps"};
          }
        }
      }
    }
    std::string active;
    if (nav2Active(&active)) {
      return GoalRejection{result_code::kNav2Active, "the Nav2 action " + active + " is active"};
    }
    if (run->goal.moving) {
      try {
        setFrameOdomTopic(run->goal.frame_odom_topic);
      } catch (const std::exception & e) {
        return GoalRejection{
          result_code::kUnsupported,
          "cannot subscribe to frame_odom_topic '" + run->goal.frame_odom_topic + "': " + e.what()};
      }
    } else {
      setFrameOdomTopic("");
    }

    // The run. A preempted run hands over its motion; its TrackerState only for an identical
    // spec and identical overrides.
    if (previous != nullptr && keepsTrackerState(previous->goal, run->goal)) {
      run->state = previous->state;
      run->goal_seq = previous->goal_seq;
    } else {
      run->state = vf::Tracker::initialState();
      if (++goal_counter == 0U) {
        goal_counter = 1U;
      }
      run->goal_seq = goal_counter;
    }
    if (previous != nullptr) {
      run->has_published = previous->has_published;
      run->last_cmd = previous->last_cmd;
    }
    run->start_time = now;
    run->rebuild = true;
    run->laps.reset(
      run->goal.reverse_direction ? -1 : 1, run->goal.laps, run->goal.curve->isClosed());
    run->supervisor.reset(supervisorConfig(run->goal.max_duration), now);
    run->frame.setConfig(frameConfig());
    run->frame.reset();

    // ~/curve (transient local).
    std_msgs::msg::Header header;
    header.frame_id = run->goal.frame_id;
    header.stamp = toTimeMsg(run->goal.spec.t0);
    curve_pub->publish(vr::toCurveMsg(run->goal.spec, header));
    if (cmd_pub->get_subscription_count() == 0) {
      RCLCPP_WARN(
        logger,
        "cmd_vel_nav has no subscriber: the commands reach no robot (it must feed the "
        "velocity smoother or collision_monitor, never the base topic)");
    }
    RCLCPP_INFO(
      logger, "following a %s %s curve (%.2f m, laps %u, %s)",
      run->goal.curve->isClosed() ? "closed" : "open",
      run->goal.curve->isTimeVarying() ? "time-varying" : "static", run->goal.curve->length(now),
      static_cast<unsigned>(run->goal.laps), run->goal.moving ? "moving frame" : "static frame");
    return std::nullopt;
  }

  /// The curve frame this cycle (plan §6 "Frames").
  struct FrameState
  {
    vf::CurveFrame frame;
    bool ok{false};
    bool stale{false};
    std::optional<GoalRejection> unsupported;
  };

  FrameState curveFrame(GoalRun & run, double now)
  {
    FrameState out;
    const std::string & global = follower_params.global_frame;
    if (!run.goal.moving) {
      out.ok = lookup(global, run.goal.frame_id, &out.frame.T_eval_curve);
      return out;
    }
    out.frame.moving = true;
    std::string reference = follower_params.moving_frame_reference;
    if (!run.goal.frame_odom_topic.empty()) {
      Snapshot<nav_msgs::msg::Odometry> sample;
      {
        const std::lock_guard<std::mutex> lock(mutex);
        sample = frame_odom;
      }
      if (sample.msg) {
        if (sample.msg->child_frame_id != run.goal.frame_id) {
          out.unsupported = GoalRejection{
            result_code::kUnsupported, "frame_odom_topic child_frame_id '" +
                                         sample.msg->child_frame_id + "' is not the curve frame '" +
                                         run.goal.frame_id + "'"};
          return out;
        }
        reference = sample.msg->header.frame_id;
        run.frame.addOdometry(
          toSeconds(sample.msg->header.stamp), vr::toPose2(sample.msg->pose.pose),
          vr::toTwist2(sample.msg->twist.twist));
      }
      if (!sample.msg || now - sample.received > follower_params.input_timeout) {
        out.stale = true;
      }
    } else {
      vf::Isometry2 T_ref_frame;
      double stamp = 0.0;
      if (lookup(reference, run.goal.frame_id, &T_ref_frame, &stamp)) {
        run.frame.addPose(stamp, T_ref_frame.toPose());
      }
    }
    const FrameEstimate estimate = run.frame.estimate(now);
    out.stale = out.stale || estimate.stale;
    vf::Isometry2 T_global_ref;
    if (reference.empty() || !lookup(global, reference, &T_global_ref)) {
      // The frame cannot be placed: a stale input until the first estimate, then an
      // untransformable curve frame (TF_ERROR after stale_abort_timeout).
      out.ok = !estimate.ready;
      out.stale = true;
      return out;
    }
    out.ok = true;
    out.frame.T_eval_curve = T_global_ref * vf::Isometry2::fromPose(estimate.pose);
    out.frame.twist_in_curve = estimate.stale ? vf::Twist2{} : estimate.twist;
    return out;
  }

  /// Ends the goal: result, markers, logs (the final zero was already published).
  void finish(GoalRun & run, std::uint16_t code, const std::string & reason, bool succeeded)
  {
    const double now = clock->now().seconds();
    auto result = std::make_shared<FollowCurve::Result>();
    result->error_code = code;
    result->error_msg = reason;
    result->laps_completed = run.laps.lapsCompleted();
    result->distance_travelled = run.distance;
    result->total_elapsed_time =
      rclcpp::Duration::from_seconds(std::max(0.0, now - run.start_time));
    if (visualizer) {
      visualizer->clear(clock->now(), follower_params.global_frame);
    }
    setFrameOdomTopic("");
    if (succeeded) {
      RCLCPP_INFO(
        logger, "curve goal succeeded (%u laps, %.2f m)",
        static_cast<unsigned>(result->laps_completed), result->distance_travelled);
      action_server->succeeded_current(result);
    } else if (code == result_code::kNone) {
      RCLCPP_INFO(logger, "curve goal canceled");
      action_server->terminate_current(result);
    } else {
      RCLCPP_WARN(
        logger, "curve goal aborted (%u): %s", static_cast<unsigned>(code), reason.c_str());
      action_server->terminate_current(result);
    }
    const std::lock_guard<std::mutex> lock(diag_mutex);
    snapshot.goal = run.serial;
    snapshot.executing = false;
    snapshot.last_result = code;
    snapshot.last_reason = reason;
    snapshot.commands_published = run.published;
    snapshot.laps_completed = result->laps_completed;
    snapshot.progress = run.laps.progress();
  }

  void reject(const GoalRejection & rejection, std::uint64_t serial)
  {
    auto result = std::make_shared<FollowCurve::Result>();
    result->error_code = rejection.code;
    result->error_msg = rejection.message;
    RCLCPP_WARN(
      logger, "curve goal refused (%u): %s", static_cast<unsigned>(rejection.code),
      rejection.message.c_str());
    action_server->terminate_current(result);
    const std::lock_guard<std::mutex> lock(diag_mutex);
    snapshot.goal = serial;
    snapshot.executing = false;
    snapshot.last_result = rejection.code;
    snapshot.last_reason = rejection.message;
    snapshot.commands_published = 0;
  }

  /// The snapshot of a cycle that returned before the Tracker step (a decelerating termination,
  /// an unsupported frame source): the cycle count and the commands it published.
  void countCycle(const GoalRun & run, bool ended)
  {
    const std::lock_guard<std::mutex> lock(diag_mutex);
    ++snapshot.cycles;
    snapshot.goal = run.serial;
    snapshot.executing = !ended;
    snapshot.command = run.last_cmd;
    snapshot.commands_published = run.published;
  }

  /// One control cycle. Returns true when the goal ended.
  bool cycle(GoalRun & run)
  {
    const auto wall_start = std::chrono::steady_clock::now();
    const rclcpp::Time stamp = clock->now();
    const double now = stamp.seconds();
    double dt = period();
    if (run.last_cycle_time) {
      const double elapsed = now - *run.last_cycle_time;
      if (elapsed > 0.0 && elapsed <= kMaxDtPeriods * period()) {
        dt = elapsed;
      }
    }
    run.last_cycle_time = now;

    // A decelerating termination: ramp to zero (the last ramp step is the one zero).
    if (run.terminating) {
      const SupervisorDecision ending = *run.terminating;
      bool ended = true;
      if (deactivating || nav2Active() || !run.has_published) {
        stopNow(run);
        finish(run, ending.code, ending.reason, false);
      } else if (isZero(decelerate(run, dt))) {
        finish(run, ending.code, ending.reason, false);
      } else {
        ended = false;
      }
      countCycle(run, ended);
      return ended;
    }

    // Parameters and the footprint: transactional rebuild.
    if (updateParameters()) {
      run.rebuild = true;
      run.supervisor.setConfig(supervisorConfig(run.goal.max_duration));
      run.frame.setConfig(frameConfig());
    }
    refreshCostmap();
    if (refreshFootprint()) {
      run.rebuild = true;
    }
    if ((run.rebuild || !run.tracker) && has_footprint && grid_valid) {
      buildTracker(run, run.tracker ? "parameter or footprint update" : "goal start");
    }

    // Inputs.
    const char * stale_reason = nullptr;
    vf::Isometry2 T_global_base;
    double pose_stamp = 0.0;
    const bool have_pose = lookup(
      follower_params.global_frame, follower_params.robot_base_frame, &T_global_base, &pose_stamp);
    Snapshot<nav_msgs::msg::Odometry> odom_now;
    Snapshot<nav2_msgs::msg::Costmap> costmap_now;
    Snapshot<nav2_msgs::msg::SpeedLimit> limit_now;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      odom_now = odom;
      costmap_now = costmap;
      limit_now = speed_limit;
    }
    const double input_timeout = follower_params.input_timeout;
    vf::Isometry2 T_costmap_global;
    if (!have_pose || (pose_stamp != 0.0 && now - pose_stamp > input_timeout)) {
      stale_reason = "the robot pose is stale or unavailable";
    } else if (!odom_now.msg || now - odom_now.received > input_timeout) {
      stale_reason = "the odometry is stale or missing";
    } else if (!grid_valid || !costmap_now.msg) {
      stale_reason = "no local costmap received yet";
    } else if (now - costmap_now.received > follower_params.costmap_timeout) {
      stale_reason = "the local costmap stalled (gap > costmap_timeout)";
    } else if (!has_footprint) {
      stale_reason = "no published footprint received yet";
    } else if (!run.tracker && !run.config_invalid) {
      stale_reason = "the tracker is not built yet";
    } else if (!lookup(costmap_frame, follower_params.global_frame, &T_costmap_global)) {
      stale_reason = "the local costmap frame is not transformable";
    }
    const vf::Pose2 pose = T_global_base.toPose();
    if (have_pose) {
      if (run.last_position) {
        run.distance += (pose.p - *run.last_position).norm();
      }
      run.last_position = pose.p;
    }
    FrameState frame = curveFrame(run, now);
    if (frame.unsupported) {
      stopNow(run);
      finish(run, frame.unsupported->code, frame.unsupported->message, false);
      countCycle(run, true);
      return true;
    }
    const bool nav2_active = nav2Active();
    const bool cancel = action_server->is_cancel_requested();

    // The Tracker.
    const double desired_speed =
      run.tracker ? run.tracker->config().field.desired_speed : common_params.desired_speed;
    const double limit_scale =
      limit_now.msg
        ? speedLimitScale(limit_now.msg->speed_limit, limit_now.msg->percentage, desired_speed)
        : 1.0;
    vf::TrackerInput input;
    vf::TrackerOutput output;
    bool ran = false;
    if (
      run.tracker && !run.config_invalid && stale_reason == nullptr && frame.ok && !frame.stale &&
      !deactivating && !nav2_active) {
      const vf::Curve & curve = *run.goal.curve;
      input.pose = pose;
      input.measured = vr::toTwist2(odom_now.msg->twist.twist);
      input.t = now;
      input.dt = dt;
      input.curve = &curve;
      input.frame = frame.frame;
      input.goal_seq = run.goal_seq;
      if (!curve.isClosed()) {
        vf::GoalInfo goal;
        const double s_goal =
          run.goal.reverse_direction ? curve.startStation() : curve.endStation(now);
        goal.pose.p = frame.frame.T_eval_curve.apply(curve.pointAt(s_goal, now));
        goal.has_yaw = false;
        goal.xy_tol = follower_params.goal_tolerance;
        goal.yaw_tol = 0.0;
        input.goal = goal;
      }
      input.obstacles = vf::ObstacleMap{&grid, &edt, T_costmap_global};
      input.speed_limit_scale = limit_scale;
      input.cancel = cancel;
      input.cap_at_curve_end = false;
      output = run.tracker->step(input, run.state, &run.state, scratch);
      ran = true;
      // Commit the goal context before acting on the status (plan §3.1).
      run.laps.update(output.field.s_star, curve.length(now));
    } else {
      vf::requestLimiterReseed(&run.state);
    }

    SupervisorInput supervision;
    supervision.now = now;
    supervision.cancel_requested = cancel;
    supervision.deactivating = deactivating;
    supervision.nav2_active = nav2_active;
    supervision.config_invalid = run.config_invalid;
    supervision.curve_frame_ok = frame.ok;
    supervision.inputs_stale = stale_reason != nullptr;
    supervision.frame_stale = frame.stale;
    supervision.tracker_ran = ran;
    supervision.status = ran ? output.status : vf::Status::kNone;
    supervision.position = pose.p;
    supervision.nominal_speed = desired_speed * limit_scale;
    const SupervisorDecision decision = run.supervisor.update(supervision);

    bool ended = false;
    switch (decision.action) {
      case SupervisorAction::kContinue:
        if (ran) {
          const bool done = run.goal.curve->isClosed() ? run.laps.lapsDone() : output.settled;
          if (done) {
            stopNow(run);
            finish(run, result_code::kNone, "", true);
            ended = true;
          } else {
            publish(run, output.cmd);
          }
        }
        break;
      case SupervisorAction::kDecelerate:
        RCLCPP_WARN_THROTTLE(
          logger, *clock, 1000, "decelerating: %s",
          stale_reason != nullptr ? stale_reason : decision.reason);
        decelerate(run, dt);
        break;
      case SupervisorAction::kZero:
        RCLCPP_WARN_THROTTLE(logger, *clock, 1000, "zero command: %s", decision.reason);
        stopNow(run);
        break;
      case SupervisorAction::kCancel:
        if (ran) {
          if (output.cancel_complete || belowStopThresholds(output.cmd)) {
            stopNow(run);
            finish(run, result_code::kNone, "canceled", false);
            ended = true;
          } else {
            publish(run, output.cmd);
          }
        } else if (isZero(decelerate(run, dt))) {
          finish(run, result_code::kNone, "canceled", false);
          ended = true;
        }
        break;
      case SupervisorAction::kAbort:
        if (decision.decelerate && run.has_published && !isZero(run.last_cmd)) {
          run.terminating = decision;
          if (isZero(decelerate(run, dt))) {
            finish(run, decision.code, decision.reason, false);
            ended = true;
          }
        } else {
          stopNow(run);
          finish(
            run, decision.code,
            run.config_invalid ? decision.reason + std::string(": ") + run.config_error
                               : std::string(decision.reason),
            false);
          ended = true;
        }
        break;
    }

    // Feedback and visualization of a cycle that ran the Tracker.
    if (ran && !ended) {
      FollowCurve::Feedback & feedback = *run.feedback;
      feedback.distance_to_curve = output.field.D;
      feedback.arc_length = output.field.s_star;
      feedback.remaining_arc_length = run.laps.remainingArcLength();
      feedback.lap = run.laps.lapsCompleted();
      feedback.speed =
        std::hypot(odom_now.msg->twist.twist.linear.x, odom_now.msg->twist.twist.linear.y);
      feedback.phase = vf::toUnderlying(output.phase);
      action_server->publish_feedback(run.feedback);
    }
    if (ran && visualizer) {
      vr::CurveInfo info;
      info.direction = run.goal.reverse_direction ? -1 : 1;
      info.laps_completed = run.laps.lapsCompleted();
      info.laps = run.goal.laps;
      info.remaining_arc_length = run.laps.remainingArcLength();
      info.frame_moving = run.goal.moving;
      info.frame_origin = frame.frame.T_eval_curve.translation();
      info.frame_velocity = frame.frame.T_eval_curve.rotate(
        vf::Vec2(frame.frame.twist_in_curve.vx, frame.frame.twist_in_curve.vy));
      vr::MarkerContext context;
      context.frame_id = follower_params.global_frame;
      context.stamp = stamp;
      context.tracker = run.tracker.get();
      context.input = &input;
      context.output = &output;
      context.state = &run.state;
      context.scratch = &scratch;
      context.curve_info = &info;
      context.compute_time_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall_start)
          .count();
      visualizer->publish(context);
    }

    const std::lock_guard<std::mutex> lock(diag_mutex);
    ++snapshot.cycles;
    snapshot.goal = run.serial;
    snapshot.executing = !ended;
    snapshot.tracker_ran = ran;
    snapshot.decision = decision;
    if (ran) {
      snapshot.status = output.status;
      snapshot.phase = output.phase;
    }
    snapshot.command = run.last_cmd;
    snapshot.commands_published = run.published;
    snapshot.laps_completed = run.laps.lapsCompleted();
    snapshot.progress = run.laps.progress();
    snapshot.config_invalid = run.config_invalid;
    return ended;
  }

  /// Waits on the node's clock until `deadline`; false if woken by deactivation or shutdown.
  bool waitUntil(const rclcpp::Time & deadline)
  {
    const auto context = node.get_node_base_interface()->get_context();
    std::unique_lock<std::mutex> lock(wait_mutex);
    while (rclcpp::ok(context) && !deactivating) {
      const rclcpp::Time now = clock->now();
      if (now >= deadline) {
        return true;
      }
      auto remaining = std::chrono::nanoseconds((deadline - now).nanoseconds());
      if (clock->ros_time_is_active()) {
        remaining = std::min<std::chrono::nanoseconds>(remaining, kSimTimePoll);
      }
      wake.wait_for(lock, remaining);
    }
    return false;
  }

  void release()
  {
    action_server.reset();
    status_subs.clear();
    status_actions.clear();
    frame_odom_sub.reset();
    frame_odom_topic.clear();
    odom_sub.reset();
    speed_limit_sub.reset();
    costmap_sub.reset();
    footprint_sub.reset();
    visualizer.reset();
    curve_pub.reset();
    cmd_pub.reset();
    tf_listener.reset();
    tf.reset();
    cross_validator.reset();
    follower_listener.reset();
    common_listener.reset();
    const std::lock_guard<std::mutex> lock(mutex);
    odom = {};
    speed_limit = {};
    costmap = {};
    footprint = {};
    frame_odom = {};
    action_active.clear();
    applied_footprint.clear();
    used_costmap.reset();
    used_footprint.reset();
    grid_valid = false;
    has_footprint = false;
    robot_footprint.clear();
    footprint_vertices.clear();
    warned_costmap.reset();
  }

  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status) const
  {
    CycleSnapshot copy;
    {
      const std::lock_guard<std::mutex> lock(diag_mutex);
      copy = snapshot;
    }
    using diagnostic_msgs::msg::DiagnosticStatus;
    if (copy.config_invalid) {
      status.summary(DiagnosticStatus::ERROR, "configuration rejected");
    } else if (copy.executing && copy.decision.action != SupervisorAction::kContinue) {
      status.summary(DiagnosticStatus::WARN, copy.decision.reason);
    } else if (copy.executing) {
      status.summary(DiagnosticStatus::OK, "following a curve");
    } else {
      status.summary(DiagnosticStatus::OK, "idle");
    }
    status.add("lifecycle_state", node.get_current_state().label());
    status.add("executing", copy.executing);
    status.add("cycles", copy.cycles);
    status.add("laps_completed", copy.laps_completed);
    status.add("commands_published", copy.commands_published);
    status.add("last_result", copy.last_result);
    status.add("last_reason", copy.last_reason);
  }
};

CurveFollower::CurveFollower(const rclcpp::NodeOptions & options)
: compat::LifecycleNode("curve_follower", "", options), impl_(std::make_unique<Impl>(*this))
{
  impl_->clock = get_clock();
  impl_->diagnostics = std::make_unique<diagnostic_updater::Updater>(this);
  impl_->diagnostics->setHardwareID("none");
  Impl * impl = impl_.get();
  impl_->diagnostics->add(
    "curve_follower",
    [impl](diagnostic_updater::DiagnosticStatusWrapper & status) { impl->diagnose(status); });
}

CurveFollower::~CurveFollower()
{
  Impl & s = *impl_;
  s.deactivating = true;
  s.wake.notify_all();
  if (s.action_server) {
    s.action_server->deactivate();
  }
  s.diagnostics.reset();
  s.release();
}

CycleSnapshot CurveFollower::lastCycle() const
{
  const std::lock_guard<std::mutex> lock(impl_->diag_mutex);
  return impl_->snapshot;
}

bool CurveFollower::preemptPending() const
{
  const auto server = impl_->action_server;
  return server && server->is_preempt_requested();
}

CurveFollower::CallbackReturn CurveFollower::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  Impl & s = *impl_;
  try {
    const auto parameters = get_node_parameters_interface();
    s.common_listener = std::make_shared<vr::CommonParamListener>(parameters, s.logger, "");
    s.follower_listener = std::make_shared<FollowerParamListener>(parameters, s.logger, "");
    s.common_params = s.common_listener->get_params();
    s.follower_params = s.follower_listener->get_params();

    // The node-level configuration must pass the cross-parameter rules (no costmap yet).
    vr::CoreParamTarget target;
    std::string failed;
    if (!toFollowerTarget(s.common_params, s.follower_params, {}, &target, &failed)) {
      RCLCPP_ERROR(
        s.logger, "parameter '%s' has a value vectorfield_core cannot use", failed.c_str());
      s.release();
      return CallbackReturn::FAILURE;
    }
    vf::ValidationReport report = vf::validateConfig(target.config, std::nullopt);
    const vf::ValidationReport options = vf::validateCurveOptions(target.curve_options);
    report.errors.insert(report.errors.end(), options.errors.begin(), options.errors.end());
    if (!report.ok()) {
      RCLCPP_ERROR(s.logger, "invalid configuration: %s", report.summary().c_str());
      s.release();
      return CallbackReturn::FAILURE;
    }
    for (const vf::ValidationIssue & warning : report.warnings) {
      RCLCPP_WARN(
        s.logger, "configuration: %s [%s]", warning.message.c_str(), warning.code.c_str());
    }
    Impl * impl = impl_.get();
    // Registered after the listeners: rclcpp runs it first (plan §4).
    s.cross_validator = std::make_unique<vr::CrossValidator>(
      parameters, "", followerSetters(), std::nullopt, [impl]() { return impl->committedTarget(); },
      s.logger,
      [](const vr::CoreParamTarget & candidate, vf::ValidationReport * out) {
        const vf::ValidationReport curve = vf::validateCurveOptions(candidate.curve_options);
        out->errors.insert(out->errors.end(), curve.errors.begin(), curve.errors.end());
      });

    const auto self = std::static_pointer_cast<compat::LifecycleNode>(shared_from_this());
    s.tf = compat::makeTfBuffer(self);
    s.tf_listener = compat::makeTfListener(*s.tf, self, true);
    s.cmd_pub = std::make_unique<compat::TwistPublisher>(self, "cmd_vel_nav", rclcpp::QoS(1));
    rclcpp_lifecycle::LifecycleNode & base = *this;
    s.curve_pub = base.create_publisher<vectorfield_msgs::msg::Curve>(
      vr::topics::topicName(vr::topics::kPrivateNamespace, vr::topics::kCurve),
      rclcpp::QoS(1).reliable().transient_local());
    s.visualizer = std::make_unique<vr::FieldVisualizer>(
      self, std::string(vr::topics::kPrivateNamespace), vr::toVizConfig(s.common_params));

    // Subscriptions: immutable snapshots under the mutex.
    s.odom_sub = compat::subscribe<nav_msgs::msg::Odometry>(
      this, s.follower_params.odom_topic, rclcpp::SystemDefaultsQoS(),
      [impl](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const double received = impl->clock->now().seconds();
        const std::lock_guard<std::mutex> lock(impl->mutex);
        impl->odom = Snapshot<nav_msgs::msg::Odometry>{std::move(msg), received};
      });
    // Volatile, as controller_server's: Nav2's speed_limit publishers (SpeedFilter, route_server)
    // are volatile, and a transient_local subscription matches none of them.
    s.speed_limit_sub = compat::subscribe<nav2_msgs::msg::SpeedLimit>(
      this, "speed_limit", rclcpp::QoS(10).reliable(),
      [impl](nav2_msgs::msg::SpeedLimit::ConstSharedPtr msg) {
        const double received = impl->clock->now().seconds();
        const std::lock_guard<std::mutex> lock(impl->mutex);
        impl->speed_limit = Snapshot<nav2_msgs::msg::SpeedLimit>{std::move(msg), received};
      });
    s.costmap_sub = compat::subscribe<nav2_msgs::msg::Costmap>(
      this, "local_costmap/costmap_raw", rclcpp::QoS(1).reliable(),
      [impl](nav2_msgs::msg::Costmap::ConstSharedPtr msg) {
        const double received = impl->clock->now().seconds();
        const std::lock_guard<std::mutex> lock(impl->mutex);
        impl->costmap = Snapshot<nav2_msgs::msg::Costmap>{std::move(msg), received};
      });
    s.footprint_sub = compat::subscribe<geometry_msgs::msg::PolygonStamped>(
      this, "local_costmap/published_footprint", rclcpp::QoS(1).reliable(),
      [impl](geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) {
        const double received = impl->clock->now().seconds();
        const std::lock_guard<std::mutex> lock(impl->mutex);
        impl->footprint = Snapshot<geometry_msgs::msg::PolygonStamped>{std::move(msg), received};
      });
    // The interlock: action-status QoS (reliable, transient local, depth 1).
    s.status_actions = s.follower_params.exclusive_actions;
    {
      const std::lock_guard<std::mutex> lock(s.mutex);
      s.action_active.assign(s.status_actions.size(), 0U);
    }
    for (std::size_t i = 0; i < s.status_actions.size(); ++i) {
      s.status_subs.push_back(
        compat::subscribe<action_msgs::msg::GoalStatusArray>(
          this, s.status_actions[i] + kStatusSuffix, rclcpp::QoS(1).reliable().transient_local(),
          [impl, i](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
            const bool active = anyGoalActive(*msg);
            const std::lock_guard<std::mutex> lock(impl->mutex);
            if (i < impl->action_active.size()) {
              impl->action_active[i] = active ? 1U : 0U;
            }
          }));
    }

    s.action_server = compat::makeActionServer<FollowCurve>(
      this, "follow_curve", [this]() { execute(); }, nullptr, kServerTimeout, true);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(s.logger, "configure failed: %s", e.what());
    s.release();
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(
    s.logger, "curve follower configured (%s, %s)", s.common_params.motion_model.c_str(),
    compat::kApiGenerationName);
  return CallbackReturn::SUCCESS;
}

CurveFollower::CallbackReturn CurveFollower::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  Impl & s = *impl_;
  s.deactivating = false;
  s.cmd_pub->on_activate();
  s.curve_pub->on_activate();
  s.visualizer->on_activate();
  s.action_server->activate();
  for (const auto & sub : s.status_subs) {
    RCLCPP_INFO(s.logger, "Nav2 interlock watches %s", sub->get_topic_name());
  }
  createBond();
  return CallbackReturn::SUCCESS;
}

CurveFollower::CallbackReturn CurveFollower::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  Impl & s = *impl_;
  s.deactivating = true;
  s.wake.notify_all();
  s.action_server->deactivate();  // waits for a running goal to stop (UNKNOWN, zero command)
  s.visualizer->on_deactivate(s.clock->now(), s.follower_params.global_frame);
  s.curve_pub->on_deactivate();
  s.cmd_pub->on_deactivate();
  destroyBond();
  return CallbackReturn::SUCCESS;
}

CurveFollower::CallbackReturn CurveFollower::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  impl_->release();
  impl_->deactivating = false;
  return CallbackReturn::SUCCESS;
}

CurveFollower::CallbackReturn CurveFollower::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  Impl & s = *impl_;
  s.deactivating = true;
  s.wake.notify_all();
  if (s.action_server) {
    s.action_server->deactivate();
  }
  s.release();
  return CallbackReturn::SUCCESS;
}

void CurveFollower::execute()
{
  Impl & s = *impl_;
  const std::shared_ptr<compat::SimpleActionServer<FollowCurve>> server = s.action_server;
  if (!server) {
    return;
  }
  const auto goal = server->get_current_goal();
  if (!goal) {
    return;
  }
  auto run = std::make_unique<GoalRun>();
  run->serial = ++s.goals_taken;
  if (const auto rejection = s.startGoal(*goal, nullptr, run.get())) {
    s.reject(*rejection, run->serial);
    return;
  }
  {
    const std::lock_guard<std::mutex> lock(s.diag_mutex);
    s.snapshot.executing = true;
  }
  const auto context = get_node_base_interface()->get_context();
  rclcpp::Time next = s.clock->now();
  while (rclcpp::ok(context)) {
    if (server->is_preempt_requested()) {
      const auto pending = server->accept_pending_goal();
      if (pending) {
        auto next_run = std::make_unique<GoalRun>();
        next_run->serial = ++s.goals_taken;
        if (const auto rejection = s.startGoal(*pending, run.get(), next_run.get())) {
          s.stopNow(*run);
          s.reject(*rejection, next_run->serial);
          return;
        }
        run = std::move(next_run);
      }
    }
    if (s.cycle(*run)) {
      return;
    }
    // Fixed rate on the node's clock; an overrun or a clock jump restarts the schedule.
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(s.period());
    next = next + period;
    const rclcpp::Time now = s.clock->now();
    if (next < now || next > now + period) {
      next = now;
    }
    s.waitUntil(next);
  }
  // Shutdown during a goal.
  s.stopNow(*run);
  s.finish(*run, result_code::kUnknown, "shutdown", false);
}

}  // namespace vectorfield_curve_follower

RCLCPP_COMPONENTS_REGISTER_NODE(vectorfield_curve_follower::CurveFollower)
