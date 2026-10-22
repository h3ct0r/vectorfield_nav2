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

#include "vectorfield_follower_3d/follower_3d.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vectorfield_core/d3/tracker.hpp"
#include "vectorfield_curve_follower/lap_counter.hpp"
#include "vectorfield_follower_3d/conversions.hpp"
#include "vectorfield_follower_3d/goal_setup.hpp"
#include "vectorfield_follower_3d/markers.hpp"
#include "vectorfield_follower_3d/params.hpp"
#include "vectorfield_follower_3d/voxelizer.hpp"
#include "vectorfield_msgs/msg/curve3.hpp"
#include "vectorfield_msgs/msg/field_state3.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace vectorfield_follower_3d
{

namespace vf = vectorfield_core;
namespace compat = vectorfield_ros::compat;
using vectorfield_curve_follower::GoalSupervisor;
using vectorfield_curve_follower::LapCounter;
using vectorfield_curve_follower::SupervisorAction;
using vectorfield_curve_follower::SupervisorConfig;
using vectorfield_curve_follower::SupervisorDecision;
using vectorfield_curve_follower::SupervisorInput;
namespace result_code = vectorfield_curve_follower::result_code;
using FollowCurve3 = Follower3D::FollowCurve3;
using FieldState3 = vectorfield_msgs::msg::FieldState3;

static_assert(result_code::kNone == FollowCurve3::Result::NONE, "FollowCurve3.action");
static_assert(
  result_code::kGoalRejected == FollowCurve3::Result::GOAL_REJECTED, "FollowCurve3.action");
static_assert(
  result_code::kSendGoalFailure == FollowCurve3::Result::SEND_GOAL_FAILURE, "FollowCurve3.action");
static_assert(result_code::kUnknown == FollowCurve3::Result::UNKNOWN, "FollowCurve3.action");
static_assert(
  result_code::kInvalidCurve == FollowCurve3::Result::INVALID_CURVE, "FollowCurve3.action");
static_assert(
  result_code::kUnsupported == FollowCurve3::Result::UNSUPPORTED, "FollowCurve3.action");
static_assert(result_code::kTfError == FollowCurve3::Result::TF_ERROR, "FollowCurve3.action");
static_assert(result_code::kStaleInput == FollowCurve3::Result::STALE_INPUT, "FollowCurve3.action");
static_assert(result_code::kBlocked == FollowCurve3::Result::BLOCKED, "FollowCurve3.action");
static_assert(result_code::kTimeout == FollowCurve3::Result::TIMEOUT, "FollowCurve3.action");
static_assert(
  result_code::kAssumption3Violated == FollowCurve3::Result::ASSUMPTION3_VIOLATED,
  "FollowCurve3.action");
static_assert(result_code::kNav2Active == FollowCurve3::Result::NAV2_ACTIVE, "FollowCurve3.action");
static_assert(
  result_code::kInvalidGoal == FollowCurve3::Result::INVALID_GOAL, "FollowCurve3.action");
static_assert(
  FieldState3::PHASE_FOLLOW == static_cast<std::uint8_t>(vf::Phase::kFollow), "FieldState3.msg");
static_assert(
  FieldState3::PHASE_FINAL_APPROACH == static_cast<std::uint8_t>(vf::Phase::kFinalApproach),
  "FieldState3.msg");
static_assert(
  FieldState3::PHASE_STOPPED == static_cast<std::uint8_t>(vf::Phase::kStopped), "FieldState3.msg");
static_assert(FieldState3::STATUS_BLOCKED == vf::Status::kBlocked, "FieldState3.msg");
static_assert(
  FieldState3::STATUS_ASSUMPTION3_CLAMPED == vf::Status::kAssumption3Clamped, "FieldState3.msg");

namespace
{

/// Grace of SimpleActionServer::deactivate() before it terminates the goals itself.
constexpr std::chrono::milliseconds kServerTimeout{500};
/// A measured Δt longer than this many periods (a stall) is replaced by the nominal period.
constexpr double kMaxDtPeriods = 5.0;
/// Polling slice of the loop's wait under simulated time.
constexpr std::chrono::milliseconds kSimTimePoll{1};
/// Stop thresholds of a cancel (as the planar follower).
constexpr double kStopLinear = 0.01;
/// @copydoc kStopLinear
constexpr double kStopAngular = 0.01;
/// The voxel window is rebuilt from the last cloud when the robot leaves this fraction of it.
constexpr double kVoxelRecentre = 0.25;

double toSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

builtin_interfaces::msg::Time toTimeMsg(double seconds)
{
  return rclcpp::Time(static_cast<std::int64_t>(std::llround(seconds * 1e9)), RCL_ROS_TIME);
}

bool isZero(const d3::Twist3 & u) noexcept
{
  return u.v.x() == 0.0 && u.v.y() == 0.0 && u.v.z() == 0.0 && u.wz == 0.0;
}

bool belowStop(const d3::Twist3 & u) noexcept
{
  return u.v.norm() < kStopLinear && std::abs(u.wz) < kStopAngular;
}

/// One step of the deceleration ramp: |v| and |ω| move toward zero by at most decel·dt (the
/// direction is kept); exactly zero once below the stop thresholds.
d3::Twist3 rampDown3(const d3::Twist3 & u, double dt, double decel, double yaw_accel) noexcept
{
  d3::Twist3 out;
  const double speed = u.v.norm();
  if (speed > 0.0 && std::isfinite(speed)) {
    out.v = u.v * (std::max(0.0, speed - decel * dt) / speed);
  }
  if (std::isfinite(u.wz)) {
    out.wz = std::copysign(std::max(0.0, std::abs(u.wz) - yaw_accel * dt), u.wz);
  }
  return belowStop(out) ? d3::Twist3{} : out;
}

/// A message with the node-clock time it was received.
template <typename MessageT>
struct Snapshot
{
  std::shared_ptr<const MessageT> msg;
  double received{0.0};
};

/// Per-goal state (goal context; the Tracker3State is in here too).
struct GoalRun3
{
  PreparedGoal3 goal;
  std::uint64_t serial{0};
  std::uint32_t goal_seq{0};
  double start_time{0.0};
  std::unique_ptr<d3::Tracker3> tracker;
  d3::Tracker3State state{d3::Tracker3::initialState()};
  bool rebuild{true};
  bool config_invalid{false};
  std::string config_error;
  LapCounter laps;
  GoalSupervisor supervisor;
  d3::FrameTwistEstimator3 frame;
  bool has_published{false};
  std::size_t published{0};
  d3::Twist3 last_cmd;
  double last_yaw{0.0};
  std::optional<double> last_cycle_time;
  std::optional<d3::Vec3> last_position;
  double distance{0.0};
  double last_markers{-1e300};
  std::optional<SupervisorDecision> terminating;
  std::shared_ptr<FollowCurve3::Feedback> feedback{std::make_shared<FollowCurve3::Feedback>()};
};

}  // namespace

struct Follower3D::Impl
{
  explicit Impl(Follower3D & owner) : node(owner), logger(owner.get_logger()) {}

  Follower3D & node;
  rclcpp::Logger logger;
  rclcpp::Clock::SharedPtr clock;

  /// @name Configure-time handles (released in cleanup)
  /// @{
  std::shared_ptr<ParamListener> listener;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr obstacles_check;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub;
  rclcpp_lifecycle::LifecyclePublisher<vectorfield_msgs::msg::Curve3>::SharedPtr curve_pub;
  rclcpp_lifecycle::LifecyclePublisher<FieldState3>::SharedPtr state_pub;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
  std::shared_ptr<compat::SimpleActionServer<FollowCurve3>> action_server;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frame_odom_sub;
  std::string frame_odom_topic;
  /// @}

  /// Parameters (control thread).
  Params params;

  /// @name Shared with the subscription callbacks (guarded by `mutex`)
  /// @{
  mutable std::mutex mutex;
  Snapshot<nav_msgs::msg::Odometry> odom;
  Snapshot<nav_msgs::msg::Odometry> frame_odom;
  Snapshot<sensor_msgs::msg::PointCloud2> cloud;
  /// @}

  /// @name Loop wake-up
  /// @{
  std::mutex wait_mutex;
  std::condition_variable wake;
  std::atomic<bool> deactivating{false};
  /// @}

  /// @name Control-thread state that outlives goals
  /// @{
  std::uint32_t goal_counter{0};
  std::uint64_t goals_taken{0};
  d3::ObstacleSet3 obstacles;
  std::string obstacles_error;
  std::shared_ptr<const sensor_msgs::msg::PointCloud2> used_cloud;
  std::vector<d3::Vec3> cloud_points;
  bool cloud_ok{false};
  VoxelWindow window;
  bool window_valid{false};
  std::vector<std::uint8_t> voxel_mask;
  d3::VoxelGrid3 voxel_grid;
  /// @}

  /// @name Snapshot of the last cycle (guarded by `diag_mutex`)
  /// @{
  mutable std::mutex diag_mutex;
  CycleSnapshot3 snapshot;
  /// @}

  // --- Configuration -----------------------------------------------------------------------

  double period() const { return 1.0 / params.controller_frequency; }

  bool body() const { return params.cmd_frame == "body"; }

  bool cbf() const { return params.obstacle_mode == "cbf"; }

  SupervisorConfig supervisorConfig(double max_duration) const
  {
    SupervisorConfig config;
    config.stale_abort_timeout = params.stale_abort_timeout;
    config.blocked_timeout = params.blocked_timeout;
    config.progress_radius = params.progress_radius;
    config.progress_timeout = params.progress_timeout;
    config.assumption3_abort_time = params.assumption3_abort_time;
    config.max_duration = max_duration;
    return config;
  }

  /// The primitives of the `obstacles` parameter (the voxels are kept).
  void refreshPrimitives()
  {
    std::vector<d3::Primitive3> primitives;
    obstacles_error.clear();
    if (!parsePrimitives(params.obstacles, &primitives, &obstacles_error)) {
      primitives.clear();
    }
    obstacles.clearPrimitives();
    for (const d3::Primitive3 & primitive : primitives) {
      obstacles.add(primitive);
    }
  }

  /// Pending parameter changes (control thread). Returns true if any value changed.
  bool updateParameters()
  {
    if (!listener || !listener->try_update_params(params)) {
      return false;
    }
    refreshPrimitives();
    return true;
  }

  /// Builds the goal's tracker from the current parameters; a failure marks the goal's
  /// configuration invalid (the goal aborts UNKNOWN).
  void buildTracker(GoalRun3 & run)
  {
    run.rebuild = false;
    try {
      if (!obstacles_error.empty()) {
        throw std::invalid_argument(obstacles_error);
      }
      const d3::Tracker3Config config = goalScopedConfig3(
        toTracker3Config(params), run.goal.desired_speed, run.goal.reverse_direction);
      auto tracker = std::make_unique<d3::Tracker3>(config);  // validates
      run.tracker = std::move(tracker);
      run.config_invalid = false;
      run.config_error.clear();
    } catch (const std::exception & e) {
      run.config_invalid = true;
      run.config_error = e.what();
      RCLCPP_ERROR(logger, "configuration rejected: %s", e.what());
    }
  }

  // --- Inputs ------------------------------------------------------------------------------

  /// Latest transform target ← source; false if unavailable.
  bool lookup(
    const std::string & target, const std::string & source, d3::Isometry3 * out,
    double * stamp = nullptr) const
  {
    if (target == source || source.empty()) {
      *out = d3::Isometry3::Identity();
      if (stamp != nullptr) {
        *stamp = 0.0;
      }
      return true;
    }
    try {
      const auto transform = tf->lookupTransform(target, source, tf2::TimePointZero);
      *out = toIsometry3(transform.transform);
      if (stamp != nullptr) {
        *stamp = toSeconds(transform.header.stamp);
      }
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  /// The body pose and measured twist (global_frame) of an odometry message.
  bool robotState(const nav_msgs::msg::Odometry & msg, d3::Pose3 * pose, d3::Twist3 * measured)
  {
    d3::Isometry3 T_global_odom;
    if (!lookup(params.global_frame, msg.header.frame_id, &T_global_odom)) {
      return false;
    }
    const d3::Isometry3 T = T_global_odom * toIsometry3(msg.pose.pose);
    const d3::Mat3 R = T.linear();
    pose->p = T.translation();
    pose->yaw = std::atan2(R(1, 0), R(0, 0));
    measured->v = R * toVec3(msg.twist.twist.linear);
    measured->wz = (R * toVec3(msg.twist.twist.angular)).z();
    return d3::isFinite(*pose) && d3::isFinite(*measured);
  }

  /// The voxel source this cycle: a new cloud (or a robot that left the window) re-voxelises.
  /// Returns a stale reason, or null.
  const char * refreshVoxels(const d3::Vec3 & position, double now)
  {
    if (params.voxel_cloud_topic.empty()) {
      return nullptr;
    }
    Snapshot<sensor_msgs::msg::PointCloud2> sample;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      sample = cloud;
    }
    if (sample.msg && sample.msg != used_cloud) {
      used_cloud = sample.msg;
      d3::Isometry3 T_global_cloud;
      cloud_ok = lookup(params.global_frame, sample.msg->header.frame_id, &T_global_cloud) &&
                 cloudPoints(*sample.msg, T_global_cloud, &cloud_points);
      window_valid = false;
    }
    const auto size = static_cast<int>(params.voxel_grid_size);
    const double extent = params.voxel_resolution * size;
    const d3::Vec3 centre = window.origin + d3::Vec3::Constant(0.5 * extent);
    if (
      cloud_ok &&
      (!window_valid || window.resolution != params.voxel_resolution || window.size != size ||
       (position - centre).cwiseAbs().maxCoeff() > kVoxelRecentre * extent)) {
      window = windowAround(position, params.voxel_resolution, size);
      voxelize(cloud_points, window, &voxel_mask);
      window_valid = voxel_grid.setFromMask(
        vf::Span<const std::uint8_t>(voxel_mask.data(), voxel_mask.size()), size, size, size,
        window.resolution, window.origin);
      if (window_valid) {
        obstacles.setVoxels(voxel_grid);
      }
    }
    if (!cbf()) {
      return nullptr;
    }
    if (!sample.msg) {
      return "no voxel cloud received yet";
    }
    if (!cloud_ok || !window_valid) {
      return "the voxel cloud is unusable (no x/y/z fields, or its frame is not transformable)";
    }
    if (params.voxel_timeout > 0.0 && now - sample.received > params.voxel_timeout) {
      return "the voxel cloud is stale (older than voxel_timeout)";
    }
    return nullptr;
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
      &node, topic, rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const double received = clock->now().seconds();
        const std::lock_guard<std::mutex> lock(mutex);
        frame_odom = Snapshot<nav_msgs::msg::Odometry>{std::move(msg), received};
      });
  }

  // --- Commands ----------------------------------------------------------------------------

  void publish(GoalRun3 & run, const d3::Twist3 & command)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.frame_id = body() ? params.robot_base_frame : params.global_frame;
    msg->header.stamp = clock->now();
    msg->twist = toCommandMsg(command, body(), run.last_yaw);
    cmd_pub->publish(std::move(msg));
    run.has_published = true;
    ++run.published;
    run.last_cmd = command;
    const std::lock_guard<std::mutex> lock(diag_mutex);
    ++snapshot.commands_sent;
  }

  /// One zero if this goal (or the goal it preempted) published anything.
  void stopNow(GoalRun3 & run)
  {
    if (run.has_published) {
      publish(run, d3::Twist3{});
    }
  }

  /// One step of the deceleration ramp; publishes nothing if nothing was published.
  d3::Twist3 decelerate(GoalRun3 & run, double dt)
  {
    if (!run.has_published) {
      return d3::Twist3{};
    }
    const d3::Twist3 command = rampDown3(run.last_cmd, dt, params.max_decel, params.max_yaw_accel);
    publish(run, command);
    return command;
  }

  // --- Goals -------------------------------------------------------------------------------

  /// Validates a goal and sets up its run: nullopt, or why it must be terminated before any
  /// command.
  std::optional<GoalRejection> startGoal(
    const FollowCurve3::Goal & goal, const GoalRun3 * previous, GoalRun3 * run)
  {
    const double now = clock->now().seconds();
    updateParameters();
    if (!obstacles_error.empty()) {
      return GoalRejection{result_code::kUnknown, "invalid obstacles: " + obstacles_error};
    }
    GoalSetupContext3 context;
    context.config = toTracker3Config(params);
    context.curve_options = toCurveOptions(params);
    context.global_frame = params.global_frame;
    context.moving_frame_reference = params.moving_frame_reference;
    context.now = now;
    if (auto rejection = prepareGoal3(goal, context, &run->goal)) {
      return rejection;
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
    if (previous != nullptr && keepsTrackerState3(previous->goal, run->goal)) {
      run->state = previous->state;
      run->goal_seq = previous->goal_seq;
    } else {
      run->state = d3::Tracker3::initialState();
      if (++goal_counter == 0U) {
        goal_counter = 1U;
      }
      run->goal_seq = goal_counter;
    }
    if (previous != nullptr) {
      run->has_published = previous->has_published;
      run->last_cmd = previous->last_cmd;
      run->last_yaw = previous->last_yaw;
    }
    run->start_time = now;
    run->rebuild = true;
    run->laps.reset(
      run->goal.reverse_direction ? -1 : 1, run->goal.laps, run->goal.curve->isClosed());
    run->supervisor.reset(supervisorConfig(run->goal.max_duration), now);
    run->frame.setConfig(toFrameTwistConfig(params));
    run->frame.reset();

    vectorfield_msgs::msg::Curve3 curve = toCurve3Msg(run->goal.spec);
    curve.header.frame_id = run->goal.frame_id;
    curve.header.stamp = toTimeMsg(run->goal.spec.t0);
    curve.sample_spacing = run->goal.sample_spacing;
    curve_pub->publish(curve);
    if (cmd_pub->get_subscription_count() == 0) {
      RCLCPP_WARN(
        logger, "%s has no subscriber: the commands reach no robot", cmd_pub->get_topic_name());
    }
    RCLCPP_INFO(
      logger, "following a %s %s 3D curve (%.2f m, laps %u, %s)",
      run->goal.curve->isClosed() ? "closed" : "open",
      run->goal.curve->isTimeVarying() ? "time-varying" : "static", run->goal.curve->length(now),
      static_cast<unsigned>(run->goal.laps), run->goal.moving ? "moving frame" : "static frame");
    return std::nullopt;
  }

  /// The curve frame this cycle.
  struct FrameState
  {
    d3::CurveFrame3 frame;
    bool ok{false};
    bool stale{false};
    std::optional<GoalRejection> unsupported;
  };

  FrameState curveFrame(GoalRun3 & run, double now)
  {
    FrameState out;
    const std::string & global = params.global_frame;
    if (!run.goal.moving) {
      out.ok = lookup(global, run.goal.frame_id, &out.frame.T_eval_curve);
      return out;
    }
    std::string reference = params.moving_frame_reference;
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
        d3::FrameTwist3 twist;
        twist.v = toVec3(sample.msg->twist.twist.linear);
        twist.w = toVec3(sample.msg->twist.twist.angular);
        run.frame.addTwist(
          toSeconds(sample.msg->header.stamp), toIsometry3(sample.msg->pose.pose), twist);
      }
      if (!sample.msg || now - sample.received > params.input_timeout) {
        out.stale = true;
      }
    } else {
      d3::Isometry3 T_ref_frame;
      double stamp = 0.0;
      if (lookup(reference, run.goal.frame_id, &T_ref_frame, &stamp)) {
        run.frame.addPose(stamp, T_ref_frame);
      }
    }
    const d3::FrameEstimate3 estimate = run.frame.estimate(now);
    out.stale = out.stale || estimate.stale;
    d3::Isometry3 T_global_ref;
    if (reference.empty() || !lookup(global, reference, &T_global_ref)) {
      // The frame cannot be placed: a stale input until the first estimate, then an
      // untransformable curve frame (TF_ERROR after stale_abort_timeout).
      out.ok = !estimate.ready;
      out.stale = true;
      return out;
    }
    out.ok = true;
    out.frame = estimate.curveFrame();
    out.frame.T_eval_curve = T_global_ref * estimate.pose;
    out.frame.moving = true;
    return out;
  }

  /// Ends the goal: result, markers, logs (the final zero was already published).
  void finish(GoalRun3 & run, std::uint16_t code, const std::string & reason, bool succeeded)
  {
    const double now = clock->now().seconds();
    auto result = std::make_shared<FollowCurve3::Result>();
    result->error_code = code;
    result->error_msg = reason;
    result->laps_completed = run.laps.lapsCompleted();
    result->distance_travelled = run.distance;
    result->total_elapsed_time =
      rclcpp::Duration::from_seconds(std::max(0.0, now - run.start_time));
    if (marker_pub && marker_pub->is_activated()) {
      marker_pub->publish(clearMarkers3(params.global_frame, clock->now()));
    }
    setFrameOdomTopic("");
    if (succeeded) {
      RCLCPP_INFO(
        logger, "3D curve goal succeeded (%u laps, %.2f m)",
        static_cast<unsigned>(result->laps_completed), result->distance_travelled);
      action_server->succeeded_current(result);
    } else if (code == result_code::kNone) {
      RCLCPP_INFO(logger, "3D curve goal canceled");
      action_server->terminate_current(result);
    } else {
      RCLCPP_WARN(
        logger, "3D curve goal aborted (%u): %s", static_cast<unsigned>(code), reason.c_str());
      action_server->terminate_current(result);
    }
    const std::lock_guard<std::mutex> lock(diag_mutex);
    snapshot.goal = run.serial;
    snapshot.executing = false;
    snapshot.last_result = code;
    snapshot.last_reason = reason;
    snapshot.commands_published = run.published;
    snapshot.laps_completed = result->laps_completed;
  }

  void reject(const GoalRejection & rejection, std::uint64_t serial)
  {
    auto result = std::make_shared<FollowCurve3::Result>();
    result->error_code = rejection.code;
    result->error_msg = rejection.message;
    RCLCPP_WARN(
      logger, "3D curve goal refused (%u): %s", static_cast<unsigned>(rejection.code),
      rejection.message.c_str());
    action_server->terminate_current(result);
    const std::lock_guard<std::mutex> lock(diag_mutex);
    snapshot.goal = serial;
    snapshot.executing = false;
    snapshot.last_result = rejection.code;
    snapshot.last_reason = rejection.message;
    snapshot.commands_published = 0;
  }

  /// The snapshot of a cycle that returned before the tracker step.
  void countCycle(const GoalRun3 & run, bool ended)
  {
    const std::lock_guard<std::mutex> lock(diag_mutex);
    ++snapshot.cycles;
    snapshot.goal = run.serial;
    snapshot.executing = !ended;
    snapshot.command = run.last_cmd;
    snapshot.commands_published = run.published;
  }

  void publishState(
    const GoalRun3 & run, const rclcpp::Time & stamp, const d3::Pose3 & pose,
    const d3::Tracker3Output & output, double compute_ms)
  {
    FieldState3 msg;
    msg.header.frame_id = params.global_frame;
    msg.header.stamp = stamp;
    msg.phase = static_cast<std::uint8_t>(output.phase);
    msg.status = output.status;
    msg.position = toPointMsg(pose.p);
    msg.yaw = pose.yaw;
    msg.desired_yaw = output.psi_d;
    msg.closest_point = toPointMsg(output.field.x_star);
    msg.tangent = toVector3Msg(output.field.T_d);
    msg.distance = output.field.D;
    msg.arc_length = output.field.s_star;
    msg.distance_to_end = run.goal.curve->isClosed() ? std::nan("") : run.laps.remainingArcLength();
    msg.lap = run.laps.lapsCompleted();
    msg.progress = run.laps.progress();
    msg.g = output.field.G;
    msg.eta = output.field.eta;
    msg.field = toVector3Msg(output.field.Phi);
    msg.feedforward = toVector3Msg(output.field.Phi_T);
    msg.frame_velocity = toVector3Msg(output.frame_velocity);
    msg.barrier = output.cbf.on ? output.cbf.h : std::numeric_limits<double>::infinity();
    msg.cbf_active = output.cbf.active;
    msg.cmd_raw = toTwistMsg(output.cmd_target);
    msg.cmd = toTwistMsg(output.cmd);
    msg.limiter_scale = output.limiter_alpha;
    msg.compute_time_ms = compute_ms;
    state_pub->publish(msg);
  }

  std::string statusText(const GoalRun3 & run, const d3::Tracker3Output & output) const
  {
    char text[160];
    std::snprintf(
      text, sizeof(text), "%s  D %.3f m  |v| %.2f m/s  lap %u", vf::phaseName(output.phase),
      output.field.D, output.cmd.v.norm(), static_cast<unsigned>(run.laps.lapsCompleted()));
    std::string out(text);
    if (run.goal.laps > 0) {
      out += "/" + std::to_string(run.goal.laps);
    }
    if (vf::Status::has(output.status, vf::Status::kBlocked)) {
      out += "  BLOCKED";
    } else if (output.cbf.active) {
      out += "  CBF";
    }
    if (vf::Status::has(output.status, vf::Status::kAssumption3Clamped)) {
      out += "  A3-CLAMP";
    }
    return out;
  }

  /// One control cycle. Returns true when the goal ended.
  bool cycle(GoalRun3 & run)
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
      if (deactivating || !run.has_published) {
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

    // Parameters: rebuild on change.
    if (updateParameters()) {
      run.rebuild = true;
      run.supervisor.setConfig(supervisorConfig(run.goal.max_duration));
      run.frame.setConfig(toFrameTwistConfig(params));
    }
    if (run.rebuild || !run.tracker) {
      buildTracker(run);
    }

    // Inputs.
    const char * stale_reason = nullptr;
    Snapshot<nav_msgs::msg::Odometry> odom_now;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      odom_now = odom;
    }
    d3::Pose3 pose;
    d3::Twist3 measured;
    if (!odom_now.msg || now - odom_now.received > params.input_timeout) {
      stale_reason = "the odometry is stale or missing";
    } else if (!robotState(*odom_now.msg, &pose, &measured)) {
      stale_reason = "the odometry frame is not transformable into global_frame";
    } else {
      run.last_yaw = pose.yaw;
      if (run.last_position) {
        run.distance += (pose.p - *run.last_position).norm();
      }
      run.last_position = pose.p;
      stale_reason = refreshVoxels(pose.p, now);
    }
    if (stale_reason == nullptr && !run.tracker && !run.config_invalid) {
      stale_reason = "the tracker is not built yet";
    }
    FrameState frame = curveFrame(run, now);
    if (frame.unsupported) {
      stopNow(run);
      finish(run, frame.unsupported->code, frame.unsupported->message, false);
      countCycle(run, true);
      return true;
    }
    const bool cancel = action_server->is_cancel_requested();

    // The tracker.
    const d3::Curve3 & curve = *run.goal.curve;
    d3::Tracker3Input input;
    d3::Tracker3Output output;
    bool ran = false;
    if (
      run.tracker && !run.config_invalid && stale_reason == nullptr && frame.ok && !frame.stale &&
      !deactivating) {
      input.pose = pose;
      input.measured = measured;
      input.t = now;
      input.dt = dt;
      input.curve = &curve;
      input.frame = frame.frame;
      input.goal_seq = run.goal_seq;
      input.cancel = cancel;
      input.obstacles = cbf() && !obstacles.empty() ? &obstacles : nullptr;
      output = run.tracker->step(input, run.state, &run.state);
      ran = true;
      run.laps.update(output.field.s_star, curve.length(now));
    } else {
      run.state.limiter.valid = false;  // re-seed from the measured twist when it resumes
    }

    SupervisorInput supervision;
    supervision.now = now;
    supervision.cancel_requested = cancel;
    supervision.deactivating = deactivating;
    supervision.config_invalid = run.config_invalid;
    supervision.curve_frame_ok = frame.ok;
    supervision.inputs_stale = stale_reason != nullptr;
    supervision.frame_stale = frame.stale;
    supervision.tracker_ran = ran;
    supervision.status = ran ? output.status : vf::Status::kNone;
    supervision.position = pose.p.head<2>();
    supervision.position_z = pose.p.z();
    supervision.nominal_speed =
      run.tracker ? run.tracker->config().desired_speed : params.desired_speed;
    const SupervisorDecision decision = run.supervisor.update(supervision);
    // The cycle's state goes out before a result can end the goal.
    if (ran) {
      publishState(
        run, stamp, pose, output,
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall_start)
          .count());
    }

    bool ended = false;
    switch (decision.action) {
      case SupervisorAction::kContinue:
        if (ran) {
          const bool done = curve.isClosed() ? run.laps.lapsDone() : output.settled;
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
          if (output.cancel_complete || belowStop(output.cmd)) {
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
          std::string reason = decision.reason;
          if (run.config_invalid) {
            reason += ": " + run.config_error;
          } else if (stale_reason != nullptr && decision.code == result_code::kStaleInput) {
            reason += std::string(": ") + stale_reason;
          }
          finish(run, decision.code, reason, false);
          ended = true;
        }
        break;
    }

    // Feedback, state and markers of a cycle that ran the tracker. No feedback while a cancel
    // decelerates: the goal is no longer executing (rclcpp_action warns on Lyrical).
    if (ran && !ended && !cancel) {
      FollowCurve3::Feedback & feedback = *run.feedback;
      feedback.distance_to_curve = output.field.D;
      feedback.arc_length = output.field.s_star;
      feedback.remaining_arc_length = run.laps.remainingArcLength();
      feedback.lap = run.laps.lapsCompleted();
      feedback.speed = toVec3(odom_now.msg->twist.twist.linear).norm();
      feedback.phase = static_cast<std::uint8_t>(output.phase);
      action_server->publish_feedback(run.feedback);
    }
    if (ran) {
      if (
        params.publish_markers && !ended && now - run.last_markers >= params.visualization_period) {
        run.last_markers = now;
        MarkerInput3 markers;
        markers.frame_id = params.global_frame;
        markers.stamp = stamp;
        markers.curve = &curve;
        markers.frame = frame.frame;
        markers.t = now;
        markers.sample_spacing = run.goal.sample_spacing;
        markers.field = &run.tracker->field();
        markers.field_extent = params.field_arrows_extent;
        markers.field_step = params.field_arrows_step;
        markers.pose = pose;
        markers.output = &output;
        markers.obstacles = &obstacles;
        markers.body_radius = cbf() ? params.body_radius : 0.0;
        markers.status_text = statusText(run, output);
        marker_pub->publish(buildMarkers3(markers));
      }
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
      snapshot.distance = output.field.D;
    }
    snapshot.command = run.last_cmd;
    snapshot.commands_published = run.published;
    snapshot.laps_completed = run.laps.lapsCompleted();
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
    frame_odom_sub.reset();
    frame_odom_topic.clear();
    odom_sub.reset();
    cloud_sub.reset();
    marker_pub.reset();
    state_pub.reset();
    curve_pub.reset();
    cmd_pub.reset();
    tf_listener.reset();
    tf.reset();
    obstacles_check.reset();
    listener.reset();
    const std::lock_guard<std::mutex> lock(mutex);
    odom = {};
    frame_odom = {};
    cloud = {};
    used_cloud.reset();
    cloud_points.clear();
    cloud_ok = false;
    window_valid = false;
    obstacles.clearPrimitives();
    obstacles.clearVoxels();
  }
};

Follower3D::Follower3D(const rclcpp::NodeOptions & options)
: compat::LifecycleNode("follower_3d", "", options), impl_(std::make_unique<Impl>(*this))
{
  impl_->clock = get_clock();
}

Follower3D::~Follower3D()
{
  Impl & s = *impl_;
  s.deactivating = true;
  s.wake.notify_all();
  if (s.action_server) {
    s.action_server->deactivate();
  }
  s.release();
}

CycleSnapshot3 Follower3D::lastCycle() const
{
  const std::lock_guard<std::mutex> lock(impl_->diag_mutex);
  return impl_->snapshot;
}

bool Follower3D::preemptPending() const
{
  const auto server = impl_->action_server;
  return server && server->is_preempt_requested();
}

Follower3D::CallbackReturn Follower3D::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  Impl & s = *impl_;
  try {
    const auto parameters = get_node_parameters_interface();
    s.listener = std::make_shared<ParamListener>(parameters, s.logger, "");
    s.params = s.listener->get_params();
    std::string error;
    if (!validateParams(s.params, &error)) {
      RCLCPP_ERROR(s.logger, "invalid configuration: %s", error.c_str());
      s.release();
      return CallbackReturn::FAILURE;
    }
    s.refreshPrimitives();
    // Registered after the listener, so rclcpp runs it first: an `obstacles` update with an
    // invalid entry is refused instead of aborting the running goal.
    s.obstacles_check =
      add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> & changed) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const rclcpp::Parameter & parameter : changed) {
          if (
            parameter.get_name() == "obstacles" &&
            parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
            std::vector<d3::Primitive3> primitives;
            std::string reason;
            if (!parsePrimitives(parameter.as_string_array(), &primitives, &reason)) {
              result.successful = false;
              result.reason = reason;
            }
          }
        }
        return result;
      });

    const auto self = std::static_pointer_cast<compat::LifecycleNode>(shared_from_this());
    s.tf = compat::makeTfBuffer(self);
    s.tf_listener = compat::makeTfListener(*s.tf, self, true);
    rclcpp_lifecycle::LifecycleNode & base = *this;
    s.cmd_pub = base.create_publisher<geometry_msgs::msg::TwistStamped>(
      s.params.cmd_vel_topic, rclcpp::QoS(1).reliable());
    s.curve_pub = base.create_publisher<vectorfield_msgs::msg::Curve3>(
      "~/curve", rclcpp::QoS(1).reliable().transient_local());
    s.state_pub = base.create_publisher<FieldState3>("~/field_state", rclcpp::QoS(10).reliable());
    s.marker_pub = base.create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/markers", rclcpp::QoS(1).reliable());

    Impl * impl = impl_.get();
    s.odom_sub = compat::subscribe<nav_msgs::msg::Odometry>(
      this, s.params.odom_topic, rclcpp::SensorDataQoS(),
      [impl](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const double received = impl->clock->now().seconds();
        const std::lock_guard<std::mutex> lock(impl->mutex);
        impl->odom = Snapshot<nav_msgs::msg::Odometry>{std::move(msg), received};
      });
    if (!s.params.voxel_cloud_topic.empty()) {
      s.cloud_sub = compat::subscribe<sensor_msgs::msg::PointCloud2>(
        this, s.params.voxel_cloud_topic, rclcpp::SensorDataQoS(),
        [impl](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          const double received = impl->clock->now().seconds();
          const std::lock_guard<std::mutex> lock(impl->mutex);
          impl->cloud = Snapshot<sensor_msgs::msg::PointCloud2>{std::move(msg), received};
        });
    }
    s.action_server = compat::makeActionServer<FollowCurve3>(
      this, "follow_curve_3d", [this]() { execute(); }, nullptr, kServerTimeout, true);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(s.logger, "configure failed: %s", e.what());
    s.release();
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(
    s.logger, "3D follower configured (cmd_frame %s, obstacles %s, %zu primitives%s)",
    s.params.cmd_frame.c_str(), s.params.obstacle_mode.c_str(), s.params.obstacles.size(),
    s.params.voxel_cloud_topic.empty() ? "" : ", voxel cloud");
  return CallbackReturn::SUCCESS;
}

Follower3D::CallbackReturn Follower3D::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  Impl & s = *impl_;
  s.deactivating = false;
  s.cmd_pub->on_activate();
  s.curve_pub->on_activate();
  s.state_pub->on_activate();
  s.marker_pub->on_activate();
  s.action_server->activate();
  createBond();
  return CallbackReturn::SUCCESS;
}

Follower3D::CallbackReturn Follower3D::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  Impl & s = *impl_;
  s.deactivating = true;
  s.wake.notify_all();
  s.action_server->deactivate();  // waits for a running goal to stop (UNKNOWN, one zero)
  s.marker_pub->publish(clearMarkers3(s.params.global_frame, s.clock->now()));
  s.marker_pub->on_deactivate();
  s.state_pub->on_deactivate();
  s.curve_pub->on_deactivate();
  s.cmd_pub->on_deactivate();
  destroyBond();
  return CallbackReturn::SUCCESS;
}

Follower3D::CallbackReturn Follower3D::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  impl_->release();
  impl_->deactivating = false;
  return CallbackReturn::SUCCESS;
}

Follower3D::CallbackReturn Follower3D::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
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

void Follower3D::execute()
{
  Impl & s = *impl_;
  const std::shared_ptr<compat::SimpleActionServer<FollowCurve3>> server = s.action_server;
  if (!server) {
    return;
  }
  const auto goal = server->get_current_goal();
  if (!goal) {
    return;
  }
  auto run = std::make_unique<GoalRun3>();
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
        auto next_run = std::make_unique<GoalRun3>();
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

}  // namespace vectorfield_follower_3d

RCLCPP_COMPONENTS_REGISTER_NODE(vectorfield_follower_3d::Follower3D)
