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

// The in-process harness of the follower tests (plan §9, layer 2b): the CurveFollower on
// simulated time with a kinematic robot that closes the loop through odometry and TF.
//
// The test thread drives everything in lock step, one control period per tick: the robot
// integrates the last command it received (vectorfield_core::sim::integrate) and publishes TF,
// odometry, the local costmap, the footprint and (optionally) a moving frame's odometry; then
// /clock advances by one period, and the tick waits until the follower's clock has reached it,
// the follower has run the cycle (CurveFollower::lastCycle()) and its command has reached the
// robot. Every timeout of the follower is therefore measured in simulated time, and the wall
// clock only bounds the waits (hang guards), so a slow build changes nothing but the run time.

#ifndef FOLLOWER_TEST_HARNESS_HPP_
#define FOLLOWER_TEST_HARNESS_HPP_

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/qos.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_curve_follower/curve_follower.hpp"
#include "vectorfield_msgs/action/follow_curve.hpp"
#include "vectorfield_ros/compat.hpp"
#include "vectorfield_ros/conversions.hpp"

namespace vectorfield_curve_follower_test
{

namespace vf = vectorfield_core;
namespace vr = vectorfield_ros;
using vectorfield_curve_follower::CurveFollower;
using vectorfield_curve_follower::CycleSnapshot;
using FollowCurve = vectorfield_msgs::action::FollowCurve;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowCurve>;
using WrappedResult = GoalHandle::WrappedResult;
using ResultFuture = std::shared_future<WrappedResult>;

/// Control period at the default controller_frequency (20 Hz): one tick.
inline constexpr std::int64_t kPeriodNs = 50'000'000;
/// @copydoc kPeriodNs
inline constexpr double kPeriod = 0.05;
/// Simulated time of the first test (away from the zero stamp, which means "now"); each test
/// starts kTestSpacingNs later than the previous one, so the throttled logs of the follower
/// (static per call site) print in every test.
inline constexpr std::int64_t kStartNs = 100'000'000'000;
/// @copydoc kStartNs
inline constexpr std::int64_t kTestSpacingNs = 1000'000'000'000;
/// Bound of every wall-clock wait (a hang guard; generous for the coverage build).
inline constexpr std::chrono::seconds kWallTimeout{60};

/// Side of the robot's square footprint (m).
inline constexpr double kFootprintSide = 0.3;
/// The local costmap: kCostmapCells² cells of kCostmapResolution m centred on the odom origin.
inline constexpr std::uint32_t kCostmapCells = 120;
/// @copydoc kCostmapCells
inline constexpr double kCostmapResolution = 0.05;
/// LETHAL_OBSTACLE.
inline constexpr std::uint8_t kLethal = 254;

/// Polls `done` until it holds or `timeout` (wall clock) expires.
template <typename Predicate>
bool waitFor(const Predicate & done, std::chrono::milliseconds timeout = kWallTimeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!done()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  return true;
}

inline bool isZero(const vf::Twist2 & twist)
{
  return twist.vx == 0.0 && twist.vy == 0.0 && twist.wz == 0.0;
}

inline bool sameTwist(const vf::Twist2 & a, const vf::Twist2 & b)
{
  return a.vx == b.vx && a.vy == b.vy && a.wz == b.wz;
}

inline double seconds(std::int64_t ns) { return 1e-9 * static_cast<double>(ns); }

/// The follower's last cycle, for failure messages.
inline std::string describe(const CycleSnapshot & snapshot)
{
  return "cycles " + std::to_string(snapshot.cycles) + ", executing " +
         std::to_string(static_cast<int>(snapshot.executing)) + ", tracker_ran " +
         std::to_string(static_cast<int>(snapshot.tracker_ran)) + ", decision " +
         std::to_string(static_cast<int>(snapshot.decision.action)) + " (" +
         snapshot.decision.reason + "), status " + std::to_string(snapshot.status) +
         ", published " + std::to_string(snapshot.commands_published) + ", last result " +
         std::to_string(snapshot.last_result) + " (" + snapshot.last_reason + ")";
}

/// An axis-aligned box of lethal cells (odom frame, m).
struct Box
{
  double x_min{0.0};
  double y_min{0.0};
  double x_max{0.0};
  double y_max{0.0};
};

/// A square box of half-size `half` centred on `centre`.
inline Box boxAt(const vf::Vec2 & centre, double half)
{
  return Box{centre.x() - half, centre.y() - half, centre.x() + half, centre.y() + half};
}

/// A frame translating at constant velocity in `map`, published as nav_msgs/Odometry.
struct MovingFrame
{
  std::string topic;
  std::string child_frame_id;
  vf::Vec2 velocity{vf::Vec2::Zero()};
  std::int64_t start_ns{0};

  [[nodiscard]] vf::Vec2 originAt(std::int64_t t_ns) const
  {
    return velocity * seconds(t_ns - start_ns);
  }
};

/// A kinematic robot that executes `cmd_vel_nav` and publishes what the follower needs.
class FakeRobot
{
public:
  FakeRobot(
    const std::string & ns, vf::Pose2 start, const vf::Kinematics & kinematics, std::int64_t t_ns)
  : node_(std::make_shared<rclcpp::Node>("fake_robot", ns)),
    kinematics_(kinematics),
    pose_(std::move(start))
  {
    // TF publishers of our own (not broadcasters), so the harness can wait until they match.
    tf_pub_ =
      node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf", tf2_ros::DynamicBroadcasterQoS());
    tf_static_pub_ = node_->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf_static", tf2_ros::StaticBroadcasterQoS());
    clock_pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    costmap_pub_ = node_->create_publisher<nav2_msgs::msg::Costmap>(
      "local_costmap/costmap_raw", rclcpp::QoS(1).reliable());
    footprint_pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "local_costmap/published_footprint", rclcpp::QoS(1).reliable());
    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_nav", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        cmd_ = vr::toTwist2(*msg);
        log_.push_back(cmd_);
      });
    geometry_msgs::msg::TransformStamped map_odom;
    map_odom.header.frame_id = "map";
    map_odom.header.stamp = rclcpp::Time(t_ns, RCL_ROS_TIME);
    map_odom.child_frame_id = "odom";
    map_odom.transform.rotation.w = 1.0;
    tf_static_.transforms.push_back(map_odom);
  }

  rclcpp::Node::SharedPtr node() const { return node_; }

  /// @name Commands received
  /// @{
  std::size_t commands() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return log_.size();
  }
  std::vector<vf::Twist2> log() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return log_;
  }
  vf::Twist2 lastCommand() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return cmd_;
  }
  /// Number of zero commands at the end of the log.
  std::size_t trailingZeros() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    std::size_t count = 0;
    for (auto it = log_.rbegin(); it != log_.rend() && isZero(*it); ++it) {
      ++count;
    }
    return count;
  }
  /// @}

  /// The true pose (no dither).
  vf::Pose2 pose() const { return pose_; }
  /// Path length actually driven (m).
  double distance() const { return distance_; }
  /// Angle actually swept around the origin, counter-clockwise positive (rad).
  double sweptAngle() const { return swept_; }

  /// @name Test controls (test thread only)
  /// @{
  void setBoxes(std::vector<Box> boxes) { boxes_ = std::move(boxes); }
  void setFrozen(bool frozen) { frozen_ = frozen; }
  void setOdometryPaused(bool paused) { odom_paused_ = paused; }
  void setCostmapPaused(bool paused) { costmap_paused_ = paused; }
  void setFramePaused(bool paused) { frame_paused_ = paused; }
  /// Published poses alternate ±`amplitude` along the heading (s* dithers by about that much).
  void setDither(double amplitude) { dither_ = amplitude; }
  void startMovingFrame(MovingFrame frame)
  {
    frame_ = std::move(frame);
    frame_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(frame_->topic, 10);
  }
  /// Origin of the moving frame at `t_ns` (zero without one).
  vf::Vec2 frameOriginAt(std::int64_t t_ns) const
  {
    return frame_ ? frame_->originAt(t_ns) : vf::Vec2::Zero();
  }
  std::size_t frameSubscribers() const
  {
    return frame_pub_ ? frame_pub_->get_subscription_count() : 0U;
  }
  /// @}

  /// The follower's endpoints are matched.
  bool connected() const
  {
    return clock_pub_->get_subscription_count() > 0 && tf_pub_->get_subscription_count() > 0 &&
           tf_static_pub_->get_subscription_count() > 0 &&
           odom_pub_->get_subscription_count() > 0 && costmap_pub_->get_subscription_count() > 0 &&
           footprint_pub_->get_subscription_count() > 0 && cmd_sub_->get_publisher_count() > 0;
  }

  void publishClock(std::int64_t t_ns)
  {
    rosgraph_msgs::msg::Clock msg;
    msg.clock = rclcpp::Time(t_ns, RCL_ROS_TIME);
    clock_pub_->publish(msg);
  }

  /// Integrates the last command over `dt` and publishes the state at `t_ns`.
  void step(std::int64_t t_ns, double dt)
  {
    vf::Twist2 cmd;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      cmd = cmd_;
    }
    if (frozen_) {
      cmd = vf::Twist2{};
    } else {
      const vf::Pose2 next = vf::sim::integrate(kinematics_, pose_, cmd, dt);
      distance_ += (next.p - pose_.p).norm();
      swept_ +=
        vf::wrapAngle(std::atan2(next.p.y(), next.p.x()) - std::atan2(pose_.p.y(), pose_.p.x()));
      pose_ = next;
    }
    vf::Pose2 published = pose_;
    if (dither_ > 0.0) {
      const double sign = ticks_ % 2 == 0 ? 1.0 : -1.0;
      published.p += sign * dither_ * vf::Vec2(std::cos(pose_.yaw), std::sin(pose_.yaw));
    }
    const rclcpp::Time stamp(t_ns, RCL_ROS_TIME);

    geometry_msgs::msg::TransformStamped odom_base;
    odom_base.header.frame_id = "odom";
    odom_base.header.stamp = stamp;
    odom_base.child_frame_id = "base_link";
    odom_base.transform.translation.x = published.p.x();
    odom_base.transform.translation.y = published.p.y();
    odom_base.transform.rotation = vr::yawToQuaternion(published.yaw);
    tf2_msgs::msg::TFMessage tf;
    tf.transforms.push_back(odom_base);
    tf_pub_->publish(tf);
    // map -> odom is static; republished because a late-joining reader may get the transient
    // local sample only after some delay.
    if (ticks_ % 10 == 0) {
      tf_static_pub_->publish(tf_static_);
    }

    if (!odom_paused_) {
      nav_msgs::msg::Odometry odom;
      odom.header = odom_base.header;
      odom.child_frame_id = "base_link";
      odom.pose.pose = vr::toPoseMsg(published);
      odom.twist.twist = vr::toTwistMsg(cmd);
      odom_pub_->publish(odom);
    }
    if (ticks_ % 2 == 0 && !costmap_paused_) {
      publishCostmap(stamp);
    }
    if (ticks_ % 10 == 0) {
      publishFootprint(stamp);
    }
    if (frame_ && !frame_paused_) {
      const vf::Vec2 origin = frame_->originAt(t_ns);
      nav_msgs::msg::Odometry odom;
      odom.header.frame_id = "map";
      odom.header.stamp = stamp;
      odom.child_frame_id = frame_->child_frame_id;
      odom.pose.pose = vr::toPoseMsg(vf::Pose2{origin, 0.0});
      odom.twist.twist =
        vr::toTwistMsg(vf::Twist2{frame_->velocity.x(), frame_->velocity.y(), 0.0});
      frame_pub_->publish(odom);
    }
    ++ticks_;
  }

private:
  void publishCostmap(const rclcpp::Time & stamp)
  {
    nav2_msgs::msg::Costmap costmap;
    costmap.header.frame_id = "odom";
    costmap.header.stamp = stamp;
    costmap.metadata.resolution = static_cast<float>(kCostmapResolution);
    costmap.metadata.size_x = kCostmapCells;
    costmap.metadata.size_y = kCostmapCells;
    const double origin = -0.5 * kCostmapResolution * static_cast<double>(kCostmapCells);
    costmap.metadata.origin.position.x = origin;
    costmap.metadata.origin.position.y = origin;
    costmap.metadata.origin.orientation.w = 1.0;
    costmap.data.assign(std::size_t{kCostmapCells} * std::size_t{kCostmapCells}, 0U);
    for (std::uint32_t j = 0; j < kCostmapCells; ++j) {
      for (std::uint32_t i = 0; i < kCostmapCells; ++i) {
        const double x = origin + (static_cast<double>(i) + 0.5) * kCostmapResolution;
        const double y = origin + (static_cast<double>(j) + 0.5) * kCostmapResolution;
        for (const Box & box : boxes_) {
          if (x >= box.x_min && x <= box.x_max && y >= box.y_min && y <= box.y_max) {
            costmap.data[std::size_t{j} * kCostmapCells + i] = kLethal;
          }
        }
      }
    }
    costmap_pub_->publish(costmap);
  }

  void publishFootprint(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PolygonStamped footprint;
    footprint.header.frame_id = "base_link";
    footprint.header.stamp = stamp;
    const auto half = static_cast<float>(0.5 * kFootprintSide);
    const std::pair<float, float> corners[] = {
      {half, half}, {-half, half}, {-half, -half}, {half, -half}};
    for (const auto & corner : corners) {
      geometry_msgs::msg::Point32 point;
      point.x = corner.first;
      point.y = corner.second;
      footprint.polygon.points.push_back(point);
    }
    footprint_pub_->publish(footprint);
  }

  rclcpp::Node::SharedPtr node_;
  vf::Kinematics kinematics_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_pub_;
  tf2_msgs::msg::TFMessage tf_static_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr costmap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr frame_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  mutable std::mutex mutex_;
  vf::Twist2 cmd_;
  std::vector<vf::Twist2> log_;

  vf::Pose2 pose_;
  double distance_{0.0};
  double swept_{0.0};
  std::size_t ticks_{0};
  std::vector<Box> boxes_;
  bool frozen_{false};
  bool odom_paused_{false};
  bool costmap_paused_{false};
  bool frame_paused_{false};
  double dither_{0.0};
  std::optional<MovingFrame> frame_;
};

/// A stub action's status topic (`<action>/_action/status`, action-status QoS).
class ActionStatusStub
{
public:
  ActionStatusStub(const rclcpp::Node::SharedPtr & node, const std::string & action)
  : pub_(node->create_publisher<action_msgs::msg::GoalStatusArray>(
      action + "/_action/status", rclcpp::QoS(1).reliable().transient_local()))
  {
  }

  /// One goal with `status` (an action_msgs/GoalStatus constant).
  void publish(std::int8_t status)
  {
    action_msgs::msg::GoalStatus goal;
    goal.goal_info.goal_id.uuid[0] = 1U;
    goal.status = status;
    action_msgs::msg::GoalStatusArray msg;
    msg.status_list.push_back(goal);
    pub_->publish(msg);
  }

  [[nodiscard]] std::string topic() const { return pub_->get_topic_name(); }
  [[nodiscard]] std::size_t subscribers() const { return pub_->get_subscription_count(); }

private:
  rclcpp::Publisher<action_msgs::msg::GoalStatusArray>::SharedPtr pub_;
};

/// What a test starts.
struct FollowerSetup
{
  /// Namespace of the follower and the robot ("" = root).
  std::string ns;
  /// The robot's start pose: on the circle x² + y² = 0.5², heading along it (counter-clockwise).
  vf::Pose2 start{vf::Vec2(0.5, 0.0), 0.5 * vf::kPi};
  /// The robot's motion model.
  vf::Kinematics kinematics{vf::Kinematics::fromConfig(vf::CoreConfig{})};
  /// Parameter overrides on top of the defaults below (same name: replaced).
  std::vector<rclcpp::Parameter> parameters;
  /// The robot publishes the local costmap.
  bool costmap{true};
};

/// @name Goals
/// @{
inline FollowCurve::Goal circleGoal(double radius = 0.5, std::uint32_t laps = 1)
{
  FollowCurve::Goal goal;
  goal.curve.type = vectorfield_msgs::msg::Curve::BUILTIN;
  goal.curve.builtin_shape = "ellipse";
  goal.curve.parameter_names = {"a", "b"};
  goal.curve.parameter_values = {radius, radius};
  goal.frame_motion = FollowCurve::Goal::FRAME_STATIC;
  goal.laps = laps;
  return goal;
}

inline FollowCurve::Goal polylineGoal(const std::vector<vf::Vec2> & points, bool closed = false)
{
  FollowCurve::Goal goal;
  goal.curve.type = vectorfield_msgs::msg::Curve::POLYLINE;
  goal.curve.closed = closed;
  for (const vf::Vec2 & p : points) {
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    goal.curve.points.push_back(point);
  }
  return goal;
}

inline FollowCurve::Goal expressionGoal(
  const std::string & x, const std::string & y, double s_min, double s_max, bool closed)
{
  FollowCurve::Goal goal;
  goal.curve.type = vectorfield_msgs::msg::Curve::EXPRESSION;
  goal.curve.x_expression = x;
  goal.curve.y_expression = y;
  goal.curve.s_min = s_min;
  goal.curve.s_max = s_max;
  goal.curve.closed = closed;
  return goal;
}
/// @}

/// The fixture: one follower and one robot per test (see the file comment).
class FollowerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void TearDown() override
  {
    if (
      follower_ &&
      follower_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      follower_->deactivate();
    }
    if (spinner_.joinable()) {
      executor_.cancel();
      spinner_.join();
    }
    client_.reset();
    if (robot_) {
      executor_.remove_node(robot_->node());
    }
    if (follower_) {
      executor_.remove_node(follower_->get_node_base_interface());
    }
    robot_.reset();
    follower_.reset();
  }

  /// Creates, configures and activates the follower and the robot, then runs 0.5 s of warm-up
  /// (call through ASSERT_NO_FATAL_FAILURE).
  void start(const FollowerSetup & setup = FollowerSetup{})
  {
    std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("use_sim_time", true),
      // Most tests have no Nav2: nothing publishes follow_path/_action/status.
      rclcpp::Parameter("require_nav2_interlock", false),
      rclcpp::Parameter("enable_stamped_cmd_vel", false),
    };
    for (const rclcpp::Parameter & extra : setup.parameters) {
      bool replaced = false;
      for (rclcpp::Parameter & parameter : parameters) {
        if (parameter.get_name() == extra.get_name()) {
          parameter = extra;
          replaced = true;
        }
      }
      if (!replaced) {
        parameters.push_back(extra);
      }
    }
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    if (!setup.ns.empty()) {
      options.arguments({"--ros-args", "-r", "__ns:=" + setup.ns});
    }
    follower_ = std::make_shared<CurveFollower>(options);
    static std::int64_t next_start_ns = kStartNs;
    t_ns_ = next_start_ns;
    next_start_ns += kTestSpacingNs;
    robot_ = std::make_unique<FakeRobot>(setup.ns, setup.start, setup.kinematics, t_ns_);
    robot_->setCostmapPaused(!setup.costmap);
    executor_.add_node(follower_->get_node_base_interface());
    executor_.add_node(robot_->node());
    spinner_ = std::thread([this]() { executor_.spin(); });
    client_ = rclcpp_action::create_client<FollowCurve>(robot_->node(), "follow_curve");

    ASSERT_EQ(follower_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(follower_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_TRUE(client_->wait_for_action_server(kWallTimeout));
    ASSERT_TRUE(waitFor([this]() { return robot_->connected(); })) << "topics not matched";
    tickFor(0.5);
  }

  /// Simulated time (s).
  double now() const { return seconds(t_ns_); }

  /// One control period (see the file comment).
  void tick()
  {
    t_ns_ += kPeriodNs;
    robot_->step(t_ns_, kPeriod);
    const std::uint64_t cycles = follower_->lastCycle().cycles;
    robot_->publishClock(t_ns_);
    // /clock is best effort: republished while the follower has not seen it.
    auto republish = std::chrono::steady_clock::now();
    const bool clock_seen = waitFor([&]() {
      if (follower_->get_clock()->now().nanoseconds() >= t_ns_) {
        return true;
      }
      if (std::chrono::steady_clock::now() - republish > std::chrono::milliseconds(50)) {
        robot_->publishClock(t_ns_);
        republish = std::chrono::steady_clock::now();
      }
      return false;
    });
    ASSERT_TRUE(clock_seen) << "the follower did not receive /clock";
    ASSERT_TRUE(waitFor([&]() {
      const CycleSnapshot snapshot = follower_->lastCycle();
      return !snapshot.executing || snapshot.cycles > cycles;
    }))
      << "no control cycle at t = " << now();
    // Compared with the latest snapshot: the cycle that ended the wait may be the goal's first
    // (run on acceptance), with this tick's cycle still running.
    ASSERT_TRUE(
      waitFor([&]() { return sameTwist(robot_->lastCommand(), follower_->lastCycle().command); }))
      << "the command did not reach the robot";
  }

  void tickFor(double duration)
  {
    const auto ticks = static_cast<int>(std::lround(duration / kPeriod));
    for (int i = 0; i < ticks && !::testing::Test::HasFatalFailure(); ++i) {
      tick();
    }
  }

  /// Ticks until `done` holds (true) or `max_duration` of simulated time passed (false).
  template <typename Predicate>
  bool tickUntil(const Predicate & done, double max_duration)
  {
    const double deadline = now() + max_duration;
    while (!done()) {
      if (now() >= deadline || ::testing::Test::HasFatalFailure()) {
        return false;
      }
      tick();
    }
    return true;
  }

  /// Sends a goal; null (and a failure) if it was not answered or rejected.
  std::shared_ptr<GoalHandle> send(const FollowCurve::Goal & goal)
  {
    auto future = client_->async_send_goal(goal);
    if (future.wait_for(kWallTimeout) != std::future_status::ready) {
      ADD_FAILURE() << "goal not answered";
      return nullptr;
    }
    auto handle = future.get();
    if (!handle) {
      ADD_FAILURE() << "goal rejected (the follower never rejects goals)";
    }
    return handle;
  }

  ResultFuture resultOf(const std::shared_ptr<GoalHandle> & handle)
  {
    return client_->async_get_result(handle);
  }

  static bool ready(const ResultFuture & future)
  {
    return future.valid() && future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }

  /// Ticks until the result arrives, for at most `max_duration` of simulated time (on failure:
  /// an empty result, whose `result` is null).
  WrappedResult waitResult(const ResultFuture & future, double max_duration)
  {
    if (!tickUntil([&]() { return ready(future); }, max_duration)) {
      ADD_FAILURE() << "no result within " << max_duration
                    << " s of simulated time: " << describe(follower_->lastCycle());
      return WrappedResult{};
    }
    return future.get();
  }

  /// Sends a goal and ticks until its result.
  WrappedResult run(const FollowCurve::Goal & goal, double max_duration)
  {
    const auto handle = send(goal);
    if (!handle) {
      return WrappedResult{};
    }
    return waitResult(resultOf(handle), max_duration);
  }

  /// A few ticks with wall-clock pauses, so in-flight messages arrive.
  void settle()
  {
    for (int i = 0; i < 5; ++i) {
      tick();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  /// Settles, then expects no command during `duration` of simulated time.
  void expectSilence(double duration = 1.0)
  {
    settle();
    const std::size_t before = robot_->commands();
    tickFor(duration);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(robot_->commands(), before) << "the follower must be silent between goals";
  }

  /// Linear speeds of the commands from `first` on never increase (a deceleration).
  ::testing::AssertionResult deceleratesFrom(std::size_t first) const
  {
    const std::vector<vf::Twist2> log = robot_->log();
    for (std::size_t k = first + 1; k < log.size(); ++k) {
      const double before = std::hypot(log[k - 1].vx, log[k - 1].vy);
      const double after = std::hypot(log[k].vx, log[k].vy);
      if (after > before + 1e-9) {
        return ::testing::AssertionFailure()
               << "command " << k << " speeds up from " << before << " to " << after;
      }
    }
    return ::testing::AssertionSuccess();
  }

  /// The robot's heading error to the tangent of the circle at (0.5, 0) (rad).
  static double headingError(const vf::Pose2 & pose)
  {
    return std::abs(vf::wrapAngle(pose.yaw - 0.5 * vf::kPi));
  }

  rclcpp::executors::MultiThreadedExecutor executor_;
  std::shared_ptr<CurveFollower> follower_;
  std::unique_ptr<FakeRobot> robot_;
  rclcpp_action::Client<FollowCurve>::SharedPtr client_;
  std::thread spinner_;
  std::int64_t t_ns_{kStartNs};
};

}  // namespace vectorfield_curve_follower_test

#endif  // FOLLOWER_TEST_HARNESS_HPP_
