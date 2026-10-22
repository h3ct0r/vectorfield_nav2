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

// The in-process harness of the 3D follower's node tests: the Follower3D on simulated time with
// a kinematic point robot that closes the loop through odometry. As the planar follower's
// harness (vectorfield_curve_follower/test/follower_test_harness.hpp), the test thread drives
// everything in lock step, one control period per tick: the robot integrates the last command
// (vectorfield_core::d3::sim::integrate) and publishes its odometry (and optionally a moving
// frame's), /clock advances by one period, and the tick waits until the follower has run its
// cycle and every command it published has reached the robot. Simulated time never advances
// while the follower is busy, so the follower's timeouts are all measured in simulated time.

#ifndef FOLLOWER_3D_HARNESS_HPP_
#define FOLLOWER_3D_HARNESS_HPP_

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

#include "action_msgs/srv/cancel_goal.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "vectorfield_core/d3/frame.hpp"
#include "vectorfield_core/d3/simulator.hpp"
#include "vectorfield_follower_3d/conversions.hpp"
#include "vectorfield_follower_3d/follower_3d.hpp"
#include "vectorfield_msgs/action/follow_curve3.hpp"

namespace vectorfield_follower_3d_test
{

namespace d3 = vectorfield_core::d3;
using vectorfield_follower_3d::CycleSnapshot3;
using vectorfield_follower_3d::Follower3D;
using FollowCurve3 = vectorfield_msgs::action::FollowCurve3;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowCurve3>;
using WrappedResult = GoalHandle::WrappedResult;
using ResultFuture = std::shared_future<WrappedResult>;

inline constexpr std::int64_t kPeriodNs = 50'000'000;
inline constexpr double kPeriod = 0.05;
inline constexpr std::int64_t kStartNs = 100'000'000'000;
inline constexpr std::int64_t kTestSpacingNs = 1000'000'000'000;
inline constexpr std::chrono::seconds kWallTimeout{60};

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

inline bool isZero(const d3::Twist3 & u)
{
  return u.v.x() == 0.0 && u.v.y() == 0.0 && u.v.z() == 0.0 && u.wz == 0.0;
}

inline double seconds(std::int64_t ns) { return 1e-9 * static_cast<double>(ns); }

inline std::string describe(const CycleSnapshot3 & s)
{
  return "cycles " + std::to_string(s.cycles) + ", executing " +
         std::to_string(static_cast<int>(s.executing)) + ", tracker_ran " +
         std::to_string(static_cast<int>(s.tracker_ran)) + ", decision " +
         std::to_string(static_cast<int>(s.decision.action)) + " (" + s.decision.reason +
         "), status " + std::to_string(s.status) + ", D " + std::to_string(s.distance) +
         ", published " + std::to_string(s.commands_published) + ", goal " +
         std::to_string(s.goal) + ", last result " + std::to_string(s.last_result) + " (" +
         s.last_reason + ")";
}

/// A frame translating and turning with a constant body twist, published as Odometry.
struct MovingFrame
{
  std::string topic;
  std::string child_frame_id;
  d3::FrameTwist3 twist;
  std::int64_t start_ns{0};
  [[nodiscard]] d3::Isometry3 poseAt(std::int64_t t_ns) const
  {
    return d3::expSE3(twist, seconds(t_ns - start_ns));
  }
};

class FakeDrone
{
public:
  explicit FakeDrone(const d3::Pose3 & start) : node_(std::make_shared<rclcpp::Node>("fake_drone"))
  {
    state_.pose = start;
    clock_pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        d3::Twist3 cmd;
        cmd.v = vectorfield_follower_3d::toVec3(msg->twist.linear);
        cmd.wz = msg->twist.angular.z;
        cmd_ = cmd;
        log_.push_back(cmd);
        frames_.push_back(msg->header.frame_id);
      });
  }

  rclcpp::Node::SharedPtr node() const { return node_; }

  std::size_t commands() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return log_.size();
  }

  std::vector<d3::Twist3> log() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return log_;
  }

  std::string lastFrame() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return frames_.empty() ? std::string() : frames_.back();
  }

  std::size_t trailingZeros() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    std::size_t count = 0;
    for (auto it = log_.rbegin(); it != log_.rend() && isZero(*it); ++it) {
      ++count;
    }
    return count;
  }

  d3::Pose3 pose() const { return state_.pose; }

  void startMovingFrame(MovingFrame frame)
  {
    frame_ = std::move(frame);
    frame_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(frame_->topic, 10);
  }

  d3::Isometry3 framePoseAt(std::int64_t t_ns) const
  {
    return frame_ ? frame_->poseAt(t_ns) : d3::Isometry3::Identity();
  }

  bool connected() const
  {
    return clock_pub_->get_subscription_count() > 0 && odom_pub_->get_subscription_count() > 0 &&
           cmd_sub_->get_publisher_count() > 0;
  }

  void publishClock(std::int64_t t_ns)
  {
    rosgraph_msgs::msg::Clock msg;
    msg.clock = rclcpp::Time(t_ns, RCL_ROS_TIME);
    clock_pub_->publish(msg);
  }

  void step(std::int64_t t_ns, double dt)
  {
    d3::Twist3 cmd;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      cmd = cmd_;
    }
    state_ = d3::sim::integrate(state_, cmd, dt, d3::sim::PlantParams3{});
    const rclcpp::Time stamp(t_ns, RCL_ROS_TIME);
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.header.stamp = stamp;
    odom.child_frame_id = "base_link";
    d3::Isometry3 T = d3::Isometry3::Identity();
    T.linear() = vectorfield_follower_3d::yawRotation(state_.pose.yaw);
    T.translation() = state_.pose.p;
    odom.pose.pose = vectorfield_follower_3d::toPoseMsg(T);
    odom.twist.twist.linear =
      vectorfield_follower_3d::toVector3Msg(T.linear().transpose() * state_.twist.v);
    odom.twist.twist.angular.z = state_.twist.wz;
    odom_pub_->publish(odom);
    if (frame_) {
      nav_msgs::msg::Odometry msg;
      msg.header = odom.header;
      msg.child_frame_id = frame_->child_frame_id;
      msg.pose.pose = vectorfield_follower_3d::toPoseMsg(frame_->poseAt(t_ns));
      msg.twist.twist.linear = vectorfield_follower_3d::toVector3Msg(frame_->twist.v);
      msg.twist.twist.angular = vectorfield_follower_3d::toVector3Msg(frame_->twist.w);
      frame_pub_->publish(msg);
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr frame_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  mutable std::mutex mutex_;
  d3::Twist3 cmd_;
  std::vector<d3::Twist3> log_;
  std::vector<std::string> frames_;
  d3::sim::PlantState3 state_;
  std::optional<MovingFrame> frame_;
};

struct RobotSetup
{
  d3::Pose3 start;
  std::vector<rclcpp::Parameter> parameters;
};

class Follower3DTest : public ::testing::Test
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

  /// Creates the follower and the robot; configures and activates the follower unless
  /// `activate` is false (then returns the configure transition's state id).
  std::uint8_t start(const RobotSetup & setup, bool activate = true)
  {
    std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("use_sim_time", true),
      rclcpp::Parameter("field_arrows_extent", 0.0),
    };
    for (const rclcpp::Parameter & extra : setup.parameters) {
      parameters.push_back(extra);
    }
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    follower_ = std::make_shared<Follower3D>(options);
    static std::int64_t next_start_ns = kStartNs;
    t_ns_ = next_start_ns;
    next_start_ns += kTestSpacingNs;
    robot_ = std::make_unique<FakeDrone>(setup.start);
    executor_.add_node(follower_->get_node_base_interface());
    executor_.add_node(robot_->node());
    spinner_ = std::thread([this]() { executor_.spin(); });
    client_ = rclcpp_action::create_client<FollowCurve3>(robot_->node(), "follow_curve_3d");
    const std::uint8_t configured = follower_->configure().id();
    if (!activate || configured != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      return configured;
    }
    EXPECT_EQ(follower_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_TRUE(client_->wait_for_action_server(kWallTimeout));
    EXPECT_TRUE(waitFor([this]() { return robot_->connected(); })) << "topics not matched";
    tickFor(0.5);
    return lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  double now() const { return seconds(t_ns_); }

  void tick()
  {
    t_ns_ += kPeriodNs;
    robot_->step(t_ns_, kPeriod);
    const CycleSnapshot3 before = follower_->lastCycle();
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
      const CycleSnapshot3 snapshot = follower_->lastCycle();
      return !before.executing || snapshot.cycles > before.cycles || snapshot.goal != before.goal;
    }))
      << "no control cycle at t = " << now() << ": " << describe(follower_->lastCycle());
    ASSERT_TRUE(
      waitFor([&]() { return robot_->commands() == follower_->lastCycle().commands_sent; }))
      << "the commands did not reach the robot";
  }

  void tickFor(double duration)
  {
    const auto ticks = static_cast<int>(std::lround(duration / kPeriod));
    for (int i = 0; i < ticks && !::testing::Test::HasFatalFailure(); ++i) {
      tick();
    }
  }

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

  std::shared_ptr<GoalHandle> send(const FollowCurve3::Goal & goal)
  {
    const CycleSnapshot3 before = follower_->lastCycle();
    auto future = client_->async_send_goal(goal);
    if (future.wait_for(kWallTimeout) != std::future_status::ready) {
      ADD_FAILURE() << "goal not answered";
      return nullptr;
    }
    auto handle = future.get();
    if (!handle) {
      ADD_FAILURE() << "goal rejected (an active follower never rejects goals)";
      return handle;
    }
    const bool taken = waitFor([&]() {
      return before.executing ? follower_->preemptPending()
                              : follower_->lastCycle().goal > before.goal;
    });
    if (!taken) {
      ADD_FAILURE() << "the follower did not take the goal: " << describe(follower_->lastCycle());
    }
    return handle;
  }

  void cancel(const std::shared_ptr<GoalHandle> & handle)
  {
    auto response = client_->async_cancel_goal(handle);
    ASSERT_EQ(response.wait_for(kWallTimeout), std::future_status::ready) << "cancel not answered";
    EXPECT_EQ(response.get()->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  }

  ResultFuture resultOf(const std::shared_ptr<GoalHandle> & handle)
  {
    return client_->async_get_result(handle);
  }

  static bool ready(const ResultFuture & future)
  {
    return future.valid() && future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }

  WrappedResult waitResult(const ResultFuture & future, double max_duration)
  {
    const bool ended =
      tickUntil([&]() { return ready(future) || !follower_->lastCycle().executing; }, max_duration);
    if (!ended || !waitFor([&]() { return ready(future); })) {
      ADD_FAILURE() << "no result within " << max_duration
                    << " s of simulated time: " << describe(follower_->lastCycle());
      return WrappedResult{};
    }
    return future.get();
  }

  WrappedResult run(const FollowCurve3::Goal & goal, double max_duration)
  {
    const auto handle = send(goal);
    if (!handle) {
      return WrappedResult{};
    }
    return waitResult(resultOf(handle), max_duration);
  }

  void expectSilence(double duration = 1.0)
  {
    for (int i = 0; i < 5; ++i) {
      tick();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const std::size_t before = robot_->commands();
    tickFor(duration);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(robot_->commands(), before) << "the follower must be silent between goals";
  }

  rclcpp::executors::MultiThreadedExecutor executor_;
  std::shared_ptr<Follower3D> follower_;
  std::unique_ptr<FakeDrone> robot_;
  rclcpp_action::Client<FollowCurve3>::SharedPtr client_;
  std::thread spinner_;
  std::int64_t t_ns_{kStartNs};
};

}  // namespace vectorfield_follower_3d_test

#endif  // FOLLOWER_3D_HARNESS_HPP_
