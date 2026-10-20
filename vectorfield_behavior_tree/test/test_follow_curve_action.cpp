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

// The FollowCurve node in a tree built from XML, against a fake follow_curve server (the
// pattern of nav2_behavior_tree's own action-node tests): ports to goal, results to outputs,
// halt to cancel, a changed blackboard curve to a preempting goal, and ports that make no goal.
// The plugin is loaded by path through BT_REGISTER_NODES, as bt_navigator loads it.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "vectorfield_behavior_tree/bt_compat.hpp"
#include "vectorfield_behavior_tree/follow_curve_action.hpp"
#include "vectorfield_msgs/action/follow_curve.hpp"
#include "vectorfield_msgs/msg/curve.hpp"

namespace
{

using namespace std::chrono_literals;
using FollowCurve = vectorfield_msgs::action::FollowCurve;
using Curve = vectorfield_msgs::msg::Curve;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowCurve>;
namespace compat = vectorfield_behavior_tree::compat;

constexpr double kDistance = 4.25;

/// A follow_curve server whose goals succeed, abort or run until canceled. Like the follower's
/// SimpleActionServer, it accepts every goal and aborts the running goal, with an empty result,
/// when a new one arrives.
class FakeFollowCurveServer
{
public:
  enum class Mode : std::uint8_t
  {
    kSucceed,
    kAbort,
    kRunUntilCanceled
  };

  explicit FakeFollowCurveServer(const rclcpp::Node::SharedPtr & node)
  {
    server_ = rclcpp_action::create_server<FollowCurve>(
      node, "follow_curve",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowCurve::Goal> goal) {
        const std::lock_guard<std::mutex> lock(mutex_);
        goals_.push_back(*goal);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<GoalHandle>) {
        const std::lock_guard<std::mutex> lock(mutex_);
        ++cancel_requests_;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandle> handle) {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (current_ && current_->is_active()) {
          current_->abort(std::make_shared<FollowCurve::Result>());
        }
        current_ = std::move(handle);
      });
    timer_ = node->create_wall_timer(10ms, [this] { step(); });
  }

  void reset(Mode mode, std::uint16_t error_code = FollowCurve::Result::NONE)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    mode_ = mode;
    error_code_ = error_code;
    goals_.clear();
    cancel_requests_ = 0;
    canceled_ = 0;
  }

  void setMode(Mode mode)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    mode_ = mode;
  }

  std::vector<FollowCurve::Goal> goals() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return goals_;
  }

  int cancelRequests() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return cancel_requests_;
  }

  int canceled() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return canceled_;
  }

private:
  void step()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!current_ || !current_->is_active()) {
      return;
    }
    auto result = std::make_shared<FollowCurve::Result>();
    if (current_->is_canceling()) {
      current_->canceled(result);
      ++canceled_;
    } else if (mode_ == Mode::kSucceed) {
      result->laps_completed = current_->get_goal()->laps;
      result->distance_travelled = kDistance;
      current_->succeed(result);
    } else if (mode_ == Mode::kAbort) {
      result->error_code = error_code_;
      result->error_msg = error_code_ == FollowCurve::Result::NONE ? "" : "blocked";
      result->laps_completed = 1;
      current_->abort(result);
    } else {
      return;
    }
    current_.reset();
  }

  mutable std::mutex mutex_;
  rclcpp_action::Server<FollowCurve>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<GoalHandle> current_;
  std::vector<FollowCurve::Goal> goals_;
  Mode mode_{Mode::kSucceed};
  std::uint16_t error_code_{FollowCurve::Result::NONE};
  int cancel_requests_{0};
  int canceled_{0};
};

class FollowCurveActionTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
    server_node = std::make_shared<rclcpp::Node>("fake_follow_curve_server");
    server = std::make_unique<FakeFollowCurveServer>(server_node);
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(server_node);
    spinner = std::thread([] { executor->spin(); });
    client_node = std::make_shared<compat::BtNode>("follow_curve_bt_test");
    factory = std::make_unique<BT::BehaviorTreeFactory>();
    factory->registerFromPlugin(VF_TEST_PLUGIN);
  }

  static void TearDownTestSuite()
  {
    executor->cancel();
    spinner.join();
    factory.reset();
    executor.reset();
    server.reset();
    server_node.reset();
    client_node.reset();
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    server->reset(FakeFollowCurveServer::Mode::kSucceed);
    blackboard = BT::Blackboard::create();
    blackboard->set<compat::BtNode::SharedPtr>("node", client_node);
    blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
    blackboard->set<std::chrono::milliseconds>("server_timeout", 1000ms);
    blackboard->set<std::chrono::milliseconds>("cancel_timeout", 1000ms);
    blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", 5000ms);
  }

  [[nodiscard]] BT::Tree makeTree(const std::string & node_xml) const
  {
    const std::string xml =
      R"(<root BTCPP_format="4" main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree">)" +
      node_xml + "</BehaviorTree></root>";
    return factory->createTreeFromText(xml, blackboard);
  }

  /// Ticks until `done` holds or the tree leaves RUNNING; returns the last status.
  template <class Predicate>
  static BT::NodeStatus tickUntil(BT::Tree & tree, Predicate done)
  {
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < deadline) {
      status = compat::tickOnce(tree);
      if (status != BT::NodeStatus::RUNNING || done()) {
        break;
      }
      std::this_thread::sleep_for(10ms);
    }
    return status;
  }

  static BT::NodeStatus tickUntilDone(BT::Tree & tree)
  {
    return tickUntil(tree, [] { return false; });
  }

  static Curve ellipse(double a)
  {
    Curve curve;
    curve.header.frame_id = "map";
    curve.type = Curve::BUILTIN;
    curve.closed = true;
    curve.builtin_shape = "ellipse";
    curve.parameter_names = {"a", "b"};
    curve.parameter_values = {a, 1.0};
    return curve;
  }

  static rclcpp::Node::SharedPtr server_node;
  static std::unique_ptr<FakeFollowCurveServer> server;
  static std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  static std::thread spinner;
  static compat::BtNode::SharedPtr client_node;
  static std::unique_ptr<BT::BehaviorTreeFactory> factory;
  BT::Blackboard::Ptr blackboard;
};

rclcpp::Node::SharedPtr FollowCurveActionTest::server_node;
std::unique_ptr<FakeFollowCurveServer> FollowCurveActionTest::server;
std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> FollowCurveActionTest::executor;
std::thread FollowCurveActionTest::spinner;
compat::BtNode::SharedPtr FollowCurveActionTest::client_node;
std::unique_ptr<BT::BehaviorTreeFactory> FollowCurveActionTest::factory;

TEST(ParseShapeParameters, NamesAndValuesInOrder)
{
  std::vector<std::string> names;
  std::vector<double> values;
  EXPECT_FALSE(
    vectorfield_behavior_tree::parseShapeParameters(" a = 2.0 ; b=1.5,n=4;", names, values)
      .has_value());
  EXPECT_EQ(names, (std::vector<std::string>{"a", "b", "n"}));
  EXPECT_EQ(values, (std::vector<double>{2.0, 1.5, 4.0}));
  EXPECT_FALSE(vectorfield_behavior_tree::parseShapeParameters("", names, values).has_value());
  EXPECT_TRUE(names.empty());
  EXPECT_TRUE(values.empty());
}

TEST(ParseShapeParameters, MalformedTextIsRejected)
{
  std::vector<std::string> names;
  std::vector<double> values;
  for (const char * text :
       {"a", "=2", "a=", "a=x", "a=2;a=3", "a=nan", "a=inf", "a=1e400", "a=2 m"}) {
    EXPECT_TRUE(vectorfield_behavior_tree::parseShapeParameters(text, names, values).has_value())
      << text;
  }
}

TEST_F(FollowCurveActionTest, BuiltinShapePortsMakeTheGoalAndSuccessSetsTheOutputs)
{
  auto tree = makeTree(
    R"(<FollowCurve shape="ellipse" parameters="a=2.0; b=1.0" frame_id="odom" laps="2")"
    R"( desired_speed="0.3" reverse="true" max_duration="12.5" error_code_id="{code}")"
    R"( error_msg="{message}" laps_completed="{laps}" distance_travelled="{distance}"/>)");
  EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::SUCCESS);

  const auto goals = server->goals();
  ASSERT_EQ(goals.size(), 1U);
  const auto & goal = goals.front();
  EXPECT_EQ(goal.curve.type, Curve::BUILTIN);
  EXPECT_TRUE(goal.curve.closed);
  EXPECT_EQ(goal.curve.builtin_shape, "ellipse");
  EXPECT_EQ(goal.curve.header.frame_id, "odom");
  EXPECT_EQ(goal.curve.parameter_names, (std::vector<std::string>{"a", "b"}));
  EXPECT_EQ(goal.curve.parameter_values, (std::vector<double>{2.0, 1.0}));
  EXPECT_EQ(goal.laps, 2U);
  EXPECT_DOUBLE_EQ(goal.desired_speed, 0.3);
  EXPECT_TRUE(goal.reverse_direction);
  EXPECT_EQ(goal.frame_motion, FollowCurve::Goal::FRAME_STATIC);
  EXPECT_EQ(goal.frame_odom_topic, "");
  EXPECT_EQ(goal.max_duration.sec, 12);
  EXPECT_EQ(goal.max_duration.nanosec, 500000000U);

  EXPECT_EQ(blackboard->get<std::uint16_t>("code"), FollowCurve::Result::NONE);
  EXPECT_EQ(blackboard->get<std::string>("message"), "");
  EXPECT_EQ(blackboard->get<std::uint32_t>("laps"), 2U);
  EXPECT_DOUBLE_EQ(blackboard->get<double>("distance"), kDistance);
}

TEST_F(FollowCurveActionTest, DefaultsFollowOneLapInTheFollowersFrame)
{
  auto tree = makeTree(R"(<FollowCurve shape="lemniscate_bernoulli"/>)");
  EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::SUCCESS);
  const auto goals = server->goals();
  ASSERT_EQ(goals.size(), 1U);
  EXPECT_EQ(goals.front().curve.builtin_shape, "lemniscate_bernoulli");
  EXPECT_TRUE(goals.front().curve.parameter_names.empty());
  EXPECT_EQ(goals.front().curve.header.frame_id, "");
  EXPECT_EQ(goals.front().laps, 1U);
  EXPECT_DOUBLE_EQ(goals.front().desired_speed, 0.0);
  EXPECT_FALSE(goals.front().reverse_direction);
  EXPECT_EQ(goals.front().max_duration.sec, 0);
  EXPECT_EQ(goals.front().max_duration.nanosec, 0U);
}

TEST_F(FollowCurveActionTest, BlackboardCurveIsSentAsIs)
{
  Curve curve;
  curve.header.frame_id = "map";
  curve.type = Curve::POLYLINE;
  curve.closed = false;
  for (const auto & [x, y] : std::vector<std::pair<double, double>>{{0, 0}, {1, 0}, {2, 1}}) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    curve.points.push_back(point);
  }
  curve.sample_spacing = 0.05;
  blackboard->set<Curve>("path_curve", curve);
  auto tree = makeTree(
    R"(<FollowCurve curve="{path_curve}" laps="0" frame_motion="moving")"
    R"( frame_odom_topic="/frame/odom" server_name="follow_curve"/>)");
  EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::SUCCESS);
  const auto goals = server->goals();
  ASSERT_EQ(goals.size(), 1U);
  EXPECT_EQ(goals.front().curve, curve);
  EXPECT_EQ(goals.front().laps, 0U);
  EXPECT_EQ(goals.front().frame_motion, FollowCurve::Goal::FRAME_MOVING);
  EXPECT_EQ(goals.front().frame_odom_topic, "/frame/odom");
}

TEST_F(FollowCurveActionTest, AbortFailsWithTheResultsErrorCode)
{
  server->reset(FakeFollowCurveServer::Mode::kAbort, FollowCurve::Result::BLOCKED);
  auto tree = makeTree(
    R"(<FollowCurve shape="ellipse" error_code_id="{code}" error_msg="{message}")"
    R"( laps_completed="{laps}"/>)");
  EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(server->goals().size(), 1U);
  EXPECT_EQ(blackboard->get<std::uint16_t>("code"), FollowCurve::Result::BLOCKED);
  EXPECT_EQ(blackboard->get<std::string>("message"), "blocked");
  EXPECT_EQ(blackboard->get<std::uint32_t>("laps"), 1U);
}

TEST_F(FollowCurveActionTest, AbortWithoutAnErrorCodeReadsUnknown)
{
  server->reset(FakeFollowCurveServer::Mode::kAbort, FollowCurve::Result::NONE);
  auto tree =
    makeTree(R"(<FollowCurve shape="ellipse" error_code_id="{code}" error_msg="{message}"/>)");
  EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(blackboard->get<std::uint16_t>("code"), FollowCurve::Result::UNKNOWN);
  EXPECT_FALSE(blackboard->get<std::string>("message").empty());
}

TEST_F(FollowCurveActionTest, HaltCancelsTheRunningGoal)
{
  server->reset(FakeFollowCurveServer::Mode::kRunUntilCanceled);
  auto tree = makeTree(R"(<FollowCurve shape="ellipse" laps="0"/>)");
  EXPECT_EQ(tickUntil(tree, [] { return !server->goals().empty(); }), BT::NodeStatus::RUNNING);
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(compat::tickOnce(tree), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(server->cancelRequests(), 0);
  tree.haltTree();
  EXPECT_EQ(server->cancelRequests(), 1);
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (server->canceled() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(server->canceled(), 1);
  EXPECT_EQ(tree.rootNode()->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(server->goals().size(), 1U);
}

TEST_F(FollowCurveActionTest, ChangedBlackboardCurvePreemptsTheGoal)
{
  server->reset(FakeFollowCurveServer::Mode::kRunUntilCanceled);
  blackboard->set<Curve>("curve", ellipse(2.0));
  auto tree = makeTree(R"(<FollowCurve curve="{curve}" laps="1"/>)");
  EXPECT_EQ(tickUntil(tree, [] { return !server->goals().empty(); }), BT::NodeStatus::RUNNING);
  // The same curve again sends nothing.
  blackboard->set<Curve>("curve", ellipse(2.0));
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(compat::tickOnce(tree), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(server->goals().size(), 1U);

  blackboard->set<Curve>("curve", ellipse(3.0));
  EXPECT_EQ(tickUntil(tree, [] { return server->goals().size() == 2; }), BT::NodeStatus::RUNNING);
  const auto goals = server->goals();
  ASSERT_EQ(goals.size(), 2U);
  EXPECT_EQ(goals.back().curve, ellipse(3.0));
  server->setMode(FakeFollowCurveServer::Mode::kSucceed);
  EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(server->goals().size(), 2U);
}

TEST_F(FollowCurveActionTest, PortsThatMakeNoGoalFailWithoutSendingOne)
{
  blackboard->set<Curve>("curve", ellipse(2.0));
  const char * outputs = R"( error_code_id="{code}" error_msg="{message}"/>)";
  for (const char * ports : {
         "",
         R"( shape="ellipse" curve="{curve}")",
         R"( curve="{no_such_entry}")",
         R"( shape="ellipse" parameters="a=x")",
         R"( shape="ellipse" parameters="a")",
         R"( shape="ellipse" frame_motion="spinning")",
         R"( shape="ellipse" max_duration="-1")",
       }) {
    SCOPED_TRACE(ports);
    auto tree = makeTree(std::string("<FollowCurve") + ports + outputs);
    EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::FAILURE);
    EXPECT_EQ(blackboard->get<std::uint16_t>("code"), FollowCurve::Result::INVALID_GOAL);
    EXPECT_FALSE(blackboard->get<std::string>("message").empty());
  }
  EXPECT_TRUE(server->goals().empty());
}

TEST_F(FollowCurveActionTest, OutputsAreResetWhenAGoalStarts)
{
  server->reset(FakeFollowCurveServer::Mode::kAbort, FollowCurve::Result::BLOCKED);
  auto tree =
    makeTree(R"(<FollowCurve shape="ellipse" error_code_id="{code}" error_msg="{message}"/>)");
  EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(blackboard->get<std::uint16_t>("code"), FollowCurve::Result::BLOCKED);
  server->setMode(FakeFollowCurveServer::Mode::kRunUntilCanceled);
  EXPECT_EQ(compat::tickOnce(tree), BT::NodeStatus::RUNNING);
  EXPECT_EQ(blackboard->get<std::uint16_t>("code"), FollowCurve::Result::NONE);
  EXPECT_EQ(blackboard->get<std::string>("message"), "");
  tree.haltTree();
}

}  // namespace
