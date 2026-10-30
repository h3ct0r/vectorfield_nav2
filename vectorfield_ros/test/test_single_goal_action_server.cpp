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

// SingleGoalActionServer against a real rclcpp_action client: goals sent back to back from the
// previous result all run (Nav2's SimpleActionServer can lose one), preemption, a replaced pending
// goal, cancels, execute returning early or throwing, deactivation, destruction and a context
// shutdown. Every test owns a private context, which also checks that the server's executor waits
// on the node's context rather than the global one.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "vectorfield_msgs/action/follow_curve.hpp"
#include "vectorfield_ros/compat.hpp"
#include "vectorfield_ros/single_goal_action_server.hpp"

namespace
{

using namespace std::chrono_literals;
using FollowCurve = vectorfield_msgs::action::FollowCurve;
using Server = vectorfield_ros::SingleGoalActionServer<FollowCurve>;
using Client = rclcpp_action::Client<FollowCurve>;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<FollowCurve>;
using rclcpp_action::ResultCode;

/// Waits until `done()` holds, at most `timeout`.
template <typename Predicate>
bool waitFor(Predicate done, std::chrono::milliseconds timeout = 10s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!done()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(1ms);
  }
  return true;
}

/// A one-shot gate between the test and an execute callback.
class Latch
{
public:
  void open()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    open_ = true;
    cv_.notify_all();
  }
  bool wait(std::chrono::milliseconds timeout = 10s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [this]() { return open_; });
  }
  bool isOpen()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return open_;
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool open_{false};
};

/// A server and a client node on one private context; goals are told apart by `laps`.
class SingleGoalActionServerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    options_.context(context_);
    server_node_ = std::make_shared<rclcpp::Node>("vf_goal_server", options_);
    client_node_ = std::make_shared<rclcpp::Node>("vf_goal_client", options_);
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    client_executor_ =
      std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    client_executor_->add_node(client_node_);
    client_thread_ = std::thread([this]() { client_executor_->spin(); });
    client_ = rclcpp_action::create_client<FollowCurve>(client_node_, "vf_goal_action");
  }

  void TearDown() override
  {
    server_owner_.reset();
    // The client executor's spin() ends at the context shutdown, even one that comes first.
    context_->shutdown("test done");
    client_thread_.join();
    client_.reset();
    client_executor_.reset();
  }

  /// Creates the server on `node` (the plain server node by default), active unless told not.
  /// Execute callbacks use server_, which stays valid while server_owner_ destroys it.
  template <typename NodeT>
  void makeServer(
    const NodeT & node, std::chrono::milliseconds timeout = 100ms, bool active = true,
    const std::string & action = "vf_goal_action")
  {
    server_owner_ = std::make_unique<Server>(
      node, action,
      [this]() {
        ++executions_;
        execute_();
      },
      timeout);
    server_ = server_owner_.get();
    if (active) {
      server_->activate();
    }
    ASSERT_TRUE(client_->wait_for_action_server(10s));
  }
  void makeServer(std::chrono::milliseconds timeout = 100ms, bool active = true)
  {
    makeServer(server_node_, timeout, active);
  }

  /// Sends goal `laps` without waiting; `on_result` runs in the client's executor thread after
  /// the result is recorded. Usable from a result callback.
  std::shared_future<ClientGoalHandle::SharedPtr> sendAsync(
    std::uint32_t laps, std::function<void(const ClientGoalHandle::WrappedResult &)> on_result = {})
  {
    FollowCurve::Goal goal;
    goal.laps = laps;
    Client::SendGoalOptions options;
    options.result_callback = [this, laps, on_result](const auto & result) {
      {
        const std::lock_guard<std::mutex> lock(results_mutex_);
        results_[laps] = result;
        result_times_[laps] = std::chrono::steady_clock::now();
      }
      if (on_result) {
        on_result(result);
      }
    };
    return client_->async_send_goal(goal, options);
  }

  /// Sends goal `laps` and returns its handle (nullptr when rejected).
  ClientGoalHandle::SharedPtr send(std::uint32_t laps)
  {
    auto future = sendAsync(laps);
    if (future.wait_for(10s) != std::future_status::ready) {
      ADD_FAILURE() << "goal " << laps << " not answered";
      return nullptr;
    }
    return future.get();
  }

  /// The result code of goal `laps`, waiting for it at most `timeout`.
  ResultCode resultOf(std::uint32_t laps, std::chrono::milliseconds timeout = 10s)
  {
    if (!waitFor([&]() { return hasResult(laps); }, timeout)) {
      ADD_FAILURE() << "no result for goal " << laps;
      return ResultCode::UNKNOWN;
    }
    const std::lock_guard<std::mutex> lock(results_mutex_);
    return results_[laps].code;
  }
  bool hasResult(std::uint32_t laps)
  {
    const std::lock_guard<std::mutex> lock(results_mutex_);
    return results_.count(laps) != 0U;
  }
  FollowCurve::Result resultMessage(std::uint32_t laps)
  {
    const std::lock_guard<std::mutex> lock(results_mutex_);
    return *results_[laps].result;
  }
  std::uint32_t currentLaps() const { return server_->get_current_goal()->laps; }

  rclcpp::Context::SharedPtr context_;
  rclcpp::NodeOptions options_;
  rclcpp::Node::SharedPtr server_node_;
  rclcpp::Node::SharedPtr client_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> client_executor_;
  std::thread client_thread_;
  Client::SharedPtr client_;
  std::unique_ptr<Server> server_owner_;
  Server * server_ = nullptr;  ///< server_owner_'s, still valid while it is destroyed
  std::function<void()> execute_ = []() {};
  std::atomic<int> executions_{0};
  std::mutex results_mutex_;
  std::map<std::uint32_t, ClientGoalHandle::WrappedResult> results_;
  std::map<std::uint32_t, std::chrono::steady_clock::time_point> result_times_;
};

TEST_F(SingleGoalActionServerTest, GoalsSentFromThePreviousResultAllRun)
{
  execute_ = [this]() { server_->succeeded_current(); };
  makeServer();
  constexpr std::uint32_t kGoals = 300;
  // Each goal goes out from the previous goal's result callback, the moment the worker finishes
  // with that goal: where Nav2's SimpleActionServer can lose one.
  std::function<void(std::uint32_t)> next = [&](std::uint32_t laps) {
    sendAsync(laps, [&, laps](const auto &) {
      if (laps < kGoals) {
        next(laps + 1);
      }
    });
  };
  next(1);
  ASSERT_TRUE(waitFor([&]() { return hasResult(kGoals); }, 60s)) << "a goal never ran";
  for (std::uint32_t laps = 1; laps <= kGoals; ++laps) {
    EXPECT_EQ(resultOf(laps), ResultCode::SUCCEEDED) << "goal " << laps;
  }
  EXPECT_EQ(executions_.load(), static_cast<int>(kGoals));
}

TEST_F(SingleGoalActionServerTest, AGoalAcceptedWhileExecuteReturnsRunsNext)
{
  Latch release;
  execute_ = [&]() {
    const std::uint32_t laps = currentLaps();
    server_->succeeded_current();
    if (laps == 1) {
      release.wait();  // goal 2 arrives here, after goal 1's result and before execute returns
    }
  };
  makeServer();
  ASSERT_NE(send(1), nullptr);
  ASSERT_EQ(resultOf(1), ResultCode::SUCCEEDED);
  ASSERT_NE(send(2), nullptr);
  ASSERT_TRUE(server_->is_preempt_requested());
  release.open();
  EXPECT_EQ(resultOf(2), ResultCode::SUCCEEDED);
  EXPECT_EQ(executions_.load(), 2);
}

TEST_F(SingleGoalActionServerTest, ANewGoalPreemptsTheRunningOne)
{
  Latch started;
  std::uint32_t taken = 0;
  execute_ = [&]() {
    started.open();
    ASSERT_TRUE(waitFor([&]() { return server_->is_preempt_requested(); }));
    taken = server_->accept_pending_goal()->laps;
    server_->succeeded_current();
  };
  makeServer();
  ASSERT_NE(send(1), nullptr);
  ASSERT_TRUE(started.wait());
  ASSERT_NE(send(2), nullptr);
  EXPECT_EQ(resultOf(1), ResultCode::ABORTED);
  EXPECT_EQ(resultMessage(1), FollowCurve::Result());  // an empty result
  EXPECT_EQ(resultOf(2), ResultCode::SUCCEEDED);
  EXPECT_EQ(taken, 2U);
  EXPECT_EQ(executions_.load(), 1);
}

TEST_F(SingleGoalActionServerTest, ANewerPendingGoalReplacesAnOlderOne)
{
  Latch started;
  Latch release;
  execute_ = [&]() {
    started.open();
    ASSERT_TRUE(release.wait());
    const auto pending = server_->accept_pending_goal();
    ASSERT_NE(pending, nullptr);
    EXPECT_EQ(pending->laps, 3U);
    server_->succeeded_current();
  };
  makeServer();
  ASSERT_NE(send(1), nullptr);
  ASSERT_TRUE(started.wait());
  ASSERT_NE(send(2), nullptr);
  ASSERT_NE(send(3), nullptr);
  EXPECT_EQ(resultOf(2), ResultCode::ABORTED);  // replaced while goal 1 still runs
  EXPECT_FALSE(hasResult(1));
  release.open();
  EXPECT_EQ(resultOf(1), ResultCode::ABORTED);
  EXPECT_EQ(resultOf(3), ResultCode::SUCCEEDED);
}

TEST_F(SingleGoalActionServerTest, ACancelledRunningGoalEndsCanceled)
{
  Latch started;
  execute_ = [&]() {
    started.open();
    ASSERT_TRUE(waitFor([&]() { return server_->is_cancel_requested(); }));
    server_->terminate_current();
  };
  makeServer();
  const auto handle = send(1);
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(started.wait());
  client_->async_cancel_goal(handle);
  EXPECT_EQ(resultOf(1), ResultCode::CANCELED);
}

TEST_F(SingleGoalActionServerTest, ACancelledPendingGoalEndsCanceledAndTheRunningOneGoesOn)
{
  Latch started;
  Latch release;
  std::atomic<bool> took_pending{true};
  execute_ = [&]() {
    started.open();
    ASSERT_TRUE(release.wait());
    // The running goal's own cancel flag stays clear (Nav2's server reported the pending one's).
    EXPECT_FALSE(server_->is_cancel_requested());
    took_pending = server_->accept_pending_goal() != nullptr;
    server_->succeeded_current();
  };
  makeServer();
  ASSERT_NE(send(1), nullptr);
  ASSERT_TRUE(started.wait());
  const auto pending = send(2);
  ASSERT_NE(pending, nullptr);
  auto cancel = client_->async_cancel_goal(pending);
  ASSERT_EQ(cancel.wait_for(10s), std::future_status::ready);
  release.open();
  EXPECT_EQ(resultOf(2), ResultCode::CANCELED);
  EXPECT_EQ(resultOf(1), ResultCode::SUCCEEDED);
  EXPECT_FALSE(took_pending.load());
}

TEST_F(SingleGoalActionServerTest, AGoalLeftActiveByExecuteIsAborted)
{
  execute_ = [&]() {
    if (currentLaps() == 2) {
      server_->succeeded_current();
    }
  };
  makeServer();
  ASSERT_NE(send(1), nullptr);
  EXPECT_EQ(resultOf(1), ResultCode::ABORTED);
  ASSERT_NE(send(2), nullptr);
  EXPECT_EQ(resultOf(2), ResultCode::SUCCEEDED);
}

TEST_F(SingleGoalActionServerTest, AnExceptionFromExecuteEndsEveryGoal)
{
  execute_ = [&]() {
    if (currentLaps() == 1) {
      ASSERT_TRUE(waitFor([&]() { return server_->is_preempt_requested(); }));
      throw std::runtime_error("execute failed on purpose");
    }
    server_->succeeded_current();
  };
  makeServer();
  ASSERT_NE(send(1), nullptr);
  ASSERT_NE(send(2), nullptr);
  EXPECT_EQ(resultOf(1), ResultCode::ABORTED);
  EXPECT_EQ(resultOf(2), ResultCode::ABORTED);
  ASSERT_NE(send(3), nullptr);  // the server still serves
  EXPECT_EQ(resultOf(3), ResultCode::SUCCEEDED);
}

TEST_F(SingleGoalActionServerTest, DeactivateEndsTheGoalAtTheTimeoutAndWaitsForExecute)
{
  Latch started;
  std::atomic<bool> execute_returned{false};
  execute_ = [&]() {
    if (currentLaps() == 1) {
      started.open();
      std::this_thread::sleep_for(1s);  // ignores the deactivation
      execute_returned = true;
      return;
    }
    server_->succeeded_current();
  };
  makeServer(100ms);
  ASSERT_NE(send(1), nullptr);
  ASSERT_TRUE(started.wait());
  const auto start = std::chrono::steady_clock::now();
  server_->deactivate();
  EXPECT_TRUE(execute_returned.load()) << "deactivate() returned while execute ran";
  ASSERT_EQ(resultOf(1), ResultCode::ABORTED);
  {
    const std::lock_guard<std::mutex> lock(results_mutex_);
    EXPECT_LT(result_times_[1] - start, 700ms) << "the goal was not ended at the timeout";
  }
  EXPECT_FALSE(server_->is_server_active());
  EXPECT_EQ(send(2), nullptr) << "an inactive server accepted a goal";
  server_->activate();
  ASSERT_NE(send(3), nullptr);
  EXPECT_EQ(resultOf(3), ResultCode::SUCCEEDED);
}

TEST_F(SingleGoalActionServerTest, AnInactiveServerRejectsGoals)
{
  makeServer(100ms, false);
  EXPECT_EQ(send(1), nullptr);
  EXPECT_EQ(executions_.load(), 0);
}

TEST_F(SingleGoalActionServerTest, DestroyingTheServerMidGoalEndsIt)
{
  Latch started;
  execute_ = [&]() {
    started.open();
    // As the followers do: the goal runs until the server is deactivated.
    while (server_->is_server_active()) {
      std::this_thread::sleep_for(5ms);
    }
  };
  makeServer();
  ASSERT_NE(send(1), nullptr);
  ASSERT_TRUE(started.wait());
  const auto start = std::chrono::steady_clock::now();
  server_owner_.reset();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
  EXPECT_EQ(resultOf(1), ResultCode::ABORTED);
}

TEST_F(SingleGoalActionServerTest, AContextShutdownMidGoalNeverHangs)
{
  Latch started;
  execute_ = [&]() {
    started.open();
    while (context_->is_valid()) {
      std::this_thread::sleep_for(5ms);
    }
    server_->terminate_current();  // its publishers throw now; the server only logs it
  };
  makeServer();
  ASSERT_NE(send(1), nullptr);
  ASSERT_TRUE(started.wait());
  const auto start = std::chrono::steady_clock::now();
  context_->shutdown("shutdown mid-goal");
  server_owner_.reset();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 2s);
}

TEST_F(SingleGoalActionServerTest, ACreatedAndDestroyedServerNeverHangs)
{
  for (int i = 0; i < 20; ++i) {
    const auto start = std::chrono::steady_clock::now();
    {
      Server server(server_node_, "vf_goal_action_" + std::to_string(i), []() {}, 100ms);
    }
    EXPECT_LT(std::chrono::steady_clock::now() - start, 1s) << "iteration " << i;
  }
}

TEST_F(SingleGoalActionServerTest, ServesOnALifecycleNode)
{
  execute_ = [this]() { server_->succeeded_current(); };
  const auto node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("vf_goal_lifecycle", options_);
  makeServer(node);
  ASSERT_NE(send(1), nullptr);
  EXPECT_EQ(resultOf(1), ResultCode::SUCCEEDED);
  server_owner_.reset();  // while the node lives
}

TEST_F(SingleGoalActionServerTest, ServesOnANav2LifecycleNode)
{
  execute_ = [this]() { server_->succeeded_current(); };
  const auto node =
    std::make_shared<vectorfield_ros::compat::LifecycleNode>("vf_goal_nav2", "", options_);
  makeServer(node);
  ASSERT_NE(send(1), nullptr);
  EXPECT_EQ(resultOf(1), ResultCode::SUCCEEDED);
  server_owner_.reset();  // while the node lives
}

}  // namespace
