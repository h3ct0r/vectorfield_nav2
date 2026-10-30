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
/// @brief An action server that runs one goal at a time, where a newer goal preempts the running
/// one: the followers' `follow_curve` and `follow_curve3` servers.
///
/// It keeps the method names and the semantics of Nav2's SimpleActionServer that the followers
/// use, without its lost goals. Nav2's worker thread decides to exit while it holds the server's
/// mutex, releases it and runs on for a moment; a goal accepted in that moment sees the thread
/// still running, waits as pending, and nothing ever runs it (nav2_util on Humble and Jazzy,
/// nav2_ros_common on Lyrical). Here one worker thread lives as long as the server and waits on a
/// condition variable under the mutex that goal acceptance takes, so an accepted goal is either
/// taken by the worker or wakes it.

#ifndef VECTORFIELD_ROS__SINGLE_GOAL_ACTION_SERVER_HPP_
#define VECTORFIELD_ROS__SINGLE_GOAL_ACTION_SERVER_HPP_

#include <chrono>
#include <condition_variable>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace vectorfield_ros
{

/// @brief An rclcpp_action server that runs one goal at a time in its own worker thread.
///
/// - **Goals:** rejected while inactive. While a goal runs, a new one waits as pending (replacing,
///   and aborting, an older pending goal) and is_preempt_requested() turns true; the execute
///   callback takes it with accept_pending_goal(), which ends the running goal with an empty
///   result. A pending goal still waiting when execute returns runs next.
/// - **Cancels:** always accepted; execute polls is_cancel_requested() (the running goal's) and
///   ends the goal itself. A cancelled pending goal ends CANCELED when it would be taken.
/// - **Execute:** a goal still active when execute returns is aborted; an exception from execute
///   is logged and ends every goal.
/// - **Threads:** the action's callbacks run in a dedicated executor thread, so they are served
///   while the node's own executor waits in deactivate(). Never call deactivate() from execute
///   expecting it to wait, and never destroy the server from execute.
/// @tparam ActionT The action type, e.g. `vectorfield_msgs::action::FollowCurve`.
template <typename ActionT>
class SingleGoalActionServer
{
public:
  using Goal = typename ActionT::Goal;
  using Result = typename ActionT::Result;
  using Feedback = typename ActionT::Feedback;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  /// Runs the current goal; called in the worker thread with no lock held.
  using ExecuteCallback = std::function<void()>;

  /// @brief Creates the server, inactive.
  /// @tparam NodeT Pointer-like node (shared_ptr or raw pointer to any rclcpp or rclcpp_lifecycle
  /// node, including Nav2's lifecycle nodes).
  /// @param node Node that owns the server.
  /// @param action_name Action name (relative names resolve in the node's namespace).
  /// @param execute_callback Runs each goal (see ExecuteCallback).
  /// @param server_timeout How long deactivate() waits for execute before it ends the goals.
  template <typename NodeT>
  SingleGoalActionServer(
    const NodeT & node, const std::string & action_name, ExecuteCallback execute_callback,
    std::chrono::milliseconds server_timeout)
  : SingleGoalActionServer(
      node->get_node_base_interface(), node->get_node_clock_interface(),
      node->get_node_logging_interface(), node->get_node_waitables_interface(), action_name,
      std::move(execute_callback), server_timeout)
  {
  }

  /// @brief Creates the server from node interfaces, inactive.
  SingleGoalActionServer(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & node_clock,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logging,
    const rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr & node_waitables,
    const std::string & action_name, ExecuteCallback execute_callback,
    std::chrono::milliseconds server_timeout)
  : context_(node_base->get_context()),
    logger_(node_logging->get_logger().get_child(action_name)),
    execute_(std::move(execute_callback)),
    server_timeout_(server_timeout),
    stop_future_(stop_.get_future().share())
  {
    group_ = node_base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    server_ = rclcpp_action::create_server<ActionT>(
      node_base, node_clock, node_logging, node_waitables, action_name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Goal>) { return handleGoal(); },
      [](const std::shared_ptr<GoalHandle> &) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](std::shared_ptr<GoalHandle> handle) { handleAccepted(std::move(handle)); },
      rcl_action_server_get_default_options(), group_);
    // The executor waits on the node's context: the default, the global context, would neither
    // see a private context's shutdown nor serve its entities.
    rclcpp::ExecutorOptions options;
    options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(options);
    executor_->add_callback_group(group_, node_base);
    worker_ = std::thread([this]() { work(); });
    spinner_ = std::thread([this]() { spin(); });
  }

  SingleGoalActionServer(const SingleGoalActionServer &) = delete;
  SingleGoalActionServer & operator=(const SingleGoalActionServer &) = delete;

  /// @brief Deactivates (ends every goal), then stops both threads.
  ~SingleGoalActionServer()
  {
    deactivate();
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      exiting_ = true;
    }
    work_cv_.notify_all();
    worker_.join();
    // A cancel() that comes before spin_until_future_complete() starts is lost; the stop future
    // ends the spin loop at its next 100 ms wake-up instead.
    stop_.set_value();
    executor_->cancel();
    spinner_.join();
    executor_.reset();
    server_.reset();
  }

  /// @brief Starts accepting goals.
  void activate()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    active_ = true;
  }

  /// @brief Stops accepting goals, ends the pending one, and waits until execute has returned.
  ///
  /// When execute is still running after the server timeout, every goal is ended (the clients get
  /// their results) and the wait goes on. Afterwards execute is not running and does not run
  /// again until activate().
  void deactivate()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    active_ = false;
    terminateLocked(pending_);
    if (!busy_) {
      terminateLocked(current_);  // accepted, not started
    }
    if (std::this_thread::get_id() == worker_.get_id()) {
      return;
    }
    const auto idle = [this]() { return !busy_; };
    if (!idle_cv_.wait_for(lock, server_timeout_, idle)) {
      RCLCPP_WARN(
        logger_, "execute is still running %lld ms after deactivation; ending its goals",
        static_cast<long long>(server_timeout_.count()));
      terminateAllLocked();
      idle_cv_.wait(lock, idle);
    }
  }

  /// @return Whether the server accepts goals.
  bool is_server_active() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return active_;
  }

  /// @return Whether a pending goal waits for accept_pending_goal().
  bool is_preempt_requested() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return pending_ != nullptr;
  }

  /// @return The current goal, or nullptr when there is none.
  std::shared_ptr<const Goal> get_current_goal() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return current_ ? current_->get_goal() : nullptr;
  }

  /// @brief Makes the pending goal the current one, ending the current goal with an empty result
  /// (CANCELED if it was being cancelled, otherwise ABORTED).
  /// @return The new current goal, or nullptr when there was no pending goal or its client had
  /// cancelled it (it ends CANCELED and the current goal goes on).
  std::shared_ptr<const Goal> accept_pending_goal()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_) {
      return nullptr;
    }
    if (pending_->is_canceling()) {
      terminateLocked(pending_);
      return nullptr;
    }
    terminateLocked(current_);
    current_ = std::move(pending_);
    return current_->get_goal();
  }

  /// @return Whether the client of the current goal asked to cancel it.
  bool is_cancel_requested() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return current_ && current_->is_canceling();
  }

  /// @brief Publishes feedback for the current goal (nothing when there is none).
  void publish_feedback(const std::shared_ptr<Feedback> & feedback)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (current_ && current_->is_active()) {
      call([&]() { current_->publish_feedback(feedback); }, "publish feedback");
    }
  }

  /// @brief Ends the current goal SUCCEEDED (nothing when there is none).
  void succeeded_current(const std::shared_ptr<Result> & result = std::make_shared<Result>())
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (current_ && current_->is_active()) {
      call([&]() { current_->succeed(result); }, "succeed the goal");
    }
    current_.reset();
  }

  /// @brief Ends the current goal CANCELED if it was being cancelled, otherwise ABORTED (nothing
  /// when there is none).
  void terminate_current(const std::shared_ptr<Result> & result = std::make_shared<Result>())
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    terminateLocked(current_, result);
  }

  /// @brief Ends the current and the pending goal as terminate_current() does.
  void terminate_all(const std::shared_ptr<Result> & result = std::make_shared<Result>())
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    terminateAllLocked(result);
  }

private:
  rclcpp_action::GoalResponse handleGoal()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {
      RCLCPP_WARN(logger_, "the server is inactive; rejecting the goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Executing at once: rcl_action does not allow ACCEPTED -> ABORTED, and a pending goal can be
    // aborted before it runs.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void handleAccepted(std::shared_ptr<GoalHandle> handle)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {
      terminateLocked(handle);  // deactivated after handleGoal()
    } else if (busy_ || current_) {
      terminateLocked(pending_);
      pending_ = std::move(handle);
    } else {
      current_ = std::move(handle);
      work_cv_.notify_one();
    }
  }

  void work()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!exiting_) {
      // The timeout notices a context shutdown, which notifies nobody.
      work_cv_.wait_for(
        lock, std::chrono::milliseconds(100), [this]() { return exiting_ || current_; });
      if (exiting_) {
        break;
      }
      if (!rclcpp::ok(context_)) {
        terminateAllLocked();
        continue;
      }
      if (!current_) {
        continue;
      }
      busy_ = true;
      while (current_) {
        lock.unlock();
        bool failed = false;
        try {
          execute_();
        } catch (const std::exception & error) {
          RCLCPP_ERROR(logger_, "execute failed: %s", error.what());
          failed = true;
        }
        lock.lock();
        if (failed) {
          terminateAllLocked();
        } else if (current_ && current_->is_active()) {
          RCLCPP_WARN(logger_, "execute returned with its goal active; aborting it");
        }
        terminateLocked(current_);
        if (pending_ && (!active_ || pending_->is_canceling())) {
          terminateLocked(pending_);
        }
        current_ = std::move(pending_);
      }
      busy_ = false;
      idle_cv_.notify_all();
    }
  }

  void spin()
  {
    try {
      while (rclcpp::ok(context_) &&
             stop_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        executor_->spin_until_future_complete(stop_future_, std::chrono::milliseconds(100));
      }
    } catch (const rclcpp::exceptions::RCLError & error) {
      // A context shut down under the waiting executor invalidates the action server's
      // publishers, and the executor throws when it next checks the server.
      if (rclcpp::ok(context_)) {
        throw;
      }
      RCLCPP_DEBUG(logger_, "the executor stopped at shutdown: %s", error.what());
    }
  }

  /// Ends `handle` CANCELED if it was being cancelled, otherwise ABORTED, and resets it.
  void terminateLocked(
    std::shared_ptr<GoalHandle> & handle,
    const std::shared_ptr<Result> & result = std::make_shared<Result>())
  {
    if (handle && handle->is_active()) {
      if (handle->is_canceling()) {
        call([&]() { handle->canceled(result); }, "cancel the goal");
      } else {
        call([&]() { handle->abort(result); }, "abort the goal");
      }
    }
    handle.reset();
  }

  void terminateAllLocked(const std::shared_ptr<Result> & result = std::make_shared<Result>())
  {
    terminateLocked(current_, result);
    terminateLocked(pending_, result);
  }

  /// Runs an rclcpp_action call; after a shutdown its publishers throw, which is only logged.
  template <typename Call>
  void call(const Call & action, const char * what)
  {
    try {
      action();
    } catch (const std::exception & error) {
      if (rclcpp::ok(context_)) {
        RCLCPP_WARN(logger_, "could not %s: %s", what, error.what());
      } else {
        RCLCPP_DEBUG(logger_, "could not %s after shutdown: %s", what, error.what());
      }
    }
  }

  rclcpp::Context::SharedPtr context_;
  rclcpp::Logger logger_;
  ExecuteCallback execute_;
  std::chrono::milliseconds server_timeout_;

  mutable std::mutex mutex_;
  std::condition_variable work_cv_;  ///< A goal to run, or exiting.
  std::condition_variable idle_cv_;  ///< busy_ turned false.
  bool active_{false};
  bool exiting_{false};
  bool busy_{false};  ///< The worker runs goals: set and cleared under mutex_.
  std::shared_ptr<GoalHandle> current_;
  std::shared_ptr<GoalHandle> pending_;

  rclcpp::CallbackGroup::SharedPtr group_;
  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
  std::promise<void> stop_;
  std::shared_future<void> stop_future_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread worker_;
  std::thread spinner_;
};

}  // namespace vectorfield_ros

#endif  // VECTORFIELD_ROS__SINGLE_GOAL_ACTION_SERVER_HPP_
