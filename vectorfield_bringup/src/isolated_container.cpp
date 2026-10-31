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
/// @brief `isolated_container`: the demos' component container, which has Nav2's lifecycle
/// managers shut their nodes down before the context shuts down.
///
/// Every component spins on its own single-threaded executor, as in rclcpp_components'
/// component_container_isolated (`component_container --isolated` on Lyrical), with two changes:
///
/// - **Managed shutdown first.** With rclcpp's signal handler the context shuts down at once, and
///   Nav2's lifecycle nodes deactivate and clean up in a pre-shutdown callback, in the thread that
///   runs the shutdown, while their executors still run their callbacks: a cleanup then frees
///   what a running callback uses (planner_server's costmap, collision_monitor's sources,
///   docking_server), and the container crashes at exit. This container takes SIGINT and SIGTERM
///   itself. At the first one it asks every Nav2 lifecycle manager among its components, the last
///   loaded first, to shut its nodes down (`manage_nodes`, SHUTDOWN): each transition then runs
///   on the node's own executor, between its callbacks, as at any other time. Then it shuts the
///   context down, and the nodes' pre-shutdown callbacks find nothing left to do. A manager in the
///   middle of its start-up answers once it is done, and the nodes it has just activated get
///   kSettleTime before it deactivates them (kSettleTime has the reason). The managers get
///   kManagedShutdownTimeout in all; a SIGTERM (launch's escalation) cuts it short, a repeated
///   SIGINT does not (a terminal's Ctrl+C reaches the container from the terminal and again from
///   launch).
/// - **No wait for spin() after a shutdown.** Humble's ComponentManagerIsolated (rclcpp_components
///   16.0) stops an executor by waiting for it to spin before cancelling it; after a shutdown
///   signal the executors have already stopped, so it never leaves its destructor. This manager
///   stops waiting once the context is shut down, as Jazzy's does.

#include <pthread.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"
#include "rclcpp_components/component_manager.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;
using Trigger = std_srvs::srv::Trigger;

/// How long the lifecycle managers get, in all, to shut their nodes down after the signal: the
/// demos' simulators keep `/clock` for 4 s, and launch escalates to SIGTERM after 5 s.
constexpr std::chrono::seconds kManagedShutdownTimeout{4};

/// How long nodes a manager has just activated run before it deactivates them. A
/// tf2_ros::TransformListener with its own thread never leaves its destructor when it comes before
/// the thread spins (its executor's cancel() is lost), and Lyrical's following_server creates one
/// in on_activate(): a shutdown queued behind the start-up would deactivate it within a few ms.
constexpr std::chrono::milliseconds kSettleTime{200};

/// SIGINT and SIGTERM: blocked in every thread, taken by takeSignal().
sigset_t shutdownSignals()
{
  sigset_t signals;
  sigemptyset(&signals);
  sigaddset(&signals, SIGINT);
  sigaddset(&signals, SIGTERM);
  return signals;
}

/// Wait up to `timeout` for SIGINT or SIGTERM; return it, or 0.
int takeSignal(const sigset_t & signals, std::chrono::milliseconds timeout)
{
  timespec wait{};
  wait.tv_sec = static_cast<std::time_t>(timeout.count() / 1000);
  wait.tv_nsec = static_cast<long>((timeout.count() % 1000) * 1000000);  // NOLINT(runtime/int)
  const int signal = sigtimedwait(&signals, nullptr, &wait);
  return signal > 0 ? signal : 0;
}

class IsolatedComponentManager : public rclcpp_components::ComponentManager
{
public:
  explicit IsolatedComponentManager(std::weak_ptr<rclcpp::Executor> executor)
  : rclcpp_components::ComponentManager(std::move(executor)),
    shutdown_group_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false))
  {
    shutdown_executor_.add_callback_group(shutdown_group_, get_node_base_interface());
  }

  ~IsolatedComponentManager() override
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      for (auto & entry : executors_) {
        stop(entry.second);
      }
      executors_.clear();
    }
    // The components are not in the base class's executor: destroy them here.
    node_wrappers_.clear();
  }

  /// Have every Nav2 lifecycle manager among the components, the last loaded first, shut its
  /// nodes down; stop at `deadline` or when `interrupted()` returns true.
  void shutdownManagedNodes(
    std::chrono::steady_clock::time_point deadline, const std::function<bool()> & interrupted)
  {
    const std::vector<Manager> managers = findManagers();
    if (managers.empty()) {
      return;
    }
    // A manager serves is_active after a start-up it is running.
    for (const Manager & manager : managers) {
      if (!call<Trigger>(
            manager.is_active, std::make_shared<Trigger::Request>(), deadline, interrupted)) {
        return;
      }
    }
    std::this_thread::sleep_for(kSettleTime);
    for (const Manager & manager : managers) {
      auto request = std::make_shared<ManageLifecycleNodes::Request>();
      request->command = ManageLifecycleNodes::Request::SHUTDOWN;
      const auto response =
        call<ManageLifecycleNodes>(manager.manage_nodes, request, deadline, interrupted);
      if (!response) {
        return;
      }
      if (!response->success) {
        RCLCPP_WARN(get_logger(), "%s failed to shut its nodes down", manager.manage_nodes.c_str());
      }
    }
  }

protected:
  void add_node_to_executor(uint64_t node_id) override
  {
    Dedicated dedicated;
    const auto node = node_wrappers_[node_id].get_node_base_interface();
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    dedicated.name = node->get_name();
    dedicated.ns = node->get_namespace();
    dedicated.executor = executor;
    dedicated.thread = std::thread([executor]() { executor->spin(); });
    const std::lock_guard<std::mutex> lock(mutex_);
    executors_[node_id] = std::move(dedicated);
  }

  void remove_node_from_executor(uint64_t node_id) override
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    const auto found = executors_.find(node_id);
    if (found != executors_.end()) {
      stop(found->second);
      executors_.erase(found);
    }
  }

private:
  struct Dedicated
  {
    std::string name;  ///< The component's node name and namespace.
    std::string ns;
    std::shared_ptr<rclcpp::Executor> executor;
    std::thread thread;
  };

  /// A Nav2 lifecycle manager's services.
  struct Manager
  {
    std::string manage_nodes;  ///< nav2_msgs/srv/ManageLifecycleNodes
    std::string is_active;     ///< std_srvs/srv/Trigger
  };

  /// The Nav2 lifecycle managers among the components, the last loaded first.
  std::vector<Manager> findManagers()
  {
    const std::string manage_type = rosidl_generator_traits::name<ManageLifecycleNodes>();
    const std::string trigger_type = rosidl_generator_traits::name<Trigger>();
    std::vector<std::pair<std::string, std::string>> nodes;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      for (auto entry = executors_.rbegin(); entry != executors_.rend(); ++entry) {
        nodes.emplace_back(entry->second.name, entry->second.ns);
      }
    }
    std::vector<Manager> managers;
    for (const auto & [name, ns] : nodes) {
      Manager manager;
      try {
        for (const auto & [service, types] : get_service_names_and_types_by_node(name, ns)) {
          const auto has = [&types](const std::string & type) {
            return std::find(types.begin(), types.end(), type) != types.end();
          };
          if (has(manage_type)) {
            manager.manage_nodes = service;
          } else if (
            has(trigger_type) && service.size() >= 10 &&
            service.compare(service.size() - 10, 10, "/is_active") == 0) {
            manager.is_active = service;
          }
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN(get_logger(), "cannot list the services of %s: %s", name.c_str(), error.what());
      }
      if (!manager.manage_nodes.empty() && !manager.is_active.empty()) {
        managers.push_back(manager);
      }
    }
    return managers;
  }

  /// Call `service` and return its response, or nullptr (with a warning) once `deadline` has
  /// passed or `interrupted()` returns true.
  template <typename ServiceT>
  typename ServiceT::Response::SharedPtr call(
    const std::string & service, typename ServiceT::Request::SharedPtr request,
    std::chrono::steady_clock::time_point deadline, const std::function<bool()> & interrupted)
  {
#if RCLCPP_VERSION_GTE(21, 0, 0)
    const auto client = create_client<ServiceT>(service, rclcpp::ServicesQoS(), shutdown_group_);
#else
    const auto client =
      create_client<ServiceT>(service, rmw_qos_profile_services_default, shutdown_group_);
#endif
    auto result = client->async_send_request(request);
    auto code = rclcpp::FutureReturnCode::TIMEOUT;
    while (code == rclcpp::FutureReturnCode::TIMEOUT &&
           std::chrono::steady_clock::now() < deadline && !interrupted()) {
      code = shutdown_executor_.spin_until_future_complete(result, std::chrono::milliseconds(20));
    }
    if (code != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "%s did not answer in time; shutting down anyway", service.c_str());
      return nullptr;
    }
    return result.get();
  }

  /// cancel() before the thread enters spin() would be lost, so wait for spin() while the
  /// context is valid; once it is shut down, spin() returns by itself.
  void stop(Dedicated & dedicated)
  {
    const auto context = get_node_base_interface()->get_context();
    while (!dedicated.executor->is_spinning() && rclcpp::ok(context)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    dedicated.executor->cancel();
    dedicated.thread.join();
  }

  rclcpp::CallbackGroup::SharedPtr shutdown_group_;  ///< The managers' clients, spun below.
  rclcpp::executors::SingleThreadedExecutor shutdown_executor_;
  std::mutex mutex_;  ///< Guards executors_: managerServices() runs in the signal thread.
  std::map<uint64_t, Dedicated> executors_;  ///< By node id, in load order.
};

}  // namespace

int main(int argc, char * argv[])
{
  // Blocked before any thread starts, so every thread inherits the mask and only the signal
  // thread below takes them.
  const sigset_t signals = shutdownSignals();
  pthread_sigmask(SIG_BLOCK, &signals, nullptr);
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto manager = std::make_shared<IsolatedComponentManager>(executor);
  executor->add_node(manager);

  std::atomic<bool> spinning{true};
  std::thread signal_thread([&]() {
    while (spinning && takeSignal(signals, std::chrono::milliseconds(100)) == 0) {
    }
    if (!spinning) {
      return;
    }
    try {
      manager->shutdownManagedNodes(
        std::chrono::steady_clock::now() + kManagedShutdownTimeout,
        [&]() { return takeSignal(signals, std::chrono::milliseconds(0)) == SIGTERM; });
    } catch (const std::exception & error) {
      RCLCPP_ERROR(manager->get_logger(), "managed shutdown failed: %s", error.what());
    }
    rclcpp::shutdown();
  });
  executor->spin();
  spinning = false;
  signal_thread.join();
  return 0;
}
