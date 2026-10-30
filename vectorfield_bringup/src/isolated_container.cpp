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
/// @brief `isolated_container`: Humble's component_container_isolated without its shutdown hang.
///
/// Every component spins on its own single-threaded executor, as in component_container_isolated.
/// Humble's ComponentManagerIsolated (rclcpp_components 16.0) stops an executor by waiting for it
/// to spin before cancelling it; after a shutdown signal the executors have already stopped, so
/// the container never leaves its destructor and launch kills it. This manager stops waiting
/// once the context is shut down, as Jazzy's does. The demos start it on Humble only.

#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <unordered_map>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

namespace
{

class IsolatedComponentManager : public rclcpp_components::ComponentManager
{
public:
  using rclcpp_components::ComponentManager::ComponentManager;

  ~IsolatedComponentManager() override
  {
    for (auto & entry : executors_) {
      stop(entry.second);
    }
    executors_.clear();
    // The components are not in the base class's executor: destroy them here.
    node_wrappers_.clear();
  }

protected:
  void add_node_to_executor(uint64_t node_id) override
  {
    Dedicated dedicated;
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node_wrappers_[node_id].get_node_base_interface());
    dedicated.executor = executor;
    dedicated.thread = std::thread([executor]() { executor->spin(); });
    executors_[node_id] = std::move(dedicated);
  }

  void remove_node_from_executor(uint64_t node_id) override
  {
    const auto found = executors_.find(node_id);
    if (found != executors_.end()) {
      stop(found->second);
      executors_.erase(found);
    }
  }

private:
  struct Dedicated
  {
    std::shared_ptr<rclcpp::Executor> executor;
    std::thread thread;
  };

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

  std::unordered_map<uint64_t, Dedicated> executors_;
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto manager = std::make_shared<IsolatedComponentManager>(executor);
  executor->add_node(manager);
  executor->spin();
  return 0;
}
