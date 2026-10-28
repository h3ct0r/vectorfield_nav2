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
/// @brief main() of a simulator that publishes `/clock`: the clock outlives the nodes that
/// read it.
///
/// launch signals every process at once. A node shutting down can wait on simulated time
/// (Nav2's collision monitor looks up a transform with a timeout in its velocity callback, and
/// its lifecycle manager waits for that callback before the process exits): with the clock gone
/// the wait never ends, and launch has to kill the process. After SIGINT or SIGTERM the
/// simulator therefore keeps running while another node subscribes to `/clock`, for at most
/// kClockSourceGrace, below launch's 5 s before it escalates to SIGTERM.

#ifndef VECTORFIELD_ROS__CLOCK_SOURCE_HPP_
#define VECTORFIELD_ROS__CLOCK_SOURCE_HPP_

#include <chrono>
#include <csignal>
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace vectorfield_ros
{

/// How long a clock source keeps running after a shutdown signal while `/clock` has readers.
inline constexpr std::chrono::seconds kClockSourceGrace{4};

namespace detail
{
inline volatile std::sig_atomic_t clock_source_signalled = 0;

inline void signalClockSource(int /*signal*/) { clock_source_signalled = 1; }
}  // namespace detail

/// @brief Run the node `make()` returns until SIGINT or SIGTERM, then until no other node
/// subscribes to `/clock` or kClockSourceGrace has passed; return main()'s exit status.
///
/// rclcpp is initialised without its signal handlers, so the node keeps publishing after the
/// signal. An exception from `make()` or a callback is logged as FATAL under `name` (status 1).
template <typename MakeNode>
int runClockSource(int argc, char ** argv, const char * name, MakeNode make)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, detail::signalClockSource);
  std::signal(SIGTERM, detail::signalClockSource);
  int status = 0;
  try {
    const std::shared_ptr<rclcpp::Node> node = make();
    // The node's own subscription, when it runs on simulated time itself.
    const std::size_t own = node->get_parameter("use_sim_time").as_bool() ? 1U : 0U;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    bool stopping = false;
    std::chrono::steady_clock::time_point deadline;
    while (rclcpp::ok()) {
      executor.spin_once(std::chrono::milliseconds(20));
      if (detail::clock_source_signalled == 0) {
        continue;
      }
      const auto now = std::chrono::steady_clock::now();
      if (!stopping) {
        stopping = true;
        deadline = now + kClockSourceGrace;
      }
      if (node->count_subscribers("/clock") <= own || now >= deadline) {
        break;
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger(name), "%s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}

}  // namespace vectorfield_ros

#endif  // VECTORFIELD_ROS__CLOCK_SOURCE_HPP_
