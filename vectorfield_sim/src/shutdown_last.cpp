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
/// @brief shutdown_last: runs a simulator that is not ours so that it stops after the nodes that
/// read its `/clock`, as ours do (vectorfield_ros/clock_source.hpp).
///
/// Usage, typically as a launch prefix: `shutdown_last <command> [args...]`.
///
/// launch signals every process at once. A simulator that stops at once (Nav2's loopback
/// simulator, Gazebo and its ros_gz_bridge) leaves Nav2's nodes waiting on simulated time, and
/// on transforms that no longer come, while they shut down, until launch kills them. This
/// process runs the command in its own process group, so that a terminal's Ctrl+C (which signals
/// the whole foreground group) reaches only this process, and forwards SIGINT to that group once
/// no node reads `/clock` (vectorfield_ros::detail::clockReaders(), believed after kDiscoveryTime)
/// or after vectorfield_ros::kClockSourceGrace. SIGTERM is forwarded at once, and the command gets
/// SIGTERM if this process dies (PR_SET_PDEATHSIG).
///
/// Exit status: the command's; 0 when it died of the SIGINT forwarded to it (Python's default
/// handler ends that way), 128 + the signal when it died of another one.

#include <signal.h>
#include <sys/prctl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "vectorfield_ros/clock_source.hpp"

namespace
{

volatile sig_atomic_t interrupted = 0;
volatile sig_atomic_t terminated = 0;

void onSignal(int number)
{
  if (number == SIGTERM) {
    terminated = 1;
  } else {
    interrupted = 1;
  }
}

/// How often the command and the readers are checked.
constexpr std::chrono::milliseconds kPoll{20};

/// How long this process's node is in the graph before it believes that no node reads `/clock`.
/// It starts with the command, and a reader it has not discovered yet would otherwise lose its
/// clock to an early Ctrl+C. Discovery has no end to wait for; on one host it takes far less.
constexpr std::chrono::seconds kDiscoveryTime{1};

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: shutdown_last <command> [args...]\n");
    return 2;
  }
  // Before fork(): a signal in between would otherwise end this process with the command
  // already out of reach of the terminal.
  struct sigaction action{};
  action.sa_handler = onSignal;
  sigemptyset(&action.sa_mask);
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);

  const pid_t parent = getpid();
  const pid_t child = fork();
  if (child < 0) {
    std::perror("shutdown_last: fork");
    return 1;
  }
  if (child == 0) {
    setpgid(0, 0);
    prctl(PR_SET_PDEATHSIG, SIGTERM);
    if (getppid() != parent) {
      _exit(1);  // the parent died before prctl()
    }
    // exec() resets the caught signals to their defaults.
    execvp(argv[1], argv + 1);
    std::perror(argv[1]);
    _exit(127);
  }
  setpgid(child, child);  // also here, so the group exists before the first signal

  // The command's own arguments (--ros-args ...) are not this process's.
  rclcpp::init(1, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  const auto node =
    std::make_shared<rclcpp::Node>("shutdown_last_" + std::to_string(static_cast<long>(getpid())));
  const auto created = std::chrono::steady_clock::now();

  std::optional<std::chrono::steady_clock::time_point> deadline;
  int forwarded = 0;
  int status = 0;
  while (true) {
    const pid_t done = waitpid(child, &status, WNOHANG);
    if (done == child) {
      break;
    }
    if (done < 0 && errno != EINTR) {
      std::perror("shutdown_last: waitpid");
      rclcpp::shutdown();
      return 1;
    }
    if (terminated != 0 && forwarded != SIGTERM) {
      kill(-child, SIGTERM);
      forwarded = SIGTERM;
    } else if (interrupted != 0 && forwarded == 0) {
      const auto now = std::chrono::steady_clock::now();
      if (!deadline) {
        deadline = now + vectorfield_ros::kClockSourceGrace;
      }
      if (
        now >= *deadline ||
        (now - created >= kDiscoveryTime && vectorfield_ros::detail::clockReaders(*node) == 0U)) {
        kill(-child, SIGINT);
        forwarded = SIGINT;
      }
    }
    std::this_thread::sleep_for(kPoll);
  }
  rclcpp::shutdown();

  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  const int cause = WTERMSIG(status);
  return cause == SIGINT && forwarded == SIGINT ? 0 : 128 + cause;
}
