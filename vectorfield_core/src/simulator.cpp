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

#include "vectorfield_core/simulator.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <utility>

namespace vectorfield_core
{
namespace sim
{

namespace
{

/// Slack on the period count of run(), so that duration / period periods are simulated despite
/// rounding (20 s at 0.05 s is 400 periods, not 401).
constexpr double kPeriodCountSlack = 1e-9;

/// @brief A uniform double in [0, 1) from the top 53 bits of one engine draw.
///
/// std::uniform_real_distribution is implementation-defined, so it differs between libstdc++ and
/// libc++; this mapping and std::mt19937_64 (fully specified by the standard) make a seeded
/// disturbance identical on every platform.
double unitUniform(std::mt19937_64 * engine) noexcept
{
  constexpr double kTwoToMinus53 = 1.0 / 9007199254740992.0;
  return static_cast<double>((*engine)() >> 11U) * kTwoToMinus53;
}

/// @brief The control period of a simulation: the option when positive and finite, else the
/// Tracker's nominal period 1/f.
double simulationPeriod(double option, const Tracker & tracker) noexcept
{
  if (option > 0.0 && std::isfinite(option)) {
    return option;
  }
  return 1.0 / tracker.config().node.controller_frequency;
}

/// @brief The robot has stopped: the command is below the cancel thresholds (math_spec §9.2).
bool stoppedCommand(const Twist2 & cmd) noexcept
{
  return std::abs(cmd.vx) < kStopLinearSpeed && std::abs(cmd.vy) < kStopLinearSpeed &&
         std::abs(cmd.wz) < kStopAngularSpeed;
}

}  // namespace

Disturbance boundedRandomDisturbance(double bound, std::uint64_t seed)
{
  if (!(bound > 0.0) || !std::isfinite(bound)) {
    return [](double, const Pose2 &) -> Vec2 { return Vec2::Zero(); };
  }
  // Uniform in the disc: radius bound * sqrt(u1), angle 2 pi u2. The engine is captured by value,
  // so every copy of the Disturbance replays the same sequence from the seed.
  return [engine = std::mt19937_64(seed), bound](double, const Pose2 &) mutable -> Vec2 {
    const double radius = bound * std::sqrt(unitUniform(&engine));
    const double angle = kTwoPi * unitUniform(&engine);
    return Vec2(radius * std::cos(angle), radius * std::sin(angle));
  };
}

ClosedLoopSimulator::ClosedLoopSimulator(Tracker tracker, SimulationOptions options)
: tracker_(std::move(tracker)),
  plant_(tracker_.kinematics()),
  options_(std::move(options)),
  state_(Tracker::initialState())
{
  options_.period = simulationPeriod(options_.period, tracker_);
}

ClosedLoopSimulator::ClosedLoopSimulator(
  Tracker tracker, const Kinematics & plant, SimulationOptions options)
: tracker_(std::move(tracker)),
  plant_(plant),
  options_(std::move(options)),
  state_(Tracker::initialState())
{
  options_.period = simulationPeriod(options_.period, tracker_);
}

void ClosedLoopSimulator::reset(const Pose2 & pose, double t0, const Twist2 & twist)
{
  pose_ = pose;
  twist_ = twist;
  t_ = t0;
  state_ = Tracker::initialState();
  last_output_ = TrackerOutput{};
  records_.clear();
}

const SimulationRecord & ClosedLoopSimulator::step(const TrackerInput & input)
{
  const double period = options_.period;
  TrackerInput cycle = input;
  cycle.pose = pose_;
  cycle.measured = twist_;  // ideal odometry: the command held over the last period
  cycle.t = t_;
  cycle.dt = period;
  last_output_ = tracker_.step(cycle, state_, &state_, scratch_);

  SimulationRecord record;
  record.t = t_;
  record.pose = pose_;
  record.measured = twist_;
  record.cmd = last_output_.cmd;
  record.phase = last_output_.phase;
  record.status = last_output_.status;
  record.settled = last_output_.settled;
  record.D = last_output_.field.D;
  record.sigma = last_output_.field.sigma;
  record.s_star = last_output_.field.s_star;
  record.x_eval = last_output_.field.x_eval;
  record.F = last_output_.field.F;
  record.s_free = last_output_.s_free;
  records_.push_back(record);

  // Zero-order hold over one period: exact integration of the plant (with slip), plus a
  // world-frame disturbance held constant over the period (exact: it adds delta * period).
  Pose2 next = integrate(plant_, pose_, last_output_.cmd, period, options_.slip);
  if (options_.disturbance) {
    const Vec2 delta = options_.disturbance(t_, pose_);
    if (isFinite(delta)) {
      next.p += delta * period;
    }
  }
  next.yaw = wrapAngle(next.yaw);
  pose_ = next;
  twist_ = last_output_.cmd;
  t_ += period;
  return records_.back();
}

std::size_t ClosedLoopSimulator::run(
  const TrackerInput & input, double duration,
  const std::function<void(double t, TrackerInput * input)> & update)
{
  if (!(duration > 0.0) || !std::isfinite(duration)) {
    return 0;
  }
  const auto periods =
    static_cast<std::size_t>(std::ceil(duration / options_.period - kPeriodCountSlack));
  TrackerInput current = input;  // the hook's edits persist from one period to the next
  std::size_t count = 0;
  while (count < periods) {
    if (update) {
      update(t_, &current);
    }
    const SimulationRecord & record = step(current);
    ++count;
    if (options_.stop_when_settled && record.settled) {
      break;
    }
    if (
      options_.stop_when_stopped && record.phase == Phase::kStopped && stoppedCommand(record.cmd)) {
      break;
    }
  }
  return count;
}

const Pose2 & ClosedLoopSimulator::pose() const noexcept { return pose_; }

double ClosedLoopSimulator::time() const noexcept { return t_; }

const TrackerState & ClosedLoopSimulator::state() const noexcept { return state_; }

const StepScratch & ClosedLoopSimulator::scratch() const noexcept { return scratch_; }

const TrackerOutput & ClosedLoopSimulator::lastOutput() const noexcept { return last_output_; }

const std::vector<SimulationRecord> & ClosedLoopSimulator::records() const noexcept
{
  return records_;
}

const Tracker & ClosedLoopSimulator::tracker() const noexcept { return tracker_; }

}  // namespace sim
}  // namespace vectorfield_core
