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

// The 3D point / multirotor plant and ClosedLoopSimulator3.

#include "vectorfield_core/d3/simulator.hpp"

#include <cmath>
#include <string>
#include <utility>

#include "vectorfield_core/errors.hpp"

namespace vectorfield_core::d3::sim
{

PlantState3 integrate(
  const PlantState3 & state, const Twist3 & cmd, double dt, const PlantParams3 & params) noexcept
{
  if (
    !isFinite(state.pose) || !isFinite(state.twist) || !isFinite(cmd) || !(dt >= 0.0) ||
    !std::isfinite(dt)) {
    return state;
  }
  PlantState3 next = state;
  const double tau = params.velocity_time_constant;
  if (!(tau > 0.0)) {
    next.twist = cmd;
    next.pose.p = state.pose.p + dt * cmd.v;
    next.pose.yaw = state.pose.yaw + dt * cmd.wz;
    return next;
  }
  // v(t) = u + (v0 - u) e^{-t/tau}; the position integrates it in closed form.
  const double decay = std::exp(-dt / tau);
  const double gain = tau * (1.0 - decay);
  next.twist.v = cmd.v + (state.twist.v - cmd.v) * decay;
  next.twist.wz = cmd.wz + (state.twist.wz - cmd.wz) * decay;
  next.pose.p = state.pose.p + dt * cmd.v + gain * (state.twist.v - cmd.v);
  next.pose.yaw = state.pose.yaw + dt * cmd.wz + gain * (state.twist.wz - cmd.wz);
  return next;
}

ClosedLoopSimulator3::ClosedLoopSimulator3(Tracker3 tracker, SimulationOptions3 options)
: tracker_(std::move(tracker)), options_(std::move(options)), state_(Tracker3::initialState())
{
  if (!(options_.period > 0.0) || !std::isfinite(options_.period)) {
    throw ConfigError(
      "SimulationOptions3.period = " + std::to_string(options_.period) + ": must be > 0");
  }
  if (
    !(options_.plant.velocity_time_constant >= 0.0) ||
    !std::isfinite(options_.plant.velocity_time_constant)) {
    throw ConfigError("PlantParams3.velocity_time_constant must be finite and >= 0");
  }
}

void ClosedLoopSimulator3::reset(const Pose3 & pose, double t0, const Twist3 & twist)
{
  plant_.pose = pose;
  plant_.twist = twist;
  t_ = t0;
  state_ = Tracker3::initialState();
  last_output_ = Tracker3Output{};
  records_.clear();
}

const SimulationRecord3 & ClosedLoopSimulator3::step(const Tracker3Input & input)
{
  Tracker3Input in = input;
  in.pose = plant_.pose;
  in.measured = plant_.twist;
  in.t = t_;
  in.dt = options_.period;
  last_output_ = tracker_.step(in, state_, &state_);

  SimulationRecord3 record;
  record.t = t_;
  record.pose = plant_.pose;
  record.cmd = last_output_.cmd;
  record.phase = last_output_.phase;
  record.status = last_output_.status;
  record.settled = last_output_.settled;
  record.D = last_output_.field.D;
  record.sigma = last_output_.field.sigma;
  record.s_star = last_output_.field.s_star;
  record.F = last_output_.field.Phi;
  records_.push_back(record);

  plant_ = integrate(plant_, last_output_.cmd, options_.period, options_.plant);
  if (options_.disturbance) {
    const Vec3 delta = options_.disturbance(t_, plant_.pose);
    if (isFinite(delta)) {
      plant_.pose.p += options_.period * delta;
    }
  }
  t_ += options_.period;
  return records_.back();
}

std::size_t ClosedLoopSimulator3::run(
  const Tracker3Input & input, double duration,
  const std::function<void(double t, Tracker3Input * input)> & update)
{
  Tracker3Input in = input;
  const double end = t_ + duration;
  std::size_t steps = 0;
  while (t_ < end - 1e-9) {
    if (update) {
      update(t_, &in);
    }
    const SimulationRecord3 & record = step(in);
    ++steps;
    if (options_.stop_when_settled && record.settled) {
      break;
    }
    if (options_.stop_when_stopped && last_output_.cancel_complete) {
      break;
    }
  }
  return steps;
}

}  // namespace vectorfield_core::d3::sim
