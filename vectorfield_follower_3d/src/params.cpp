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

#include "vectorfield_follower_3d/params.hpp"

#include <cmath>
#include <cstddef>
#include <exception>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "vectorfield_core/d3/types.hpp"

namespace vectorfield_follower_3d
{

namespace vf = vectorfield_core;
namespace d3 = vectorfield_core::d3;

d3::Tracker3Config toTracker3Config(const Params & params)
{
  d3::Tracker3Config config;
  config.desired_speed = params.desired_speed;
  config.convergence_gain = params.convergence_gain;
  config.gain_function =
    params.gain_function == "rational" ? vf::GainType::kRational : vf::GainType::kAtan;
  config.direction = 1;
  config.assumption3_margin = params.assumption3_margin;
  config.goal_tolerance = params.goal_tolerance;
  config.terminal_braking_distance = params.terminal_braking_distance;
  config.min_approach_speed = params.min_approach_speed;
  config.final_approach_distance =
    params.final_approach_distance < 0.0 ? vf::kAutoValue : params.final_approach_distance;
  config.control_frequency = params.controller_frequency;
  config.limits.max_speed = params.max_speed;
  config.limits.max_axis_speed =
    d3::Vec3(params.max_axis_speed_x, params.max_axis_speed_y, params.max_axis_speed_z);
  config.limits.max_accel = params.max_accel;
  config.limits.max_decel = params.max_decel;
  config.limits.cancel_decel = params.cancel_decel;
  config.limits.max_yaw_rate = params.max_yaw_rate;
  config.limits.max_yaw_accel = params.max_yaw_accel;
  if (params.yaw_policy == "field") {
    config.yaw.policy = vf::OmniHeadingPolicy::kField;
  } else if (params.yaw_policy == "fixed") {
    config.yaw.policy = vf::OmniHeadingPolicy::kFixed;
  } else {
    config.yaw.policy = vf::OmniHeadingPolicy::kPathTangent;
  }
  config.yaw.gain = params.yaw_gain;
  config.yaw.fixed_yaw = params.fixed_yaw;
  config.obstacles.obstacle_mode =
    params.obstacle_mode == "cbf" ? vf::ObstacleMode::kCbf : vf::ObstacleMode::kNone;
  config.obstacles.body_radius = params.body_radius;
  config.obstacles.cbf_margin = params.cbf_margin;
  config.obstacles.cbf_gain = params.cbf_gain;
  config.obstacles.cbf_sites = static_cast<int>(params.cbf_sites);
  return config;
}

vf::CurveOptions toCurveOptions(const Params & params)
{
  vf::CurveOptions options;
  options.default_sample_spacing = params.default_sample_spacing;
  options.max_curve_points = static_cast<int>(params.max_curve_points);
  return options;
}

d3::FrameTwistConfig3 toFrameTwistConfig(const Params & params)
{
  d3::FrameTwistConfig3 config;
  config.cutoff_hz = params.frame_twist_cutoff_hz;
  config.max_speed = params.frame_twist_max_speed;
  config.max_angular_rate = params.frame_twist_max_angular_rate;
  config.input_timeout = params.input_timeout;
  // A differencing pair spans at most the staleness age.
  config.max_dt = params.input_timeout;
  return config;
}

namespace
{

bool fail(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

}  // namespace

std::optional<d3::Primitive3> parsePrimitive(const std::string & text, std::string * error)
{
  std::istringstream stream(text);
  std::string kind;
  stream >> kind;
  std::vector<double> values;
  std::string token;
  while (stream >> token) {
    try {
      std::size_t used = 0;
      const double value = std::stod(token, &used);
      if (used != token.size()) {
        throw std::invalid_argument(token);
      }
      values.push_back(value);
    } catch (const std::exception &) {
      fail(error, "'" + token + "' is not a number");
      return std::nullopt;
    }
  }
  const auto count = values.size();
  const auto expect = [&](std::size_t lo, std::size_t hi, const char * form) {
    if (count < lo || count > hi) {
      fail(error, kind + " takes " + form);
      return false;
    }
    return true;
  };
  const auto at = [&](std::size_t i) { return d3::Vec3(values[i], values[i + 1], values[i + 2]); };
  d3::Primitive3 primitive;
  if (kind == "sphere") {
    if (!expect(4, 4, "x y z r")) {
      return std::nullopt;
    }
    primitive = d3::Primitive3::sphere(at(0), values[3]);
  } else if (kind == "box") {
    if (count != 6 && count != 9) {
      fail(error, "box takes x y z hx hy hz [yaw pitch roll]");
      return std::nullopt;
    }
    const d3::Mat3 rotation =
      count == 9 ? d3::rotationZYX(values[6], values[7], values[8]) : d3::Mat3::Identity();
    primitive = d3::Primitive3::box(at(0), at(3), rotation);
  } else if (kind == "cylinder") {
    if (!expect(4, 5, "x y z r [half_height]")) {
      return std::nullopt;
    }
    primitive = d3::Primitive3::cylinder(at(0), values[3], count == 5 ? values[4] : vf::kInf);
  } else if (kind == "plane") {
    if (!expect(6, 6, "px py pz nx ny nz")) {
      return std::nullopt;
    }
    primitive = d3::Primitive3::plane(at(0), at(3));
  } else {
    fail(error, "unknown primitive '" + kind + "' (sphere, box, cylinder or plane)");
    return std::nullopt;
  }
  try {
    d3::validatePrimitive3(primitive);
  } catch (const std::exception & e) {
    fail(error, e.what());
    return std::nullopt;
  }
  return primitive;
}

bool parsePrimitives(
  const std::vector<std::string> & entries, std::vector<d3::Primitive3> * out, std::string * error)
{
  out->clear();
  for (std::size_t i = 0; i < entries.size(); ++i) {
    std::string reason;
    const auto primitive = parsePrimitive(entries[i], &reason);
    if (!primitive) {
      return fail(error, "obstacles[" + std::to_string(i) + "] '" + entries[i] + "': " + reason);
    }
    out->push_back(*primitive);
  }
  return true;
}

bool validateParams(const Params & params, std::string * error)
{
  try {
    d3::validateTracker3Config(toTracker3Config(params));
  } catch (const std::exception & e) {
    return fail(error, e.what());
  }
  std::vector<d3::Primitive3> primitives;
  return parsePrimitives(params.obstacles, &primitives, error);
}

}  // namespace vectorfield_follower_3d
