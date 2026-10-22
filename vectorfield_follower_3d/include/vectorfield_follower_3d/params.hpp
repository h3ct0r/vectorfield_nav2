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
/// @brief The 3D follower's parameters (params/vectorfield_follower_3d_params.yaml) as core
/// configuration, and the obstacle primitives of the `obstacles` parameter. Pure.

#ifndef VECTORFIELD_FOLLOWER_3D__PARAMS_HPP_
#define VECTORFIELD_FOLLOWER_3D__PARAMS_HPP_

#include <optional>
#include <string>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/d3/frame.hpp"
#include "vectorfield_core/d3/obstacles.hpp"
#include "vectorfield_core/d3/tracker.hpp"
#include "vectorfield_follower_3d/vectorfield_follower_3d_params.hpp"

namespace vectorfield_follower_3d
{

/// @brief The generated parameter struct.
using Params = vectorfield_follower_3d_params::Params;
/// @brief The generated listener.
using ParamListener = vectorfield_follower_3d_params::ParamListener;

/// @brief The Tracker3 configuration of the parameters (not validated; the caller runs
/// vectorfield_core::d3::validateTracker3Config()).
vectorfield_core::d3::Tracker3Config toTracker3Config(const Params & params);

/// @brief The curve-intake options (`default_sample_spacing`, `max_curve_points`).
vectorfield_core::CurveOptions toCurveOptions(const Params & params);

/// @brief The settings of the moving-frame estimator (`input_timeout`, `frame_twist_*`).
vectorfield_core::d3::FrameTwistConfig3 toFrameTwistConfig(const Params & params);

/// @brief Parses one entry of the `obstacles` parameter (see the YAML): `sphere x y z r`,
/// `box x y z hx hy hz [yaw pitch roll]`, `cylinder x y z r [half_height]` or
/// `plane px py pz nx ny nz`.
/// @param text The entry.
/// @param error Receives the reason on failure (may be null).
/// @return The primitive (validated by vectorfield_core::d3::validatePrimitive3()), or nullopt.
std::optional<vectorfield_core::d3::Primitive3> parsePrimitive(
  const std::string & text, std::string * error = nullptr);

/// @brief Parses every entry of `obstacles`.
/// @param entries The entries.
/// @param out Receives the primitives (cleared first).
/// @param error Receives the first failure, naming the entry (may be null).
/// @return False if an entry is invalid (`out` then unspecified).
bool parsePrimitives(
  const std::vector<std::string> & entries, std::vector<vectorfield_core::d3::Primitive3> * out,
  std::string * error = nullptr);

/// @brief The whole configuration check of a parameter set: the core's validateTracker3Config()
/// and the `obstacles` entries.
/// @param params The parameters.
/// @param error Receives the reason on failure (may be null).
/// @return True if the set is usable.
bool validateParams(const Params & params, std::string * error = nullptr);

}  // namespace vectorfield_follower_3d

#endif  // VECTORFIELD_FOLLOWER_3D__PARAMS_HPP_
