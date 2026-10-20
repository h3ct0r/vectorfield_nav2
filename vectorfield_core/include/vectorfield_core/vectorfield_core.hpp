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
/// @brief Umbrella header of vectorfield_core, the ROS-free core of vectorfield_nav2.
///
/// Includes every public header: basic types and views (types, span, function_ref, status,
/// errors), configuration and validation (config), gains, curves (geometry, curve,
/// polyline_curve, parametric_curve, curve_spec, conditioning), the field, occupancy and
/// footprint, kinematics, tracking laws, the limiter, the contour, the collision predictor, the
/// Tracker and the closed-loop simulator. The mathematics is specified in
/// docs/design/math_spec.md; section references in the API documentation point there.

#ifndef VECTORFIELD_CORE__VECTORFIELD_CORE_HPP_
#define VECTORFIELD_CORE__VECTORFIELD_CORE_HPP_

#include "vectorfield_core/cbf.hpp"
#include "vectorfield_core/collision_predictor.hpp"
#include "vectorfield_core/conditioning.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/contour.hpp"
#include "vectorfield_core/curve.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/errors.hpp"
#include "vectorfield_core/field.hpp"
#include "vectorfield_core/footprint.hpp"
#include "vectorfield_core/function_ref.hpp"
#include "vectorfield_core/gains.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/limiter.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/parametric_curve.hpp"
#include "vectorfield_core/polyline_curve.hpp"
#include "vectorfield_core/simulator.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/tracker.hpp"
#include "vectorfield_core/tracking_law.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_core/version.hpp"

/// @brief Everything in the vectorfield_core package.
namespace vectorfield_core
{

/// @brief Version of the compiled library, "MAJOR.MINOR.PATCH".
///
/// The value comes from package.xml at configure time. It can differ from VF_CORE_VERSION_STR
/// only if a program is compiled against headers from a different installation than the
/// library it loads.
/// @return Pointer to a static, null-terminated string; never null.
const char * version() noexcept;

/// @brief Compile-time check of the header version, in the spirit of EIGEN_VERSION_AT_LEAST.
/// @param major Required major version.
/// @param minor Required minor version.
/// @param patch Required patch version.
/// @return True if the headers are at least version major.minor.patch.
constexpr bool versionAtLeast(int major, int minor, int patch) noexcept
{
  if (VF_CORE_VERSION_MAJOR != major) {
    return VF_CORE_VERSION_MAJOR > major;
  }
  if (VF_CORE_VERSION_MINOR != minor) {
    return VF_CORE_VERSION_MINOR > minor;
  }
  return VF_CORE_VERSION_PATCH >= patch;
}

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__VECTORFIELD_CORE_HPP_
