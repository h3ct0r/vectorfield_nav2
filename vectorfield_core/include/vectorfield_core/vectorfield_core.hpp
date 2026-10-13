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
/// At M0 the library exposes only its version. Curves, gain functions, distance and terminal
/// fields, obstacle contouring, tracking laws, the command limiter, the collision predictor, the
/// Tracker and the kinematic simulator arrive in M1 (see docs/design/math_spec.md).

#ifndef VECTORFIELD_CORE__VECTORFIELD_CORE_HPP_
#define VECTORFIELD_CORE__VECTORFIELD_CORE_HPP_

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
