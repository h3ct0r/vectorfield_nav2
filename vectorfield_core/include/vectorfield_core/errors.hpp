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
/// @brief The exception type of vectorfield_core.
///
/// Error policy: constructors, factories (`makeCurve`) and validation helpers throw
/// ConfigError. Everything on the control path (`Tracker::step`, curve evaluation, field
/// evaluation, tracking laws, `CommandLimiter::limit`, `sim::integrate`) is `noexcept` and
/// reports problems through status bits (status.hpp) instead. The only exception that can escape
/// a `noexcept` function is `std::bad_alloc` while a caller-owned buffer grows, which terminates.

#ifndef VECTORFIELD_CORE__ERRORS_HPP_
#define VECTORFIELD_CORE__ERRORS_HPP_

#include <stdexcept>

namespace vectorfield_core
{

/// @brief Invalid configuration or input detected while constructing a core object.
///
/// Thrown only by constructors, factories and validation helpers, never from `step`, `evaluate`
/// or `limit`. Derives from `std::invalid_argument`, so ROS callers can catch either type.
class ConfigError : public std::invalid_argument
{
public:
  using std::invalid_argument::invalid_argument;
};

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__ERRORS_HPP_
