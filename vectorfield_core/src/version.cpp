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

#include "vectorfield_core/vectorfield_core.hpp"

#include <Eigen/Core>

// find_package(Eigen3) takes no version argument: Eigen 5's config-version file rejects a "3.4"
// request. The floor is enforced here instead (jammy, noble and resolute ship 3.4.0).
static_assert(EIGEN_VERSION_AT_LEAST(3, 4, 0), "vectorfield_core requires Eigen 3.4.0 or newer");

namespace vectorfield_core
{

const char * version() noexcept { return VF_CORE_VERSION_STR; }

}  // namespace vectorfield_core
