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

#include "vectorfield_ros/field_state.hpp"

namespace vectorfield_ros
{

void toFieldState(
  const vectorfield_core::TrackerOutput & /*output*/, double /*compute_time_ms*/,
  vectorfield_msgs::msg::FieldState * /*msg*/) noexcept
{
  // M2 wave 2: ros-viz
}

}  // namespace vectorfield_ros
