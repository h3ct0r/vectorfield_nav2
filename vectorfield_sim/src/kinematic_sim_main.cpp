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
/// @brief The `kinematic_sim` executable: one KinematicSimNode on a single-threaded executor,
/// which keeps publishing /clock after a shutdown signal while other nodes read it
/// (vectorfield_ros/clock_source.hpp).

#include <memory>

#include "vectorfield_ros/clock_source.hpp"
#include "vectorfield_sim/kinematic_sim_node.hpp"

int main(int argc, char ** argv)
{
  return vectorfield_ros::runClockSource(argc, argv, "kinematic_sim", []() {
    return std::make_shared<vectorfield_sim::KinematicSimNode>();
  });
}
