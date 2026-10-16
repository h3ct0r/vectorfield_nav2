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

#include "vectorfield_sim/obstacles.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace vectorfield_sim
{

namespace
{

using vectorfield_core::Vec2;

bool fail(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

/// Checks the length and finiteness of a flat list; `layout` names one entry's fields.
bool checkLayout(
  const char * name, const std::vector<double> & values, std::size_t stride, const char * layout,
  std::string * error)
{
  if (values.size() % stride != 0) {
    std::ostringstream message;
    message << name << ": " << values.size() << " values is not a multiple of " << stride << " ("
            << layout << " per obstacle)";
    return fail(error, message.str());
  }
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (!std::isfinite(values[i])) {
      std::ostringstream message;
      message << name << "[" << i / stride << "]: value " << i % stride << " of " << layout
              << " is not finite";
      return fail(error, message.str());
    }
  }
  return true;
}

std::string positiveError(const char * name, std::size_t index, const char * field, double value)
{
  std::ostringstream message;
  message << name << "[" << index << "]: " << field << " " << value << " must be > 0";
  return message.str();
}

}  // namespace

bool parseCircleObstacles(
  const std::vector<double> & values, std::vector<Circle> * out, std::string * error)
{
  constexpr const char * kName = "circle_obstacles";
  if (!checkLayout(kName, values, kCircleStride, "[x, y, radius]", error)) {
    return false;
  }
  std::vector<Circle> circles;
  circles.reserve(values.size() / kCircleStride);
  for (std::size_t i = 0; i < values.size(); i += kCircleStride) {
    const Circle circle{Vec2(values[i], values[i + 1]), values[i + 2]};
    if (!(circle.radius > 0.0)) {
      return fail(error, positiveError(kName, i / kCircleStride, "radius", circle.radius));
    }
    circles.push_back(circle);
  }
  *out = std::move(circles);
  return true;
}

bool parseBoxObstacles(
  const std::vector<double> & values, std::vector<Box> * out, std::string * error)
{
  constexpr const char * kName = "box_obstacles";
  if (!checkLayout(kName, values, kBoxStride, "[x, y, size_x, size_y, yaw]", error)) {
    return false;
  }
  std::vector<Box> boxes;
  boxes.reserve(values.size() / kBoxStride);
  for (std::size_t i = 0; i < values.size(); i += kBoxStride) {
    const Box box{
      Vec2(values[i], values[i + 1]), Vec2(values[i + 2], values[i + 3]), values[i + 4]};
    if (!(box.size.x() > 0.0)) {
      return fail(error, positiveError(kName, i / kBoxStride, "size_x", box.size.x()));
    }
    if (!(box.size.y() > 0.0)) {
      return fail(error, positiveError(kName, i / kBoxStride, "size_y", box.size.y()));
    }
    boxes.push_back(box);
  }
  *out = std::move(boxes);
  return true;
}

}  // namespace vectorfield_sim
