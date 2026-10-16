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
/// @brief Shared fixtures of the visualization tests and benchmark: an obstacle grid and a
/// closed-loop scenario that runs a real vectorfield_core::Tracker and exposes its last cycle as
/// a MarkerContext (patterns of vectorfield_core/test/test_support.hpp).

#ifndef VIZ_TEST_SUPPORT_HPP_
#define VIZ_TEST_SUPPORT_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/polyline_curve.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/tracker.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_ros/marker_builder.hpp"

namespace vf_viz_test
{

namespace vf = vectorfield_core;
using vf::Pose2;
using vf::Twist2;
using vf::Vec2;

/// Control period (s).
inline constexpr double kPeriod = 0.05;
/// Control frame of every context.
inline constexpr const char * kFrame = "odom";

/// @brief The stamp of cycle k (k * kPeriod).
inline builtin_interfaces::msg::Time stampOf(std::int64_t cycle)
{
  const std::int64_t ns = cycle * 50'000'000;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(ns / 1'000'000'000);
  stamp.nanosec = static_cast<std::uint32_t>(ns % 1'000'000'000);
  return stamp;
}

/// @brief An open-curve goal.
inline vf::GoalInfo goalAt(const Vec2 & g, double yaw = 0.0, bool has_yaw = false)
{
  vf::GoalInfo goal;
  goal.pose = Pose2{g, yaw};
  goal.has_yaw = has_yaw;
  goal.xy_tol = 0.25;
  goal.yaw_tol = 0.25;
  return goal;
}

/// @brief The default configuration with contour mode on (auto lambda, D_in, D'_in).
inline vf::CoreConfig contourConfig()
{
  vf::CoreConfig config;
  config.obstacles.obstacle_mode = vf::ObstacleMode::kContour;
  return config;
}

/// @brief Points on a circle (closed polyline vertices, the first not repeated).
inline std::vector<Vec2> circlePoints(const Vec2 & centre, double radius, int count)
{
  std::vector<Vec2> out;
  for (int k = 0; k < count; ++k) {
    const double a = vf::kTwoPi * k / count;
    out.emplace_back(centre + radius * Vec2(std::cos(a), std::sin(a)));
  }
  return out;
}

/// @brief An occupancy grid of boxes and discs with its EDT (identity grid frame).
class World
{
public:
  World(const Vec2 & origin, double width_m, double height_m, double res = 0.05)
  : origin_(origin),
    width_(static_cast<int>(std::lround(width_m / res))),
    height_(static_cast<int>(std::lround(height_m / res))),
    res_(res)
  {
    commit();
  }

  void addBox(const Vec2 & lo, const Vec2 & hi) { boxes_.emplace_back(lo, hi); }
  void addDisc(const Vec2 & centre, double radius) { discs_.emplace_back(centre, radius); }

  /// @brief Rasterises the shapes and updates the EDT.
  void commit()
  {
    mask_.assign(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_), 0);
    for (int iy = 0; iy < height_; ++iy) {
      for (int ix = 0; ix < width_; ++ix) {
        const Vec2 c = origin_ + res_ * Vec2(ix + 0.5, iy + 0.5);
        bool site = false;
        for (const auto & box : boxes_) {
          site = site || (c.x() >= box.first.x() && c.x() <= box.second.x() &&
                          c.y() >= box.first.y() && c.y() <= box.second.y());
        }
        for (const auto & disc : discs_) {
          site = site || (c - disc.first).norm() <= disc.second;
        }
        mask_[static_cast<std::size_t>(iy * width_ + ix)] = site ? 1 : 0;
      }
    }
    grid_.setFromMask(
      vf::Span<const std::uint8_t>(mask_.data(), mask_.size()), width_, height_, res_, origin_);
    edt_.update(grid_);
  }

  vf::ObstacleMap map() const
  {
    vf::ObstacleMap m;
    m.grid = &grid_;
    m.edt = &edt_;
    m.T_grid_eval = vf::Isometry2();
    return m;
  }

  const vf::OccupancyGrid & grid() const { return grid_; }
  const vf::EuclideanDistanceTransform & edt() const { return edt_; }

private:
  Vec2 origin_;
  int width_;
  int height_;
  double res_;
  std::vector<std::pair<Vec2, Vec2>> boxes_;
  std::vector<std::pair<Vec2, double>> discs_;
  std::vector<std::uint8_t> mask_;
  vf::OccupancyGrid grid_;
  vf::EuclideanDistanceTransform edt_;
};

/// @brief A closed loop around a real Tracker: plugin mode (a window assigned to a
/// PolylineCurve, passed to the builder as the window) or follower mode (a curve from makeCurve,
/// sampled by the builder, with CurveInfo). Not copyable: contexts point into it.
class Scenario
{
public:
  /// @brief Plugin mode: `window` is the conditioned window (open), in the control frame.
  Scenario(const vf::CoreConfig & config, std::vector<Vec2> window, const Pose2 & start)
  : tracker_(config), window_(std::move(window)), pose_(start)
  {
    polyline_.assign(vf::Span<const Vec2>(window_.data(), window_.size()), false);
    input_.curve = &polyline_;
    init();
  }

  /// @brief Follower mode: a closed polyline curve through `points`.
  Scenario(
    const vf::CoreConfig & config, const std::vector<Vec2> & points, const Pose2 & start,
    const vectorfield_ros::CurveInfo & info)
  : tracker_(config), pose_(start), curve_info_(info), follower_(true)
  {
    vf::CurveSpec spec;
    spec.type = vf::CurveSpec::Type::kPolyline;
    spec.closed = true;
    spec.points = points;
    curve_ = vf::makeCurve(spec, vf::CurveOptions{});
    input_.curve = curve_.get();
    init();
  }

  Scenario(const Scenario &) = delete;
  Scenario & operator=(const Scenario &) = delete;

  /// @brief The input template of the next steps (goal, obstacles, cancel).
  vf::TrackerInput & input() { return input_; }
  /// @brief Index of the virtual tail vertex passed to the builder (plugin mode).
  void setVirtualVertex(int index) { virtual_vertex_ = index; }

  /// @brief One control period: step at the current pose, then integrate the command.
  const vf::TrackerOutput & step()
  {
    input_.pose = pose_;
    input_.measured = measured_;
    input_.t = t_;
    input_.dt = kPeriod;
    vf::TrackerState next;
    output_ = tracker_.step(input_, state_, &next, scratch_);
    state_ = next;
    pose_ = vf::sim::integrate(tracker_.kinematics(), pose_, output_.cmd, kPeriod);
    measured_ = output_.cmd;
    t_ += kPeriod;
    ++cycle_;
    return output_;
  }

  /// @brief Steps until `done` holds after a step, at most `max_steps`; true if it held.
  bool runUntil(const std::function<bool(const vf::TrackerOutput &)> & done, int max_steps)
  {
    for (int k = 0; k < max_steps; ++k) {
      if (done(step())) {
        return true;
      }
    }
    return false;
  }

  /// @brief Steps `count` periods.
  void run(int count)
  {
    for (int k = 0; k < count; ++k) {
      step();
    }
  }

  /// @brief The last cycle as the builder sees it (`output` may be overridden by the caller).
  vectorfield_ros::MarkerContext context() const
  {
    vectorfield_ros::MarkerContext c;
    c.frame_id = kFrame;
    c.stamp = stampOf(cycle_);
    c.tracker = &tracker_;
    c.input = &input_;
    c.output = &output_;
    c.state = &state_;
    c.scratch = &scratch_;
    if (!follower_) {
      c.window = vf::Span<const Vec2>(window_.data(), window_.size());
      c.virtual_vertex = virtual_vertex_;
    } else {
      c.curve_info = &curve_info_;
    }
    c.compute_time_ms = 0.1;
    return c;
  }

  const vf::Tracker & tracker() const { return tracker_; }
  const vf::TrackerOutput & output() const { return output_; }
  const vf::StepScratch & scratch() const { return scratch_; }
  const Pose2 & pose() const { return pose_; }

private:
  void init()
  {
    input_.dt = kPeriod;
    state_ = vf::Tracker::initialState();
  }

  vf::Tracker tracker_;
  std::vector<Vec2> window_;
  vf::PolylineCurve polyline_;
  std::shared_ptr<const vf::Curve> curve_;
  vf::TrackerInput input_;
  vf::TrackerState state_;
  vf::StepScratch scratch_;
  vf::TrackerOutput output_;
  Pose2 pose_;
  Twist2 measured_;
  double t_{0.0};
  std::int64_t cycle_{0};
  int virtual_vertex_{-1};
  vectorfield_ros::CurveInfo curve_info_;
  bool follower_{false};
};

}  // namespace vf_viz_test

#endif  // VIZ_TEST_SUPPORT_HPP_
