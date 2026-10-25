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
/// @brief Shared helpers of the closed-loop oracle, scenario and upstream-regression tests
/// (math_spec §12). Written from the specification and the public headers only.
///
/// Contents:
/// - the default test configuration TC of math_spec §12 (tcConfig() and friends);
/// - curve builders through makeCurve() and point generators;
/// - ObstacleWorld: discs and boxes rasterised on an OccupancyGrid (a cell is a site iff its
///   centre lies in a shape), with its EDT, a rolling origin and brute-force distances;
/// - PrunedWindow: a plugin-style window re-pruned around the robot every cycle (math_spec §2.8);
/// - Runner: a closed loop on top of sim::ClosedLoopSimulator that keeps, per cycle, the record,
///   the full TrackerOutput and the TrackerState before and after the step;
/// - metrics: Ḋ as defined in math_spec §12, phase-entry and episode counters, progress.

#ifndef VECTORFIELD_CORE__TEST_SUPPORT_HPP_
#define VECTORFIELD_CORE__TEST_SUPPORT_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "vectorfield_core/conditioning.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/footprint.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/polyline_curve.hpp"
#include "vectorfield_core/simulator.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/tracker.hpp"
#include "vectorfield_core/types.hpp"

namespace vf_test
{

namespace vf = vectorfield_core;
using vf::Phase;
using vf::Pose2;
using vf::Status;
using vf::Twist2;
using vf::Vec2;

// ------------------------------------------------------------------------------------------------
// Default test configuration TC (math_spec §12)
// ------------------------------------------------------------------------------------------------

/// Costmap resolution of TC (m).
inline constexpr double kRes = 0.05;
/// Control period of TC: f = 20 Hz (s).
inline constexpr double kPeriod = 0.05;
/// \f$v_r\f$.
inline constexpr double kVr = 0.4;
/// \f$k_f\f$ (atan gain).
inline constexpr double kKf = 2.0;
/// Control-point offset \f$d\f$.
inline constexpr double kD = 0.2;
/// \f$v_{\max} = v_{y,\max}\f$.
inline constexpr double kVmax = 0.5;
/// \f$\omega_{\max}\f$.
inline constexpr double kWmax = 1.5;
/// \f$a_{\mathrm{acc}} = a_{\mathrm{dec}}\f$.
inline constexpr double kAccel = 1.0;
/// \f$\alpha_{\max}\f$.
inline constexpr double kAlpha = 3.2;
/// \f$\sigma_b\f$.
inline constexpr double kSigmaB = 0.5;
/// \f$v_{\min}\f$.
inline constexpr double kVmin = 0.05;
/// xy_tol.
inline constexpr double kXyTol = 0.25;
/// yaw_tol.
inline constexpr double kYawTol = 0.25;
/// \f$\sigma_{\mathrm{stop}} = \mathrm{xy\_tol}/2\f$.
inline constexpr double kSigmaStop = 0.5 * kXyTol;
/// \f$m_s\f$.
inline constexpr double kStopMargin = 0.10;
/// \f$T_h\f$.
inline constexpr double kHorizon = 1.5;
/// Footprint circumradius (16-gon), \f$w_{\mathrm{lat}} = r_{\mathrm{circ}}\f$.
inline constexpr double kRCirc = 0.22;
/// \f$r_{\mathrm{insc}} = 0.22\cos(\pi/16)\f$.
inline const double kRInsc = 0.22 * std::cos(vf::kPi / 16.0);
/// TC auto contour distances with FL (math_spec §11, §12): \f$\lambda, D_{\mathrm{in}},
/// D'_{\mathrm{in}}\f$.
inline constexpr double kLambda = 0.47;
inline constexpr double kDin = 0.62;
inline constexpr double kDinPrime = 0.82;
/// \f$h_c\f$.
inline constexpr double kHc = 0.05;

/// @brief Vertices of a regular polygon (counter-clockwise), first vertex at angle `phase`.
inline std::vector<Vec2> regularPolygon(double circumradius, int sides, double phase = 0.0)
{
  std::vector<Vec2> out;
  out.reserve(static_cast<std::size_t>(sides));
  for (int k = 0; k < sides; ++k) {
    const double a = phase + vf::kTwoPi * k / sides;
    out.emplace_back(circumradius * std::cos(a), circumradius * std::sin(a));
  }
  return out;
}

/// @brief TC footprint: a 16-gon of circumradius 0.22 with vertices at 2πk/16, so that
/// \f$w_{\mathrm{lat}} = r_{\mathrm{circ}} = 0.22\f$ and \f$r_{\mathrm{insc}} = 0.2158\f$.
inline vf::FootprintSpec tcFootprint()
{
  vf::FootprintSpec spec;
  spec.polygon = regularPolygon(kRCirc, 16);
  spec.radius = kRCirc;
  return spec;
}

/// @brief The default test configuration TC of math_spec §12: differential drive, FL (auto),
/// contour off, collision detection on.
inline vf::CoreConfig tcConfig()
{
  vf::CoreConfig c;
  c.field.desired_speed = kVr;
  c.field.convergence_gain = kKf;
  c.field.gain_function = vf::GainType::kAtan;
  c.field.direction = 1;
  c.field.assumption3_margin = 0.1;
  c.motion.motion_model = vf::MotionModel::kDifferential;
  c.motion.tracking_law = vf::TrackingLaw::kAuto;
  c.motion.control_point_offset = kD;
  c.motion.drive_backward = false;
  c.motion.allow_reversing = false;
  c.motion.heading_gain = 2.0;
  c.motion.max_linear_vel = kVmax;
  c.motion.max_lateral_vel = kVmax;
  c.motion.max_angular_vel = kWmax;
  c.motion.max_linear_accel = kAccel;
  c.motion.max_linear_decel = kAccel;
  c.motion.max_angular_accel = kAlpha;
  c.phases.terminal_braking_distance = kSigmaB;
  c.phases.min_approach_speed = kVmin;
  c.obstacles.obstacle_mode = vf::ObstacleMode::kNone;
  c.safety.use_collision_detection = true;
  c.safety.collision_check_horizon = kHorizon;
  c.safety.stop_margin = kStopMargin;
  c.node.controller_frequency = 1.0 / kPeriod;
  c.node.footprint = tcFootprint();
  return c;
}

/// @brief TC with `obstacle_mode: contour` (auto \f$\lambda, D_{\mathrm{in}}, D'_{\mathrm{in}}\f$).
inline vf::CoreConfig tcContourConfig()
{
  vf::CoreConfig c = tcConfig();
  c.obstacles.obstacle_mode = vf::ObstacleMode::kContour;
  return c;
}

/// @brief TC with another motion model and law.
inline vf::CoreConfig tcConfig(vf::MotionModel model, vf::TrackingLaw law)
{
  vf::CoreConfig c = tcConfig();
  c.motion.motion_model = model;
  c.motion.tracking_law = law;
  if (model == vf::MotionModel::kAckermann && c.motion.min_turning_radius <= 0.0) {
    c.motion.min_turning_radius = 0.5;
  }
  return c;
}

/// @brief An open-curve goal (math_spec §9.1). xy_reached = nullopt: the core latch decides.
inline vf::GoalInfo goalAt(
  const Vec2 & g, double yaw = 0.0, bool has_yaw = false, double xy_tol = kXyTol,
  double yaw_tol = kYawTol)
{
  vf::GoalInfo goal;
  goal.pose = Pose2{g, yaw};
  goal.has_yaw = has_yaw;
  goal.xy_tol = xy_tol;
  goal.yaw_tol = yaw_tol;
  return goal;
}

// ------------------------------------------------------------------------------------------------
// Curves
// ------------------------------------------------------------------------------------------------

/// @brief A POLYLINE through makeCurve() (follower conditioning; Hinted search). `spacing` <= 0
/// uses CurveOptions::default_sample_spacing (0.05 m).
inline std::shared_ptr<const vf::Curve> makePolyline(
  const std::vector<Vec2> & points, bool closed, double spacing = 0.0)
{
  vf::CurveSpec spec;
  spec.type = vf::CurveSpec::Type::kPolyline;
  spec.closed = closed;
  spec.points = points;
  spec.sample_spacing = spacing;
  return vf::makeCurve(spec, vf::CurveOptions{});
}

/// @brief A POLYLINE whose vertices are kept exactly (no densification: the spacing exceeds
/// every segment).
inline std::shared_ptr<const vf::Curve> makePolylineExact(
  const std::vector<Vec2> & points, bool closed)
{
  return makePolyline(points, closed, 1.0e3);
}

/// @brief A BUILTIN curve with named parameters.
inline std::shared_ptr<const vf::Curve> makeBuiltin(
  const std::string & shape, const std::vector<std::pair<std::string, double>> & parameters = {})
{
  vf::CurveSpec spec;
  spec.type = vf::CurveSpec::Type::kBuiltin;
  spec.closed = true;
  spec.builtin_shape = shape;
  for (const auto & p : parameters) {
    spec.parameter_names.push_back(p.first);
    spec.parameter_values.push_back(p.second);
  }
  return vf::makeCurve(spec, vf::CurveOptions{});
}

/// @brief An EXPRESSION curve x(s, t), y(s, t) on [s_min, s_max].
inline std::shared_ptr<const vf::Curve> makeExpression(
  const std::string & x, const std::string & y, double s_min, double s_max, bool closed)
{
  vf::CurveSpec spec;
  spec.type = vf::CurveSpec::Type::kExpression;
  spec.closed = closed;
  spec.x_expression = x;
  spec.y_expression = y;
  spec.s_min = s_min;
  spec.s_max = s_max;
  return vf::makeCurve(spec, vf::CurveOptions{});
}

/// @brief Points from a to b (both included), at most `spacing` apart.
inline std::vector<Vec2> linePoints(const Vec2 & a, const Vec2 & b, double spacing)
{
  const double length = (b - a).norm();
  const int pieces = std::max(1, static_cast<int>(std::ceil(length / spacing - 1e-9)));
  std::vector<Vec2> out;
  for (int k = 0; k <= pieces; ++k) {
    out.push_back(a + (b - a) * (static_cast<double>(k) / pieces));
  }
  return out;
}

/// @brief Arc of centre c and radius R from angle a0 to a1 (both included; a1 < a0 runs
/// clockwise), at most `spacing` apart along the arc.
inline std::vector<Vec2> arcPoints(const Vec2 & c, double R, double a0, double a1, double spacing)
{
  const int pieces =
    std::max(1, static_cast<int>(std::ceil(std::abs(a1 - a0) * R / spacing - 1e-9)));
  std::vector<Vec2> out;
  for (int k = 0; k <= pieces; ++k) {
    const double a = a0 + (a1 - a0) * (static_cast<double>(k) / pieces);
    out.push_back(c + R * Vec2(std::cos(a), std::sin(a)));
  }
  return out;
}

/// @brief Closed circle (counter-clockwise, no repeated vertex) with N = round(2πR/spacing)
/// vertices, the first at angle `phase`.
inline std::vector<Vec2> circlePoints(const Vec2 & c, double R, double spacing, double phase = 0.0)
{
  const int n = std::max(3, static_cast<int>(std::lround(vf::kTwoPi * R / spacing)));
  std::vector<Vec2> out;
  for (int k = 0; k < n; ++k) {
    const double a = phase + vf::kTwoPi * k / n;
    out.push_back(c + R * Vec2(std::cos(a), std::sin(a)));
  }
  return out;
}

/// @brief Appends `more` to `out`, skipping its first point when it repeats the last one.
inline void appendPoints(std::vector<Vec2> * out, const std::vector<Vec2> & more)
{
  for (std::size_t k = 0; k < more.size(); ++k) {
    if (k == 0 && !out->empty() && (out->back() - more[0]).norm() < 1e-9) {
      continue;
    }
    out->push_back(more[k]);
  }
}

/// @brief Distance from x to the segment [a, b] and the parameter of the foot in [0, 1].
inline double segmentDistance(const Vec2 & x, const Vec2 & a, const Vec2 & b, double * tau)
{
  const Vec2 ab = b - a;
  const double l2 = ab.squaredNorm();
  double t = l2 > 0.0 ? (x - a).dot(ab) / l2 : 0.0;
  t = std::min(1.0, std::max(0.0, t));
  if (tau != nullptr) {
    *tau = t;
  }
  return (x - (a + t * ab)).norm();
}

/// @brief Brute-force distance from x to a polyline (segments only, no rays) and the station of
/// the foot; an independent check of the Tracker's D and s*.
inline double polylineDistance(
  const std::vector<Vec2> & points, bool closed, const Vec2 & x, double * station = nullptr)
{
  double best = vf::kInf;
  double s_best = 0.0;
  double s = 0.0;
  const std::size_t n = points.size();
  const std::size_t segments = closed ? n : n - 1;
  for (std::size_t k = 0; k < segments; ++k) {
    const Vec2 & a = points[k];
    const Vec2 & b = points[(k + 1) % n];
    double tau = 0.0;
    const double dist = segmentDistance(x, a, b, &tau);
    const double length = (b - a).norm();
    if (dist < best) {
      best = dist;
      s_best = s + tau * length;
    }
    s += length;
  }
  if (station != nullptr) {
    *station = s_best;
  }
  return best;
}

/// @brief Length of a polyline.
inline double polylineLength(const std::vector<Vec2> & points, bool closed)
{
  double s = 0.0;
  const std::size_t n = points.size();
  const std::size_t segments = closed ? n : n - 1;
  for (std::size_t k = 0; k < segments; ++k) {
    s += (points[(k + 1) % n] - points[k]).norm();
  }
  return s;
}

// ------------------------------------------------------------------------------------------------
// Obstacles
// ------------------------------------------------------------------------------------------------

/// @brief World-fixed discs and boxes rasterised on an OccupancyGrid whose frame is the
/// evaluation frame (plugin convention: `T_grid_eval` = identity).
///
/// A cell is a site iff its centre lies in an enabled shape (closed disc or closed box), so
/// shapes whose edges lie on grid lines rasterise exactly. setOrigin() moves the grid (a rolling
/// window) while the shapes stay fixed. Every brute-force distance is to site **centres**
/// (math_spec §6.1).
class ObstacleWorld
{
public:
  /// @brief A grid with its corner at `origin`, covering width_m × height_m.
  ObstacleWorld(const Vec2 & origin, double width_m, double height_m, double res = kRes)
  : origin_(origin),
    width_(static_cast<int>(std::lround(width_m / res))),
    height_(static_cast<int>(std::lround(height_m / res))),
    res_(res)
  {
    commit();
  }

  /// @brief Adds a closed disc; returns its id.
  int addDisc(const Vec2 & centre, double radius, int tag = 0)
  {
    shapes_.push_back(Shape{true, centre, centre, radius, tag, true});
    return static_cast<int>(shapes_.size()) - 1;
  }

  /// @brief Adds a closed axis-aligned box [lo, hi]; returns its id.
  int addBox(const Vec2 & lo, const Vec2 & hi, int tag = 0)
  {
    shapes_.push_back(Shape{false, lo, hi, 0.0, tag, true});
    return static_cast<int>(shapes_.size()) - 1;
  }

  /// @brief Enables or disables a shape (call commit() afterwards).
  void setEnabled(int id, bool enabled) { shapes_[static_cast<std::size_t>(id)].enabled = enabled; }

  /// @brief Moves a box (call commit() afterwards).
  void moveBox(int id, const Vec2 & lo, const Vec2 & hi)
  {
    shapes_[static_cast<std::size_t>(id)].a = lo;
    shapes_[static_cast<std::size_t>(id)].b = hi;
  }

  /// @brief Moves the grid origin (rolling window; call commit() afterwards).
  void setOrigin(const Vec2 & origin) { origin_ = origin; }

  /// @brief Rasterises the shapes into the grid and updates the EDT.
  void commit()
  {
    mask_.assign(static_cast<std::size_t>(width_ * height_), 0);
    sites_.clear();
    for (int iy = 0; iy < height_; ++iy) {
      for (int ix = 0; ix < width_; ++ix) {
        const Vec2 c = cellCentre(ix, iy);
        std::uint32_t tags = 0;
        for (const Shape & s : shapes_) {
          if (s.enabled && contains(s, c)) {
            tags |= 1U << static_cast<unsigned>(s.tag);
          }
        }
        if (tags != 0U) {
          mask_[static_cast<std::size_t>(iy * width_ + ix)] = 1;
          sites_.push_back(Site{c, tags});
        }
      }
    }
    grid_.setFromMask(
      vf::Span<const std::uint8_t>(mask_.data(), mask_.size()), width_, height_, res_, origin_);
    edt_.update(grid_);
  }

  /// @brief The obstacle input of a cycle (identity T_grid_eval).
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
  const Vec2 & origin() const { return origin_; }
  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return res_; }

  /// @brief Centre of cell (ix, iy) in the grid (= evaluation) frame.
  Vec2 cellCentre(int ix, int iy) const { return origin_ + res_ * Vec2(ix + 0.5, iy + 0.5); }

  /// @brief True iff a point lies in an enabled shape with this tag (any tag when tag < 0).
  bool inShape(const Vec2 & x, int tag = -1) const
  {
    for (const Shape & s : shapes_) {
      if (s.enabled && (tag < 0 || s.tag == tag) && contains(s, x)) {
        return true;
      }
    }
    return false;
  }

  /// @brief Brute-force distance from x to the nearest site centre (of shapes with this tag
  /// when tag >= 0); +inf without such sites.
  double distanceToSites(const Vec2 & x, int tag = -1) const
  {
    double best = vf::kInf;
    for (const Site & s : sites_) {
      if (tag >= 0 && (s.tags & (1U << static_cast<unsigned>(tag))) == 0U) {
        continue;
      }
      best = std::min(best, (x - s.centre).norm());
    }
    return best;
  }

  /// @brief Number of site cells (of shapes with this tag when tag >= 0).
  std::size_t siteCount(int tag = -1) const
  {
    std::size_t n = 0;
    for (const Site & s : sites_) {
      if (tag < 0 || (s.tags & (1U << static_cast<unsigned>(tag))) != 0U) {
        ++n;
      }
    }
    return n;
  }

private:
  struct Shape
  {
    bool disc;
    Vec2 a;
    Vec2 b;
    double r;
    int tag;
    bool enabled;
  };
  struct Site
  {
    Vec2 centre;
    std::uint32_t tags;
  };

  static bool contains(const Shape & s, const Vec2 & x)
  {
    if (s.disc) {
      return (x - s.a).norm() <= s.r + 1e-12;
    }
    return x.x() >= s.a.x() - 1e-12 && x.x() <= s.b.x() + 1e-12 && x.y() >= s.a.y() - 1e-12 &&
           x.y() <= s.b.y() + 1e-12;
  }

  Vec2 origin_;
  int width_;
  int height_;
  double res_;
  std::vector<Shape> shapes_;
  std::vector<Site> sites_;
  std::vector<std::uint8_t> mask_;
  vf::OccupancyGrid grid_;
  vf::EuclideanDistanceTransform edt_;
};

/// @brief Ground-truth contact distance: how far the footprint can translate from `pose` along
/// `direction` (unit, evaluation frame) before it collides (1 mm steps, up to `max_travel`).
inline double travelToContact(
  const vf::FootprintChecker & checker, const ObstacleWorld & world, const Pose2 & pose,
  const Vec2 & direction, double max_travel = 2.0)
{
  for (double s = 0.0; s <= max_travel; s += 1e-3) {
    const Pose2 p{pose.p + s * direction, pose.yaw};
    if (checker.collidesGroundTruth(p, world.grid())) {
      return s;
    }
  }
  return vf::kInf;
}

// ------------------------------------------------------------------------------------------------
// Plugin-style window (math_spec §2.8 "Plugin window")
// ------------------------------------------------------------------------------------------------

/// @brief A raw plan re-pruned around the robot every cycle, Gen A/B style: the start index is
/// the closest raw pose within `search_distance` of path length ahead of the previous start (it
/// never decreases); the window holds the poses up to `prune_distance` of path length; then
/// dedupe (1 mm) and, when the window ends at the goal, tail conditioning with the cached end
/// tangent; a PolylineCurve with WindowScan [s_min, s_max] (plugin: [−(d + 0.3), d + 0.5]).
class PrunedWindow
{
public:
  PrunedWindow(
    std::vector<Vec2> raw, double prune_distance, double s_min = -(kD + 0.3),
    double s_max = kD + 0.5, double search_distance = 1.5)
  : raw_(std::move(raw)),
    prune_(prune_distance),
    search_(search_distance),
    chord_(vf::chordLength(kRes)),
    curve_(vf::SearchPolicy::windowScan(s_min, s_max))
  {
    const vf::EndTangent tangent =
      vf::endTangent(vf::Span<const Vec2>(raw_.data(), raw_.size()), chord_);
    end_tangent_ = tangent.valid ? tangent.tangent : Vec2::UnitX();
  }

  /// @brief Re-prunes around `robot` and re-assigns the window curve.
  const vf::PolylineCurve & update(const Vec2 & robot)
  {
    std::size_t best = start_;
    double best_d = vf::kInf;
    double travelled = 0.0;
    for (std::size_t k = start_; k < raw_.size(); ++k) {
      if (k > start_) {
        travelled += (raw_[k] - raw_[k - 1]).norm();
        if (travelled > search_) {
          break;
        }
      }
      const double dist = (raw_[k] - robot).norm();
      if (dist < best_d) {
        best_d = dist;
        best = k;
      }
    }
    start_ = best;
    window_.clear();
    double length = 0.0;
    for (std::size_t k = start_; k < raw_.size(); ++k) {
      if (k > start_) {
        length += (raw_[k] - raw_[k - 1]).norm();
        if (length > prune_) {
          break;
        }
      }
      window_.push_back(raw_[k]);
    }
    vf::dedupe(vf::Span<const Vec2>(window_.data(), window_.size()), false, &deduped_);
    if (deduped_.size() < 2) {
      const std::array<Vec2, 2> one = vf::onePoseWindow(
        robot, Pose2{raw_.back(), std::atan2(end_tangent_.y(), end_tangent_.x())});
      deduped_.assign(one.begin(), one.end());
    }
    ends_at_goal_ = (deduped_.back() - raw_.back()).norm() < vf::kWindowAtGoalDistance;
    if (ends_at_goal_) {
      vf::tailCondition(
        vf::Span<const Vec2>(deduped_.data(), deduped_.size()), end_tangent_, chord_,
        &conditioned_);
    } else {
      conditioned_ = deduped_;
    }
    status_ = curve_.assign(vf::Span<const Vec2>(conditioned_.data(), conditioned_.size()), false);
    return curve_;
  }

  const vf::PolylineCurve & curve() const { return curve_; }
  const Vec2 & goal() const { return raw_.back(); }
  bool endsAtGoal() const { return ends_at_goal_; }
  vf::AssignStatus status() const { return status_; }
  std::size_t start() const { return start_; }

private:
  std::vector<Vec2> raw_;
  double prune_;
  double search_;
  double chord_;
  vf::PolylineCurve curve_;
  Vec2 end_tangent_{Vec2::UnitX()};
  std::size_t start_{0};
  bool ends_at_goal_{false};
  vf::AssignStatus status_{vf::AssignStatus::kTooFew};
  std::vector<Vec2> window_;
  std::vector<Vec2> deduped_;
  std::vector<Vec2> conditioned_;
};

// ------------------------------------------------------------------------------------------------
// Closed loop
// ------------------------------------------------------------------------------------------------

/// @brief Everything recorded about one control period.
///
/// `out.predicted` views the simulator's scratch and is only valid until the next step: never
/// read it from a stored Cycle.
struct Cycle
{
  /// Simulator record (pose at the start of the period, command held, phase, status, ...).
  vf::sim::SimulationRecord rec;
  /// Full Tracker output of the period.
  vf::TrackerOutput out;
  /// TrackerState before the step.
  vf::TrackerState before;
  /// TrackerState after the step.
  vf::TrackerState after;
  /// The input as the Tracker saw it (pose, measured, t and dt filled in).
  vf::TrackerInput input;
  /// Plant pose at the end of the period.
  Pose2 pose_after;
};

/// @brief A closed loop on top of sim::ClosedLoopSimulator that records full outputs and states.
class Runner
{
public:
  /// @brief Hook run before every step; may edit the input template (curve, goal, obstacles,
  /// cancel, frame, speed limit).
  using PreStep = std::function<void(Runner &, vf::TrackerInput &)>;

  explicit Runner(const vf::CoreConfig & config, vf::sim::SimulationOptions options = {})
  : sim_(vf::Tracker(config), options), period_(options.period)
  {
  }

  Runner(
    const vf::CoreConfig & config, const vf::Kinematics & plant, vf::sim::SimulationOptions options)
  : sim_(vf::Tracker(config), plant, options), period_(options.period)
  {
  }

  /// @brief Resets the plant, the time, the TrackerState and the records.
  void reset(const Pose2 & pose, double t0 = 0.0, const Twist2 & twist = {})
  {
    sim_.reset(pose, t0, twist);
    cycles_.clear();
  }

  /// @brief The input template.
  vf::TrackerInput & input() { return input_; }
  void setPreStep(PreStep hook) { pre_ = std::move(hook); }

  /// @brief One control period.
  const Cycle & step()
  {
    if (pre_) {
      pre_(*this, input_);
    }
    Cycle c;
    c.before = sim_.state();
    c.rec = sim_.step(input_);
    c.out = sim_.lastOutput();
    c.after = sim_.state();
    c.input = input_;
    c.input.pose = c.rec.pose;
    c.input.measured = c.rec.measured;
    c.input.t = c.rec.t;
    c.input.dt = period_;
    c.pose_after = sim_.pose();
    cycles_.push_back(c);
    return cycles_.back();
  }

  /// @brief Steps until `duration` elapses or `stop` returns true (checked after each step).
  /// @return The number of periods simulated.
  std::size_t run(double duration, const std::function<bool(const Cycle &)> & stop = {})
  {
    const std::size_t steps = static_cast<std::size_t>(std::lround(duration / period_));
    for (std::size_t k = 0; k < steps; ++k) {
      const Cycle & c = step();
      if (stop && stop(c)) {
        return k + 1;
      }
    }
    return steps;
  }

  const std::vector<Cycle> & cycles() const { return cycles_; }
  vf::sim::ClosedLoopSimulator & sim() { return sim_; }
  const vf::sim::ClosedLoopSimulator & sim() const { return sim_; }
  const vf::Tracker & tracker() const { return sim_.tracker(); }
  const Pose2 & pose() const { return sim_.pose(); }
  double time() const { return sim_.time(); }
  double period() const { return period_; }

private:
  vf::sim::ClosedLoopSimulator sim_;
  double period_;
  vf::TrackerInput input_;
  PreStep pre_;
  std::vector<Cycle> cycles_;
};

// ------------------------------------------------------------------------------------------------
// Metrics (math_spec §12)
// ------------------------------------------------------------------------------------------------

/// @brief Ḋ of math_spec §12 for FL: \f$n^\top(\dot x - v_c)\f$ with \f$\dot x = v\mathbf h +
/// q\,d\,\omega J\mathbf h\f$ from the command before the acceleration stage (cmd_target), n and
/// \f$v_c\f$ at the law's point.
inline double flDdot(const Cycle & c, double d, double q = 1.0)
{
  const Vec2 h = vf::heading(c.input.pose.yaw);
  const Vec2 xdot = c.out.cmd_target.vx * h + q * d * c.out.cmd_target.wz * vf::J(h);
  return c.out.field.n.dot(xdot - c.out.field.v_c);
}

/// @brief Ḋ of math_spec §12 for the holonomic law: \f$\dot x = R(\psi)(v_x, v_y)\f$ from
/// cmd_target.
inline double holonomicDdot(const Cycle & c)
{
  const Vec2 xdot = vf::rotate(c.input.pose.yaw, Vec2(c.out.cmd_target.vx, c.out.cmd_target.vy));
  return c.out.field.n.dot(xdot - c.out.field.v_c);
}

/// @brief Number of entries into `phase` (cycles whose phase is `phase` while the previous
/// cycle's, or the initial state's, is not).
inline int countEntries(const std::vector<Cycle> & cycles, Phase phase)
{
  int n = 0;
  for (std::size_t k = 0; k < cycles.size(); ++k) {
    const Phase previous = k == 0 ? cycles[0].before.phase : cycles[k - 1].out.phase;
    if (cycles[k].out.phase == phase && previous != phase) {
      ++n;
    }
  }
  return n;
}

/// @brief Index of the first cycle satisfying `pred`, or cycles.size().
inline std::size_t firstIndex(
  const std::vector<Cycle> & cycles, const std::function<bool(const Cycle &)> & pred)
{
  for (std::size_t k = 0; k < cycles.size(); ++k) {
    if (pred(cycles[k])) {
      return k;
    }
  }
  return cycles.size();
}

/// @brief A contour episode: cycles [start, end) with an active episode after the step.
struct Episode
{
  std::size_t start{0};
  std::size_t end{0};
  int sigma_c{0};
  bool sigma_constant{true};
};

/// @brief The contour episodes of a run (from TrackerState::contour after each step).
inline std::vector<Episode> episodes(const std::vector<Cycle> & cycles)
{
  std::vector<Episode> out;
  bool active = false;
  for (std::size_t k = 0; k < cycles.size(); ++k) {
    const vf::ContourState & cs = cycles[k].after.contour;
    if (cs.active && !active) {
      out.push_back(Episode{k, cycles.size(), cs.sigma_c, true});
    } else if (cs.active && active && cs.sigma_c != out.back().sigma_c) {
      out.back().sigma_constant = false;
    } else if (!cs.active && active) {
      out.back().end = k;
    }
    active = cs.active;
  }
  return out;
}

/// @brief True iff any cycle has all bits of `flag` set.
inline bool anyStatus(const std::vector<Cycle> & cycles, std::uint32_t flag)
{
  for (const Cycle & c : cycles) {
    if (Status::has(c.out.status, flag)) {
      return true;
    }
  }
  return false;
}

/// @brief Signed progress along a closed curve: the sum of wrapped differences of the record's
/// s* (stations at the law's point), with period L.
inline double closedProgress(const std::vector<Cycle> & cycles, double length)
{
  double total = 0.0;
  for (std::size_t k = 1; k < cycles.size(); ++k) {
    total += vf::wrapPeriodic(cycles[k].rec.s_star - cycles[k - 1].rec.s_star, length);
  }
  return total;
}

/// @brief Base path length travelled up to and including cycle `index`.
inline double odometry(const std::vector<Cycle> & cycles, std::size_t index)
{
  double total = 0.0;
  for (std::size_t k = 0; k <= index && k < cycles.size(); ++k) {
    total += (cycles[k].pose_after.p - cycles[k].rec.pose.p).norm();
  }
  return total;
}

// ------------------------------------------------------------------------------------------------
// Straight-path contour runs (O31–O34, upstream regressions)
// ------------------------------------------------------------------------------------------------

/// @brief A grid whose cell centres lie on multiples of 0.05 m (so (2, 0) is a centre).
inline ObstacleWorld centredWorld(double x0, double x1, double y0, double y1)
{
  return ObstacleWorld(Vec2(x0 - 0.025, y0 - 0.025), x1 - x0, y1 - y0);
}

/// @brief Outcome of runStraight().
struct StraightRun
{
  std::vector<Cycle> cycles;
  std::vector<Episode> eps;
};

/// @brief Runs the configured law along y = 0 from `start` toward the goal (x_goal, 0) until
/// settled (+0.5 s), `stop` returns true, or the timeout. `per_step` (optional) inspects the
/// simulator after each step (scratch, K).
inline StraightRun runStraight(
  const vf::CoreConfig & config, ObstacleWorld & world, const Pose2 & start, double x_goal,
  double timeout, const std::function<void(const Cycle &, const vf::StepScratch &)> & per_step = {},
  const std::function<bool(const Cycle &)> & stop = {})
{
  const auto curve = makePolyline({Vec2(start.p.x() - 1.0, 0.0), Vec2(x_goal, 0.0)}, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(config, options);
  runner.reset(start);
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(x_goal, 0.0));
  runner.input().obstacles = world.map();
  const std::size_t steps = static_cast<std::size_t>(std::lround(timeout / kPeriod));
  double settled_at = -1.0;
  for (std::size_t k = 0; k < steps; ++k) {
    const Cycle & c = runner.step();
    if (per_step) {
      per_step(c, runner.sim().scratch());
    }
    if (c.out.settled && settled_at < 0.0) {
      settled_at = c.rec.t;
    }
    if ((settled_at >= 0.0 && c.rec.t > settled_at + 0.5) || (stop && stop(c))) {
      break;
    }
  }
  StraightRun r;
  r.cycles = runner.cycles();
  r.eps = episodes(r.cycles);
  return r;
}

// ------------------------------------------------------------------------------------------------
// Shared scenario: box centred on the path with a parallel wall (O35, U17)
// ------------------------------------------------------------------------------------------------

/// @brief Outcome of runBoxWithWall().
struct BoxWallResult
{
  std::vector<Cycle> cycles;
  std::vector<Episode> eps;
  /// Lateral base position when the base crosses the box centre (x = 3); NaN if never.
  double y_at_box{vf::kNaN};
  /// Number of consecutive in-episode cycle pairs whose n_o reversed: turned by more than 120°
  /// (n_k · n_{k−1} < −0.5), i.e. flipped toward the opposite side. A switch of the nearest point
  /// between two faces of K at a concave corner turns n_o by about 90° and is not a reversal.
  int n_o_reversals{0};
  /// Largest turn of n_o between consecutive in-episode cycles (degrees), for the record.
  double max_n_o_turn_deg{0.0};
  /// Number of cycles with kBlocked.
  int blocked_cycles{0};
};

/// @brief TC with contour; path y = 0 from (−1, 0) to the goal (6.5, 0); a 0.6 × 0.6 m box
/// centred at (3, 0); a 0.1 m thick wall from x = 2.2 to 3.8 (0.5 m beyond each end of the box)
/// parallel to the path at `gap` from the box, on the side `wall_side` (+1: y > 0); the robot
/// starts at (0.5, initial_error) heading +x (before the gate can open). Runs until the base is
/// past x = 4.0 (beyond the box and the 1.6 m wall) with no active episode, or `timeout`.
///
/// The wall length is not specified by math_spec O35. For gaps >= 0.45 m it does not matter. At
/// 0.3 m the wall merges with the box into one K, and a point-level reference of the math_spec §6.4
/// rules (gate, start, side ranking, exits) meets O35 only when the wall starts 0.3–0.7 m before
/// the box: longer walls put K beside the path where the gate opens (episodes chatter on the wall
/// side), a wall as short as the box is circumnavigated on the wall side. 0.5 m is the middle.
/// `wall_x0`, `wall_x1`: the wall's extent along x (default the 1.6 m wall described above).
inline BoxWallResult runBoxWithWall(
  double gap, int wall_side, double initial_error, double timeout = 25.0, double wall_x0 = 2.2,
  double wall_x1 = 3.8)
{
  // The grid covers the robot, its rollouts and the wall: x from min(0, wall_x0 − 0.5) to 5.5,
  // |y| <= 1.75 (the circulation stays within |y| < 0.9).
  const double grid_x0 = std::min(0.0, wall_x0 - 0.5);
  ObstacleWorld world(Vec2(grid_x0, -1.75), 5.5 - grid_x0, 3.5);
  world.addBox(Vec2(2.7, -0.3), Vec2(3.3, 0.3), 1);
  const double y_near = wall_side * (0.3 + gap);
  const double y_far = wall_side * (0.3 + gap + 0.1);
  world.addBox(Vec2(wall_x0, std::min(y_near, y_far)), Vec2(wall_x1, std::max(y_near, y_far)), 2);
  world.commit();
  const auto curve = makePolyline({Vec2(-1.0, 0.0), Vec2(6.5, 0.0)}, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(tcContourConfig(), options);
  runner.reset(Pose2{Vec2(0.5, initial_error), 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(6.5, 0.0));
  runner.input().obstacles = world.map();
  BoxWallResult r;
  runner.run(
    timeout, [](const Cycle & c) { return c.pose_after.p.x() > 4.0 && !c.after.contour.active; });
  r.cycles = runner.cycles();
  r.eps = episodes(r.cycles);
  for (std::size_t k = 0; k < r.cycles.size(); ++k) {
    const Cycle & c = r.cycles[k];
    if (std::isnan(r.y_at_box) && c.rec.pose.p.x() <= 3.0 && c.pose_after.p.x() > 3.0) {
      r.y_at_box = 0.5 * (c.rec.pose.p.y() + c.pose_after.p.y());
    }
    if (Status::has(c.out.status, Status::kBlocked)) {
      ++r.blocked_cycles;
    }
    if (k > 0 && c.after.contour.active && r.cycles[k - 1].after.contour.active) {
      const double dot = c.out.field.contour.n_o.dot(r.cycles[k - 1].out.field.contour.n_o);
      r.max_n_o_turn_deg = std::max(
        r.max_n_o_turn_deg, std::acos(std::max(-1.0, std::min(1.0, dot))) * 180.0 / vf::kPi);
      if (dot < -0.5) {
        ++r.n_o_reversals;
      }
    }
  }
  return r;
}

/// @brief Bitwise equality of doubles (NaN-safe, distinguishes ±0).
inline bool bitEqual(double a, double b)
{
  std::uint64_t ua = 0;
  std::uint64_t ub = 0;
  std::memcpy(&ua, &a, sizeof(a));
  std::memcpy(&ub, &b, sizeof(b));
  return ua == ub;
}

inline bool bitEqual(const Vec2 & a, const Vec2 & b)
{
  return bitEqual(a.x(), b.x()) && bitEqual(a.y(), b.y());
}

inline bool bitEqual(const Twist2 & a, const Twist2 & b)
{
  return bitEqual(a.vx, b.vx) && bitEqual(a.vy, b.vy) && bitEqual(a.wz, b.wz);
}

/// @brief Member-wise bitwise equality of two TrackerStates (tracker.hpp, every member).
inline bool bitEqual(const vf::TrackerState & a, const vf::TrackerState & b)
{
  const vf::ContourState & ca = a.contour;
  const vf::ContourState & cb = b.contour;
  return a.phase == b.phase && bitEqual(a.hint.x_star, b.hint.x_star) &&
         bitEqual(a.hint.s, b.hint.s) && a.hint.segment == b.hint.segment &&
         a.hint.valid == b.hint.valid && bitEqual(a.last_tangent, b.last_tangent) &&
         bitEqual(a.last_psi_d, b.last_psi_d) && ca.gate_open == cb.gate_open &&
         bitEqual(ca.gate_clear_s, cb.gate_clear_s) && ca.active == cb.active &&
         ca.sigma_c == cb.sigma_c && bitEqual(ca.travelled_m, cb.travelled_m) &&
         bitEqual(ca.elapsed_s, cb.elapsed_s) && bitEqual(ca.k_seed, cb.k_seed) &&
         ca.k_valid == cb.k_valid && bitEqual(ca.b_site, cb.b_site) && ca.b_valid == cb.b_valid &&
         a.blocked == b.blocked && a.terminal_mode == b.terminal_mode &&
         a.xy_latched == b.xy_latched && a.holding == b.holding && a.goal_seq == b.goal_seq &&
         bitEqual(a.limiter.u_prev, b.limiter.u_prev) &&
         a.limiter.cancel_hard == b.limiter.cancel_hard && a.limiter.valid == b.limiter.valid &&
         bitEqual(a.rotate_target, b.rotate_target) && a.stopped_status == b.stopped_status &&
         a.ackermann_band_lifted == b.ackermann_band_lifted;
}

/// @brief Bitwise equality of the decision-relevant parts of two outputs.
inline bool bitEqual(const vf::TrackerOutput & a, const vf::TrackerOutput & b)
{
  return bitEqual(a.cmd, b.cmd) && bitEqual(a.cmd_target, b.cmd_target) && a.phase == b.phase &&
         a.status == b.status && a.settled == b.settled && bitEqual(a.s_free, b.s_free) &&
         bitEqual(a.cap, b.cap) && bitEqual(a.field.F, b.field.F) &&
         bitEqual(a.field.x_star, b.field.x_star) && bitEqual(a.field.D, b.field.D) &&
         bitEqual(a.field.s_star, b.field.s_star) && bitEqual(a.field.sigma, b.field.sigma) &&
         bitEqual(a.field.psi_d_dot, b.field.psi_d_dot) &&
         a.field.contour.active == b.field.contour.active &&
         a.field.contour.sigma_c == b.field.contour.sigma_c &&
         bitEqual(a.field.contour.D_o, b.field.contour.D_o) &&
         a.predicted.size() == b.predicted.size() && a.first_collision == b.first_collision;
}

}  // namespace vf_test

#endif  // VECTORFIELD_CORE__TEST_SUPPORT_HPP_
