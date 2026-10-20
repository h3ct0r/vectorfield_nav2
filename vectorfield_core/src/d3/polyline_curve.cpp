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

// PolylineCurve3 (math_spec §2.2–§2.4, §14.2) and Curve3::sampleStations().

#include "vectorfield_core/d3/polyline_curve.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/geometry.hpp"

namespace vectorfield_core::d3
{

std::size_t Curve3::sampleStations(
  double s0, double s1, double ds, double t, Span<Vec3> out) const noexcept
{
  if (!(ds > 0.0) || !std::isfinite(s0) || !std::isfinite(s1) || !std::isfinite(ds)) {
    return 0;
  }
  if (!isClosed()) {
    const double end = length(t);
    s0 = std::max(s0, 0.0);
    s1 = std::min(s1, end);
  }
  if (s1 < s0) {
    return 0;
  }
  std::size_t count = 0;
  double last = s0;
  for (std::size_t k = 0; count < out.size(); ++k) {
    const double s = s0 + static_cast<double>(k) * ds;
    if (s > s1) {
      break;
    }
    out[count++] = pointAt(s, t);
    last = s;
  }
  if (count < out.size() && s1 - last > 1e-9) {
    out[count++] = pointAt(s1, t);
  }
  return count;
}

namespace
{

[[noreturn]] void reject(const std::string & message)
{
  throw CurveError(CurveError::Code::kInvalidCurve, "POLYLINE (3D): " + message);
}

// Piece identifiers of a candidate.
constexpr int kBackward = -1;
constexpr int kForward = -2;

}  // namespace

struct PolylineCurve3::Candidate
{
  // Segment index, or kBackward / kForward for the rays.
  int k{0};
  // Clamped ray/segment coordinate.
  double tau{0.0};
  Vec3 foot{Vec3::Zero()};
  double D{kInf};
  // Arc-length station (unwrapped: [0, L] on segments).
  double s{0.0};
  bool valid{false};
};

PolylineCurve3::PolylineCurve3(
  const std::vector<Vec3> & points, bool closed, const CurveOptions & options)
: closed_(closed), options_(options)
{
  const ValidationReport report = validateCurveOptions(options);
  if (!report.ok()) {
    reject(report.summary());
  }
  for (const Vec3 & p : points) {
    if (!isFinite(p)) {
      reject("a vertex is not finite");
    }
    if (points_.empty() || (p - points_.back()).norm() >= kDedupeDistance) {
      points_.push_back(p);
    }
  }
  if (
    closed_ && points_.size() > 1 && (points_.back() - points_.front()).norm() < kDedupeDistance) {
    points_.pop_back();
  }
  const std::size_t needed = closed_ ? 3 : 2;
  if (points_.size() < needed) {
    reject(
      "needs at least " + std::to_string(needed) + " distinct vertices (1 mm apart), got " +
      std::to_string(points_.size()));
  }
  if (points_.size() > static_cast<std::size_t>(options.max_curve_points)) {
    reject(
      std::to_string(points_.size()) +
      " vertices exceed max_curve_points = " + std::to_string(options.max_curve_points));
  }
  const std::size_t n = points_.size();
  const std::size_t segments = closed_ ? n : n - 1;
  tangents_.resize(segments);
  lengths_.resize(segments);
  stations_.resize(segments + 1);
  stations_[0] = 0.0;
  for (std::size_t k = 0; k < segments; ++k) {
    const Vec3 d = points_[(k + 1) % n] - points_[k];
    lengths_[k] = d.norm();
    tangents_[k] = d / lengths_[k];
    stations_[k + 1] = stations_[k] + lengths_[k];
  }
}

double PolylineCurve3::length(double /*t*/) const noexcept { return stations_.back(); }

Vec3 PolylineCurve3::velocity(const ClosestPoint3 & /*cp*/, double /*t*/) const noexcept
{
  return Vec3::Zero();
}

int PolylineCurve3::segmentAt(double s) const noexcept
{
  const int segments = segmentCount();
  const auto it = std::upper_bound(stations_.begin(), stations_.end(), s);
  const int k = static_cast<int>(it - stations_.begin()) - 1;
  return std::min(std::max(k, 0), segments - 1);
}

Vec3 PolylineCurve3::pointAt(double s, double /*t*/) const noexcept
{
  if (!std::isfinite(s)) {
    return Vec3::Constant(kNaN);
  }
  const double L = stations_.back();
  if (closed_) {
    s = std::fmod(s, L);
    if (s < 0.0) {
      s += L;
    }
  } else if (s < 0.0) {
    return points_.front() + s * tangents_.front();
  } else if (s > L) {
    return points_.back() + (s - L) * tangents_.back();
  }
  const int k = segmentAt(s);
  const auto index = static_cast<std::size_t>(k);
  return points_[index] + (s - stations_[index]) * tangents_[index];
}

Vec3 PolylineCurve3::tangentAt(double s, double /*t*/) const noexcept
{
  if (!std::isfinite(s)) {
    return Vec3::Constant(kNaN);
  }
  const double L = stations_.back();
  if (closed_) {
    s = std::fmod(s, L);
    if (s < 0.0) {
      s += L;
    }
  } else if (s < 0.0) {
    return tangents_.front();
  } else if (s >= L) {
    return tangents_.back();
  }
  return tangents_[static_cast<std::size_t>(segmentAt(s))];
}

PolylineCurve3::Candidate PolylineCurve3::project(const Vec3 & x, int k) const noexcept
{
  const auto index = static_cast<std::size_t>(k);
  Candidate c;
  c.k = k;
  const double tau_hat = (x - points_[index]).dot(tangents_[index]);
  c.tau = clamp(tau_hat, 0.0, lengths_[index]);
  c.foot = points_[index] + c.tau * tangents_[index];
  c.D = (x - c.foot).norm();
  c.s = stations_[index] + c.tau;
  c.valid = true;
  return c;
}

PolylineCurve3::Candidate PolylineCurve3::projectRay(const Vec3 & x, bool forward) const noexcept
{
  Candidate c;
  c.k = forward ? kForward : kBackward;
  const Vec3 & anchor = forward ? points_.back() : points_.front();
  const Vec3 & t = forward ? tangents_.back() : tangents_.front();
  const double tau_hat = (x - anchor).dot(t);
  c.tau = forward ? std::max(0.0, tau_hat) : std::min(0.0, tau_hat);
  c.foot = anchor + c.tau * t;
  c.D = (x - c.foot).norm();
  c.s = forward ? stations_.back() + c.tau : c.tau;
  c.valid = true;
  return c;
}

PolylineCurve3::Candidate PolylineCurve3::scan(
  const Vec3 & x, int k0, int count, const EvalContext3 & context,
  const CurveHint3 & hint) const noexcept
{
  const int segments = segmentCount();
  // Ties prefer the station nearest the hint, or the smallest dir·s at initialisation.
  const auto preference = [&](const Candidate & c) {
    if (hint.valid) {
      return std::abs(wrapStationDifference(c.s - hint.s, 0.0));
    }
    return static_cast<double>(context.direction < 0 ? -1 : 1) * c.s;
  };
  Candidate best;
  const auto consider = [&](const Candidate & c) {
    if (
      !best.valid || c.D < best.D - kTieTolerance ||
      (isTie(c.D, best.D) && preference(c) < preference(best))) {
      best = c;
    }
  };
  for (int j = 0; j < count; ++j) {
    int k = k0 + j;
    if (closed_) {
      k = ((k % segments) + segments) % segments;
    }
    consider(project(x, k));
  }
  if (!closed_) {
    if (k0 == 0) {
      consider(projectRay(x, false));
    }
    if (k0 + count - 1 == segments - 1) {
      consider(projectRay(x, true));
    }
  }
  return best;
}

ClosestPoint3 PolylineCurve3::closestPoint(
  const Vec3 & x, const EvalContext3 & context, const CurveHint3 & hint) const noexcept
{
  if (!isFinite(x) || points_.size() < 2) {
    return ClosestPoint3{};
  }
  const int segments = segmentCount();
  const double L = stations_.back();
  if (!hint.valid) {
    return finish(x, scan(x, 0, segments, context, hint), context);
  }

  // Local window [s_prev - w/2, s_prev + w/2], extended while the winner sits on its edge.
  const double half = 0.5 * options_.search_window;
  int k0 = 0;
  int count = 1;
  if (closed_) {
    double a = std::fmod(hint.s - half, L);
    if (a < 0.0) {
      a += L;
    }
    k0 = segmentAt(a);
    double covered = stations_[static_cast<std::size_t>(k0) + 1] - a;
    while (covered < 2.0 * half && count < segments) {
      covered += lengths_[static_cast<std::size_t>((k0 + count) % segments)];
      ++count;
    }
  } else {
    const double a = std::max(hint.s - half, 0.0);
    const double b = std::min(hint.s + half, L);
    if (a > b) {
      k0 = hint.s + half < 0.0 ? 0 : segments - 1;
    } else {
      k0 = segmentAt(a);
      count = segmentAt(b) - k0 + 1;
    }
  }
  Candidate local = scan(x, k0, count, context, hint);
  const auto wrapped = [&](int k) { return closed_ ? ((k % segments) + segments) % segments : k; };
  for (int guard = 0; guard < segments; ++guard) {
    const int k_first = wrapped(k0);
    const int k_last = wrapped(k0 + count - 1);
    const bool at_start = local.k == k_first && local.tau <= 0.0;
    const bool at_end =
      local.k == k_last && local.tau >= lengths_[static_cast<std::size_t>(k_last)];
    if (count >= segments || !(at_start || at_end)) {
      break;
    }
    if (at_start && (closed_ || k0 > 0)) {
      --k0;
      ++count;
    } else if (at_end && (closed_ || k0 + count < segments)) {
      ++count;
    } else {
      break;
    }
    local = scan(x, k0, count, context, hint);
  }

  ClosestPoint3 result = finish(x, local, context);
  if (local.D > options_.global_search_trigger) {
    const Candidate global = scan(x, 0, segments, context, hint);
    const double arc = std::abs(wrapStationDifference(global.s - local.s, 0.0));
    const double chord = (global.foot - local.foot).norm();
    if (adoptGlobalCandidate(local.D, global.D, arc, chord, options_.global_search_hysteresis)) {
      result = finish(x, global, context);
      result.status_bits |= Status::kAmbiguousClosest;
    }
  }
  return result;
}

ClosestPoint3 PolylineCurve3::finish(
  const Vec3 & x, const Candidate & c, const EvalContext3 & context) const noexcept
{
  ClosestPoint3 cp;
  if (!c.valid) {
    return cp;
  }
  const int segments = segmentCount();
  const auto n_points = static_cast<int>(points_.size());
  cp.valid = true;
  cp.x_star = c.foot;
  cp.D = c.D;
  cp.s_star = c.s;
  cp.segment = c.k;
  cp.piece = PieceKind::kSegment;
  Vec3 T = Vec3::UnitX();
  if (c.k == kBackward || c.k == kForward) {
    const bool forward = c.k == kForward;
    cp.piece = forward ? PieceKind::kForwardRay : PieceKind::kBackwardRay;
    cp.on_ray = c.tau != 0.0;
    cp.segment = forward ? n_points - 1 : -1;
    T = forward ? tangents_.back() : tangents_.front();
    if (!cp.on_ray) {
      cp.piece = PieceKind::kSegment;
      cp.segment = forward ? segments - 1 : 0;
    }
  } else {
    T = tangents_[static_cast<std::size_t>(c.k)];
    // A foot at a segment end that is an interior vertex: resolve the owner (math_spec §2.3).
    int v = -1;
    if (c.tau <= 0.0 && (closed_ || c.k > 0)) {
      v = c.k;
    } else if (
      c.tau >= lengths_[static_cast<std::size_t>(c.k)] && (closed_ || c.k < segments - 1)) {
      v = (c.k + 1) % n_points;
    }
    if (v >= 0) {
      const auto v_in = static_cast<std::size_t>((v - 1 + segments) % segments);
      const auto v_out = static_cast<std::size_t>(v % segments);
      const Vec3 & t_in = tangents_[v_in];
      const Vec3 & t_out = tangents_[v_out];
      const Vec3 y = x - points_[static_cast<std::size_t>(v)];
      const double a1 = y.dot(t_in);
      const double a2 = -y.dot(t_out);
      const bool hairpin = (t_in + t_out).norm() < kEpsHp;
      cp.s_star = stations_[static_cast<std::size_t>(v)];
      if (closed_ && v == 0) {
        cp.s_star = 0.0;
      }
      if (c.D < kEpsD) {
        // On the curve at the vertex point (math_spec §2.3).
        cp.piece = PieceKind::kVertex;
        cp.segment = v;
        cp.T = hairpin ? t_out : unitOr(t_in + t_out, t_out);
        cp.n = Vec3::Zero();
        cp.u_star = cp.s_star;
        return cp;
      }
      if (a1 > 0.0 && a2 > 0.0) {
        cp.piece = PieceKind::kVertex;
        cp.segment = v;
        cp.in_wedge = true;
        const Vec3 n = y / c.D;
        cp.n = n;
        Vec3 candidate = Vec3::Zero();
        if (!hairpin) {
          const double alpha = a1 / (a1 + a2);
          candidate = rejectFrom((1.0 - alpha) * t_in + alpha * t_out, n);
        } else {
          // Hairpin fallback chain (math_spec §14.2).
          cp.hairpin = true;
          cp.status_bits |= Status::kHairpin;
          candidate = rejectFrom(context.last_tangent, n);
          if (!(candidate.norm() >= kEpsHp)) {
            candidate = rejectFrom(t_in, n);
          }
          if (!(candidate.norm() >= kEpsHp)) {
            int axis = 0;
            n.cwiseAbs().minCoeff(&axis);
            candidate = rejectFrom(Vec3::Unit(axis), n);
          }
        }
        const Vec3 Tw = unitOr(candidate, t_out);
        // Re-orthogonalise once more against n (rounding only).
        cp.T = unitOr(rejectFrom(Tw, n), Tw);
        cp.u_star = cp.s_star;
        return cp;
      }
      // Boundary of the wedge: the adjacent segment with a1 <= 0 (incoming) or a2 <= 0 owns.
      if (a1 <= 0.0) {
        T = t_in;
        cp.segment = static_cast<int>(v_in);
        cp.s_star = stations_[v_in] + lengths_[v_in];
      } else {
        T = t_out;
        cp.segment = static_cast<int>(v_out);
        cp.s_star = stations_[v_out];
      }
    }
  }
  if (closed_ && cp.s_star >= stations_.back()) {
    cp.s_star -= stations_.back();
  }
  cp.u_star = cp.s_star;
  cp.T = T;
  if (c.D < kEpsD) {
    cp.n = Vec3::Zero();
  } else {
    cp.n = unitOr(rejectFrom(x - c.foot, T), Vec3::Zero());
  }
  return cp;
}

}  // namespace vectorfield_core::d3
