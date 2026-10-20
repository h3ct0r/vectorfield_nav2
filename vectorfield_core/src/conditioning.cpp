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

// Path conditioning (math_spec §2.8): follower polylines (dedupe, corner fillet, densify-only
// resampling) and the plugin window helpers (end tangent, tail conditioning, cusp decimation,
// one-pose rule, kappa_path).

#include "vectorfield_core/conditioning.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace vectorfield_core
{
namespace
{

/// Turning angles at or below this (rad) are collinear for the fillet: the corner's deviation
/// is negligible and filleting would only add vertices.
constexpr double kFilletMinTurn = 1e-9;

/// @brief Number of equal pieces for a segment of length `length` at maximum spacing `spacing`:
/// \f$\max(1, \lceil\ell/\mathrm{spacing}\rceil)\f$ (math_spec §2.8 step 3).
std::size_t piecesFor(double length, double spacing) noexcept
{
  const double pieces = std::ceil(length / spacing);
  if (!(pieces > 1.0)) {
    return 1;  // also NaN
  }
  return static_cast<std::size_t>(pieces);
}

/// @brief Appends `point` unless it is within kEpsD of the last appended point, so the fillet
/// never emits duplicates (adjacent tangent points meet at a segment midpoint when both corners
/// are bounded by \f$R_\ell\f$).
void appendDistinct(std::vector<Vec2> * out, const Vec2 & point)
{
  if (out->empty() || (point - out->back()).norm() >= kEpsD) {
    out->push_back(point);
  }
}

/// @brief Replaces the corner at `vertex` (between `previous` and `next`) by a circular arc
/// (math_spec §2.8 step 2), appending the arc points to `out`; appends the vertex itself when it
/// is collinear, an exact reversal or too small to fillet.
///
/// The radius is \f$R = \min(R_\delta, R_\ell)\f$ with \f$R_\delta = \delta/(1/\cos(\theta/2) -
/// 1)\f$ and \f$R_\ell = \min(\ell_{v-1}, \ell_v)/(2\tan(\theta/2))\f$; the tangent points are
/// \f$A = P_v - d\,t_{v-1}\f$ and \f$B = P_v + d\,t_v\f$ with \f$d = R\tan(\theta/2)\f$. Arc
/// points are written relative to A as \f$A + R\sin\varphi\,t_{v-1} + \sigma R(1 -
/// \cos\varphi)\,J t_{v-1}\f$, which is exact at \f$\varphi = \theta\f$ (it reaches B) and stable
/// for large R. The arc has an even number of pieces, at most `arc_spacing` long, so its
/// midpoint is a vertex and the vertex's distance to the filleted path is exactly
/// \f$R(1/\cos(\theta/2) - 1) \le \delta\f$.
void appendFillet(
  const Vec2 & previous, const Vec2 & vertex, const Vec2 & next, double deviation,
  double arc_spacing, std::vector<Vec2> * out)
{
  const Vec2 incoming = vertex - previous;
  const Vec2 outgoing = next - vertex;
  const double incoming_length = incoming.norm();
  const double outgoing_length = outgoing.norm();
  if (!(incoming_length > 0.0) || !(outgoing_length > 0.0)) {
    appendDistinct(out, vertex);
    return;
  }
  const Vec2 t1 = incoming / incoming_length;
  const Vec2 t2 = outgoing / outgoing_length;
  const double turn_sine = cross2(t1, t2);
  const double theta = std::atan2(std::abs(turn_sine), t1.dot(t2));
  if (!(theta > kFilletMinTurn) || (t1 + t2).norm() < kEpsHp) {
    appendDistinct(out, vertex);  // collinear, or an exact reversal (hairpin): kept as is
    return;
  }
  const double half = 0.5 * theta;
  const double tan_half = std::tan(half);
  const double sin_quarter = std::sin(0.5 * half);
  // 1/cos(θ/2) − 1 = 2 sin²(θ/4) / cos(θ/2), without cancellation at small θ.
  const double excess = 2.0 * sin_quarter * sin_quarter / std::cos(half);
  const double radius_deviation = deviation / excess;
  const double radius_length = std::min(incoming_length, outgoing_length) / (2.0 * tan_half);
  const double radius = std::min(radius_deviation, radius_length);
  const double tangent_distance = radius * tan_half;
  if (!(tangent_distance > kEpsD) || !std::isfinite(tangent_distance)) {
    appendDistinct(out, vertex);
    return;
  }
  const double side = turn_sine > 0.0 ? 1.0 : -1.0;
  const Vec2 normal = side * J(t1);
  const Vec2 start = vertex - tangent_distance * t1;
  const Vec2 end = vertex + tangent_distance * t2;

  std::size_t pieces = 2;
  if (arc_spacing > 0.0 && std::isfinite(arc_spacing)) {
    pieces = std::max<std::size_t>(2, piecesFor(radius * theta, arc_spacing));
  }
  if (pieces % 2 != 0) {
    ++pieces;
  }
  appendDistinct(out, start);
  for (std::size_t j = 1; j < pieces; ++j) {
    const double phi = theta * static_cast<double>(j) / static_cast<double>(pieces);
    const double sin_half_phi = std::sin(0.5 * phi);
    appendDistinct(
      out, start + (radius * std::sin(phi)) * t1 +
             (2.0 * radius * sin_half_phi * sin_half_phi) * normal);
  }
  appendDistinct(out, end);
}

}  // namespace

void dedupe(Span<const Vec2> in, bool closed, std::vector<Vec2> * out, double min_spacing) noexcept
{
  out->clear();
  const std::size_t count = in.size();
  if (count == 0) {
    return;
  }
  const double spacing = min_spacing > 0.0 ? min_spacing : 0.0;
  out->push_back(in[0]);
  for (std::size_t i = 1; i + 1 < count; ++i) {
    if ((in[i] - out->back()).norm() >= spacing) {
      out->push_back(in[i]);
    }
  }
  if (count >= 2) {
    const Vec2 & last = in[count - 1];
    const bool closing_duplicate = closed && (last - out->front()).norm() < spacing;
    if (!closing_duplicate) {
      if ((last - out->back()).norm() >= spacing) {
        out->push_back(last);
      } else if (out->size() > 1) {
        out->back() = last;  // the last vertex (the goal) replaces a close previous vertex
      } else if ((last - out->front()).norm() >= kEpsD) {
        out->push_back(last);  // both endpoints are kept, even closer than min_spacing
      }
    }
  }
  if (closed) {
    // Closing duplicates: the last vertex must be at least min_spacing from the first.
    while (out->size() > 1 && (out->back() - out->front()).norm() < spacing) {
      out->pop_back();
    }
  }
}

void filletCorners(
  Span<const Vec2> in, bool closed, double max_corner_deviation, double arc_spacing,
  std::vector<Vec2> * out)
{
  out->clear();
  const std::size_t count = in.size();
  if (!(max_corner_deviation > 0.0) || count < 3) {
    out->assign(in.begin(), in.end());
    return;
  }
  if (closed) {
    for (std::size_t v = 0; v < count; ++v) {
      appendFillet(
        in[v == 0 ? count - 1 : v - 1], in[v], in[v + 1 == count ? 0 : v + 1], max_corner_deviation,
        arc_spacing, out);
    }
    while (out->size() > 1 && (out->back() - out->front()).norm() < kEpsD) {
      out->pop_back();
    }
    return;
  }
  out->push_back(in[0]);  // endpoints are preserved bit for bit
  for (std::size_t v = 1; v + 1 < count; ++v) {
    appendFillet(in[v - 1], in[v], in[v + 1], max_corner_deviation, arc_spacing, out);
  }
  const Vec2 & last = in[count - 1];
  if (out->size() > 1 && (last - out->back()).norm() < kEpsD) {
    out->back() = last;
  } else {
    out->push_back(last);
  }
}

void resample(Span<const Vec2> in, bool closed, double spacing, std::vector<Vec2> * out)
{
  out->clear();
  const std::size_t count = in.size();
  if (count < 2 || !(spacing > 0.0) || !std::isfinite(spacing)) {
    out->assign(in.begin(), in.end());
    return;
  }
  const std::size_t segments = closed ? count : count - 1;
  std::size_t total = closed ? 0 : 1;
  for (std::size_t k = 0; k < segments; ++k) {
    total += piecesFor((in[k + 1 == count ? 0 : k + 1] - in[k]).norm(), spacing);
  }
  out->reserve(total);
  for (std::size_t k = 0; k < segments; ++k) {
    const Vec2 & a = in[k];
    const Vec2 & b = in[k + 1 == count ? 0 : k + 1];
    const Vec2 delta = b - a;
    const std::size_t pieces = piecesFor(delta.norm(), spacing);
    out->push_back(a);  // every input vertex is kept exactly
    for (std::size_t j = 1; j < pieces; ++j) {
      out->push_back(a + delta * (static_cast<double>(j) / static_cast<double>(pieces)));
    }
  }
  if (!closed) {
    out->push_back(in[count - 1]);
  }
}

EndTangent endTangent(Span<const Vec2> plan, double chord_length) noexcept
{
  EndTangent result;
  const std::size_t count = plan.size();
  if (count < 2) {
    return result;
  }
  const Vec2 & goal = plan[count - 1];
  for (std::size_t i = count - 1; i-- > 0;) {
    const Vec2 chord = goal - plan[i];
    const double chord_norm = chord.norm();
    if (chord_norm >= chord_length && chord_norm > 0.0) {
      result.tangent = chord / chord_norm;
      result.found_chord = true;
      result.valid = true;
      return result;
    }
  }
  const Vec2 chord = goal - plan[0];
  const double chord_norm = chord.norm();
  if (chord_norm >= kDedupeDistance) {
    result.tangent = chord / chord_norm;
    result.valid = true;
  }
  return result;
}

void tailCondition(
  Span<const Vec2> window, const Vec2 & end_tangent, double chord_length,
  std::vector<Vec2> * out) noexcept
{
  out->clear();
  const std::size_t count = window.size();
  if (count == 0) {
    return;
  }
  const Vec2 & back = window[count - 1];
  const Vec2 virtual_vertex = back - chord_length * end_tangent;
  // The trailing run of vertices closer than l_c to W_back ...
  std::size_t kept = count - 1;
  while (kept > 0 && (window[kept - 1] - back).norm() < chord_length) {
    --kept;
  }
  // ... and remaining last vertices within 1 mm of the virtual vertex, so a later dedupe keeps it.
  while (kept > 0 && (window[kept - 1] - virtual_vertex).norm() < kDedupeDistance) {
    --kept;
  }
  for (std::size_t i = 0; i < kept; ++i) {
    out->push_back(window[i]);
  }
  out->push_back(virtual_vertex);
  out->push_back(back);
}

void decimateChords(Span<const Vec2> plan, double chord_length, std::vector<Vec2> * out) noexcept
{
  out->clear();
  const std::size_t count = plan.size();
  if (count == 0) {
    return;
  }
  out->push_back(plan[0]);
  if (count == 1) {
    return;
  }
  for (std::size_t i = 1; i + 1 < count; ++i) {
    if ((plan[i] - out->back()).norm() >= chord_length) {
      out->push_back(plan[i]);
    }
  }
  // Pop before append: every chord ends up >= l_c except a final chord from the first pose.
  const Vec2 & last = plan[count - 1];
  while (out->size() > 1 && (out->back() - last).norm() < chord_length) {
    out->pop_back();
  }
  out->push_back(last);
}

bool hasCusp(Span<const Vec2> decimated) noexcept
{
  for (std::size_t i = 1; i + 1 < decimated.size(); ++i) {
    const Vec2 before = decimated[i] - decimated[i - 1];
    const Vec2 after = decimated[i + 1] - decimated[i];
    if (before.dot(after) < 0.0) {
      return true;
    }
  }
  return false;
}

std::array<Vec2, 2> onePoseWindow(const Vec2 & base, const Pose2 & last) noexcept
{
  if ((base - last.p).norm() < kDedupeDistance) {
    return {{Vec2(last.p - kOnePoseOffset * heading(last.yaw)), last.p}};
  }
  return {{base, last.p}};
}

double kappaPath(Span<const Vec2> vertices, bool closed) noexcept
{
  const std::size_t count = vertices.size();
  if (count < 3) {
    return 0.0;
  }
  double kappa = 0.0;
  const std::size_t first = closed ? 0 : 1;
  const std::size_t end = closed ? count : count - 1;
  for (std::size_t v = first; v < end; ++v) {
    const Vec2 incoming = vertices[v] - vertices[v == 0 ? count - 1 : v - 1];
    const Vec2 outgoing = vertices[v + 1 == count ? 0 : v + 1] - vertices[v];
    const double mean_length = 0.5 * (incoming.norm() + outgoing.norm());
    if (!(incoming.norm() > 0.0) || !(outgoing.norm() > 0.0) || !(mean_length > 0.0)) {
      continue;
    }
    // |wrap(angle(t_v) - angle(t_{v-1}))| in [0, pi].
    const double theta = std::abs(std::atan2(cross2(incoming, outgoing), incoming.dot(outgoing)));
    kappa = std::max(kappa, theta / mean_length);
  }
  return kappa;
}

bool segmentPlan(
  Span<const Pose2> plan, double chord_length, int default_sign, std::vector<std::size_t> * work,
  std::vector<PlanSegment> * out) noexcept
{
  out->clear();
  work->clear();
  const std::size_t count = plan.size();
  if (count == 0) {
    return false;
  }
  // Chord decimation by index: the rule of decimateChords().
  std::vector<std::size_t> & kept = *work;
  kept.push_back(0);
  for (std::size_t i = 1; i + 1 < count; ++i) {
    if ((plan[i].p - plan[kept.back()].p).norm() >= chord_length) {
      kept.push_back(i);
    }
  }
  if (count > 1) {
    while (kept.size() > 1 && (plan[kept.back()].p - plan[count - 1].p).norm() < chord_length) {
      kept.pop_back();
    }
    kept.push_back(count - 1);
  }

  // Cusps, each moved to the raw pose of largest projection on the incoming chord.
  PlanSegment segment;
  for (std::size_t j = 1; j + 1 < kept.size(); ++j) {
    const Vec2 & a = plan[kept[j - 1]].p;
    const Vec2 before = plan[kept[j]].p - a;
    const Vec2 after = plan[kept[j + 1]].p - plan[kept[j]].p;
    if (!(before.dot(after) < 0.0)) {
      continue;
    }
    const Vec2 u = before.normalized();
    std::size_t cusp = kept[j];
    double best = -kInf;
    for (std::size_t k = kept[j - 1]; k <= kept[j + 1]; ++k) {
      const double projection = (plan[k].p - a).dot(u);
      if (projection > best) {
        best = projection;
        cusp = k;
      }
    }
    if (cusp > segment.first && cusp + 1 < count) {
      segment.last = cusp;
      out->push_back(segment);
      segment.first = cusp;
    }
  }
  segment.last = count - 1;
  out->push_back(segment);

  bool headings = false;
  for (std::size_t i = 1; i + 1 < count; ++i) {
    if (plan[i].yaw != 0.0) {
      headings = true;
      break;
    }
  }
  int previous = 0;
  for (PlanSegment & s : *out) {
    double along = 0.0;
    double length = 0.0;
    for (std::size_t k = s.first; k < s.last; ++k) {
      const Vec2 step = plan[k + 1].p - plan[k].p;
      length += step.norm();
      along += step.dot(heading(plan[k].yaw));
    }
    s.length = length;
    int sign = 0;
    if (headings && length > 0.0 && std::abs(along) >= 0.5 * length) {
      sign = along < 0.0 ? -1 : 1;
    } else if (previous != 0) {
      sign = -previous;
    } else {
      sign = default_sign < 0 ? -1 : 1;
    }
    s.drive_sign = static_cast<std::int8_t>(sign);
    previous = sign;
  }
  return headings;
}

std::size_t firstActiveSegment(
  Span<const PlanSegment> segments, int active_sign, double hysteresis) noexcept
{
  if (
    active_sign != 0 && segments.size() > 1 && segments[0].length < hysteresis &&
    segments[0].drive_sign != active_sign && segments[1].drive_sign == active_sign) {
    return 1;
  }
  return 0;
}

bool hasInPlaceRotation(Span<const Pose2> plan) noexcept
{
  constexpr double kInPlaceDistance = 1.0e-4;
  constexpr double kInPlaceYaw = 1.0e-3;
  for (std::size_t i = 0; i + 2 < plan.size(); ++i) {
    if (
      (plan[i + 1].p - plan[i].p).norm() < kInPlaceDistance &&
      std::abs(wrapAngle(plan[i + 1].yaw - plan[i].yaw)) > kInPlaceYaw) {
      return true;
    }
  }
  return false;
}

}  // namespace vectorfield_core
