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

// PolylineCurve: segment projection with semi-infinite end rays, ownership and wedge tangents,
// and the WindowScan and Hinted search policies (math_spec §2.2-§2.4).
//
// Structure of the search:
// - The borderless polyline is a sequence of *pieces* (math_spec §2.2 table). Open curves use
//   piece indices -1 (backward ray), 0..N-2 (segments) and N-1 (forward ray). Closed curves use
//   unwrapped indices: piece p is segment p mod N.
// - A search scans a contiguous *range* of pieces. forEachCandidate() lists its candidates: the
//   owners of x (segments and rays whose unclamped tau lies in their interval, and wedge vertices
//   where both adjacent pieces clamp), plus *edge* pseudo-candidates when the closest point of the
//   range is a clamped end of the range.
// - selectCandidate() applies the candidate rule of math_spec §2.4 in two passes: the minimum
//   distance, then the tie rule among every candidate within kTieTolerance of it.
// - searchPieces() extends the range while an edge wins, so a clamped edge point is never
//   returned; finalize*() build the ClosestPoint with the tangent rules of math_spec §2.3.

#include "vectorfield_core/polyline_curve.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace vectorfield_core
{
namespace
{

using Index = std::ptrdiff_t;

/// @brief Read-only view of an assigned polyline: vertices \f$P_k\f$, segment tangents
/// \f$t_k\f$, lengths \f$\ell_k\f$ and stations \f$S_k\f$ (math_spec §2.2).
///
/// The search helpers work on this view so that they need no access to PolylineCurve's private
/// members. `S` has segments() + 1 entries, so `S[segments()]` is the length L.
struct PolylineView
{
  const Vec2 * P{nullptr};
  const Vec2 * t{nullptr};
  const double * len{nullptr};
  const double * S{nullptr};
  Index n{0};
  bool closed{false};

  Index segments() const noexcept { return closed ? n : n - 1; }
  double length() const noexcept { return S[segments()]; }
};

PolylineView makeView(
  const std::vector<Vec2> & vertices, const std::vector<Vec2> & tangents,
  const std::vector<double> & lengths, const std::vector<double> & stations, bool closed) noexcept
{
  PolylineView view;
  view.P = vertices.data();
  view.t = tangents.data();
  view.len = lengths.data();
  view.S = stations.data();
  view.n = static_cast<Index>(vertices.size());
  view.closed = closed;
  return view;
}

/// @brief i modulo n in [0, n) for n > 0.
Index wrapIndex(Index i, Index n) noexcept
{
  const Index r = i % n;
  return r < 0 ? r + n : r;
}

/// @brief Station s wrapped into [0, L) for a closed curve of length L > 0.
double wrapStation(double s, double length) noexcept
{
  double r = std::fmod(s, length);
  if (r < 0.0) {
    r += length;
  }
  if (r >= length) {
    r -= length;
  }
  return r;
}

/// @brief One piece of the borderless polyline (math_spec §2.2 table): a segment, or one of the
/// two semi-infinite rays of an open polyline. Points are anchor + tau * tangent with tau in
/// [lo, hi]; a ray is clamped only at its anchor.
struct Piece
{
  Vec2 anchor{Vec2::Zero()};
  Vec2 tangent{Vec2::UnitX()};
  double lo{0.0};
  double hi{0.0};
  /// Station at tau = 0.
  double station{0.0};
  /// Vertex at tau = lo (-1 on the backward ray).
  Index start_vertex{-1};
  /// Vertex at tau = hi (-1 on the forward ray).
  Index end_vertex{-1};
  PieceKind kind{PieceKind::kSegment};
  /// ClosestPoint::segment of a foot on this piece.
  int segment{0};
};

/// @brief Piece p of the view (open: -1 backward ray, 0..N-2 segments, N-1 forward ray; closed:
/// segment p mod N).
Piece pieceAt(const PolylineView & view, Index p) noexcept
{
  Piece piece;
  if (view.closed) {
    const Index k = wrapIndex(p, view.n);
    piece.anchor = view.P[k];
    piece.tangent = view.t[k];
    piece.hi = view.len[k];
    piece.station = view.S[k];
    piece.start_vertex = k;
    piece.end_vertex = (k + 1 == view.n) ? 0 : k + 1;
    piece.segment = static_cast<int>(k);
    return piece;
  }
  const Index last = view.n - 1;
  if (p < 0) {
    piece.anchor = view.P[0];
    piece.tangent = view.t[0];
    piece.lo = -kInf;
    piece.hi = 0.0;
    piece.station = 0.0;
    piece.start_vertex = -1;
    piece.end_vertex = 0;
    piece.kind = PieceKind::kBackwardRay;
    piece.segment = -1;
  } else if (p >= last) {
    piece.anchor = view.P[last];
    piece.tangent = view.t[last - 1];
    piece.lo = 0.0;
    piece.hi = kInf;
    piece.station = view.S[last];
    piece.start_vertex = last;
    piece.end_vertex = -1;
    piece.kind = PieceKind::kForwardRay;
    piece.segment = static_cast<int>(last);
  } else {
    piece.anchor = view.P[p];
    piece.tangent = view.t[p];
    piece.hi = view.len[p];
    piece.station = view.S[p];
    piece.start_vertex = p;
    piece.end_vertex = p + 1;
    piece.segment = static_cast<int>(p);
  }
  return piece;
}

/// @brief A contiguous range of pieces [lo, hi]. `full` marks a closed range that covers every
/// segment: it is cyclic, so it has no edges.
struct PieceRange
{
  Index lo{0};
  Index hi{0};
  bool full{false};
};

PieceRange fullRange(const PolylineView & view) noexcept
{
  if (view.closed) {
    return PieceRange{0, view.n - 1, true};
  }
  return PieceRange{-1, view.n - 1, false};
}

bool canExtendForward(const PolylineView & view, const PieceRange & range) noexcept
{
  return view.closed ? !range.full : range.hi < view.n - 1;
}

bool canExtendBackward(const PolylineView & view, const PieceRange & range) noexcept
{
  return view.closed ? !range.full : range.lo > -1;
}

/// @brief Open curves: the first piece whose station interval ends at or after s.
Index firstPieceEndingAtOrAfterOpen(const PolylineView & view, double s) noexcept
{
  if (!(s > 0.0)) {
    return -1;  // the backward ray ends at 0 (NaN: widest choice)
  }
  const Index last = view.n - 1;
  if (s > view.S[last]) {
    return last;  // only the forward ray
  }
  // Smallest segment k in [0, N-2] with S[k+1] >= s.
  return static_cast<Index>(std::lower_bound(view.S + 1, view.S + view.n, s) - (view.S + 1));
}

/// @brief Open curves: the last piece whose station interval starts at or before s.
Index lastPieceStartingAtOrBeforeOpen(const PolylineView & view, double s) noexcept
{
  const Index last = view.n - 1;
  if (!(s < view.S[last])) {
    return last;  // the forward ray starts at S[N-1] (NaN: widest choice)
  }
  if (s < 0.0) {
    return -1;
  }
  // Largest segment k in [0, N-2] with S[k] <= s.
  return static_cast<Index>(std::upper_bound(view.S, view.S + last, s) - view.S) - 1;
}

/// @brief Closed curves: the first piece (unwrapped index) ending at or after s, for s in
/// [0, L).
Index firstPieceEndingAtOrAfterClosed(const PolylineView & view, double s) noexcept
{
  const Index k =
    static_cast<Index>(std::lower_bound(view.S + 1, view.S + view.n + 1, s) - (view.S + 1));
  return std::min(k, view.n - 1);
}

/// @brief Closed curves: the last piece (unwrapped index) starting at or before s, for s in
/// [0, 2L).
Index lastPieceStartingAtOrBeforeClosed(const PolylineView & view, double s) noexcept
{
  const double total = view.length();
  Index lap = 0;
  if (s >= total) {
    s -= total;
    lap = 1;
  }
  const Index k = static_cast<Index>(std::upper_bound(view.S, view.S + view.n, s) - view.S) - 1;
  return std::max<Index>(k, 0) + lap * view.n;
}

/// @brief The pieces whose station intervals meet [s_lo, s_hi] (math_spec §2.4), wrapping only
/// when closed. A closed window of at least one lap is the full cyclic range.
PieceRange stationRange(const PolylineView & view, double s_lo, double s_hi) noexcept
{
  if (view.closed) {
    const double total = view.length();
    const double width = s_hi - s_lo;
    if (!(width < total) || !std::isfinite(s_lo)) {
      return fullRange(view);
    }
    const double base = wrapStation(s_lo, total);
    const Index lo = firstPieceEndingAtOrAfterClosed(view, base);
    const Index hi =
      std::max(lo, lastPieceStartingAtOrBeforeClosed(view, base + std::max(0.0, width)));
    if (hi - lo + 1 >= view.n) {
      return fullRange(view);
    }
    return PieceRange{lo, hi, false};
  }
  const Index lo = firstPieceEndingAtOrAfterOpen(view, s_lo);
  const Index hi = std::max(lo, lastPieceStartingAtOrBeforeOpen(view, s_hi));
  return PieceRange{lo, hi, false};
}

/// @brief Kind of a search candidate.
enum class CandidateKind : std::uint8_t
{
  /// A segment or ray whose unclamped tau lies in its interval (math_spec §2.3 ownership).
  kOwner,
  /// A wedge vertex: both adjacent pieces clamp at it (a1 > 0 and a2 > 0).
  kVertex,
  /// The far end of the last piece of the range: the scan must extend forward.
  kEdgeForward,
  /// The near end of the first piece of the range: the scan must extend backward.
  kEdgeBackward,
};

/// @brief A candidate closest point of a range of pieces.
struct Candidate
{
  CandidateKind kind{CandidateKind::kOwner};
  /// Owning piece (kOwner), or the piece the vertex bounds.
  Index piece{0};
  /// Vertex index (every kind but kOwner).
  Index vertex{-1};
  /// Unclamped tau on the owning piece (kOwner).
  double tau{0.0};
  /// Distance from x: perpendicular for owners, Euclidean to the vertex otherwise.
  double D{kInf};
  /// Station of the foot (closed curves: in [0, L)).
  double s{0.0};
  /// Foot point.
  Vec2 foot{Vec2::Zero()};
  /// False when no candidate was found (non-finite geometry).
  bool found{false};
};

Candidate ownerCandidate(
  const PolylineView & view, const Vec2 & x, Index p, const Piece & piece, double tau) noexcept
{
  Candidate candidate;
  candidate.kind = CandidateKind::kOwner;
  candidate.piece = p;
  candidate.tau = tau;
  candidate.D = std::abs(cross2(piece.tangent, x - piece.anchor));
  candidate.s = piece.station + tau;
  if (view.closed && candidate.s >= view.length()) {
    candidate.s -= view.length();  // tau = l on the closing segment is P0, station 0
  }
  candidate.foot = piece.anchor + tau * piece.tangent;
  candidate.found = true;
  return candidate;
}

Candidate vertexCandidate(
  const PolylineView & view, const Vec2 & x, CandidateKind kind, Index p, Index v) noexcept
{
  Candidate candidate;
  candidate.kind = kind;
  candidate.piece = p;
  candidate.vertex = v;
  candidate.foot = view.P[v];
  candidate.D = (x - view.P[v]).norm();
  candidate.s = view.S[v];
  candidate.found = true;
  return candidate;
}

/// @brief Calls `visit(candidate)` for every candidate of a range (math_spec §2.3-§2.4).
///
/// For each piece, tau-hat = (x - A)^T t decides ownership:
/// - inside the closed interval: the piece owns x, except that a foot exactly at an extendable
///   end of the range is an edge;
/// - clamped at the far end: the end vertex is an edge if it closes the range, otherwise it is a
///   wedge candidate when the next piece clamps at its start (handled when that piece is seen);
/// - clamped at the near end: the start vertex is an edge if it opens the range, otherwise a
///   wedge iff the previous piece clamped at its end.
///
/// A minimiser of the distance over the union of the pieces is always among the candidates, so
/// the closest point lies on an owner unless it is an edge of the range.
template <typename Visitor>
void forEachCandidate(
  const PolylineView & view, const Vec2 & x, const PieceRange & range, Visitor && visit) noexcept
{
  const bool extend_forward = canExtendForward(view, range);
  const bool extend_backward = canExtendBackward(view, range);
  bool previous_clamped_at_end = false;
  if (range.full) {
    // Cyclic range: the vertex between the last and the first piece is interior.
    const Piece last = pieceAt(view, range.hi);
    previous_clamped_at_end = (x - last.anchor).dot(last.tangent) > last.hi;
  }
  for (Index p = range.lo; p <= range.hi; ++p) {
    const Piece piece = pieceAt(view, p);
    const double tau = (x - piece.anchor).dot(piece.tangent);
    const bool first = p == range.lo;
    const bool last = p == range.hi;
    if (tau < piece.lo) {
      if (first && extend_backward) {
        visit(vertexCandidate(view, x, CandidateKind::kEdgeBackward, p, piece.start_vertex));
      } else if (previous_clamped_at_end) {
        visit(vertexCandidate(view, x, CandidateKind::kVertex, p, piece.start_vertex));
      }
      previous_clamped_at_end = false;
    } else if (tau > piece.hi) {
      if (last && extend_forward) {
        visit(vertexCandidate(view, x, CandidateKind::kEdgeForward, p, piece.end_vertex));
      }
      previous_clamped_at_end = true;
    } else {
      if (last && extend_forward && tau >= piece.hi) {
        visit(vertexCandidate(view, x, CandidateKind::kEdgeForward, p, piece.end_vertex));
      } else if (first && extend_backward && tau <= piece.lo) {
        visit(vertexCandidate(view, x, CandidateKind::kEdgeBackward, p, piece.start_vertex));
      } else {
        visit(ownerCandidate(view, x, p, piece, tau));
      }
      previous_clamped_at_end = false;
    }
  }
}

/// @brief Tie rule of math_spec §2.4: among candidates within kTieTolerance of the minimum, the
/// smallest key wins, then the smallest dir·s.
struct TieRule
{
  enum class Mode : std::uint8_t
  {
    /// No hint: key = dir·s (the start of traversal wins).
    kDirection,
    /// WindowScan with a hint: key = distance from the foot to the hint position.
    kHintPosition,
    /// Hinted with a hint: key = |wrapped s - s_prev|.
    kHintStation,
  };

  Mode mode{Mode::kDirection};
  double direction{1.0};
  Vec2 hint_position{Vec2::Zero()};
  double hint_station{0.0};
  /// L for closed curves (station differences wrap), 0 for open curves.
  double period{0.0};

  double key(const Candidate & candidate) const noexcept
  {
    switch (mode) {
      case Mode::kHintPosition:
        return (candidate.foot - hint_position).norm();
      case Mode::kHintStation: {
        const double difference = candidate.s - hint_station;
        return std::abs(period > 0.0 ? wrapPeriodic(difference, period) : difference);
      }
      case Mode::kDirection:
        break;
    }
    return direction * candidate.s;
  }
};

/// @brief The candidate rule of math_spec §2.4 over one range, in two passes: the minimum
/// distance D_min, then the best key among the candidates with D <= D_min + kTieTolerance.
Candidate selectCandidate(
  const PolylineView & view, const Vec2 & x, const PieceRange & range,
  const TieRule & rule) noexcept
{
  double minimum = kInf;
  forEachCandidate(view, x, range, [&minimum](const Candidate & candidate) {
    minimum = std::min(minimum, candidate.D);
  });
  const double limit = minimum + kTieTolerance;
  Candidate best;
  double best_key = 0.0;
  double best_secondary = 0.0;
  forEachCandidate(view, x, range, [&](const Candidate & candidate) {
    if (!(candidate.D <= limit)) {
      return;
    }
    const double key = rule.key(candidate);
    const double secondary = rule.direction * candidate.s;
    if (!best.found || key < best_key || (key == best_key && secondary < best_secondary)) {
      best = candidate;
      best_key = key;
      best_secondary = secondary;
    }
  });
  return best;
}

/// @brief Adds pieces after the range while each new piece's closest point is its far end (the
/// distance strictly decreases along the walk); stops at the first piece whose foot lies before
/// its far end, at the forward ray, or when a closed range becomes full.
void walkForward(const PolylineView & view, const Vec2 & x, PieceRange * range) noexcept
{
  for (;;) {
    ++range->hi;
    if (view.closed && range->hi - range->lo + 1 >= view.n) {
      range->full = true;
      return;
    }
    const Piece piece = pieceAt(view, range->hi);
    if ((x - piece.anchor).dot(piece.tangent) < piece.hi || !canExtendForward(view, *range)) {
      return;
    }
  }
}

/// @brief Mirror of walkForward(): adds pieces before the range.
void walkBackward(const PolylineView & view, const Vec2 & x, PieceRange * range) noexcept
{
  for (;;) {
    --range->lo;
    if (view.closed && range->hi - range->lo + 1 >= view.n) {
      range->full = true;
      return;
    }
    const Piece piece = pieceAt(view, range->lo);
    if ((x - piece.anchor).dot(piece.tangent) > piece.lo || !canExtendBackward(view, *range)) {
      return;
    }
  }
}

/// @brief Exact search over a range extended at its edges (math_spec §2.4): while the winner is a
/// clamped edge point, the range grows in that direction and the selection repeats, so the
/// result is always an owner. Each extension grows the range, so the loop ends.
Candidate searchPieces(
  const PolylineView & view, const Vec2 & x, PieceRange range, const TieRule & rule) noexcept
{
  for (;;) {
    const Candidate best = selectCandidate(view, x, range, rule);
    if (best.found && best.kind == CandidateKind::kEdgeForward) {
      walkForward(view, x, &range);
    } else if (best.found && best.kind == CandidateKind::kEdgeBackward) {
      walkBackward(view, x, &range);
    } else {
      return best;
    }
  }
}

/// @brief Incoming and outgoing tangents of vertex v; the ends of an open polyline are joints
/// with their rays, so both tangents are the end segment's.
void vertexTangents(const PolylineView & view, Index v, Vec2 * t_in, Vec2 * t_out) noexcept
{
  if (view.closed) {
    *t_in = view.t[v == 0 ? view.n - 1 : v - 1];
    *t_out = view.t[v];
  } else if (v <= 0) {
    *t_in = view.t[0];
    *t_out = view.t[0];
  } else if (v >= view.n - 1) {
    *t_in = view.t[view.n - 2];
    *t_out = view.t[view.n - 2];
  } else {
    *t_in = view.t[v - 1];
    *t_out = view.t[v];
  }
}

/// @brief On-curve tangent at vertex v (math_spec §2.3): unit(t_in + t_out), t_out at a
/// hairpin, the end segment's tangent at an open end.
Vec2 vertexTangent(const PolylineView & view, Index v) noexcept
{
  Vec2 t_in;
  Vec2 t_out;
  vertexTangents(view, v, &t_in, &t_out);
  if (!view.closed && (v <= 0 || v >= view.n - 1)) {
    return t_out;
  }
  const Vec2 sum = t_in + t_out;
  const double norm = sum.norm();
  if (norm < kEpsHp) {
    return t_out;
  }
  return sum / norm;
}

/// @brief x on the curve at vertex v: n = 0, T = vertexTangent(), foot P_v.
ClosestPoint onVertexResult(const PolylineView & view, const Vec2 & x, Index v) noexcept
{
  ClosestPoint result;
  result.valid = true;
  result.x_star = view.P[v];
  result.D = (x - view.P[v]).norm();
  result.n = Vec2::Zero();
  result.T = vertexTangent(view, v);
  result.s_star = view.S[v];
  result.u_star = result.s_star;
  result.segment = static_cast<int>(v);
  result.piece = PieceKind::kVertex;
  return result;
}

/// @brief Vertex v owns x (math_spec §2.3): the wedge tangent T = s_w J n with
/// s_w = sgn((Jn)^T (t_in + t_out)), or the hairpin rule when ||t_in + t_out|| < eps_hp.
///
/// In a strict, non-hairpin wedge with turning angle theta, |(Jn)^T (t_in + t_out)| >=
/// 1 + cos(theta) = ||t_in + t_out||^2 / 2 >= 5e-7, so its sign is robust even at shallow turns;
/// T = ±Jn is orthogonal to n exactly. For a hairpin, unit((I - nn^T) T_last) is
/// sgn((Jn)^T T_last) Jn in 2D, and +Jn when |(Jn)^T T_last| < eps_hp.
ClosestPoint wedgeResult(
  const PolylineView & view, const Vec2 & x, Index v, const Vec2 & last_tangent) noexcept
{
  const Vec2 offset = x - view.P[v];
  const double distance = offset.norm();
  if (distance < kEpsD) {
    return onVertexResult(view, x, v);
  }
  ClosestPoint result;
  result.valid = true;
  result.x_star = view.P[v];
  result.D = distance;
  result.n = offset / distance;
  const Vec2 jn = J(result.n);
  Vec2 t_in;
  Vec2 t_out;
  vertexTangents(view, v, &t_in, &t_out);
  const Vec2 sum = t_in + t_out;
  if (sum.norm() < kEpsHp) {
    const double c = jn.dot(last_tangent);
    result.T = (std::abs(c) >= kEpsHp && c < 0.0) ? Vec2(-jn) : jn;
    result.hairpin = true;
    result.status_bits |= Status::kHairpin;
  } else {
    result.T = jn.dot(sum) >= 0.0 ? jn : Vec2(-jn);
  }
  result.s_star = view.S[v];
  result.u_star = result.s_star;
  result.segment = static_cast<int>(v);
  result.piece = PieceKind::kVertex;
  result.in_wedge = true;
  return result;
}

/// @brief Piece `candidate.piece` owns x: T = t, n = sgn(perp) J t with D = |perp|, so
/// n^T T = 0 exactly. On the curve (D < eps_D), n = 0 and a foot at a vertex becomes the
/// vertex point of math_spec §2.3.
ClosestPoint ownerResult(
  const PolylineView & view, const Vec2 & x, const Candidate & candidate) noexcept
{
  const Piece piece = pieceAt(view, candidate.piece);
  const double perp = cross2(piece.tangent, x - piece.anchor);
  const double distance = std::abs(perp);
  if (distance < kEpsD) {
    // x at a bounding vertex of the owning piece: the on-curve vertex rule.
    Index nearest = -1;
    double nearest_distance = kEpsD;
    for (const Index v : {piece.start_vertex, piece.end_vertex}) {
      if (v >= 0) {
        const double d = (x - view.P[v]).norm();
        if (d < nearest_distance) {
          nearest = v;
          nearest_distance = d;
        }
      }
    }
    if (nearest >= 0) {
      return onVertexResult(view, x, nearest);
    }
  }
  ClosestPoint result;
  result.valid = true;
  result.x_star = candidate.foot;
  result.D = distance;
  result.T = piece.tangent;
  if (distance < kEpsD) {
    result.n = Vec2::Zero();
  } else {
    result.n = perp > 0.0 ? J(piece.tangent) : Vec2(-J(piece.tangent));
  }
  result.s_star = candidate.s;
  result.u_star = candidate.s;
  result.segment = piece.segment;
  result.piece = piece.kind;
  result.on_ray = piece.kind == PieceKind::kBackwardRay || piece.kind == PieceKind::kForwardRay;
  return result;
}

ClosestPoint finalizeCandidate(
  const PolylineView & view, const Vec2 & x, const Candidate & candidate,
  const EvalContext & context) noexcept
{
  if (!candidate.found || !std::isfinite(candidate.D)) {
    return ClosestPoint{};
  }
  if (candidate.kind == CandidateKind::kVertex) {
    return wedgeResult(view, x, candidate.vertex, context.last_tangent);
  }
  return ownerResult(view, x, candidate);
}

/// @brief Segment k whose half-open station interval [S_k, S_{k+1}) contains s, for s in
/// [0, L) (clamped to a valid segment).
Index segmentAt(const PolylineView & view, double s) noexcept
{
  const Index segments = view.segments();
  const Index k = static_cast<Index>(std::upper_bound(view.S, view.S + segments, s) - view.S) - 1;
  return std::min(std::max<Index>(k, 0), segments - 1);
}

TieRule directionRule(const PolylineView & view, const EvalContext & context) noexcept
{
  TieRule rule;
  rule.direction = context.direction < 0 ? -1.0 : 1.0;
  rule.period = view.closed ? view.length() : 0.0;
  return rule;
}

}  // namespace

PolylineCurve::PolylineCurve() noexcept : policy_(SearchPolicy::hinted(CurveOptions{})) {}

PolylineCurve::PolylineCurve(const SearchPolicy & policy) noexcept : policy_(policy) {}

AssignStatus PolylineCurve::assign(Span<const Vec2> points, bool closed) noexcept
{
  for (const Vec2 & point : points) {
    if (!isFinite(point)) {
      clear();
      return AssignStatus::kNonFinite;
    }
  }
  // Compact into vertices_. Writes never overtake reads, so `points` may even view vertices_.
  const std::size_t count = points.size();
  if (count > vertices_.size()) {
    vertices_.resize(count);
  }
  std::size_t kept = 0;
  for (std::size_t i = 0; i < count; ++i) {
    const Vec2 point = points[i];
    if (kept > 0 && (point - vertices_[kept - 1]).norm() < kEpsD) {
      continue;
    }
    vertices_[kept++] = point;
  }
  if (closed) {
    while (kept > 1 && (vertices_[kept - 1] - vertices_[0]).norm() < kEpsD) {
      --kept;
    }
  }
  vertices_.resize(kept);
  if (kept < (closed ? std::size_t{3} : std::size_t{2})) {
    clear();
    return AssignStatus::kTooFew;
  }
  const std::size_t segments = closed ? kept : kept - 1;
  tangents_.resize(segments);
  lengths_.resize(segments);
  stations_.resize(segments + 1);
  stations_[0] = 0.0;
  for (std::size_t k = 0; k < segments; ++k) {
    const Vec2 delta = vertices_[k + 1 == kept ? 0 : k + 1] - vertices_[k];
    const double segment_length = delta.norm();
    if (!std::isfinite(segment_length)) {
      clear();
      return AssignStatus::kNonFinite;
    }
    lengths_[k] = segment_length;
    tangents_[k] = delta / segment_length;
    stations_[k + 1] = stations_[k] + segment_length;
  }
  if (!std::isfinite(stations_[segments])) {
    clear();
    return AssignStatus::kNonFinite;
  }
  closed_ = closed;
  return AssignStatus::kOk;
}

void PolylineCurve::reserve(std::size_t vertex_count)
{
  vertices_.reserve(vertex_count);
  tangents_.reserve(vertex_count);
  lengths_.reserve(vertex_count);
  stations_.reserve(vertex_count + 1);
}

void PolylineCurve::clear() noexcept
{
  vertices_.clear();
  tangents_.clear();
  lengths_.clear();
  stations_.clear();
  closed_ = false;
}

void PolylineCurve::setSearchPolicy(const SearchPolicy & policy) noexcept { policy_ = policy; }

const SearchPolicy & PolylineCurve::searchPolicy() const noexcept { return policy_; }

void PolylineCurve::setSpec(std::shared_ptr<const CurveSpec> spec) noexcept
{
  spec_ = std::move(spec);
}

std::size_t PolylineCurve::size() const noexcept { return vertices_.size(); }

bool PolylineCurve::empty() const noexcept { return vertices_.empty(); }

std::size_t PolylineCurve::segmentCount() const noexcept { return tangents_.size(); }

Span<const Vec2> PolylineCurve::vertices() const noexcept
{
  return Span<const Vec2>(vertices_.data(), vertices_.size());
}

double PolylineCurve::station(std::size_t k) const noexcept { return stations_[k]; }

Vec2 PolylineCurve::segmentTangent(std::size_t k) const noexcept { return tangents_[k]; }

double PolylineCurve::segmentLength(std::size_t k) const noexcept { return lengths_[k]; }

ClosestPoint PolylineCurve::closestPoint(
  const Vec2 & x, const EvalContext & context, const CurveHint & hint) const noexcept
{
  if (empty() || !isFinite(x)) {
    return ClosestPoint{};
  }
  const PolylineView view = makeView(vertices_, tangents_, lengths_, stations_, closed_);
  TieRule rule = directionRule(view, context);

  if (policy_.kind == SearchPolicy::Kind::kWindowScan) {
    // Plugin: exact over the window, always including the backward ray; position-only hint.
    if (hint.valid && isFinite(hint.x_star)) {
      rule.mode = TieRule::Mode::kHintPosition;
      rule.hint_position = hint.x_star;
    }
    const PieceRange range =
      view.closed ? stationRange(view, policy_.s_min, policy_.s_max)
                  : PieceRange{-1, lastPieceStartingAtOrBeforeOpen(view, policy_.s_max), false};
    return finalizeCandidate(view, x, searchPieces(view, x, range, rule), context);
  }

  // Hinted (follower). Initialisation: the global stage, adopted unconditionally.
  if (!hint.valid || !std::isfinite(hint.s)) {
    return finalizeCandidate(view, x, searchPieces(view, x, fullRange(view), rule), context);
  }
  const double s_prev = view.closed ? wrapStation(hint.s, view.length()) : hint.s;
  rule.mode = TieRule::Mode::kHintStation;
  rule.hint_station = s_prev;
  const double half_window = 0.5 * policy_.window;
  const PieceRange local_range = stationRange(view, s_prev - half_window, s_prev + half_window);
  const ClosestPoint local =
    finalizeCandidate(view, x, searchPieces(view, x, local_range, rule), context);
  if (!local.valid || !(local.D > policy_.global_trigger)) {
    return local;
  }
  // Global stage (D_loc > D_trig): an exact scan of every piece, adopted only with hysteresis
  // and the detour test (math_spec §2.4).
  ClosestPoint global =
    finalizeCandidate(view, x, searchPieces(view, x, fullRange(view), rule), context);
  if (!global.valid) {
    return local;
  }
  const double difference = global.s_star - local.s_star;
  const double arc = std::abs(view.closed ? wrapPeriodic(difference, view.length()) : difference);
  const double chord = (global.x_star - local.x_star).norm();
  if (adoptGlobalCandidate(local.D, global.D, arc, chord, policy_.global_hysteresis)) {
    global.status_bits |= Status::kAmbiguousClosest;
    return global;
  }
  return local;
}

bool PolylineCurve::isClosed() const noexcept { return closed_; }

bool PolylineCurve::isTimeVarying() const noexcept { return false; }

double PolylineCurve::length(double /*t*/) const noexcept
{
  return stations_.empty() ? 0.0 : stations_.back();
}

Vec2 PolylineCurve::velocity(const ClosestPoint & /*cp*/, double /*t*/) const noexcept
{
  return Vec2::Zero();
}

Vec2 PolylineCurve::pointAt(double s, double /*t*/) const noexcept
{
  if (empty() || !std::isfinite(s)) {
    return Vec2(kNaN, kNaN);
  }
  const PolylineView view = makeView(vertices_, tangents_, lengths_, stations_, closed_);
  const double total = view.length();
  if (view.closed) {
    s = wrapStation(s, total);
  } else if (s <= 0.0) {
    return view.P[0] + s * view.t[0];  // backward ray (exactly P0 at s = 0)
  } else if (s >= total) {
    return view.P[view.n - 1] + (s - total) * view.t[view.n - 2];  // forward ray
  }
  // Interpolate from the nearer vertex, so stations near a vertex reproduce it closely.
  const Index k = segmentAt(view, s);
  const double tau = s - view.S[k];
  if (tau <= 0.5 * view.len[k]) {
    return view.P[k] + tau * view.t[k];
  }
  const Index next = (k + 1 == view.n) ? 0 : k + 1;
  return view.P[next] - (view.S[k + 1] - s) * view.t[k];
}

Vec2 PolylineCurve::tangentAt(double s, double /*t*/) const noexcept
{
  if (empty() || !std::isfinite(s)) {
    return Vec2(kNaN, kNaN);
  }
  const PolylineView view = makeView(vertices_, tangents_, lengths_, stations_, closed_);
  const double total = view.length();
  if (view.closed) {
    s = wrapStation(s, total);
  } else if (s < 0.0) {
    return view.t[0];
  } else if (s >= total) {
    return view.t[view.n - 2];
  }
  return view.t[segmentAt(view, s)];
}

std::size_t PolylineCurve::sampleStations(
  double s0, double s1, double ds, double t, Span<Vec2> out) const noexcept
{
  return Curve::sampleStations(s0, s1, ds, t, out);
}

const CurveSpec * PolylineCurve::spec() const noexcept { return spec_.get(); }

}  // namespace vectorfield_core
