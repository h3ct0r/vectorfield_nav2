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
/// @brief 3D core (experimental): the distance field \f$\Phi = \eta\Phi_S + \Phi_T\f$ and the
/// terminal field in \f$\mathbb R^3\f$ (math_spec §4, §5.1, §14.3).

#ifndef VECTORFIELD_CORE__D3__FIELD_HPP_
#define VECTORFIELD_CORE__D3__FIELD_HPP_

#include <cstddef>
#include <cstdint>

#include "vectorfield_core/d3/curve.hpp"
#include "vectorfield_core/d3/types.hpp"
#include "vectorfield_core/field.hpp"
#include "vectorfield_core/function_ref.hpp"
#include "vectorfield_core/gains.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/status.hpp"

namespace vectorfield_core::d3
{

/// @brief One evaluation of the 3D field and its intermediate quantities.
struct FieldSample3
{
  /// Evaluation point x.
  Vec3 x_eval{Vec3::Zero()};
  /// Foot \f$x^\star\f$.
  Vec3 x_star{Vec3::Zero()};
  /// Unit tangent (parameter direction).
  Vec3 T{Vec3::UnitX()};
  /// \f$T_d = \mathrm{dir}\,T\f$.
  Vec3 T_d{Vec3::UnitX()};
  /// Unit normal, zero on the curve.
  Vec3 n{Vec3::Zero()};
  /// Distance D.
  double D{0.0};
  /// Station of the foot.
  double s_star{0.0};
  /// Terminal \f$\sigma\f$ (NaN outside the terminal field).
  double sigma{kNaN};
  /// G(D).
  double G{0.0};
  /// H(D).
  double H{1.0};
  /// \f$\eta\f$ (\f$v_r\f$ for static and terminal fields).
  double eta{0.0};
  /// Terminal speed profile \f$\varphi(\sigma)\f$ (1 outside the terminal field).
  double phi{1.0};
  /// \f$\Phi_S = -Gn + H\varphi T_d\f$.
  Vec3 Phi_S{Vec3::Zero()};
  /// \f$\Phi_T = \Pi_T v_c\f$ after the Assumption-3 clamp; for an open curve in a moving frame
  /// the clamped transport velocity \f$v_{\mathrm{tr}}\f$ (math_spec §14.7).
  Vec3 Phi_T{Vec3::Zero()};
  /// Curve velocity \f$v_c\f$ at the foot (frame motion included); for an open curve in a moving
  /// frame the transport velocity \f$v_{\mathrm{tr}}\f$ at x before the clamp.
  Vec3 v_c{Vec3::Zero()};
  /// \f$\Phi = \eta\Phi_S + \Phi_T\f$.
  Vec3 Phi{Vec3::Zero()};
  /// The terminal field was evaluated.
  bool terminal{false};
  /// Status bits (kNonFinite, kAssumption3Clamped, and the search's bits).
  std::uint32_t status{Status::kNone};
  /// False for an invalid closest point or a non-finite result (then Phi = 0).
  bool valid{false};
};

/// @brief \f$\Pi_T v_c = v_c - (T^\top v_c)T\f$ (math_spec §4.2).
inline Vec3 feedforward(const Vec3 & T, const Vec3 & v_c) noexcept { return v_c - T.dot(v_c) * T; }

/// @brief Result of clampFeedforward().
struct ClampedFeedforward3
{
  /// The clamped feedforward.
  Vec3 Phi_T{Vec3::Zero()};
  /// The Assumption-3 clamp acted.
  bool clamped{false};
};

/// @brief Assumption-3 clamp: scale \f$\Phi_T\f$ to norm \f$(1-\varepsilon_v)v_r\f$ if larger.
ClampedFeedforward3 clampFeedforward(
  const Vec3 & Phi_T, double v_r, double assumption3_margin) noexcept;

/// @brief The larger root \f$\eta\f$ of \f$\lVert\eta\Phi_S + \Phi_T\rVert = \mathrm{speed}\f$
/// for a unit \f$\Phi_S\f$ (math_spec §4.3); NaN when \f$\lVert\Phi_T\rVert \ge\f$ speed.
double solveEta(const Vec3 & Phi_S, const Vec3 & Phi_T, double speed) noexcept;

/// @brief The 3D distance field and terminal field. Stateless, const, `noexcept`.
class DistanceField3
{
public:
  /// @param gain G-type function (shared with the planar core).
  /// @param v_r Reference speed \f$v_r > 0\f$.
  /// @param direction Traversal direction ±1.
  /// @param assumption3_margin \f$\varepsilon_v \in (0, 1)\f$.
  DistanceField3(
    const GainFunction & gain, double v_r, int direction, double assumption3_margin) noexcept;

  /// @brief \f$\Phi = \eta\Phi_S + \Phi_T\f$ at x for a closest point `cp` and curve velocity
  /// `v_c` (zero for static curves).
  void evaluate(const Vec3 & x, const ClosestPoint3 & cp, const Vec3 & v_c, FieldSample3 * sample)
    const noexcept;

  /// @brief Terminal field \f$v_r(-Gn + H\varphi(\sigma)T_d)\f$ of a static open curve.
  void evaluateTerminal(
    const Vec3 & x, const ClosestPoint3 & cp, double sigma, const TerminalProfile & profile,
    FieldSample3 * sample) const noexcept;

  /// @brief Terminal field of an open curve that is static in a moving frame (math_spec §14.7
  /// [Design, 0.3.0]): \f$\Phi = v_r(-Gn + H\varphi(\sigma)T_d) + \Phi_T\f$ with
  /// \f$\Phi_T\f$ the transport velocity `v_transport` of the frame at x after the
  /// Assumption-3 clamp. Relative to the frame the body then follows the static terminal field.
  void evaluateTerminal(
    const Vec3 & x, const ClosestPoint3 & cp, double sigma, const TerminalProfile & profile,
    const Vec3 & v_transport, FieldSample3 * sample) const noexcept;

  /// @brief Convenience: closest point (no hint, global stage) and evaluate() on `curve` at
  /// time t. For plots and tests; the Tracker keeps hints.
  Vec3 at(const Curve3 & curve, const Vec3 & x, double t) const noexcept;

  /// @brief The gain function.
  const GainFunction & gain() const noexcept { return gain_; }
  /// @brief \f$v_r\f$.
  double referenceSpeed() const noexcept { return v_r_; }
  /// @brief dir.
  int direction() const noexcept { return direction_; }
  /// @brief \f$\varepsilon_v\f$.
  double assumption3Margin() const noexcept { return eps_v_; }

private:
  GainFunction gain_;
  double v_r_;
  int direction_;
  double eps_v_;
};

/// @brief Integrates a streamline of `field` from `start` with fixed-step RK4 (step in metres
/// along the normalised field), writing at most `out.size()` points; stops at a non-finite or
/// zero field value. Allocation-free.
/// @return The number of points written (the start included).
std::size_t streamline(
  FunctionRef<Vec3(const Vec3 &)> field, const Vec3 & start, double step, Span<Vec3> out) noexcept;

}  // namespace vectorfield_core::d3

#endif  // VECTORFIELD_CORE__D3__FIELD_HPP_
