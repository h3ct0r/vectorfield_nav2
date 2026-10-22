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
/// @brief 3D core (experimental): moving curve frames, the SE(3) exponential and logarithm, and
/// FrameTwistEstimator3 (math_spec §14.7 [Design, 0.3.0]).
///
/// A curve may live in a frame \f$\mathcal F\f$ that moves rigidly in the evaluation frame:
/// \f$T_{\mathrm{eval}\leftarrow\mathcal F} = (R, c)\f$ with the body twist
/// \f$\xi = (v_{\mathcal F}, \omega_{\mathcal F})\f$ expressed in \f$\mathcal F\f$, so that
/// \f$\dot R = R[\omega_{\mathcal F}]_\times\f$ and \f$\dot c = R v_{\mathcal F}\f$. A point
/// \f$r_{\mathcal F}(s, t)\f$ of the curve then moves at
/// \f$\partial_t r = R\,(v_{\mathcal F} + \omega_{\mathcal F} \times r_{\mathcal F} +
/// \partial_t r_{\mathcal F})\f$.

#ifndef VECTORFIELD_CORE__D3__FRAME_HPP_
#define VECTORFIELD_CORE__D3__FRAME_HPP_

#include <cstdint>

#include <Eigen/Geometry>

#include "vectorfield_core/d3/types.hpp"

namespace vectorfield_core::d3
{

/// @brief A rigid transform of \f$\mathbb R^3\f$ (rotation and translation).
using Isometry3 = Eigen::Isometry3d;

/// @brief A body twist of a rigid frame, expressed in that frame.
struct FrameTwist3
{
  /// Linear velocity \f$v_{\mathcal F}\f$ (m/s), in \f$\mathcal F\f$'s axes.
  Vec3 v{Vec3::Zero()};
  /// Angular velocity \f$\omega_{\mathcal F}\f$ (rad/s), in \f$\mathcal F\f$'s axes.
  Vec3 w{Vec3::Zero()};
};

/// @brief The frame a curve lives in, at the time of one cycle (the 3D CurveFrame).
struct CurveFrame3
{
  /// Curve frame → evaluation frame, \f$T_{\mathrm{eval}\leftarrow\mathcal F} = (R, c)\f$. The
  /// linear part must be a rotation. Default: identity.
  Isometry3 T_eval_curve{Isometry3::Identity()};
  /// Body twist \f$(v_{\mathcal F}, \omega_{\mathcal F})\f$ in \f$\mathcal F\f$ (used only when
  /// `moving`).
  FrameTwist3 twist;
  /// The frame moves: the curve velocity includes the frame motion, open curves are transported
  /// with the frame, and extrapolation uses the twist.
  bool moving{false};
};

/// @brief True iff every component is finite.
inline bool isFinite(const FrameTwist3 & xi) noexcept { return isFinite(xi.v) && isFinite(xi.w); }

/// @brief True iff the transform is finite and its linear part a rotation to 1e-6.
bool isRigid(const Isometry3 & T) noexcept;

/// @brief True iff the transform is rigid (isRigid()) and, for a moving frame, the twist finite.
inline bool isFinite(const CurveFrame3 & frame) noexcept
{
  return isRigid(frame.T_eval_curve) && (!frame.moving || isFinite(frame.twist));
}

/// @brief The skew matrix \f$[w]_\times\f$ (\f$[w]_\times x = w \times x\f$).
Mat3 skew(const Vec3 & w) noexcept;

/// @brief \f$\exp(\tau\,\hat\xi)\f$: the displacement of a frame moving with the constant body
/// twist \f$\xi\f$ for \f$\tau\f$ seconds (negative \f$\tau\f$ runs backward). Exact (closed
/// form, with series below 1e-2 rad).
Isometry3 expSE3(const FrameTwist3 & xi, double tau) noexcept;

/// @brief \f$\log_{SE(3)}\f$: the twist \f$\xi\f$ with \f$\exp(\hat\xi) = T\f$ and
/// \f$\lVert\omega\rVert \le \pi\f$ (unit time). Uses a quaternion for the rotation angle, so it
/// is robust near \f$\pi\f$.
FrameTwist3 logSE3(const Isometry3 & T) noexcept;

/// @brief \f$\log_{SE(3)}(T_a^{-1} T_b)/\Delta t\f$: the constant body twist (in a's axes) that
/// moves `a` to `b` in `dt`. Exact for a constant twist with \f$\lVert\omega\rVert\Delta t <
/// \pi\f$.
/// @return Zero for a non-positive or non-finite `dt`.
FrameTwist3 bodyTwist(const Isometry3 & a, const Isometry3 & b, double dt) noexcept;

/// @brief The frame's transform \f$\tau\f$ seconds later under its constant twist:
/// \f$T\exp(\tau\hat\xi)\f$ for a moving frame, `T_eval_curve` otherwise.
Isometry3 extrapolate(const CurveFrame3 & frame, double tau) noexcept;

/// @brief Velocity of the frame-fixed point \f$x_{\mathcal F}\f$, in the evaluation frame:
/// \f$R\,(v_{\mathcal F} + \omega_{\mathcal F} \times x_{\mathcal F})\f$; zero for a static frame.
Vec3 frameVelocity(const CurveFrame3 & frame, const Vec3 & x_curve) noexcept;

/// @brief Settings of FrameTwistEstimator3 (the defaults of the planar FrameTwistConfig).
struct FrameTwistConfig3
{
  /// Minimum Δt of a differencing pair (s).
  double min_dt{0.01};
  /// Maximum Δt of a differencing pair (s); longer gaps restart the estimator.
  double max_dt{0.3};
  /// LPF cutoff (Hz) on \f$(v, \omega)\f$; <= 0 disables the filter.
  double cutoff_hz{2.0};
  /// Outlier limit of the linear speed \f$\lVert v\rVert\f$ (m/s).
  double max_speed{2.0};
  /// Outlier limit of the angular rate \f$\lVert\omega\rVert\f$ (rad/s).
  double max_angular_rate{2.0 * kPi};
  /// Extrapolation limit and staleness age (s).
  double input_timeout{0.3};
};

/// @brief What a sample did to FrameTwistEstimator3.
enum class FrameSampleResult3 : std::uint8_t
{
  /// A twist was computed (poses) or taken (twist samples) and the latest pose updated.
  kAccepted = 0,
  /// The first sample, or the first after a gap: pose only, no twist yet.
  kStarted = 1,
  /// Same stamp as the latest sample: ignored.
  kDuplicate = 2,
  /// Closer than `min_dt` to the differencing base: the latest pose only.
  kTooClose = 3,
  /// The twist exceeds the outlier limits: dropped.
  kOutlier = 4,
  /// Non-finite or non-rigid input, or a stamp older than the latest: dropped.
  kInvalid = 5,
};

/// @brief The frame at a query time.
struct FrameEstimate3
{
  /// A pose and a twist are known.
  bool ready{false};
  /// Not ready, or the latest sample is older than `input_timeout`: the pose is held and the
  /// twist is zero.
  bool stale{true};
  /// \f$T_{\mathrm{ref}\leftarrow\mathcal F}\f$ at the query time (extrapolated, or held).
  Isometry3 pose{Isometry3::Identity()};
  /// \f$(v_{\mathcal F}, \omega_{\mathcal F})\f$ in \f$\mathcal F\f$ (zero when stale).
  FrameTwist3 twist;
  /// Age of the latest sample at the query time (s); +Inf without samples.
  double age{kInf};

  /// @brief The CurveFrame3 for Tracker3: this pose and twist, moving iff not stale.
  CurveFrame3 curveFrame() const noexcept
  {
    CurveFrame3 frame;
    frame.T_eval_curve = pose;
    frame.twist = twist;
    frame.moving = !stale;
    return frame;
  }
};

/// @brief Pose and body twist of a moving frame from sampled poses (TF) or from twist samples
/// (Odometry); the 3D counterpart of the follower's FrameTwistEstimator.
///
/// Each pair of accepted pose samples gives \f$\xi = \log_{SE(3)}(T_{k-1}^{-1}T_k)/\Delta t\f$,
/// exact for a constant twist (also when the orientation crosses \f$\pm\pi\f$, because only
/// the relative rotation is logged). Duplicates are ignored, pairs closer than `min_dt` update
/// the latest pose only, a gap longer than `max_dt` restarts, a pair above `max_speed` or
/// `max_angular_rate` is dropped (the base is kept), and a first-order low-pass with cutoff
/// `cutoff_hz` filters the six twist components (never angles). estimate() extrapolates the
/// latest pose with the exact exponential for at most `input_timeout`; older input is stale.
/// Plain data: no allocation, not thread-safe.
class FrameTwistEstimator3
{
public:
  /// @brief An empty estimator.
  explicit FrameTwistEstimator3(const FrameTwistConfig3 & config = FrameTwistConfig3{}) noexcept;

  /// @brief New settings; keeps the samples and the filter state.
  void setConfig(const FrameTwistConfig3 & config) noexcept { config_ = config; }
  /// @brief The settings.
  const FrameTwistConfig3 & config() const noexcept { return config_; }
  /// @brief Forgets every sample and the filter state.
  void reset() noexcept;

  /// @brief Pose source: a sample of \f$T_{\mathrm{ref}\leftarrow\mathcal F}\f$ at `stamp` (s).
  FrameSampleResult3 addPose(double stamp, const Isometry3 & pose) noexcept;
  /// @brief Twist source (Odometry): a pose and the body twist in \f$\mathcal F\f$; the same
  /// outlier test, no filter.
  FrameSampleResult3 addTwist(
    double stamp, const Isometry3 & pose, const FrameTwist3 & twist) noexcept;

  /// @brief The frame at time `now`.
  FrameEstimate3 estimate(double now) const noexcept;

  /// @brief Number of samples dropped as outliers since the last reset().
  std::uint32_t outliers() const noexcept { return outliers_; }

private:
  bool isOutlier(const FrameTwist3 & twist) const noexcept;
  void filter(const FrameTwist3 & twist, double dt) noexcept;
  FrameSampleResult3 checkStamp(double stamp, const Isometry3 & pose) const noexcept;

  FrameTwistConfig3 config_;
  bool has_latest_{false};
  double latest_stamp_{0.0};
  Isometry3 latest_pose_{Isometry3::Identity()};
  bool has_base_{false};
  double base_stamp_{0.0};
  Isometry3 base_pose_{Isometry3::Identity()};
  bool has_twist_{false};
  FrameTwist3 twist_;
  std::uint32_t outliers_{0};
};

}  // namespace vectorfield_core::d3

#endif  // VECTORFIELD_CORE__D3__FRAME_HPP_
