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
/// @brief FrameTwistEstimator: pose and body twist of a moving curve frame from TF samples or
/// from Odometry. Pure: no node, no clock, no I/O.
///
/// **TF source.** Each pair of accepted samples \f$(t_{k-1}, T_{k-1})\f$, \f$(t_k, T_k)\f$ of
/// \f$T_{\mathrm{ref}\leftarrow\mathcal F}\f$ gives the body twist
/// \f$(v_{\mathcal F}, \omega_{\mathcal F}) = \log_{SE(2)}(T_{k-1}^{-1} T_k)/\Delta t\f$ with
/// \f$\Delta\theta = \mathrm{wrap}(\theta_k - \theta_{k-1})\f$, which is exact for a constant
/// twist (also across \f$\pm\pi\f$). Then:
/// - samples with a stamp equal to the last one are duplicates and change nothing;
/// - **Δt window:** a sample closer than `min_dt` to the differencing base updates only the
///   latest pose; a gap longer than `max_dt` restarts the estimator (no twist across a gap);
/// - **outliers:** a pair whose speed exceeds `max_speed` (or yaw rate `max_angular_rate`) is
///   dropped and the base kept;
/// - **LPF:** a first-order low-pass with cutoff `cutoff_hz` filters \f$(v_x, v_y, \omega)\f$
///   (velocities, never angles); the first accepted pair initialises it.
///
/// **Odometry source.** The twist is the Odometry's (child frame = \f$\mathcal F\f$), with the
/// same outlier test and no filter.
///
/// **Extrapolation.** estimate(t) integrates the twist exactly from the latest sample over
/// \f$\tau = t - t_{\mathrm{latest}} \le\f$ `input_timeout`. Beyond it the input is stale: the
/// latest pose is held (no extrapolation) and the twist is zero, so \f$\Phi_T = 0\f$.
///
/// **Threading:** not thread-safe; the follower uses it from the control thread only.

#ifndef VECTORFIELD_CURVE_FOLLOWER__FRAME_TWIST_ESTIMATOR_HPP_
#define VECTORFIELD_CURVE_FOLLOWER__FRAME_TWIST_ESTIMATOR_HPP_

#include <cstdint>

#include "vectorfield_core/types.hpp"

namespace vectorfield_curve_follower
{

/// @brief Settings of FrameTwistEstimator. Defaults equal the follower's parameter defaults
/// (`frame_twist_cutoff_hz`, `frame_twist_max_speed`, `input_timeout`).
struct FrameTwistConfig
{
  /// Minimum Δt of a differencing pair (s).
  double min_dt{0.01};
  /// Maximum Δt of a differencing pair (s); longer gaps restart the estimator.
  double max_dt{0.3};
  /// LPF cutoff (Hz); <= 0 disables the filter.
  double cutoff_hz{2.0};
  /// Outlier limit of the linear speed (m/s).
  double max_speed{2.0};
  /// Outlier limit of the yaw rate (rad/s).
  double max_angular_rate{2.0 * vectorfield_core::kPi};
  /// Extrapolation limit and staleness age (s).
  double input_timeout{0.3};
};

/// @brief What a sample did to the estimator.
enum class SampleResult : std::uint8_t
{
  /// A twist was computed (TF) or taken (Odometry) and the latest pose updated.
  kAccepted = 0,
  /// The first sample, or the first after a gap: pose only, no twist yet.
  kStarted = 1,
  /// Same stamp as the latest sample: ignored.
  kDuplicate = 2,
  /// Closer than `min_dt` to the differencing base: the latest pose only.
  kTooClose = 3,
  /// The twist exceeds the outlier limits: dropped.
  kOutlier = 4,
  /// Non-finite input, or a stamp older than the latest: dropped.
  kInvalid = 5,
};

/// @brief The frame at a query time.
struct FrameEstimate
{
  /// A pose and a twist are known (at least one accepted pair, or one Odometry sample).
  bool ready{false};
  /// Not ready, or the latest sample is older than `input_timeout`: the pose is held and the
  /// twist is zero.
  bool stale{true};
  /// \f$T_{\mathrm{ref}\leftarrow\mathcal F}\f$ at the query time (extrapolated, or held).
  vectorfield_core::Pose2 pose;
  /// \f$(v_{\mathcal F}, \omega_{\mathcal F})\f$ in \f$\mathcal F\f$ (zero when stale).
  vectorfield_core::Twist2 twist;
  /// Age of the latest sample at the query time (s); +Inf without samples.
  double age{vectorfield_core::kInf};
};

/// @brief Estimates the pose and twist of a moving frame (see the file comment).
class FrameTwistEstimator
{
public:
  /// @brief An empty estimator.
  explicit FrameTwistEstimator(const FrameTwistConfig & config = FrameTwistConfig{}) noexcept;

  /// @brief New settings; keeps the samples and the filter state.
  void setConfig(const FrameTwistConfig & config) noexcept;
  /// @brief The settings.
  const FrameTwistConfig & config() const noexcept { return config_; }
  /// @brief Forgets every sample and the filter state.
  void reset() noexcept;

  /// @brief TF source: a sample of \f$T_{\mathrm{ref}\leftarrow\mathcal F}\f$.
  /// @param stamp Sample stamp (s, the node's clock).
  /// @param pose The frame's pose in the reference frame.
  SampleResult addPose(double stamp, const vectorfield_core::Pose2 & pose) noexcept;

  /// @brief Odometry source: a pose and the twist in the frame.
  /// @param stamp Message stamp (s).
  /// @param pose The frame's pose in the Odometry's parent frame.
  /// @param twist Twist of the frame expressed in the frame (Odometry child-frame twist).
  SampleResult addOdometry(
    double stamp, const vectorfield_core::Pose2 & pose,
    const vectorfield_core::Twist2 & twist) noexcept;

  /// @brief The frame at time `now` (see the file comment).
  FrameEstimate estimate(double now) const noexcept;

  /// @brief Number of samples dropped as outliers since the last reset().
  std::uint32_t outliers() const noexcept { return outliers_; }

  /// @brief \f$\log_{SE(2)}(T_a^{-1} T_b)/\Delta t\f$: the constant body twist (in frame a's
  /// axes) that moves `a` to `b` in `dt`, with the yaw difference wrapped into
  /// \f$(-\pi, \pi]\f$.
  /// @return Zero for a non-positive or non-finite `dt`.
  static vectorfield_core::Twist2 bodyTwist(
    const vectorfield_core::Pose2 & a, const vectorfield_core::Pose2 & b, double dt) noexcept;

private:
  bool isOutlier(const vectorfield_core::Twist2 & twist) const noexcept;
  void filter(const vectorfield_core::Twist2 & twist, double dt) noexcept;

  FrameTwistConfig config_;
  bool has_latest_{false};
  double latest_stamp_{0.0};
  vectorfield_core::Pose2 latest_pose_;
  bool has_base_{false};
  double base_stamp_{0.0};
  vectorfield_core::Pose2 base_pose_;
  bool has_twist_{false};
  vectorfield_core::Twist2 twist_;
  std::uint32_t outliers_{0};
};

}  // namespace vectorfield_curve_follower

#endif  // VECTORFIELD_CURVE_FOLLOWER__FRAME_TWIST_ESTIMATOR_HPP_
