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

// Compile check of every public header (each header is also compiled alone by the
// vf_core_header_check target) and of the stable numeric surfaces they define.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "vectorfield_core/collision_predictor.hpp"
#include "vectorfield_core/conditioning.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/contour.hpp"
#include "vectorfield_core/curve.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/errors.hpp"
#include "vectorfield_core/field.hpp"
#include "vectorfield_core/footprint.hpp"
#include "vectorfield_core/function_ref.hpp"
#include "vectorfield_core/gains.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/limiter.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/parametric_curve.hpp"
#include "vectorfield_core/polyline_curve.hpp"
#include "vectorfield_core/simulator.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/tracker.hpp"
#include "vectorfield_core/tracking_law.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_core/vectorfield_core.hpp"

namespace vf = vectorfield_core;

// FieldState.msg PHASE_* constants (vectorfield_msgs); vectorfield_ros asserts the same.
static_assert(vf::toUnderlying(vf::Phase::kRotateToField) == 0, "PHASE_ROTATE_TO_FIELD");
static_assert(vf::toUnderlying(vf::Phase::kFollow) == 1, "PHASE_FOLLOW");
static_assert(vf::toUnderlying(vf::Phase::kFinalApproach) == 2, "PHASE_FINAL_APPROACH");
static_assert(vf::toUnderlying(vf::Phase::kRotateToHeading) == 3, "PHASE_ROTATE_TO_HEADING");
static_assert(vf::toUnderlying(vf::Phase::kStopped) == 4, "PHASE_STOPPED");

// FieldState.msg STATUS_* constants.
static_assert(vf::Status::kNonFinite == 1U, "STATUS_NON_FINITE");
static_assert(vf::Status::kAssumption3Clamped == 2U, "STATUS_ASSUMPTION3_CLAMPED");
static_assert(vf::Status::kAmbiguousClosest == 4U, "STATUS_AMBIGUOUS_CLOSEST");
static_assert(vf::Status::kHairpin == 8U, "STATUS_HAIRPIN");
static_assert(vf::Status::kCurvatureSaturated == 16U, "STATUS_CURVATURE_SATURATED");
static_assert(vf::Status::kContourWatchdog == 32U, "STATUS_CONTOUR_WATCHDOG");
static_assert(vf::Status::kHeadingInfeasible == 64U, "STATUS_HEADING_INFEASIBLE");
static_assert(vf::Status::kCollisionAhead == 128U, "STATUS_COLLISION_AHEAD");
static_assert(vf::Status::kBlocked == 256U, "STATUS_BLOCKED");

// Curve.msg type constants.
static_assert(static_cast<int>(vf::CurveSpec::Type::kPolyline) == 0, "POLYLINE");
static_assert(static_cast<int>(vf::CurveSpec::Type::kExpression) == 1, "EXPRESSION");
static_assert(static_cast<int>(vf::CurveSpec::Type::kBuiltin) == 2, "BUILTIN");

// Plan §3.1: the cross-cycle state is trivially destructible, sub-structs included.
static_assert(std::is_trivially_destructible_v<vf::TrackerState>, "TrackerState");
static_assert(std::is_trivially_destructible_v<vf::ContourState>, "ContourState");
static_assert(std::is_trivially_destructible_v<vf::CurveHint>, "CurveHint");
static_assert(std::is_trivially_destructible_v<vf::LimiterState>, "LimiterState");
static_assert(std::is_trivially_destructible_v<vf::Pose2>, "Pose2");
static_assert(std::is_trivially_destructible_v<vf::Twist2>, "Twist2");
static_assert(std::is_trivially_destructible_v<vf::Isometry2>, "Isometry2");

// Hot-path records are fixed-size too (no allocation in steady state, oracle O55).
static_assert(std::is_trivially_destructible_v<vf::FieldSample>, "FieldSample");
static_assert(std::is_trivially_destructible_v<vf::ClosestPoint>, "ClosestPoint");
static_assert(std::is_trivially_destructible_v<vf::ProbeSample>, "ProbeSample");
static_assert(std::is_trivially_destructible_v<vf::TrackerOutput>, "TrackerOutput");
static_assert(std::is_trivially_destructible_v<vf::Kinematics>, "Kinematics");
static_assert(std::is_trivially_copyable_v<vf::FieldProbe>, "FieldProbe never allocates");
static_assert(std::is_trivially_copyable_v<vf::Span<const vf::Pose2>>, "Span is a view");

// The error hierarchy.
static_assert(std::is_base_of_v<std::invalid_argument, vf::ConfigError>, "ConfigError");
static_assert(std::is_base_of_v<vf::ConfigError, vf::CurveError>, "CurveError");

// Curves are used through the abstract base.
static_assert(std::is_abstract_v<vf::Curve>, "Curve is an interface");
static_assert(std::is_abstract_v<vf::ParametricCurve>, "ParametricCurve needs r()");
static_assert(std::is_base_of_v<vf::Curve, vf::PolylineCurve>, "PolylineCurve is a Curve");
static_assert(std::has_virtual_destructor_v<vf::Curve>, "Curve has a virtual destructor");

TEST(HeadersCompile, UmbrellaExposesVersion)
{
  EXPECT_EQ(std::string(VF_CORE_VERSION_STR), vf::version());
}

TEST(HeadersCompile, DefaultRecordsAreValueInitialised)
{
  const vf::TrackerState state{};
  EXPECT_EQ(state.phase, vf::Phase::kFollow);
  EXPECT_FALSE(state.hint.valid);
  EXPECT_FALSE(state.limiter.valid);

  const vf::FieldSample sample{};
  EXPECT_EQ(sample.status, vf::Status::kNone);
  EXPECT_TRUE(std::isinf(sample.contour.D_o));

  const vf::TrackerOutput output{};
  EXPECT_TRUE(output.predicted.empty());
  EXPECT_EQ(output.first_collision, -1);

  const vf::TrackerInput input{};
  EXPECT_EQ(input.curve, nullptr);
  EXPECT_DOUBLE_EQ(input.speed_limit_scale, 1.0);
}

TEST(HeadersCompile, StatusNames)
{
  EXPECT_STREQ(vf::phaseName(vf::Phase::kRotateToHeading), "ROTATE_TO_HEADING");
  EXPECT_STREQ(vf::statusFlagName(vf::Status::kBlocked), "BLOCKED");
  EXPECT_STREQ(vf::statusFlagName(vf::Status::kNone), "NONE");
  EXPECT_STREQ(vf::statusFlagName(3U), "UNKNOWN");
  EXPECT_TRUE(vf::Status::has(vf::Status::kBlocked | vf::Status::kHairpin, vf::Status::kHairpin));
  EXPECT_FALSE(vf::Status::has(vf::Status::kBlocked, vf::Status::kNone));
  EXPECT_TRUE(vf::isRotationPhase(vf::Phase::kRotateToField));
  EXPECT_TRUE(vf::isTranslationPhase(vf::Phase::kFinalApproach));
  EXPECT_FALSE(vf::isTranslationPhase(vf::Phase::kStopped));
}
