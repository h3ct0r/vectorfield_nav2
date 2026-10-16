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

// O10 Lemniscates (math_spec §2.4 hinted search and adoption rule, §13 "Self-intersecting
// curves", §12.1): Gerono (a = 4, b = 1.5; perimeter 21.5 m) and Bernoulli (a = 2; 10.5 m) under
// HT and FL, with 1–5 cm lateral noise injected near the crossing: over 5 laps the visited lobe
// alternates every half-period (no repeated lobe).

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

// One lateral kick of 1–5 cm (seeded magnitude and sign) per passage through the crossing
// (base within 0.6 m of the origin); a passage ends when the base is more than 1 m away. The kick
// is a world-frame velocity offset/period along J h(ψ) for one control period, i.e. a lateral
// displacement of exactly `offset` (sim::Disturbance semantics).
struct CrossingNoise
{
  explicit CrossingNoise(std::uint64_t seed) : rng(seed) {}
  std::mt19937_64 rng;
  bool in_passage{false};
  int kicks{0};
};

vf::sim::Disturbance crossingNoise(const std::shared_ptr<CrossingNoise> & state)
{
  return [state](double, const Pose2 & pose) -> Vec2 {
    const double r = pose.p.norm();
    if (r > 1.0) {
      state->in_passage = false;
      return Vec2::Zero();
    }
    if (r > 0.6 || state->in_passage) {
      return Vec2::Zero();
    }
    state->in_passage = true;
    ++state->kicks;
    std::uniform_real_distribution<double> magnitude(0.01, 0.05);
    std::bernoulli_distribution sign(0.5);
    const double offset = (sign(state->rng) ? 1.0 : -1.0) * magnitude(state->rng);
    return (offset / kPeriod) * vf::J(vf::heading(pose.yaw));
  };
}

struct LobeLog
{
  std::vector<int> visits;
  int kicks{0};
};

// Records a lobe visit (sign of x) each time the base enters |x| > threshold after having been
// near the crossing (‖p‖ < 0.5) or at the start.
LobeLog runLemniscate(
  const std::shared_ptr<const vf::Curve> & curve, vf::TrackingLaw law, const Vec2 & tip,
  double threshold, double timeout, std::uint64_t seed)
{
  auto noise = std::make_shared<CrossingNoise>(seed);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  options.disturbance = crossingNoise(noise);
  Runner runner(tcConfig(vf::MotionModel::kDifferential, law), options);
  runner.reset(Pose2{tip, vf::kHalfPi});
  runner.input().curve = curve.get();
  LobeLog log;
  int current = 0;
  runner.run(timeout, [&](const Cycle & c) {
    const Vec2 & p = c.pose_after.p;
    if (p.norm() < 0.5) {
      current = 0;
    } else if (current == 0 && std::abs(p.x()) > threshold) {
      current = p.x() > 0.0 ? 1 : -1;
      log.visits.push_back(current);
    }
    return log.visits.size() >= 11U;
  });
  log.kicks = noise->kicks;
  return log;
}

void expectAlternatingLobes(const LobeLog & log)
{
  ASSERT_GE(log.visits.size(), 11U) << "fewer than 5 laps within the timeout";
  EXPECT_GE(log.kicks, 10) << "noise was not injected at every crossing";
  for (std::size_t k = 1; k < log.visits.size(); ++k) {
    EXPECT_NE(log.visits[k], log.visits[k - 1]) << "repeated lobe at visit " << k;
  }
}

}  // namespace

// Gerono a = 4, b = 1.5 (crossing angle 106°), FL and HT, start at the right tip along the
// tangent. Timeout 380 s (5 laps at v_r take 269 s).
TEST(ScenarioLemniscate, O10_GeronoLobesAlternate)
{
  const auto curve = makeBuiltin("lemniscate_gerono", {{"a", 4.0}, {"b", 1.5}});
  std::uint64_t seed = 1010;
  for (const vf::TrackingLaw law :
       {vf::TrackingLaw::kFeedbackLinearization, vf::TrackingLaw::kHeadingTracking}) {
    SCOPED_TRACE(law == vf::TrackingLaw::kHeadingTracking ? "HT" : "FL");
    expectAlternatingLobes(runLemniscate(curve, law, Vec2(4.0, 0.0), 2.0, 380.0, seed++));
  }
}

// Bernoulli a = 2 (crossing angle 90°), FL and HT. Timeout 190 s (5 laps at v_r take 131 s).
TEST(ScenarioLemniscate, O10_BernoulliLobesAlternate)
{
  const auto curve = makeBuiltin("lemniscate_bernoulli", {{"a", 2.0}});
  std::uint64_t seed = 1020;
  for (const vf::TrackingLaw law :
       {vf::TrackingLaw::kFeedbackLinearization, vf::TrackingLaw::kHeadingTracking}) {
    SCOPED_TRACE(law == vf::TrackingLaw::kHeadingTracking ? "HT" : "FL");
    expectAlternatingLobes(runLemniscate(curve, law, Vec2(2.0, 0.0), 1.0, 190.0, seed++));
  }
}
