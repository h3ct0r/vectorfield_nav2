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

// M7 reversing (math_spec §9.4 [Design, M7], docs/design/m7_reversing.md): plan segmentation,
// the replan hysteresis, and closed-loop oracles that follow a plan segment by segment the way
// the plugin does (window = the active segment, goal = its cusp, drive_sign = its q).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

constexpr double kCuspTol = 0.05;
const double kChord = vf::chordLength(kRes);

/// Poses every `spacing` from a to b (both included), all with the vehicle yaw `yaw`.
std::vector<Pose2> leg(const Vec2 & a, const Vec2 & b, double spacing, double yaw)
{
  std::vector<Pose2> out;
  const int n = std::max(1, static_cast<int>(std::ceil((b - a).norm() / spacing)));
  for (int k = 0; k <= n; ++k) {
    out.push_back(Pose2{Vec2(a + (b - a) * (static_cast<double>(k) / n)), yaw});
  }
  return out;
}

/// An arc driven from `start` with drive sign q and steering side s (+1 left, -1 right) through
/// `angle` (rad) at radius R; the heading turns by q s angle, as the position does about the
/// centre.
std::vector<Pose2> arc(const Pose2 & start, double R, int q, int s, double angle, double spacing)
{
  const Vec2 centre = start.p + (s * R) * vf::J(vf::heading(start.yaw));
  const double alpha = q * s * angle;
  const auto n = static_cast<int>(std::ceil(R * angle / spacing));
  std::vector<Pose2> out;
  for (int k = 0; k <= n; ++k) {
    const double a = alpha * k / n;
    out.push_back(Pose2{Vec2(centre + vf::rotate(a, start.p - centre)), start.yaw + a});
  }
  return out;
}

/// Appends `more` without its first pose (the shared cusp).
void append(std::vector<Pose2> * plan, const std::vector<Pose2> & more)
{
  plan->insert(plan->end(), more.begin() + 1, more.end());
}

std::vector<vf::PlanSegment> segments(const std::vector<Pose2> & plan, int default_sign = 1)
{
  std::vector<vf::PlanSegment> out;
  std::vector<std::size_t> work;
  vf::segmentPlan(
    vf::Span<const Pose2>(plan.data(), plan.size()), kChord, default_sign, &work, &out);
  return out;
}

/// The plugin's segment logic on top of the Runner (math_spec §9.4).
class SegmentFollower
{
public:
  void setPlan(const std::vector<Pose2> & plan, bool replan)
  {
    raw_ = plan;
    std::vector<std::size_t> work;
    vf::segmentPlan(vf::Span<const Pose2>(raw_.data(), raw_.size()), kChord, 1, &work, &segments_);
    active_ = replan ? vf::firstActiveSegment(
                         vf::Span<const vf::PlanSegment>(segments_.data(), segments_.size()), sign_,
                         2.0 * kChord)
                     : 0;
    activate(replan);
  }

  void hook(Runner & runner, vf::TrackerInput & input)
  {
    if (!runner.cycles().empty() && runner.cycles().back().out.settled && !last()) {
      switches.push_back((runner.pose().p - stop_).norm());
      ++active_;
      activate(false);
    }
    input.curve = &window_->update(runner.pose().p);
    input.goal = goalAt(stop_, 0.0, false, last() ? kXyTol : 2.0 * kCuspTol);
    input.goal_seq = goal_seq_;
    input.drive_sign = static_cast<std::int8_t>(sign_);
    input.cap_at_curve_end = true;
    signs.push_back(sign_);
  }

  bool last() const { return active_ + 1 == segments_.size(); }
  int sign() const { return sign_; }

  /// Distance to the cusp at each switch.
  std::vector<double> switches;
  /// Drive sign of every cycle.
  std::vector<int> signs;

private:
  void activate(bool keep_identity)
  {
    const vf::PlanSegment & segment = segments_[active_];
    std::vector<Vec2> points;
    for (std::size_t k = segment.first; k <= segment.last; ++k) {
      points.push_back(raw_[k].p);
    }
    window_ = std::make_unique<PrunedWindow>(points, 2.0);
    const Vec2 stop = raw_[segment.last].p;
    const int sign = static_cast<int>(segment.drive_sign);
    const bool same = keep_identity && sign == sign_ && (stop - stop_).norm() < 2.0 * kChord;
    if (!same) {
      ++goal_seq_;
    }
    stop_ = stop;
    sign_ = sign;
  }

  std::vector<Pose2> raw_;
  std::vector<vf::PlanSegment> segments_;
  std::size_t active_{0};
  std::unique_ptr<PrunedWindow> window_;
  std::uint32_t goal_seq_{0};
  int sign_{0};
  Vec2 stop_{Vec2::Zero()};
};

vf::CoreConfig reversingConfig(vf::MotionModel model, vf::TrackingLaw law)
{
  vf::CoreConfig config = tcConfig(model, law);
  config.motion.allow_reversing = true;
  config.phases.cusp_tolerance = kCuspTol;
  return config;
}

struct RunResult
{
  std::vector<Cycle> cycles;
  std::vector<double> switches;
  std::vector<int> signs;
  bool settled{false};
  Pose2 final_pose;
};

RunResult follow(
  const vf::CoreConfig & config, const std::vector<Pose2> & plan, const Pose2 & start,
  const ObstacleWorld * world, double duration,
  const std::function<void(Runner &, SegmentFollower &)> & each = {})
{
  Runner runner(config);
  runner.reset(start);
  SegmentFollower follower;
  follower.setPlan(plan, false);
  if (world != nullptr) {
    runner.input().obstacles = world->map();
  }
  runner.setPreStep([&](Runner & r, vf::TrackerInput & input) {
    if (each) {
      each(r, follower);
    }
    follower.hook(r, input);
  });
  RunResult result;
  runner.run(duration, [&](const Cycle & c) { return follower.last() && c.out.settled; });
  result.cycles = runner.cycles();
  result.switches = follower.switches;
  result.signs = follower.signs;
  result.settled = !result.cycles.empty() && result.cycles.back().out.settled;
  result.final_pose = runner.pose();
  return result;
}

/// Largest D at the law's point over the cycles with drive sign q (skipping `skip` cycles after
/// each switch, while the robot leaves the cusp).
double maxDistance(const RunResult & r, int q)
{
  double worst = 0.0;
  for (std::size_t k = 0; k < r.cycles.size(); ++k) {
    if (r.signs[k] == q && std::isfinite(r.cycles[k].out.field.D)) {
      worst = std::max(worst, r.cycles[k].out.field.D);
    }
  }
  return worst;
}

double minClearance(const RunResult & r, const ObstacleWorld & world)
{
  double worst = vf::kInf;
  for (const Cycle & c : r.cycles) {
    worst = std::min(worst, world.distanceToSites(c.input.pose.p) - kRCirc);
  }
  return worst;
}

// ------------------------------------------------------------------------------------------------
// Segmentation

TEST(Reversing, SegmentsTakeTheirDirectionFromHeadings)
{
  // A -> B forward, then B -> C in reverse, along 0.5 rad (non-zero yaws: the plan has headings).
  const double th = 0.5;
  const Vec2 u = vf::heading(th);
  std::vector<Pose2> plan = leg(Vec2::Zero(), 2.0 * u, 0.05, th);
  append(&plan, leg(2.0 * u, 0.8 * u, 0.05, th));
  const std::vector<vf::PlanSegment> s = segments(plan);
  ASSERT_EQ(s.size(), 2U);
  EXPECT_EQ(s[0].drive_sign, 1);
  EXPECT_EQ(s[1].drive_sign, -1);
  EXPECT_LT((plan[s[0].last].p - 2.0 * u).norm(), 1e-9);  // the cusp is the turning pose
  EXPECT_EQ(s[0].last, s[1].first);
  EXPECT_NEAR(s[0].length, 2.0, 1e-9);
  EXPECT_NEAR(s[1].length, 1.2, 1e-9);

  // A plan that reverses from the start (Smac: the goal behind the car): one reverse segment.
  const std::vector<Pose2> back = leg(Vec2::Zero(), -1.5 * u, 0.05, th);
  const std::vector<vf::PlanSegment> b = segments(back);
  ASSERT_EQ(b.size(), 1U);
  EXPECT_EQ(b[0].drive_sign, -1);
}

TEST(Reversing, PlansWithoutHeadingsAlternateFromTheConfiguredSign)
{
  // NavFn-style identity orientations: forward first, then alternate.
  std::vector<Pose2> plan = leg(Vec2::Zero(), Vec2(0.0, 1.5), 0.05, 0.0);
  append(&plan, leg(Vec2(0.0, 1.5), Vec2(0.0, 0.5), 0.05, 0.0));
  append(&plan, leg(Vec2(0.0, 0.5), Vec2(0.0, 1.0), 0.05, 0.0));
  std::vector<vf::PlanSegment> s = segments(plan);
  ASSERT_EQ(s.size(), 3U);
  EXPECT_EQ(s[0].drive_sign, 1);
  EXPECT_EQ(s[1].drive_sign, -1);
  EXPECT_EQ(s[2].drive_sign, 1);
  s = segments(plan, -1);  // drive_backward
  EXPECT_EQ(s[0].drive_sign, -1);
  EXPECT_EQ(s[1].drive_sign, 1);
  // A reversal shorter than l_c is no cusp (math_spec §2.8).
  std::vector<Pose2> hook = leg(Vec2::Zero(), Vec2(1.0, 0.0), 0.025, 0.0);
  hook.push_back(Pose2{Vec2(0.97, 0.0), 0.0});
  EXPECT_EQ(segments(hook).size(), 1U);
  // A forward plan with headings is one forward segment.
  EXPECT_EQ(segments(leg(Vec2::Zero(), Vec2(1.0, 1.0), 0.05, 0.25 * vf::kPi))[0].drive_sign, 1);
}

TEST(Reversing, AShortContraryFirstSegmentIsSkippedOnReplans)
{
  // The robot reversed at a cusp; a replan from its pose puts the cusp one cell further.
  const double th = 0.3;
  const Vec2 u = vf::heading(th);
  std::vector<Pose2> plan = leg(Vec2::Zero(), 0.05 * u, 0.025, th);
  append(&plan, leg(0.05 * u, -1.0 * u, 0.05, th));
  // The 5 cm forward piece is below l_c: no cusp at all, one reverse segment.
  std::vector<vf::PlanSegment> s = segments(plan);
  ASSERT_EQ(s.size(), 1U);
  EXPECT_EQ(s[0].drive_sign, -1);
  // A 15 cm forward piece (between l_c and 2 l_c) is a cusp, skipped while reversing.
  plan = leg(Vec2::Zero(), 0.15 * u, 0.025, th);
  append(&plan, leg(0.15 * u, -1.0 * u, 0.05, th));
  s = segments(plan);
  ASSERT_EQ(s.size(), 2U);
  const vf::Span<const vf::PlanSegment> view(s.data(), s.size());
  EXPECT_EQ(vf::firstActiveSegment(view, -1, 2.0 * kChord), 1U);
  EXPECT_EQ(vf::firstActiveSegment(view, 1, 2.0 * kChord), 0U);
  EXPECT_EQ(vf::firstActiveSegment(view, 0, 2.0 * kChord), 0U);
  EXPECT_EQ(vf::firstActiveSegment(view, -1, 0.1), 0U);
}

TEST(Reversing, InPlaceRotationsAreFound)
{
  std::vector<Pose2> plan = leg(Vec2::Zero(), Vec2(1.0, 0.0), 0.05, 0.0);
  EXPECT_FALSE(vf::hasInPlaceRotation(vf::Span<const Pose2>(plan.data(), plan.size())));
  plan.push_back(Pose2{Vec2(1.0, 0.0), 0.5});  // the goal yaw on the last pose: not a rotation
  EXPECT_FALSE(vf::hasInPlaceRotation(vf::Span<const Pose2>(plan.data(), plan.size())));
  plan.push_back(Pose2{Vec2(1.0, 0.5), 0.5});
  EXPECT_TRUE(vf::hasInPlaceRotation(vf::Span<const Pose2>(plan.data(), plan.size())));
}

// ------------------------------------------------------------------------------------------------
// Closed loop: A -> B forward, B -> C in reverse, in a corridor

struct LawCase
{
  vf::MotionModel model;
  vf::TrackingLaw law;
  double max_d;
};

class StraightReversal : public ::testing::TestWithParam<LawCase>
{
};

TEST_P(StraightReversal, StopsAtTheCuspAndReversesToTheGoal)
{
  const LawCase & param = GetParam();
  const double th = 0.5;
  const Vec2 u = vf::heading(th);
  const Vec2 B = 2.0 * u;
  const Vec2 C = 0.8 * u;
  std::vector<Pose2> plan = leg(Vec2::Zero(), B, 0.05, th);
  append(&plan, leg(B, C, 0.05, th));
  // A 1.1 m corridor along the path (walls 0.55 m from its centre line).
  ObstacleWorld world(Vec2(-2.0, -2.0), 6.0, 6.0);
  const Vec2 n = vf::J(u);
  for (int k = 0; k <= 90; ++k) {
    const double s = -0.8 + 0.04 * k;
    world.addDisc(s * u + 0.6 * n, 0.04);
    world.addDisc(s * u - 0.6 * n, 0.04);
  }
  world.commit();

  const RunResult r =
    follow(reversingConfig(param.model, param.law), plan, Pose2{Vec2::Zero(), th}, &world, 40.0);
  ASSERT_TRUE(r.settled) << "final pose " << r.final_pose.p.transpose();
  ASSERT_EQ(r.switches.size(), 1U);
  EXPECT_LE(r.switches[0], (param.model == vf::MotionModel::kAckermann ? 2.0 : 1.0) * kCuspTol);
  EXPECT_LT((r.final_pose.p - C).norm(), 0.5 * kXyTol + 1e-6);
  // The robot keeps facing +u: it reversed, it did not turn around.
  EXPECT_LT(std::abs(vf::wrapAngle(r.final_pose.yaw - th)), 0.15);
  EXPECT_LT(maxDistance(r, 1), param.max_d);
  EXPECT_LT(maxDistance(r, -1), param.max_d);
  EXPECT_GT(minClearance(r, world), 0.0);
  bool reversed = false;
  for (std::size_t k = 0; k < r.cycles.size(); ++k) {
    const Cycle & c = r.cycles[k];
    EXPECT_NE(c.out.phase, vf::Phase::kRotateToField) << "cycle " << k;
    EXPECT_NE(c.out.phase, vf::Phase::kRotateToHeading) << "cycle " << k;
    EXPECT_FALSE(vf::Status::has(c.out.status, vf::Status::kBlocked)) << "cycle " << k;
    EXPECT_FALSE(vf::Status::has(c.out.status, vf::Status::kHeadingInfeasible)) << "cycle " << k;
    // Motion in the drive direction (the limiter takes a cycle to pass through zero).
    EXPECT_GE(r.signs[k] * c.out.cmd.vx, -0.06) << "cycle " << k;
    reversed = reversed || c.out.cmd.vx < -0.1;
    if (param.model == vf::MotionModel::kAckermann) {
      EXPECT_LE(std::abs(c.out.cmd.wz), std::abs(c.out.cmd.vx) / 0.5 + 1e-9) << "cycle " << k;
    }
  }
  EXPECT_TRUE(reversed);
}

INSTANTIATE_TEST_SUITE_P(
  M7, StraightReversal,
  ::testing::Values(
    LawCase{vf::MotionModel::kDifferential, vf::TrackingLaw::kFeedbackLinearization, 0.10},
    LawCase{vf::MotionModel::kDifferential, vf::TrackingLaw::kHeadingTracking, 0.10},
    LawCase{vf::MotionModel::kAckermann, vf::TrackingLaw::kHeadingTracking, 0.10}));

TEST(Reversing, AckermannThreePointTurn)
{
  // Forward left 60 deg, reverse right 60 deg, forward left 60 deg at R = 0.6 > R_min = 0.5:
  // back at the origin facing -x.
  const double R = 0.6;
  const double a = vf::kPi / 3.0;
  std::vector<Pose2> plan = arc(Pose2{Vec2::Zero(), 0.0}, R, 1, 1, a, 0.05);
  append(&plan, arc(plan.back(), R, -1, -1, a, 0.05));
  append(&plan, arc(plan.back(), R, 1, 1, a, 0.05));
  const std::vector<vf::PlanSegment> s = segments(plan);
  ASSERT_EQ(s.size(), 3U);
  EXPECT_EQ(s[0].drive_sign, 1);
  EXPECT_EQ(s[1].drive_sign, -1);
  EXPECT_EQ(s[2].drive_sign, 1);

  const RunResult r = follow(
    reversingConfig(vf::MotionModel::kAckermann, vf::TrackingLaw::kAuto), plan,
    Pose2{Vec2::Zero(), 0.0}, nullptr, 40.0);
  ASSERT_TRUE(r.settled) << "final pose " << r.final_pose.p.transpose();
  ASSERT_EQ(r.switches.size(), 2U);
  for (const double d : r.switches) {
    EXPECT_LE(d, 2.0 * kCuspTol);
  }
  // Ackermann settles along the track (math_spec §9.1): within xy_tol of the goal.
  EXPECT_LT(r.final_pose.p.norm(), kXyTol);
  EXPECT_LT(std::abs(vf::wrapAngle(r.final_pose.yaw - vf::kPi)), 0.3);
  EXPECT_LT(maxDistance(r, 1), 0.15);
  EXPECT_LT(maxDistance(r, -1), 0.15);
  for (const Cycle & c : r.cycles) {
    EXPECT_LE(std::abs(c.out.cmd.wz), std::abs(c.out.cmd.vx) / 0.5 + 1e-9);
    EXPECT_FALSE(vf::Status::has(c.out.status, vf::Status::kHeadingInfeasible));
  }
}

TEST(Reversing, CuspJitterAcrossReplansDoesNotFlipTheDirection)
{
  // Replans every second from the robot's pose, with the cusp moved by +-1 cell each time.
  const double th = 0.5;
  const Vec2 u = vf::heading(th);
  const Vec2 C = 0.8 * u;
  int replan = 0;
  const auto planFrom = [&](const Vec2 & p, double cusp) {
    std::vector<Pose2> plan = leg(p, cusp * u, 0.05, th);
    append(&plan, leg(cusp * u, C, 0.05, th));
    return plan;
  };
  const std::vector<Pose2> plan = planFrom(Vec2::Zero(), 2.0);
  const RunResult r = follow(
    reversingConfig(vf::MotionModel::kDifferential, vf::TrackingLaw::kAuto), plan,
    Pose2{Vec2::Zero(), th}, nullptr, 40.0, [&](Runner & runner, SegmentFollower & follower) {
      const auto k = runner.cycles().size();
      if (k > 0 && k % 20 == 0) {
        const double cusp = 2.0 + ((++replan % 2 == 0) ? kRes : -kRes);
        // Project the robot onto the line: the planner starts at the robot.
        const Vec2 p = runner.pose().p.dot(u) * u;
        const double s = p.dot(u);
        if (follower.sign() > 0 && s < cusp - 0.02) {
          follower.setPlan(planFrom(p, cusp), true);
        } else if (follower.sign() < 0 && s > C.dot(u) + 0.3) {
          std::vector<Pose2> back = leg(p, C, 0.05, th);
          if (s < cusp && s > cusp - 2.0 * kChord) {
            back = planFrom(p, cusp);  // still within 2 l_c of the jittered cusp
          }
          follower.setPlan(back, true);
        }
      }
    });
  ASSERT_TRUE(r.settled);
  int flips = 0;
  for (std::size_t k = 1; k < r.signs.size(); ++k) {
    flips += r.signs[k] != r.signs[k - 1] ? 1 : 0;
  }
  EXPECT_EQ(flips, 1);
  EXPECT_LT((r.final_pose.p - C).norm(), 0.5 * kXyTol + 1e-6);
}

TEST(Reversing, TheContourIsOffInAReverseSegment)
{
  // A disc on the reverse leg with obstacle_mode contour: no episode starts while reversing.
  const double th = 0.0;
  std::vector<Pose2> plan = leg(Vec2::Zero(), Vec2(1.5, 0.0), 0.05, th);
  append(&plan, leg(Vec2(1.5, 0.0), Vec2(-1.5, 0.0), 0.05, th));
  ObstacleWorld world(Vec2(-3.0, -2.0), 6.0, 4.0);
  world.addDisc(Vec2(-0.8, 0.0), 0.15);
  world.commit();
  vf::CoreConfig config = reversingConfig(vf::MotionModel::kDifferential, vf::TrackingLaw::kAuto);
  config.obstacles.obstacle_mode = vf::ObstacleMode::kContour;
  config.obstacles.obstacle_activation = vf::ObstacleActivation::kAlways;
  const RunResult r = follow(config, plan, Pose2{Vec2::Zero(), th}, &world, 30.0);
  bool blocked = false;
  for (std::size_t k = 0; k < r.cycles.size(); ++k) {
    if (r.signs[k] < 0) {
      EXPECT_FALSE(r.cycles[k].after.contour.active) << "cycle " << k;
      blocked = blocked || vf::Status::has(r.cycles[k].out.status, vf::Status::kBlocked);
    }
  }
  EXPECT_TRUE(blocked);  // the obstacle stops the robot instead
  EXPECT_GT(minClearance(r, world), 0.0);
}

}  // namespace
