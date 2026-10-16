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

// Unit tests of the CollisionPredictor rollout and the braking cap (math_spec §10.2–§10.3).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "vectorfield_core/collision_predictor.hpp"
#include "vectorfield_core/footprint.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/occupancy.hpp"

namespace vectorfield_core
{
namespace
{

constexpr double kRes = 0.05;

/// A square grid of `cells` x `cells` cells of 0.05 m whose lower-left corner is `origin`.
class GridWorld
{
public:
  explicit GridWorld(int cells = 80, const Vec2 & origin = Vec2(-2.0, -2.0))
  : cells_(cells), origin_(origin), mask_(static_cast<std::size_t>(cells * cells), 0)
  {
  }

  /// Marks the cell containing (x, y) as a site.
  void site(double x, double y)
  {
    const int ix = static_cast<int>(std::floor((x - origin_.x()) / kRes));
    const int iy = static_cast<int>(std::floor((y - origin_.y()) / kRes));
    mask_[static_cast<std::size_t>(iy * cells_ + ix)] = 1;
  }

  /// A wall of sites along the full column containing x.
  void wallAtX(double x)
  {
    for (int iy = 0; iy < cells_; ++iy) {
      site(x, origin_.y() + (iy + 0.5) * kRes);
    }
  }

  /// A wall of sites along the full row containing y.
  void wallAtY(double y)
  {
    for (int ix = 0; ix < cells_; ++ix) {
      site(origin_.x() + (ix + 0.5) * kRes, y);
    }
  }

  ObstacleMap map()
  {
    EXPECT_TRUE(grid_.setFromMask(Span<const std::uint8_t>(mask_), cells_, cells_, kRes, origin_));
    edt_.update(grid_);
    return ObstacleMap{&grid_, &edt_, Isometry2()};
  }

private:
  int cells_;
  Vec2 origin_;
  std::vector<std::uint8_t> mask_;
  OccupancyGrid grid_;
  EuclideanDistanceTransform edt_;
};

/// Holds one command for every period; optionally settles from period `settle_at` on.
struct ConstantPolicy
{
  Twist2 cmd;
  Phase phase{Phase::kFollow};
  int settle_at{-1};
  int calls{0};

  RolloutStep operator()(const Pose2 &, double)
  {
    const int k = calls++;
    return RolloutStep{cmd, phase, settle_at >= 0 && k >= settle_at};
  }
};

/// A STOPPED ramp: the speed falls by `step` per period from `start`.
struct RampPolicy
{
  double start{0.4};
  double step{0.02};
  int calls{0};

  RolloutStep operator()(const Pose2 &, double)
  {
    const double v = std::max(0.0, start - step * calls++);
    return RolloutStep{Twist2{v, 0.0, 0.0}, Phase::kStopped, false};
  }
};

RolloutOptions options(double horizon = 1.5, bool through_cancel = false)
{
  RolloutOptions result;
  result.t0 = 0.0;
  result.period = 0.05;
  result.horizon = horizon;
  result.through_cancel = through_cancel;
  return result;
}

TEST(BrakingCap, FollowsTheStoppingDistance)
{
  EXPECT_EQ(brakingCap(kInf, 0.1, 1.0), kInf);
  EXPECT_NEAR(brakingCap(0.18, 0.1, 1.0), 0.4, 1e-12);
  EXPECT_EQ(brakingCap(0.05, 0.1, 1.0), 0.0);  // inside the margin
  EXPECT_EQ(brakingCap(kNaN, 0.1, 1.0), 0.0);
  EXPECT_EQ(brakingCap(1.0, 0.1, 0.0), 0.0);
}

TEST(CollisionPredictor, FreeSpaceIsInfiniteAndSubStepsStayWithinOneCell)
{
  GridWorld world(120, Vec2(-3.0, -3.0));  // the 2.25 m arc stays on the grid
  const ObstacleMap map = world.map();
  const CollisionPredictor predictor(Kinematics{}, FootprintChecker(Footprint::circle(0.22)));
  ConstantPolicy policy{Twist2{1.5, 0.0, 0.4}};  // 0.075 m per period: two sub-steps
  std::vector<Pose2> poses;
  const RolloutResult result = predictor.rollout(Pose2{}, map, options(), policy, &poses);
  EXPECT_EQ(result.s_free, kInf);
  EXPECT_FALSE(result.collided);
  EXPECT_TRUE(result.reached_horizon);
  EXPECT_EQ(result.steps, 30U);
  EXPECT_EQ(result.first_collision, -1);
  ASSERT_EQ(poses.size(), 1U + 30U * 2U);
  for (std::size_t i = 1; i < poses.size(); ++i) {
    EXPECT_LE((poses[i].p - poses[i - 1].p).norm(), kRes + 1e-12);
  }
}

TEST(CollisionPredictor, FreeArcIsTheLastFreePoseWithinOneSubStep)
{
  // A wall whose first cell column starts at x = 0.6: a 0.22 m disc touches it at x = 0.38.
  GridWorld world;
  world.wallAtX(0.61);
  const ObstacleMap map = world.map();
  const CollisionPredictor predictor(Kinematics{}, FootprintChecker(Footprint::circle(0.22)));
  ConstantPolicy policy{Twist2{0.4, 0.0, 0.0}};
  std::vector<Pose2> poses;
  const RolloutResult result = predictor.rollout(Pose2{}, map, options(), policy, &poses);
  ASSERT_TRUE(result.collided);
  EXPECT_FALSE(result.collided_in_rotation);
  EXPECT_LE(result.s_free, 0.38 + 1e-9);
  EXPECT_GE(result.s_free, 0.38 - kRes - 1e-9);
  ASSERT_GE(result.first_collision, 1);
  EXPECT_GE(poses[static_cast<std::size_t>(result.first_collision)].p.x(), 0.38 - 1e-9);
  EXPECT_EQ(static_cast<std::size_t>(result.first_collision), poses.size() - 1U);
}

TEST(CollisionPredictor, StartPoseCollisionGivesZero)
{
  GridWorld world;
  world.site(0.0, 0.0);
  const ObstacleMap map = world.map();
  const CollisionPredictor predictor(Kinematics{}, FootprintChecker(Footprint::circle(0.22)));
  ConstantPolicy policy{Twist2{0.4, 0.0, 0.0}};
  std::vector<Pose2> poses;
  const RolloutResult result = predictor.rollout(Pose2{}, map, options(), policy, &poses);
  EXPECT_TRUE(result.collided);
  EXPECT_EQ(result.first_collision, 0);
  EXPECT_EQ(result.s_free, 0.0);
  EXPECT_EQ(policy.calls, 1);  // the first step is always computed
}

TEST(CollisionPredictor, RotationIntoAWallGivesZeroAndFlagsTheRotation)
{
  // A long nose (0.6 m x 0.2 m) whose tip sweeps into a wall at y = 0.3 when turning left.
  GridWorld world;
  world.wallAtY(0.31);
  const ObstacleMap map = world.map();
  const std::vector<Vec2> nose = {
    Vec2(-0.1, -0.1), Vec2(0.5, -0.1), Vec2(0.5, 0.1), Vec2(-0.1, 0.1)};
  const FootprintChecker checker(Footprint::polygon(Span<const Vec2>(nose)));
  const CollisionPredictor predictor(Kinematics{}, checker);
  ConstantPolicy policy{Twist2{0.0, 0.0, 1.0}, Phase::kRotateToField};
  std::vector<Pose2> poses;
  const RolloutResult result = predictor.rollout(Pose2{}, map, options(), policy, &poses);
  ASSERT_TRUE(result.collided);
  EXPECT_TRUE(result.collided_in_rotation);
  EXPECT_EQ(result.s_free, 0.0);
  // Yaw sub-steps: a point at r_circ moves by at most one cell.
  const double max_yaw_step = 2.0 * std::asin(kRes / (2.0 * checker.footprint().rCirc()));
  for (std::size_t i = 1; i < poses.size(); ++i) {
    EXPECT_LE(std::abs(wrapAngle(poses[i].yaw - poses[i - 1].yaw)), max_yaw_step + 1e-12);
  }
  // Starting in a translation phase, the same sweep is not the kBlocked rotation case.
  ConstantPolicy translating{Twist2{0.0, 0.0, 1.0}, Phase::kFollow};
  const RolloutResult other = predictor.rollout(Pose2{}, map, options(), translating, &poses);
  EXPECT_TRUE(other.collided);
  EXPECT_FALSE(other.collided_in_rotation);
}

TEST(CollisionPredictor, OffGridEndsAtTheLastOnGridArc)
{
  // A 1 m grid centred on the robot; the 0.27 m reach leaves it beyond x = 0.23.
  GridWorld world(20, Vec2(-0.5, -0.5));
  const ObstacleMap map = world.map();
  const CollisionPredictor predictor(Kinematics{}, FootprintChecker(Footprint::circle(0.22)));
  ConstantPolicy policy{Twist2{0.4, 0.0, 0.0}};
  std::vector<Pose2> poses;
  const RolloutResult result = predictor.rollout(Pose2{}, map, options(), policy, &poses);
  EXPECT_TRUE(result.off_grid);
  EXPECT_FALSE(result.collided);
  EXPECT_NEAR(result.s_free, 0.22, 1e-9);

  const RolloutResult outside =
    predictor.rollout(Pose2{Vec2(0.4, 0.0), 0.0}, map, options(), policy, &poses);
  EXPECT_TRUE(outside.off_grid);
  EXPECT_EQ(outside.s_free, 0.0);
}

TEST(CollisionPredictor, StopsAtSettledAndAtStoppedUnlessCancelling)
{
  GridWorld world;
  const ObstacleMap map = world.map();
  const CollisionPredictor predictor(Kinematics{}, FootprintChecker(Footprint::circle(0.22)));
  std::vector<Pose2> poses;

  ConstantPolicy settling{Twist2{0.2, 0.0, 0.0}, Phase::kFinalApproach, 3};
  const RolloutResult settled = predictor.rollout(Pose2{}, map, options(), settling, &poses);
  EXPECT_TRUE(settled.settled);
  EXPECT_EQ(settled.steps, 4U);  // the settled step is integrated, then the rollout ends
  EXPECT_EQ(settling.calls, 4);
  EXPECT_EQ(settled.s_free, kInf);

  ConstantPolicy stopping{Twist2{0.3, 0.0, 0.0}, Phase::kStopped};
  const RolloutResult stopped = predictor.rollout(Pose2{}, map, options(), stopping, &poses);
  EXPECT_TRUE(stopped.stopped);
  EXPECT_EQ(stopped.steps, 0U);
  EXPECT_EQ(poses.size(), 1U);

  // Through a cancel the ramp is followed past the horizon until the stop thresholds.
  RampPolicy ramp;
  const RolloutResult cancel = predictor.rollout(Pose2{}, map, options(0.5, true), ramp, &poses);
  EXPECT_TRUE(cancel.stopped);
  EXPECT_FALSE(cancel.reached_horizon);
  EXPECT_EQ(cancel.steps, 20U);  // 0.4 - 0.02 k < 0.01 first at k = 20
}

}  // namespace
}  // namespace vectorfield_core
