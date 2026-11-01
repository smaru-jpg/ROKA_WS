#include <gtest/gtest.h>

#include "roka_terrain_mapping/leader_follow_math.hpp"

using roka_terrain_mapping::ConstantVelocityTracker;
using roka_terrain_mapping::LeaderTrail;
using roka_terrain_mapping::Point2;
using roka_terrain_mapping::TargetAssociationLimits;
using roka_terrain_mapping::TrailControlLimits;
using roka_terrain_mapping::kinematic_candidate_is_consistent;
using roka_terrain_mapping::oriented_horizontal_extents;
using roka_terrain_mapping::oriented_target_size_is_valid;
using roka_terrain_mapping::rectangular_footprint_clearance;
using roka_terrain_mapping::required_stopping_clearance;
using roka_terrain_mapping::shape_ratio_is_plausible;
using roka_terrain_mapping::three_point_curvature;
using roka_terrain_mapping::trail_command;

TEST(LeaderClearance, MeasuresFromRobotShellInsteadOfLidarOrCentroid)
{
  EXPECT_NEAR(
    rectangular_footprint_clearance(Point2{0.69, 0.0}, 0.59, 0.29, 0.312),
    0.10, 1e-9);
  EXPECT_NEAR(
    rectangular_footprint_clearance(Point2{0.30, 0.412}, 0.59, 0.29, 0.312),
    0.10, 1e-9);
  EXPECT_DOUBLE_EQ(
    rectangular_footprint_clearance(Point2{0.0, 0.0}, 0.59, 0.29, 0.312),
    0.0);
}

TEST(LeaderClearance, StartsBrakingEarlyEnoughForTenCentimeterFinalGap)
{
  EXPECT_NEAR(required_stopping_clearance(0.60, 0.10, 0.30, 1.0), 0.46, 1e-9);
  EXPECT_NEAR(required_stopping_clearance(1.20, 0.10, 0.30, 1.0), 1.18, 1e-9);
  EXPECT_TRUE(std::isinf(required_stopping_clearance(1.0, 0.10, 0.30, 0.0)));
}

TEST(TargetDimensions, RotatedNarrowBodyDoesNotBecomeArtificiallyWide)
{
  std::vector<Point2> points;
  const double angle = 45.0 * M_PI / 180.0;
  for (const double length : {-0.60, 0.60}) {
    for (const double width : {-0.30, 0.30}) {
      points.push_back(Point2{
        std::cos(angle) * length - std::sin(angle) * width,
        std::sin(angle) * length + std::cos(angle) * width});
    }
  }
  const auto extents = oriented_horizontal_extents(points);
  EXPECT_NEAR(extents.length, 1.20, 1e-9);
  EXPECT_NEAR(extents.width, 0.60, 1e-9);
  EXPECT_TRUE(oriented_target_size_is_valid(extents, 1.0, 0.12, 1.20, 0.60, 2.0));
  EXPECT_FALSE(oriented_target_size_is_valid(extents, 1.0, 0.12, 1.20, 0.59, 2.0));
}

TEST(LeaderGeometry, MeasuresSignedCurvatureAcrossAFilteredChord)
{
  EXPECT_NEAR(
    three_point_curvature(Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{2.0, 0.0}),
    0.0, 1e-9);
  EXPECT_GT(
    three_point_curvature(Point2{0.0, 0.0}, Point2{1.0, 0.2}, Point2{1.8, 0.8}),
    0.16);
}

TEST(LeaderTracker, EstimatesLeaderVelocityAndCoastsThroughMissingMeasurements)
{
  ConstantVelocityTracker tracker;
  tracker.initialize(Point2{0.0, 0.0}, 0.05, 1.0);
  for (int index = 1; index <= 40; ++index) {
    tracker.predict(0.1, 0.5);
    tracker.update(Point2{0.1 * index, 0.0}, 0.02);
  }
  EXPECT_NEAR(tracker.speed(), 1.0, 0.08);
  const double before = tracker.position().x;
  tracker.predict(0.5, 0.5);
  tracker.predict(0.5, 0.5);
  EXPECT_GT(tracker.position().x, before + 0.8);
  EXPECT_GT(tracker.position_uncertainty(), 0.0);
}

TEST(LeaderTracker, RejectMetricGrowsForWrongReacquisitionCandidate)
{
  ConstantVelocityTracker tracker;
  tracker.initialize(Point2{2.0, 1.0}, 0.05, 0.5);
  tracker.predict(0.1, 0.5);
  const double near_score = tracker.innovation_distance_squared(Point2{2.1, 1.0}, 0.05);
  const double far_score = tracker.innovation_distance_squared(Point2{5.0, -2.0}, 0.05);
  EXPECT_LT(near_score, far_score);
  EXPECT_GT(far_score, 12.25);
}

TEST(LeaderTracker, StartsWithMeasuredVelocityAndDampsWithoutDroppingIdentity)
{
  ConstantVelocityTracker tracker;
  tracker.initialize(Point2{2.0, 0.0}, 0.1, 0.5, Point2{1.0, 0.0});
  EXPECT_NEAR(tracker.speed(), 1.0, 1e-9);
  tracker.predict(0.5, 0.5);
  EXPECT_GT(tracker.position().x, 2.45);
  tracker.damp_velocity(0.5);
  EXPECT_NEAR(tracker.speed(), 0.5, 1e-9);
  EXPECT_TRUE(tracker.initialized());
}

TEST(TargetAssociation, RejectsTeleportAndSuddenReverseButAllowsStoppedTarget)
{
  TargetAssociationLimits limits;
  EXPECT_FALSE(kinematic_candidate_is_consistent(
      Point2{2.0, 0.0}, Point2{3.0, 0.0}, 0.1,
      Point2{1.0, 0.0}, false, limits));
  EXPECT_FALSE(kinematic_candidate_is_consistent(
      Point2{2.0, 0.0}, Point2{1.9, 0.0}, 0.1,
      Point2{1.0, 0.0}, false, limits));
  EXPECT_TRUE(kinematic_candidate_is_consistent(
      Point2{2.0, 0.0}, Point2{2.0, 0.0}, 0.1,
      Point2{0.2, 0.0}, false, limits));
}

TEST(TargetAssociation, UsesLooseShapeRatiosOnlyAsGrossRejection)
{
  EXPECT_TRUE(shape_ratio_is_plausible(0.4, 1.0));
  EXPECT_TRUE(shape_ratio_is_plausible(2.5, 1.0));
  EXPECT_FALSE(shape_ratio_is_plausible(3.0, 1.0));
}

TEST(LeaderTrail, SeedsTheUnknownPathBetweenFollowerAndLeader)
{
  LeaderTrail trail;
  trail.seed(Point2{0.0, 0.0}, Point2{2.0, 0.0}, 0.1);
  ASSERT_GT(trail.size(), 10U);
  EXPECT_NEAR(trail.end_station(), 2.0, 1e-9);
  const auto midpoint = trail.sample(1.0);
  ASSERT_TRUE(midpoint.has_value());
  EXPECT_NEAR(midpoint->x, 1.0, 1e-9);
  EXPECT_NEAR(trail.nearest_station(Point2{0.7, 0.2}), 0.7, 1e-9);
}

TEST(LeaderTrail, ProgressWindowPreventsJumpingAcrossNearbyUTurnLanes)
{
  LeaderTrail trail;
  trail.append(Point2{0.0, 0.0}, 0.0);
  trail.append(Point2{4.0, 0.0}, 0.0);
  trail.append(Point2{5.0, 1.0}, 0.0);
  trail.append(Point2{4.0, 2.0}, 0.0);
  trail.append(Point2{0.0, 2.0}, 0.0);
  const double outbound = trail.nearest_station(Point2{1.0, 0.9}, 0.0, 2.0);
  const double returning = trail.nearest_station(
    Point2{1.0, 1.1}, trail.end_station() - 2.0, trail.end_station());
  EXPECT_NEAR(outbound, 1.0, 1e-9);
  EXPECT_GT(returning, trail.end_station() - 1.1);
}

TEST(LeaderTrail, RemovesWrongOcclusionPredictionTailOnReacquisition)
{
  LeaderTrail trail;
  trail.append(Point2{0.0, 0.0}, 0.0);
  trail.append(Point2{1.0, 0.0}, 0.0);
  const double last_confirmed_station = trail.end_station();
  trail.append(Point2{2.0, 0.8}, 0.0);
  trail.append(Point2{3.0, 1.6}, 0.0);
  trail.truncate_after(last_confirmed_station);
  ASSERT_EQ(trail.size(), 2U);
  EXPECT_NEAR(trail.points().back().position.x, 1.0, 1e-9);
  EXPECT_NEAR(trail.points().back().position.y, 0.0, 1e-9);
}

TEST(LeaderController, UsesLeaderSpeedFeedForwardAtTheRequiredGap)
{
  LeaderTrail trail;
  trail.seed(Point2{0.0, 0.0}, Point2{4.0, 0.0}, 0.1);
  TrailControlLimits limits;
  limits.desired_distance = 2.0;
  limits.maximum_linear = 1.2;
  const auto command = trail_command(
    trail, Point2{2.0, 0.0}, 0.0, 0.8, 0.8, limits);
  ASSERT_TRUE(command.valid);
  EXPECT_NEAR(command.gap, 2.0, 1e-9);
  EXPECT_NEAR(command.linear, 0.8, 1e-9);
  EXPECT_NEAR(command.angular, 0.0, 1e-9);
}

TEST(LeaderController, ReservesSpeedAboveOneMeterPerSecondLeader)
{
  LeaderTrail trail;
  trail.seed(Point2{0.0, 0.0}, Point2{4.8, 0.0}, 0.1);
  TrailControlLimits limits;
  limits.desired_distance = 2.0;
  limits.maximum_linear = 1.2;
  const auto command = trail_command(
    trail, Point2{2.0, 0.0}, 0.0, 1.0, 1.0, limits);
  ASSERT_TRUE(command.valid);
  EXPECT_NEAR(command.linear, 1.2, 1e-9);
}

TEST(LeaderController, SlowsAndTurnsAlongTheLeaderUTurnTrail)
{
  LeaderTrail trail;
  trail.append(Point2{0.0, 0.0}, 0.0);
  trail.append(Point2{1.0, 0.0}, 0.0);
  trail.append(Point2{2.0, 0.0}, 0.0);
  trail.append(Point2{2.7, 0.3}, 0.0);
  trail.append(Point2{3.0, 1.0}, 0.0);
  trail.append(Point2{2.7, 1.7}, 0.0);
  trail.append(Point2{2.0, 2.0}, 0.0);
  TrailControlLimits limits;
  limits.desired_distance = 1.0;
  limits.maximum_linear = 1.0;
  limits.minimum_lookahead = 0.6;
  limits.maximum_lookahead = 0.6;
  const auto command = trail_command(
    trail, Point2{2.2, 0.05}, 0.0, 1.0, 0.8, limits);
  ASSERT_TRUE(command.valid);
  EXPECT_TRUE(command.curve_mode);
  EXPECT_GT(command.angular, 0.0);
  EXPECT_LT(command.linear, 1.0);
}

TEST(LeaderController, CapsSpeedAndKeepsTurningThroughTwoMeterRadiusTurn)
{
  LeaderTrail trail;
  for (int index = 0; index <= 60; ++index) {
    const double angle = 1.5 * static_cast<double>(index) / 60.0;
    trail.append(Point2{2.0 * std::sin(angle), 2.0 * (1.0 - std::cos(angle))}, 0.0);
  }
  TrailControlLimits limits;
  limits.desired_distance = 2.0;
  limits.maximum_linear = 1.2;
  const auto command = trail_command(
    trail, Point2{0.0, 0.0}, 0.0, 1.0, 1.0, limits);
  ASSERT_TRUE(command.valid);
  EXPECT_TRUE(command.curve_mode);
  EXPECT_GT(command.linear, 0.0);
  EXPECT_LE(command.linear, limits.maximum_turn_linear);
  EXPECT_GT(command.angular, 0.0);
  EXPECT_LE(command.angular, limits.maximum_angular);
}

TEST(LeaderController, StraightDeadbandRejectsSmallCentroidAndHeadingNoise)
{
  LeaderTrail trail;
  trail.seed(Point2{0.0, 0.0}, Point2{5.0, 0.0}, 0.1);
  TrailControlLimits limits;
  const auto command = trail_command(
    trail, Point2{1.0, 0.03}, 2.0 * M_PI / 180.0, 0.6, 0.6, limits);
  ASSERT_TRUE(command.valid);
  EXPECT_FALSE(command.curve_mode);
  EXPECT_DOUBLE_EQ(command.angular, 0.0);
}

TEST(LeaderController, StraightCorrectionIsBoundedAndSteersTowardThePath)
{
  LeaderTrail trail;
  trail.seed(Point2{0.0, 0.0}, Point2{5.0, 0.0}, 0.1);
  TrailControlLimits limits;
  const auto command = trail_command(
    trail, Point2{1.0, 0.25}, 0.0, 0.6, 0.6, limits);
  ASSERT_TRUE(command.valid);
  EXPECT_FALSE(command.curve_mode);
  EXPECT_LT(command.angular, 0.0);
  EXPECT_LE(std::abs(command.angular), limits.straight_maximum_angular);
}

TEST(LeaderController, CurvatureHysteresisDoesNotChatterAtTheBoundary)
{
  LeaderTrail trail;
  const double radius = 8.0;
  for (int index = 0; index <= 60; ++index) {
    const double angle = 0.30 * static_cast<double>(index) / 60.0;
    trail.append(Point2{radius * std::sin(angle), radius * (1.0 - std::cos(angle))}, 0.0);
  }
  TrailControlLimits limits;
  const auto stays_straight = trail_command(
    trail, Point2{0.0, 0.0}, 0.0, 0.6, 0.6, limits,
    -std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(), false);
  const auto stays_turning = trail_command(
    trail, Point2{0.0, 0.0}, 0.0, 0.6, 0.6, limits,
    -std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(), true);
  ASSERT_TRUE(stays_straight.valid);
  ASSERT_TRUE(stays_turning.valid);
  EXPECT_FALSE(stays_straight.curve_mode);
  EXPECT_TRUE(stays_turning.curve_mode);
}

TEST(LeaderController, RotatesWithoutDrivingWhenTrailIsLateral)
{
  LeaderTrail trail;
  trail.append(Point2{0.0, 0.0}, 0.0);
  trail.append(Point2{0.0, 1.0}, 0.0);
  trail.append(Point2{0.0, 2.0}, 0.0);
  TrailControlLimits limits;
  limits.desired_distance = 0.5;
  limits.rotate_to_heading_angle = 0.8;
  const auto command = trail_command(
    trail, Point2{0.0, 0.2}, 0.0, 0.5, 0.2, limits);
  ASSERT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.linear, 0.0);
  EXPECT_GE(command.angular, limits.minimum_rotate_angular);
}
