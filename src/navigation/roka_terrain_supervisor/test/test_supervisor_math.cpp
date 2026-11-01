#include <gtest/gtest.h>

#include <limits>

#include "roka_terrain_supervisor/supervisor_math.hpp"

TEST(SupervisorMath, LeavesFirstEightyPercentOpen)
{
  EXPECT_DOUBLE_EQ(roka_terrain_supervisor::speedScaleFromRisk(0.80), 1.0);
  EXPECT_NEAR(roka_terrain_supervisor::speedScaleFromRisk(0.85), 0.9, 1e-9);
}

TEST(SupervisorMath, StopsAtHardLimitAndInvalidInput)
{
  EXPECT_DOUBLE_EQ(roka_terrain_supervisor::speedScaleFromRisk(1.0), 0.0);
  EXPECT_DOUBLE_EQ(
    roka_terrain_supervisor::speedScaleFromRisk(
      std::numeric_limits<double>::quiet_NaN()), 0.0);
}

TEST(SupervisorMath, ClampsFiniteValues)
{
  EXPECT_DOUBLE_EQ(roka_terrain_supervisor::clampFinite(2.0, 0.6), 0.6);
  EXPECT_DOUBLE_EQ(roka_terrain_supervisor::clampFinite(-2.0, 0.6), -0.6);
}

TEST(SupervisorMath, LevelFallbackIsExplicitAndFailClosedByDefault)
{
  EXPECT_DOUBLE_EQ(
    roka_terrain_supervisor::effectiveTerrainRisk(false, false, 0.2), 1.0);
  EXPECT_DOUBLE_EQ(
    roka_terrain_supervisor::effectiveTerrainRisk(false, true, 0.2), 0.0);
  EXPECT_DOUBLE_EQ(
    roka_terrain_supervisor::effectiveTerrainRisk(true, true, 0.6), 0.6);
}

TEST(SupervisorMath, ProjectsOnlyAnOutwardAttitudeRate)
{
  EXPECT_NEAR(
    roka_terrain_supervisor::projectedAbsoluteAngleDeg(
      10.0, 10.0 * M_PI / 180.0, 0.2),
    12.0, 1e-9);
  EXPECT_NEAR(
    roka_terrain_supervisor::projectedAbsoluteAngleDeg(
      10.0, -10.0 * M_PI / 180.0, 0.2),
    10.0, 1e-9);
}

TEST(SupervisorMath, AttitudeEnvelopeDeratesBeforeHardStop)
{
  EXPECT_DOUBLE_EQ(
    roka_terrain_supervisor::attitudeScale(10.0, 10.5, 26.0, 41.0), 1.0);
  EXPECT_NEAR(
    roka_terrain_supervisor::attitudeScale(26.0, 10.5, 26.0, 41.0), 0.75, 1e-9);
  EXPECT_DOUBLE_EQ(
    roka_terrain_supervisor::attitudeScale(41.0, 10.5, 26.0, 41.0), 0.0);
}

TEST(SupervisorMath, ContactAwareSlewLimitDoesNotCreateAStep)
{
  EXPECT_NEAR(
    roka_terrain_supervisor::slewLimited(0.30, 0.60, 0.10, 0.15, 0.10),
    0.31, 1e-9);
  EXPECT_NEAR(
    roka_terrain_supervisor::slewLimited(0.30, 0.0, 0.10, 0.15, 0.10),
    0.285, 1e-9);
}
