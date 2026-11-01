#include <gtest/gtest.h>

#include "roka_terrain_mapping/terrain_math.hpp"

using roka_terrain_mapping::TraversabilityInput;
using roka_terrain_mapping::TraversabilityLimits;

TEST(TerrainMath, FlatTerrainIsTraversable)
{
  TraversabilityInput input;
  input.confidence = 1.0;
  const auto risk = roka_terrain_mapping::normalizedRisk(input, TraversabilityLimits{});
  EXPECT_DOUBLE_EQ(risk, 0.0);
  EXPECT_DOUBLE_EQ(roka_terrain_mapping::traversabilityFromRisk(risk), 1.0);
}

TEST(TerrainMath, HardLimitProducesFullRisk)
{
  TraversabilityInput input;
  input.slope_deg = 35.0;
  input.confidence = 1.0;
  const auto risk = roka_terrain_mapping::normalizedRisk(input, TraversabilityLimits{});
  EXPECT_NEAR(risk, 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(roka_terrain_mapping::traversabilityFromRisk(risk), 0.0);
}

TEST(TerrainMath, WorstFeatureDominates)
{
  TraversabilityInput input;
  input.slope_deg = 7.0;
  input.step_m = 0.18;
  input.confidence = 1.0;
  const auto risk = roka_terrain_mapping::normalizedRisk(input, TraversabilityLimits{});
  EXPECT_NEAR(risk, 0.9, 1e-9);
}
