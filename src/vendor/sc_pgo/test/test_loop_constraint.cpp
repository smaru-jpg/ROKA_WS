#include <cmath>

#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include "sc_pgo/loop_constraint.hpp"

namespace
{

void expectPoseNear(
  const gtsam::Pose3 & actual, const gtsam::Pose3 & expected,
  const double tolerance = 1.0e-9)
{
  const gtsam::Vector6 error = gtsam::Pose3::Logmap(expected.between(actual));
  EXPECT_LT(error.norm(), tolerance);
}

TEST(LoopConstraint, IdentityCorrectionPreservesOdometricRelativePose)
{
  const gtsam::Pose3 previous(
    gtsam::Rot3::RzRyRx(0.02, -0.03, 0.40), gtsam::Point3(2.0, 1.0, 0.2));
  const gtsam::Pose3 current(
    gtsam::Rot3::RzRyRx(-0.01, 0.04, 1.10), gtsam::Point3(4.0, 2.0, 0.5));

  expectPoseNear(
    sc_pgo::correctedRelativePose(previous, current, gtsam::Pose3()),
    previous.between(current));
}

TEST(LoopConstraint, GlobalIcpCorrectionIsAppliedBeforeBuildingBetweenFactor)
{
  const gtsam::Pose3 previous(
    gtsam::Rot3::Rz(0.25), gtsam::Point3(3.0, -1.0, 0.1));
  const gtsam::Pose3 current(
    gtsam::Rot3::Rz(1.00), gtsam::Point3(5.0, 2.0, 0.4));
  const gtsam::Pose3 correction(
    gtsam::Rot3::Rz(-0.10), gtsam::Point3(-0.3, 0.2, -0.05));
  const gtsam::Pose3 expected = previous.between(correction.compose(current));

  const auto actual =
    sc_pgo::correctedRelativePose(previous, current, correction);
  expectPoseNear(actual, expected);
  EXPECT_FALSE(actual.equals(correction.inverse(), 1.0e-6));
}

}  // namespace
