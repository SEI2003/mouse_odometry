#include <gtest/gtest.h>

#include "mouse_odometry/planar_motion_estimator.hpp"

#include <stdexcept>

namespace
{

constexpr double kTolerance = 1.0e-12;

mouse_odometry::PlanarMotionEstimator makeEstimator()
{
  return mouse_odometry::PlanarMotionEstimator(
    {0.220, 0.055}, {0.220, -0.055});
}

void expectMotion(
  const mouse_odometry::MotionEstimate & estimate,
  double dx, double dy, double dyaw)
{
  EXPECT_NEAR(estimate.dx, dx, kTolerance);
  EXPECT_NEAR(estimate.dy, dy, kTolerance);
  EXPECT_NEAR(estimate.dyaw, dyaw, kTolerance);
  EXPECT_NEAR(estimate.residual, 0.0, kTolerance);
}

TEST(PlanarMotionEstimator, TestAForward)
{
  const auto estimate = makeEstimator().estimate({0.100, 0.0}, {0.100, 0.0});
  expectMotion(estimate, 0.100, 0.0, 0.0);
}

TEST(PlanarMotionEstimator, TestBLateral)
{
  const auto estimate = makeEstimator().estimate({0.0, 0.100}, {0.0, 0.100});
  expectMotion(estimate, 0.0, 0.100, 0.0);
}

TEST(PlanarMotionEstimator, TestCPureYawRemovesApparentLateralMotion)
{
  const auto estimate = makeEstimator().estimate(
    {-0.0055, 0.022}, {0.0055, 0.022});
  expectMotion(estimate, 0.0, 0.0, 0.100);
}

TEST(PlanarMotionEstimator, TestDCombinedMotion)
{
  constexpr double dx = 0.080;
  constexpr double dy = 0.020;
  constexpr double dyaw = 0.050;

  const mouse_odometry::SensorDelta left{
    dx - 0.055 * dyaw,
    dy + 0.220 * dyaw};
  const mouse_odometry::SensorDelta right{
    dx - (-0.055) * dyaw,
    dy + 0.220 * dyaw};

  expectMotion(makeEstimator().estimate(left, right), dx, dy, dyaw);
}

TEST(PlanarMotionEstimator, ResidualDetectsInconsistentObservation)
{
  const auto estimate = makeEstimator().estimate({0.100, 0.010}, {0.100, -0.010});
  EXPECT_GT(estimate.residual, 0.0);
}

TEST(PlanarMotionEstimator, RejectsDegenerateCoincidentGeometry)
{
  EXPECT_THROW(
    mouse_odometry::PlanarMotionEstimator({0.220, 0.055}, {0.220, 0.055}),
    std::invalid_argument);
}

}  // namespace
