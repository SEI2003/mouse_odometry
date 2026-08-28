#include <gtest/gtest.h>

#include "mouse_odometry/flow_validation.hpp"

#include <limits>

namespace
{

mouse_odometry::FlowObservation validObservation()
{
  mouse_odometry::FlowObservation observation;
  observation.raw_dx = 100;
  observation.raw_dy = 0;
  observation.converted_dx = 0.100;
  observation.converted_dy = 0.0;
  observation.measurement_end_time_sec = 10.0;
  observation.receive_time_sec = 10.001;
  observation.integration_time_sec = 0.010;
  observation.quality = 50;
  observation.quality_available = true;
  observation.source_valid = true;
  observation.timestamp_valid = true;
  return observation;
}

mouse_odometry::ValidationLimits limits()
{
  mouse_odometry::ValidationLimits result;
  result.sensor_timeout_sec = 0.1;
  result.max_sensor_time_difference_sec = 0.02;
  result.max_integration_time_difference_sec = 0.005;
  result.minimum_quality = 20;
  result.max_abs_raw_delta_x = 1000;
  result.max_abs_raw_delta_y = 1000;
  return result;
}

TEST(FlowValidation, TestGRejectsIntegrationTimeMismatch)
{
  auto left = validObservation();
  auto right = validObservation();
  left.integration_time_sec = 0.010;
  right.integration_time_sec = 0.020;

  const auto result = mouse_odometry::validateFlowPair(left, right, 10.002, limits());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.reject_reason,
    mouse_odometry::RejectReason::kIntegrationTimeMismatch);
}

TEST(FlowValidation, TestHRejectsStaleTimestampPair)
{
  auto left = validObservation();
  auto right = validObservation();
  right.measurement_end_time_sec = left.measurement_end_time_sec - 0.1;

  const auto result = mouse_odometry::validateFlowPair(left, right, 10.002, limits());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, mouse_odometry::RejectReason::kTimeMismatch);
}

TEST(FlowValidation, TestIRejectsLowTrackingQuality)
{
  auto left = validObservation();
  auto right = validObservation();
  left.quality = 19;

  const auto result = mouse_odometry::validateFlowPair(left, right, 10.002, limits());
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.left.valid);
  EXPECT_TRUE(result.right.valid);
  EXPECT_EQ(result.left.reject_reason, mouse_odometry::RejectReason::kLowQuality);
}

TEST(FlowValidation, RejectsRawDeltaOutlierBeforeEstimation)
{
  auto left = validObservation();
  auto right = validObservation();
  left.raw_dx = 1001;

  const auto result = mouse_odometry::validateFlowPair(left, right, 10.002, limits());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.left.reject_reason, mouse_odometry::RejectReason::kRawDeltaOutlier);
}

TEST(FlowValidation, RejectsSensorTimeout)
{
  auto left = validObservation();
  auto right = validObservation();
  left.receive_time_sec = 9.8;

  const auto result = mouse_odometry::validateFlowPair(left, right, 10.002, limits());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.left.reject_reason, mouse_odometry::RejectReason::kTimeout);
}

TEST(FlowValidation, RejectsInvalidTimestamp)
{
  auto left = validObservation();
  auto right = validObservation();
  left.timestamp_valid = false;

  const auto result = mouse_odometry::validateFlowPair(left, right, 10.002, limits());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.left.reject_reason,
    mouse_odometry::RejectReason::kInvalidTimestamp);
}

TEST(FlowValidation, RejectsNonPositiveIntegrationTime)
{
  auto left = validObservation();
  auto right = validObservation();
  left.integration_time_sec = 0.0;

  const auto result = mouse_odometry::validateFlowPair(left, right, 10.002, limits());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.left.reject_reason,
    mouse_odometry::RejectReason::kInvalidIntegrationTime);
}

TEST(FlowValidation, RejectsNonfiniteConvertedDelta)
{
  auto left = validObservation();
  auto right = validObservation();
  left.converted_dx = std::numeric_limits<double>::quiet_NaN();

  const auto result = mouse_odometry::validateFlowPair(left, right, 10.002, limits());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.left.reject_reason,
    mouse_odometry::RejectReason::kNonfiniteValue);
}

TEST(FlowValidation, RejectsSourceInvalid)
{
  auto left = validObservation();
  auto right = validObservation();
  left.source_valid = false;

  const auto result = mouse_odometry::validateFlowPair(left, right, 10.002, limits());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.left.reject_reason, mouse_odometry::RejectReason::kSourceInvalid);
}

}  // namespace
