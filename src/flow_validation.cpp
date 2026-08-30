#include "mouse_odometry/flow_validation.hpp"

#include <cmath>
#include <cstdlib>

namespace mouse_odometry
{

const char * rejectReasonName(RejectReason reason)
{
  switch (reason) {
    case RejectReason::kNone:
      return "ok";
    case RejectReason::kTimeout:
      return "timeout";
    case RejectReason::kLowQuality:
      return "low_quality";
    case RejectReason::kInvalidTimestamp:
      return "invalid_timestamp";
    case RejectReason::kTimeMismatch:
      return "timestamp_mismatch";
    case RejectReason::kInvalidIntegrationTime:
      return "invalid_integration_time";
    case RejectReason::kIntegrationTimeMismatch:
      return "integration_time_mismatch";
    case RejectReason::kNonfiniteValue:
      return "nonfinite_value";
    case RejectReason::kRawDeltaOutlier:
      return "raw_delta_outlier";
    case RejectReason::kVelocityOutlier:
      return "velocity_outlier";
    case RejectReason::kResidualOutlier:
      return "motion_residual_outlier";
    case RejectReason::kSourceInvalid:
      return "source_invalid";
    case RejectReason::kCycleMismatch:
      return "cycle_mismatch";
    default:
      return "unknown";
  }
}

bool cycleIsOlder(uint32_t lhs, uint32_t rhs)
{
  constexpr uint32_t kHalfCycleRange = uint32_t{1} << 31U;
  const uint32_t forward_distance = rhs - lhs;
  return forward_distance != 0U && forward_distance < kHalfCycleRange;
}

namespace
{

SensorValidity validateSensor(
  const FlowObservation & observation,
  double current_time_sec,
  const ValidationLimits & limits)
{
  SensorValidity result;
  result.timestamp_valid = observation.timestamp_valid &&
    std::isfinite(observation.measurement_end_time_sec);
  result.integration_time_valid = std::isfinite(observation.integration_time_sec) &&
    observation.integration_time_sec > 0.0;
  result.timeout_valid = std::isfinite(observation.receive_time_sec) &&
    std::isfinite(current_time_sec) &&
    current_time_sec >= observation.receive_time_sec &&
    current_time_sec - observation.receive_time_sec <= limits.sensor_timeout_sec;
  result.quality_valid = !observation.quality_available ||
    observation.quality >= limits.minimum_quality;
  result.numeric_valid = std::isfinite(observation.converted_dx) &&
    std::isfinite(observation.converted_dy);
  result.raw_delta_range_valid =
    std::abs(static_cast<int64_t>(observation.raw_dx)) <= limits.max_abs_raw_delta_x &&
    std::abs(static_cast<int64_t>(observation.raw_dy)) <= limits.max_abs_raw_delta_y;
  result.source_valid = observation.source_valid;

  if (!result.timeout_valid) {
    result.reject_reason = RejectReason::kTimeout;
  } else if (!result.source_valid) {
    result.reject_reason = RejectReason::kSourceInvalid;
  } else if (!result.timestamp_valid) {
    result.reject_reason = RejectReason::kInvalidTimestamp;
  } else if (!result.integration_time_valid) {
    result.reject_reason = RejectReason::kInvalidIntegrationTime;
  } else if (!result.quality_valid) {
    result.reject_reason = RejectReason::kLowQuality;
  } else if (!result.numeric_valid) {
    result.reject_reason = RejectReason::kNonfiniteValue;
  } else if (!result.raw_delta_range_valid) {
    result.reject_reason = RejectReason::kRawDeltaOutlier;
  }

  result.valid = result.reject_reason == RejectReason::kNone;
  return result;
}

}  // namespace

PairValidity validateFlowPair(
  const FlowObservation & left,
  const FlowObservation & right,
  double current_time_sec,
  const ValidationLimits & limits)
{
  PairValidity result;
  result.left = validateSensor(left, current_time_sec, limits);
  result.right = validateSensor(right, current_time_sec, limits);
  result.measurement_time_difference_sec = std::abs(
    left.measurement_end_time_sec - right.measurement_end_time_sec);
  result.integration_time_difference_sec = std::abs(
    left.integration_time_sec - right.integration_time_sec);
  result.cycles_match = left.cycle_id == right.cycle_id;
  result.timestamps_match = std::isfinite(result.measurement_time_difference_sec) &&
    result.measurement_time_difference_sec <= limits.max_sensor_time_difference_sec;
  result.integration_times_match =
    std::isfinite(result.integration_time_difference_sec) &&
    result.integration_time_difference_sec <=
    limits.max_integration_time_difference_sec;

  if (!result.left.valid) {
    result.reject_reason = result.left.reject_reason;
  } else if (!result.right.valid) {
    result.reject_reason = result.right.reject_reason;
  } else if (!result.cycles_match) {
    result.reject_reason = RejectReason::kCycleMismatch;
  } else if (!result.timestamps_match) {
    result.reject_reason = RejectReason::kTimeMismatch;
  } else if (!result.integration_times_match) {
    result.reject_reason = RejectReason::kIntegrationTimeMismatch;
  }

  result.valid = result.reject_reason == RejectReason::kNone;
  return result;
}

}  // namespace mouse_odometry
