#ifndef MOUSE_ODOMETRY__FLOW_VALIDATION_HPP_
#define MOUSE_ODOMETRY__FLOW_VALIDATION_HPP_

#include <cstdint>

namespace mouse_odometry
{

enum class RejectReason : uint8_t
{
  kNone = 0,
  kTimeout = 1,
  kLowQuality = 2,
  kInvalidTimestamp = 3,
  kTimeMismatch = 4,
  kInvalidIntegrationTime = 5,
  kIntegrationTimeMismatch = 6,
  kNonfiniteValue = 7,
  kRawDeltaOutlier = 8,
  kVelocityOutlier = 9,
  kResidualOutlier = 10,
  kSourceInvalid = 11,
  kCycleMismatch = 12,
};

const char * rejectReasonName(RejectReason reason);

// uint32_tの循環シーケンス上でlhsがrhsより古いかを判定する。
bool cycleIsOlder(uint32_t lhs, uint32_t rhs);

struct FlowObservation
{
  uint32_t cycle_id{0};
  int32_t raw_dx{0};
  int32_t raw_dy{0};
  double converted_dx{0.0};
  double converted_dy{0.0};
  double measurement_end_time_sec{0.0};
  double receive_time_sec{0.0};
  double integration_time_sec{0.0};
  uint16_t quality{0};
  bool quality_available{false};
  bool source_valid{false};
  bool timestamp_valid{false};
};

struct ValidationLimits
{
  double sensor_timeout_sec{0.1};
  double max_sensor_time_difference_sec{0.02};
  double max_integration_time_difference_sec{0.02};
  uint16_t minimum_quality{0};
  int64_t max_abs_raw_delta_x{32767};
  int64_t max_abs_raw_delta_y{32767};
};

struct SensorValidity
{
  bool timestamp_valid{false};
  bool integration_time_valid{false};
  bool timeout_valid{false};
  bool quality_valid{false};
  bool numeric_valid{false};
  bool raw_delta_range_valid{false};
  bool source_valid{false};
  bool valid{false};
  RejectReason reject_reason{RejectReason::kNone};
};

struct PairValidity
{
  SensorValidity left;
  SensorValidity right;
  bool cycles_match{false};
  bool timestamps_match{false};
  bool integration_times_match{false};
  bool valid{false};
  double measurement_time_difference_sec{0.0};
  double integration_time_difference_sec{0.0};
  RejectReason reject_reason{RejectReason::kNone};
};

PairValidity validateFlowPair(
  const FlowObservation & left,
  const FlowObservation & right,
  double current_time_sec,
  const ValidationLimits & limits);

}  // namespace mouse_odometry

#endif  // MOUSE_ODOMETRY__FLOW_VALIDATION_HPP_
