#include "mouse_odometry/esp_ros_time_synchronizer.hpp"

#include <limits>
#include <stdexcept>

namespace mouse_odometry
{
namespace
{

std::optional<int64_t> microsecondsToNanoseconds(const uint64_t microseconds)
{
  constexpr uint64_t kMaximumMicroseconds =
    static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) / 1000U;
  if (microseconds > kMaximumMicroseconds) {
    return std::nullopt;
  }
  return static_cast<int64_t>(microseconds * 1000U);
}

}  // namespace

EspRosTimeSynchronizer::EspRosTimeSynchronizer(
  const std::size_t required_sample_count)
{
  setRequiredSampleCount(required_sample_count);
}

bool EspRosTimeSynchronizer::observe(
  const uint64_t esp_timestamp_us, const int64_t ros_receive_time_ns)
{
  if (ros_receive_time_ns <= 0) {
    return false;
  }

  const auto esp_timestamp_ns = microsecondsToNanoseconds(esp_timestamp_us);
  if (!esp_timestamp_ns.has_value()) {
    reset();
    return false;
  }

  if ((valid_ || collected_sample_count_ != 0U) &&
    ros_receive_time_ns < previous_ros_receive_time_ns_)
  {
    reset();
  }

  if (has_previous_esp_timestamp_ && esp_timestamp_us < previous_esp_timestamp_us_) {
    reset();
  }

  if (!valid_) {
    const int64_t candidate_offset_ns = ros_receive_time_ns - *esp_timestamp_ns;
    if (!has_candidate_offset_ || candidate_offset_ns < minimum_candidate_offset_ns_) {
      minimum_candidate_offset_ns_ = candidate_offset_ns;
      has_candidate_offset_ = true;
    }
    ++collected_sample_count_;
    if (collected_sample_count_ >= required_sample_count_) {
      offset_ns_ = minimum_candidate_offset_ns_;
      valid_ = true;
    }
  }
  previous_ros_receive_time_ns_ = ros_receive_time_ns;
  previous_esp_timestamp_us_ = esp_timestamp_us;
  has_previous_esp_timestamp_ = true;
  return valid_;
}

std::optional<int64_t> EspRosTimeSynchronizer::toRosTimeNanoseconds(
  const uint64_t esp_timestamp_us) const
{
  if (!valid_) {
    return std::nullopt;
  }
  const auto esp_timestamp_ns = microsecondsToNanoseconds(esp_timestamp_us);
  if (!esp_timestamp_ns.has_value()) {
    return std::nullopt;
  }

  if (offset_ns_ > 0 && *esp_timestamp_ns >
    std::numeric_limits<int64_t>::max() - offset_ns_)
  {
    return std::nullopt;
  }
  if (offset_ns_ < 0 && *esp_timestamp_ns <
    std::numeric_limits<int64_t>::min() - offset_ns_)
  {
    return std::nullopt;
  }
  return *esp_timestamp_ns + offset_ns_;
}

void EspRosTimeSynchronizer::reset()
{
  valid_ = false;
  collected_sample_count_ = 0;
  has_candidate_offset_ = false;
  has_previous_esp_timestamp_ = false;
  offset_ns_ = 0;
  minimum_candidate_offset_ns_ = 0;
  previous_ros_receive_time_ns_ = 0;
  previous_esp_timestamp_us_ = 0;
}

void EspRosTimeSynchronizer::setRequiredSampleCount(
  const std::size_t required_sample_count)
{
  if (required_sample_count == 0U) {
    throw std::invalid_argument("clock synchronization sample count must be > 0");
  }
  required_sample_count_ = required_sample_count;
  reset();
}

double EspRosTimeSynchronizer::offsetSeconds() const
{
  return static_cast<double>(offset_ns_) * 1.0e-9;
}

}  // namespace mouse_odometry
