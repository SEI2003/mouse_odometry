#ifndef MOUSE_ODOMETRY__ESP_ROS_TIME_SYNCHRONIZER_HPP_
#define MOUSE_ODOMETRY__ESP_ROS_TIME_SYNCHRONIZER_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>

namespace mouse_odometry
{

class EspRosTimeSynchronizer
{
public:
  explicit EspRosTimeSynchronizer(std::size_t required_sample_count = 1);

  // During the initial window, selects the minimum of
  // ros_receive_time - esp_measurement_time to reduce positive USB delay.
  // A backward ESP or ROS clock jump starts a new synchronization epoch.
  bool observe(uint64_t esp_timestamp_us, int64_t ros_receive_time_ns);
  std::optional<int64_t> toRosTimeNanoseconds(uint64_t esp_timestamp_us) const;
  void reset();
  void setRequiredSampleCount(std::size_t required_sample_count);

  bool valid() const {return valid_;}
  int64_t offsetNanoseconds() const {return offset_ns_;}
  double offsetSeconds() const;
  std::size_t requiredSampleCount() const {return required_sample_count_;}
  std::size_t collectedSampleCount() const {return collected_sample_count_;}

private:
  std::size_t required_sample_count_{1};
  std::size_t collected_sample_count_{0};
  bool valid_{false};
  bool has_candidate_offset_{false};
  bool has_previous_esp_timestamp_{false};
  int64_t offset_ns_{0};
  int64_t minimum_candidate_offset_ns_{0};
  int64_t previous_ros_receive_time_ns_{0};
  uint64_t previous_esp_timestamp_us_{0};
};

}  // namespace mouse_odometry

#endif  // MOUSE_ODOMETRY__ESP_ROS_TIME_SYNCHRONIZER_HPP_
