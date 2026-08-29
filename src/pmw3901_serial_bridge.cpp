#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <memory>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "mouse_odometry/esp_ros_time_synchronizer.hpp"
#include "mouse_odometry/msg/pmw3901_flow.hpp"
#include "mouse_odometry/msg/pmw3901_serial_debug.hpp"
#include "mouse_odometry/pmw3901_serial_protocol.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mouse_odometry
{
namespace
{

std::optional<speed_t> baudRateConstant(const int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
      return std::nullopt;
  }
}

builtin_interfaces::msg::Duration durationFromMicroseconds(
  const uint32_t microseconds)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(microseconds / 1000000U);
  duration.nanosec = (microseconds % 1000000U) * 1000U;
  return duration;
}

}  // namespace

class Pmw3901SerialBridge : public rclcpp::Node
{
public:
  Pmw3901SerialBridge()
  : Node("pmw3901_serial_bridge")
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    serial_baud_ = declare_parameter<int>("serial_baud", 115200);
    reconnect_interval_sec_ = declare_parameter<double>("reconnect_interval_sec", 1.0);
    io_poll_period_ms_ = declare_parameter<int>("io_poll_period_ms", 2);
    debug_publish_period_sec_ = declare_parameter<double>("debug_publish_period_sec", 1.0);
    clock_sync_sample_count_ = declare_parameter<int>("clock_sync_sample_count", 20);
    left_topic_ = declare_parameter<std::string>(
      "left_flow_topic", "/pmw3901/left/flow");
    right_topic_ = declare_parameter<std::string>(
      "right_flow_topic", "/pmw3901/right/flow");
    left_frame_id_ = declare_parameter<std::string>("left_frame_id", "pmw3901_left");
    right_frame_id_ = declare_parameter<std::string>("right_frame_id", "pmw3901_right");

    validateParameters();
    time_synchronizer_.setRequiredSampleCount(
      static_cast<std::size_t>(clock_sync_sample_count_));

    left_publisher_ = create_publisher<msg::Pmw3901Flow>(
      left_topic_, rclcpp::SensorDataQoS());
    right_publisher_ = create_publisher<msg::Pmw3901Flow>(
      right_topic_, rclcpp::SensorDataQoS());
    debug_publisher_ = create_publisher<msg::Pmw3901SerialDebug>(
      "/pmw3901/serial_debug", 10);

    io_timer_ = create_wall_timer(
      std::chrono::milliseconds(io_poll_period_ms_),
      [this]() {pollSerial();});
    debug_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(debug_publish_period_sec_)),
      [this]() {publishDebug();});

    next_reconnect_attempt_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(),
      "PMW3901 Serial Bridge ready: port=%s baud=%d left=%s right=%s",
      serial_port_.c_str(), serial_baud_, left_topic_.c_str(), right_topic_.c_str());
  }

  ~Pmw3901SerialBridge() override
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  void validateParameters() const
  {
    if (serial_port_.empty()) {
      throw std::invalid_argument("serial_port must not be empty");
    }
    if (!baudRateConstant(serial_baud_).has_value()) {
      throw std::invalid_argument("serial_baud is not supported by termios");
    }
    if (!std::isfinite(reconnect_interval_sec_) || reconnect_interval_sec_ <= 0.0) {
      throw std::invalid_argument("reconnect_interval_sec must be finite and > 0");
    }
    if (io_poll_period_ms_ <= 0) {
      throw std::invalid_argument("io_poll_period_ms must be > 0");
    }
    if (!std::isfinite(debug_publish_period_sec_) || debug_publish_period_sec_ <= 0.0) {
      throw std::invalid_argument("debug_publish_period_sec must be finite and > 0");
    }
    if (clock_sync_sample_count_ <= 0 || clock_sync_sample_count_ > 1000) {
      throw std::invalid_argument("clock_sync_sample_count must be in [1, 1000]");
    }
  }

  void scheduleReconnect()
  {
    next_reconnect_attempt_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(reconnect_interval_sec_));
  }

  void tryOpenSerial()
  {
    if (std::chrono::steady_clock::now() < next_reconnect_attempt_) {
      return;
    }

    const int candidate_fd = open(
      serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (candidate_fd < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unable to open %s: %s; retrying",
        serial_port_.c_str(), std::strerror(errno));
      scheduleReconnect();
      return;
    }

    termios configuration{};
    if (tcgetattr(candidate_fd, &configuration) != 0) {
      const std::string error = std::strerror(errno);
      close(candidate_fd);
      RCLCPP_WARN(get_logger(), "tcgetattr(%s) failed: %s", serial_port_.c_str(), error.c_str());
      scheduleReconnect();
      return;
    }

    cfmakeraw(&configuration);
    configuration.c_cflag |= CLOCAL | CREAD;
    configuration.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CRTSCTS | CSIZE));
    configuration.c_cflag |= CS8;
    configuration.c_cc[VMIN] = 0;
    configuration.c_cc[VTIME] = 0;
    const speed_t baud_constant = *baudRateConstant(serial_baud_);
    const bool speed_failed = cfsetispeed(&configuration, baud_constant) != 0 ||
      cfsetospeed(&configuration, baud_constant) != 0;
    if (speed_failed || tcsetattr(candidate_fd, TCSANOW, &configuration) != 0) {
      const std::string error = std::strerror(errno);
      close(candidate_fd);
      RCLCPP_WARN(
        get_logger(), "Serial configuration for %s failed: %s",
        serial_port_.c_str(), error.c_str());
      scheduleReconnect();
      return;
    }

    tcflush(candidate_fd, TCIFLUSH);
    serial_fd_ = candidate_fd;
    parser_.clearBufferedData();
    sequence_tracker_.reset();
    time_synchronizer_.reset();
    if (has_connected_once_) {
      ++serial_reconnect_count_;
    }
    has_connected_once_ = true;
    RCLCPP_INFO(get_logger(), "Connected to PMW3901 serial device %s", serial_port_.c_str());
  }

  void disconnectSerial(const char * reason)
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
    parser_.clearBufferedData();
    sequence_tracker_.reset();
    time_synchronizer_.reset();
    scheduleReconnect();
    RCLCPP_WARN(get_logger(), "PMW3901 serial disconnected: %s", reason);
  }

  void pollSerial()
  {
    if (serial_fd_ < 0) {
      tryOpenSerial();
      return;
    }

    pollfd descriptor{};
    descriptor.fd = serial_fd_;
    descriptor.events = POLLIN;
    const int poll_result = poll(&descriptor, 1, 0);
    if (poll_result < 0) {
      if (errno != EINTR) {
        disconnectSerial(std::strerror(errno));
      }
      return;
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      disconnectSerial("poll reported device error or hangup");
      return;
    }
    if (poll_result == 0 || (descriptor.revents & POLLIN) == 0) {
      return;
    }

    std::array<uint8_t, 4096> bytes{};
    for (int read_count = 0; read_count < 8; ++read_count) {
      const ssize_t received = read(serial_fd_, bytes.data(), bytes.size());
      if (received > 0) {
        const rclcpp::Time receive_time = now();
        const auto packets = parser_.push(
          bytes.data(), static_cast<std::size_t>(received));
        for (const auto & packet : packets) {
          processPacket(packet, receive_time);
        }
        continue;
      }
      if (received == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      if (errno == EINTR) {
        continue;
      }
      disconnectSerial(std::strerror(errno));
      break;
    }
  }

  void processPacket(
    const Pmw3901PacketV1 & packet, const rclcpp::Time & receive_time)
  {
    current_cycle_id_ = packet.cycle_id;
    current_esp_timestamp_us_ = packet.timestamp_us;
    current_measurement_interval_us_ = packet.measurement_interval_us;
    current_left_status_ = packet.left.status;
    current_right_status_ = packet.right.status;
    last_ros_receive_time_ = receive_time;

    const PacketSequenceUpdate sequence = sequence_tracker_.observe(
      packet.cycle_id, packet.timestamp_us);
    if (sequence.reboot_detected) {
      ++esp_reboot_count_;
      time_synchronizer_.reset();
      RCLCPP_WARN(get_logger(), "ESP32 timestamp reset detected; clock synchronization reset");
    }
    packet_drop_count_ += sequence.dropped_packets;
    if (!sequence.accepted) {
      if (sequence.duplicate) {
        ++duplicate_packet_count_;
      }
      if (sequence.out_of_order) {
        ++out_of_order_packet_count_;
      }
      return;
    }

    const int64_t receive_time_ns = receive_time.nanoseconds();
    if (!time_synchronizer_.observe(packet.timestamp_us, receive_time_ns)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Collecting clock-sync samples or ROS time is inactive; "
        "suppressing PMW3901 flow publication");
      return;
    }

    const auto measurement_time_ns =
      time_synchronizer_.toRosTimeNanoseconds(packet.timestamp_us);
    constexpr int64_t kMaximumRosTimeNanoseconds =
      static_cast<int64_t>(std::numeric_limits<int32_t>::max()) * 1000000000LL +
      999999999LL;
    if (!measurement_time_ns.has_value() || *measurement_time_ns < 0 ||
      *measurement_time_ns > kMaximumRosTimeNanoseconds)
    {
      time_synchronizer_.reset();
      RCLCPP_ERROR(get_logger(), "Converted PMW3901 measurement timestamp is out of ROS range");
      return;
    }

    const rclcpp::Time measurement_time(
      *measurement_time_ns, get_clock()->get_clock_type());
    last_measurement_ros_time_ = measurement_time;
    transport_delay_estimate_sec_ =
      static_cast<double>(receive_time_ns - *measurement_time_ns) * 1.0e-9;

    const auto integration_time = durationFromMicroseconds(
      packet.measurement_interval_us);
    const bool interval_valid = packet.measurement_interval_us != 0U;

    msg::Pmw3901Flow left_message;
    left_message.header.stamp = last_measurement_ros_time_;
    left_message.header.frame_id = left_frame_id_;
    left_message.cycle_id = packet.cycle_id;
    left_message.delta_x_count = packet.left.dx;
    left_message.delta_y_count = packet.left.dy;
    left_message.integration_time = integration_time;
    left_message.valid = interval_valid &&
      isPmw3901SensorStatusValid(packet.left.status);
    left_message.quality_available =
      (packet.left.status & kSensorStatusSqualAvailable) != 0U;
    left_message.quality = packet.left.squal;

    msg::Pmw3901Flow right_message;
    right_message.header.stamp = last_measurement_ros_time_;
    right_message.header.frame_id = right_frame_id_;
    right_message.cycle_id = packet.cycle_id;
    right_message.delta_x_count = packet.right.dx;
    right_message.delta_y_count = packet.right.dy;
    right_message.integration_time = integration_time;
    right_message.valid = interval_valid &&
      isPmw3901SensorStatusValid(packet.right.status);
    right_message.quality_available =
      (packet.right.status & kSensorStatusSqualAvailable) != 0U;
    right_message.quality = packet.right.squal;

    left_publisher_->publish(left_message);
    right_publisher_->publish(right_message);
  }

  void publishDebug()
  {
    const PacketParserStatistics & parser_statistics = parser_.statistics();
    msg::Pmw3901SerialDebug debug;
    debug.header.stamp = now();
    debug.serial_connected = serial_fd_ >= 0;
    debug.packet_received_total = parser_statistics.packet_received_total;
    debug.packet_valid_total = parser_statistics.packet_valid_total;
    debug.crc_error_count = parser_statistics.crc_error_count;
    debug.header_resync_count = parser_statistics.header_resync_count;
    debug.version_error_count = parser_statistics.version_error_count;
    debug.packet_drop_count = packet_drop_count_;
    debug.duplicate_packet_count = duplicate_packet_count_;
    debug.out_of_order_packet_count = out_of_order_packet_count_;
    debug.serial_reconnect_count = serial_reconnect_count_;
    debug.esp_reboot_count = esp_reboot_count_;
    debug.current_cycle_id = current_cycle_id_;
    debug.current_esp_timestamp_us = current_esp_timestamp_us_;
    debug.measurement_interval_us = current_measurement_interval_us_;
    debug.clock_offset_sec = time_synchronizer_.valid() ?
      time_synchronizer_.offsetSeconds() : 0.0;
    debug.clock_sync_valid = time_synchronizer_.valid();
    debug.clock_sync_samples_collected = static_cast<uint32_t>(
      time_synchronizer_.collectedSampleCount());
    debug.clock_sync_samples_required = static_cast<uint32_t>(
      time_synchronizer_.requiredSampleCount());
    debug.last_ros_receive_time = last_ros_receive_time_;
    debug.last_measurement_ros_time = last_measurement_ros_time_;
    debug.transport_delay_estimate_sec = transport_delay_estimate_sec_;
    debug.left_status = current_left_status_;
    debug.right_status = current_right_status_;
    debug_publisher_->publish(debug);
  }

  std::string serial_port_;
  int serial_baud_{115200};
  double reconnect_interval_sec_{1.0};
  int io_poll_period_ms_{2};
  double debug_publish_period_sec_{1.0};
  int clock_sync_sample_count_{20};
  std::string left_topic_;
  std::string right_topic_;
  std::string left_frame_id_;
  std::string right_frame_id_;

  int serial_fd_{-1};
  bool has_connected_once_{false};
  std::chrono::steady_clock::time_point next_reconnect_attempt_;
  Pmw3901PacketStreamParser parser_;
  EspPacketSequenceTracker sequence_tracker_;
  EspRosTimeSynchronizer time_synchronizer_;

  uint64_t packet_drop_count_{0};
  uint64_t duplicate_packet_count_{0};
  uint64_t out_of_order_packet_count_{0};
  uint64_t serial_reconnect_count_{0};
  uint64_t esp_reboot_count_{0};
  uint32_t current_cycle_id_{0};
  uint64_t current_esp_timestamp_us_{0};
  uint32_t current_measurement_interval_us_{0};
  uint8_t current_left_status_{0};
  uint8_t current_right_status_{0};
  builtin_interfaces::msg::Time last_ros_receive_time_;
  builtin_interfaces::msg::Time last_measurement_ros_time_;
  double transport_delay_estimate_sec_{0.0};

  rclcpp::Publisher<msg::Pmw3901Flow>::SharedPtr left_publisher_;
  rclcpp::Publisher<msg::Pmw3901Flow>::SharedPtr right_publisher_;
  rclcpp::Publisher<msg::Pmw3901SerialDebug>::SharedPtr debug_publisher_;
  rclcpp::TimerBase::SharedPtr io_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
};

}  // namespace mouse_odometry

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mouse_odometry::Pmw3901SerialBridge>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("pmw3901_serial_bridge"),
      "Node initialization failed: %s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
