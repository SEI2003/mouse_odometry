#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include "mouse_odometry/msg/pmw3901_debug.hpp"
#include "mouse_odometry/msg/pmw3901_flow.hpp"
#include "mouse_odometry/planar_motion_estimator.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

namespace mouse_odometry
{

class MouseOdomNode : public rclcpp::Node
{
public:
  MouseOdomNode()
  : Node("mouse_odom_node"), reset_epoch_(now()), start_time_(now())
  {
    left_sensor_x_ = declare_parameter<double>("left_sensor_x", 0.220);
    left_sensor_y_ = declare_parameter<double>("left_sensor_y", 0.055);
    left_sensor_z_ = declare_parameter<double>("left_sensor_z", 0.282);
    left_sensor_yaw_ = declare_parameter<double>("left_sensor_yaw", 0.0);
    right_sensor_x_ = declare_parameter<double>("right_sensor_x", 0.220);
    right_sensor_y_ = declare_parameter<double>("right_sensor_y", -0.055);
    right_sensor_z_ = declare_parameter<double>("right_sensor_z", 0.282);
    right_sensor_yaw_ = declare_parameter<double>("right_sensor_yaw", 0.0);

    // Placeholder values: these must be replaced by measurements on the real vehicle.
    left_x_meter_per_count_ =
      declare_parameter<double>("left_x_meter_per_count", 1.0e-4);
    left_y_meter_per_count_ =
      declare_parameter<double>("left_y_meter_per_count", 1.0e-4);
    right_x_meter_per_count_ =
      declare_parameter<double>("right_x_meter_per_count", 1.0e-4);
    right_y_meter_per_count_ =
      declare_parameter<double>("right_y_meter_per_count", 1.0e-4);
    sensor_height_from_ground_ =
      declare_parameter<double>("sensor_height_from_ground", 0.0);

    minimum_quality_ = declare_parameter<int>("minimum_quality", 0);
    sensor_timeout_sec_ = declare_parameter<double>("sensor_timeout_sec", 0.1);
    max_sensor_time_difference_sec_ =
      declare_parameter<double>("max_sensor_time_difference_sec", 0.02);
    max_sensor_interval_difference_sec_ =
      declare_parameter<double>("max_sensor_interval_difference_sec", 0.02);
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 5.0);
    max_lateral_speed_ = declare_parameter<double>("max_lateral_speed", 5.0);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 10.0);
    max_motion_residual_ = declare_parameter<double>("max_motion_residual", 0.01);

    left_topic_ =
      declare_parameter<std::string>("left_flow_topic", "/pmw3901/left/flow");
    right_topic_ =
      declare_parameter<std::string>("right_flow_topic", "/pmw3901/right/flow");
    frame_id_ = declare_parameter<std::string>("frame_id", "mouse_odom");
    child_frame_id_ =
      declare_parameter<std::string>("child_frame_id", "mouse_base_link");

    validateParameters();
    estimator_ = std::make_unique<PlanarMotionEstimator>(
      SensorPosition{left_sensor_x_, left_sensor_y_},
      SensorPosition{right_sensor_x_, right_sensor_y_});

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/mouse_odom", 10);
    debug_pub_ = create_publisher<msg::Pmw3901Debug>("/mouse_odom/debug", 10);

    left_sub_ = create_subscription<msg::Pmw3901Flow>(
      left_topic_, rclcpp::SensorDataQoS(),
      [this](msg::Pmw3901Flow::ConstSharedPtr message) {
        handleFlow(true, *message);
      });
    right_sub_ = create_subscription<msg::Pmw3901Flow>(
      right_topic_, rclcpp::SensorDataQoS(),
      [this](msg::Pmw3901Flow::ConstSharedPtr message) {
        handleFlow(false, *message);
      });

    reset_service_ = create_service<std_srvs::srv::Empty>(
      "/reset_mouse_odom",
      std::bind(
        &MouseOdomNode::resetCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    watchdog_timer_ = create_wall_timer(20ms, [this]() {checkSensorTimeouts();});

    RCLCPP_WARN(
      get_logger(),
      "PMW3901 meter_per_count defaults are placeholders; calibrate all four axes before use");
    RCLCPP_INFO(
      get_logger(),
      "PMW3901 odometry ready: left=%s, right=%s, output=/mouse_odom",
      left_topic_.c_str(), right_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Sensor poses: left=(%.3f, %.3f, %.3f), right=(%.3f, %.3f, %.3f) m; "
      "calibration height=%.3f m",
      left_sensor_x_, left_sensor_y_, left_sensor_z_,
      right_sensor_x_, right_sensor_y_, right_sensor_z_,
      sensor_height_from_ground_);
  }

private:
  struct Sample
  {
    int32_t raw_dx{0};
    int32_t raw_dy{0};
    uint16_t quality{0};
    bool quality_available{false};
    bool valid{false};
    bool pending{false};
    bool received{false};
    double integration_time{0.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time receive_time{0, 0, RCL_ROS_TIME};
  };

  struct ProcessedOutput
  {
    msg::Pmw3901Debug debug;
    std::optional<nav_msgs::msg::Odometry> odometry;
    bool warn{false};
  };

  void validateParameters() const
  {
    const auto require_positive = [](double value, const char * name) {
        if (!std::isfinite(value) || value <= 0.0) {
          throw std::invalid_argument(std::string(name) + " must be finite and > 0");
        }
      };
    const auto require_finite = [](double value, const char * name) {
        if (!std::isfinite(value)) {
          throw std::invalid_argument(std::string(name) + " must be finite");
        }
      };

    require_finite(left_sensor_x_, "left_sensor_x");
    require_finite(left_sensor_y_, "left_sensor_y");
    require_finite(left_sensor_z_, "left_sensor_z");
    require_finite(left_sensor_yaw_, "left_sensor_yaw");
    require_finite(right_sensor_x_, "right_sensor_x");
    require_finite(right_sensor_y_, "right_sensor_y");
    require_finite(right_sensor_z_, "right_sensor_z");
    require_finite(right_sensor_yaw_, "right_sensor_yaw");
    require_finite(left_x_meter_per_count_, "left_x_meter_per_count");
    require_finite(left_y_meter_per_count_, "left_y_meter_per_count");
    require_finite(right_x_meter_per_count_, "right_x_meter_per_count");
    require_finite(right_y_meter_per_count_, "right_y_meter_per_count");
    require_finite(sensor_height_from_ground_, "sensor_height_from_ground");
    require_positive(sensor_timeout_sec_, "sensor_timeout_sec");
    require_positive(max_sensor_time_difference_sec_, "max_sensor_time_difference_sec");
    require_positive(
      max_sensor_interval_difference_sec_, "max_sensor_interval_difference_sec");
    require_positive(max_linear_speed_, "max_linear_speed");
    require_positive(max_lateral_speed_, "max_lateral_speed");
    require_positive(max_angular_speed_, "max_angular_speed");
    require_positive(max_motion_residual_, "max_motion_residual");
    if (minimum_quality_ < 0 || minimum_quality_ > 65535) {
      throw std::invalid_argument("minimum_quality must be in [0, 65535]");
    }
  }

  static double durationSeconds(const builtin_interfaces::msg::Duration & duration)
  {
    return static_cast<double>(duration.sec) +
           static_cast<double>(duration.nanosec) * 1.0e-9;
  }

  rclcpp::Time measurementStamp(
    const msg::Pmw3901Flow & message,
    const rclcpp::Time & receive_time) const
  {
    if (message.header.stamp.sec == 0 && message.header.stamp.nanosec == 0) {
      return receive_time;
    }
    return rclcpp::Time(message.header.stamp, get_clock()->get_clock_type());
  }

  void handleFlow(bool is_left, const msg::Pmw3901Flow & message)
  {
    const rclcpp::Time receive_time = now();
    Sample sample;
    sample.raw_dx = message.delta_x_count;
    sample.raw_dy = message.delta_y_count;
    sample.quality = message.quality;
    sample.quality_available = message.quality_available;
    sample.integration_time = durationSeconds(message.integration_time);
    sample.stamp = measurementStamp(message, receive_time);
    sample.receive_time = receive_time;
    sample.received = true;
    sample.pending = true;
    sample.valid = message.valid && std::isfinite(sample.integration_time) &&
      sample.integration_time > 0.0 &&
      (!sample.quality_available || sample.quality >= minimum_quality_);

    std::optional<ProcessedOutput> output;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      // A stamped interval ending before reset may have been queued in an executor.
      const bool has_source_stamp =
        message.header.stamp.sec != 0 || message.header.stamp.nanosec != 0;
      if (has_source_stamp && sample.stamp <= reset_epoch_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Discarding pre-reset PMW3901 sample received after reset");
        return;
      }

      if (is_left) {
        left_sample_ = sample;
      } else {
        right_sample_ = sample;
      }
      output = tryProcessPairLocked(receive_time);
    }

    if (output.has_value()) {
      publishOutput(*output);
    }
  }

  std::optional<ProcessedOutput> tryProcessPairLocked(const rclcpp::Time & current_time)
  {
    if (!left_sample_.pending || !right_sample_.pending) {
      return std::nullopt;
    }

    const double left_age = (current_time - left_sample_.receive_time).seconds();
    const double right_age = (current_time - right_sample_.receive_time).seconds();
    if (left_age > sensor_timeout_sec_ || right_age > sensor_timeout_sec_) {
      auto output = makeRejectedOutputLocked("sensor_timeout");
      if (left_age > sensor_timeout_sec_) {
        left_sample_.pending = false;
      }
      if (right_age > sensor_timeout_sec_) {
        right_sample_.pending = false;
      }
      return output;
    }

    const double time_difference =
      std::abs((left_sample_.stamp - right_sample_.stamp).seconds());
    if (time_difference > max_sensor_time_difference_sec_) {
      auto output = makeRejectedOutputLocked("timestamp_mismatch");
      if (left_sample_.stamp < right_sample_.stamp) {
        left_sample_.pending = false;
      } else {
        right_sample_.pending = false;
      }
      return output;
    }

    if (std::abs(left_sample_.integration_time - right_sample_.integration_time) >
      max_sensor_interval_difference_sec_)
    {
      auto output = makeRejectedOutputLocked("integration_interval_mismatch");
      left_sample_.pending = false;
      right_sample_.pending = false;
      return output;
    }

    if (!left_sample_.valid || !right_sample_.valid) {
      std::string reason = "invalid_sensor";
      if ((left_sample_.quality_available && left_sample_.quality < minimum_quality_) ||
        (right_sample_.quality_available && right_sample_.quality < minimum_quality_))
      {
        reason = "quality_below_minimum";
      } else if (left_sample_.integration_time <= 0.0 ||
        right_sample_.integration_time <= 0.0)
      {
        reason = "invalid_integration_time";
      }
      auto output = makeRejectedOutputLocked(reason);
      left_sample_.pending = false;
      right_sample_.pending = false;
      return output;
    }

    const SensorDelta left_delta = convertToBody(
      left_sample_, left_x_meter_per_count_, left_y_meter_per_count_, left_sensor_yaw_);
    const SensorDelta right_delta = convertToBody(
      right_sample_, right_x_meter_per_count_, right_y_meter_per_count_,
      right_sensor_yaw_);
    const MotionEstimate estimate = estimator_->estimate(left_delta, right_delta);
    const double dt =
      0.5 * (left_sample_.integration_time + right_sample_.integration_time);
    const double vx = estimate.dx / dt;
    const double vy = estimate.dy / dt;
    const double wz = estimate.dyaw / dt;

    ProcessedOutput output;
    fillDebugLocked(output.debug, left_delta, right_delta);
    output.debug.delta_x = estimate.dx;
    output.debug.delta_y = estimate.dy;
    output.debug.delta_yaw = estimate.dyaw;
    output.debug.residual = estimate.residual;
    output.debug.dt = dt;

    if (!std::isfinite(estimate.dx) || !std::isfinite(estimate.dy) ||
      !std::isfinite(estimate.dyaw) || !std::isfinite(estimate.residual))
    {
      output.debug.status = "non_finite_estimate";
      output.warn = true;
    } else if (estimate.residual > max_motion_residual_) {
      output.debug.status = "motion_residual_exceeded";
      output.warn = true;
    } else if (std::abs(vx) > max_linear_speed_) {
      output.debug.status = "linear_speed_exceeded";
      output.warn = true;
    } else if (std::abs(vy) > max_lateral_speed_) {
      output.debug.status = "lateral_speed_exceeded";
      output.warn = true;
    } else if (std::abs(wz) > max_angular_speed_) {
      output.debug.status = "angular_speed_exceeded";
      output.warn = true;
    } else {
      const double yaw_mid = yaw_ + estimate.dyaw * 0.5;
      pos_x_ += estimate.dx * std::cos(yaw_mid) - estimate.dy * std::sin(yaw_mid);
      pos_y_ += estimate.dx * std::sin(yaw_mid) + estimate.dy * std::cos(yaw_mid);
      yaw_ = normalizeAngle(yaw_ + estimate.dyaw);

      output.debug.estimate_valid = true;
      output.debug.status = "ok";
      output.odometry = makeOdometryLocked(
        output.debug.header.stamp, vx, vy, wz, estimate.residual);
    }

    left_sample_.pending = false;
    right_sample_.pending = false;
    return output;
  }

  ProcessedOutput makeRejectedOutputLocked(const std::string & status) const
  {
    ProcessedOutput output;
    const SensorDelta left_delta = convertToBody(
      left_sample_, left_x_meter_per_count_, left_y_meter_per_count_, left_sensor_yaw_);
    const SensorDelta right_delta = convertToBody(
      right_sample_, right_x_meter_per_count_, right_y_meter_per_count_,
      right_sensor_yaw_);
    fillDebugLocked(output.debug, left_delta, right_delta);
    output.debug.status = status;
    output.warn = true;
    return output;
  }

  void fillDebugLocked(
    msg::Pmw3901Debug & debug,
    const SensorDelta & left_delta,
    const SensorDelta & right_delta) const
  {
    debug.header.stamp = left_sample_.stamp > right_sample_.stamp ?
      left_sample_.stamp : right_sample_.stamp;
    debug.header.frame_id = child_frame_id_;
    debug.left_raw_dx_count = left_sample_.raw_dx;
    debug.left_raw_dy_count = left_sample_.raw_dy;
    debug.right_raw_dx_count = right_sample_.raw_dx;
    debug.right_raw_dy_count = right_sample_.raw_dy;
    debug.left_dx_m = left_delta.x;
    debug.left_dy_m = left_delta.y;
    debug.right_dx_m = right_delta.x;
    debug.right_dy_m = right_delta.y;
    debug.left_quality = left_sample_.quality;
    debug.right_quality = right_sample_.quality;
    debug.left_quality_available = left_sample_.quality_available;
    debug.right_quality_available = right_sample_.quality_available;
    debug.left_valid = left_sample_.valid;
    debug.right_valid = right_sample_.valid;
    debug.estimate_valid = false;
  }

  static SensorDelta convertToBody(
    const Sample & sample,
    double x_meter_per_count,
    double y_meter_per_count,
    double sensor_yaw)
  {
    const double sensor_x = static_cast<double>(sample.raw_dx) * x_meter_per_count;
    const double sensor_y = static_cast<double>(sample.raw_dy) * y_meter_per_count;
    const double cosine = std::cos(sensor_yaw);
    const double sine = std::sin(sensor_yaw);
    return {
      cosine * sensor_x - sine * sensor_y,
      sine * sensor_x + cosine * sensor_y};
  }

  nav_msgs::msg::Odometry makeOdometryLocked(
    const builtin_interfaces::msg::Time & stamp,
    double vx, double vy, double wz, double residual) const
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = frame_id_;
    odometry.child_frame_id = child_frame_id_;
    odometry.pose.pose.position.x = pos_x_;
    odometry.pose.pose.position.y = pos_y_;
    odometry.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
    odometry.pose.pose.orientation.w = std::cos(yaw_ * 0.5);
    odometry.twist.twist.linear.x = vx;
    odometry.twist.twist.linear.y = vy;
    odometry.twist.twist.angular.z = wz;

    // Fixed baseline covariance. The residual is passed in deliberately so a
    // future calibrated dynamic model can be added at this single boundary.
    (void)residual;
    odometry.pose.covariance.fill(0.0);
    odometry.pose.covariance[0] = 0.01;
    odometry.pose.covariance[7] = 0.01;
    odometry.pose.covariance[14] = 999999.0;
    odometry.pose.covariance[21] = 999999.0;
    odometry.pose.covariance[28] = 999999.0;
    odometry.pose.covariance[35] = 0.05;
    odometry.twist.covariance.fill(0.0);
    odometry.twist.covariance[0] = 0.05;
    odometry.twist.covariance[7] = 0.05;
    odometry.twist.covariance[14] = 999999.0;
    odometry.twist.covariance[21] = 999999.0;
    odometry.twist.covariance[28] = 999999.0;
    odometry.twist.covariance[35] = 0.1;
    return odometry;
  }

  void publishOutput(const ProcessedOutput & output)
  {
    debug_pub_->publish(output.debug);
    if (output.odometry.has_value()) {
      odom_pub_->publish(*output.odometry);
    }
    if (output.warn) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "PMW3901 pair rejected: %s", output.debug.status.c_str());
    }
  }

  void checkSensorTimeouts()
  {
    const rclcpp::Time current_time = now();
    std::lock_guard<std::mutex> lock(data_mutex_);
    const double left_age = left_sample_.received ?
      (current_time - left_sample_.receive_time).seconds() :
      (current_time - start_time_).seconds();
    const double right_age = right_sample_.received ?
      (current_time - right_sample_.receive_time).seconds() :
      (current_time - start_time_).seconds();

    if (left_age > sensor_timeout_sec_ || right_age > sensor_timeout_sec_) {
      if (left_age > sensor_timeout_sec_) {
        left_sample_.valid = false;
        left_sample_.pending = false;
      }
      if (right_age > sensor_timeout_sec_) {
        right_sample_.valid = false;
        right_sample_.pending = false;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "PMW3901 timeout; integration stopped (left age=%.3f s, right age=%.3f s)",
        left_age, right_age);
    }
  }

  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pos_x_ = 0.0;
    pos_y_ = 0.0;
    yaw_ = 0.0;
    left_sample_.pending = false;
    right_sample_.pending = false;
    left_sample_.raw_dx = 0;
    left_sample_.raw_dy = 0;
    right_sample_.raw_dx = 0;
    right_sample_.raw_dy = 0;
    reset_epoch_ = now();
    RCLCPP_INFO(get_logger(), "PMW3901 odometry and pending deltas reset");
  }

  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  double left_sensor_x_;
  double left_sensor_y_;
  double left_sensor_z_;
  double left_sensor_yaw_;
  double right_sensor_x_;
  double right_sensor_y_;
  double right_sensor_z_;
  double right_sensor_yaw_;
  double left_x_meter_per_count_;
  double left_y_meter_per_count_;
  double right_x_meter_per_count_;
  double right_y_meter_per_count_;
  double sensor_height_from_ground_;
  int minimum_quality_;
  double sensor_timeout_sec_;
  double max_sensor_time_difference_sec_;
  double max_sensor_interval_difference_sec_;
  double max_linear_speed_;
  double max_lateral_speed_;
  double max_angular_speed_;
  double max_motion_residual_;
  std::string left_topic_;
  std::string right_topic_;
  std::string frame_id_;
  std::string child_frame_id_;

  double pos_x_{0.0};
  double pos_y_{0.0};
  double yaw_{0.0};
  Sample left_sample_;
  Sample right_sample_;
  rclcpp::Time reset_epoch_;
  rclcpp::Time start_time_;
  std::mutex data_mutex_;
  std::unique_ptr<PlanarMotionEstimator> estimator_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<msg::Pmw3901Debug>::SharedPtr debug_pub_;
  rclcpp::Subscription<msg::Pmw3901Flow>::SharedPtr left_sub_;
  rclcpp::Subscription<msg::Pmw3901Flow>::SharedPtr right_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace mouse_odometry

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mouse_odometry::MouseOdomNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("mouse_odom_node"),
      "Node initialization failed: %s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
