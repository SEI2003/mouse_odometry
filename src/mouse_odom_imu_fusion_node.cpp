#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mouse_odometry/msg/pmw3901_debug.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace mouse_odometry
{

class MouseOdomImuFusionNode : public rclcpp::Node
{
public:
  MouseOdomImuFusionNode()
  : Node("mouse_odom_imu_fusion_node"), start_time_(now())
  {
    imu_topic_ = declare_parameter<std::string>(
      "imu_topic", "/aiformula_sensing/zed_node/imu");
    pmw_debug_topic_ = declare_parameter<std::string>(
      "pmw_debug_topic", "/mouse_odom/debug");
    output_odom_topic_ = declare_parameter<std::string>(
      "output_odom_topic", "/mouse_odom_imu");
    output_pose2d_topic_ = declare_parameter<std::string>(
      "output_pose2d_topic", "/mouse_odom_imu/pose2d");
    frame_id_ = declare_parameter<std::string>("frame_id", "mouse_odom_imu");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "mouse_base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    imu_yaw_weight_ = declare_parameter<double>("imu_yaw_weight", 0.5);
    imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.2);
    imu_gyro_z_sign_ = declare_parameter<double>("imu_gyro_z_sign", 1.0);
    enable_debug_csv_log_ = declare_parameter<bool>("enable_debug_csv_log", false);

    validateParameters();
    if (enable_debug_csv_log_) {
      initializeDebugCsvLog();
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);
    pose2d_pub_ = create_publisher<geometry_msgs::msg::Pose2D>(output_pose2d_topic_, 10);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        handleImu(*message);
      });
    pmw_debug_sub_ = create_subscription<msg::Pmw3901Debug>(
      pmw_debug_topic_, 10,
      [this](msg::Pmw3901Debug::ConstSharedPtr message) {
        handlePmwDebug(*message);
      });
    reset_service_ = create_service<std_srvs::srv::Empty>(
      "/reset_mouse_odom_imu",
      [this](
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>)
      {
        reset();
      });

    RCLCPP_INFO(
      get_logger(),
      "PMW3901/IMU yaw fusion ready: pmw=%s imu=%s output=%s weight=%.3f",
      pmw_debug_topic_.c_str(), imu_topic_.c_str(), output_odom_topic_.c_str(),
      imu_yaw_weight_);
    RCLCPP_INFO(
      get_logger(), "TF publishing %s: %s -> %s",
      publish_tf_ ? "enabled" : "disabled", frame_id_.c_str(), child_frame_id_.c_str());
  }

private:
  struct ImuState
  {
    bool received{false};
    bool angular_velocity_valid{false};
    bool orientation_valid{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time receive_time{0, 0, RCL_ROS_TIME};
    double orientation_x{0.0};
    double orientation_y{0.0};
    double orientation_z{0.0};
    double orientation_w{0.0};
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
    double angular_velocity_x{0.0};
    double angular_velocity_y{0.0};
    double angular_velocity_z{0.0};
    double yaw_rate_used{0.0};
  };

  struct FusionResult
  {
    bool produced{false};
    bool imu_fresh{false};
    double imu_delta_yaw{std::numeric_limits<double>::quiet_NaN()};
    double fused_delta_yaw{std::numeric_limits<double>::quiet_NaN()};
    double fused_wz{std::numeric_limits<double>::quiet_NaN()};
  };

  void validateParameters() const
  {
    if (!std::isfinite(imu_yaw_weight_) || imu_yaw_weight_ < 0.0 ||
      imu_yaw_weight_ > 1.0)
    {
      throw std::invalid_argument("imu_yaw_weight must be finite and in [0, 1]");
    }
    if (!std::isfinite(imu_timeout_sec_) || imu_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("imu_timeout_sec must be finite and > 0");
    }
    if (!std::isfinite(imu_gyro_z_sign_) || imu_gyro_z_sign_ == 0.0) {
      throw std::invalid_argument("imu_gyro_z_sign must be finite and non-zero");
    }
  }

  void initializeDebugCsvLog()
  {
    const char * home = std::getenv("HOME");
    if (home == nullptr || home[0] == '\0') {
      throw std::runtime_error("HOME is not set; cannot create PMW3901/IMU fusion log");
    }

    const std::filesystem::path log_directory = std::filesystem::path(home) / ".ros";
    std::filesystem::create_directories(log_directory);

    const std::time_t current_time = std::time(nullptr);
    std::tm local_time{};
    if (localtime_r(&current_time, &local_time) == nullptr) {
      throw std::runtime_error("Failed to create timestamp for PMW3901/IMU fusion log");
    }

    char filename[48];
    if (std::strftime(
        filename, sizeof(filename),
        "pmw3901_imu_fusion_%Y%m%d_%H%M%S.csv", &local_time) == 0)
    {
      throw std::runtime_error("Failed to format PMW3901/IMU fusion log filename");
    }

    debug_csv_log_path_ = (log_directory / filename).string();
    debug_csv_log_.open(debug_csv_log_path_, std::ios::out | std::ios::trunc);
    if (!debug_csv_log_.is_open()) {
      throw std::runtime_error(
              "Failed to open PMW3901/IMU fusion log: " + debug_csv_log_path_);
    }

    debug_csv_log_ << std::setprecision(17);
    debug_csv_log_ <<
      "elapsed_time_sec,left_cycle_id,right_cycle_id,pair_valid,reject_reason,"
      "left_raw_dx_count,left_raw_dy_count,right_raw_dx_count,right_raw_dy_count,"
      "left_dx_m,left_dy_m,right_dx_m,right_dy_m,left_quality,right_quality,"
      "pmw_delta_x,pmw_delta_y,pmw_delta_yaw,pmw_vx,pmw_vy,pmw_wz,motion_residual,"
      "imu_fresh,imu_orientation_valid,imu_orientation_x,imu_orientation_y,"
      "imu_orientation_z,imu_orientation_w,imu_roll,imu_pitch,imu_yaw,"
      "imu_angular_velocity_x,imu_angular_velocity_y,imu_angular_velocity_z,"
      "imu_yaw_rate_used,imu_delta_yaw,imu_yaw_weight,fused_delta_yaw,"
      "fused_x,fused_y,fused_yaw,fused_vx,fused_vy,fused_wz\n";
    RCLCPP_INFO(
      get_logger(), "PMW3901/IMU fusion log: %s", debug_csv_log_path_.c_str());
  }

  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  rclcpp::Time messageTime(
    const builtin_interfaces::msg::Time & stamp,
    const rclcpp::Time & receive_time) const
  {
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      return receive_time;
    }
    return rclcpp::Time(stamp, get_clock()->get_clock_type());
  }

  void handleImu(const sensor_msgs::msg::Imu & message)
  {
    const rclcpp::Time receive_time = now();
    const rclcpp::Time stamp = messageTime(message.header.stamp, receive_time);
    const double yaw_rate = message.angular_velocity.z * imu_gyro_z_sign_;
    const bool angular_velocity_valid =
      message.angular_velocity_covariance[0] >= 0.0 && std::isfinite(yaw_rate);

    if (angular_velocity_valid) {
      if (imu_integrator_initialized_) {
        const double dt = (stamp - last_imu_stamp_).seconds();
        if (dt > 0.0 && dt <= imu_timeout_sec_) {
          imu_yaw_integral_ += 0.5 * (last_imu_yaw_rate_ + yaw_rate) * dt;
          imu_integration_duration_ += dt;
        } else {
          imu_yaw_integral_ = 0.0;
          imu_integration_duration_ = 0.0;
        }
      }
      last_imu_stamp_ = stamp;
      last_imu_yaw_rate_ = yaw_rate;
      imu_integrator_initialized_ = true;
    } else {
      imu_integrator_initialized_ = false;
      imu_yaw_integral_ = 0.0;
      imu_integration_duration_ = 0.0;
    }

    imu_state_.received = true;
    imu_state_.angular_velocity_valid = angular_velocity_valid;
    imu_state_.stamp = stamp;
    imu_state_.receive_time = receive_time;
    imu_state_.orientation_x = message.orientation.x;
    imu_state_.orientation_y = message.orientation.y;
    imu_state_.orientation_z = message.orientation.z;
    imu_state_.orientation_w = message.orientation.w;
    imu_state_.angular_velocity_x = message.angular_velocity.x;
    imu_state_.angular_velocity_y = message.angular_velocity.y;
    imu_state_.angular_velocity_z = message.angular_velocity.z;
    imu_state_.yaw_rate_used = yaw_rate;

    const double quaternion_norm = std::sqrt(
      message.orientation.x * message.orientation.x +
      message.orientation.y * message.orientation.y +
      message.orientation.z * message.orientation.z +
      message.orientation.w * message.orientation.w);
    imu_state_.orientation_valid = message.orientation_covariance[0] >= 0.0 &&
      std::isfinite(quaternion_norm) && quaternion_norm > 1.0e-12;
    if (imu_state_.orientation_valid) {
      const double x = message.orientation.x / quaternion_norm;
      const double y = message.orientation.y / quaternion_norm;
      const double z = message.orientation.z / quaternion_norm;
      const double w = message.orientation.w / quaternion_norm;
      imu_state_.roll = std::atan2(
        2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
      imu_state_.pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
      imu_state_.yaw = std::atan2(
        2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    }
  }

  FusionResult fuse(const msg::Pmw3901Debug & debug, const rclcpp::Time & receive_time)
  {
    FusionResult result;
    const double dt =
      0.5 * (debug.left_integration_time_sec + debug.right_integration_time_sec);
    result.imu_fresh = imu_state_.received && imu_state_.angular_velocity_valid &&
      (receive_time - imu_state_.receive_time).seconds() >= 0.0 &&
      (receive_time - imu_state_.receive_time).seconds() <= imu_timeout_sec_;

    if (result.imu_fresh && std::isfinite(dt) && dt > 0.0) {
      const double average_imu_yaw_rate = imu_integration_duration_ > 0.0 ?
        imu_yaw_integral_ / imu_integration_duration_ : imu_state_.yaw_rate_used;
      result.imu_delta_yaw = average_imu_yaw_rate * dt;
    }

    imu_yaw_integral_ = 0.0;
    imu_integration_duration_ = 0.0;

    if (!debug.pair_valid) {
      return result;
    }

    result.fused_delta_yaw = result.imu_fresh ?
      (1.0 - imu_yaw_weight_) * debug.delta_yaw +
      imu_yaw_weight_ * result.imu_delta_yaw :
      debug.delta_yaw;
    result.fused_wz = result.fused_delta_yaw / dt;
    if (!std::isfinite(result.fused_delta_yaw) || !std::isfinite(result.fused_wz)) {
      return result;
    }

    const double yaw_mid = fused_yaw_ + result.fused_delta_yaw * 0.5;
    fused_x_ +=
      debug.delta_x_body * std::cos(yaw_mid) - debug.delta_y_body * std::sin(yaw_mid);
    fused_y_ +=
      debug.delta_x_body * std::sin(yaw_mid) + debug.delta_y_body * std::cos(yaw_mid);
    fused_yaw_ = normalizeAngle(fused_yaw_ + result.fused_delta_yaw);
    result.produced = true;
    return result;
  }

  void handlePmwDebug(const msg::Pmw3901Debug & debug)
  {
    const rclcpp::Time receive_time = now();
    const FusionResult fusion = fuse(debug, receive_time);

    if (debug.pair_valid && !fusion.imu_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU unavailable or stale; using PMW3901 yaw only");
    }

    if (fusion.produced) {
      publishFusion(debug, fusion);
    }
    if (enable_debug_csv_log_) {
      writeDebugCsvLog(debug, fusion, receive_time);
    }
  }

  void publishFusion(const msg::Pmw3901Debug & debug, const FusionResult & fusion)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = debug.header.stamp;
    odometry.header.frame_id = frame_id_;
    odometry.child_frame_id = child_frame_id_;
    odometry.pose.pose.position.x = fused_x_;
    odometry.pose.pose.position.y = fused_y_;
    odometry.pose.pose.orientation.z = std::sin(fused_yaw_ * 0.5);
    odometry.pose.pose.orientation.w = std::cos(fused_yaw_ * 0.5);
    odometry.twist.twist.linear.x = debug.vx;
    odometry.twist.twist.linear.y = debug.vy;
    odometry.twist.twist.angular.z = fusion.fused_wz;
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
    odom_pub_->publish(odometry);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = odometry.child_frame_id;
      transform.transform.translation.x = odometry.pose.pose.position.x;
      transform.transform.translation.y = odometry.pose.pose.position.y;
      transform.transform.translation.z = odometry.pose.pose.position.z;
      transform.transform.rotation = odometry.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }

    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = fused_x_;
    pose2d.y = fused_y_;
    pose2d.theta = fused_yaw_;
    pose2d_pub_->publish(pose2d);
  }

  void writeDebugCsvLog(
    const msg::Pmw3901Debug & debug,
    const FusionResult & fusion,
    const rclcpp::Time & receive_time)
  {
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    const bool orientation_available = imu_state_.received && imu_state_.orientation_valid;
    const double fused_x = fusion.produced ? fused_x_ : unavailable;
    const double fused_y = fusion.produced ? fused_y_ : unavailable;
    const double fused_yaw = fusion.produced ? fused_yaw_ : unavailable;

    debug_csv_log_ <<
      (receive_time - start_time_).seconds() << ',' <<
      debug.left_cycle_id << ',' << debug.right_cycle_id << ',' <<
      static_cast<int>(debug.pair_valid) << ',' << debug.status << ',' <<
      debug.left_raw_dx_count << ',' << debug.left_raw_dy_count << ',' <<
      debug.right_raw_dx_count << ',' << debug.right_raw_dy_count << ',' <<
      debug.left_dx_m << ',' << debug.left_dy_m << ',' <<
      debug.right_dx_m << ',' << debug.right_dy_m << ',' <<
      debug.left_quality << ',' << debug.right_quality << ',' <<
      debug.delta_x_body << ',' << debug.delta_y_body << ',' << debug.delta_yaw << ',' <<
      debug.vx << ',' << debug.vy << ',' << debug.wz << ',' << debug.motion_residual << ',' <<
      static_cast<int>(fusion.imu_fresh) << ',' <<
      static_cast<int>(orientation_available) << ',' <<
      (imu_state_.received ? imu_state_.orientation_x : unavailable) << ',' <<
      (imu_state_.received ? imu_state_.orientation_y : unavailable) << ',' <<
      (imu_state_.received ? imu_state_.orientation_z : unavailable) << ',' <<
      (imu_state_.received ? imu_state_.orientation_w : unavailable) << ',' <<
      (orientation_available ? imu_state_.roll : unavailable) << ',' <<
      (orientation_available ? imu_state_.pitch : unavailable) << ',' <<
      (orientation_available ? imu_state_.yaw : unavailable) << ',' <<
      (imu_state_.received ? imu_state_.angular_velocity_x : unavailable) << ',' <<
      (imu_state_.received ? imu_state_.angular_velocity_y : unavailable) << ',' <<
      (imu_state_.received ? imu_state_.angular_velocity_z : unavailable) << ',' <<
      (imu_state_.angular_velocity_valid ? imu_state_.yaw_rate_used : unavailable) << ',' <<
      fusion.imu_delta_yaw << ',' << imu_yaw_weight_ << ',' << fusion.fused_delta_yaw << ',' <<
      fused_x << ',' << fused_y << ',' << fused_yaw << ',' <<
      (fusion.produced ? debug.vx : unavailable) << ',' <<
      (fusion.produced ? debug.vy : unavailable) << ',' << fusion.fused_wz << '\n';

    ++debug_csv_log_rows_;
    if (debug_csv_log_rows_ % 100 == 0) {
      debug_csv_log_.flush();
    }
  }

  void reset()
  {
    fused_x_ = 0.0;
    fused_y_ = 0.0;
    fused_yaw_ = 0.0;
    imu_integrator_initialized_ = false;
    imu_yaw_integral_ = 0.0;
    imu_integration_duration_ = 0.0;
    RCLCPP_INFO(get_logger(), "PMW3901/IMU fused odometry reset");
  }

  std::string imu_topic_;
  std::string pmw_debug_topic_;
  std::string output_odom_topic_;
  std::string output_pose2d_topic_;
  std::string frame_id_;
  std::string child_frame_id_;
  bool publish_tf_;
  double imu_yaw_weight_;
  double imu_timeout_sec_;
  double imu_gyro_z_sign_;
  bool enable_debug_csv_log_;

  ImuState imu_state_;
  bool imu_integrator_initialized_{false};
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
  double last_imu_yaw_rate_{0.0};
  double imu_yaw_integral_{0.0};
  double imu_integration_duration_{0.0};
  double fused_x_{0.0};
  double fused_y_{0.0};
  double fused_yaw_{0.0};
  rclcpp::Time start_time_;

  std::ofstream debug_csv_log_;
  std::string debug_csv_log_path_;
  std::size_t debug_csv_log_rows_{0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<msg::Pmw3901Debug>::SharedPtr pmw_debug_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
};

}  // namespace mouse_odometry

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mouse_odometry::MouseOdomImuFusionNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("mouse_odom_imu_fusion_node"),
      "Node initialization failed: %s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
