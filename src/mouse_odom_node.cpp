#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/input.h>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <sys/ioctl.h>
#include <algorithm>
#include <filesystem>
#include <vector>
#include <atomic>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class MouseOdomNode : public rclcpp::Node
{
public:
  MouseOdomNode()
  : Node("mouse_odom_node"),
    running_(true),
    fd1_(-1),
    fd2_(-1),
    dx_count_1_(0),
    dy_count_1_(0),
    dx_count_2_(0),
    dy_count_2_(0),
    pos_x_(0.0),
    pos_y_(0.0),
    yaw_(0.0)
  {
    // ==========================
    // パラメータ
    // ==========================
    auto_detect_devices_ = declare_parameter<bool>(
    "auto_detect_devices", true);

    device_path_1_ = declare_parameter<std::string>(
    "device_path_1", "");

    device_path_2_ = declare_parameter<std::string>(
    "device_path_2", "");

    if (auto_detect_devices_) {
    const std::vector<std::string> detected_devices =
        findMouseDevices();

    if (detected_devices.size() < 2) {
        RCLCPP_ERROR(
        get_logger(),
        "マウスが2台必要ですが、%zu台しか検出できませんでした",
        detected_devices.size());

        throw std::runtime_error(
        "Two mouse devices are required");
    }

    device_path_1_ = detected_devices[0];
    device_path_2_ = detected_devices[1];

    RCLCPP_INFO(
        get_logger(),
        "マウス1を自動選択: %s",
        device_path_1_.c_str());

    RCLCPP_INFO(
        get_logger(),
        "マウス2を自動選択: %s",
        device_path_2_.c_str());
    } else {
    if (device_path_1_.empty() || device_path_2_.empty()) {
        throw std::runtime_error(
        "device_path_1 and device_path_2 must be specified");
    }
    }

    cpi_ = declare_parameter<double>("cpi", 1000.0);

    // 元コードのY方向補正
    y_scale_ = declare_parameter<double>("y_scale", -1.0);

    // マウスの取り付け方向に応じて変更可能
    x_scale_ = declare_parameter<double>("x_scale", 1.0);

    mouse_separation_m_ = declare_parameter<double>(
    "mouse_separation_m", 0.135);

    // true の場合：マウス1が左、マウス2が右
    // false の場合：マウス1が右、マウス2が左
    mouse1_is_left_ = declare_parameter<bool>(
      "mouse1_is_left", true);

    // yawの符号が逆だったとき用
    yaw_sign_ = declare_parameter<double>(
      "yaw_sign", 1.0);

    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);

    frame_id_ = declare_parameter<std::string>(
      "frame_id", "mouse_odom");

    child_frame_id_ = declare_parameter<std::string>(
      "child_frame_id", "mouse_base_link");

    grab_device_ = declare_parameter<bool>("grab_device", true);

    // 1カウントあたりの移動距離[m]
    //
    // 1 inch = 0.0254 m
    // CPI = count / inch
    count_to_meter_ = 0.0254 / cpi_;

    // ==========================
    // Publisher
    // ==========================
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/mouse_odom", 10);

    // ==========================
    // 位置リセットサービス
    // ==========================
    reset_service_ = create_service<std_srvs::srv::Empty>(
      "/reset_mouse_odom",
      std::bind(
        &MouseOdomNode::resetCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    // ==========================
    // マウスデバイスを開く
    // ==========================
    fd1_ = openMouseDevice(device_path_1_, 1);
    fd2_ = openMouseDevice(device_path_2_, 2);

    if (fd1_ < 0 || fd2_ < 0) {
      RCLCPP_ERROR(
        get_logger(),
        "マウスデバイスを開けなかったため終了します");

      running_ = false;
      throw std::runtime_error("Failed to open mouse devices");
    }

    // ==========================
    // マウス読み取りスレッド
    // ==========================
    mouse_thread_1_ = std::thread(
      &MouseOdomNode::mouseListener,
      this,
      fd1_,
      1);

    mouse_thread_2_ = std::thread(
      &MouseOdomNode::mouseListener,
      this,
      fd2_,
      2);

    // ==========================
    // 配信タイマー
    // ==========================
    if (publish_rate_ <= 0.0) {
      publish_rate_ = 10.0;
    }

    const auto timer_period =
      std::chrono::duration<double>(1.0 / publish_rate_);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        timer_period),
      std::bind(&MouseOdomNode::publishOdometry, this));

    last_publish_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "Mouse odometry node started");

    RCLCPP_INFO(
      get_logger(),
      "Mouse 1: %s",
      device_path_1_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Mouse 2: %s",
      device_path_2_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Publishing: /mouse_odom at %.1f Hz",
      publish_rate_);

    RCLCPP_INFO(
      get_logger(),
      "CPI: %.1f, X scale: %.4f, Y scale: %.4f",
      cpi_,
      x_scale_,
      y_scale_);

    RCLCPP_INFO(
      get_logger(),
      "Reset service: /reset_mouse_odom");
  }

  ~MouseOdomNode() override
  {
    running_ = false;

    if (mouse_thread_1_.joinable()) {
      mouse_thread_1_.join();
    }

    if (mouse_thread_2_.joinable()) {
      mouse_thread_2_.join();
    }

    closeMouseDevice(fd1_);
    closeMouseDevice(fd2_);
  }

private:
  bool endsWith(
    const std::string & text,
    const std::string & suffix) const
  {
    if (text.size() < suffix.size()) {
      return false;
    }

    return text.compare(
      text.size() - suffix.size(),
      suffix.size(),
      suffix) == 0;
  }

  double normalizeAngle(double angle) const
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  std::vector<std::string> findMouseDevices()
  {
    namespace fs = std::filesystem;

  const std::string input_by_id_dir = "/dev/input/by-id";
  std::vector<std::string> mouse_devices;

  try {
    if (!fs::exists(input_by_id_dir)) {
      RCLCPP_ERROR(
        get_logger(),
        "%s が存在しません",
        input_by_id_dir.c_str());

      return mouse_devices;
    }

    for (const auto & entry : fs::directory_iterator(input_by_id_dir)) {
      const std::string filename =
        entry.path().filename().string();

      // event-mouseで終わるものだけを使用する
      if (filename.size() < std::string("event-mouse").size()) {
        continue;
      }

      if (endsWith(filename, "event-mouse")) {
        const std::string device_path =
          entry.path().string();

        mouse_devices.push_back(device_path);

        RCLCPP_INFO(
          get_logger(),
          "マウス候補を検出: %s",
          device_path.c_str());
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(),
      "マウス自動検出中にエラー: %s",
      e.what());
  }

  // 検出順が毎回変わらないように並べ替える
  std::sort(
    mouse_devices.begin(),
    mouse_devices.end());

  return mouse_devices;
}

  int openMouseDevice(
    const std::string & device_path,
    int mouse_id)
  {
    const int fd = open(
      device_path.c_str(),
      O_RDONLY | O_NONBLOCK);

    if (fd < 0) {
      RCLCPP_ERROR(
        get_logger(),
        "Mouse %d (%s) open failed: %s",
        mouse_id,
        device_path.c_str(),
        std::strerror(errno));

      return -1;
    }

    char device_name[256] = "Unknown";

    if (ioctl(
        fd,
        EVIOCGNAME(sizeof(device_name)),
        device_name) < 0)
    {
      std::strncpy(
        device_name,
        "Unknown",
        sizeof(device_name) - 1);
    }

    if (grab_device_) {
      if (ioctl(fd, EVIOCGRAB, 1) < 0) {
        RCLCPP_WARN(
          get_logger(),
          "Mouse %d grab failed: %s",
          mouse_id,
          std::strerror(errno));
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Mouse %d connected: %s (%s)",
      mouse_id,
      device_name,
      device_path.c_str());

    return fd;
  }

  void closeMouseDevice(int & fd)
  {
    if (fd < 0) {
      return;
    }

    if (grab_device_) {
      ioctl(fd, EVIOCGRAB, 0);
    }

    close(fd);
    fd = -1;
  }

  void mouseListener(
    int fd,
    int mouse_id)
  {
    constexpr std::size_t EVENT_BUFFER_SIZE = 64;
    input_event events[EVENT_BUFFER_SIZE];

    while (running_ && rclcpp::ok()) {
      const ssize_t bytes_read = read(
        fd,
        events,
        sizeof(events));

      if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          std::this_thread::sleep_for(2ms);
          continue;
        }

        if (errno == EINTR) {
          continue;
        }

        RCLCPP_ERROR(
          get_logger(),
          "Mouse %d read error: %s",
          mouse_id,
          std::strerror(errno));

        break;
      }

      if (bytes_read == 0) {
        std::this_thread::sleep_for(2ms);
        continue;
      }

      const std::size_t event_count =
        static_cast<std::size_t>(bytes_read) /
        sizeof(input_event);

      std::lock_guard<std::mutex> lock(data_mutex_);

      for (std::size_t i = 0; i < event_count; ++i) {
        const input_event & event = events[i];

        if (event.type != EV_REL) {
          continue;
        }

        if (mouse_id == 1) {
          if (event.code == REL_X) {
            dx_count_1_ += event.value;
          } else if (event.code == REL_Y) {
            dy_count_1_ += event.value;
          }
        } else {
          if (event.code == REL_X) {
            dx_count_2_ += event.value;
          } else if (event.code == REL_Y) {
            dy_count_2_ += event.value;
          }
        }
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Mouse %d listener stopped",
      mouse_id);
  }

  void publishOdometry()
  {
    const rclcpp::Time current_time = now();
    const double dt =
      (current_time - last_publish_time_).seconds();

    if (dt <= 0.0) {
      return;
    }

    int64_t m1_dx = 0;
    int64_t m1_dy = 0;
    int64_t m2_dx = 0;
    int64_t m2_dy = 0;

    // マウス移動量を回収してカウンタをリセット
    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      m1_dx = dx_count_1_;
      m1_dy = dy_count_1_;
      m2_dx = dx_count_2_;
      m2_dy = dy_count_2_;

      dx_count_1_ = 0;
      dy_count_1_ = 0;
      dx_count_2_ = 0;
      dy_count_2_ = 0;
    }

    // 各マウスのカウントをメートルに変換
    const double m1_dx_m =
      static_cast<double>(m1_dx) * count_to_meter_ * x_scale_;

    const double m1_dy_m =
      static_cast<double>(m1_dy) * count_to_meter_ * y_scale_;

    const double m2_dx_m =
      static_cast<double>(m2_dx) * count_to_meter_ * x_scale_;

    const double m2_dy_m =
      static_cast<double>(m2_dy) * count_to_meter_ * y_scale_;

    // --------------------------------------------------
    // 前提：
    // REL_Y方向をロボットの前後方向として扱う
    // REL_X方向をロボットの左右方向として扱う
    // --------------------------------------------------

    double left_forward = 0.0;
    double right_forward = 0.0;

    if (mouse1_is_left_) {
      left_forward = m1_dy_m;
      right_forward = m2_dy_m;
    } else {
      left_forward = m2_dy_m;
      right_forward = m1_dy_m;
    }

    // ロボット座標系での並進移動量
    // ROSでは x が前方向、y が左方向
    const double delta_x_body =
      (left_forward + right_forward) / 2.0;

    const double delta_y_body =
      (m1_dx_m + m2_dx_m) / 2.0;

    // yaw変化量
    double delta_yaw = 0.0;

    if (mouse_separation_m_ > 0.0) {
      delta_yaw =
        yaw_sign_ *
        (right_forward - left_forward) /
        mouse_separation_m_;
    }

    double pos_x;
    double pos_y;
    double yaw;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      // 旋回中の移動を少し自然にするため、中間yawで座標変換
      const double yaw_mid = yaw_ + delta_yaw * 0.5;

      // body座標系の移動量をodom座標系へ変換
      pos_x_ +=
        delta_x_body * std::cos(yaw_mid) -
        delta_y_body * std::sin(yaw_mid);

      pos_y_ +=
        delta_x_body * std::sin(yaw_mid) +
        delta_y_body * std::cos(yaw_mid);

      yaw_ = normalizeAngle(yaw_ + delta_yaw);

      pos_x = pos_x_;
      pos_y = pos_y_;
      yaw = yaw_;
    }

    const double velocity_x = delta_x_body / dt;
    const double velocity_y = delta_y_body / dt;
    const double angular_z = delta_yaw / dt;

    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = frame_id_;
    odom_msg.child_frame_id = child_frame_id_;

    // ==========================
    // 位置
    // ==========================
    odom_msg.pose.pose.position.x = pos_x;
    odom_msg.pose.pose.position.y = pos_y;
    odom_msg.pose.pose.position.z = 0.0;

    // yaw -> quaternion
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = std::sin(yaw * 0.5);
    odom_msg.pose.pose.orientation.w = std::cos(yaw * 0.5);

    // ==========================
    // 速度
    // ==========================
    odom_msg.twist.twist.linear.x = velocity_x;
    odom_msg.twist.twist.linear.y = velocity_y;
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_z;

    // ==========================
    // 共分散
    // ==========================
    odom_msg.pose.covariance.fill(0.0);

    odom_msg.pose.covariance[0] = 0.01;       // x
    odom_msg.pose.covariance[7] = 0.01;       // y
    odom_msg.pose.covariance[14] = 999999.0;  // z
    odom_msg.pose.covariance[21] = 999999.0;  // roll
    odom_msg.pose.covariance[28] = 999999.0;  // pitch
    odom_msg.pose.covariance[35] = 0.05;      // yaw

    odom_msg.twist.covariance.fill(0.0);

    odom_msg.twist.covariance[0] = 0.05;       // vx
    odom_msg.twist.covariance[7] = 0.05;       // vy
    odom_msg.twist.covariance[14] = 999999.0;  // vz
    odom_msg.twist.covariance[21] = 999999.0;  // wx
    odom_msg.twist.covariance[28] = 999999.0;  // wy
    odom_msg.twist.covariance[35] = 0.1;       // wz

    odom_pub_->publish(odom_msg);

    last_publish_time_ = current_time;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      500,
      "x=%.3f m, y=%.3f m, yaw=%.3f rad | "
      "vx=%.3f m/s, vy=%.3f m/s, wz=%.3f rad/s",
      pos_x,
      pos_y,
      yaw,
      velocity_x,
      velocity_y,
      angular_z);
  }

  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    pos_x_ = 0.0;
    pos_y_ = 0.0;
    yaw_ = 0.0;

    dx_count_1_ = 0;
    dy_count_1_ = 0;
    dx_count_2_ = 0;
    dy_count_2_ = 0;

    last_publish_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "Mouse odometry position reset");
  }

  // デバイス設定
  std::string device_path_1_;
  std::string device_path_2_;

  double cpi_;
  double count_to_meter_;
  double x_scale_;
  double y_scale_;
  double publish_rate_;

  double mouse_separation_m_;
  bool mouse1_is_left_;
  double yaw_sign_;

  std::string frame_id_;
  std::string child_frame_id_;

  bool auto_detect_devices_;
  bool grab_device_;

  // デバイス
  int fd1_;
  int fd2_;

  // スレッド
  std::atomic<bool> running_;
  std::thread mouse_thread_1_;
  std::thread mouse_thread_2_;

  // マウス移動量
  int64_t dx_count_1_;
  int64_t dy_count_1_;
  int64_t dx_count_2_;
  int64_t dy_count_2_;

  // 積算位置[m]
  double pos_x_;
  double pos_y_;
  double yaw_;

  std::mutex data_mutex_;

  // ROS
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_publish_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<MouseOdomNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("mouse_odom_node"),
      "Node initialization failed: %s",
      e.what());
  }

  rclcpp::shutdown();
  return 0;
}