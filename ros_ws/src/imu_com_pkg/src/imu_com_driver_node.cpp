#include <rclcpp/rclcpp.hpp>
#include <sensor_driver_msgs/reconnecting_serial.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace
{
constexpr uint8_t kFrameHead = 0xFC;
constexpr uint8_t kFrameEnd = 0xFD;
constexpr uint8_t kImuType = 0x40;
constexpr uint8_t kAhrsType = 0x41;
constexpr std::size_t kHeaderSize = 7;
constexpr std::size_t kMaxPayloadSize = 128;
constexpr std::size_t kMaxReadSize = 8192;
constexpr std::size_t kMaxBufferedBytes = 16384;

uint8_t crc8_maxim(const uint8_t * data, const std::size_t size)
{
  uint8_t crc = 0;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x01U) != 0U ? static_cast<uint8_t>((crc >> 1U) ^ 0x8CU) :
        static_cast<uint8_t>(crc >> 1U);
    }
  }
  return crc;
}

uint16_t crc16_ccitt(const uint8_t * data, const std::size_t size)
{
  uint16_t crc = 0;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8U;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U) :
        static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

float read_float_le(const uint8_t * data)
{
  const uint32_t raw = static_cast<uint32_t>(data[0]) |
    (static_cast<uint32_t>(data[1]) << 8U) |
    (static_cast<uint32_t>(data[2]) << 16U) |
    (static_cast<uint32_t>(data[3]) << 24U);
  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

bool finite3(const double x, const double y, const double z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}
}  // namespace

// Receive-only driver for the FDILink stream currently emitted by the H30 IMU.
class ImuComNode : public rclcpp::Node
{
public:
  ImuComNode()
  : Node("imu_driver_node")
  {
    baudrate_ = declare_parameter<int>("baudrate_IMU", 921600);
    port_name_ = declare_parameter<std::string>("IMU_ComName", "/dev/sensors/imu");
    frame_id_ = declare_parameter<std::string>("imu_frame_id", "tamagawa/imu_link");
    imu_topic_ = declare_parameter<std::string>(
      "imu_topic", "/sensing/imu/tamagawa/imu_raw");
    mag_topic_ = declare_parameter<std::string>(
      "mag_topic", "/sensing/imu/tamagawa/magnetic_field");
    convert_to_ros_flu_ = declare_parameter<bool>("convert_to_ros_flu", true);
    orientation_timeout_ = std::chrono::milliseconds(std::max<int64_t>(
      20, declare_parameter<int64_t>("orientation_timeout_ms", 200)));
    reconnect_period_ = std::chrono::milliseconds(std::max<int64_t>(
      100, declare_parameter<int64_t>("reconnect_period_ms", 1000)));
    data_timeout_ = std::chrono::milliseconds(std::max<int64_t>(
      100, declare_parameter<int64_t>("data_timeout_ms", 2000)));

    // autoware_imu_corrector subscribes with RELIABLE QoS. A BEST_EFFORT
    // publisher is incompatible and leaves /sensing/imu/imu_data silent even
    // while raw H30 frames are healthy.
    const auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, imu_qos);
    mag_publisher_ = create_publisher<sensor_msgs::msg::MagneticField>(mag_topic_, imu_qos);

    serial_ = std::make_unique<sensor_driver_msgs::ReconnectingSerial>(
      sensor_driver_msgs::ReconnectingSerial::Options{
        port_name_, static_cast<uint32_t>(baudrate_), std::chrono::milliseconds(20),
        reconnect_period_, data_timeout_},
      [this](const std::string & message) {
        RCLCPP_ERROR(get_logger(), "H30 offline: %s", message.c_str());
      },
      [this](const std::string & message) {
        RCLCPP_INFO(get_logger(), "H30 recovered: %s", message.c_str());
      });

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(2ms, std::bind(&ImuComNode::read_serial, this));
    RCLCPP_INFO(
      get_logger(), "FDILink H30 driver started: port=%s baud=%d frame=%s receive-only",
      port_name_.c_str(), baudrate_, frame_id_.c_str());
  }

private:
  void parse_ahrs(const uint8_t * data)
  {
    const double qw = read_float_le(data + 24);
    const double qx = read_float_le(data + 28);
    const double qy = read_float_le(data + 32);
    const double qz = read_float_le(data + 36);
    const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (!finite3(qx, qy, qz) || !std::isfinite(qw) || norm < 0.5 || norm > 1.5) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected invalid H30 quaternion");
      return;
    }

    tf2::Quaternion device(qx / norm, qy / norm, qz / norm, qw / norm);
    if (convert_to_ros_flu_) {
      tf2::Quaternion left;
      left.setRPY(0.0, M_PI, M_PI);
      tf2::Quaternion right;
      right.setRPY(M_PI, 0.0, 0.0);
      orientation_ = left * device * right;
    } else {
      orientation_ = device;
    }
    orientation_.normalize();
    orientation_valid_ = true;
    last_orientation_time_ = std::chrono::steady_clock::now();
  }

  void parse_imu(const uint8_t * data)
  {
    double gx = read_float_le(data + 0);
    double gy = read_float_le(data + 4);
    double gz = read_float_le(data + 8);
    double ax = read_float_le(data + 12);
    double ay = read_float_le(data + 16);
    double az = read_float_le(data + 20);
    double mx = read_float_le(data + 24);
    double my = read_float_le(data + 28);
    double mz = read_float_le(data + 32);
    if (!finite3(gx, gy, gz) || !finite3(ax, ay, az) || !finite3(mx, my, mz)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected non-finite H30 sample");
      return;
    }
    if (convert_to_ros_flu_) {
      gy = -gy;
      gz = -gz;
      ay = -ay;
      az = -az;
      my = -my;
      mz = -mz;
    }

    const auto stamp = now();
    sensor_msgs::msg::Imu message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.angular_velocity.x = gx;
    message.angular_velocity.y = gy;
    message.angular_velocity.z = gz;
    message.linear_acceleration.x = ax;
    message.linear_acceleration.y = ay;
    message.linear_acceleration.z = az;
    if (orientation_valid_ &&
      std::chrono::steady_clock::now() - last_orientation_time_ <= orientation_timeout_)
    {
      message.orientation.x = orientation_.x();
      message.orientation.y = orientation_.y();
      message.orientation.z = orientation_.z();
      message.orientation.w = orientation_.w();
      message.orientation_covariance[0] = 1.0e-4;
      message.orientation_covariance[4] = 1.0e-4;
      message.orientation_covariance[8] = 4.0e-4;
    } else {
      message.orientation.w = 1.0;
      message.orientation_covariance[0] = -1.0;
    }
    message.angular_velocity_covariance[0] = 1.0e-5;
    message.angular_velocity_covariance[4] = 1.0e-5;
    message.angular_velocity_covariance[8] = 1.0e-5;
    message.linear_acceleration_covariance[0] = 1.0e-3;
    message.linear_acceleration_covariance[4] = 1.0e-3;
    message.linear_acceleration_covariance[8] = 1.0e-3;
    imu_publisher_->publish(message);

    sensor_msgs::msg::MagneticField magnetic;
    magnetic.header = message.header;
    constexpr double kMilliGaussToTesla = 1.0e-7;
    magnetic.magnetic_field.x = mx * kMilliGaussToTesla;
    magnetic.magnetic_field.y = my * kMilliGaussToTesla;
    magnetic.magnetic_field.z = mz * kMilliGaussToTesla;
    mag_publisher_->publish(magnetic);
  }

  void process_frames()
  {
    std::size_t cursor = 0;
    while (cursor + kHeaderSize + 1 <= receive_buffer_.size()) {
      if (receive_buffer_[cursor] != kFrameHead) {
        ++cursor;
        continue;
      }
      const uint8_t type = receive_buffer_[cursor + 1];
      const std::size_t payload_size = receive_buffer_[cursor + 2];
      if (payload_size > kMaxPayloadSize) {
        ++cursor;
        continue;
      }
      const std::size_t frame_size = kHeaderSize + payload_size + 1U;
      if (cursor + frame_size > receive_buffer_.size()) {
        break;
      }
      const uint8_t * frame = receive_buffer_.data() + cursor;
      const uint8_t * payload = frame + kHeaderSize;
      const uint16_t expected_crc16 =
        (static_cast<uint16_t>(frame[5]) << 8U) | frame[6];
      if (frame[4] != crc8_maxim(frame, 4) ||
        expected_crc16 != crc16_ccitt(payload, payload_size) ||
        frame[frame_size - 1] != kFrameEnd)
      {
        ++invalid_frames_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Rejected invalid H30 frame; cumulative invalid frames=%llu",
          static_cast<unsigned long long>(invalid_frames_));
        ++cursor;
        continue;
      }

      if (type == kAhrsType && payload_size == 48U) {
        parse_ahrs(payload);
        serial_->mark_valid_frame();
      } else if (type == kImuType && payload_size == 56U) {
        parse_imu(payload);
        serial_->mark_valid_frame();
      }
      cursor += frame_size;
    }
    if (cursor > 0) {
      receive_buffer_.erase(
        receive_buffer_.begin(), receive_buffer_.begin() + static_cast<std::ptrdiff_t>(cursor));
    }
  }

  void read_serial()
  {
    if (!serial_->service()) {
      if (serial_->take_connection_reset()) {
        reset_parser_state();
      }
      return;
    }
    if (serial_->take_connection_reset()) {
      reset_parser_state();
    }
    const std::size_t count = serial_->read(read_buffer_.data(), read_buffer_.size());
    if (serial_->take_connection_reset()) {
      reset_parser_state();
      return;
    }
    if (count > 0) {
      receive_buffer_.insert(
        receive_buffer_.end(), read_buffer_.begin(),
        read_buffer_.begin() + static_cast<std::ptrdiff_t>(count));
    }

    if (receive_buffer_.size() > kMaxBufferedBytes) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "H30 receive buffer overflow; resynchronising");
      receive_buffer_.erase(
        receive_buffer_.begin(),
        receive_buffer_.end() - static_cast<std::ptrdiff_t>(kMaxBufferedBytes / 2));
    }
    process_frames();
  }

  void reset_parser_state()
  {
    receive_buffer_.clear();
    orientation_valid_ = false;
  }

  int baudrate_{};
  std::string port_name_;
  std::string frame_id_;
  std::string imu_topic_;
  std::string mag_topic_;
  bool convert_to_ros_flu_{true};
  std::chrono::milliseconds reconnect_period_{1000};
  std::chrono::milliseconds data_timeout_{2000};
  std::chrono::milliseconds orientation_timeout_{200};
  std::chrono::steady_clock::time_point last_orientation_time_{};

  std::unique_ptr<sensor_driver_msgs::ReconnectingSerial> serial_;
  std::array<uint8_t, kMaxReadSize> read_buffer_{};
  std::vector<uint8_t> receive_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
  tf2::Quaternion orientation_{0.0, 0.0, 0.0, 1.0};
  bool orientation_valid_{false};
  uint64_t invalid_frames_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ImuComNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("imu_com"), "Driver startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
