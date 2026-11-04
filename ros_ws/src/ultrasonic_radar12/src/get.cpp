#include <rclcpp/rclcpp.hpp>
#include <sensor_driver_msgs/reconnecting_serial.hpp>
#include <sensor_driver_msgs/msg/ultrasonic_radar12.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/int16.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{
constexpr std::size_t kActiveFrameSize = 18;
constexpr std::size_t kMaxReadSize = 4096;
constexpr std::size_t kMaxBufferedBytes = 4096;
}  // namespace

// Receive-only driver for the WL300D active-report stream.
class UltrasonicRadar12 : public rclcpp::Node
{
public:
  UltrasonicRadar12()
  : Node("ultrasonic_radar12_node")
  {
    device_ = declare_parameter<std::string>("device", "/dev/ttyUSB1");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    reconnect_period_ = std::chrono::milliseconds(std::max<int64_t>(
      100, declare_parameter<int64_t>("reconnect_period_ms", 1000)));
    data_timeout_ = std::chrono::milliseconds(std::max<int64_t>(
      100, declare_parameter<int64_t>("data_timeout_ms", 2000)));
    danger_topic_ = declare_parameter<std::string>("danger_topic", "/Control_msgs/danger_flag");
    danger_publisher_ = create_publisher<std_msgs::msg::Int16>(danger_topic_, 10);
    ranges_publisher_ = create_publisher<sensor_driver_msgs::msg::UltrasonicRadar12>(
      "/Sensor_msgs/UltrasonicRadar/UltrasonicRadar12", rclcpp::SensorDataQoS());
    for (std::size_t i = 0; i < range_publishers_.size(); ++i) {
      const std::string topic = "/sensing/ultrasonic/channel_" +
        (i + 1 < 10 ? std::string("0") : std::string()) + std::to_string(i + 1) + "/range";
      range_publishers_[i] = create_publisher<sensor_msgs::msg::Range>(
        topic, rclcpp::SensorDataQoS());
    }

    serial_ = std::make_unique<sensor_driver_msgs::ReconnectingSerial>(
      sensor_driver_msgs::ReconnectingSerial::Options{
        device_, static_cast<uint32_t>(baudrate_), std::chrono::milliseconds(20),
        reconnect_period_, data_timeout_},
      [this](const std::string & message) {
        RCLCPP_ERROR(get_logger(), "WL300D offline: %s", message.c_str());
      },
      [this](const std::string & message) {
        RCLCPP_INFO(get_logger(), "WL300D recovered: %s", message.c_str());
      });

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(5ms, std::bind(&UltrasonicRadar12::service_serial, this));
    RCLCPP_INFO(
      get_logger(), "WL300D driver started: device=%s baud=%d active receive-only",
      device_.c_str(), baudrate_);
  }

private:
  void publish_ranges(const std::array<uint16_t, 12> & centimeters)
  {
    sensor_driver_msgs::msg::UltrasonicRadar12 legacy;
    legacy.header.stamp = now();
    const auto legacy_value = [](const uint16_t value) {
      return static_cast<uint8_t>(std::min<uint16_t>(value, 255U));
    };
    legacy.dat1 = legacy_value(centimeters[0]);
    legacy.dat2 = legacy_value(centimeters[1]);
    legacy.dat3 = legacy_value(centimeters[2]);
    legacy.dat4 = legacy_value(centimeters[3]);
    legacy.dat5 = legacy_value(centimeters[4]);
    legacy.dat6 = legacy_value(centimeters[5]);
    legacy.dat7 = legacy_value(centimeters[6]);
    legacy.dat8 = legacy_value(centimeters[7]);
    legacy.dat9 = legacy_value(centimeters[8]);
    legacy.dat10 = legacy_value(centimeters[9]);
    legacy.dat11 = legacy_value(centimeters[10]);
    legacy.dat12 = legacy_value(centimeters[11]);
    ranges_publisher_->publish(legacy);

    for (std::size_t i = 0; i < centimeters.size(); ++i) {
      sensor_msgs::msg::Range range;
      range.header.stamp = legacy.header.stamp;
      range.header.frame_id = "ultrasonic_" +
        (i + 1 < 10 ? std::string("0") : std::string()) + std::to_string(i + 1);
      range.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
      range.field_of_view = 0.5F;
      range.min_range = 0.02F;
      range.max_range = 2.55F;
      const uint16_t value = centimeters[i];
      range.range = (value == 0U || value == 0xFFU || value == 0xFFFFU) ?
        std::numeric_limits<float>::quiet_NaN() : static_cast<float>(value) / 100.0F;
      range_publishers_[i]->publish(range);
    }

    const auto within = [&centimeters](const std::size_t index, const uint16_t threshold) {
      const uint16_t value = centimeters[index];
      return value > 0U && value != 0xFFU && value != 0xFFFFU && value <= threshold;
    };
    std_msgs::msg::Int16 danger;
    if (within(0, 70) || within(1, 55) || within(2, 40) || within(3, 40) ||
      within(4, 40) || within(5, 40) || within(6, 40) || within(7, 40) ||
      within(8, 40) || within(9, 40) || within(10, 55) || within(11, 70))
    {
      danger.data = 2;
    } else if (within(0, 200) || within(1, 150) || within(10, 150) || within(11, 200)) {
      danger.data = 1;
    } else {
      danger.data = 0;
    }
    danger_publisher_->publish(danger);
  }

  void parse_active_frames()
  {
    while (receive_buffer_.size() >= 2) {
      if (receive_buffer_[0] != 0x55U || receive_buffer_[1] != 0x54U) {
        receive_buffer_.erase(receive_buffer_.begin());
        continue;
      }
      if (receive_buffer_.size() < kActiveFrameSize) {
        return;
      }
      uint8_t checksum = 0;
      for (std::size_t i = 2; i <= 16; ++i) {
        checksum ^= receive_buffer_[i];
      }
      if (receive_buffer_[2] != 0x0CU || receive_buffer_[15] != 0xFFU ||
        checksum != receive_buffer_[17])
      {
        ++invalid_frames_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Rejected invalid WL300D active frame; count=%llu",
          static_cast<unsigned long long>(invalid_frames_));
        receive_buffer_.erase(receive_buffer_.begin());
        continue;
      }
      std::array<uint16_t, 12> distances{};
      for (std::size_t i = 0; i < distances.size(); ++i) {
        distances[i] = receive_buffer_[i + 3];
      }
      serial_->mark_valid_frame();
      publish_ranges(distances);
      receive_buffer_.erase(
        receive_buffer_.begin(), receive_buffer_.begin() + kActiveFrameSize);
    }
  }

  void service_serial()
  {
    if (!serial_->service()) {
      if (serial_->take_connection_reset()) {
        receive_buffer_.clear();
      }
      return;
    }
    if (serial_->take_connection_reset()) {
      receive_buffer_.clear();
    }
    const std::size_t count = serial_->read(read_buffer_.data(), read_buffer_.size());
    if (serial_->take_connection_reset()) {
      receive_buffer_.clear();
      return;
    }
    if (count > 0) {
      receive_buffer_.insert(
        receive_buffer_.end(), read_buffer_.begin(),
        read_buffer_.begin() + static_cast<std::ptrdiff_t>(count));
    }

    if (receive_buffer_.size() > kMaxBufferedBytes) {
      receive_buffer_.erase(
        receive_buffer_.begin(),
        receive_buffer_.end() - static_cast<std::ptrdiff_t>(kMaxBufferedBytes / 2));
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "WL300D receive buffer overflow; resynchronised");
    }
    parse_active_frames();
  }

  int baudrate_{};
  std::string device_;
  std::string danger_topic_;
  std::chrono::milliseconds reconnect_period_{1000};
  std::chrono::milliseconds data_timeout_{2000};
  std::unique_ptr<sensor_driver_msgs::ReconnectingSerial> serial_;
  std::array<uint8_t, kMaxReadSize> read_buffer_{};
  std::vector<uint8_t> receive_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr danger_publisher_;
  rclcpp::Publisher<sensor_driver_msgs::msg::UltrasonicRadar12>::SharedPtr ranges_publisher_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, 12> range_publishers_;
  uint64_t invalid_frames_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<UltrasonicRadar12>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ultrasonic_radar12"), "Driver startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
