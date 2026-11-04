#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_driver_msgs/reconnecting_serial.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
constexpr std::size_t kMaxLineSize = 1024;
constexpr std::size_t kMaxBufferedBytes = 4096;
constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kKnotsToMetersPerSecond = 0.514444444444;

std::vector<std::string> split(const std::string & text, const char separator)
{
  std::vector<std::string> fields;
  std::size_t start = 0;
  while (start <= text.size()) {
    const std::size_t end = text.find(separator, start);
    fields.emplace_back(text.substr(start, end - start));
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  return fields;
}

bool parse_double(const std::string & text, double & value)
{
  if (text.empty()) {
    return false;
  }
  try {
    std::size_t parsed = 0;
    value = std::stod(text, &parsed);
    return parsed == text.size() && std::isfinite(value);
  } catch (const std::exception &) {
    return false;
  }
}

bool parse_int(const std::string & text, int & value)
{
  if (text.empty()) {
    return false;
  }
  try {
    std::size_t parsed = 0;
    value = std::stoi(text, &parsed);
    return parsed == text.size();
  } catch (const std::exception &) {
    return false;
  }
}

bool valid_nmea_checksum(const std::string & line)
{
  if (line.size() < 5 || line.front() != '$') {
    return false;
  }
  const std::size_t star = line.find('*');
  if (star == std::string::npos || star + 3 != line.size()) {
    return false;
  }
  uint8_t checksum = 0;
  for (std::size_t i = 1; i < star; ++i) {
    checksum ^= static_cast<uint8_t>(line[i]);
  }
  try {
    const int expected = std::stoi(line.substr(star + 1, 2), nullptr, 16);
    return checksum == static_cast<uint8_t>(expected);
  } catch (const std::exception &) {
    return false;
  }
}

bool nmea_coordinate(
  const std::string & value_text, const std::string & hemisphere, double & degrees)
{
  double raw = 0.0;
  if (!parse_double(value_text, raw) || raw <= 0.0) {
    return false;
  }
  const double whole_degrees = std::floor(raw / 100.0);
  const double minutes = raw - whole_degrees * 100.0;
  if (minutes >= 60.0) {
    return false;
  }
  degrees = whole_degrees + minutes / 60.0;
  if (hemisphere == "S" || hemisphere == "W") {
    degrees = -degrees;
  } else if (hemisphere != "N" && hemisphere != "E") {
    return false;
  }
  return true;
}
}  // namespace

// Receive-only WTRTK/NMEA driver. It never sends configuration commands to the receiver.
class WtrtkGnssNode : public rclcpp::Node
{
public:
  WtrtkGnssNode()
  : Node("wtrtk_gnss_node")
  {
    baudrate_ = declare_parameter<int>("baudrate", 230400);
    device_ = declare_parameter<std::string>("device", "/dev/ttyUSB0");
    frame_id_ = declare_parameter<std::string>("frame_id", "gnss_link");
    fix_topic_ = declare_parameter<std::string>(
      "fix_topic", "/sensing/gnss/ublox/nav_sat_fix");
    legacy_fix_topic_ = declare_parameter<std::string>(
      "legacy_fix_topic", "/Sensor_msgs/INS/NavSatFix_BYINS_gnss");
    reconnect_period_ = std::chrono::milliseconds(std::max<int64_t>(
      100, declare_parameter<int64_t>("reconnect_period_ms", 1000)));
    data_timeout_ = std::chrono::milliseconds(std::max<int64_t>(
      100, declare_parameter<int64_t>("data_timeout_ms", 2000)));

    // Autoware's gnss_poser requests RELIABLE input.  A BEST_EFFORT
    // SensorDataQoS publisher is incompatible with that subscription and
    // silently disconnects the positioning chain.  RELIABLE publishers also
    // remain compatible with BEST_EFFORT diagnostic subscribers.
    const auto navigation_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    fix_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      fix_topic_, navigation_qos);
    legacy_fix_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      legacy_fix_topic_, navigation_qos);
    orientation_publisher_ = create_publisher<
      autoware_sensing_msgs::msg::GnssInsOrientationStamped>(
      "/Sensor_msgs/INS/GnssInsOrientation", navigation_qos);
    velocity_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/sensing/gnss/velocity", rclcpp::SensorDataQoS());
    raw_publisher_ = create_publisher<std_msgs::msg::String>(
      "/sensing/gnss/wtrtk/raw_sentence", 10);

    serial_ = std::make_unique<sensor_driver_msgs::ReconnectingSerial>(
      sensor_driver_msgs::ReconnectingSerial::Options{
        device_, static_cast<uint32_t>(baudrate_), std::chrono::milliseconds(50),
        reconnect_period_, data_timeout_},
      [this](const std::string & message) {
        RCLCPP_ERROR(get_logger(), "WTRTK offline: %s", message.c_str());
      },
      [this](const std::string & message) {
        RCLCPP_INFO(get_logger(), "WTRTK recovered: %s", message.c_str());
      });

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(5ms, std::bind(&WtrtkGnssNode::read_serial, this));
    RCLCPP_INFO(
      get_logger(), "WTRTK GNSS driver started: device=%s baud=%d receive-only",
      device_.c_str(), baudrate_);
  }

private:
  void publish_fix(sensor_msgs::msg::NavSatFix & fix)
  {
    fix.header.stamp = now();
    fix.header.frame_id = frame_id_;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
      sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
      sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO |
      sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
    fix_publisher_->publish(fix);
    legacy_fix_publisher_->publish(fix);
  }

  void publish_no_fix()
  {
    const auto current = std::chrono::steady_clock::now();
    if (last_no_fix_publish_.time_since_epoch().count() != 0 &&
      current - last_no_fix_publish_ < std::chrono::milliseconds(100))
    {
      return;
    }
    last_no_fix_publish_ = current;
    sensor_msgs::msg::NavSatFix fix;
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    fix.latitude = std::numeric_limits<double>::quiet_NaN();
    fix.longitude = std::numeric_limits<double>::quiet_NaN();
    fix.altitude = std::numeric_limits<double>::quiet_NaN();
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    publish_fix(fix);
  }

  void parse_wtrtk(const std::vector<std::string> & fields)
  {
    // Official WTRTK-4G order: dX,dY,dZ,dR,pitch,roll,heading,RTK state,...
    if (fields.size() < 9) {
      return;
    }
    double pitch_deg = 0.0;
    double roll_deg = 0.0;
    double heading_deg = 0.0;
    int rtk_state = 0;
    if (!parse_double(fields[5], pitch_deg) || !parse_double(fields[6], roll_deg) ||
      !parse_double(fields[7], heading_deg) || !parse_int(fields[8], rtk_state))
    {
      return;
    }

    const double yaw_enu = M_PI_2 - heading_deg * kDegreesToRadians;
    tf2::Quaternion quaternion;
    quaternion.setRPY(
      roll_deg * kDegreesToRadians, pitch_deg * kDegreesToRadians, yaw_enu);
    quaternion.normalize();
    autoware_sensing_msgs::msg::GnssInsOrientationStamped orientation;
    orientation.header.stamp = now();
    orientation.header.frame_id = frame_id_;
    orientation.orientation.orientation.x = quaternion.x();
    orientation.orientation.orientation.y = quaternion.y();
    orientation.orientation.orientation.z = quaternion.z();
    orientation.orientation.orientation.w = quaternion.w();
    orientation.orientation.rmse_rotation_x = -1.0F;
    orientation.orientation.rmse_rotation_y = -1.0F;
    orientation.orientation.rmse_rotation_z = -1.0F;
    orientation_publisher_->publish(orientation);

    // $WTRTK has no latitude/longitude fields. Only GGA can produce a valid NavSatFix.
    if (rtk_state == 0 &&
      std::chrono::steady_clock::now() - last_gga_time_ > std::chrono::seconds(1))
    {
      publish_no_fix();
    }
  }

  void parse_gga(const std::vector<std::string> & fields)
  {
    if (fields.size() < 10) {
      return;
    }
    int quality = 0;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double hdop = 0.0;
    if (!parse_int(fields[6], quality)) {
      return;
    }
    if (quality <= 0 || !nmea_coordinate(fields[2], fields[3], latitude) ||
      !nmea_coordinate(fields[4], fields[5], longitude) ||
      !parse_double(fields[9], altitude))
    {
      publish_no_fix();
      return;
    }
    parse_double(fields[8], hdop);
    sensor_msgs::msg::NavSatFix fix;
    fix.status.status = quality == 4 ? sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX :
      sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.latitude = latitude;
    fix.longitude = longitude;
    fix.altitude = altitude;
    if (hdop > 0.0) {
      const double horizontal_variance = std::pow(hdop * 1.5, 2.0);
      fix.position_covariance[0] = horizontal_variance;
      fix.position_covariance[4] = horizontal_variance;
      fix.position_covariance[8] = horizontal_variance * 4.0;
      fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    }
    last_gga_time_ = std::chrono::steady_clock::now();
    publish_fix(fix);
  }

  void parse_rmc(const std::vector<std::string> & fields)
  {
    if (fields.size() < 9 || fields[2] != "A") {
      return;
    }
    double speed_knots = 0.0;
    double course_deg = 0.0;
    if (!parse_double(fields[7], speed_knots) || !parse_double(fields[8], course_deg)) {
      return;
    }
    const double speed = speed_knots * kKnotsToMetersPerSecond;
    const double yaw_enu = M_PI_2 - course_deg * kDegreesToRadians;
    geometry_msgs::msg::TwistStamped velocity;
    velocity.header.stamp = now();
    velocity.header.frame_id = "map";
    velocity.twist.linear.x = speed * std::cos(yaw_enu);
    velocity.twist.linear.y = speed * std::sin(yaw_enu);
    velocity_publisher_->publish(velocity);
  }

  void process_line(std::string line)
  {
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
      line.pop_back();
    }
    if (line.size() > kMaxLineSize || !valid_nmea_checksum(line)) {
      ++invalid_lines_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejected WTRTK/NMEA line with invalid length/checksum; count=%llu",
        static_cast<unsigned long long>(invalid_lines_));
      return;
    }
    serial_->mark_valid_frame();
    std_msgs::msg::String raw;
    raw.data = line;
    raw_publisher_->publish(raw);

    const std::size_t star = line.find('*');
    const auto fields = split(line.substr(1, star - 1), ',');
    if (fields.empty()) {
      return;
    }
    if (fields[0] == "WTRTK") {
      parse_wtrtk(fields);
    } else if (fields[0] == "GPGGA" || fields[0] == "GNGGA") {
      parse_gga(fields);
    } else if (fields[0] == "GPRMC" || fields[0] == "GNRMC") {
      parse_rmc(fields);
    }
  }

  void process_lines()
  {
    while (true) {
      const std::size_t end = receive_buffer_.find('\n');
      if (end == std::string::npos) {
        break;
      }
      std::string line = receive_buffer_.substr(0, end);
      receive_buffer_.erase(0, end + 1);
      process_line(std::move(line));
    }
  }

  void read_serial()
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
    receive_buffer_.append(reinterpret_cast<const char *>(read_buffer_.data()), count);
    if (receive_buffer_.size() > kMaxBufferedBytes) {
      const std::size_t next_start = receive_buffer_.rfind('$');
      receive_buffer_ = next_start == std::string::npos ? std::string{} :
        receive_buffer_.substr(next_start);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "WTRTK receive buffer overflow; resynchronised");
    }
    process_lines();
  }

  int baudrate_{};
  std::string device_;
  std::string frame_id_;
  std::string fix_topic_;
  std::string legacy_fix_topic_;
  std::chrono::milliseconds reconnect_period_{1000};
  std::chrono::milliseconds data_timeout_{2000};
  std::chrono::steady_clock::time_point last_gga_time_{};
  std::chrono::steady_clock::time_point last_no_fix_publish_{};
  std::unique_ptr<sensor_driver_msgs::ReconnectingSerial> serial_;
  std::array<uint8_t, 4096> read_buffer_{};
  std::string receive_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr legacy_fix_publisher_;
  rclcpp::Publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr
  orientation_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_publisher_;
  uint64_t invalid_lines_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<WtrtkGnssNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("wtrtk_gnss"), "Driver startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
