#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// This bridge is reserved for the independent Raspberry Pi camera pipeline.
namespace camera_usb_pkg
{

class TrafficLightCameraBridge final : public rclcpp::Node
{
public:
  TrafficLightCameraBridge()
  : Node("traffic_light_camera_bridge"), started_at_(SteadyClock::now())
  {
    const auto input_image = declare_parameter<std::string>(
      "input_image_topic", "/traffic_light_front/image_raw");
    const auto output_image = declare_parameter<std::string>(
      "output_image_topic", "/sensing/camera/traffic_light_front/image_raw");
    const auto input_info = declare_parameter<std::string>(
      "input_camera_info_topic", "/traffic_light_front/camera_info");
    const auto output_info = declare_parameter<std::string>(
      "output_camera_info_topic", "/sensing/camera/traffic_light_front/camera_info");
    if (
      input_image.empty() || output_image.empty() || input_info.empty() || output_info.empty() ||
      input_image == output_image || input_info == output_info)
    {
      throw std::invalid_argument(
              "traffic-light camera bridge topics must be non-empty and distinct");
    }

    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(output_image, qos);
    info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(output_info, qos);
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_image, qos,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr message) {
        image_publisher_->publish(*message);
        mark_received(image_health_, "traffic-light camera image");
      });
    info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      input_info, qos,
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        info_publisher_->publish(*message);
        mark_received(info_health_, "traffic-light camera info");
      });
    watchdog_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&TrafficLightCameraBridge::check_health, this));
    RCLCPP_INFO(
      get_logger(), "Traffic-light camera bridge: %s -> %s, %s -> %s", input_image.c_str(),
      output_image.c_str(), input_info.c_str(), output_info.c_str());
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  struct StreamHealth
  {
    SteadyClock::time_point last_received{};
    SteadyClock::time_point last_error{};
    bool received{false};
    bool offline{true};
  };

  void mark_received(StreamHealth & health, const char * name)
  {
    health.last_received = SteadyClock::now();
    health.received = true;
    if (health.offline) {
      RCLCPP_INFO(get_logger(), "%s stream recovered", name);
      health.offline = false;
    }
  }

  void check_stream(
    StreamHealth & health, const char * name, const SteadyClock::time_point now)
  {
    const auto newest = health.received ? health.last_received : started_at_;
    if (now - newest <= std::chrono::seconds(3)) {
      return;
    }
    health.offline = true;
    if (health.last_error.time_since_epoch().count() == 0 ||
      now - health.last_error >= std::chrono::seconds(5))
    {
      RCLCPP_ERROR(get_logger(), "%s stream has produced no data for 3 seconds", name);
      health.last_error = now;
    }
  }

  void check_health()
  {
    const auto now = SteadyClock::now();
    check_stream(image_health_, "traffic-light camera image", now);
    check_stream(info_health_, "traffic-light camera info", now);
  }

  const SteadyClock::time_point started_at_;
  StreamHealth image_health_;
  StreamHealth info_health_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace camera_usb_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camera_usb_pkg::TrafficLightCameraBridge>());
  rclcpp::shutdown();
  return 0;
}
