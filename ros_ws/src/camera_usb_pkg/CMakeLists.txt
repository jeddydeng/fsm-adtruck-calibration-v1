cmake_minimum_required(VERSION 3.16)
project(camera_usb_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(traffic_light_camera_bridge_node src/traffic_light_camera_bridge_node.cpp)
ament_target_dependencies(traffic_light_camera_bridge_node rclcpp sensor_msgs)

install(TARGETS traffic_light_camera_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS
  scripts/capture_calibration_images.py
  scripts/calibrate_intrinsics.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES CALIBRATION_GUIDE.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
