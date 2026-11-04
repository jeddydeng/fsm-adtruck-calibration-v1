cmake_minimum_required(VERSION 3.16)
project(ultrasonic_radar12)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_driver_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ultrasonic_radar12_node src/get.cpp)
ament_target_dependencies(
  ultrasonic_radar12_node rclcpp sensor_driver_msgs sensor_msgs serial std_msgs
)

install(TARGETS ultrasonic_radar12_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
