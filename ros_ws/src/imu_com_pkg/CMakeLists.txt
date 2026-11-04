cmake_minimum_required(VERSION 3.16)
project(imu_com_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_driver_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(tf2 REQUIRED)

add_executable(imu_com_driver_node src/imu_com_driver_node.cpp)
ament_target_dependencies(imu_com_driver_node
  rclcpp sensor_driver_msgs sensor_msgs serial tf2
)

install(TARGETS imu_com_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
