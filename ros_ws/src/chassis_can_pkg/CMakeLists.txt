cmake_minimum_required(VERSION 3.16)
project(chassis_can_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_driver_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(chassis_can_driver_node src/chassis_can_driver_node.cpp)
ament_target_dependencies(chassis_can_driver_node rclcpp sensor_driver_msgs std_msgs std_srvs)

install(TARGETS chassis_can_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
