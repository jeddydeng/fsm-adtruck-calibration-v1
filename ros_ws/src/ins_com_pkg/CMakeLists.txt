cmake_minimum_required(VERSION 3.16)
project(ins_com_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(autoware_sensing_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_driver_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(wtrtk_gnss_node src/wtrtk_gnss_node.cpp)
ament_target_dependencies(wtrtk_gnss_node
  autoware_sensing_msgs geometry_msgs rclcpp sensor_driver_msgs sensor_msgs serial std_msgs tf2
)

install(TARGETS wtrtk_gnss_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
