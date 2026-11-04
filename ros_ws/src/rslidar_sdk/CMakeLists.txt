cmake_minimum_required(VERSION 3.16)
project(rslidar_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rslidar_msg REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_library(PCAP_LIBRARY pcap REQUIRED)

add_executable(rslidar_sdk_node
  node/rslidar_sdk_node.cpp
  src/manager/node_manager.cpp
)
target_compile_definitions(rslidar_sdk_node PRIVATE ROS2_FOUND POINT_TYPE_XYZIRT)
target_compile_options(rslidar_sdk_node PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_include_directories(rslidar_sdk_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/rs_driver/src
)
ament_target_dependencies(rslidar_sdk_node
  ament_index_cpp rclcpp rslidar_msg sensor_msgs std_msgs
)
target_link_libraries(rslidar_sdk_node yaml-cpp Threads::Threads ${PCAP_LIBRARY})

install(TARGETS rslidar_sdk_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz config DESTINATION share/${PROJECT_NAME})

ament_package()
