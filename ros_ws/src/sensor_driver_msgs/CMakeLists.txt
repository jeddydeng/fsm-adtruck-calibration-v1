cmake_minimum_required(VERSION 3.16)
project(sensor_driver_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)

# Match rosidl's package-scoped include directory on both Humble and Jazzy.
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UltrasonicRadar12.msg"
  "msg/UltrasonicRadar20.msg"
  "msg/InsByins.msg"
  "msg/GnssGpdop.msg"
  "msg/GnssGpfpd.msg"
  "msg/GnssGpgga.msg"
  "msg/GnssGpgst.msg"
  "msg/GnssGphdt.msg"
  "msg/GnssGpzda.msg"
  "msg/GnssPashr.msg"
  "msg/ImuHwt9073can.msg"
  "msg/ChassisCanSended.msg"
  "msg/ChassisCanRecived.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime serial)
ament_package()
