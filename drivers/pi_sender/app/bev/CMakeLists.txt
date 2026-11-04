cmake_minimum_required(VERSION 3.16)
project(w102_udp_gateway VERSION 0.1.0 LANGUAGES CXX)

# The production release intentionally does not ship development tests.  Keep
# a plain `cmake .. && cmake --build .` usable after cloning the repository.
option(BUILD_TESTING "Build development tests" OFF)
include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc imgcodecs videoio calib3d)

add_library(w102_protocol
    src/udp_protocol.cpp
)
target_include_directories(w102_protocol PUBLIC include)
target_compile_options(w102_protocol PRIVATE -Wall -Wextra -Wpedantic)

add_library(w102_bev
    src/bev_composer.cpp
    src/adaptive_illumination.cpp
)
target_include_directories(w102_bev PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(w102_bev PUBLIC ${OpenCV_LIBS})
target_compile_options(w102_bev PRIVATE -Wall -Wextra -Wpedantic)

add_library(w102_vehicle_can
    src/vehicle_can.cpp
)
target_include_directories(w102_vehicle_can PUBLIC include)
target_link_libraries(w102_vehicle_can PUBLIC w102_protocol pthread)
target_compile_options(w102_vehicle_can PRIVATE -Wall -Wextra -Wpedantic)

add_executable(w102_camera_gateway
    src/camera_gateway.cpp
)
target_link_libraries(w102_camera_gateway PRIVATE w102_protocol w102_bev w102_vehicle_can ${OpenCV_LIBS} pthread)
target_compile_options(w102_camera_gateway PRIVATE -Wall -Wextra -Wpedantic)

add_executable(w102_udp_receiver
    src/udp_receiver.cpp
)
target_link_libraries(w102_udp_receiver PRIVATE w102_protocol pthread)
target_compile_options(w102_udp_receiver PRIVATE -Wall -Wextra -Wpedantic)

add_executable(w102_udp_control
    src/udp_control.cpp
)
target_link_libraries(w102_udp_control PRIVATE w102_protocol)
target_compile_options(w102_udp_control PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS w102_camera_gateway w102_udp_receiver w102_udp_control
        RUNTIME DESTINATION bin)
if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
  add_executable(w102_protocol_test tests/protocol_test.cpp)
  target_link_libraries(w102_protocol_test PRIVATE w102_protocol)
  target_compile_options(w102_protocol_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME w102_protocol_test COMMAND w102_protocol_test)

  add_executable(w102_bev_composer_test tests/bev_composer_test.cpp)
  target_link_libraries(w102_bev_composer_test PRIVATE w102_bev)
  target_compile_options(w102_bev_composer_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME w102_bev_composer_test COMMAND w102_bev_composer_test)

  add_executable(w102_adaptive_illumination_test
                 tests/adaptive_illumination_test.cpp)
  target_link_libraries(w102_adaptive_illumination_test PRIVATE w102_bev)
  target_compile_options(w102_adaptive_illumination_test
                         PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME w102_adaptive_illumination_test
           COMMAND w102_adaptive_illumination_test)

  add_executable(w102_provisional_four_camera_test
                 tests/provisional_four_camera_test.cpp)
  target_link_libraries(w102_provisional_four_camera_test PRIVATE w102_bev)
  target_compile_definitions(
      w102_provisional_four_camera_test
      PRIVATE W102_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_options(w102_provisional_four_camera_test
                         PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME w102_provisional_four_camera_test
           COMMAND w102_provisional_four_camera_test)

  add_executable(w102_vehicle_can_test tests/vehicle_can_test.cpp)
  target_link_libraries(w102_vehicle_can_test PRIVATE w102_vehicle_can)
  target_compile_options(w102_vehicle_can_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME w102_vehicle_can_test COMMAND w102_vehicle_can_test)
endif()
