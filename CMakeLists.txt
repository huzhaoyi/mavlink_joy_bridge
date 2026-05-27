cmake_minimum_required(VERSION 3.8)
project(sealien_mavlink_joy_bridge)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)
include_directories(SYSTEM
  third_party/mavlink
  third_party/mavlink/sealien_mavlink
)

add_executable(mixed_io_joy_bridge
  src/mixed_io_joy_bridge_node.cpp
  src/mixed_io_joy_bridge_core.cpp
)
add_executable(mavlink_mixed_io_udp_send tools/mavlink_mixed_io_udp_send.cpp)

target_link_libraries(mixed_io_joy_bridge pthread)
target_link_libraries(mavlink_mixed_io_udp_send pthread)

ament_target_dependencies(mixed_io_joy_bridge rclcpp sensor_msgs)

install(TARGETS
  mixed_io_joy_bridge
  mavlink_mixed_io_udp_send
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY web/ DESTINATION share/${PROJECT_NAME}/web)

install(PROGRAMS
  scripts/rc_web_sim_server.py
  scripts/start_rc_web_sim.sh
  scripts/e2e_test.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
