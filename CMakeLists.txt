cmake_minimum_required(VERSION 3.8)
project(sealien_mavlink_joy_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AdcRaw.msg"
  DEPENDENCIES std_msgs
)

include_directories(include)
# MAVLink headers use anonymous structs; treat as system includes to avoid -Wpedantic noise
include_directories(SYSTEM
  third_party/mavlink
  third_party/mavlink/sealien_rc
)

set(MJ_DEPENDENCIES
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(mavlink_adc_bridge
  src/mavlink_adc_bridge_node.cpp
  src/mavlink_adc_bridge_core.cpp
)
add_executable(adc_joy_mapper
  src/adc_joy_mapper_node.cpp
  src/adc_joy_mapper_core.cpp
)
add_executable(mavlink_joy_bridge
  src/mavlink_joy_bridge_node.cpp
  src/mavlink_joy_bridge_core.cpp
)
add_executable(mavlink_rc_udp_send tools/mavlink_rc_udp_send.cpp)
add_executable(mavlink_io_udp_send tools/mavlink_io_udp_send.cpp)
add_executable(io_joy_bridge
  src/io_joy_bridge_node.cpp
  src/io_joy_bridge_core.cpp
)

target_link_libraries(mavlink_adc_bridge pthread)
target_link_libraries(adc_joy_mapper pthread)
target_link_libraries(mavlink_joy_bridge pthread)
target_link_libraries(mavlink_rc_udp_send pthread)
target_link_libraries(mavlink_io_udp_send pthread)
target_link_libraries(io_joy_bridge pthread)

ament_target_dependencies(mavlink_adc_bridge ${MJ_DEPENDENCIES})
ament_target_dependencies(io_joy_bridge ${MJ_DEPENDENCIES})
ament_target_dependencies(adc_joy_mapper ${MJ_DEPENDENCIES})
ament_target_dependencies(mavlink_joy_bridge ${MJ_DEPENDENCIES})

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mavlink_adc_bridge "${cpp_typesupport_target}")
target_link_libraries(adc_joy_mapper "${cpp_typesupport_target}")

install(TARGETS
  mavlink_adc_bridge
  adc_joy_mapper
  mavlink_joy_bridge
  io_joy_bridge
  mavlink_rc_udp_send
  mavlink_io_udp_send
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY mavlink/ DESTINATION share/${PROJECT_NAME}/mavlink)
install(DIRECTORY web/ DESTINATION share/${PROJECT_NAME}/web)

install(PROGRAMS
  scripts/rc_web_sim_server.py
  scripts/start_rc_web_sim.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
