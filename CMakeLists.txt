cmake_minimum_required(VERSION 3.8)
project(sealien_mavlink_joy_bridge)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/MixedIoRaw.msg
  DEPENDENCIES std_msgs
)

include_directories(include)
include_directories(SYSTEM
  third_party/mavlink
  third_party/mavlink/sealien_mavlink
)

add_executable(mavlink_mixed_io_receiver
  src/mavlink_mixed_io_receiver_node.cpp
  src/mavlink_mixed_io_receiver.cpp
)
ament_target_dependencies(mavlink_mixed_io_receiver rclcpp std_msgs)
target_link_libraries(mavlink_mixed_io_receiver pthread)

add_executable(adc_to_joy_mapper
  src/adc_to_joy_mapper_node.cpp
  src/adc_to_joy_mapper.cpp
)
ament_target_dependencies(adc_to_joy_mapper rclcpp std_msgs sensor_msgs)

add_executable(mavlink_mixed_io_udp_send tools/mavlink_mixed_io_udp_send.cpp)
target_link_libraries(mavlink_mixed_io_udp_send pthread)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(mavlink_mixed_io_receiver "${cpp_typesupport_target}")
target_link_libraries(adc_to_joy_mapper "${cpp_typesupport_target}")

install(TARGETS
  mavlink_mixed_io_receiver
  adc_to_joy_mapper
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
