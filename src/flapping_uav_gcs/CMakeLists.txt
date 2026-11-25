cmake_minimum_required(VERSION 3.8)
project(flapping_uav_gcs)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(SDL2 REQUIRED)
find_path(JOY_INCLUDE_DIR joy/joy.hpp HINTS "${joy_DIR}/../../../include" REQUIRED)
find_library(JOY_LIBRARY joy HINTS "${joy_DIR}/../../../lib" REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nlink_parser2 REQUIRED)
find_package(flapping_uav_msgs REQUIRED)
add_library(uwb_processor src/uwb_processor.cpp)
target_include_directories(uwb_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
add_library(uwb_position src/uwb_position_node.cpp)
target_link_libraries(uwb_position uwb_processor)
target_include_directories(uwb_position PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(uwb_position rclcpp rcl_interfaces geometry_msgs nav_msgs
  visualization_msgs diagnostic_msgs std_srvs nlink_parser2 flapping_uav_msgs)
add_executable(uwb_position_node src/uwb_position_main.cpp)
target_link_libraries(uwb_position_node uwb_position)
ament_target_dependencies(uwb_position_node rclcpp)
add_executable(uwb_simulator_node src/uwb_simulator_node.cpp)
ament_target_dependencies(uwb_simulator_node rclcpp nlink_parser2)
add_executable(manual_control_node src/manual_control_node.cpp)
ament_target_dependencies(manual_control_node rclcpp sensor_msgs flapping_uav_msgs)
target_include_directories(manual_control_node PRIVATE include)
set(MAVLINK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/mavlink_c" CACHE PATH
  "Pinned official MAVLink v2 C headers")
if(NOT EXISTS "${MAVLINK_INCLUDE_DIR}/common/mavlink.h")
  message(FATAL_ERROR "Fetch MAVLink headers with scripts/fetch_dependencies.sh")
endif()
add_executable(mavlink_bridge_node src/mavlink_bridge_node.cpp)
target_include_directories(mavlink_bridge_node PRIVATE include)
target_include_directories(mavlink_bridge_node SYSTEM PRIVATE "${MAVLINK_INCLUDE_DIR}")
ament_target_dependencies(mavlink_bridge_node rclcpp diagnostic_msgs flapping_uav_msgs)
add_executable(communication_node src/communication_node.cpp
  src/manual_control_node.cpp src/mavlink_bridge_node.cpp)
target_compile_definitions(communication_node PRIVATE FLAPPING_UAV_NO_MAIN)
target_include_directories(communication_node PRIVATE include ${JOY_INCLUDE_DIR} ${SDL2_INCLUDE_DIRS})
target_include_directories(communication_node SYSTEM PRIVATE "${MAVLINK_INCLUDE_DIR}")
target_link_libraries(communication_node ${JOY_LIBRARY} ${SDL2_LIBRARIES})
ament_target_dependencies(communication_node rclcpp sensor_msgs diagnostic_msgs flapping_uav_msgs)
add_executable(velocity_transform_node src/velocity_transform_node.cpp)
target_include_directories(velocity_transform_node PRIVATE include)
ament_target_dependencies(velocity_transform_node rclcpp geometry_msgs flapping_uav_msgs)
add_executable(position_controller_node src/position_controller_node.cpp)
target_include_directories(position_controller_node PRIVATE include)
ament_target_dependencies(position_controller_node rclcpp geometry_msgs std_srvs
  visualization_msgs flapping_uav_msgs)
add_executable(fusion_odometry_node src/fusion_odometry_node.cpp)
target_include_directories(fusion_odometry_node PRIVATE include)
ament_target_dependencies(fusion_odometry_node rclcpp geometry_msgs nav_msgs std_srvs visualization_msgs flapping_uav_msgs)
add_executable(experiment_recorder_node src/experiment_recorder_node.cpp)
target_include_directories(experiment_recorder_node PRIVATE include)
ament_target_dependencies(experiment_recorder_node rclcpp rcl_interfaces std_msgs std_srvs
  geometry_msgs flapping_uav_msgs)
add_executable(scene_visualizer_node src/scene_visualizer_node.cpp)
target_include_directories(scene_visualizer_node PRIVATE include)
ament_target_dependencies(scene_visualizer_node rclcpp geometry_msgs visualization_msgs
  diagnostic_msgs tf2_ros flapping_uav_msgs)
# Compile the pinned official protocol initializer unchanged under a lifecycle
# host, rather than editing vendor sources to fix Foxy publisher destruction.
set(NLINK_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../nlink_parser2" CACHE PATH
  "Pinned official Nooploop source checkout")
if(NOT EXISTS "${NLINK_SOURCE_DIR}/src/linktrack_node/src/linktrackinit.cpp")
  message(FATAL_ERROR "Fetch the official driver with scripts/fetch_dependencies.sh")
endif()
add_executable(linktrack_host_node src/linktrack_host_node.cpp
  "${NLINK_SOURCE_DIR}/src/linktrack_node/src/linktrackinit.cpp")
target_include_directories(linktrack_host_node SYSTEM PRIVATE
  "${NLINK_SOURCE_DIR}/src/linktrack_node/include"
  "${NLINK_SOURCE_DIR}/src/nlink_utils/include"
  "${NLINK_SOURCE_DIR}/src/nlink_utils/protocol_extracter"
  "${NLINK_SOURCE_DIR}/src/nlink_utils/nlink_unpack"
  "${NLINK_SOURCE_DIR}/extern/serial/include"
  "${NLINK_SOURCE_DIR}/extern/asio/asio/include")
ament_target_dependencies(linktrack_host_node rclcpp nlink_parser2)
target_compile_definitions(linktrack_host_node PRIVATE ASIO_STANDALONE)
# Foxy's ament_target_dependencies selects rosidl imported targets in preference
# to the driver's legacy *_LIBRARIES export, so link its native archives too.
find_library(NLINK_UTILS_LIBRARY nlink_utils
  PATHS "${nlink_parser2_DIR}/../../../lib" NO_DEFAULT_PATH)
find_library(NLINK_SERIAL_LIBRARY serial
  PATHS "${nlink_parser2_DIR}/../../../lib" NO_DEFAULT_PATH)
if(NOT NLINK_UTILS_LIBRARY OR NOT NLINK_SERIAL_LIBRARY)
  message(FATAL_ERROR "The pinned driver's native serial/nlink_utils archives are required")
endif()
target_link_libraries(linktrack_host_node ${NLINK_UTILS_LIBRARY} ${NLINK_SERIAL_LIBRARY})
install(TARGETS uwb_processor uwb_position
  EXPORT uwb_targets ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS uwb_position_node uwb_simulator_node linktrack_host_node
  manual_control_node mavlink_bridge_node velocity_transform_node position_controller_node
  communication_node fusion_odometry_node experiment_recorder_node scene_visualizer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/flight_log.py scripts/velocity_test.py scripts/attitude_test.py DESTINATION lib/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(uwb_processor uwb_position)
ament_export_dependencies(rclcpp rcl_interfaces geometry_msgs nav_msgs visualization_msgs
  diagnostic_msgs std_srvs nlink_parser2 flapping_uav_msgs)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_uwb_processor test/test_uwb_processor.cpp)
  target_link_libraries(test_uwb_processor uwb_processor)
  ament_add_gtest(test_uwb_ros test/test_uwb_ros.cpp)
  target_link_libraries(test_uwb_ros uwb_position)
  ament_target_dependencies(test_uwb_ros rclcpp nlink_parser2 flapping_uav_msgs
    geometry_msgs nav_msgs visualization_msgs)
  ament_add_gtest(test_rc_mapping test/test_rc_mapping.cpp)
  target_include_directories(test_rc_mapping PRIVATE include src)
  target_include_directories(test_rc_mapping SYSTEM PRIVATE "${MAVLINK_INCLUDE_DIR}")
  ament_add_gtest(test_rc_bridge test/test_rc_bridge.cpp
    src/manual_control_node.cpp src/mavlink_bridge_node.cpp)
  target_compile_definitions(test_rc_bridge PRIVATE FLAPPING_UAV_NO_MAIN)
  target_include_directories(test_rc_bridge PRIVATE include src)
  target_include_directories(test_rc_bridge SYSTEM PRIVATE "${MAVLINK_INCLUDE_DIR}")
  ament_target_dependencies(test_rc_bridge rclcpp sensor_msgs diagnostic_msgs flapping_uav_msgs)
  ament_add_gtest(test_velocity test/test_velocity.cpp
    src/velocity_transform_node.cpp src/mavlink_bridge_node.cpp)
  target_compile_definitions(test_velocity PRIVATE FLAPPING_UAV_NO_MAIN)
  target_include_directories(test_velocity PRIVATE include src)
  target_include_directories(test_velocity SYSTEM PRIVATE "${MAVLINK_INCLUDE_DIR}")
  ament_target_dependencies(test_velocity rclcpp geometry_msgs diagnostic_msgs flapping_uav_msgs)
  ament_add_gtest(test_fusion_graph test/test_fusion_graph.cpp src/fusion_odometry_node.cpp)
  target_compile_definitions(test_fusion_graph PRIVATE FLAPPING_UAV_NO_MAIN)
  target_include_directories(test_fusion_graph PRIVATE include)
  ament_target_dependencies(test_fusion_graph rclcpp geometry_msgs nav_msgs std_srvs visualization_msgs flapping_uav_msgs)
  ament_add_gtest(test_loose_odometry test/test_loose_odometry.cpp)
  target_include_directories(test_loose_odometry PRIVATE include)
  ament_add_gtest(test_position_source test/test_position_source.cpp)
  target_include_directories(test_position_source PRIVATE include)
  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_include_directories(test_trajectory PRIVATE include)
  ament_add_gtest(test_sample_order test/test_sample_order.cpp)
  target_include_directories(test_sample_order PRIVATE include)
  ament_add_gtest(test_position_controller test/test_position_controller.cpp)
  target_include_directories(test_position_controller PRIVATE include)
  ament_add_gtest(test_position_graph test/test_position_graph.cpp
    src/position_controller_node.cpp src/mavlink_bridge_node.cpp src/velocity_transform_node.cpp)
  target_compile_definitions(test_position_graph PRIVATE FLAPPING_UAV_NO_MAIN)
  target_include_directories(test_position_graph PRIVATE include src)
  target_include_directories(test_position_graph SYSTEM PRIVATE "${MAVLINK_INCLUDE_DIR}")
  ament_target_dependencies(test_position_graph rclcpp geometry_msgs std_srvs
    visualization_msgs flapping_uav_msgs diagnostic_msgs)
  ament_add_gtest(test_scene_visualizer test/test_scene_visualizer.cpp src/scene_visualizer_node.cpp)
  target_compile_definitions(test_scene_visualizer PRIVATE FLAPPING_UAV_NO_MAIN)
  target_include_directories(test_scene_visualizer PRIVATE include)
  ament_target_dependencies(test_scene_visualizer rclcpp geometry_msgs visualization_msgs
    diagnostic_msgs tf2_ros flapping_uav_msgs)
endif()
ament_package()
