cmake_minimum_required(VERSION 3.8)
project(flapping_uav_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UwbStatus.msg"
  "msg/RcOverride.msg"
  "msg/MavlinkFrame.msg"
  "msg/AttitudeState.msg"
  "msg/BodyVelocitySetpoint.msg"
  "msg/BodyAttitudeSetpoint.msg"
  "msg/PositionAttitudeDebug.msg"
  "msg/BodyMotionState.msg"
  "msg/NavigationStatus.msg"
  "msg/HeadingAlignment.msg"
  "msg/FusionStatus.msg"
  "msg/PositionEstimate.msg"
  DEPENDENCIES std_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
