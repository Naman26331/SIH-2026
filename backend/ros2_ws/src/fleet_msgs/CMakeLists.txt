cmake_minimum_required(VERSION 3.8)
project(fleet_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # The seven named in 03_ROBOT_AND_ROS2_IMPLEMENTATION section 2
  "msg/RobotState.msg"
  "msg/RobotIntent.msg"
  "msg/ConflictAlert.msg"
  "msg/PathReservation.msg"
  "msg/Task.msg"
  "msg/BlockedAisle.msg"
  "msg/BatteryStatus.msg"
  # Four more the working system needed. See the README.
  "msg/TaskBid.msg"
  "msg/TaskClaim.msg"
  "msg/WaitReport.msg"
  "msg/YieldRequest.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
