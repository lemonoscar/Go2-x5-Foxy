cmake_minimum_required(VERSION 3.5)
project(robot_msgs VERSION 3.0.0)

if(NOT "$ENV{ROS_DISTRO}" STREQUAL "foxy")
  message(FATAL_ERROR "robot_msgs in rl_ras_n supports ROS2 Foxy only. Current ROS_DISTRO=$ENV{ROS_DISTRO}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
  "msg/MotorState.msg"
  "msg/RobotCommand.msg"
  "msg/RobotState.msg"
  "msg/IMU.msg"
)
ament_package()
