cmake_minimum_required(VERSION 3.8)
project(krsbi_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Declare message files
set(msg_files
  "msg/BallPosition.msg"
  "msg/RobotState.msg"
  "msg/MotorCommand.msg"
  "msg/MotorFeedback.msg"
  "msg/SensorData.msg"
  "msg/ImuData.msg"
  "msg/DistanceSensors.msg"
  "msg/GripperState.msg"
  "msg/KickerState.msg"
  "msg/GameState.msg"
  "msg/DetectedObject.msg"
  "msg/DetectedObjects.msg"
  "msg/FieldLines.msg"
  "msg/RobotPose.msg"
  "msg/TeamRobots.msg"
)

# Declare service files
set(srv_files
  "srv/SetMotorSpeed.srv"
  "srv/SetGripper.srv"
  "srv/ChargeKicker.srv"
  "srv/Kick.srv"
  "srv/GetRobotState.srv"
  "srv/Calibrate.srv"
  "srv/SetGameState.srv"
)

# Declare action files
set(action_files
  "action/MoveTo.action"
  "action/RotateTo.action"
  "action/KickBall.action"
  "action/GrabBall.action"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
