cmake_minimum_required(VERSION 3.8)
project(transbot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # msg
  "msg/Adjust.msg"
  "msg/Arm.msg"
  "msg/Battery.msg"
  "msg/Edition.msg"
  "msg/General.msg"
  "msg/ImageMsg.msg"
  "msg/Joint.msg"
  "msg/LaserAvoid.msg"
  "msg/JoyState.msg"
  "msg/PatrolWarning.msg"
  "msg/PointArray.msg"
  "msg/Position.msg"
  "msg/PWMServo.msg"
  # srv
  "srv/MoveForward.srv"
  "srv/Buzzer.srv"
  "srv/CamDevice.srv"
  "srv/Headlight.srv"
  "srv/Patrol.srv"
  "srv/RGBLight.srv"
  "srv/RobotArm.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
