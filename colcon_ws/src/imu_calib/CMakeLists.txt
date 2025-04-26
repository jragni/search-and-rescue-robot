cmake_minimum_required(VERSION 3.8)
project(imu_calib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

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

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# include libraries
add_library(${PROJECT_NAME} SHARED
  src/accel_calib.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
)

add_executable(apply_calib src/apply_calib.cpp src/accel_calib.cpp)
target_link_libraries(apply_calib yaml-cpp)
ament_target_dependencies(apply_calib rclcpp sensor_msgs)

add_executable(do_calib src/accel_calib src/do_calib.cpp)
target_link_libraries(do_calib yaml-cpp)
ament_target_dependencies(do_calib rclcpp sensor_msgs)


install(TARGETS
  apply_calib
  do_calib
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
