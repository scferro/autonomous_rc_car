cmake_minimum_required(VERSION 3.22)
project(car_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

add_executable(read_encoder  src/read_encoder.cpp)
ament_target_dependencies(read_encoder rclcpp std_msgs)

add_executable(control_servos  src/control_servos.cpp)
ament_target_dependencies(control_servos rclcpp std_msgs)

add_executable(drive_and_steer  src/drive_and_steer.cpp)
ament_target_dependencies(drive_and_steer rclcpp std_msgs geometry_msgs)

add_executable(controller_interface  src/controller_interface.cpp)
ament_target_dependencies(controller_interface rclcpp std_msgs sensor_msgs)

install(TARGETS
  read_encoder
  control_servos
  drive_and_steer
  controller_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
