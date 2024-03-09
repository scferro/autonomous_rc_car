cmake_minimum_required(VERSION 3.8)
project(racing)

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
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(drag_race src/drag_race.cpp)
ament_target_dependencies(drag_race rclcpp std_msgs geometry_msgs nav_msgs std_srvs sensor_msgs tf2 tf2_ros)

add_executable(plan_path src/plan_path.cpp)
ament_target_dependencies(plan_path rclcpp std_msgs geometry_msgs nav_msgs std_srvs sensor_msgs tf2 tf2_ros)
target_link_libraries(plan_path ${YAML_CPP_LIBRARIES})
target_include_directories(plan_path PRIVATE ${YAML_CPP_INCLUDE_DIRS})

install(TARGETS
  drag_race
  plan_path
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
