cmake_minimum_required(VERSION 3.8)
project(vesc_socketcan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(vesc_socketcan_node src/vesc_socketcan_node.cpp)
add_executable(vesc_socketcan_node2 src/vesc_socketcan_node2.cpp)
add_executable(joy_converter_node src/joy_converter_node.cpp)
ament_target_dependencies(vesc_socketcan_node rclcpp vesc_msgs std_msgs)
ament_target_dependencies(vesc_socketcan_node2 rclcpp vesc_msgs std_msgs)
ament_target_dependencies(joy_converter_node rclcpp vesc_msgs std_msgs sensor_msgs)

install(TARGETS
  vesc_socketcan_node
  vesc_socketcan_node2
  joy_converter_node
  DESTINATION lib/${PROJECT_NAME})

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


