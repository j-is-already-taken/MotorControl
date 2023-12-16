cmake_minimum_required(VERSION 3.8)
project(servo_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(servo_control_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(servo_control_server SHARED)
target_sources(servo_control_server PRIVATE 
  src/servo_control_server.cpp
  src/servo_controler.cpp
  )
target_include_directories(servo_control_server PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

ament_target_dependencies(servo_control_server
    "servo_control_interfaces"
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components")

target_link_libraries(servo_control_server  pigpiod_if2)
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
rclcpp_components_register_node(servo_control_server PLUGIN
  "servo_control::ServoControlActionServer" EXECUTABLE
  servo_control_action_server)

install(TARGETS
    servo_control_action_server
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

ament_package()
