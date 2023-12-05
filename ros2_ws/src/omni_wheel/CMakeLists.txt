cmake_minimum_required(VERSION 3.8)
project(omni_wheel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(omni_wheel_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(Eigen3 REQUIRED)
#find_package(pigpio REQUIRED)
#include(FetchContent)
#FetchContent_Declare(
#  pigpio
#  URL https://github.com/joan2937/pigpio/archive/master.zip
#)

#FetchContent_MakeAvailable(pigpio)
add_library(omni_wheel_action_server_lib SHARED)

target_sources(omni_wheel_action_server_lib PRIVATE
  src/omni_wheel_server.cpp
  src/omni_wheel_control.cpp
  src/motor_control.cpp
)

target_include_directories(omni_wheel_action_server_lib PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(omni_wheel_action_server_lib
  omni_wheel_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
)
target_link_libraries(omni_wheel_action_server_lib
  Eigen3::Eigen
  #pigpio
  #pigpiod_if2
  )

rclcpp_components_register_node(omni_wheel_action_server_lib PLUGIN
  omni_wheel::OmniWheelActionServer EXECUTABLE omni_wheel_action_server
) 

target_compile_features(omni_wheel_action_server_lib PRIVATE cxx_std_20)

install(TARGETS
  omni_wheel_action_server_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
