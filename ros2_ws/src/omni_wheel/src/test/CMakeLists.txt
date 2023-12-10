cmake_minimum_required(VERSION 3.16)

project(omni_test CXX)

find_package(Eigen3 REQUIRED)
add_executable(test_omni)

target_sources(test_omni PRIVATE
  omni_wheel_control_test.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../omni_wheel_control.cpp
)

target_link_libraries(test_omni PRIVATE Eigen3::Eigen)
target_include_directories(test_omni PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include/)

target_compile_features(test_omni PRIVATE cxx_std_20)
