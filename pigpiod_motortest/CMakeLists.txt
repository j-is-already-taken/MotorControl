cmake_minimum_required(VERSION 3.18)

project(pigpio_sample CXX)

#find_package(pigpio REQUIRED)

add_executable(sample)

target_sources(sample PRIVATE sample.cpp)


#target_link_libraries(sample PRIVATE pigpio thread)
target_link_libraries(sample PRIVATE  pigpiod_if2)
