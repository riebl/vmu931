project(VMU931)
cmake_minimum_required(VERSION 3.0)

find_package(Boost COMPONENTS system REQUIRED)

add_library(vmu931 vmu931_sensor.cpp vmu931_types.cpp)
target_link_libraries(vmu931 Boost::system)

add_executable(vmu931_example main.cpp)
target_link_libraries(vmu931_example vmu931)
