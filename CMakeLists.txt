project(VMU931)
cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 11)
include(GNUInstallDirs)

find_package(Boost COMPONENTS system REQUIRED)

add_library(vmu931 src/sensor.cpp src/types.cpp)
target_link_libraries(vmu931 Boost::system)
target_include_directories(vmu931
    INTERFACE "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
    PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>")

install(TARGETS vmu931 EXPORT VMU931 DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/vmu931 DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp")
install(EXPORT VMU931 NAMESPACE VMU931:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake)

add_executable(vmu931_example src/main.cpp)
target_link_libraries(vmu931_example vmu931)
