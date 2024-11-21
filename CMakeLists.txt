cmake_minimum_required(VERSION 3.20)

project(ros-eeros)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_BUILD_TYPE MATCHES "Debug|DEBUG")
  logger_on()
endif()


# Find EEROS
find_package(EEROS 1.4.1)

add_subdirectory(lib)

INSTALL(FILES 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/RosNodeDevice.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/DigIn.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/DigOut.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/AnalogIn.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/AnalogOut.hpp
DESTINATION include/ros-eeros)

