cmake_minimum_required(VERSION 2.6)
project(ros-eeros)

## Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find package EEROS
find_package(EEROS REQUIRED)
include_directories(${EEROS_INCLUDE_DIR})
link_directories(${EEROS_LIB_DIR})

## Find package ROS
find_package( roslib REQUIRED )
add_definitions(-DROS_FOUND)
set( ROS_FOUND true)
include_directories( "${roslib_INCLUDE_DIRS}" )
list(APPEND ROS_LIBRARIES "${roslib_LIBRARIES}")
find_package( rosconsole REQUIRED)
list(APPEND ROS_LIBRARIES "${rosconsole_LIBRARIES}")
find_package( roscpp REQUIRED )
list(APPEND ROS_LIBRARIES "${roscpp_LIBRARIES}")
list(APPEND EXTERNAL_LIBS "${ROS_LIBRARIES}")


include_directories(${ADDITIONAL_INCLUDE_DIRS})
link_directories(${ADDITIONAL_LINK_DIRS})

add_subdirectory(lib)
#add_subdirectory(test)

INSTALL(FILES 
	${CMAKE_CURRENT_SOURCE_DIR}/include/RosNodeDevice.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/DigIn.hpp 
	${CMAKE_CURRENT_SOURCE_DIR}/include/DigOut.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/AnalogIn.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/AnalogOut.hpp
DESTINATION include/ros-eeros)
