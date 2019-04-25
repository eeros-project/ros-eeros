cmake_minimum_required(VERSION 3.5.1)

project(ros-eeros)

set(CMAKE_CXX_STANDARD 14)

set(RECOMMENDED_EEROS_VERSION 1.0.0.0)

if(NOT DEFINED REQUIRED_EEROS_VERSION OR
   NOT REQUIRED_EEROS_VERSION MATCHES
       "^(0|[1-9][0-9]*)[.](0|[1-9][0-9]*)[.](0|[1-9][0-9]*)[.](0|[1-9][0-9]*)$")

  set(REQUIRED_EEROS_VERSION ${RECOMMENDED_EEROS_VERSION})
  message("Using recommended EEROS version, which is: v${REQUIRED_EEROS_VERSION}")

else()
  if(NOT ${REQUIRED_EEROS_VERSION} EQUAL ${RECOMMENDED_EEROS_VERSION})
     message("You are not using the recommended EEROS version, "
             "which would be: v${RECOMMENDED_EEROS_VERSION}")
  endif(NOT ${REQUIRED_EEROS_VERSION} EQUAL ${RECOMMENDED_EEROS_VERSION})
endif()

find_package(EEROS ${REQUIRED_EEROS_VERSION} REQUIRED)

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
