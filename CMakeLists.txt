cmake_minimum_required(VERSION 3.5.1)

project(ros-eeros)

set(CMAKE_CXX_STANDARD 14)


set(RECOMMENDED_EEROS_VERSION 1.1.0.0)


if(CMAKE_BUILD_TYPE MATCHES "Debug|DEBUG")
	set(LOGGER ON)
endif()


include(cmake/versionCheckHelpers.cmake)


setLibraryVersion()


# Find EEROS
setVersionToFind("EEROS" "REQUIRED_EEROS_VERSION" ${RECOMMENDED_EEROS_VERSION} ${REQUIRED_EEROS_VERSION})
find_package(EEROS ${VERSION_TO_FIND} EXACT)
checkIfModuleFound("EEROS" "REQUIRED_EEROS_VERSION")


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


createPackageFiles(ros-eerosConfig.cmake.in include/ lib/)


INSTALL(FILES 
	${CMAKE_CURRENT_SOURCE_DIR}/include/RosNodeDevice.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/DigIn.hpp 
	${CMAKE_CURRENT_SOURCE_DIR}/include/DigOut.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/AnalogIn.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/AnalogOut.hpp
DESTINATION include/ros-eeros)

