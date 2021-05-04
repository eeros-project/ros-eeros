cmake_minimum_required(VERSION 3.5.1)

project(ros-eeros)

set(CMAKE_CXX_STANDARD 14)


set(RECOMMENDED_EEROS_VERSION 1.2)


# Load CMake modules library at specified version
set(LOAD_CMAKE_MODULES_LIB TRUE CACHE BOOL "TRUE if the cmake-modules library should be loaded.")

if(LOAD_CMAKE_MODULES_LIB)
  include(cmake/CMakeModulesLibLoader.cmake)
  load_cmake_modules_lib(https://github.com/eeros-project/cmake-modules.git)
endif()

include(CMakeModulesLib)

if(CMAKE_BUILD_TYPE MATCHES "Debug|DEBUG")
  logger_on()
endif()

if(LOAD_CMAKE_MODULES_LIB)
  checkout_cmake_modules_lib_version(a50add2)
endif()


# Fetch the version information from git tag
include(VersioningHelpers)
setLibraryVersion()


# Find EEROS
setVersionToFind("EEROS" "REQUIRED_EEROS_VERSION" ${RECOMMENDED_EEROS_VERSION} ${REQUIRED_EEROS_VERSION})
find_package(EEROS ${VERSION_TO_FIND})
checkIfModuleFound("EEROS" "REQUIRED_EEROS_VERSION")
set(REQUIRED_DEGREE_OF_VERSION_COMPATIBILITY 2 CACHE STRING "The required degree of EEROS version compatibilty.")
checkVersionCompatibility(${EEROS_VERSION} ${VERSION_TO_FIND} ${REQUIRED_DEGREE_OF_VERSION_COMPATIBILITY})


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

