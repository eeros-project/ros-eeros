cmake_minimum_required(VERSION 3.10)

project(ros-eeros)

set(CMAKE_CXX_STANDARD 14)


set(RECOMMENDED_EEROS_VERSION 1.3)


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
  checkout_cmake_modules_lib_version(8fcde84)
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
find_package( rclcpp REQUIRED )
if (rclcpp_FOUND)
  message(STATUS "--> 'ROS2' is found.")
else()
  message(FATAL_ERROR "ERROR: ROS2 is not installed or setup.bash (for ROS2) was not executed.")
endif()
include_directories(${rclcpp_INCLUDE_DIRS})
list(APPEND ROS_LIBRARIES "${rclcpp_LIBRARIES}")
find_package(rcl_interfaces REQUIRED)
include_directories(${rcl_interfaces_INCLUDE_DIRS})
list(APPEND ROS_LIBRARIES "${rcl_interfaces_LIBRARIES}")
find_package(std_msgs REQUIRED)
include_directories(${std_msgs_INCLUDE_DIRS})
list(APPEND ROS_LIBRARIES ${std_msgs_LIBRARIES})
find_package( sensor_msgs REQUIRED )
include_directories(${sensor_msgs_INCLUDE_DIRS})
list(APPEND ROS_LIBRARIES ${sensor_msgs_LIBRARIES})


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

