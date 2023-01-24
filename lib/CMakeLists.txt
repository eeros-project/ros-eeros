include_directories(../include/)

set(ROSEEROS_SRCS RosNodeDevice.cpp DigIn.cpp DigOut.cpp AnalogIn.cpp AnalogOut.cpp)

find_package( std_msgs REQUIRED )
list(APPEND MSG_LIBRARIES ${std_msgs_LIBRARIES})
include_directories(${std_msgs_INCLUDE_DIRS})

find_package( sensor_msgs REQUIRED )
list(APPEND MSG_LIBRARIES ${sensor_msgs_LIBRARIES})
include_directories(${sensor_msgs_INCLUDE_DIRS})

add_library(roseeros SHARED ${ROSEEROS_SRCS})
add_library(roseeros_static ${ROSEEROS_SRCS})

target_link_libraries(roseeros PRIVATE eeros ${roslib_LIBRARIES} ${rclcpp_LIBRARIES} ${rcl_interfaces_LIBRARIES} ${MSG_LIBRARIES})
set_target_properties(roseeros PROPERTIES VERSION ${LIBRARY_VERSION})

target_link_libraries(roseeros_static PRIVATE eeros ${roslib_LIBRARIES} ${rclcpp_LIBRARIES} ${rcl_interfaces_LIBRARIES} ${MSG_LIBRARIES})

INSTALL(TARGETS roseeros roseeros_static LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

