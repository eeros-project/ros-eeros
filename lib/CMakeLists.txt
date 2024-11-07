include_directories(../include/)

set(ROSEEROS_SRCS RosNodeDevice.cpp DigIn.cpp DigOut.cpp AnalogIn.cpp AnalogOut.cpp)

add_library(roseeros SHARED ${ROSEEROS_SRCS})
add_library(roseeros_static ${ROSEEROS_SRCS})


target_link_libraries(roseeros EEROS::eeros ${ROS_LIBRARIES})
set_target_properties(roseeros PROPERTIES VERSION ${LIBRARY_VERSION})

target_link_libraries(roseeros_static EEROS::eeros ${ROS_LIBRARIES})


INSTALL(TARGETS roseeros roseeros_static LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

