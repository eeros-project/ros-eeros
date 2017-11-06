include_directories(../include/)

set(ROSEEROS_SRCS RosNodeDevice.cpp DigIn.cpp DigOut.cpp AnalogIn.cpp AnalogOut.cpp)

add_library(roseeros SHARED ${ROSEEROS_SRCS})
add_library(roseeros_static ${ROSEEROS_SRCS})

target_link_libraries(roseeros eeros)
target_link_libraries(roseeros_static eeros)

INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/libroseeros.so ${CMAKE_CURRENT_BINARY_DIR}/libroseeros_static.a DESTINATION lib)
