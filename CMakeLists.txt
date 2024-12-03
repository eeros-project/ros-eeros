cmake_minimum_required(VERSION 3.14)

include(cmake/git.cmake)

version_from_git(GIT_VERSION)
message("git version: ${GIT_VERSION}")

project(ros-eeros LANGUAGES CXX VERSION ${GIT_VERSION})

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library("${PROJECT_NAME}_ros" SHARED)
add_library("${PROJECT_NAME}::ros" ALIAS "${PROJECT_NAME}_ros")
set_target_properties("${PROJECT_NAME}_ros" PROPERTIES VERSION ${GIT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR} OUTPUT_NAME roseeros EXPORT_NAME roseeros)

find_package(EEROS 1.4 REQUIRED)

add_subdirectory(lib)
target_link_libraries("${PROJECT_NAME}_ros" EEROS::eeros)

target_include_directories("${PROJECT_NAME}_ros" PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>)

target_compile_features("${PROJECT_NAME}_ros" PRIVATE cxx_std_14)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY AnyNewerVersion
)

install(FILES
${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake
DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

install(TARGETS "${PROJECT_NAME}_ros"
EXPORT roseeros
RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(EXPORT roseeros NAMESPACE ${PROJECT_NAME}:: DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
