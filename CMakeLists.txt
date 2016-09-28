cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(PkgConfig REQUIRED)
pkg_check_modules(MAVLINK REQUIRED mavlink)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/mavlink_vehicles ${MAVLINK_INCLUDE_DIRS}/v1.0/autoquad)

add_subdirectory(mavlink_vehicles)
add_subdirectory(tests)

