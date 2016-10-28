cmake_minimum_required(VERSION 2.8.12.2 FATAL_ERROR)

# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(MAVLINK REQUIRED mavlink)

# Generate documentation
find_package(Doxygen)
set(GENERATED_DOC_DIR "${CMAKE_CURRENT_BINARY_DIR}/doc")
if(DOXYGEN_FOUND)
    file(MAKE_DIRECTORY ${GENERATED_DOC_DIR})
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in
                   ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(doc ALL
                      ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
                      WORKING_DIRECTORY ${GENERATED_DOC_DIR}
                      COMMENT "Generating API documentation" VERBATIM)
endif(DOXYGEN_FOUND)

# Set debug variables
if(NOT DEFINED PRINT_MAVLINK)
    set(PRINT_MAVLINK false)
endif()
add_definitions(-DPRINT_MAVLINK=${PRINT_MAVLINK})
if(NOT DEFINED VERBOSE)
    set(VERBOSE true)
endif()
add_definitions(-DVERBOSE=${VERBOSE})

# Set additional flags
set(ADDITIONAL_COMPILE_FLAGS "-fstack-protector-all -fPIE -fPIC -O2 -D_FORTIFY_SOURCE=2")
set(ADDITIONAL_COMPILE_FLAGS "${ADDITIONAL_COMPILE_FLAGS} -Wformat -Wformat-security")
set(ADDITIONAL_COMPILE_FLAGS "${ADDITIONAL_COMPILE_FLAGS} -Wall -std=c++11")
set(ADDITIONAL_LINK_FLAGS "-z noexecstack -z relro -z now")

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src ${MAVLINK_INCLUDE_DIRS}/v1.0/autoquad)

# Build Library
add_library(mavlink_vehicles src/mavlink_vehicles.cc)
set_target_properties(mavlink_vehicles PROPERTIES COMPILE_FLAGS "${ADDITIONAL_COMPILE_FLAGS}")

# Build Tests
add_executable(tests tests/tests.cc)
target_link_libraries(tests mavlink_vehicles)
set_target_properties(tests PROPERTIES COMPILE_FLAGS "${ADDITIONAL_COMPILE_FLAGS}")
set_target_properties(tests PROPERTIES LINK_FLAGS "${ADDITIONAL_LINK_FLAGS} -pthread")
