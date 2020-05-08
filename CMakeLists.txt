cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

find_program(PYTHON2 python2)
if (NOT PYTHON2)
    message(SEND_ERROR "Python 2 required but not found.")
endif()

# Make out of source builds mandatory
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

# Support Yocto SDK
if(DEFINED ENV{SDKTARGETSYSROOT})
    message(STATUS "Using Yocto Environment to build")
    set(CMAKE_FIND_ROOT_PATH $ENV{SDKTARGETSYSROOT})
endif()

# Generate Mavlink Messages
set(MAVLINK_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include/mavlink)
include_directories(${MAVLINK_INCLUDE_DIR} ${MAVLINK_INCLUDE_DIR}/common)
execute_process(COMMAND ${PYTHON2}
                ${CMAKE_CURRENT_SOURCE_DIR}/modules/mavlink/pymavlink/tools/mavgen.py
                ${CMAKE_CURRENT_SOURCE_DIR}/modules/mavlink/message_definitions/v1.0/common.xml
                --wire-protocol 2.0
                --lang C
                -o ${MAVLINK_INCLUDE_DIR})

# Generate documentation
find_package(Doxygen)
set(GENERATED_DOC_DIR "${CMAKE_CURRENT_BINARY_DIR}/doc")
if(DOXYGEN_FOUND)
    file(MAKE_DIRECTORY ${GENERATED_DOC_DIR})
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in
                   ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(doc
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

# Build Library
add_library(mavlink_vehicles src/mavlink_vehicles.cc)
set_target_properties(mavlink_vehicles PROPERTIES COMPILE_FLAGS "${ADDITIONAL_COMPILE_FLAGS}")
target_include_directories(mavlink_vehicles PUBLIC
                           "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Build Tests
add_executable(tests tests/tests.cc)
target_link_libraries(tests mavlink_vehicles)
set_target_properties(tests PROPERTIES COMPILE_FLAGS "${ADDITIONAL_COMPILE_FLAGS}")
set_target_properties(tests PROPERTIES LINK_FLAGS "${ADDITIONAL_LINK_FLAGS} -pthread")
