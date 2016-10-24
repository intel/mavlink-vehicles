cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_program(PYTHON python)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
find_package(Threads REQUIRED)

set(CUSTOM_COMPILE_FLAGS "${CUSTOM_COMPILE_FLAGS} -Wall -pthread -std=c++11")
set(LDFLAGS "${LDFLAGS} -z noexecstack -z relro -z now")
set(CFLAGS "${CFLAGS} -fstack-protector-all -fPIE -fPIC -O2 -D_FORTIFY_SOURCE=2")
set(CFLAGS "${CFLAGS} -Wformat -Wformat-security")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CFLAGS} ${CUSTOM_COMPILE_FLAGS}")

if(NOT DEFINED PRINT_MAVLINK)
    set(PRINT_MAVLINK false)
endif()
add_definitions(-DPRINT_MAVLINK=${PRINT_MAVLINK})

if(NOT DEFINED VERBOSE)
    set(VERBOSE true)
endif()
add_definitions(-DVERBOSE=${VERBOSE})

add_library(mavlink_vehicles mavlink_vehicles.cc)
target_link_libraries(mavlink_vehicles ${CMAKE_THREAD_LIBS_INIT})
