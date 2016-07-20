cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_program(PYTHON python)

if (NOT PYTHON)
    message(SEND_ERROR "Python not found")
endif()

# Generate Mavlink Messages
if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/lib/)
    execute_process(
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/modules/mavlink
            COMMAND
            ${PYTHON}
            -m pymavlink.tools.mavgen
            -o ${CMAKE_CURRENT_BINARY_DIR}/lib
            --lang C
            ./message_definitions/v1.0/common.xml)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/mavserver)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/common)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/lib)

add_subdirectory(mavserver)
add_subdirectory(tests)
