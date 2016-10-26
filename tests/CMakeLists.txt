cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CUSTOM_COMPILE_FLAGS "${CUSTOM_COMPILE_FLAGS} -Wall -std=c++11")
set(LDFLAGS "${LDFLAGS} -z noexecstack -z relro -z now")
set(CFLAGS "${CFLAGS} -fstack-protector-all -fPIE -fPIC -O2 -D_FORTIFY_SOURCE=2")
set(CFLAGS "${CFLAGS} -Wformat -Wformat-security")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CFLAGS} ${CUSTOM_COMPILE_FLAGS}")

add_executable(tests tests.cc)
target_link_libraries(tests mavlink_vehicles)
