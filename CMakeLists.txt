cmake_minimum_required(VERSION 3.5)
project(bobik_driver)

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

add_executable(bobik_driver src/bobik_driver.cpp src/transporter.cpp src/uart_transporter.cpp src/ring_buffer.cpp src/loguru.cpp)

target_link_libraries(bobik_driver /usr/local/lib/libzmq.so pthread ${CMAKE_DL_LIBS})
