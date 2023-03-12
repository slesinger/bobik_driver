cmake_minimum_required(VERSION 2.8)
project(bobik_driver)
set(CMAKE_INSTALL_PREFIX /usr/local)

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
   set(CMAKE_CXX_STANDARD 14)
# endif()

 if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
   add_compile_options(-Wall -Wextra -Wpedantic)
#   add_compile_options(-Wall -Wextra -Wpedantic -std=c++14)
 endif()

add_executable(bobik_driver src/bobik_driver.cpp src/bobik_zmq.cpp src/bobik_kinect.cpp src/transporter.cpp src/uart_transporter.cpp src/ring_buffer.cpp src/xv11_laser.cpp src/loguru.cpp)
target_include_directories(bobik_driver PUBLIC /usr/local/include /usr/local/include/libfreenect)
target_link_libraries(bobik_driver /usr/local/lib/libzmq.so /usr/local/lib/libzstd.so pthread boost_system freenect ${CMAKE_DL_LIBS})

message("Installing to: ${CMAKE_INSTALL_PREFIX}/bin  (when you do: make install)")
install(TARGETS bobik_driver DESTINATION bin)
