cmake_minimum_required(VERSION 3.13)
project(mavlink_sender_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(send_mavlink_cpp src/main.cpp src/mavlink_client.cpp)
target_include_directories(send_mavlink_cpp PRIVATE "${CMAKE_SOURCE_DIR}/include")