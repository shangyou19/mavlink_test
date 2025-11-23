cmake_minimum_required(VERSION 3.13)
project(mavlink_sender_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(GENERATE_DIALECT "Generate MAVLink C dialect headers" ON)
set(DIALECT_NAME "my_guided" CACHE STRING "MAVLink dialect name")
set(DIALECT_XML "${CMAKE_SOURCE_DIR}/dialects/my_guided.xml" CACHE FILEPATH "Dialect XML path")

find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(GENERATED_DIR "${CMAKE_BINARY_DIR}/generated")
set(DIALECT_OUT_DIR "${GENERATED_DIR}/include/mavlink")
file(MAKE_DIRECTORY "${DIALECT_OUT_DIR}")

if(GENERATE_DIALECT)
  add_custom_command(
    OUTPUT "${DIALECT_OUT_DIR}/mavlink.h"
    COMMAND ${Python3_EXECUTABLE} -c "import os; from pymavlink.generator import mavgen; out='${DIALECT_OUT_DIR}'; xml='${DIALECT_XML}'; os.makedirs(out, exist_ok=True); opts=mavgen.Opts(output=out, wire_protocol='2.0', language='C', validate=True, strict_units=False); mavgen.mavgen(opts, [xml])"
    DEPENDS ${DIALECT_XML} ${CMAKE_SOURCE_DIR}/dialects/common.xml ${CMAKE_SOURCE_DIR}/dialects/standard.xml ${CMAKE_SOURCE_DIR}/dialects/minimal.xml
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    VERBATIM
    COMMENT "Generating MAVLink C headers from ${DIALECT_XML}"
  )
  add_custom_target(generate_dialect DEPENDS "${DIALECT_OUT_DIR}/mavlink.h")
endif()

add_executable(send_mavlink_cpp src/main.cpp src/mavlink_client.cpp)
target_include_directories(send_mavlink_cpp PRIVATE "${DIALECT_OUT_DIR}" "${DIALECT_OUT_DIR}/my_guided")
add_dependencies(send_mavlink_cpp generate_dialect)