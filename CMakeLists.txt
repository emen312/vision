cmake_minimum_required(VERSION 3.10)

project(libxr_linux_topic)

set(CMAKE_CXX_STANDARD 20)

add_executable(writer writer.cpp)

add_executable(reader reader.cpp)

target_link_libraries(writer pthread)

target_link_libraries(reader pthread)