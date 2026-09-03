cmake_minimum_required(VERSION 3.13)
project(dart_fc LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(closed_loop_demo examples/closed_loop_demo.cpp)
add_executable(test_hw_remap examples/test_hw_remap.cpp)

foreach(tgt closed_loop_demo test_hw_remap)
  target_include_directories(${tgt} PRIVATE ${CMAKE_SOURCE_DIR}/include)
endforeach()
