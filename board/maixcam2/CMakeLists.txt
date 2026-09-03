# MaixCDK 工程入口。在本目录执行:
#   set MAIXCDK_PATH=<你的 MaixCDK 路径>
#   maixcdk build -p maixcam2
cmake_minimum_required(VERSION 3.13)

if(DEFINED ENV{MAIXCDK_PATH} AND EXISTS "$ENV{MAIXCDK_PATH}/tools/cmake/compile.cmake")
  include("$ENV{MAIXCDK_PATH}/tools/cmake/compile.cmake")
elseif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../tools/cmake/compile.cmake")
  include("${CMAKE_CURRENT_LIST_DIR}/../../tools/cmake/compile.cmake")
else()
  message(FATAL_ERROR "Set MAIXCDK_PATH to your MaixCDK clone, or copy this folder into MaixCDK/projects/dart_fc")
endif()
