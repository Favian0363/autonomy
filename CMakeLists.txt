cmake_minimum_required(VERSION 3.20)

project(autonomy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(ExternalProject)

set(STONEFISH_DIR "${CMAKE_BINARY_DIR}/stonefish")
set(STONEFISH_BUILD_DIR "${CMAKE_BINARY_DIR}/stonefish-build")

ExternalProject_Add(stonefish
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/stonefish"
  BINARY_DIR "${STONEFISH_BUILD_DIR}"

  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${STONEFISH_DIR}
    -DCMAKE_BUILD_TYPE=Release

  BUILD_BYPRODUCTS
    "${STONEFISH_DIR}/lib/libStonefish.so"
)

add_library(auv_stonefish_sim SHARED
    auvs/stonefish_sim/auv.cpp
)

add_dependencies(auv_stonefish_sim stonefish)

target_include_directories(auv_stonefish_sim PRIVATE
  "${STONEFISH_DIR}/include"
  "${STONEFISH_DIR}/include/Stonefish"
)

target_compile_definitions(auv_stonefish_sim PRIVATE
  _GNU_SOURCE
  BT_EULER_DEFAULT_ZYX
  BT_USE_DOUBLE_PRECISION
  SHADER_DIR_PATH="${STONEFISH_DIR}/share/Stonefish/shaders/"
)

target_compile_options(auv_stonefish_sim PRIVATE
  -fopenmp
)

target_link_directories(auv_stonefish_sim PRIVATE
  "${STONEFISH_DIR}/lib"
)

target_link_libraries(auv_stonefish_sim PRIVATE
  Stonefish
  SDL2
  freetype
  OpenGL
  GLX
  GLU
  gomp
)

set_target_properties(auv_stonefish_sim PROPERTIES
  BUILD_RPATH "${STONEFISH_DIR}/lib"
)

install(
  TARGETS auv_stonefish_sim
)
