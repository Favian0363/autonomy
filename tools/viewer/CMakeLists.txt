cmake_minimum_required(VERSION 3.16)
project(viewer C CXX)

include(FetchContent)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(raylib URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz)
FetchContent_MakeAvailable(raylib)

find_package(ZED REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_executable(viewer main.cpp net.cpp video.cpp)
target_compile_features(viewer PRIVATE cxx_std_17)
target_include_directories(viewer PRIVATE ${ZED_INCLUDE_DIRS})
target_link_libraries(viewer PRIVATE raylib ${ZED_LIBRARIES} CUDA::cuda_driver CUDA::cudart)

if(WIN32)
  target_link_libraries(viewer PRIVATE ws2_32)
endif()
