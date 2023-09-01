cmake_minimum_required(VERSION 3.16)
project(lidar_viewer)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic)

if(CMAKE_BUILD_TYPE STREQUAL Release)
    add_compile_options(-O2)
elseif(CMAKE_BUILD_TYPE STREQUAL Debug)
    add_compile_options(-O0 -g)
endif()

add_subdirectory(dev)
add_subdirectory(ui)

add_executable(lidar_viewer main.cxx)
target_link_libraries(lidar_viewer ui dev pthread)
