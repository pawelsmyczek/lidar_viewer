cmake_minimum_required(VERSION 3.17)
project(lidar_viewer)
set(CMAKE_CXX_STANDARD 20)

add_subdirectory(dev)
add_subdirectory(ui)
add_executable(lidar_viewer main.cxx)
target_link_libraries(lidar_viewer dev ui)
