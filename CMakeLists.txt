cmake_minimum_required(VERSION 3.17)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_COMPILER "/usr/bin/gcc-10")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-10")

option(BUILD_FOR_UNIT_TESTS "Build Project just for unit test" OFF)

if(BUILD_FOR_UNIT_TESTS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -O0 -fprofile-arcs -ftest-coverage")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
endif()

project(lidar_viewer)

add_subdirectory(dev)
add_subdirectory(ui)

if(BUILD_FOR_UNIT_TESTS)
    add_subdirectory(tests)
endif()

add_executable(lidar_viewer main.cxx)
target_link_libraries(lidar_viewer dev ui)
