#!/bin/bash

if [ ! -d build ]; then
    mkdir build
fi
cd build

# Set up default CMake arguments
CMAKE_ARGS="-DCMAKE_C_COMPILER=/usr/bin/gcc-10 -DCMAKE_CXX_COMPILER=/usr/bin/g++-10"

# Check if the first argument is '--test'
if [ "$1" == "--test" ]; then
    CMAKE_ARGS="$CMAKE_ARGS -DBUILD_FOR_UNIT_TESTS=ON"
fi

# Run CMake with the constructed arguments
cmake $CMAKE_ARGS ..
make

if [ "$1" == "--test" ]; then
    ../tools/run_tests.sh
fi