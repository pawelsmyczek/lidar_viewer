#!/bin/bash

cd build

# Run tests and save output to report.xml file
./tests/lidar_viewer_test --gtest_output=xml:report.xml

# Generate lcov report
lcov --gcov-tool /usr/bin/gcov-10 \
     --capture \
     --directory tests/CMakeFiles/lidar_viewer_test.dir \
     --output-file coverage.info \
     --exclude '*/lidar_viewer/tests/*' \
     --exclude '*/gtest/*' \
     --exclude '*/10/*'


genhtml coverage.info --output-directory coverage_report
