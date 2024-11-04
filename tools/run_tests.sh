#!/bin/bash

cd $1

# Run tests and save output to report.xml file
./tests/units/lidar_viewer_unit_test --gtest_output=xml:report.xml

# Generate lcov report
lcov --gcov-tool /usr/bin/gcov-10 \
     --capture \
     --directory tests/units/CMakeFiles/lidar_viewer_unit_test.dir \
     --output-file coverage.info \
     --exclude '*/lidar_viewer/tests/*' \
     --exclude '*/gtest/*' \
     --exclude '*/10/*'

genhtml coverage.info --output-directory coverage_report_units

# Run tests and save output to report.xml file
./tests/smoke/lidar_viewer_smoke_test --gtest_output=xml:report.xml

# Generate lcov report
lcov --gcov-tool /usr/bin/gcov-10 \
     --capture \
     --directory tests/smoke/CMakeFiles/lidar_viewer_smoke_test.dir \
     --output-file coverage.info \
     --exclude '*/lidar_viewer/tests/*' \
     --exclude '*/gtest/*' \
     --exclude '*/10/*'


genhtml coverage.info --output-directory coverage_report_smoke
