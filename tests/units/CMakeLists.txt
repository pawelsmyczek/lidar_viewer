set(NAME lidar_viewer_unit_test)

find_package(GTest REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -O0 -fprofile-arcs -ftest-coverage")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")

add_executable(${NAME} UnitTest.cxx)

target_link_libraries(${NAME} GTest::GTest GTest::Main lidar_viewer_device lidar_viewer_ui)

enable_testing()

add_test(NAME ${NAME} COMMAND ${NAME} --gtest_output=xml:${CMAKE_BINARY_DIR}/test_reports/report.xml)

