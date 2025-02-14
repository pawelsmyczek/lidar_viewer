set(NAME lidar_viewer_unit_test)

find_package(GTest REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -O0 -fprofile-arcs -ftest-coverage")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")

add_executable(${NAME}
        dev/StaturOrTest.cxx
        dev/BinaryFileTest.cxx
        dev/CygLidarD1Test.cxx
        dev/CyglidarFrameTest.cxx
        dev/IoStreamTest.cxx
        dev/PointCloudProviderTest.cxx
        dev/FrameWriterTest.cxx
        geometry/BoxTest.cxx
        geometry/DownSampleTest.cxx
        geometry/GetDepthImageToPointCloudProcessorTest.cxx
        geometry/OctreeTest.cxx
        geometry/OctreeFromPointCloudTest.cxx
        geometry/OctreeIteratorTest.cxx
        geometry/PointTest.cxx
        geometry/UtilitiesTest.cxx
        geometry/ScreenRangesTest.cxx
        ui/ViewerTest.cxx
        ui/DisplayManagerTest.cxx)

target_link_libraries(${NAME}
        GTest::GTest
        GTest::Main
        GTest::gmock
        lidar_viewer_device
        lidar_viewer_ui
        lidar_viewer_geometry)

enable_testing()

add_test(NAME ${NAME} COMMAND ${NAME} --gtest_output=xml:${CMAKE_BINARY_DIR}/test_reports/report.xml)

