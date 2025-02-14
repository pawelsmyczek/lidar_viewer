#include "lidar_viewer/dev/BinaryFile.h"
#include "lidar_viewer/dev/CygLidarD1.h"
#include "lidar_viewer/dev/SerialPort.h"
#include "lidar_viewer/ui/Viewer.h"
#include "lidar_viewer/ui/DisplayPointCloud.h"
#include "lidar_viewer/ui/DisplayFlatDepthImage.h"
#include "lidar_viewer/ui/DisplayOctreeFromPointCloud.h"
#include "lidar_viewer/ui/DisplayStatistics.h"

#include <array>

#include <gtest/gtest.h>

namespace lidar_viewer::tests::smoke
{

TEST(SmokeTestLidarViewer, SmokeTestLidarViewer3DMode) {
    using namespace lidar_viewer::dev;
    using namespace lidar_viewer::ui;
    using Mode = CygLidarD1::Mode;
    using namespace std::string_literals;
    using namespace std::chrono_literals;

    int argc = 0;
    char** argv = nullptr;
    auto executeBasicViewManipulationKeyPresses = []()
    {
        // allow the main thread to start the window
        std::this_thread::sleep_for(2s);
        const auto toolToExecuteCommands = "xdotool"s;

        std::array<std::string, 11> keysToPress
                {
                        "equal"s,
                        "minus"s,
                        "b"s,
                        "1"s,
                        "2"s,
                        "3"s,
                        "s"s,
                        "Up"s,
                        "Down"s,
                        "Left"s,
                        "Right"s,
                };
        for(const auto & key : keysToPress)
        {
            const auto command = toolToExecuteCommands + " key " + key;
            ::system(command.c_str());
            std::this_thread::sleep_for(1s);
        }
    };
    Viewer::Config viewerCfg
            {
                    .x = 0u,
                    .y = 0u,
                    .w = 1596u, // 160 / 60 = 2.66(7)
                    .h = 600u
            };
    try
    {
        IoStream input{};
        input.createAndOpen<BinaryFile>(ABSOLUTE_PATH_TO_ASSETS "coffee_cup.bin");

        std::cout << "Starting lidar\n";
        CygLidarD1 lidar{input};

        std::cout << "Starting to read point clouds\n";
        PointCloudReader pointCloudReader{lidar};
        pointCloudReader.start(Mode::Mode3D);
        auto commandsTestingThread = std::async(std::launch::async, executeBasicViewManipulationKeyPresses);
        std::cout << "Opening viewer\n";
        Viewer viewer{viewerCfg};

        viewer.registerViewerFunction(Viewer::ViewType::Flat, displayFlatDepthImage, &lidar);
        viewer.registerViewerFunction(Viewer::ViewType::PointCloud, displayPointCloud3D, &lidar);
        viewer.registerViewerFunction(Viewer::ViewType::Octree, displayOctreeFromPointCloud, &lidar);
        viewer.registerViewerFunction(Viewer::ViewType::Statistics, displayStatistics);
        viewer.start(&argc, argv);
        if(commandsTestingThread.valid())
        {
            commandsTestingThread.wait();
        }
    } catch (const std::exception & exc)
    {
        FAIL() << "Test failed, no exception expected, got : " << exc.what();
    }
    catch (...)
    {
        FAIL() << "Test failed, no exception expected, got unknown";
    }

}

} // namespace lidar_viewer::tests::smoke

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
