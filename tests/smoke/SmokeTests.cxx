#include "lidar_viewer/dev/BinaryFile.h"
#include "lidar_viewer/dev/CygLidarD1.h"
#include "lidar_viewer/dev/FrameWriter.h"
#include "lidar_viewer/dev/PointCloudReader.h"
#include "lidar_viewer/dev/SerialPort.h"
#include "lidar_viewer/ui/display/DisplayPointCloud.h"
#include "lidar_viewer/ui/display/DisplayFlatDepthImage.h"
#include "lidar_viewer/ui/display/DisplayOctreeFromPointCloud.h"
#include "lidar_viewer/ui/display/DisplayStatistics.h"
#include "lidar_viewer/ui/window/gl2/ViewManagerGl.h"
#include "lidar_viewer/ui/window/gl2/GetScreenParameters.h"
#include "lidar_viewer/ui/display/gl2/DisplayManagerGl.h"
#include "lidar_viewer/ui/drawing/gl2/DrawCube.h"
#include "lidar_viewer/ui/drawing/gl2/DrawPoint.h"
#include "lidar_viewer/ui/drawing/gl2/DrawString.h"

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
    ViewManagerGl::Config viewerGlCfg
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
        ViewManagerGl window{viewerGlCfg};
        DisplayManagerGl displayManagerGl{window, std::chrono::milliseconds{16}};
        displayManagerGl.registerDisplayFunction(DisplayManagerBase::ViewType::Flat, displayFlatDepthImage, &lidar
                , drawing::drawPointByteColored<float>);
        displayManagerGl.registerDisplayFunction(DisplayManagerBase::ViewType::PointCloud, displayPointCloud3D, &lidar
                , drawing::drawPoint<float>);
        displayManagerGl.registerDisplayFunction(DisplayManagerBase::ViewType::Octree,
                                                 display::displayOctreeFromPointCloud,
                                                 &lidar, drawing::drawCube<float>, drawing::drawPoint<float>);
        displayManagerGl.registerDisplayFunction(DisplayManagerBase::ViewType::Statistics, display::displayStatistics,
                                                 drawing::drawStdStringFloatPos, getScreenParameters);
        window.start(&argc, argv);
        displayManagerGl.start();

        if(commandsTestingThread.valid())
        {
            commandsTestingThread.wait();
        }
//      stop call will be performed when display function returns
        window.stop();
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
