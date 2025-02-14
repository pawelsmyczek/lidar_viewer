#include "lidar_viewer/dev/SerialPort.h"
#include "lidar_viewer/dev/CygLidarD1.h"
#include "lidar_viewer/ui/Viewer.h"
#include "lidar_viewer/ui/DisplayFunctions.h"
#include "lidar_viewer/dev/BinaryFile.h"

#include <iostream>
#include <functional>
#include <memory>

#include <unistd.h>
#include <csignal>
#include <termios.h>

namespace
{

std::function<void(int)> handleSignal;

void signalHandler(int sigNum)
{
    if(!handleSignal)
    {
        return;
    }
    handleSignal(sigNum);

}

std::string usageString()
{
    using namespace std::string_literals;
    auto usageStr = "Usage: \n\t./lidar_viewer"
            "\n\t\t-d [deviceName] "
            "\n\t\t-f [inputFileName] "
            "\n\t\t-b serialBaudRate { B57k6,B115k2,B250k,B3M } "
            "\n\t\t-l lidarBaudRateConfig { B57k6,B115k2,B250k,B3M } "
            "\n\t\t-m lidarMode { 2D,3D,Dual } "
            "\n\t\t-c frequencyChannel {0-15} "
            "\n\t\t-p pulseDuration (for 3D) { 0-10000 } [in ms] "
            "\n\t\t-s sensitivity {0-255}"
            "\n\t\t-o outputFileName file to write frames to"s;
    return usageStr;
}

}

int main(int argc, char** argv)
{
    using namespace lidar_viewer::dev;
    using namespace lidar_viewer::ui;
    using namespace std::chrono_literals;
    using PulseMode = CygLidarD1::PulseDuration::PulseMode;
    using Mode = CygLidarD1::Mode;
    using BaudRate = CygLidarD1::BaudRate;

    ::signal(SIGTERM, signalHandler);
    ::signal(SIGINT, signalHandler); // Ctrl-C...

    int opt;
    CygLidarD1::Config lidarCfg{};
    Mode mode{};
    int speedForSerial {};
    std::string deviceName {};
    std::string inputFileName {};
    std::string outputFileName {};
    IoStream input{};
    IoStream output{};
    Viewer viewer{{
                          .x = 0u,
                          .y = 0u,
                          .w = 1596u, // 160 / 60 = 2.66(7)
                          .h = 600u,
                          .viewType = Viewer::ViewType::PointCloud
                  }};

    while( ( opt = ::getopt(argc, argv, "d:f:b:l:m:c:p:s:o:h")) != -1 )
    {
        switch ( opt )
        {
            case 'd':
                deviceName = std::string { optarg };
                break;
            case 'f':
                inputFileName = std::string { optarg };
                break;
            case 'b':
            {
                using namespace std::string_literals;
                const std::string bRate{optarg};

                speedForSerial =
                bRate == "B57k6"s? B57600:
                bRate == "B115k2"s? B115200 :
                bRate == "B250k"s? 250000 :
                bRate == "B3M"s? B3000000   :
                              250000;
                break;
            }
            case 'l':
            {
                using namespace std::string_literals;
                const std::string bRate{optarg};
                lidarCfg.baudRate =
                bRate == "B57k6"s? BaudRate::B57k6 :
                bRate == "B115k2"s? BaudRate::B115k2 :
                bRate == "B250k"s? BaudRate::B250k :
                bRate == "B3M"s? BaudRate::B3M   :
                              BaudRate::B250k;
                break;
            }
            case 'm':
            {
                using namespace std::string_literals;
                const std::string lMode{optarg};
                mode =
                lMode == "2D"s? Mode::Mode2D :
                lMode == "3D"s? Mode::Mode3D :
                lMode == "Dual"s? Mode::Dual :
                Mode::Mode3D;
                break;
            }
            case 'c':
            {
                char* endptr = nullptr;
                lidarCfg.frequencyCh = ::strtoul(optarg, &endptr, 10u);
                if(endptr == optarg || lidarCfg.frequencyCh > 0xfu)
                {
                    std::cerr << "Failed to parse the option for frequency channel\n"
                                                    << usageString() << "\n";
                    return EXIT_FAILURE;
                }
                break;
            }
            case 'p':
            {
                using PulseDuration = CygLidarD1::PulseDuration;
                char* endptr = nullptr;
                auto pulse = ::strtoul(optarg, &endptr, 10u);
                if(endptr == optarg || pulse > 10000u)
                {
                    std::cerr << "Failed to parse the option for frequency channel\n"
                                                    << usageString() << "\n";
                    return EXIT_FAILURE;
                }

                lidarCfg.pulseDuration = PulseDuration(PulseMode::Fixed3D, pulse);
                break;
            }
            case 's':
            {
                char* endptr = nullptr;
                lidarCfg.sensitivity = ::strtoul(optarg, &endptr, 10u);
                if(endptr == optarg)
                {
                    std::cerr << "Failed to parse the option for sensitivity\n"
                                                    << usageString() << "\n";
                    return EXIT_FAILURE;
                }
                break;
            }
            case 'o':
            {
                outputFileName = std::string { optarg };
                break;
            }
            case 'h':
                std::cout << usageString() << "\n";
                return 0;
            default:
                std::cerr << "No options provided" << "\n" << usageString() << "\n";
                return EXIT_FAILURE;
        }
    }

    if( !argv[1] )
    {
        std::cerr << "No options provided\n" << usageString() << "\n";
        return EXIT_FAILURE;
    }

    try
    {
        if(!deviceName.empty())
        {
            std::cout << "Opening device: " << deviceName << "\n";
            input.createAndOpen<SerialPort>(deviceName, speedForSerial);
        }
        else if(!inputFileName.empty())
        {
            std::cout << "Opening file: " << inputFileName << "\n";
            input.createAndOpen<BinaryFile>(inputFileName);
        }
        else
        {
            std::cerr << "Can't have 2 separate lidar inputs\n" << usageString() << "\n";
            return EXIT_FAILURE;
        }

        std::cout << "Starting lidar\n";
        CygLidarD1 lidar{input};
        if(!deviceName.empty())
        {
            lidar.configure(lidarCfg);
            lidar.printDeviceInfo();
            lidar.run(mode);
        }

        std::cout << "Starting to read point clouds\n";
        PointCloudReader pointCloudReader{lidar};
        pointCloudReader.start(mode);

        if(!outputFileName.empty())
        {
            output.createAndOpen<BinaryFile>(outputFileName);
        }
        FrameWriter frameWriter{lidar, output};
        if(!outputFileName.empty())
        {
            frameWriter.start();
        }

        handleSignal = [&]
        (int sigNum)
        {
            std::cout << "Received signal : " << sigNum << ", trying to stop all resources\n";
            viewer.stop();
            lidar.stop();
            pointCloudReader.stop();
            input.close();
            frameWriter.stop();
        };
        switch(mode)
        {
            case Mode::Mode3D:
            {
                viewer.registerViewerFunction(lidar3D_Display, &lidar);
                break;
            }
            case Mode::Mode2D:
            {
                viewer.registerViewerFunction(lidar2D_Display, &lidar);
                break ;
            }
            case Mode::Dual:
            {
                 // TODO
                std::cout << "Dual mode to be added in future :)\n";
                // viewer->registerViewerFunction(lidar3D_Display, &lidar);
                // viewer->registerViewerFunction(lidar2D_Display, &lidar);
                // (?)
                return EXIT_SUCCESS;
            }
            default:
                viewer.registerViewerFunction(lidar3D_Display, &lidar);
                break;
        }

        std::cout << "Opening viewer\n";
        viewer.start(&argc, argv);

    }
    catch (std::exception const& exc )
    {
        viewer.stop();
        std::cerr << "Exception thrown during viewer runtime: " << exc.what() << '\n';
        return EXIT_FAILURE;
    }
    catch( ... )
    {
        std::cerr << "Unknown exception thrown during viewer runtime\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
