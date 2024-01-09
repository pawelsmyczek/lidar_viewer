#include "lidar_viewer/dev/SerialPort.h"
#include "lidar_viewer/dev/CygLidarD1.h"
#include "lidar_viewer/ui/Viewer.h"
#include "lidar_viewer/ui/DisplayFunctions.h"


#include <iostream>
#include <thread>
#include <chrono>

#include <unistd.h>
#include <csignal>
#include <termios.h>

namespace
{

lidar_viewer::dev::CygLidarD1* cygLidarD1Ptr = nullptr;
lidar_viewer::ui::Viewer* viewerPtr = nullptr;
lidar_viewer::dev::SerialPort* serialPtr = nullptr;

void handleSigTerm(int )
{
    if (!serialPtr || !cygLidarD1Ptr || !viewerPtr )
    {
        return ;
    }
    viewerPtr->stop();
    cygLidarD1Ptr->stop();
    serialPtr->close();
}

std::string usageString()
{
    using namespace std::string_literals;
    auto usageStr = "Usage: \n\t./lidar_viewer"
            "\n\t\t-d [deviceName] "
            "\n\t\t-b serialBaudRate { B57k6,B115k2,B250k,B3M } "
            "\n\t\t-l lidarBaudRateConfig { B57k6,B115k2,B250k,B3M } "
            "\n\t\t-m lidarMode { 2D,3D,Dual } "
            "\n\t\t-f frequencyChannel {0-15} "
            "\n\t\t-p pulseDuration (for 3D) { 0-10000 } [in ms] "
            "\n\t\t-s sensitivity {0-255}"s;
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

    ::signal(SIGTERM, handleSigTerm);
    ::signal(SIGINT, handleSigTerm); // Ctrl-C...

    auto opt = 0;
    CygLidarD1::Config lidarCfg{};
    Mode mode{};
    int speedForSerial {};
    std::string deviceName {};

    while(( opt = ::getopt(argc, argv, "d:b:l:m:f:p:s:h")) != -1 )
    {
        switch ( opt )
        {
            case 'd':
                deviceName = std::string { optarg };
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
            case 'f':
            {
                char* endptr = nullptr;
                lidarCfg.frequencyCh = ::strtoul(optarg, &endptr, 10u);
                if(endptr == optarg || lidarCfg.frequencyCh > 0xfu)
                {
                    std::cerr << "Failed to parse the option for frequency channel\n"
                                                    << usageString() << "\n";
                    return -1;
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
                    return -1;
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
                    return -1;
                }
                break;
            }
            case 'h':
                std::cout << usageString() << "\n";
                return 0;
            default:
                std::cerr << "No options provided" << "\n" << usageString() << "\n";
                return -1;
        }
    }

    Viewer::Config viewerCfg
            {
                .x = 0u,
                .y = 0u,
                .w = 1596u, // 160 / 60 = 2.66(7)
                .h = 600u,
                .viewType = Viewer::ViewType::PointCloud
            };

    if(!argv[1])
    {
        std::cerr << "No options provided\n" << usageString() << "\n";
        return -1;
    }

    try
    {
        std::cout << "Opening device: " << deviceName << "\n";

        SerialPort serial{deviceName};
        serialPtr = &serial;
        serial.configure(speedForSerial);

        std::cout << "Starting lidar\n";
        CygLidarD1 lidar{serial};
        cygLidarD1Ptr = &lidar;
        lidar.configure(lidarCfg);
        lidar.printDeviceInfo();

        lidar.start(mode);

        std::cout << "Opening viewer\n";

        Viewer viewer{viewerCfg, &argc, argv};
        viewerPtr = &viewer;
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
                // viewer.registerViewerFunction(lidar3D_Display, &lidar);
                // viewer.registerViewerFunction(lidar2D_Display, &lidar);
                // (?)
                return 0;
            }
            default:
                viewer.registerViewerFunction(lidar3D_Display, &lidar);
                break;
        }

        viewer.start();

    }
    catch ( std::exception& exc )
    {
        std::cerr << "Exception thrown during viewer runtime: " << exc.what() << "\n";
    }
    catch( ... )
    {
        std::cerr << "Unknown exception thrown during viewer runtime\n";
    }

    return 0;
}
