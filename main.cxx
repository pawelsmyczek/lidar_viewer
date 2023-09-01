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
    ::signal(SIGINT, handleSigTerm);

    auto opt = 0;
    CygLidarD1::Config lidarCfg{};
    int speedForSerial {};
    std::string deviceName {};

    while(( opt = ::getopt(argc, argv, "d:b:l:f:p:s:h")) != -1 )
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
            case 'f':
            {
                char* endptr = nullptr;
                auto freqCh = ::strtoul(optarg, &endptr, 10u);
                if(endptr == optarg || freqCh > 0xfu)
                {
                    std::cerr << "Failed to parse the option for frequency channel\n"
                                                    << usageString() << "\n";
                    return -1;
                }
                lidarCfg.frequencyCh = freqCh;
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
                auto sensitivity = ::strtoul(optarg, &endptr, 10u);
                if(endptr == optarg || sensitivity > 0xffu)
                {
                    std::cerr << "Failed to parse the option for sensitivity\n"
                                                    << usageString() << "\n";
                    return -1;
                }
                lidarCfg.sensitivity = sensitivity;
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
        std::cout << "Opening device: " << deviceName <<"\n";

        SerialPort serial{deviceName};
        serialPtr = &serial;
        serial.configure(speedForSerial);

        std::cout << "Starting lidar\n";
        std::this_thread::sleep_for(1s);
        CygLidarD1 lidar{serial};
        cygLidarD1Ptr = &lidar;
        lidar.configure(lidarCfg);
        std::this_thread::sleep_for(1s);
        lidar.start(Mode::Mode3D);

        std::cout << "Opening viewer\n";

        Viewer viewer{viewerCfg, &argc, argv};
        viewerPtr = &viewer;
        viewer.registerViewerFunction(lidar3D_Display, &lidar);
        viewer.start();

    }
    catch ( std::exception& exc )
    {
        std::cerr << "Exception thrown during viewer runtime: " << exc.what() << "\n";
    }

    return 0;
}
