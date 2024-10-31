#include "CygLidarD1.h"
#include "DisplayFunctions.h"
#include "Viewer.h"

#include <iostream>

int main(int argc, char **argv) {
  using namespace lidar_viewer::dev;
  using namespace lidar_viewer::ui;
  using namespace std::string_literals;
  using PulseMode = CygLidarD1::PulseDuration::PulseMode;
  using Mode = CygLidarD1::Mode;

  CygLidarD1::Config lidarCfg{
      .baudRate = CygLidarD1::BaudRate::B250k,
      .frequencyCh = 0xau,
      .pulseDuration = CygLidarD1::PulseDuration(PulseMode::Auto3D, 7000u),
      .sensitivity = 10u};
  const auto ioBaudRate = CygLidarD1::BaudRate::B250k;
  Viewer::Config viewerCfg{.x = 0u,
                           .y = 0u,
                           .w = 1596u, // 160 / 60 = 2.(6)
                           .h = 600u,
                           .viewType = Viewer::ViewType::PointCloud};

  std::cout << "Opening device\n";
  CygLidarD1 lidar{"/dev/ttyUSB0"s};
  lidar.ioconfigure(ioBaudRate);
  if (lidar.connected()) {
    std::cerr << "Lidar not connected!";
    return -1;
  }
  lidar.configure(lidarCfg);
  lidar.start(Mode::Mode3D);

  std::cout << "Opening viewer\n";

  Viewer viewer(viewerCfg, &argc, argv);
  viewer.registerViewerFunction(lidar3D_Display, &lidar);
  viewer.start();

  return 0;
}
