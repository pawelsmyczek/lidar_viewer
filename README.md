# Lidar Viewer

Yet another app to view the point cloud from a lidar. Made specifically for __CygLidar D1__.

---

## Motivation
The purpose is to create a bare minimum application for __Linux/Unix__ that can view data from CygLidar D1.
Right now its focus is 3D data, but more is to be added in future...

Currently, applications on the market for viewing CygLidar data require quite heavy dependencies:
either boost, ros, are written in py or are for Windows only etc... They are in my opinion, simply, not necessary :)

---

## Usage

### Dependencies

On debian based distributions run following command to install necessary for OpenGL (assuming OpenGL is not present):

$ sudo apt install -y mesa-utils libglu1-mesa-dev freeglut3-dev mesa-common-dev libglew-dev libglfw3-dev libglm-dev libao-dev libmpg123-dev

### Build & run

## Build

Inside the cloned project repo, run following commands:

$ mkdir build; cd build;

$ cmake ..

$ make

## Run

./lidar_viewer
            -d [deviceName]
            -b serialBaudRate { B57k6,B115k2,B250k,B3M }
            -l lidarBaudRateConfig { B57k6,B115k2,B250k,B3M }
            -f frequencyChannel {0-15}
            -p pulseDuration (for 3D) { 0-10000 } [in ms]
            -s sensitivity {0-255}

## UI functions

Arrows - move around the projection
'-' / '=' - zoom out / zoom in

