# Lidar Viewer

Yet another app to view the point cloud from a lidar. Made specifically for __CygLidar D1__.

[![CMake on a Ubuntu](https://github.com/pawelsmyczek/lidar_viewer/actions/workflows/cmake-single-platform.yml/badge.svg?branch=main)](https://github.com/pawelsmyczek/lidar_viewer/actions/workflows/cmake-single-platform.yml)
---

## Motivation
The purpose is to create a bare minimum application for __Linux/Unix__ that can view data from CygLidar D1.
Right now its focus is 3D data, but more is to be added in future...

Currently, applications on the market for viewing CygLidar data require quite heavy dependencies:
either boost, ros, are written in py or are for Windows only etc... They are in my opinion, simply, not necessary :)

---

## Usage

### Dependencies

run:

    $ ./tools/install_tools.sh to install all needed dependencies

On debian based distributions run following command to install necessary for OpenGL (assuming OpenGL is not present):

    $ sudo apt install -y mesa-utils libglu1-mesa-dev freeglut3-dev mesa-common-dev libglew-dev libglfw3-dev libglm-dev libao-dev libmpg123-dev

## Build & run

### Build

Inside the cloned project repo, run following commands:

    $ mkdir build; cd build;
    $ cmake ..
    $ make

### Run

	./lidar_viewer
		-d [deviceName] 
		-f [inputFileName] 
		-b serialBaudRate { B57k6,B115k2,B250k,B3M } 
		-l lidarBaudRateConfig { B57k6,B115k2,B250k,B3M } 
		-m lidarMode { 2D,3D,Dual } 
		-c frequencyChannel {0-15} 
		-p pulseDuration (for 3D) { 0-10000 } [in ms] 
		-s sensitivity {0-255}
		-o outputFileName file to write frames to



## Unit Tests

On Ubuntu, you can install Google Test by running:

    $ sudo apt-get update
    $ sudo apt-get install libgtest-dev

Afterward, you need to build the library manually, as libgtest-dev only provides the source code:

    $ cd /usr/src/gtest
    $ sudo cmake .
    $ sudo make
    $ sudo cp *.a /usr/lib


#If you have problems with generation of code coverage try to install newest lcov version:

Use wget to download the latest tarball. For example, if the latest version is 1.16, you can do:
#wget https://github.com/linux-test-project/lcov/archive/refs/tags/v1.16.tar.gz

Extract the downloaded tarball outside the repo:

    $ tar -xvzf v1.16.tar.gz
    $ cd lcov-1.16

Build and Install lcov

    $ sudo make install

## UI functions

Arrows - move around the projection
'-' / '=' - zoom out / zoom in

### Demo

<img src="https://github.com/pawelsmyczek/lidar_viewer/blob/main/assets/demo22.gif" />

