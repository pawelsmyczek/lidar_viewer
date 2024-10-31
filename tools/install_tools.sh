#!/bin/bash

# Install Gcc 10
sudo apt install gcc-10 g++-10 -y

# Add Kitware repository
sudo apt install apt-transport-https ca-certificates gnupg -y
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://apt.kitware.com/kitware-archive.sh | sudo bash

# Install CMake
sudo apt install cmake -y

# Install Make
sudo apt install make -y

# Install core depencencies
sudo apt install -y mesa-utils libglu1-mesa-dev freeglut3-dev mesa-common-dev libglew-dev libglfw3-dev libglm-dev libao-dev libmpg123-dev

# Install Gtest
sudo apt-get install libgtest-dev -y
cd /usr/src/gtest
sudo cmake .
sudo make
sudo cp *.a /usr/lib

# Install Lcov
sudo apt-get install lcov -y