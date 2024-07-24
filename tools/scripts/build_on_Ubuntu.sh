#!/bin/sh
# A script to build the main program on Ubuntu. 
# Tested in x86 environment (Ubuntu 22.04 running in VM and WSL).
# Run from the project root directory.

# Record project root path
cwd=$(pwd)

# Update package list
sudo apt update

# Install dependencies
sudo apt install -y vim cmake git build-essential 
sudo apt install -y libeigen3-dev libopencv-dev libyaml-cpp-dev libgflags-dev libgoogle-glog-dev

# Install Ceres Solver dependencies
sudo apt install -y libatlas-base-dev libsuitesparse-dev wget

# Build Ceres Solver 2.1.0 from source
# GICI requires ceres-solver >= 2.1.0 (in README). In version 2.2.0, Ceres-solver removed 
# LocalParameterization interface, which is used in GICI. So we can only install 2.1.0.
cd ~/Downloads
wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz
tar zxf ceres-solver-2.1.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.1.0
make -j8
sudo make install

# Build main program
cd $cwd
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
