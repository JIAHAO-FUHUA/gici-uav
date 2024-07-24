##glog 0.6.0
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
sudo make install


##gflags
mkdir build && cd build
cmake -DBUILD_SHARED_LIBS=ON -DBUILD_STATIC_LIBS=ON -DINSTALL_HEADERS=ON -DINSTALL_SHARED_LIBS=ON -DINSTALL_STATIC_LIBS=ON ..
make -j4
sudo make install

##ceres-solver 2.1.0
sudo apt install liblapack-dev libsuitesparse-dev libcxsparse3 libgtest-dev
mkdir build && cd build
make -j4
sudo make install

##opencv 4.7.0
if you use ubuntu 20.04, you don't have to install opencv-4.7.0 from source code again

##gici-lib [only difference between intel and arm is the CMakeLists.txt and third_party/]
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
cd ros_wrapper
catkin_make .. -DCMAKE_BUILD_TYPE=Release 

##run gici-lib in ros mode
source devel/setup.bash
rosrun gici_ros gici_ros_main  XXXXXXXXX.yaml



