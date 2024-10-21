# Kinect V2 Drivers

## Install Build Tools and Libraries
```
sudo apt-get install build-essential cmake pkg-config  -y  # Build tools
sudo apt-get install libusb-1.0-0-dev          # libusb
sudo apt-get install libturbojpeg0-dev -y      # TurboJPEG
sudo apt-get install libglfw3-dev -y           # OpenGL
sudo apt-get install libva-dev libjpeg-dev -y  # VAAPI for Intel only
sudo apt-get install libopenni2-dev -y         # OpenNI2
```

## Build Libfreenect 2
```
cd
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
make install
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
```
plug the Kinect and run the test program. The test program can be found in the compiled project’s bin folder of the repository.
```
cd ~/libfreenect2/build
./bin/Protonect
```

## IAI Kinect2 for OpenCV 4
```
cd ~/catkin_ws/src/
git clone https://github.com/paul-shuvo/iai_kinect2_opencv4.git
cd iai_kinect2_opencv4
rosdep install -r --from-paths .
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"

# If you get the following error "pcl_conversions build error: PCL requires C++14 or above", 
# Go to your CMakeLists.txt in the src folder and add the line "set( CMAKE_CXX_STANDARD 14)" on the very top, and rebuild.

source devel/setup.bash
```

Now, we can start the service by

roslaunch kinect2_bridge kinect2_bridge.launch

Now we can check if it is working by

# normal
rosrun kinect2_viewer kinect2_viewer

# For the superimposed images: 
rosrun kinect2_viewer kinect2_viewer hd image
    
# For the point cloud images: 
rosrun kinect2_viewer kinect2_viewer hd cloud

# For the superimposed and point cloud images: 
rosrun kinect2_viewer kinect2_viewer hd both
```