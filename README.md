# Flipper Angle Control With Pointcloud and IMU data
## This package is developed to control crawler robot's flipper autonomously. Used for team RO:BIT, 2023 Robocup Rescue bordeaux.
## The concept of this package is simple. - Calculate the angle from the object, get feedback from the IMU data, and publish the data. 


## REQUIREMENTS
### realsense_filter node 
#### - Realsense SDK 2.0 
####   https://www.intelrealsense.com/sdk-2/ : Intel realsense official site
#### - Realsense ros package 
```shell 
cd
mkdir -p realsesne_ws/src
cd realsense_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1` 
cd ..
catkin_init_workspace
cd .. 
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
echo "source ~/realsense_ws/devel/setup.bash" >> ~/.bashrc
soruce ~/.bashrc
```

#### - PCL ros 
```shell 
sudo apt-get install ros-melodic-pcl-ros && sudo apt-get install ros-melodic-pcl-conversions
```
### flipper_control_data node
#### - QT4
