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

## Package Description 
### realsense_filter 
#### This package calculates the angel from the object in the surface. Our team had to control four flippers, so I used two realsense camera(d435i) to get pointcloud data. 
![KakaoTalk_20230604_190737533](https://github.com/mjlee111/pointcloud_and_imu_flipper_angle_control/assets/66550892/33506bf5-38d4-4263-b509-22351ca19d45)
#### The input from the realsense camera is filtered to compress the raw data size, and remove the unneccesary parts. Three filters were used, descripted below.

