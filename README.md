# Flipper Angle Control With Pointcloud and IMU data
## This package is developed to control crawler robot's flipper autonomously. Used for team RO:BIT, 2023 Robocup Rescue bordeaux.
## The concept of this package is simple. - Calculate the angle from the object, get feedback from the IMU data, and publish the data. 
## Development Environment
### - Ubuntu 18.04
### - ROS melodic

## REQUIREMENTS
### realsense_filter node 
#### - Realsense SDK 2.0 
####   https://www.intelrealsense.com/sdk-2/ : Intel realsense official site
#### or
```shell
sudo apt update
sudo apt install ros-melodic-librealsense*
```
#### - Realsense ros package 
```shell 
sudo apt update
sudo apt install ros-melodic-realsense2*
```

#### - PCL ros 
```shell 
sudo apt-get install ros-melodic-pcl-ros && sudo apt-get install ros-melodic-pcl-conversions
```
### flipper_control_data node
#### - QT4 build depends
```shell
sudo apt-get install ros-melodic-ddynamic-reconfigure && sudo apt-get install ros-melodic-qt-build
```

## How to use
###
```shell 
roslaunch realsense_filter realsense_flipper.launch 
```
### Launch parameters
#### - visuals (true/false) : rviz & flipper data UI(flipper_control_data node) available default -> false
#### - rs_visuals (on/off) : realsense_filter node pointcloud data publish parameter default -> off
#### - front (on/off) : front two fillper control parameter default -> on
#### - back (on/off) : back two flipper control parameter default -> on

## Package Description 
### realsense_filter 
#### This package calculates the angel from the object in the surface. Our team had to control four flippers, so I used two realsense camera(d435i) to get pointcloud data. 
![KakaoTalk_20230604_190737533](https://github.com/mjlee111/pointcloud_and_imu_flipper_angle_control/assets/66550892/33506bf5-38d4-4263-b509-22351ca19d45)
#### The input from the realsense camera is filtered to compress the raw data size, and remove the unneccesary parts. Three filters were used, descripted below.
#### - Passthrough Filter
![image](https://github.com/mjlee111/pointcloud_and_imu_flipper_angle_control/assets/66550892/cdb386fd-8195-4826-a262-4e84d4a7528d)
#### - Voxel Grid Filter 
![image](https://github.com/mjlee111/pointcloud_and_imu_flipper_angle_control/assets/66550892/135fe60f-a172-46e9-8a47-143cff9ff9cd)
#### - Outline Removal Filter 
![image](https://github.com/mjlee111/pointcloud_and_imu_flipper_angle_control/assets/66550892/e6646897-e953-41b3-8cc0-e024c8c477fd)
####
#### - Three Filters at once
![image](https://github.com/mjlee111/pointcloud_and_imu_flipper_angle_control/assets/66550892/8e39d5f4-a71e-4f90-b67a-e3be5bed26f3)
#### Using These Filters, two pointcloud raw data is splitted to four pointcloud data for flippers each.
#### Four pointcloud data are used to calculate the angle of the object from the surface. Simple math is used in this process. 
#### atan() function returns a float data from the input. This function refers to arctangent. 
#### From the pointcloud data, a point which has the biggest Z coordinate data is founded. The Z coordinate and Y coordinate can be used as height of the object and distance from the robot each.
#### Having Z and Y data, tangent value can be derived. Finaly, the target angle's value equals to atan(tan(angle)) = atan(tan(Y/Z)) 
#### Here is a simple image that explains the description.
![그림1](https://github.com/mjlee111/pointcloud_and_imu_flipper_angle_control/assets/66550892/2e068655-2e60-45db-bcfb-76b141eef201)
####
#### Back flipper simple test video without IMU feedback. (click)
[![Video Label](http://img.youtube.com/vi/kS8Jb-Vb5oQ/0.jpg)](https://youtu.be/kS8Jb-Vb5oQ)
#### Four flipper angle calculation test in Gazebo
![image10](https://github.com/mjlee111/pointcloud_and_imu_flipper_angle_control/assets/66550892/d520463a-ce0e-4fed-a2f2-410af5514067)



