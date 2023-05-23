cmake_minimum_required(VERSION 3.0.2)
project(realsense_filter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  std_msgs
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/realsense_filter.cpp)
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
