cmake_minimum_required(VERSION 3.0.2)
project(realsense_filter)

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
