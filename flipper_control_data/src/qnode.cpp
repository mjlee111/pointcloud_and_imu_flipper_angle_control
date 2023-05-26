/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/flipper_control_data/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace flipper_control_data
{
  using namespace std;

  /*****************************************************************************
  ** Implementation
  *****************************************************************************/

  QNode::QNode(int argc, char **argv) : init_argc(argc),
                                        init_argv(argv)
  {
  }

  QNode::~QNode()
  {
    if (ros::isStarted())
    {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
  }

  bool QNode::init()
  {
    ros::init(init_argc, init_argv, "flipper_control_data");
    if (!ros::master::check())
    {
      return false;
    }
    // ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    flipper_FL = n.subscribe("/flipper_FL", 10, &QNode::flipper_fl_callback, this);
    flipper_FR = n.subscribe("/flipper_FR", 10, &QNode::flipper_fr_callback, this);
    flipper_BL = n.subscribe("/flipper_BL", 10, &QNode::flipper_bl_callback, this);
    flipper_BR = n.subscribe("/flipper_BR", 10, &QNode::flipper_br_callback, this);

    IMU = n.subscribe("/imu", 10, &QNode::imu_callback, this);
    cout << "START" << endl;
    // Add your ros communications here.
    start();
    return true;
  }

  void QNode::run()
  {
    ros::Rate loop_rate(33);
    while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
  }

  void QNode::flipper_fl_callback(const std_msgs::Float64 &input)
  {
    FLS = QString::number(toDEG(input.data));
    Q_EMIT FL_signal();
  }

  void QNode::flipper_fr_callback(const std_msgs::Float64 &input)
  {
    FRS = QString::number(toDEG(input.data));
    Q_EMIT FR_signal();
  }

  void QNode::flipper_bl_callback(const std_msgs::Float64 &input)
  {
    BLS = QString::number(toDEG(input.data));
    Q_EMIT BL_signal();
  }

  void QNode::flipper_br_callback(const std_msgs::Float64 &input)
  {
    BRS = QString::number(toDEG(input.data));
    Q_EMIT BR_signal();
  }

  void QNode::imu_callback(const sensor_msgs::Imu &input_imu)
  {
    float w, x, y, z, roll, pitch, yaw;
    w = input_imu.orientation.w;
    x = input_imu.orientation.x;
    y = input_imu.orientation.y;
    z = input_imu.orientation.z;

    float sinr_cosp = 2.0 * (w * x + y * z);
    float cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0)
      pitch = std::copysign(M_PI / 2.0, sinp);
    else
      pitch = std::asin(sinp);

    float siny_cosp = 2.0 * (w * z + x * y);
    float cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    imu_pitch = toDEG(roll);
    imu_roll = toDEG(pitch);
    imu_yaw = toDEG(yaw);
    Q_EMIT IMU_signal();
  }

  float QNode::toRAD(float deg)
  {
    return deg * M_PI / 180;
  }

  float QNode::toDEG(float rad)
  {
    return rad * 180 / M_PI;
  }

} // namespace flipper_control_data
