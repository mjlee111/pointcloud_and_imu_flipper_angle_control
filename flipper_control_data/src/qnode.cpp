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

    flipperF = n.subscribe("/flipper_front", 10, &QNode::flipper_f_callback, this);
    flipperB = n.subscribe("/flipper_back", 10, &QNode::flipper_b_callback, this);

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

  void QNode::flipper_f_callback(const std_msgs::Float64MultiArray &input)
  {
    FLS = QString::number(toDEG(input.data[0]));
    FRS = QString::number(toDEG(-input.data[1]));
    Q_EMIT FL_signal();
    Q_EMIT FR_signal();
  }

  void QNode::flipper_b_callback(const std_msgs::Float64MultiArray &input)
  {
    BLS = QString::number(toDEG(input.data[0]));
    BRS = QString::number(toDEG(-input.data[1]));
    Q_EMIT BL_signal();
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
