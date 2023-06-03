/**
 * @file /include/flipper_control_data/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef flipper_control_data_QNODE_HPP_
#define flipper_control_data_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <QString>
#include <std_msgs/Float64MultiArray.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace flipper_control_data
{

  /*****************************************************************************
  ** Class
  *****************************************************************************/

  class QNode : public QThread
  {
    Q_OBJECT
  public:
    QNode(int argc, char **argv);
    virtual ~QNode();
    bool init();
    void run();

    QString FLS, FRS, BLS, BRS;
    float imu_roll = 0;
    float imu_pitch = 0;
    float imu_yaw = 0;


    float toRAD(float deg);
    float toDEG(float rad);


  Q_SIGNALS:
    void rosShutdown();
    void FL_signal();
    void FR_signal();
    void BL_signal();
    void BR_signal();
    void IMU_signal();

  private:
    int init_argc;
    char **init_argv;

    void flipper_f_callback(const std_msgs::Float64MultiArray &input);
    void flipper_b_callback(const std_msgs::Float64MultiArray &input);
    void imu_callback(const sensor_msgs::Imu &input_imu);
    ros::Subscriber IMU;
    ros::Subscriber flipperF;
    ros::Subscriber flipperB;
    ros::Subscriber flipper_FL;
    ros::Subscriber flipper_FR;
    ros::Subscriber flipper_BL;
    ros::Subscriber flipper_BR;
  };
} // namespace flipper_control_data

#endif /* flipper_control_data_QNODE_HPP_ */
