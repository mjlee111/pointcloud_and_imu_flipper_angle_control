#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/network.h>
#include <iostream>
#include <QtGui>
#include <thread>
#include <QThread>
#include <string>
#include <QStringListModel>
#include <QString>

#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#endif

namespace Ui {
using namespace std;
using namespace Qt;
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    virtual ~MainWindow();
    bool init();
    void run();

    ros::Subscriber IMU;
    ros::Subscriber flipper_FL;
    ros::Subscriber flipper_FR;
    ros::Subscriber flipper_BL;
    ros::Subscriber flipper_BR;

    void flipper_fl_callback(const std_msgs::Float64 input);
    void flipper_fr_callback(const std_msgs::Float64 input);
    void flipper_bl_callback(const std_msgs::Float64 input);
    void flipper_br_callback(const std_msgs::Float64 input);
    void imu_callback(const sensor_msgs::Imu input);


Q_SIGNALS:
    void rosShutdown();

private:
    Ui::MainWindow *ui;
    int init_argc;
    char** init_argv;
    std::thread th;
};

#endif // MAINWINDOW_H
