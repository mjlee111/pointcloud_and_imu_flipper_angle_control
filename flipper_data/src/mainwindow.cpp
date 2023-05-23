#include "../include/flipper_data/mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;
using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowIcon(QIcon(":/images/icon.jpg"));
    ros::init(init_argc,init_argv,"flipper_data");
    if ( ! ros::master::check() ) {
        return;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    flipper_FL = n.subscribe<std_msgs::Float64>("/target_flipper_FL", 10, &MainWindow::flipper_fl_callback, this);
    flipper_FR = n.subscribe<std_msgs::Float64>("/target_flipper_FR", 10, &MainWindow::flipper_fr_callback, this);
    flipper_BL = n.subscribe<std_msgs::Float64>("/target_flipper_BL", 10, &MainWindow::flipper_bl_callback, this);
    flipper_BR = n.subscribe<std_msgs::Float64>("/target_flipper_BR", 10, &MainWindow::flipper_br_callback, this);

    IMU = n.subscribe<sensor_msgs::Imu>("/imu_data", 10, &MainWindow::imu_callback, this);

    th=std::thread(&MainWindow::run,this);
    return;
}

MainWindow::~MainWindow()
{
    if(ros::isStarted()) {
        ROS_INFO("exiting");
        delete ui;
        th.detach();
        ros::shutdown(); // explicitly needed since we use ros::start();
    }
    th.join();
    delete ui;
}

void MainWindow::run(){
    ros::Rate loop_rate(10);
    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MainWindow::flipper_fl_callback(const std_msgs::Float64 input)
{
    QString inputs = "";
    inputs = QString::number(input.data);
    ui->FL->setText(inputs);
    cout << "FL ";
}

void MainWindow::flipper_fr_callback(const std_msgs::Float64 input)
{
    QString inputs = "";
    inputs = QString::number(input.data);
    ui->FR->setText(inputs);
    cout << "FR ";
}


void MainWindow::flipper_bl_callback(const std_msgs::Float64 input)
{
    QString inputs = "";
    inputs = QString::number(input.data);
    ui->BL->setText(inputs);
    cout << "BL ";
}


void MainWindow::flipper_br_callback(const std_msgs::Float64 input)
{
    QString inputs = "";
    inputs = QString::number(input.data);
    ui->BR->setText(inputs);
    cout << "BR ";
}

void MainWindow::imu_callback(const sensor_msgs::Imu input)
{
    QString roll_s = "";
    QString pitch_s = "";
    QString yaw_s = "";
    roll_s = QString::number(input.orientation.x);
    pitch_s = QString::number(input.orientation.y);
    yaw_s = QString::number(input.orientation.z);
    ui->roll->setText(roll_s);
    ui->pitch->setText(pitch_s);
    ui->yaw->setText(yaw_s);
    cout << "IMU " << endl;
}