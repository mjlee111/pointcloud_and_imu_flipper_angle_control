# qtros
## This package is for arm processor PC like Jetson Nano.
#### I have been working on ARM based PC's like Jetson Nano. I need UI based ROS package but qt_ros_plugin doesn't supports arm64(aarch64). So I made a qt_ros template to solve this out. I've tested UDP communication package, cv based camera ui package and so on. This template will allow you to have GUI expereance with ROS melodic. 
This package will allow you to make QT based UI package without installing qt-ros-plugin which is not available to install on ARM PC's.
You can clone this package and use. 

### To make this possible, I used ~~_MultiThread_~~. I used different thread for ROS, and UI.
### I couldn't find any other solutions. If you have any other solution, let me know.
### Because this git repository contains qtros named package, you have to edit all the names if you want to create another package. So to make it easier please use my repo over here -> https://github.com/mjlee111/qt_create 

# Codes
## 1. src
#### mainwindow.cpp
```cpp
#include "../include/qtros/mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;
using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowIcon(QIcon(":/images/icon.jpg"));
    ros::init(init_argc,init_argv,"qtros");
    if ( ! ros::master::check() ) {
        return;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    th=std::thread(&MainWindow::run,this);
    return;
}

MainWindow::~MainWindow()
{
    if(ros::isStarted()) {
        ROS_INFO("exiting");
        ros::shutdown(); // explicitly needed since we use ros::start();
    }
    th.join();
    delete ui;
}

void MainWindow::run(){
    ros::Rate loop_rate(33);
    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
```

#### main.cpp
```cpp
#include "../include/qtros/mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w(argc, argv);
    w.show();

    return a.exec();
}
```

## 2. headers
#### mainwindow.h
```h
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

Q_SIGNALS:
    void rosShutdown();

private:
    Ui::MainWindow *ui;
    int init_argc;
    char** init_argv;
    std::thread th;
};

#endif // MAINWINDOW_H
```
