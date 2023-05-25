/**
 * @file /include/flipper_control_data/main_window.hpp
 *
 * @brief Qt based gui for flipper_control_data.
 *
 * @date November 2010
 **/
#ifndef flipper_control_data_MAIN_WINDOW_H
#define flipper_control_data_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QString>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/visualization_manager.h>
#include <rviz/config.h>

// #include <rviz/ros_node_abstraction.h>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace flipper_control_data {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	rviz::RenderPanel* renderPanel_;
//	rviz::Display* pcl_raw_F;
//	rviz::Display* pcl_raw_B;
//	rviz::Display* pcl_FL;
//	rviz::Display* pcl_FR;
//	rviz::Display* pcl_BL;
//	rviz::Display* pcl_BR;
//	rviz::Display* marker_FL;
//	rviz::Display* marker_FR;
//	rviz::Display* marker_BL;
//	rviz::Display* marker_BR;
	

public Q_SLOTS:
	void FL_slot(void);
	void FR_slot(void);
	void BL_slot(void);
	void BR_slot(void);
	void IMU_slot(void);


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace flipper_control_data

#endif // flipper_control_data_MAIN_WINDOW_H
