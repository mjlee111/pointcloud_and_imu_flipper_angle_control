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
