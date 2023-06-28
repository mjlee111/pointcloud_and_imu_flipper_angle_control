/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/flipper_control_data/main_window.hpp"
#include <QString>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace flipper_control_data
{

	using namespace Qt;
	using namespace std;
	/*****************************************************************************
	** Implementation [MainWindow]
	*****************************************************************************/

	MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
		: QMainWindow(parent), qnode(argc, argv)
	{
		ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
		qnode.init();
		setWindowIcon(QIcon(":/images/icon.png"));
		QObject::connect(&qnode, SIGNAL(FL_signal()), this, SLOT(FL_slot()));
		QObject::connect(&qnode, SIGNAL(FR_signal()), this, SLOT(FR_slot()));
		QObject::connect(&qnode, SIGNAL(BL_signal()), this, SLOT(BL_slot()));
		QObject::connect(&qnode, SIGNAL(BR_signal()), this, SLOT(BR_slot()));
		QObject::connect(&qnode, SIGNAL(IMU_signal()), this, SLOT(IMU_slot()));
		QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(quit_slot()));
	}

	MainWindow::~MainWindow() {}

	void MainWindow::quit_slot(void)
	{
		this->close();
	}

	void MainWindow::FL_slot(void)
	{
		ui.FL->setText(qnode.FLS);
	}

	void MainWindow::FR_slot(void)
	{
		ui.FR->setText(qnode.FRS);
	}

	void MainWindow::BL_slot(void)
	{
		ui.BL->setText(qnode.BLS);
	}

	void MainWindow::BR_slot(void)
	{
		ui.BR->setText(qnode.BRS);
	}

	void MainWindow::IMU_slot(void)
	{
		ui.roll->setText(QString::number(qnode.imu_roll));
		ui.pitch->setText(QString::number(qnode.imu_pitch));
	}
	/*****************************************************************************
	** Implementation [Slots]
	*****************************************************************************/

} // namespace flipper_control_data
