/*
 * Copyright (C) Institute of Applied Mechanics, TU Muenchen
 * All rights reserved.
 * Contact: {baur, schuetz}@amm.mw.tum.de
 */


#include "MainWindow.h"
#include <QtGui>

using namespace std;

MainWindow::MainWindow(QWidget *parent)
: QMainWindow(parent)
{

	readSettings();
	createMenus();
	//the central widget of the main window should have tabs
	QTabWidget* tabWidget = new QTabWidget(this);
	QVBoxLayout* mainLayout = new QVBoxLayout();
	QHBoxLayout* splitLayout = new QHBoxLayout();
	QWidget* mainWidget = new QWidget();

	stateMachine = new ManipulatorStatusWidget;
//
//	onlinecontrol = new OnlineControlWidget;
	telemetry     = new TelemetryWidget;
	simplecontrol = new SimpleControlWidget;


	QCoreApplication::setOrganizationName("AMM");
	QCoreApplication::setApplicationName("RobotControl");



	//tabWidget->addTab(onlinecontrol, "Online Control");
	//tabWidget->addTab(telemetry, "Telemetry");
	tabWidget->addTab(simplecontrol, "Simple Control");


	mainLayout->setSpacing(5);
	mainLayout->addWidget(stateMachine);
	mainLayout->addLayout(splitLayout);
	splitLayout->addWidget(telemetry);
	splitLayout->addWidget(tabWidget);

	mainLayout->addStretch(1);

	mainWidget->setLayout(mainLayout);

	setCentralWidget(mainWidget);

	//ROS
	//generate timer to call ros::spin() for callback functions
	QTimer *timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(RosSpin()));
	timer->start(500);

}



MainWindow::~MainWindow()
{
	delete stateMachine;

}

void MainWindow::RosInit()
{

}


void MainWindow::RosSpin()
{
	if(ros::ok())
	{
		ros::spinOnce();
		//qDebug()<<"ROS spin called";
	}
	else
		quit();
}


void MainWindow::createMenus()
{
	// Create a "File" menu
	QMenu* fileMenu = menuBar()->addMenu(tr("File"));


	// Add exit action to the menu
	QAction* exitAction = new QAction(tr("Exit"), this);
	connect(exitAction, SIGNAL(triggered()), this, SLOT(quit()));
	fileMenu->addAction(exitAction);

}



void MainWindow::writeSettings()
{
	QSettings settings("AMM", "RobotControl");

	settings.beginGroup("MainWindow");
	settings.setValue("size", size());
	settings.setValue("pos", pos());
	settings.endGroup();
}

void MainWindow::readSettings()
{
	QSettings settings("AMM", "RobotControl");

	settings.beginGroup("MainWindow");
	resize(settings.value("size", QSize(1000, 600)).toSize());
	move(settings.value("pos", QPoint(200, 200)).toPoint());
	settings.endGroup();
}

void MainWindow::closeEvent(QCloseEvent *event)
{

	//  QMessageBox msgBox;
	//  msgBox.setText("Close Application?");
	//  msgBox.setInformativeText("Do you really want to close the application?");
	//  msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
	//  msgBox.setDefaultButton(QMessageBox::Cancel);
	//  msgBox.setIcon(QMessageBox::Question);
	//  msgBox.setWindowTitle("Close Application?");
	//  int ret = msgBox.exec();
	//
	//  if (ret == QMessageBox::Ok) {
	writeSettings();
	event->accept();
	//  } else {
	//    event->ignore();
	//  }
}



void MainWindow::quit()
{
	qApp->closeAllWindows();
}
