/*
 * Copyright (C) Institute of Applied Mechanics, TU Muenchen
 * All rights reserved.
 * Contact: schuetz@amm.mw.tum.de
 */

#include "manipulatorstatuswidget.hpp"
//#include "ROSinterface.hpp"	


ManipulatorStatusWidget::ManipulatorStatusWidget(QWidget *parent) :
QWidget(parent)
{
	rosinterface = ROSinterface::getInstance();

	//Layout
	QHBoxLayout* mainLayout = new QHBoxLayout();
	QVBoxLayout* buttonLayout = new QVBoxLayout();


	//Buttons
	QPushButton* startButton = new QPushButton("&Start Simulator");
	QPushButton* stopButton = new QPushButton("&Stop Simulator");

	//Dropdown List
	taskSelection = new QComboBox(this);

	//add Widgets to layouts
	buttonLayout->addWidget(startButton);
	buttonLayout->addWidget(stopButton);
	mainLayout->addWidget(taskSelection);
	mainLayout->addLayout(buttonLayout);

	//  connect(stopButton, SIGNAL(clicked()), this, SLOT(stopPressed()));
	//  connect(resetButton, SIGNAL(clicked()), this, SLOT(resetPressed()));
	setLayout(mainLayout);


	// Qt Connections
	connect(startButton, SIGNAL(clicked()),this,SLOT(startSimulatorWithTask()));

	connect(this,SIGNAL(callStartSimulator(std::string)),rosinterface,SLOT(callStartSimulator(std::string)));
	connect(stopButton, SIGNAL(clicked()),rosinterface,SLOT(callStopSimulator()));

	updateTasks();

}

void ManipulatorStatusWidget::updateTasks()
{
	for (int i=0;i<rosinterface->scenes_.size();i++)
	{
		taskSelection->addItem(QString::fromStdString(rosinterface->scenes_[i].name));
	}
}

void ManipulatorStatusWidget::startSimulatorWithTask()
{
	emit callStartSimulator(taskSelection->currentText().toStdString());
}
