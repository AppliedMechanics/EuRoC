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

	for (int i=1;i<=6;i++)
	{
		taskSelection->addItem("Task "+QString::number(i));
	}
	//  connect(stopButton, SIGNAL(clicked()), this, SLOT(stopPressed()));
	//  connect(resetButton, SIGNAL(clicked()), this, SLOT(resetPressed()));
	setLayout(mainLayout);


	// Qt Connections
	connect(startButton, SIGNAL(clicked()),this,SLOT(startSimulatorWithTask()));

	connect(this,SIGNAL(callStartSimulator(int)),rosinterface,SLOT(callStartSimulator(int)));
	connect(stopButton, SIGNAL(clicked()),rosinterface,SLOT(callStopSimulator()));



}



void ManipulatorStatusWidget::startSimulatorWithTask()
{
	emit callStartSimulator(taskSelection->currentIndex());
}
