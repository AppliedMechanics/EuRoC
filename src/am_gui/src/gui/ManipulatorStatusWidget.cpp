/*
 * Copyright (C) Institute of Applied Mechanics, TU Muenchen
 * All rights reserved.
 * Contact: schuetz@amm.mw.tum.de
 */

#include "ManipulatorStatusWidget.hpp"
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
	callNextObjectButton_ = new QPushButton("&Next Object on belt");

	//Dropdown List
	taskSelection = new QComboBox(this);

	//add Widgets to layouts
	buttonLayout->addWidget(startButton);
	buttonLayout->addWidget(stopButton);
	buttonLayout->addWidget(callNextObjectButton_);
	mainLayout->addWidget(taskSelection);
	mainLayout->addLayout(buttonLayout);


	//  connect(stopButton, SIGNAL(clicked()), this, SLOT(stopPressed()));
	//  connect(resetButton, SIGNAL(clicked()), this, SLOT(resetPressed()));
	setLayout(mainLayout);


	// Qt Connections
	connect(startButton, SIGNAL(clicked()),this,SLOT(startSimulatorWithTask()));

	connect(this,SIGNAL(callStartSimulator(std::string)),rosinterface,SLOT(callStartSimulator(std::string)));
	connect(stopButton, SIGNAL(clicked()),rosinterface,SLOT(callStopSimulator()));
	connect(callNextObjectButton_,SIGNAL(clicked()),rosinterface,SLOT(callNextObject()));

	connect(taskSelection,SIGNAL(currentIndexChanged(QString)),this,SLOT(taskSelectionChanged(QString)));

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

void ManipulatorStatusWidget::taskSelectionChanged(QString curr_selection)
{
	callNextObjectButton_->setEnabled(curr_selection.startsWith("task6"));
}
