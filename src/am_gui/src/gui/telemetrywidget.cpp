/*
 * telemetrywidget.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: Christopph Schuetz
 */
#include "telemetrywidget.hpp"


TelemetryWidget::TelemetryWidget(QWidget *parent):
QWidget(parent)
{

	// layouts
	QHBoxLayout* mainLayout = new QHBoxLayout();
	QGroupBox* measuredBox = new QGroupBox("Measured");
	QGridLayout* measuredGrid = new QGridLayout();

	for (int i=0;i<12;i++)
	{
		jointLabels[i]						= new QLabel("none");
		jointPositionMeasure[i]				= new QLabel("0.0 m");
		jointForceMeasure[i]				= new QLabel("0.0 Nm");
		jointExternalForceMeasure[i]		= new QLabel("0.0 Nm");

		measuredGrid->addWidget(jointLabels[i],i,0);
		measuredGrid->addWidget(jointPositionMeasure[i],i,1);
		measuredGrid->addWidget(jointForceMeasure[i],i,2);
		measuredGrid->addWidget(jointExternalForceMeasure[i],i,3);

	}

	setLayout(mainLayout);
	mainLayout->addWidget(measuredBox);
	measuredBox->setLayout(measuredGrid);

	//! Connect with ROSinterface
	ROSinterface* rosinterface = ROSinterface::getInstance();
connect(rosinterface,SIGNAL(emitMeasuredValues(QString*, double*, double*, double*)),this,SLOT(showMeasure(QString*,double*, double*, double*)));
}

void TelemetryWidget::showMeasure(QString* jointNames,double* position, double* force, double* extforce )
{

	for (int i=0;i<12;i++)
	{
		jointLabels[i]->setText(jointNames[i]);
		jointPositionMeasure[i]->setText(QString::number(position[i]));
		jointForceMeasure[i]->setText(QString::number(force[i]));
		jointExternalForceMeasure[i]->setText(QString::number(extforce[i]));
	}

}




