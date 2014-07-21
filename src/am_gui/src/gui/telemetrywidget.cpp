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


	for (int i=0;i<4;i++){
		description[i] = new QLabel("");
		measuredGrid->addWidget(description[i],0,i);
		description[i]->setStyleSheet("QLabel { background-color : white;}");
		if (i)
			description[i]->setAlignment(Qt::AlignRight);
	}
	description[0]->setText("Joint Name");
	description[1]->setText("Position");
	description[2]->setText("Force");
	description[3]->setText("External Force");

	for (int i=0;i<12;i++)
	{
		jointLabels[i]						= new QLabel("none");
		jointPositionMeasure[i]				= new QLabel("0.0 m");
		jointForceMeasure[i]				= new QLabel("0.0 Nm");
		jointExternalForceMeasure[i]		= new QLabel("0.0 Nm");

		jointLabels[i]->setAlignment(Qt::AlignLeft);
		jointPositionMeasure[i]->setAlignment(Qt::AlignRight);
		jointForceMeasure[i]->setAlignment(Qt::AlignRight);
		jointExternalForceMeasure[i]->setAlignment(Qt::AlignRight);

		if (i%2){
			jointLabels[i]->setStyleSheet("QLabel { background-color : white;}");
			jointPositionMeasure[i]->setStyleSheet("QLabel { background-color : white;}");
			jointForceMeasure[i]->setStyleSheet("QLabel { background-color : white;}");
			jointExternalForceMeasure[i]->setStyleSheet("QLabel { background-color : white;}");
		}
		measuredGrid->addWidget(jointLabels[i],i+1,0);
		measuredGrid->addWidget(jointPositionMeasure[i],i+1,1);
		measuredGrid->addWidget(jointForceMeasure[i],i+1,2);
		measuredGrid->addWidget(jointExternalForceMeasure[i],i+1,3);

		if ((i<2) || (i==9))
			isRevolute[i] = false;
		else
			isRevolute[i] = true;

		posUnit[0] = QString("\tm");
		forceUnit[0] = QString("\tN");
		posUnit[1] = QString("\trad");
		forceUnit[1] = QString("\tNm");
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
		jointPositionMeasure[i]->setText(QString::number(position[i],'f',3)+posUnit[isRevolute[i]]);
		jointForceMeasure[i]->setText(QString::number(force[i],'f',2)+forceUnit[isRevolute[i]]);
		jointExternalForceMeasure[i]->setText(QString::number(extforce[i],'f',2)+forceUnit[isRevolute[i]]);
	}

}




