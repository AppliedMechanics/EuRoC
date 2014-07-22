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
	QGroupBox* measuredBox = new QGroupBox("Telemetry");
	QGridLayout* measuredGrid = new QGridLayout();


	for (int i=0;i<4;i++){
		description[i] = new QLabel("");
		measuredGrid->addWidget(description[i],0,i);

		description[i]->setStyleSheet("QLabel { background-color : white; font-weight : bold;}");
		if (i)
			description[i]->setAlignment(Qt::AlignRight);
			description[i]->setAlignment(Qt::AlignVCenter);
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

		jointLabels[i]->setStyleSheet("QLabel { qproperty-alignment: 'AlignVCenter | AlignLeft';}");
		jointPositionMeasure[i]->setStyleSheet("QLabel { qproperty-alignment: 'AlignVCenter | AlignRight';}");
		jointForceMeasure[i]->setStyleSheet("QLabel { qproperty-alignment: 'AlignVCenter | AlignRight';}");
		jointExternalForceMeasure[i]->setStyleSheet("QLabel { qproperty-alignment: 'AlignVCenter | AlignRight';}");

		if (i%2){
			jointLabels[i]->setStyleSheet("QLabel { background-color : white; qproperty-alignment: 'AlignVCenter | AlignLeft';}");
			jointPositionMeasure[i]->setStyleSheet("QLabel { background-color : white; qproperty-alignment: 'AlignVCenter | AlignRight';}");
			jointForceMeasure[i]->setStyleSheet("QLabel { background-color : white; qproperty-alignment: 'AlignVCenter | AlignRight';}");
			jointExternalForceMeasure[i]->setStyleSheet("QLabel { background-color : white; qproperty-alignment: 'AlignVCenter | AlignRight';}");
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
	connect(rosinterface,SIGNAL(emitMeasuredValues(std::vector<std::string>, double*, double*, double*)),this,SLOT(showMeasure(std::vector<std::string>,double*, double*, double*)));
}

void TelemetryWidget::showMeasure(std::vector<std::string> jointNames,double* position, double* force, double* extforce )
{

	for (int i=0;i<12;i++)
	{
		jointLabels[i]->setText(QString::fromStdString(jointNames[i]));
		jointPositionMeasure[i]->setText(QString::number(position[i],'f',3)+posUnit[isRevolute[i]]);
		jointForceMeasure[i]->setText(QString::number(force[i],'f',2)+forceUnit[isRevolute[i]]);
		jointExternalForceMeasure[i]->setText(QString::number(extforce[i],'f',2)+forceUnit[isRevolute[i]]);
	}

}




