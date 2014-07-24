/*
 * StopConditionWidget.cpp
 *
 *  Created on: Jul 23, 2014
 *      Author: euroc_admin
 */

#include "StopConditionWidget.h"

StopConditionWidget::StopConditionWidget(QWidget *parent):
QWidget(parent)
{
	// TODO Auto-generated constructor stub

	QVBoxLayout* mainLayout = new QVBoxLayout;
	QGridLayout* gridLayout = new QGridLayout;

	QPushButton* requestLimitsButton = new QPushButton("&Request Current Limits");
	QPushButton* setStopConditionButton = new QPushButton("&Set Stopping Conditions");

	mainLayout->addLayout(gridLayout);
	mainLayout->addWidget(requestLimitsButton);
	mainLayout->addWidget(setStopConditionButton);

	QLabel* title_1 = new QLabel("Active");
	QLabel* title_2 = new QLabel("Joint");
	QLabel* title_3 = new QLabel("Threshhold");

	gridLayout->addWidget(title_1,0,0);
	gridLayout->addWidget(title_2,0,1);
	gridLayout->addWidget(title_3,0,2);

	for (int i=0;i<10;i++)
	{
		active_[i] = new QCheckBox;
		gridLayout->addWidget(active_[i],i+1,0);

		joint_name_[i] = new QLabel("none");
		gridLayout->addWidget(joint_name_[i],i+1,1);
		joint_name_[i]->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

		operator_[i] = new QComboBox;
		operator_[i]->addItem("<");
		operator_[i]->addItem(">");
		operator_[i]->addItem("|<");
		operator_[i]->addItem("|>");
		gridLayout->addWidget(operator_[i],i+1,2);

		value_[i] = new QDoubleSpinBox;
		value_[i]->setValue(100.0);
		gridLayout->addWidget(value_[i],i+1,3);

	}
	setLayout(mainLayout);

	ROSinterface* rosinterface = ROSinterface::getInstance();

	connect(requestLimitsButton,SIGNAL(clicked()),rosinterface,SLOT(sendCurrentCfgOnce()));
	connect(rosinterface,SIGNAL(emitCurrentCfgOnce(std::vector<std::string>,double*)),this,SLOT(updateJointNames(std::vector<std::string>,double*)));

}

StopConditionWidget::~StopConditionWidget() {
	// TODO Auto-generated destructor stub
}

void StopConditionWidget::updateJointNames(std::vector<std::string> joint_names, double* currentValues)
{
	for (int i=0;i<10;i++)
	{
		joint_name_[i]->setText(QString::fromStdString(joint_names[i]));
	}

}
