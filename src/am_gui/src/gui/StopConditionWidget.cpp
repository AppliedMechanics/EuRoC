/*
 * StopConditionWidget.cpp
 *
 *  Created on: Jul 23, 2014
 *      Author: euroc_admin
 */

#include "StopConditionWidget.h"

StopConditionWidget::StopConditionWidget(QWidget *parent):
vec_joint_names_(1),
vec_operator_(1),
vec_values_(1),
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
		value_[i]->setDecimals(0);
		value_[i]->setValue(100.0);
		value_[i]->setRange(0.0,200);
		gridLayout->addWidget(value_[i],i+1,3);

	}
	setLayout(mainLayout);

	rosinterface = ROSinterface::getInstance();

	connect(requestLimitsButton,SIGNAL(clicked()),this,SLOT(updateJointNames()));
	connect(setStopConditionButton,SIGNAL(clicked()),this,SLOT(callSetStopCondition()));
	connect(this,SIGNAL(emitSetStopCondition(std::vector<std::string>, std::vector<std::string>, std::vector<double>)),rosinterface,SLOT(callSetStopConditions(std::vector<std::string>,std::vector<std::string>,std::vector<double>)));
}

StopConditionWidget::~StopConditionWidget() {
	// TODO Auto-generated destructor stub
}

void StopConditionWidget::updateJointNames()
{
	// TODO Limits from real task description
	double force_limits[10];
	double security = 2.0;

	force_limits[0] = 120;
	force_limits[1] = 120;
	force_limits[2] = 176;
	force_limits[3] = 176;
	force_limits[4] = 100;
	force_limits[5] = 100;
	force_limits[6] = 100;
	force_limits[7] = 30;
	force_limits[8] = 30;
	force_limits[9] = 200;

	for (int i=0;i<10;i++)
	{
		joint_name_[i]->setText(QString::fromStdString(rosinterface->_telemetry.joint_names[i]));
		value_[i]->setValue(1.0/security*abs(rosinterface->_telemetry.measured.torque[i]-force_limits[i]));
	}

}

void StopConditionWidget::callSetStopCondition()
{
	num_active_ = 0;


	for (int i=0;i<10;i++)
		if (active_[i]->isChecked())
			num_active_++;

	vec_joint_names_.resize(num_active_);
	vec_operator_.resize(num_active_);
	vec_values_.resize(num_active_);

	int ii=0;

	for (int i=0;i<10;i++)
	{
		if (active_[i]->isChecked())
		{
			vec_joint_names_[ii] = joint_name_[i]->text().toStdString();
			vec_operator_[ii] = operator_[i]->currentText().toStdString();
			vec_values_[ii] = value_[i]->value();
			ii++;
		}

	}

	emit emitSetStopCondition(vec_joint_names_,vec_operator_,vec_values_);

}
