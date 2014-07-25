/*
 * ServoControlWidget.cpp
 *
 *  Created on: Jul 23, 2014
 *      Author: euroc_admin
 */

#include "ServoControlWidget.h"

ServoControlWidget::ServoControlWidget(QWidget *parent) {

	QGridLayout* mainLayout = new QGridLayout;
	setLayout(mainLayout);

	enable_servo_mode_ = new QCheckBox;
	QLabel* enable_label = new QLabel("Enable Servo Mode");

	signalMapperPressed_ = new QSignalMapper(this);



	signal_active_ = false;

	for (int i=0;i<12;i++)
	{
		joint_names_[i] = new QLabel("nn");

		joint_servo_offsets_[i] = new QSlider(Qt::Horizontal);
		joint_servo_offsets_[i]->setRange(-20,20);
		joint_servo_offsets_[i]->setTracking(true);
		joint_servo_offsets_[i]->setValue(0.0);
		joint_servo_offsets_[i]->setTickPosition(QSlider::TicksBelow);
		joint_servo_offsets_[i]->setEnabled(false);

		connect(joint_servo_offsets_[i],SIGNAL(sliderPressed()),signalMapperPressed_,SLOT(map()));
		connect(joint_servo_offsets_[i],SIGNAL(sliderReleased()),this,SLOT(sendZero()));
		//connect(joint_servo_offsets_[i],SIGNAL(valueChanged(int)),this,SLOT(sendServoCommand(int)));

		signalMapperPressed_->setMapping(joint_servo_offsets_[i],i);

		mainLayout->addWidget(joint_names_[i],i,0);
		mainLayout->addWidget(joint_servo_offsets_[i],i,1);
	}

    connect(signalMapperPressed_, SIGNAL(mapped(const int)), this, SIGNAL(sliderPressed(const int)));
    connect(this,SIGNAL(sliderPressed(const int)),this,SLOT(activateSignal(const int)));


	mainLayout->addWidget(enable_servo_mode_,0,2);
	mainLayout->addWidget(enable_label,0,3);

	rosinterface = ROSinterface::getInstance();

	connect(enable_servo_mode_,SIGNAL(stateChanged(int)),this, SLOT(enableServoModeChanged(int)));

	connect(this,SIGNAL(emitEnableServoMode(bool)),rosinterface,SLOT(callEnableServoMode(bool)));

}

ServoControlWidget::~ServoControlWidget() {
	// TODO Auto-generated destructor stub
}

void ServoControlWidget::enableServoModeChanged(int state)
{
	for (int i=0;i<12;i++)
	{
		joint_names_[i]->setText(QString::fromStdString(rosinterface->_telemetry.joint_names[i]));
	}

	emit emitEnableServoMode((bool)state);

	for (int i=0;i<12;i++)
		joint_servo_offsets_[i]->setEnabled(state);

}

void ServoControlWidget::activateSignal(const int slider_no)
{
	signal_active_ = true;
	while (joint_servo_offsets_[slider_no]->isSliderDown())
	{
		std::cout<< "Joint "<<slider_no<<"  value: "<<joint_servo_offsets_[slider_no]->value()<<std::endl;
	}

}

void ServoControlWidget::sendZero()
{
	for (int i=0;i<12;i++)
	{	joint_servo_offsets_[i]->setValue(0);
	}
	signal_active_ = false;
}

void ServoControlWidget::sendServoCommand(int value)
{

	std::cout << value << std::endl;
}
