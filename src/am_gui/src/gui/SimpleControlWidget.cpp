/*
 * Copyright (C) Institute of Applied Mechanics, TU Muenchen
 * All rights reserved.
 * Contact: schuetz@amm.mw.tum.de
 */

#include "SimpleControlWidget.hpp"
#include "ROSinterface.hpp"


SimpleControlWidget::SimpleControlWidget(QWidget *parent):
QWidget(parent)
{

	// layouts
	QVBoxLayout* mainLayout = new QVBoxLayout();

	QGroupBox* fkGroupBox = new QGroupBox("Forward Kinematics");
	QHBoxLayout* fkBoxLayout = new QHBoxLayout();
	QGridLayout* fkLayout = new QGridLayout();

	QGroupBox* targetPoseGroupBox = new QGroupBox("Workspace Control");
	QHBoxLayout* targetPoseControlLayout = new QHBoxLayout();
	QGridLayout* targetPoseLayout = new QGridLayout();


	QGroupBox* targetCfgGroupBox = new QGroupBox("Joint Control");
	QHBoxLayout* targetCfgLayout = new QHBoxLayout();
	QVBoxLayout* targetCfgButtonLayout = new QVBoxLayout();
	QGridLayout* targetCfgGridLayout = new QGridLayout();

	setLayout(mainLayout);
	mainLayout->addWidget(fkGroupBox);
	mainLayout->addWidget(targetPoseGroupBox);
	mainLayout->addWidget(targetCfgGroupBox);
	fkGroupBox->setLayout(fkBoxLayout);
	fkBoxLayout->addLayout(fkLayout);
	targetPoseGroupBox->setLayout(targetPoseControlLayout);
	targetPoseControlLayout->addLayout(targetPoseLayout);
	targetCfgGroupBox->setLayout(targetCfgLayout);
	targetCfgLayout->addLayout(targetCfgGridLayout);
	targetCfgLayout->addLayout(targetCfgButtonLayout);

	//! Forward Kinematics
	QLabel* fkNames[10];
	fkNames[0] = new QLabel("Pos X");
	fkNames[1] = new QLabel("Pos Y");
	fkNames[2] = new QLabel("Pos Z");
	fkNames[3] = new QLabel("Roll");
	fkNames[4] = new QLabel("Pitch");
	fkNames[5] = new QLabel("Yaw");
	fkNames[6] = new QLabel("Quat x");
	fkNames[7] = new QLabel("Quat y");
	fkNames[8] = new QLabel("Quat z");
	fkNames[9] = new QLabel("Quat w");
	for (int i=0;i<10;i++)
		fkPose[i] = new QLabel("0");
	for (int ii=0;ii<20;ii++)
	{
		if (ii<3)
			fkLayout->addWidget(fkNames[ii],0,ii);
		else if (ii<6)
			fkLayout->addWidget(fkPose[ii-3],1,ii-3);
		else if (ii<9)
			fkLayout->addWidget(fkNames[ii-3],2,ii-6);
		else if (ii<12)
			fkLayout->addWidget(fkPose[ii-6],3,ii-9);
		else if (ii<16)
			fkLayout->addWidget(fkNames[ii-6],4,ii-12);
		else
			fkLayout->addWidget(fkPose[ii-10],5,ii-16);
	}

	QPushButton* getForwardKinematicsButton = new QPushButton("&Get FK");
	fkBoxLayout->addWidget(getForwardKinematicsButton);

	//! Workspace Control
	QLabel* trans_x_label = new QLabel("x");
	QLabel* trans_y_label = new QLabel("y");
	QLabel* trans_z_label = new QLabel("z");
	QLabel* rpy_r_label = new QLabel("Roll");
	QLabel* rpy_p_label = new QLabel("Pitch");
	QLabel* rpy_y_label = new QLabel("Yaw");

	for (int i=0;i<3;i++){
		trans_input[i] = new QDoubleSpinBox();
		rpy_input[i] = new QDoubleSpinBox();
		trans_input[i]->setValue(0.5);
		trans_input[i]->setDecimals(3);
		trans_input[i]->setRange(-2.0,2.0);
		rpy_input[i]->setDecimals(3);
		rpy_input[i]->setRange(-3.14,3.14);
	}

	targetPoseLayout->addWidget(trans_x_label,0,0);
	targetPoseLayout->addWidget(trans_y_label,0,1);
	targetPoseLayout->addWidget(trans_z_label,0,2);

	for (int i=0;i<3;i++)
		targetPoseLayout->addWidget(trans_input[i],1,i);

	targetPoseLayout->addWidget(rpy_r_label,2,0);
	targetPoseLayout->addWidget(rpy_p_label,2,1);
	targetPoseLayout->addWidget(rpy_y_label,2,2);

	for (int i=0;i<3;i++)
		targetPoseLayout->addWidget(rpy_input[i],3,i);

	QPushButton* moveToTargetPoseButton = new QPushButton("&Move to Target Pose");
	targetPoseControlLayout->addWidget(moveToTargetPoseButton);

	//! Joint Control

	for (int i=0;i<12;i++){
		joint_names_[i] = new QLabel("none");
		cfg_input_[i] = new QDoubleSpinBox();
		cfg_input_[i]->setDecimals(3);
		cfg_input_[i]->setRange(-10.0,10.0);
		if (i%2){
			targetCfgGridLayout->addWidget(joint_names_[i-1],(int)i/2,0);
			targetCfgGridLayout->addWidget(cfg_input_[i-1],(int)i/2,1);
			targetCfgGridLayout->addWidget(joint_names_[i],(int)i/2,2);
			targetCfgGridLayout->addWidget(cfg_input_[i],(int)i/2,3);
		}
	}

	QPushButton* updateCfgButton = new QPushButton("&Update Configuration");
	QPushButton* commandCfgButton = new QPushButton("Move to &Configuration");
	targetCfgButtonLayout->addWidget(updateCfgButton);
	targetCfgButtonLayout->addWidget(commandCfgButton);
	//! Connect with ROSinterface
	ROSinterface* rosinterface = ROSinterface::getInstance();

	connect(getForwardKinematicsButton,SIGNAL(clicked()),rosinterface,SLOT(callGetFK()));
	connect(rosinterface,SIGNAL(emitFK(geometry_msgs::Pose)),this,SLOT(updateFK(geometry_msgs::Pose)));

	connect(moveToTargetPoseButton,SIGNAL(clicked()),this,SLOT(moveToTargetPoseCallback()));
	connect(this,SIGNAL(moveToTargetPose(geometry_msgs::Pose)),rosinterface,SLOT(callMoveToTargetPose(geometry_msgs::Pose)));

	connect(updateCfgButton,SIGNAL(clicked()),rosinterface,SLOT(sendCurrentCfgOnce()));
	connect(rosinterface,SIGNAL(emitCurrentCfgOnce(std::vector<std::string>,double*)),this,SLOT(updateCfg(std::vector<std::string>,double*)));

	connect(commandCfgButton,SIGNAL(clicked()),this,SLOT(moveToTargetCfgCallback()));
	connect(this,SIGNAL(moveToTargetCfg(double*)),rosinterface,SLOT(callSetCustomGoalConfiguration(double*)));
}

void SimpleControlWidget::moveToTargetPoseCallback()
{
	//! Read inputs
	pose.position.x = trans_input[0]->value();
	pose.position.y = trans_input[1]->value();
	pose.position.z = trans_input[2]->value();
	q_input_.setRPY(rpy_input[0]->value(),rpy_input[1]->value(),rpy_input[2]->value());
	pose.orientation.w = q_input_.w();
	pose.orientation.x = q_input_.x();
	pose.orientation.y = q_input_.y();
	pose.orientation.z = q_input_.z();

	emit moveToTargetPose(pose);
}

void SimpleControlWidget::moveToTargetCfgCallback()
{
	double command_cfg[12];
	for (int i=0;i<12;i++)
		command_cfg[i] = cfg_input_[i]->value();
	emit moveToTargetCfg(command_cfg);
}

void SimpleControlWidget::updateCfg(std::vector<std::string> joint_names,double* current_cfg)
{
	for (int i=0;i<12;i++){
		joint_names_[i]->setText(QString::fromStdString(joint_names[i]));
		cfg_input_[i]->setValue(current_cfg[i]);
	}
}

void SimpleControlWidget::updateFK(geometry_msgs::Pose pose)
{
	q_current_.setX(pose.orientation.x);
	q_current_.setY(pose.orientation.y);
	q_current_.setZ(pose.orientation.z);
	q_current_.setW(pose.orientation.w);

	dcm_.setRotation(q_current_);
	double roll,pitch,yaw;

	dcm_.getRPY(roll,pitch,yaw);

	fkPose[0]->setText(QString::number(pose.position.x,'f',6));
	fkPose[1]->setText(QString::number(pose.position.y,'f',6));
	fkPose[2]->setText(QString::number(pose.position.z,'f',6));

	fkPose[3]->setText(QString::number(roll,'f',3));
	fkPose[4]->setText(QString::number(pitch,'f',3));
	fkPose[5]->setText(QString::number(yaw,'f',3));

	fkPose[6]->setText(QString::number(q_current_.x(),'f',6));
	fkPose[7]->setText(QString::number(q_current_.y(),'f',6));
	fkPose[8]->setText(QString::number(q_current_.z(),'f',6));
	fkPose[9]->setText(QString::number(q_current_.w(),'f',6));


}
