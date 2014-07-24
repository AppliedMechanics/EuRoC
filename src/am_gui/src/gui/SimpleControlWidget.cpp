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

  QGroupBox* targetPoseGroupBox = new QGroupBox("Workspace Control");
  QHBoxLayout* targetPoseControlLayout = new QHBoxLayout();
  QGridLayout* targetPoseLayout = new QGridLayout();


  QGroupBox* targetCfgGroupBox = new QGroupBox("Joint Control");
  QHBoxLayout* targetCfgLayout = new QHBoxLayout();
  QVBoxLayout* targetCfgButtonLayout = new QVBoxLayout();
  QGridLayout* targetCfgGridLayout = new QGridLayout();

  setLayout(mainLayout);
  mainLayout->addWidget(targetPoseGroupBox);
  mainLayout->addWidget(targetCfgGroupBox);
  targetPoseGroupBox->setLayout(targetPoseControlLayout);
  targetPoseControlLayout->addLayout(targetPoseLayout);
  targetCfgGroupBox->setLayout(targetCfgLayout);
  targetCfgLayout->addLayout(targetCfgGridLayout);
  targetCfgLayout->addLayout(targetCfgButtonLayout);
  //! Workspace Control
  QLabel* trans_x_label = new QLabel("x");
  QLabel* trans_y_label = new QLabel("y");
  QLabel* trans_z_label = new QLabel("z");
  QLabel* kardan_a_label = new QLabel("alpha");
  QLabel* kardan_b_label = new QLabel("beta");
  QLabel* kardan_g_label = new QLabel("gamma");

  for (int i=0;i<3;i++){
    trans_input[i] = new QDoubleSpinBox();
    kardan_input[i] = new QDoubleSpinBox();
    trans_input[i]->setValue(0.5);
    trans_input[i]->setDecimals(3);
    trans_input[i]->setRange(-2.0,2.0);
    kardan_input[i]->setDecimals(3);
    kardan_input[i]->setRange(-3.14,3.14);
  }

  kardan_input[0]->setValue(3.14);

  targetPoseLayout->addWidget(trans_x_label,0,0);
  targetPoseLayout->addWidget(trans_y_label,0,1);
  targetPoseLayout->addWidget(trans_z_label,0,2);

  for (int i=0;i<3;i++)
    targetPoseLayout->addWidget(trans_input[i],1,i);

  targetPoseLayout->addWidget(kardan_a_label,2,0);
  targetPoseLayout->addWidget(kardan_b_label,2,1);
  targetPoseLayout->addWidget(kardan_g_label,2,2);

  for (int i=0;i<3;i++)
    targetPoseLayout->addWidget(kardan_input[i],3,i);

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

  connect(moveToTargetPoseButton,SIGNAL(clicked()),this,SLOT(moveToTargetPoseCallback()));
  connect(this,SIGNAL(moveToTargetPose(double*)),rosinterface,SLOT(callMoveToTargetPose(double*)));

  connect(updateCfgButton,SIGNAL(clicked()),rosinterface,SLOT(sendCurrentCfgOnce()));
  connect(rosinterface,SIGNAL(emitCurrentCfgOnce(std::vector<std::string>,double*)),this,SLOT(updateCfg(std::vector<std::string>,double*)));

  connect(commandCfgButton,SIGNAL(clicked()),this,SLOT(moveToTargetCfgCallback()));
  connect(this,SIGNAL(moveToTargetCfg(double*)),rosinterface,SLOT(callSetCustomGoalConfiguration(double*)));
}

void SimpleControlWidget::moveToTargetPoseCallback()
{
	//! Read inputs
	for (int i=0;i<3;i++)
	{
		pose[i] = trans_input[i]->value();
		kardan[i] = kardan_input[i]->value();
	}

	kardan2quat(kardan,quat,dcm);

	for (int i=0;i<4;i++)
		pose[i+3] = quat[i];

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

