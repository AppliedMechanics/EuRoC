/*
 * Copyright (C) Institute of Applied Mechanics, TU Muenchen
 * All rights reserved.
 * Contact: schuetz@amm.mw.tum.de
 */

#include "simplecontrolwidget.hpp"
#include "ROSinterface.hpp"


SimpleControlWidget::SimpleControlWidget(QWidget *parent):
  QWidget(parent)
{

  // layouts


  QHBoxLayout* mainLayout = new QHBoxLayout();
  QHBoxLayout* targetPoseControlLayout = new QHBoxLayout();
  QGridLayout* targetPoseLayout = new QGridLayout();
  setLayout(mainLayout);
  mainLayout->addLayout(targetPoseControlLayout);
  targetPoseControlLayout->addLayout(targetPoseLayout);


  QVBoxLayout* startTargetMotionLayout = new QVBoxLayout();


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


  //! Connect with ROSinterface
  ROSinterface* rosinterface = ROSinterface::getInstance();

  connect(moveToTargetPoseButton,SIGNAL(clicked()),this,SLOT(moveToTargetPoseCallback()));
  connect(this,SIGNAL(moveToTargetPose(double*)),rosinterface,SLOT(callMoveToTargetPose(double*)));
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



