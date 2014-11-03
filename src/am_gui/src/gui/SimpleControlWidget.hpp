/*
 * Copyright (C) Institute of Applied Mechanics, TU Muenchen
 * All rights reserved.
 * Contact: schuetz@amm.mw.tum.de
 */

#ifndef SIMPLECONTROLWIDGET
#define SIMPLECONTROLWIDGET

#include <QWidget>
#include <QGridLayout>
#include <QtGui>
#include <QDoubleSpinBox>
#include <tf_rot.hpp>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Pose.h>
#include "ROSinterface.hpp"
#include <QSlider>

class SimpleControlWidget : public QWidget
{
	Q_OBJECT

private:

	double trans_[3];
	tf::Quaternion q_input_;
	tf::Quaternion q_current_;
	tf::Matrix3x3 dcm_;
	geometry_msgs::Pose pose;
	double rpy_[3];

	//double pose[7]; //Format: x,y,z,w,x,y,z
	double cfg_[12]; //Format: x,y,q1,q2,....,q7,gripper,cam pan,cam tilt

	QDoubleSpinBox* trans_input[3];
	QDoubleSpinBox* rpy_input[3];

	QDoubleSpinBox* cfg_input_[12];
	QLabel* joint_names_[12];

	QLabel* fkPose[10];
	QSlider*   speed_percentage_slider_;
	QLabel*    speed_percentage_label_;

public:

	explicit SimpleControlWidget(QWidget *parent = 0);

	signals:

	void moveToTargetPose(geometry_msgs::Pose);
	void moveToTargetCfg(double*);


public slots:

void updateCfg(std::vector<std::string>,double*);
void updateFK(geometry_msgs::Pose);

private slots:

void moveToTargetPoseCallback();
void moveToTargetCfgCallback();

};

#endif //SIMPLECONTROLWIDGET
