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
#include "ROSinterface.hpp"

class SimpleControlWidget : public QWidget
{
  Q_OBJECT

  private:

  double trans[3];
  double dcm[9];
  double quat[4];
  double kardan[3];

  double pose[7]; //Format: x,y,z,w,x,y,z
  double cfg_[12]; //Format: x,y,q1,q2,....,q7,gripper,cam pan,cam tilt

  QDoubleSpinBox* trans_input[3];
  QDoubleSpinBox* kardan_input[3];

  QDoubleSpinBox* cfg_input_[12];
  QLabel* joint_names_[12];

  public:

  explicit SimpleControlWidget(QWidget *parent = 0);

  signals:

  void moveToTargetPose(double*);
  void moveToTargetCfg(double*);


   public slots:

   void updateCfg(std::vector<std::string>,double*);

    private slots:

    void moveToTargetPoseCallback();
    void moveToTargetCfgCallback();

};

#endif //SIMPLECONTROLWIDGET
