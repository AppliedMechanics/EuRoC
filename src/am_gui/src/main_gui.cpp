/*
 * Copyright (C) Institute of Applied Mechanics, TU Muenchen
 * All rights reserved.
 * Contact: schuetz@amm.mw.tum.de
 */

#include <ros/ros.h>
#include <QtGui/QApplication>
#include "mainwindow.h"
#include <QIcon>

int main(int argc, char** argv) {

  QApplication a(argc, argv);

  ros::init(argc, argv, "am_gui");
  //ros::NodeHandle n;
  

  MainWindow w;
  w.RosInit();
  w.show();
  return a.exec();
}
