/*
 * Copyright (C) Institute of Applied Mechanics, TU Muenchen
 * All rights reserved.
 * Contact: baur@amm.mw.tum.de
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include "ManipulatorStatusWidget.hpp"
#include "TelemetryWidget.hpp"
#include "SimpleControlWidget.hpp"

#include "ROSinterface.hpp"
#include "ros/ros.h"


#include <boost/thread.hpp>

#define DOFS 9

class MainWindow : public QMainWindow
{
  Q_OBJECT

    public:
  MainWindow(QWidget *parent = 0);
  ~MainWindow();

 private:
  void createMenus();
  void writeSettings();
  void readSettings();
  void closeEvent(QCloseEvent *event);

  ManipulatorStatusWidget *stateMachine;

  TelemetryWidget *telemetry;
  SimpleControlWidget *simplecontrol;

 public:
  //ros
  void RosInit();
  
  
 signals:

 private slots:


  void quit();

  //ros
  void RosSpin();

 public slots:
 
};

#endif // MAINWINDOW_H
