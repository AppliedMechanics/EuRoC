/*
 * Copyright (C) Institute of Applied Mechanics, TU Muenchen
 * All rights reserved.
 * Contact: schuetz@amm.mw.tum.de
 */

#ifndef MANIPULATORSTATUSWIDGET_HPP
#define MANIPULATORSTATUSWIDGET_HPP

#include <QtGui>
#include <QString>
#include <QWidget>
#include "ROSinterface.hpp"

class ManipulatorStatusWidget : public QWidget
{
  Q_OBJECT

  private:

  ROSinterface  *rosinterface;
  QComboBox* taskSelection;

public:
  
  explicit ManipulatorStatusWidget(QWidget *parent = 0);

signals:
void callStartSimulator(std::string);

public slots:

void updateTasks();

private slots:
void startSimulatorWithTask();

};

#endif // MANIPULATORSTATUSWIDGET_H

