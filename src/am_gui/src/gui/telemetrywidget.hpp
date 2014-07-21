/*
 * telemetrywidget.hpp
 *
 *  Created on: Jul 18, 2014
 *      Author: euroc_admin
 */

#ifndef TELEMETRYWIDGET_HPP_
#define TELEMETRYWIDGET_HPP_

#include <QWidget>
#include <QGridLayout>
#include <QtGui>
#include <QString>

#include "ROSinterface.hpp"

class TelemetryWidget : public QWidget
{
  Q_OBJECT

  private:

  QLabel* jointLabels[12];
  QLabel* jointPositionMeasure[12];
  QLabel* jointForceMeasure[12];
  QLabel* jointExternalForceMeasure[12];

  public:

  explicit TelemetryWidget(QWidget *parent = 0);

  signals:


   public slots:

   void showMeasure(QString* jointNames,double* position, double* force, double* extforce );

    private slots:

};



#endif /* TELEMETRYWIDGET_HPP_ */
