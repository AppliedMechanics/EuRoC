/*
 * ServoControlWidget.h
 *
 *  Created on: Jul 23, 2014
 *      Author: euroc_admin
 */

#ifndef SERVOCONTROLWIDGET_H_
#define SERVOCONTROLWIDGET_H_

#include <qwidget.h>
#include <QGridLayout>
#include <QtGui>
#include <QDoubleSpinBox>

class ServoControlWidget: public QWidget {

	Q_OBJECT
public:
	explicit ServoControlWidget(QWidget *parent = 0);
	virtual ~ServoControlWidget();
};

#endif /* SERVOCONTROLWIDGET_H_ */
