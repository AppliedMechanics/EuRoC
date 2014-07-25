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
#include <QCheckBox>
#include <QSlider>
#include <QSignalMapper>
#include "ROSinterface.hpp"

class ServoControlWidget: public QWidget {

	Q_OBJECT
public:
	explicit ServoControlWidget(QWidget *parent = 0);
	virtual ~ServoControlWidget();

private:
	QCheckBox* enable_servo_mode_;
	QSlider*   joint_servo_offsets_[12];
	QLabel*    joint_names_[12];
	QSignalMapper *signalMapperPressed_;

	bool signal_active_;

	ROSinterface* rosinterface;

	void enable_servo_mode();

	private slots:

	void sendZero();
	void sendServoCommand(int);
	void activateSignal(const int);
	void enableServoModeChanged(int);

	signals:
	void emitEnableServoMode(bool);
	void sliderPressed(const int);

};

#endif /* SERVOCONTROLWIDGET_H_ */
