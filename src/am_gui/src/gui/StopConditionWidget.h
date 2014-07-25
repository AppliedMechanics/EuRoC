/*
 * StopConditionWidget.h
 *
 *  Created on: Jul 23, 2014
 *      Author: euroc_admin
 */

#ifndef STOPCONDITIONWIDGET_H_
#define STOPCONDITIONWIDGET_H_

#include <QWidget>
#include <QtGui>
#include <QGridLayout>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include "ROSinterface.hpp"

class StopConditionWidget: public QWidget
{
	Q_OBJECT

public:
	explicit StopConditionWidget(QWidget *parent = 0);
	virtual ~StopConditionWidget();

	public slots:
	void updateJointNames();

private:

	QCheckBox* active_[10];
	QLabel* joint_name_[10];
	QComboBox* operator_[10];
	QDoubleSpinBox* value_[10];
	ROSinterface* rosinterface;

	std::vector<std::string> vec_joint_names_;
	std::vector<std::string> vec_operator_;
	std::vector<double> vec_values_;

	int num_active_;

	private slots:
	void callSetStopCondition();

	signals:

	void emitSetStopCondition(std::vector<std::string>, std::vector<std::string>,std::vector<double>);

};

#endif /* STOPCONDITIONWIDGET_H_ */
