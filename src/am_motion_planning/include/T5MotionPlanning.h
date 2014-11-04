/*
 * T6MotionPlanning.h
 *
 *  Created on: Nov 3, 2014
 *      Author: euroc_admin
 */

#ifndef T5MOTIONPLANNING_H_
#define T5MOTIONPLANNING_H_

#include <MotionPlanning.h>

class T5MotionPlanning: public MotionPlanning {
public:
	T5MotionPlanning();
	virtual ~T5MotionPlanning();

private:
	void executeGoalPoseT5();
};

#endif /* T5MOTIONPLANNING_H_ */
