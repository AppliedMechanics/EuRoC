/*
 * T6MotionPlanning.h
 *
 *  Created on: Nov 3, 2014
 *      Author: euroc_admin
 */

#ifndef T6MOTIONPLANNING_H_
#define T6MOTIONPLANNING_H_

#include <T5MotionPlanning.h>

class T6MotionPlanning: public T5MotionPlanning {
public:
	T6MotionPlanning();
	virtual ~T6MotionPlanning();

private:
	void executeGoalPoseT6();
};

#endif /* T6MOTIONPLANNING_H_ */
