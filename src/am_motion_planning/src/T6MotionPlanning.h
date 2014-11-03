/*
 * T6MotionPlanning.h
 *
 *  Created on: Nov 3, 2014
 *      Author: euroc_admin
 */

#ifndef T6MOTIONPLANNING_H_
#define T6MOTIONPLANNING_H_

#include <MotionPlanning.h>

class T6MotionPlanning: public MotionPlanning {
public:
	T6MotionPlanning();
	virtual ~T6MotionPlanning();

	void executeGoalPose_CB(const am_msgs::goalPoseGoal::ConstPtr &goal);

private:
	actionlib::SimpleActionServer<am_msgs::goalPoseAction> T6_goalPose_server_;

};

#endif /* T6MOTIONPLANNING_H_ */
