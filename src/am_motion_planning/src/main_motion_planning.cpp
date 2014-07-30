/*
 * main_motion_planning.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: euroc_admin
 */

#include <ros/ros.h>
#include "MotionPlanning.h"

int main(int argc, char** argv) {

	ros::init(argc, argv, "am_motion_planning");

	MotionPlanning mp;

	ros::Rate r(10); // 10 hz
	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}

	return 0;

}


