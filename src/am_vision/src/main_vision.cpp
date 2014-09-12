/*
 * main_vision.cpp
 *
 *  Created on: Aug 11, 2014
 *      Author: sahandy
 */
#define DUMMY

#include <ros/ros.h>

#ifndef DUMMY
#include "vision.hpp"
#else
#include "vision_dummy.hpp"
#endif

#include "MovePantilt.h"
#include <config.hpp>

int main(int argc, char** argv) {

	ros::init(argc, argv, "am_vision");
	ros::NodeHandle nodeHandle;

#ifndef DUMMY
	Vision vision;
#else
	VisionDummy vision;
#endif


	// Example, how MovePantilt class can be used
	MovePantilt* mpt;
	mpt = mpt->get_instance();

	//mpt->move_pan_tilt_rel(0.5, 0);
	// end of Example

	ros::Rate r(10); // 10 hz
	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();

	}

	return 0;
}

