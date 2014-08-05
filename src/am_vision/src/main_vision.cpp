/*
 * main_motion_planning.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: euroc_admin
 */

#include <ros/ros.h>
#include <vision.hpp>


int main(int argc, char** argv) {

	ros::init(argc, argv, "am_vision");

	Vision vision;

	ros::Rate r(10); // 10 hz
	while (ros::ok())
	{
	// Subscribe to ROS topics
		ros::spinOnce();
		r.sleep();
	}

	return 0;

}
