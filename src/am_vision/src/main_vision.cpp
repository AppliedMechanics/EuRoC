/*
 * main_motion_planning.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: euroc_admin
 */

#include <ros/ros.h>

int main(int argc, char** argv) {

	ros::init(argc, argv, "am_vision");

	ros::NodeHandle nh;


	ros::Rate r(10); // 10 hz
	while (ros::ok())
	{


		ros::spinOnce();
		r.sleep();
	}

	return 0;

}


