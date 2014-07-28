/*
 * main_motion_planning.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: euroc_admin
 */

#include <ros/ros.h>

int main(int argc, char** argv) {

	ros::init(argc, argv, "am_motion_planning");

	ros::NodeHandle nh;

	ros::spin();

	return 0;

}


