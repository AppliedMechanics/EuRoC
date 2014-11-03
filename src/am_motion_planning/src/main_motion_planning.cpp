/*
 * main_motion_planning.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: euroc_admin
 */

#include <ros/ros.h>
#include "T6MotionPlanning.h"
//#include "MotionPlanning.h"
#include <config.hpp>


//!external kill functionality from a message
bool kill_flag=false;
void kill_cb(const std_msgs::BoolConstPtr& rst)
{
	kill_flag=true;
	ROS_INFO("stop received!");
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "am_motion_planning");
	ros::NodeHandle nodeHandle;
	ros::Subscriber rs_=nodeHandle.subscribe("kill",1000,kill_cb);

//	MotionPlanning mp;
	T6MotionPlanning mp;

	ros::Rate r(FREQ*10); // 10 hz
	while (ros::ok() && !kill_flag)
	{
		ros::spinOnce();
		r.sleep();
	}
	msg_warn("exiting MotionPlanning node!");

	return 0;

}


