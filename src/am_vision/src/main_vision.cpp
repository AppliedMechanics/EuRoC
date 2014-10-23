/*
 * main_vision.cpp
 *
 *  Created on: Aug 11, 2014
 *      Author: sahandy
 */


#include <ros/ros.h>


#include "vision.hpp"
#include "vision_dummy.hpp"


#include "MovePantilt.h"
#include <config.hpp>

//!external kill functionality from a message
bool kill_flag=false;
void kill_cb(const std_msgs::BoolConstPtr& rst)
{
	kill_flag=true;
	ROS_INFO("stop received!");
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "am_vision");
	ros::NodeHandle nodeHandle;
	ros::Subscriber rs_=nodeHandle.subscribe("kill",1000,kill_cb);

	bool skip_vision;
	ros::param::get("/skip_vision",skip_vision);

	Vision* vision;
	if (skip_vision)
		vision = new VisionDummy();
	else
		vision = new Vision();



	// Example, how MovePantilt class can be used
	MovePantilt* mpt;
	mpt = mpt->get_instance();

	//mpt->move_pan_tilt_rel(0.5, 0);
	// end of Example

	ros::Rate r(10); // 10 hz
	while (ros::ok() && !kill_flag)
	{
		ros::spinOnce();
		r.sleep();

	}
	msg_warn("exiting Vision node!");

	return 0;
}

