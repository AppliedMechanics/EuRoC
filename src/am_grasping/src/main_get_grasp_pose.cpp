/*
 * main_get_grasp_pose.cpp
 *
 *  Created on: Aug 13, 2014
 *      Author: Nikolas Tekles
 */

/*
 * GetGraspPose_server.cpp
 *
 *  Created on: Jul 25, 2014
 *      Author: Nikolas Tekles
 */

#include "ros/ros.h"
#include <iostream>
#include <utils.hpp>
#include <std_msgs/Bool.h>

#include "GraspPose2.hpp"

//!external kill functionality from a message
bool kill_flag=false;
void kill_cb(const std_msgs::BoolConstPtr& rst)
{
	kill_flag=true;
	ROS_INFO("stop received!");
}

// Main Function -- Server-Node
int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_grasp_pose_srv_node");
	ros::NodeHandle nodeHandle;
	ros::Subscriber rs_=nodeHandle.subscribe("kill",1000,kill_cb);

	GraspPose2 grasp_pose;
	ros::ServiceServer service = grasp_pose.n.advertiseService("GraspPose_srv", &GraspPose2::return_grasp_pose, &grasp_pose);
	ROS_INFO("Ready to calculate Grasp-Pose.");

	while (ros::ok() && !kill_flag)
	{
		ros::Duration(0.5).sleep();
		ros::spinOnce();
	}
	msg_warn("exiting Grasping node!");

	//ros::spin();

	return 0;
}


