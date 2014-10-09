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

#include "GraspPose2.hpp"


// Main Function -- Server-Node
int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_grasp_pose_srv_node");

	GraspPose2 grasp_pose;
	ros::ServiceServer service = grasp_pose.n.advertiseService("GraspPose_srv", &GraspPose2::return_grasp_pose, &grasp_pose);
	ROS_INFO("Ready to calculate Grasp-Pose.");

	while (ros::ok())
	{
		ros::Duration(0.5).sleep();
		ros::spinOnce();
	}

	//ros::spin();

	return 0;
}


