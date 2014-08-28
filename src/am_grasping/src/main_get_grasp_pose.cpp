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
	ros::NodeHandle n;

	//GraspPose2 grasp_pose;
	GraspPose2 grasp_pose;
	ros::ServiceServer service = n.advertiseService("GraspPose_srv", &GraspPose2::return_grasp_pose, &grasp_pose);
	ROS_INFO("Ready to calculate Grasp-Pose.");

	static tf::TransformBroadcaster br;


	while (ros::ok()) {
//		br.sendTransform(tf::StampedTransform(grasp_pose.transform_gripper, ros::Time::now(), ORIGIN, "grasp pose"));
//		br.sendTransform(tf::StampedTransform(grasp_pose.transform_object, ros::Time::now(), ORIGIN, OBJ_POSE));
		ros::Duration(0.5).sleep();
		ros::spinOnce();
	}

	//ros::spin();

	return 0;
}


