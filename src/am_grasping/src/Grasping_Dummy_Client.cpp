/*
 * Grasping_Dummy_Client.cpp
 *
 *  Created on: Jul 25, 2014
 *      Author: Nikolas Tekles
 */

#include "ros/ros.h"
#include <am_msgs/GetGraspPose.h>
#include <cstdlib>
#include <cmath>


int main(int argc, char **argv)
{
	if (argc == 4)
	{
		ROS_INFO("Input: string object_type, float64[] object_dimensions, geometry_msgs/Pose object_pose");
		ROS_INFO("Output: string error_message, geometry_msgs/Pose object_pose");
		ROS_INFO(" -- NOT IMPLEMENTED YET -- ");
		return 1;
	}

  ros::init(argc, argv, "get_grasp_pose_client_node");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<am_msgs::GetGraspPose>("GraspPose_srv");
  am_msgs::GetGraspPose srv;

  //srv.request.object_type = "handle";
//  srv.request.object_type = "green_cylinder";
//
//  srv.request.object_dimensions.resize(3);
//  srv.request.object_dimensions[0] = 1.0;
//  srv.request.object_dimensions[1] = 2.0;
//  srv.request.object_dimensions[2] = 3.0;

  geometry_msgs::Pose &return_pose = srv.request.object.abs_pose;
  return_pose.position.x = 5.0;
  return_pose.position.y = 6.0;
  return_pose.position.z = 7.0;
  geometry_msgs::Quaternion towards_table;
  double rot_axis[3] = {0,0,1};
  double norm = sqrt( rot_axis[0]*rot_axis[0] + rot_axis[1]*rot_axis[1] + rot_axis[2]*rot_axis[2] );
  double rot_angle = M_PI / 2.0;
  double s_th2 = sin(rot_angle/2.0);
  double c_th2 = cos(rot_angle/2.0);
  towards_table.x = s_th2 * rot_axis[0] / norm;
  towards_table.y = s_th2 * rot_axis[1] / norm;
  towards_table.z = s_th2 * rot_axis[2] / norm;
  towards_table.w = c_th2 / norm;
  return_pose.orientation = towards_table;

  if (client.call(srv))
  {
	  geometry_msgs::Pose return_arg = srv.response.grasp_pose;
	  ROS_INFO("Position returned to client: x=%.1f, y=%.1f, z=%.1f", return_arg.position.x, return_arg.position.y, return_arg.position.z);
	  ROS_INFO("Orientation returned to client: x=%.1f, y=%.1f, z=%.1f, w=%.1f", return_arg.orientation.x, return_arg.orientation.y,\
			  return_arg.orientation.z,return_arg.orientation.z);
  }
  else
  {
	  ROS_ERROR("Failed to call service GetGraspPose");
	  return 1;
  }

  return 0;
}
