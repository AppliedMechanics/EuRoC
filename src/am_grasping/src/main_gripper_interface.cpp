/*
 * main_gripper_interface.cpp
 *
 *  Created on: Jul 30, 2014
 *      Author: Christoph Schuetz
 */

#include <ros/ros.h>
#include <euroc_c2_msgs/MoveAlongJointPath.h>
#include <am_msgs/GripperControl.h>
#include <config.hpp>
#include <utils.hpp>
#include <std_msgs/Bool.h>

//! Services
std::string euroc_c2_interface_ = "/euroc_interface_node";
std::string move_along_joint_path_ = euroc_c2_interface_ + "/move_along_joint_path";
euroc_c2_msgs::MoveAlongJointPath move_along_joint_path_srv_;
euroc_c2_msgs::Configuration commanded_configuration_;


//!external kill functionality from a message
bool kill_flag=false;
void kill_cb(const std_msgs::BoolConstPtr& rst)
{
	kill_flag=true;
	ROS_INFO("stop received!");
}

bool gripper_interface(am_msgs::GripperControl::Request &req, am_msgs::GripperControl::Response &res)
{
	ros::NodeHandle n;
	ros::ServiceClient move_along_joint_path_client_ = n.serviceClient<euroc_c2_msgs::MoveAlongJointPath>(move_along_joint_path_);


	// Read Request variables: ...
	//			... object type, ....
	gripping_mode_t gripping_mode = (gripping_mode_t)req.gripping_mode;
	double object_width = req.object_width;
	double gripping_force = req.gripping_force;
	double gripper_position = req.gripper_position;

	std::string gripper_name = "gripper";
	move_along_joint_path_srv_.request.joint_names.resize(1);
	move_along_joint_path_srv_.request.joint_names[0] = gripper_name;
	move_along_joint_path_srv_.request.path.resize(1);
	move_along_joint_path_srv_.request.joint_limits.resize(1);
	move_along_joint_path_srv_.request.joint_limits[0].max_velocity = 0.1; // max 0.5
	move_along_joint_path_srv_.request.joint_limits[0].max_acceleration = 150;

	commanded_configuration_.q.resize(1);


	switch (gripping_mode)
	{
	case POSITION:
		commanded_configuration_.q[0] = gripper_position;
		break;
	case FF_FORCE:
		//! Assumption: "stiffness" of position control = 1500 . FORCE = 1500 (object_width - POSITION). POSITION=object_width - FORCE/1500
		commanded_configuration_.q[0] = object_width - (gripping_force/1500.0);
		if (commanded_configuration_.q[0]<0.0)
			commanded_configuration_.q[0]=0.0;
		else if (commanded_configuration_.q[0]>0.07)
			commanded_configuration_.q[0]=0.07;
		break;
	case FB_FORCE:
		ROS_ERROR("FB FORCE not yet implemented");
		break;

	case RELEASE:
		commanded_configuration_.q[0] = 0.07;
		break;
	default:
		ROS_ERROR("Gripping Mode not matched.");
		break;
	}

	move_along_joint_path_srv_.request.path[0] = commanded_configuration_;
	if(!move_along_joint_path_client_.call(move_along_joint_path_srv_))
	{
		res.error_reason = fsm::GRIPPING_ERROR;
		return false;
	}

	//
	//	res.grasp_pose = TCP_target_pose;

	// Print out stop reason
	std::string joint = "joint ";
	std::string tool = "tool force";
	std::string ext_torque = "ext_torque ";
	std::string is = "is ";
	std::string StopReason = move_along_joint_path_srv_.response.stop_reason;

	if( (StopReason != "path finished") && (StopReason != "path finished (already at target)") )
	{
		if (StopReason == "")
		{
			msg_error("%s", move_along_joint_path_srv_.response.error_message.c_str());
			res.error_reason = fsm::GRIPPING_ERROR;
			return false;
		}
		else
		{
			msg_info("Stop Reason: %s", StopReason.c_str());

			res.error_reason = fsm::STOP_COND;
			return false;
		}
	}
	else
	{
		return true;
	}
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "am_gripper_interface");
	ros::NodeHandle nh_;

	ros::ServiceServer service = nh_.advertiseService("GripperInterface", gripper_interface);

	ros::Subscriber rs_=nh_.subscribe("kill",1000,kill_cb);



	ros::Rate r(FREQ); // 10 hz
	while (ros::ok() && !kill_flag)
	{
		ros::spinOnce();
		r.sleep();
	}
	msg_warn("exiting GrippingInterface node!");

	return 0;

}
