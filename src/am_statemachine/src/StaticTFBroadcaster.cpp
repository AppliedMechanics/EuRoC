/*
 * StaticTFBroadcaster.cpp
 *
 *  Created on: Aug 6, 2014
 *      Author: euroc_admin
 */

#include <StaticTFBroadcaster.h>
#include <euroc_input.hpp>


StaticTFBroadcaster::StaticTFBroadcaster() {

}

StaticTFBroadcaster::~StaticTFBroadcaster() {
	// TODO Auto-generated destructor stub
}

void StaticTFBroadcaster::fill_tf_information(EurocInput*data)
{
	T_LA_Base.setOrigin(tf::Vector3(data->robot_.pose.position.x,data->robot_.pose.position.y,data->robot_.pose.position.z));
	T_LA_Base.setRotation(tf::Quaternion(data->robot_.pose.orientation.x,data->robot_.pose.orientation.y,data->robot_.pose.orientation.z,data->robot_.pose.orientation.w));

	T_GP.setOrigin(tf::Vector3(data->robot_.gripper_pose.position.x,data->robot_.gripper_pose.position.y,data->robot_.gripper_pose.position.z));
	T_GP.setRotation(tf::Quaternion(data->robot_.gripper_pose.orientation.x,data->robot_.gripper_pose.orientation.y,data->robot_.gripper_pose.orientation.z,data->robot_.gripper_pose.orientation.w));

	T_GP_TCP.setOrigin(tf::Vector3(data->robot_.gripper_tcp.position.x,data->robot_.gripper_tcp.position.y,data->robot_.gripper_tcp.position.z));
	T_GP_TCP.setRotation(tf::Quaternion(data->robot_.gripper_tcp.orientation.x,data->robot_.gripper_tcp.orientation.y,data->robot_.gripper_tcp.orientation.z,data->robot_.gripper_tcp.orientation.w));

	T_CM.setOrigin(tf::Vector3(data->base_pose_.position.x,data->base_pose_.position.y,data->base_pose_.position.z));

	T_CM.setRotation(tf::Quaternion(data->base_pose_.orientation.x,data->base_pose_.orientation.y,data->base_pose_.orientation.z,data->base_pose_.orientation.w));
	std::cout<<"Orientation: x = "<<data->base_pose_.orientation.x<<" y= "<<data->base_pose_.orientation.y<<" z= "<<data->base_pose_.orientation.z<<" w = "<<data->base_pose_.orientation.w<<std::endl;
	//	tf::Quaternion q_CM;
//	q_CM.setRPY(0,0,-2.356);
//	T_CM.setRotation(q_CM);

	T_PT_Base.setOrigin(tf::Vector3(data->pan_tilt_base_.position.x,data->pan_tilt_base_.position.y,data->pan_tilt_base_.position.z));
	T_PT_Base.setRotation(tf::Quaternion(data->pan_tilt_base_.orientation.x,data->pan_tilt_base_.orientation.y,data->pan_tilt_base_.orientation.z,data->pan_tilt_base_.orientation.w));

	for (int ii=0;ii<data->sensors_.size();ii++)
	{
		if (!data->sensors_[ii].name.compare("tcp_rgb_cam"))
		{
			T_TRGB.setOrigin(tf::Vector3(data->sensors_[ii].pose.position.x,data->sensors_[ii].pose.position.y,data->sensors_[ii].pose.position.z));
			T_TRGB.setRotation(tf::Quaternion(data->sensors_[ii].pose.orientation.x,data->sensors_[ii].pose.orientation.y,data->sensors_[ii].pose.orientation.z,data->sensors_[ii].pose.orientation.w));
		}
		else if (!data->sensors_[ii].name.compare("tcp_depth_cam"))
		{
			T_TDEPTH.setOrigin(tf::Vector3(data->sensors_[ii].pose.position.x,data->sensors_[ii].pose.position.y,data->sensors_[ii].pose.position.z));
			T_TDEPTH.setRotation(tf::Quaternion(data->sensors_[ii].pose.orientation.x,data->sensors_[ii].pose.orientation.y,data->sensors_[ii].pose.orientation.z,data->sensors_[ii].pose.orientation.w));
		}
		else if (!data->sensors_[ii].name.compare("scene_rgb_cam"))
		{
			T_SRGB.setOrigin(tf::Vector3(data->sensors_[ii].pose.position.x,data->sensors_[ii].pose.position.y,data->sensors_[ii].pose.position.z));
			T_SRGB.setRotation(tf::Quaternion(data->sensors_[ii].pose.orientation.x,data->sensors_[ii].pose.orientation.y,data->sensors_[ii].pose.orientation.z,data->sensors_[ii].pose.orientation.w));
		}
		else if (!data->sensors_[ii].name.compare("scene_depth_cam"))
		{
			T_SDEPTH.setOrigin(tf::Vector3(data->sensors_[ii].pose.position.x,data->sensors_[ii].pose.position.y,data->sensors_[ii].pose.position.z));
			T_SDEPTH.setRotation(tf::Quaternion(data->sensors_[ii].pose.orientation.x,data->sensors_[ii].pose.orientation.y,data->sensors_[ii].pose.orientation.z,data->sensors_[ii].pose.orientation.w));
		}
		else
		{
			msg_error("Not registered sensor.");
			std::cout<<"Sensor name: "<<data->sensors_[ii].name<<std::endl;
		}
	}

}

void StaticTFBroadcaster::publish_static_tf()
{
	//! TODO: Attention!!!
	//  StampedTransform(..,..,PARENT,CHILD) !!!
	//  Bug in Euroc description?
	//
	br_.sendTransform(tf::StampedTransform(T_LA_Base,ros::Time::now(),ORIGIN,LA_0));
	br_.sendTransform(tf::StampedTransform(T_GP,ros::Time::now(),LWR_TCP,GP_0));
	br_.sendTransform(tf::StampedTransform(T_GP_TCP,ros::Time::now(),GP_0,GP_TCP));
	br_.sendTransform(tf::StampedTransform(T_TRGB,ros::Time::now(),GP_0,T_RGB));
	br_.sendTransform(tf::StampedTransform(T_TDEPTH,ros::Time::now(),T_RGB,T_DEPTH));
	br_.sendTransform(tf::StampedTransform(T_CM,ros::Time::now(),ORIGIN,CM));
	br_.sendTransform(tf::StampedTransform(T_PT_Base,ros::Time::now(),CM,PT_0));
	br_.sendTransform(tf::StampedTransform(T_SRGB,ros::Time::now(),PT_TCP,S_RGB));
	br_.sendTransform(tf::StampedTransform(T_SDEPTH,ros::Time::now(),S_RGB,S_DEPTH));
}



