/*
 * StaticTFBroadcaster.cpp
 *
 *  Created on: Aug 6, 2014
 *      Author: euroc_admin
 */

#include <StaticTFBroadcaster.h>
#include <euroc_input.hpp>


StaticTFBroadcaster::StaticTFBroadcaster() {

	T_LA_Base.header.frame_id = ORIGIN;
	T_LA_Base.child_frame_id = LA_0;

	T_GP.header.frame_id = LWR_TCP;
	T_GP.child_frame_id  = GP_0;

	T_GP_TCP.header.frame_id = GP_0;
	T_GP_TCP.child_frame_id  = GP_TCP;

	T_TRGB.header.frame_id = GP_0;
	T_TRGB.child_frame_id  = T_RGB;

	T_TDEPTH.header.frame_id = T_RGB;
	T_TDEPTH.child_frame_id  = T_DEPTH;

	T_CM.header.frame_id = ORIGIN;
	T_CM.child_frame_id  = CM;

	T_PT_Base.header.frame_id = CM;
	T_PT_Base.child_frame_id  = PT_0;

	T_SRGB.header.frame_id = PT_TCP;
	T_SRGB.child_frame_id  = S_RGB;

	T_SDEPTH.header.frame_id = S_RGB;
	T_SDEPTH.child_frame_id  = S_DEPTH;

}

StaticTFBroadcaster::~StaticTFBroadcaster() {
	// TODO Auto-generated destructor stub
}

void StaticTFBroadcaster::fill_tf_information(EurocInput*data)
{
	T_LA_Base.transform.translation.x = data->robot_.pose.position.x;
	T_LA_Base.transform.translation.y = data->robot_.pose.position.y;
	T_LA_Base.transform.translation.z = data->robot_.pose.position.z;
	T_LA_Base.transform.rotation    = data->robot_.pose.orientation;

	T_GP.transform.translation.x = data->robot_.gripper_pose.position.x;
	T_GP.transform.translation.y = data->robot_.gripper_pose.position.y;
	T_GP.transform.translation.z = data->robot_.gripper_pose.position.z;
	T_GP.transform.rotation = data->robot_.gripper_pose.orientation;

	T_GP_TCP.transform.translation.x = data->robot_.gripper_tcp.position.x;
	T_GP_TCP.transform.translation.y = data->robot_.gripper_tcp.position.y;
	T_GP_TCP.transform.translation.z = data->robot_.gripper_tcp.position.z;
	T_GP_TCP.transform.rotation = data->robot_.gripper_tcp.orientation;

	T_CM.transform.translation.x = data->base_pose_.position.x;
	T_CM.transform.translation.y = data->base_pose_.position.y;
	T_CM.transform.translation.z = data->base_pose_.position.z;
	T_CM.transform.rotation = data->base_pose_.orientation;
	//	tf::Quaternion q_CM;
//	q_CM.setRPY(0,0,-2.356);
//	T_CM.setRotation(q_CM);

	T_PT_Base.transform.translation.x = data->pan_tilt_base_.position.x;
	T_PT_Base.transform.translation.y = data->pan_tilt_base_.position.y;
	T_PT_Base.transform.translation.z = data->pan_tilt_base_.position.z;
	T_PT_Base.transform.rotation = data->pan_tilt_base_.orientation;

	for (int ii=0;ii<data->sensors_.size();ii++)
	{
		if (!data->sensors_[ii].name.compare("tcp_rgb_cam"))
		{
			T_TRGB.transform.translation.x = data->sensors_[ii].pose.position.x;
			T_TRGB.transform.translation.y = data->sensors_[ii].pose.position.y;
			T_TRGB.transform.translation.z = data->sensors_[ii].pose.position.z;
			T_TRGB.transform.rotation = data->sensors_[ii].pose.orientation;
		}
		else if (!data->sensors_[ii].name.compare("tcp_depth_cam"))
		{
			T_TDEPTH.transform.translation.x = data->sensors_[ii].pose.position.x;
			T_TDEPTH.transform.translation.y = data->sensors_[ii].pose.position.y;
			T_TDEPTH.transform.translation.z = data->sensors_[ii].pose.position.z;
			T_TDEPTH.transform.rotation = data->sensors_[ii].pose.orientation;
		}
		else if (!data->sensors_[ii].name.compare("scene_rgb_cam"))
		{
			T_SRGB.transform.translation.x = data->sensors_[ii].pose.position.x;
			T_SRGB.transform.translation.y = data->sensors_[ii].pose.position.y;
			T_SRGB.transform.translation.z = data->sensors_[ii].pose.position.z;
			T_SRGB.transform.rotation = data->sensors_[ii].pose.orientation;
		}
		else if (!data->sensors_[ii].name.compare("scene_depth_cam"))
		{
			T_SDEPTH.transform.translation.x = data->sensors_[ii].pose.position.x;
			T_SDEPTH.transform.translation.y = data->sensors_[ii].pose.position.y;
			T_SDEPTH.transform.translation.z = data->sensors_[ii].pose.position.z;
			T_SDEPTH.transform.rotation = data->sensors_[ii].pose.orientation;
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
	sbr_.sendTransform(T_LA_Base);
	sbr_.sendTransform(T_GP);
	sbr_.sendTransform(T_GP_TCP);
	sbr_.sendTransform(T_TRGB);
	sbr_.sendTransform(T_TDEPTH);
	sbr_.sendTransform(T_CM);
	sbr_.sendTransform(T_PT_Base);
	sbr_.sendTransform(T_SRGB);
	sbr_.sendTransform(T_SDEPTH);
}



