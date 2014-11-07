/*
 * StaticTFBroadcaster.cpp
 *
 *  Created on: Aug 6, 2014
 *      Author: euroc_admin
 */

#include <StaticTFBroadcaster.h>
#include <euroc_input.hpp>


StaticTFBroadcaster::StaticTFBroadcaster() {

	set_static_tf_data_ = "/set_static_tf_data";
	set_static_tf_data_client_ = nh_.serviceClient<am_msgs::SetStaticTFData>(set_static_tf_data_);

	set_static_tf_data_srv_.request.T_LA_Base.header.frame_id = ORIGIN;
	set_static_tf_data_srv_.request.T_LA_Base.child_frame_id = LA_0;

	set_static_tf_data_srv_.request.T_GP.header.frame_id = LWR_TCP;
	set_static_tf_data_srv_.request.T_GP.child_frame_id  = GP_0;

	set_static_tf_data_srv_.request.T_GP_TCP.header.frame_id = GP_0;
	set_static_tf_data_srv_.request.T_GP_TCP.child_frame_id  = GP_TCP;

	set_static_tf_data_srv_.request.T_TRGB.header.frame_id = GP_0;
	set_static_tf_data_srv_.request.T_TRGB.child_frame_id  = T_RGB;

	set_static_tf_data_srv_.request.T_TDEPTH.header.frame_id = T_RGB;
	set_static_tf_data_srv_.request.T_TDEPTH.child_frame_id  = T_DEPTH;

	set_static_tf_data_srv_.request.T_CM.header.frame_id = ORIGIN;
	set_static_tf_data_srv_.request.T_CM.child_frame_id  = CM;

	set_static_tf_data_srv_.request.T_PT_Base.header.frame_id = CM;
	set_static_tf_data_srv_.request.T_PT_Base.child_frame_id  = PT_0;

	set_static_tf_data_srv_.request.T_SRGB.header.frame_id = PT_TCP;
	set_static_tf_data_srv_.request.T_SRGB.child_frame_id  = S_RGB;

	set_static_tf_data_srv_.request.T_SDEPTH.header.frame_id = S_RGB;
	set_static_tf_data_srv_.request.T_SDEPTH.child_frame_id  = S_DEPTH;

	set_static_tf_data_srv_.request.T_PUZZLE.header.frame_id = ORIGIN;
	set_static_tf_data_srv_.request.T_PUZZLE.child_frame_id  = PUZZLE_FIXTURE;

}

StaticTFBroadcaster::~StaticTFBroadcaster() {
	// TODO Auto-generated destructor stub
}

void StaticTFBroadcaster::fill_tf_information(EurocInput*data)
{

	int active_task_nr = 0;
	ros::param::get("/active_task_number_", active_task_nr);

	if (active_task_nr==5)
	{
		set_static_tf_data_srv_.request.T_PUZZLE.transform.translation.x = data->fixture_pose_.position.x;
		set_static_tf_data_srv_.request.T_PUZZLE.transform.translation.y = data->fixture_pose_.position.y;
		set_static_tf_data_srv_.request.T_PUZZLE.transform.translation.z = data->fixture_pose_.position.z;
		set_static_tf_data_srv_.request.T_PUZZLE.transform.rotation      = data->fixture_pose_.orientation;
	}
	else
	{
		set_static_tf_data_srv_.request.T_PUZZLE.transform.translation.x = 0.0;
		set_static_tf_data_srv_.request.T_PUZZLE.transform.translation.y = 0.0;
		set_static_tf_data_srv_.request.T_PUZZLE.transform.translation.z = 0.0;
		set_static_tf_data_srv_.request.T_PUZZLE.transform.rotation.x    = 0.0;
		set_static_tf_data_srv_.request.T_PUZZLE.transform.rotation.y    = 0.0;
		set_static_tf_data_srv_.request.T_PUZZLE.transform.rotation.z    = 0.0;
		set_static_tf_data_srv_.request.T_PUZZLE.transform.rotation.w    = 1.0;
	}

	set_static_tf_data_srv_.request.T_LA_Base.transform.translation.x = data->robot_.pose.position.x;
	set_static_tf_data_srv_.request.T_LA_Base.transform.translation.y = data->robot_.pose.position.y;
	set_static_tf_data_srv_.request.T_LA_Base.transform.translation.z = data->robot_.pose.position.z;
	set_static_tf_data_srv_.request.T_LA_Base.transform.rotation    = data->robot_.pose.orientation;

	set_static_tf_data_srv_.request.T_GP.transform.translation.x = data->robot_.gripper_pose.position.x;
	set_static_tf_data_srv_.request.T_GP.transform.translation.y = data->robot_.gripper_pose.position.y;
	set_static_tf_data_srv_.request.T_GP.transform.translation.z = data->robot_.gripper_pose.position.z;
	set_static_tf_data_srv_.request.T_GP.transform.rotation = data->robot_.gripper_pose.orientation;

	set_static_tf_data_srv_.request.T_GP_TCP.transform.translation.x = data->robot_.gripper_tcp.position.x;
	set_static_tf_data_srv_.request.T_GP_TCP.transform.translation.y = data->robot_.gripper_tcp.position.y;
	set_static_tf_data_srv_.request.T_GP_TCP.transform.translation.z = data->robot_.gripper_tcp.position.z;
	set_static_tf_data_srv_.request.T_GP_TCP.transform.rotation = data->robot_.gripper_tcp.orientation;

	set_static_tf_data_srv_.request.T_CM.transform.translation.x = data->base_pose_.position.x;
	set_static_tf_data_srv_.request.T_CM.transform.translation.y = data->base_pose_.position.y;
	set_static_tf_data_srv_.request.T_CM.transform.translation.z = data->base_pose_.position.z;
	set_static_tf_data_srv_.request.T_CM.transform.rotation = data->base_pose_.orientation;
	//	tf::Quaternion q_CM;
	//	q_CM.setRPY(0,0,-2.356);
	//	T_CM.setRotation(q_CM);

	set_static_tf_data_srv_.request.T_PT_Base.transform.translation.x = data->pan_tilt_base_.position.x;
	set_static_tf_data_srv_.request.T_PT_Base.transform.translation.y = data->pan_tilt_base_.position.y;
	set_static_tf_data_srv_.request.T_PT_Base.transform.translation.z = data->pan_tilt_base_.position.z;
	set_static_tf_data_srv_.request.T_PT_Base.transform.rotation = data->pan_tilt_base_.orientation;

	for (int ii=0;ii<data->sensors_.size();ii++)
	{
		if (!data->sensors_[ii].name.compare("tcp_rgb_cam"))
		{
			set_static_tf_data_srv_.request.T_TRGB.transform.translation.x = data->sensors_[ii].pose.position.x;
			set_static_tf_data_srv_.request.T_TRGB.transform.translation.y = data->sensors_[ii].pose.position.y;
			set_static_tf_data_srv_.request.T_TRGB.transform.translation.z = data->sensors_[ii].pose.position.z;
			set_static_tf_data_srv_.request.T_TRGB.transform.rotation = data->sensors_[ii].pose.orientation;
		}
		else if (!data->sensors_[ii].name.compare("tcp_depth_cam"))
		{
			set_static_tf_data_srv_.request.T_TDEPTH.transform.translation.x = data->sensors_[ii].pose.position.x;
			set_static_tf_data_srv_.request.T_TDEPTH.transform.translation.y = data->sensors_[ii].pose.position.y;
			set_static_tf_data_srv_.request.T_TDEPTH.transform.translation.z = data->sensors_[ii].pose.position.z;
			set_static_tf_data_srv_.request.T_TDEPTH.transform.rotation = data->sensors_[ii].pose.orientation;
		}
		else if (!data->sensors_[ii].name.compare("scene_rgb_cam"))
		{
			set_static_tf_data_srv_.request.T_SRGB.transform.translation.x = data->sensors_[ii].pose.position.x;
			set_static_tf_data_srv_.request.T_SRGB.transform.translation.y = data->sensors_[ii].pose.position.y;
			set_static_tf_data_srv_.request.T_SRGB.transform.translation.z = data->sensors_[ii].pose.position.z;
			set_static_tf_data_srv_.request.T_SRGB.transform.rotation = data->sensors_[ii].pose.orientation;
		}
		else if (!data->sensors_[ii].name.compare("scene_depth_cam"))
		{
			set_static_tf_data_srv_.request.T_SDEPTH.transform.translation.x = data->sensors_[ii].pose.position.x;
			set_static_tf_data_srv_.request.T_SDEPTH.transform.translation.y = data->sensors_[ii].pose.position.y;
			set_static_tf_data_srv_.request.T_SDEPTH.transform.translation.z = data->sensors_[ii].pose.position.z;
			set_static_tf_data_srv_.request.T_SDEPTH.transform.rotation = data->sensors_[ii].pose.orientation;
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
	try
	{
		if (ros::service::waitForService(set_static_tf_data_,ros::Duration(10.0)) )
		{
			if(!set_static_tf_data_client_.call(set_static_tf_data_srv_))
			{
				msg_error("Setting Static TF Data call failed");
			}

		}
	}
	catch (...)
	{
		msg_error("TRY CATCH --> Setting Static TF Data call failed");
	}
}



