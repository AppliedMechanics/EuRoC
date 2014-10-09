/*
 * TFBroadcaster.cpp
 *
 *  Created on: Oct 9, 2014
 *      Author: euroc_admin
 */

#include <TFBroadcaster.hpp>

TFBroadcaster::TFBroadcaster() {

	euroc_c2_interface_ = "/euroc_interface_node";
	telemetry_ = euroc_c2_interface_ + "/telemetry";
	configuration_.q.resize(7,0);
	static_data_set_ = false;

	// Populate a vector with all the lwr joint names
	nr_lwr_joints_ = 7;
	lwr_joints_.resize(nr_lwr_joints_);
	for(unsigned int i = 0; i < nr_lwr_joints_; ++i){
		name_.str("lwr_joint_");
		name_.seekp(0, std::ios_base::end);
		name_ << (i + 1);
		lwr_joints_[i] = name_.str();
	}
}

TFBroadcaster::~TFBroadcaster() {
	// TODO Auto-generated destructor stub
}

void TFBroadcaster::init_tf()
{
	q_t_.w = 1;
	q_t_.x = 0;
	q_t_.y = 0;
	q_t_.z = 0;
	t_t_.x = 0;
	t_t_.y = 0;
	t_t_.z = 0;

	T_LA_.header.frame_id 		= LA_0;
	T_LA_.child_frame_id  		= LWR_0;
	T_LA_.transform.translation = t_t_;
	T_LA_.transform.rotation 	= q_t_;
	T_PT_.header.frame_id 		= PT_0;
	T_PT_.child_frame_id  		= PT_TCP;
	T_PT_.transform.translation = t_t_;
	T_PT_.transform.rotation 	= q_t_;
	A_LWR_10_.header.frame_id		= LWR_0;
	A_LWR_10_.child_frame_id		= lwr_1;
	A_LWR_10_.transform.rotation 	= q_t_;
	A_LWR_10_.transform.translation = t_t_;
	A_LWR_21_.header.frame_id		= lwr_1;
	A_LWR_21_.child_frame_id		= lwr_2;
	A_LWR_21_.transform.rotation 	= q_t_;
	A_LWR_21_.transform.translation = t_t_;
	A_LWR_32_.header.frame_id		= lwr_2;
	A_LWR_32_.child_frame_id		= lwr_3;
	A_LWR_32_.transform.rotation 	= q_t_;
	A_LWR_32_.transform.translation = t_t_;
	A_LWR_43_.header.frame_id		= lwr_3;
	A_LWR_43_.child_frame_id		= lwr_4;
	A_LWR_43_.transform.rotation 	= q_t_;
	A_LWR_43_.transform.translation = t_t_;
	A_LWR_54_.header.frame_id		= lwr_4;
	A_LWR_54_.child_frame_id		= lwr_5;
	A_LWR_54_.transform.rotation 	= q_t_;
	A_LWR_54_.transform.translation = t_t_;
	A_LWR_65_.header.frame_id		= lwr_5;
	A_LWR_65_.child_frame_id		= lwr_6;
	A_LWR_65_.transform.rotation 	= q_t_;
	A_LWR_65_.transform.translation = t_t_;
	A_LWR_76_.header.frame_id		= lwr_6;
	A_LWR_76_.child_frame_id		= LWR_TCP;
	A_LWR_76_.transform.rotation 	= q_t_;
	A_LWR_76_.transform.translation = t_t_;

	//! Setting up direct kinematics
	q_lwr_pi_2_.setRPY(1.57079632679,0,0);
	q_lwr_m_pi_2_.setRPY(-1.57079632679,0,0);
}


// ROS-Service Function
bool TFBroadcaster::set_static_tf_data(am_msgs::SetStaticTFData::Request &req, am_msgs::SetStaticTFData::Response &res)
{
	T_LA_Base_ = req.T_LA_Base;
	T_GP_      = req.T_GP;
	T_GP_TCP_  = req.T_GP_TCP;
	T_TRGB_    = req.T_TRGB;
	T_TDEPTH_  = req.T_TDEPTH;
	T_CM_      = req.T_CM;
	T_PT_Base_ = req.T_PT_Base;
	T_SRGB_    = req.T_SRGB;
	T_SDEPTH_  = req.T_SDEPTH;

	res.success = true;

	static_data_set_ = true;

	init_tf();
	return true;
}

bool TFBroadcaster::get_static_tf_data(am_msgs::GetStaticTFData::Request &req, am_msgs::GetStaticTFData::Response &res)
{
	if (!T_GP_.header.frame_id.compare(req.parent_frame) || !T_GP_TCP_.child_frame_id.compare(req.child_frame)){

		transform1.setOrigin(tf::Vector3(T_GP_.transform.translation.x,T_GP_.transform.translation.y,T_GP_.transform.translation.z));
		transform1.setRotation(tf::Quaternion(T_GP_.transform.rotation.x,T_GP_.transform.rotation.y,T_GP_.transform.rotation.z,T_GP_.transform.rotation.w));
		transform2.setOrigin(tf::Vector3(T_GP_TCP_.transform.translation.x,T_GP_TCP_.transform.translation.y,T_GP_TCP_.transform.translation.z));
		transform2.setRotation(tf::Quaternion(T_GP_TCP_.transform.rotation.x,T_GP_TCP_.transform.rotation.y,T_GP_TCP_.transform.rotation.z,T_GP_TCP_.transform.rotation.w));

		transformRes = transform1*transform2;

		res.transform.transform.translation.x = transformRes.getOrigin().x();
		res.transform.transform.translation.y = transformRes.getOrigin().y();
		res.transform.transform.translation.z = transformRes.getOrigin().z();

		res.transform.transform.rotation.x = transformRes.getRotation().x();
		res.transform.transform.rotation.y = transformRes.getRotation().y();
		res.transform.transform.rotation.z = transformRes.getRotation().z();
		res.transform.transform.rotation.w = transformRes.getRotation().w();

	}
	else {
		res.error_message = "No other static TFs can be given. If needed, add me!";
		return false;
	}

	return true;
}

void TFBroadcaster::update_tf()
{
	//! Broadcast TCP TF
	try
	{
		if (static_data_set_)
		{
			//! Get Current Telemetry data
			telemetry_msg_ = *(ros::topic::waitForMessage<euroc_c2_msgs::Telemetry>(telemetry_,ros::Duration(1.0)));
			now_ = ros::Time(0);

			// Get the current configuration from the telemetry message
			for(unsigned int i = 0; i < 7; ++i){
				std::vector<std::string> &joint_names_ = telemetry_msg_.joint_names;
				telemetry_index_ = std::find(joint_names_.begin(), joint_names_.end(), lwr_joints_[i]) - joint_names_.begin();
				configuration_.q[i] = telemetry_msg_.measured.position[telemetry_index_];
			}

			// Joint 1
			A_LWR_10_.transform.translation.z = 0.31;
			q_lwr_.setRPY(0.0,0,0);
			q_lwr_q_.setRPY(0,0,configuration_.q[0]);
			q_lwr_ *= q_lwr_q_;
			A_LWR_10_.transform.rotation.w = q_lwr_.w();
			A_LWR_10_.transform.rotation.x = q_lwr_.x();
			A_LWR_10_.transform.rotation.y = q_lwr_.y();
			A_LWR_10_.transform.rotation.z = q_lwr_.z();
			// Joint 2
			q_lwr_ = q_lwr_pi_2_;
			q_lwr_q_.setRPY(0,0,configuration_.q[1]);
			q_lwr_ *= q_lwr_q_;
			A_LWR_21_.transform.rotation.w = q_lwr_.w();
			A_LWR_21_.transform.rotation.x = q_lwr_.x();
			A_LWR_21_.transform.rotation.y = q_lwr_.y();
			A_LWR_21_.transform.rotation.z = q_lwr_.z();
			// Joint 3
			A_LWR_32_.transform.translation.y = 0.2;
			q_lwr_ = q_lwr_m_pi_2_;
			q_lwr_q_.setRPY(0,0,configuration_.q[2]);
			q_lwr_ *= q_lwr_q_;
			A_LWR_32_.transform.rotation.w = q_lwr_.w();
			A_LWR_32_.transform.rotation.x = q_lwr_.x();
			A_LWR_32_.transform.rotation.y = q_lwr_.y();
			A_LWR_32_.transform.rotation.z = q_lwr_.z();
			// Joint 4
			A_LWR_43_.transform.translation.z = 0.2;
			q_lwr_ = q_lwr_m_pi_2_;
			q_lwr_q_.setRPY(0,0,configuration_.q[3]);
			q_lwr_ *= q_lwr_q_;
			A_LWR_43_.transform.rotation.w = q_lwr_.w();
			A_LWR_43_.transform.rotation.x = q_lwr_.x();
			A_LWR_43_.transform.rotation.y = q_lwr_.y();
			A_LWR_43_.transform.rotation.z = q_lwr_.z();
			// Joint 5
			A_LWR_54_.transform.translation.y = -0.2;
			q_lwr_ = q_lwr_pi_2_;
			q_lwr_q_.setRPY(0,0,configuration_.q[4]);
			q_lwr_ *= q_lwr_q_;
			A_LWR_54_.transform.rotation.w = q_lwr_.w();
			A_LWR_54_.transform.rotation.x = q_lwr_.x();
			A_LWR_54_.transform.rotation.y = q_lwr_.y();
			A_LWR_54_.transform.rotation.z = q_lwr_.z();
			// Joint 6
			A_LWR_65_.transform.translation.z = 0.19;
			q_lwr_ = q_lwr_pi_2_;
			q_lwr_q_.setRPY(0,0,configuration_.q[5]);
			q_lwr_ *= q_lwr_q_;
			A_LWR_65_.transform.rotation.w = q_lwr_.w();
			A_LWR_65_.transform.rotation.x = q_lwr_.x();
			A_LWR_65_.transform.rotation.y = q_lwr_.y();
			A_LWR_65_.transform.rotation.z = q_lwr_.z();
			// Joint 7
			q_lwr_ = q_lwr_m_pi_2_;
			q_lwr_q_.setRPY(0,0,configuration_.q[6]);
			q_lwr_ *= q_lwr_q_;
			A_LWR_76_.transform.rotation.w = q_lwr_.w();
			A_LWR_76_.transform.rotation.x = q_lwr_.x();
			A_LWR_76_.transform.rotation.y = q_lwr_.y();
			A_LWR_76_.transform.rotation.z = q_lwr_.z();

			//! Transformation for linear Axis
			la_x_ = 0;
			la_y_ = 0;
			cam_pan_ = 0;
			cam_tilt_ = 0;
			for (int ii=0;ii<12;ii++)
			{
				if (!telemetry_msg_.joint_names[ii].compare("axis_x"))
					la_x_ = telemetry_msg_.measured.position[ii];
				if (!telemetry_msg_.joint_names[ii].compare("axis_y"))
					la_y_ = telemetry_msg_.measured.position[ii];
				if (!telemetry_msg_.joint_names[ii].compare("cam_pan"))
					cam_pan_ = telemetry_msg_.measured.position[ii];
				if (!telemetry_msg_.joint_names[ii].compare("cam_tilt"))
					cam_tilt_ = telemetry_msg_.measured.position[ii];
			}
			T_LA_.transform.translation.x = la_x_;
			T_LA_.transform.translation.y = la_y_;
			//! Transformation for Pan-tilt Unit
			//! Pan: z-axis, Tilt: new y-Axis
			q_cam_.setRPY(0.0,cam_tilt_,cam_pan_);
			T_PT_.transform.rotation.w = q_cam_.w();
			T_PT_.transform.rotation.x = q_cam_.x();
			T_PT_.transform.rotation.y = q_cam_.y();
			T_PT_.transform.rotation.z = q_cam_.z();
			//! Publish Transformations

			T_LA_.header.stamp = now_;
			T_PT_.header.stamp = now_;
			A_LWR_10_.header.stamp = now_;
			A_LWR_21_.header.stamp = now_;
			A_LWR_32_.header.stamp = now_;
			A_LWR_43_.header.stamp = now_;
			A_LWR_54_.header.stamp = now_;
			A_LWR_65_.header.stamp = now_;
			A_LWR_76_.header.stamp = now_;

			//	br.sendTransform(tf::StampedTransform(T_LWR, ros::Time::now(),LWR_0,LWR_TCP));
			br_.sendTransform(T_LA_);
			br_.sendTransform(T_PT_);
			br_.sendTransform(A_LWR_10_);
			br_.sendTransform(A_LWR_21_);
			br_.sendTransform(A_LWR_32_);
			br_.sendTransform(A_LWR_43_);
			br_.sendTransform(A_LWR_54_);
			br_.sendTransform(A_LWR_65_);
			br_.sendTransform(A_LWR_76_);
			//br.sendTransform(tf::StampedTransform(A_LWR_TCP7, ros::Time::now(),lwr_7,LWR_TCP));


			T_LA_Base_.header.stamp 	= now_;
			T_GP_.header.stamp 		= now_;
			T_GP_TCP_.header.stamp 	= now_;
			T_TRGB_.header.stamp 	= now_;
			T_TDEPTH_.header.stamp 	= now_;
			T_CM_.header.stamp 		= now_;
			T_PT_Base_.header.stamp 	= now_;
			T_SRGB_.header.stamp 	= now_;
			T_SDEPTH_.header.stamp 	= now_;

			br_.sendTransform(T_LA_Base_);
			br_.sendTransform(T_GP_);
			br_.sendTransform(T_GP_TCP_);
			br_.sendTransform(T_TRGB_);
			br_.sendTransform(T_TDEPTH_);
			br_.sendTransform(T_CM_);
			br_.sendTransform(T_PT_Base_);
			br_.sendTransform(T_SRGB_);
			br_.sendTransform(T_SDEPTH_);
		}
	}
	catch (...)
	{
		msg_warn("No telemetry msg received.");
	}

}
