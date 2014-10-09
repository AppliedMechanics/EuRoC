/*
 * TFBroadcaster.h
 *
 *  Created on: Oct 9, 2014
 *      Author: euroc_admin
 */

#ifndef TFBROADCASTER_H_
#define TFBROADCASTER_H_

//! ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

//! EUROC
#include <euroc_c2_msgs/Configuration.h>
#include <euroc_c2_msgs/Telemetry.h>
#include <config.hpp>
#include <utils.hpp>

//! AM
#include <am_msgs/SetStaticTFData.h>
#include <am_msgs/GetStaticTFData.h>

class TFBroadcaster {
public:
	TFBroadcaster();
	virtual ~TFBroadcaster();

	bool set_static_tf_data(am_msgs::SetStaticTFData::Request &req, am_msgs::SetStaticTFData::Response &res);
	bool get_static_tf_data(am_msgs::GetStaticTFData::Request &req, am_msgs::GetStaticTFData::Response &res);

	void update_tf();

private:
	tf::TransformBroadcaster br_;

	//! Init Dynamic Transformations
	geometry_msgs::TransformStamped T_LA_;
	geometry_msgs::TransformStamped T_PT_;
	//! Inner LWR Transformations
	geometry_msgs::TransformStamped A_LWR_10_;
	geometry_msgs::TransformStamped A_LWR_21_;
	geometry_msgs::TransformStamped A_LWR_32_;
	geometry_msgs::TransformStamped A_LWR_43_;
	geometry_msgs::TransformStamped A_LWR_54_;
	geometry_msgs::TransformStamped A_LWR_65_;
	geometry_msgs::TransformStamped A_LWR_76_;
	geometry_msgs::TransformStamped A_LWR_TCP7_;

	geometry_msgs::Quaternion  q_t_;
	geometry_msgs::Vector3 t_t_;

	//! Static TF Data
	geometry_msgs::TransformStamped T_LA_Base_;
	geometry_msgs::TransformStamped T_GP_;
	geometry_msgs::TransformStamped T_GP_TCP_;
	geometry_msgs::TransformStamped T_TRGB_;
	geometry_msgs::TransformStamped T_TDEPTH_;
	geometry_msgs::TransformStamped T_CM_;
	geometry_msgs::TransformStamped T_PT_Base_;
	geometry_msgs::TransformStamped T_SRGB_;
	geometry_msgs::TransformStamped T_SDEPTH_;

	std::string euroc_c2_interface_;
	std::string telemetry_;

	tf::Quaternion q_cam_;
	tf::Quaternion q_lwr_;
	tf::Quaternion q_lwr_q_;

	//! Setting up direct kinematics
	tf::Quaternion q_lwr_pi_2_;
	tf::Quaternion q_lwr_m_pi_2_;

	//! Return transform of kinematic chain
	tf::Transform transform1, transform2, transformRes;

	double la_x_,la_y_,cam_pan_,cam_tilt_;

	//! Broadcast TCP TF
	// current_configuration will hold our current joint position data extracted from the measured telemetry
	euroc_c2_msgs::Configuration configuration_;

	euroc_c2_msgs::Telemetry telemetry_msg_;
	euroc_c2_msgs::TelemetryConstPtr telemetry_ptr_;

	bool static_data_set_;
	ros::Time now_;

	unsigned int nr_lwr_joints_;
	unsigned int telemetry_index_;
	std::vector<std::string> lwr_joints_;
	std::stringstream name_;

	void init_tf();
};

#endif /* TFBROADCASTER_H_ */
