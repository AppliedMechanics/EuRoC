/*
 * main_dyn_tf_broadcaster.cpp
 *
 *  Created on: Aug 7, 2014
 *      Author: euroc_admin
 */

//! ROS
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

//! EUROC
#include <euroc_c2_msgs/Configuration.h>
#include <euroc_c2_msgs/Telemetry.h>
#include <euroc_c2_msgs/GetForwardKinematics.h>
#include <config.hpp>
#include <utils.hpp>

//! Init Transformations
geometry_msgs::TransformStamped T_LA;
geometry_msgs::TransformStamped T_PT;
//! Inner LWR Transformations
geometry_msgs::TransformStamped A_LWR_10;
geometry_msgs::TransformStamped A_LWR_21;
geometry_msgs::TransformStamped A_LWR_32;
geometry_msgs::TransformStamped A_LWR_43;
geometry_msgs::TransformStamped A_LWR_54;
geometry_msgs::TransformStamped A_LWR_65;
geometry_msgs::TransformStamped A_LWR_76;
geometry_msgs::TransformStamped A_LWR_TCP7;

geometry_msgs::Quaternion  q_t;
geometry_msgs::Vector3 t_t;

void init_tf()
{
	q_t.w = 1;
	q_t.x = 0;
	q_t.y = 0;
	q_t.z = 0;
	t_t.x = 0;
	t_t.y = 0;
	t_t.z = 0;

	T_LA.header.frame_id = LA_0;
	T_LA.child_frame_id  = LWR_0;
	T_LA.transform.translation = t_t;
	T_LA.transform.rotation = q_t;
	T_PT.header.frame_id = PT_0;
	T_PT.child_frame_id  = PT_TCP;
	T_PT.transform.translation = t_t;
	T_PT.transform.rotation = q_t;
	A_LWR_10.header.frame_id	= LWR_0;
	A_LWR_10.child_frame_id		= lwr_1;
	A_LWR_10.transform.rotation = q_t;
	A_LWR_10.transform.translation = t_t;
	A_LWR_21.header.frame_id	= lwr_1;
	A_LWR_21.child_frame_id		= lwr_2;
	A_LWR_21.transform.rotation = q_t;
	A_LWR_21.transform.translation = t_t;
	A_LWR_32.header.frame_id	= lwr_2;
	A_LWR_32.child_frame_id		= lwr_3;
	A_LWR_32.transform.rotation = q_t;
	A_LWR_32.transform.translation = t_t;
	A_LWR_43.header.frame_id	= lwr_3;
	A_LWR_43.child_frame_id		= lwr_4;
	A_LWR_43.transform.rotation = q_t;
	A_LWR_43.transform.translation = t_t;
	A_LWR_54.header.frame_id	= lwr_4;
	A_LWR_54.child_frame_id		= lwr_5;
	A_LWR_54.transform.rotation = q_t;
	A_LWR_54.transform.translation = t_t;
	A_LWR_65.header.frame_id	= lwr_5;
	A_LWR_65.child_frame_id		= lwr_6;
	A_LWR_65.transform.rotation = q_t;
	A_LWR_65.transform.translation = t_t;
	A_LWR_76.header.frame_id	= lwr_6;
	A_LWR_76.child_frame_id		= LWR_TCP;
	A_LWR_76.transform.rotation = q_t;
	A_LWR_76.transform.translation = t_t;

}

void update_tf(const ros::TimerEvent& event)
{
	ros::NodeHandle nh;
	static tf2_ros::TransformBroadcaster br;
	std::string euroc_c2_interface = "/euroc_interface_node";
	std::string telemetry = euroc_c2_interface + "/telemetry";
	std::string forward_kin = euroc_c2_interface + "/get_forward_kinematics";

	ros::ServiceClient forward_kin_client = nh.serviceClient<euroc_c2_msgs::GetForwardKinematics>(forward_kin);

	tf2::Quaternion q_cam;
	tf2::Quaternion q_lwr;
	tf2::Quaternion q_lwr_q;

	//! Broadcast TCP TF
	// current_configuration will hold our current joint position data extracted from the measured telemetry
	euroc_c2_msgs::Configuration configuration;

	euroc_c2_msgs::Telemetry telemetry_msg;
	euroc_c2_msgs::TelemetryConstPtr telemetry_ptr;

	configuration.q.resize(7,0);

	try
	{

		//! Get Current Telemetry data
		telemetry_msg = *(ros::topic::waitForMessage<euroc_c2_msgs::Telemetry>(telemetry,ros::Duration(1.0)));
		ros::Time now;
		now = ros::Time::now();

		// Populate a vector with all the lwr joint names
		const unsigned int nr_lwr_joints = 7;
		std::vector<std::string> lwr_joints(nr_lwr_joints);
		std::stringstream name;
		for(unsigned int i = 0; i < nr_lwr_joints; ++i){
			name.str("lwr_joint_");
			name.seekp(0, std::ios_base::end);
			name << (i + 1);
			lwr_joints[i] = name.str();
		}
		// Get the current configuration from the telemetry message
		for(unsigned int i = 0; i < 7; ++i){
			std::vector<std::string> &joint_names = telemetry_msg.joint_names;
			unsigned int telemetry_index = std::find(joint_names.begin(), joint_names.end(), lwr_joints[i]) - joint_names.begin();
			configuration.q[i] = telemetry_msg.measured.position[telemetry_index];

		}

		//! Setting up direct kinematics
		tf2::Quaternion q_lwr_pi_2;
		q_lwr_pi_2.setRPY(1.57079632679,0,0);
		tf2::Quaternion q_lwr_m_pi_2;
		q_lwr_m_pi_2.setRPY(-1.57079632679,0,0);
		// Joint 1
		A_LWR_10.transform.translation.z = 0.31;
		//A_LWR_10.setOrigin(tf::Vector3(0,0,0.31));
		q_lwr.setRPY(0.0,0,0);
		q_lwr_q.setRPY(0,0,configuration.q[0]);
		q_lwr *= q_lwr_q;
		A_LWR_10.transform.rotation.w = q_lwr.w();
		A_LWR_10.transform.rotation.x = q_lwr.x();
		A_LWR_10.transform.rotation.y = q_lwr.y();
		A_LWR_10.transform.rotation.z = q_lwr.z();
		// Joint 2
		q_lwr = q_lwr_pi_2;
		q_lwr_q.setRPY(0,0,configuration.q[1]);
		q_lwr *= q_lwr_q;
		A_LWR_21.transform.rotation.w = q_lwr.w();
		A_LWR_21.transform.rotation.x = q_lwr.x();
		A_LWR_21.transform.rotation.y = q_lwr.y();
		A_LWR_21.transform.rotation.z = q_lwr.z();
		// Joint 3
		A_LWR_32.transform.translation.y = 0.2;
		q_lwr = q_lwr_m_pi_2;
		q_lwr_q.setRPY(0,0,configuration.q[2]);
		q_lwr *= q_lwr_q;
		A_LWR_32.transform.rotation.w = q_lwr.w();
		A_LWR_32.transform.rotation.x = q_lwr.x();
		A_LWR_32.transform.rotation.y = q_lwr.y();
		A_LWR_32.transform.rotation.z = q_lwr.z();
		// Joint 4
		A_LWR_43.transform.translation.z = 0.2;
		q_lwr = q_lwr_m_pi_2;
		q_lwr_q.setRPY(0,0,configuration.q[3]);
		q_lwr *= q_lwr_q;
		A_LWR_43.transform.rotation.w = q_lwr.w();
		A_LWR_43.transform.rotation.x = q_lwr.x();
		A_LWR_43.transform.rotation.y = q_lwr.y();
		A_LWR_43.transform.rotation.z = q_lwr.z();
		// Joint 5
		A_LWR_54.transform.translation.y = -0.2;
		q_lwr = q_lwr_pi_2;
		q_lwr_q.setRPY(0,0,configuration.q[4]);
		q_lwr *= q_lwr_q;
		A_LWR_54.transform.rotation.w = q_lwr.w();
		A_LWR_54.transform.rotation.x = q_lwr.x();
		A_LWR_54.transform.rotation.y = q_lwr.y();
		A_LWR_54.transform.rotation.z = q_lwr.z();
		// Joint 6
		A_LWR_65.transform.translation.z = 0.19;
		q_lwr = q_lwr_pi_2;
		q_lwr_q.setRPY(0,0,configuration.q[5]);
		q_lwr *= q_lwr_q;
		A_LWR_65.transform.rotation.w = q_lwr.w();
		A_LWR_65.transform.rotation.x = q_lwr.x();
		A_LWR_65.transform.rotation.y = q_lwr.y();
		A_LWR_65.transform.rotation.z = q_lwr.z();
		// Joint 7
		q_lwr = q_lwr_m_pi_2;
		q_lwr_q.setRPY(0,0,configuration.q[6]);
		q_lwr *= q_lwr_q;
		A_LWR_76.transform.rotation.w = q_lwr.w();
		A_LWR_76.transform.rotation.x = q_lwr.x();
		A_LWR_76.transform.rotation.y = q_lwr.y();
		A_LWR_76.transform.rotation.z = q_lwr.z();

		//! Transformation for linear Axis
		double la_x,la_y,cam_pan,cam_tilt;
		la_x = 0;
		la_y = 0;
		cam_pan = 0;
		cam_tilt = 0;
		for (int ii=0;ii<12;ii++)
		{
			if (!telemetry_msg.joint_names[ii].compare("axis_x"))
				la_x = telemetry_msg.measured.position[ii];
			if (!telemetry_msg.joint_names[ii].compare("axis_y"))
				la_y = telemetry_msg.measured.position[ii];
			if (!telemetry_msg.joint_names[ii].compare("cam_pan"))
				cam_pan = telemetry_msg.measured.position[ii];
			if (!telemetry_msg.joint_names[ii].compare("cam_tilt"))
				cam_tilt = telemetry_msg.measured.position[ii];
		}
		T_LA.transform.translation.x = la_x;
		T_LA.transform.translation.y = la_y;
		//! Transformation for Pan-tilt Unit
		//! Pan: z-axis, Tilt: new y-Axis
		q_cam.setRPY(0.0,cam_tilt,cam_pan);
		T_PT.transform.rotation.w = q_cam.w();
		T_PT.transform.rotation.x = q_cam.x();
		T_PT.transform.rotation.y = q_cam.y();
		T_PT.transform.rotation.z = q_cam.z();
		//! Publish Transformations

		T_LA.header.stamp = now;
		T_PT.header.stamp = now;
		A_LWR_10.header.stamp = now;
		A_LWR_21.header.stamp = now;
		A_LWR_32.header.stamp = now;
		A_LWR_43.header.stamp = now;
		A_LWR_54.header.stamp = now;
		A_LWR_65.header.stamp = now;
		A_LWR_76.header.stamp = now;

		//	br.sendTransform(tf::StampedTransform(T_LWR, ros::Time::now(),LWR_0,LWR_TCP));
		br.sendTransform(T_LA);
		br.sendTransform(T_PT);
		br.sendTransform(A_LWR_10);
		br.sendTransform(A_LWR_21);
		br.sendTransform(A_LWR_32);
		br.sendTransform(A_LWR_43);
		br.sendTransform(A_LWR_54);
		br.sendTransform(A_LWR_65);
		br.sendTransform(A_LWR_76);
		//br.sendTransform(tf::StampedTransform(A_LWR_TCP7, ros::Time::now(),lwr_7,LWR_TCP));
	}
	catch (...)
	{
		msg_warn("No telemetry msg received.");
	}

}


int main(int argc, char** argv) {

	ros::init(argc, argv, "am_dyn_tf_broadcaster");

	ros::NodeHandle node;
	ros::Timer br_timer_ = node.createTimer(ros::Duration(0.1),update_tf,false,true);


	std::string euroc_c2_interface = "/euroc_interface_node";
	std::string telemetry = euroc_c2_interface + "/telemetry";

	//ros::Subscriber telemetry_subscriber = node.subscribe(telemetry, 1, update_tf);

	init_tf();
	ros::spin();

	return 0;

}

