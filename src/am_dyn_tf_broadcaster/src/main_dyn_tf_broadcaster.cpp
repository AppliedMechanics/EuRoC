/*
 * main_dyn_tf_broadcaster.cpp
 *
 *  Created on: Aug 7, 2014
 *      Author: euroc_admin
 */

//! ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//! EUROC
#include <euroc_c2_msgs/Configuration.h>
#include <euroc_c2_msgs/Telemetry.h>
#include <euroc_c2_msgs/GetForwardKinematics.h>
#include <config.hpp>
#include <utils.hpp>

bool call_active = false;

void update_tf(const ros::TimerEvent& event)
{
	ros::NodeHandle nh;

	static tf::TransformBroadcaster br;
	tf::Transform T_LWR;
	tf::Transform T_LA;
	tf::Transform T_PT;
	//! Inner LWR Transformations
	tf::Transform A_LWR_10;
	tf::Transform A_LWR_21;
	tf::Transform A_LWR_32;
	tf::Transform A_LWR_43;
	tf::Transform A_LWR_54;
	tf::Transform A_LWR_65;
	tf::Transform A_LWR_76;
	tf::Transform A_LWR_TCP7;


	tf::Quaternion q_cam;
	tf::Quaternion q_lwr;
	tf::Quaternion q_lwr_q;

	q_cam.setRPY(0,0,0);

	T_LWR.setOrigin(tf::Vector3(0,0,0));
	T_LWR.setRotation(q_cam);
	T_LA.setOrigin(tf::Vector3(0,0,0));
	T_LA.setRotation(q_cam);
	T_PT.setOrigin(tf::Vector3(0,0,0));
	T_PT.setRotation(q_cam);

	//	br.sendTransform(tf::StampedTransform(T_LWR, ros::Time::now(),LWR_0,LWR_TCP));
	//	br.sendTransform(tf::StampedTransform(T_LA, ros::Time::now(),LA_0,LWR_0));
	//	br.sendTransform(tf::StampedTransform(T_LWR, ros::Time::now(),PT_0,PT_TCP));

	std::string euroc_c2_interface = "/euroc_interface_node";
	std::string telemetry = euroc_c2_interface + "/telemetry";
	std::string forward_kin = euroc_c2_interface + "/get_forward_kinematics";

	ros::ServiceClient forward_kin_client = nh.serviceClient<euroc_c2_msgs::GetForwardKinematics>(forward_kin);

	//! Broadcast TCP TF
	// current_configuration will hold our current joint position data extracted from the measured telemetry
	euroc_c2_msgs::Configuration configuration;

	euroc_c2_msgs::Telemetry telemetry_msg;
	euroc_c2_msgs::TelemetryConstPtr telemetry_ptr;

	configuration.q.resize(7,0);

	try
	{
		telemetry_msg = *(ros::topic::waitForMessage<euroc_c2_msgs::Telemetry>(telemetry,ros::Duration(1.0)));


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
		tf::Quaternion q_lwr_pi_2;
		q_lwr_pi_2.setRPY(1.57079632679,0,0);
		tf::Quaternion q_lwr_m_pi_2;
		q_lwr_m_pi_2.setRPY(-1.57079632679,0,0);
		// Joint 1
		A_LWR_10.setOrigin(tf::Vector3(0,0,0.31));
		q_lwr.setRPY(0.0,0,0);
		q_lwr_q.setRPY(0,0,configuration.q[0]);
		q_lwr *= q_lwr_q;
		A_LWR_10.setRotation(q_lwr);
		// Joint 2
		A_LWR_21.setOrigin(tf::Vector3(0,0.0,0.0));
		q_lwr = q_lwr_pi_2;
		q_lwr_q.setRPY(0,0,configuration.q[1]);
		q_lwr *= q_lwr_q;
		A_LWR_21.setRotation(q_lwr);
		// Joint 3
		A_LWR_32.setOrigin(tf::Vector3(0,0.2,0.0));
		q_lwr = q_lwr_m_pi_2;
		q_lwr_q.setRPY(0,0,configuration.q[2]);
		q_lwr *= q_lwr_q;
		A_LWR_32.setRotation(q_lwr);
		// Joint 4
		A_LWR_43.setOrigin(tf::Vector3(0,0,0.2));
		q_lwr = q_lwr_m_pi_2;
		q_lwr_q.setRPY(0,0,configuration.q[3]);
		q_lwr *= q_lwr_q;
		A_LWR_43.setRotation(q_lwr);
		// Joint 5
		A_LWR_54.setOrigin(tf::Vector3(0,-0.2,0.0));
		q_lwr = q_lwr_pi_2;
		q_lwr_q.setRPY(0,0,configuration.q[4]);
		q_lwr *= q_lwr_q;
		A_LWR_54.setRotation(q_lwr);
		// Joint 6
		A_LWR_65.setOrigin(tf::Vector3(0,0,0.19));
		q_lwr = q_lwr_pi_2;
		q_lwr_q.setRPY(0,0,configuration.q[5]);
		q_lwr *= q_lwr_q;
		A_LWR_65.setRotation(q_lwr);
		// Joint 7
		A_LWR_76.setOrigin(tf::Vector3(0,0,0.0));
		q_lwr = q_lwr_m_pi_2;
		q_lwr_q.setRPY(0,0,configuration.q[6]);
		q_lwr *= q_lwr_q;
		A_LWR_76.setRotation(q_lwr);
		// TCP
		A_LWR_TCP7.setOrigin(tf::Vector3(0,0,0.0));
		q_lwr.setRPY(0,0,0);
		//	q_lwr_q.setRPY(0,0,0);
		//	q_lwr *= q_lwr_q;
		A_LWR_TCP7.setRotation(q_lwr);

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
		T_LA.setOrigin(tf::Vector3(la_x,la_y,0.0));
		//! Pan: z-axis, Tilt: new y-Axis
		q_cam.setRPY(0.0,cam_tilt,cam_pan);
		T_PT.setRotation(q_cam);



		//	br.sendTransform(tf::StampedTransform(T_LWR, ros::Time::now(),LWR_0,LWR_TCP));
		br.sendTransform(tf::StampedTransform(T_LA, ros::Time::now(),LA_0,LWR_0));
		br.sendTransform(tf::StampedTransform(T_LWR, ros::Time::now(),PT_0,PT_TCP));

		br.sendTransform(tf::StampedTransform(A_LWR_10, ros::Time::now(),LWR_0,lwr_1));
		br.sendTransform(tf::StampedTransform(A_LWR_21, ros::Time::now(),lwr_1,lwr_2));
		br.sendTransform(tf::StampedTransform(A_LWR_32, ros::Time::now(),lwr_2,lwr_3));
		br.sendTransform(tf::StampedTransform(A_LWR_43, ros::Time::now(),lwr_3,lwr_4));
		br.sendTransform(tf::StampedTransform(A_LWR_54, ros::Time::now(),lwr_4,lwr_5));
		br.sendTransform(tf::StampedTransform(A_LWR_65, ros::Time::now(),lwr_5,lwr_6));
		br.sendTransform(tf::StampedTransform(A_LWR_76, ros::Time::now(),lwr_6,lwr_7));
		br.sendTransform(tf::StampedTransform(A_LWR_TCP7, ros::Time::now(),lwr_7,LWR_TCP));
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

	ros::spin();

	return 0;

}

