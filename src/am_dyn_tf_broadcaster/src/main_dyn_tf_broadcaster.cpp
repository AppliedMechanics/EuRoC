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

void update_tf(const euroc_c2_msgs::Telemetry &msg)
{
	ros::NodeHandle nh;
	ros::ServiceClient forward_kin_client;

	static tf::TransformBroadcaster br;
	tf::Transform T_LWR;
	tf::Transform T_LA;
	tf::Transform T_PT;
	tf::Quaternion q_cam;

	q_cam.setRPY(0,0,0);

	T_LWR.setOrigin(tf::Vector3(0,0,0));
	T_LWR.setRotation(q_cam);
	T_LA.setOrigin(tf::Vector3(0,0,0));
	T_LA.setRotation(q_cam);
	T_PT.setOrigin(tf::Vector3(0,0,0));
	T_PT.setRotation(q_cam);

	//	br.sendTransform(tf::StampedTransform(T_LWR, ros::Time::now(),LWR_0,LWR_TCP));
	br.sendTransform(tf::StampedTransform(T_LA, ros::Time::now(),LA_0,LWR_0));
	br.sendTransform(tf::StampedTransform(T_LWR, ros::Time::now(),PT_0,PT_TCP));

	std::string euroc_c2_interface = "/euroc_interface_node";
	std::string forward_kin = euroc_c2_interface + "/get_forward_kinematics";


	//! Broadcast TCP TF
	// current_configuration will hold our current joint position data extracted from the measured telemetry
	euroc_c2_msgs::Configuration configuration;
	euroc_c2_msgs::Telemetry telemetry_msg;
	euroc_c2_msgs::GetForwardKinematics forward_kin_srv;
	configuration.q.resize(7);

	telemetry_msg = msg;

	//	// Populate a vector with all the lwr joint names
	//	const unsigned int nr_lwr_joints = 7;
	//	std::vector<std::string> lwr_joints(nr_lwr_joints);
	//	std::stringstream name;
	//	for(unsigned int i = 0; i < nr_lwr_joints; ++i){
	//		name.str("lwr_joint_");
	//		name.seekp(0, std::ios_base::end);
	//		name << (i + 1);
	//		lwr_joints[i] = name.str();
	//	}
	//	// Get the current configuration from the telemetry message
	//	for(unsigned int i = 0; i < 7; ++i){
	//		std::vector<std::string> &joint_names = telemetry_msg.joint_names;
	//		unsigned int telemetry_index = std::find(joint_names.begin(), joint_names.end(), lwr_joints[i]) - joint_names.begin();
	//		configuration.q[i] = telemetry_msg.measured.position[telemetry_index];
	//	}
	//	if (ros::service::waitForService(forward_kin,ros::Duration(0.5)))
	//	{
	//		forward_kin_srv.request.configuration = configuration;
	//		forward_kin_client.call(forward_kin_srv);
	//		std::string &ls_error_message = forward_kin_srv.response.error_message;
	//		if(!ls_error_message.empty()){
	//			ROS_ERROR("Forward Kinematics failed: %s", ls_error_message.c_str());
	//		}
	//		else
	//		{
	//			geometry_msgs::Pose &lwr_tcp_pose = forward_kin_srv.response.ee_frame;
	//			T_LWR.setOrigin(tf::Vector3(lwr_tcp_pose.position.x,lwr_tcp_pose.position.y,lwr_tcp_pose.position.z));
	//			T_LWR.setRotation(tf::Quaternion(lwr_tcp_pose.orientation.x,lwr_tcp_pose.orientation.y,lwr_tcp_pose.orientation.z,lwr_tcp_pose.orientation.w));
	//			std::cout<< "configuration "<<configuration.q[0]<<configuration.q[1]<<configuration.q[2]<<std::endl;
	//			std::cout << "x = "<<lwr_tcp_pose.position.x<<" y = "<<lwr_tcp_pose.position.y <<" z = "<<lwr_tcp_pose.position.z <<std::endl;
	//			std::cout << "x = "<<lwr_tcp_pose.orientation.x<<" y = "<<lwr_tcp_pose.orientation.y <<" z = "<<lwr_tcp_pose.orientation.z <<" w = "<< lwr_tcp_pose.orientation.w<<std::endl;
	//			br.sendTransform(tf::StampedTransform(T_LWR, ros::Time::now(),LWR_0,LWR_TCP));
	//		}
	//	}

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
	q_cam.setRPY(0.0,cam_pan,cam_tilt);
	T_PT.setRotation(q_cam);



	//	br.sendTransform(tf::StampedTransform(T_LWR, ros::Time::now(),LWR_0,LWR_TCP));
	br.sendTransform(tf::StampedTransform(T_LA, ros::Time::now(),LA_0,LWR_0));
	br.sendTransform(tf::StampedTransform(T_LWR, ros::Time::now(),PT_0,PT_TCP));

}


int main(int argc, char** argv) {

	ros::init(argc, argv, "am_dyn_tf_broadcaster");

	ros::NodeHandle node;
	//	ros::Timer br_timer_ = node.createTimer(ros::Duration(0.1),update_tf,false,true);
	//	br_timer_.start();

	std::string euroc_c2_interface = "/euroc_interface_node";
	std::string telemetry = euroc_c2_interface + "/telemetry";

	ros::Subscriber telemetry_subscriber = node.subscribe(telemetry, 1, update_tf);

	ros::spin();

	return 0;

}

