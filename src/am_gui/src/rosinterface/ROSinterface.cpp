/*
 * Copyright (C) Institute of Applied Mechanics, TU Muenchen
 * All rights reserved.
 * Contact: schuetz@amm.mw.tum.de
 */


#include "ROSinterface.hpp"
#include <QSettings>
#include <QString>
#include <string.h>

#include <math.h>

ROSinterface* ROSinterface::m_ROSinterface = 0;

ROSinterface::ROSinterface(QObject *parent) :
				  joint_names_(12),
						  QObject(parent)
{
	//Wait for the simulator services and wait for them to be available:
	task_selector = "/euroc_c2_task_selector";
	start_simulator= (task_selector + "/start_simulator");
	stop_simulator= (task_selector + "/stop_simulator");

	euroc_c2_interface = "/euroc_interface_node";
	telemetry = euroc_c2_interface + "/telemetry";
	move_along_joint_path = euroc_c2_interface + "/move_along_joint_path";
	search_ik_solution = euroc_c2_interface + "/search_ik_solution";

	ros::service::waitForService(start_simulator);
	ros::service::waitForService(stop_simulator);

	move_along_joint_path_client = nh.serviceClient<euroc_c2_msgs::MoveAlongJointPath>(move_along_joint_path);
	search_ik_solution_client = nh.serviceClient<euroc_c2_msgs::SearchIkSolution>(search_ik_solution);

	start_simulator_client = nh.serviceClient<euroc_c2_msgs::StartSimulator>(start_simulator);
	stop_simulator_client  = nh.serviceClient<euroc_c2_msgs::StopSimulator>(stop_simulator);


	telemetry_subscriber = nh.subscribe(telemetry, 1, &ROSinterface::on_telemetry,this);

}

ROSinterface::~ROSinterface()
{

}

ROSinterface* ROSinterface::getInstance()
{

	if (m_ROSinterface == NULL)euroc_c2_msgs::SearchIkSolution search_ik_solution_srv;
	{
		m_ROSinterface = new ROSinterface();
	}
	return m_ROSinterface;

}


int ROSinterface::sgn(double x)
{
	if (x>0.0L)
		return 1.0L;
	else if(x<0.0L)
		return -1.0L;
	else
		return 0.0L;
}

void ROSinterface::callStartSimulator(int task)
{
	euroc_c2_msgs::StartSimulator start_simulator_srv;
	start_simulator_srv.request.user_id = "AM-Robotics";
	switch(task){
	case 0:
		start_simulator_srv.request.scene_name = "task1_v1";
		break;
	case 1:
		start_simulator_srv.request.scene_name = "task2_v1_1";
		break;
	case 2:
		start_simulator_srv.request.scene_name = "task3_v1";
		break;
	case 3:
		start_simulator_srv.request.scene_name = "task4_v1_1";
		break;
	case 4:
		start_simulator_srv.request.scene_name = "task5_v1";
		break;
	case 5:
		start_simulator_srv.request.scene_name = "task6_v1";
		break;
	default:
		start_simulator_srv.request.scene_name = "task1_v1";
		break;

	}
	start_simulator_client.call(start_simulator_srv);

	// Check the response for errors
	std::string &error_message = start_simulator_srv.response.error_message;
	if(!error_message.empty()){
		ROS_ERROR("Starting the simulator failed failed: %s", error_message.c_str());
	}
}

void ROSinterface::callStopSimulator()
{

	euroc_c2_msgs::StopSimulator stop_simulator_srv;
	stop_simulator_client.call(stop_simulator_srv);

}

void ROSinterface::callMoveToTargetPose(double* target_pose)
{
	ros::service::waitForService(move_along_joint_path);
	ros::service::waitForService(search_ik_solution);

	pose_.position.x = target_pose[0];
	pose_.position.y = target_pose[1];
	pose_.position.z = target_pose[2];
	pose_.orientation.w = target_pose[3];
	pose_.orientation.x = target_pose[4];
	pose_.orientation.y = target_pose[5];
	pose_.orientation.z = target_pose[6];

	if (getIKSolution7DOF())
		moveToTarget = boost::thread(&ROSinterface::moveToTargetCB,this);
	else
		ROS_WARN("IK Solution not found. Skipping Target.");

}

void ROSinterface::callSetCustomGoalConfiguration(double* commanded_joint_positions)
{
		move_along_joint_path_srv_.request.joint_names = joint_names_; // Select all lwr joints
		move_along_joint_path_srv_.request.path.resize(1); // Our path has only one waypoint

		// Initialize the velocity and acceleration limits of the joints
		move_along_joint_path_srv_.request.joint_limits.resize(12);
		for(unsigned int i = 0; i < 12; ++i){
			euroc_c2_msgs::Limits &limits = move_along_joint_path_srv_.request.joint_limits[i];
			limits.max_velocity = 20 * M_PI / 180.0; // 20 degrees per second
			limits.max_acceleration = 400 * M_PI / 180.0;
		}
		// current_configuration will hold our current joint position data extracted from the measured telemetry
		euroc_c2_msgs::Configuration commanded_configuration;
		commanded_configuration.q.resize(12);

		// Get the current configuration from the telemetry message
		for(unsigned int i = 0; i < 12; ++i){
			commanded_configuration.q[i] = commanded_joint_positions[i];
		}

		// Extract the solution configuration from the response and fill it into the path of the move request
		std::vector<euroc_c2_msgs::Configuration> &path = move_along_joint_path_srv_.request.path;
		path[0] = commanded_configuration;

		moveToTarget = boost::thread(&ROSinterface::moveToTargetCB,this);
}



bool ROSinterface::getIKSolution7DOF()
{
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
	move_along_joint_path_srv_.request.joint_names = lwr_joints; // Select all lwr joints
	move_along_joint_path_srv_.request.path.resize(1); // Our path has only one waypoint
	// Initialize the velocity and acceleration limits of the joints
	move_along_joint_path_srv_.request.joint_limits.resize(nr_lwr_joints);
	for(unsigned int i = 0; i < nr_lwr_joints; ++i){
		euroc_c2_msgs::Limits &limits = move_along_joint_path_srv_.request.joint_limits[i];
		limits.max_velocity = 20 * M_PI / 180.0; // 20 degrees per second
		limits.max_acceleration = 400 * M_PI / 180.0;
	}

	// current_configuration will hold our current joint position data extracted from the measured telemetry
	euroc_c2_msgs::Configuration current_configuration;
	current_configuration.q.resize(nr_lwr_joints);
	// Wait for the subscribed topics to be published by the euroc simulator to avoid race conditions
	if (ros::topic::waitForMessage<euroc_c2_msgs::Telemetry>(telemetry,ros::Duration(10))==NULL){
		ROS_WARN("No telemetry message received.");
		return false;
	}

	// Get the current configuration from the telemetry message
	for(unsigned int i = 0; i < nr_lwr_joints; ++i){
		std::vector<std::string> &joint_names = _telemetry.joint_names;
		unsigned int telemetry_index = std::find(joint_names.begin(), joint_names.end(), lwr_joints[i]) - joint_names.begin();
		current_configuration.q[i] = _telemetry.measured.position[telemetry_index];
	}

	// Select the next desired position of the tcp from the target zone poses and fill
	// the search inverse kinematic solution request with the current configuration as
	// start configuration and the desired position
	geometry_msgs::Pose &desired_pose = pose_;
	search_ik_solution_srv_.request.start = current_configuration;
	search_ik_solution_srv_.request.tcp_frame = desired_pose;

	// Call the search inverse kinematic solution service and check for errors
	search_ik_solution_client.call(search_ik_solution_srv_);
	std::string &search_error_message = search_ik_solution_srv_.response.error_message;
	if(!search_error_message.empty()){
		ROS_ERROR("Search IK Solution failed: %s", search_error_message.c_str());
		return false;
	}


	// Extract the solution configuration from the response and fill it into the path of the move request
	euroc_c2_msgs::Configuration &solution_configuration = search_ik_solution_srv_.response.solution;
	std::vector<euroc_c2_msgs::Configuration> &path = move_along_joint_path_srv_.request.path;
	path[0] = solution_configuration;

	return true;
}


void ROSinterface::moveToTargetCB()
{


	// Call the move request and check for errors
	move_along_joint_path_client.call(move_along_joint_path_srv_);
	std::string &move_error_message = move_along_joint_path_srv_.response.error_message;
	if(!move_error_message.empty()){
		std::cout << "Move failed: " + move_error_message << std::endl;
	}

}

void ROSinterface::sendCurrentCfgOnce()
{
	emit emitCurrentCfgOnce(joint_names_,measured_positions_);
}

// ROS Callback for the telemetry topic
void ROSinterface::on_telemetry(const euroc_c2_msgs::Telemetry &telemetry){
	_telemetry = telemetry;
	for (int i=0;i<12;i++){
		joint_names_[i] = _telemetry.joint_names[i];
		measured_positions_[i] = _telemetry.measured.position[i];
		measured_forces_[i] = _telemetry.measured.torque[i];
		measured_external_forces_[i] = _telemetry.measured.external_torque[i];
	}

	emit emitMeasuredValues(joint_names_,measured_positions_,measured_forces_,measured_external_forces_);
}
