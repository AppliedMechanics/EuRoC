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
				  QObject(parent)
{
	//Wait for the simulator services and wait for them to be available:
	//  const std::string task_selector("/euroc_c2_task_selector");
	//  const std::string start_simulator= (task_selector + "/start_simulator");
	//  const std::string stop_simulator= (task_selector + "/stop_simulator");
	//
	//  const std::string euroc_c2_interface = "/euroc_interface_node";
	//  const std::string telemetry = euroc_c2_interface + "/telemetry";
	//  const std::string move_along_joint_path = euroc_c2_interface + "/move_along_joint_path";
	//  const std::string search_ik_solution = euroc_c2_interface + "/search_ik_solution";

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

void ROSinterface::callMoveToTargetPose(double* target_pose_)
{
	ros::service::waitForService(move_along_joint_path);
	ros::service::waitForService(search_ik_solution);


//	geometry_msgs::Pose pose;
//	euroc_c2_msgs::SearchIkSolution search_ik_solution_srv;
//	euroc_c2_msgs::MoveAlongJointPath move_along_joint_path_srv;

	pose.position.x = target_pose_[0];
	pose.position.y = target_pose_[1];
	pose.position.z = target_pose_[2];
	pose.orientation.w = target_pose_[3];
	pose.orientation.x = target_pose_[4];
	pose.orientation.y = target_pose_[5];
	pose.orientation.z = target_pose_[6];


	moveToTarget = boost::thread(&ROSinterface::moveToTargetCB,this);
}

void ROSinterface::moveToTargetCB()
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
	move_along_joint_path_srv.request.joint_names = lwr_joints; // Select all lwr joints
	move_along_joint_path_srv.request.path.resize(1); // Our path has only one waypoint
	// Initialize the velocity and acceleration limits of the joints
	move_along_joint_path_srv.request.joint_limits.resize(nr_lwr_joints);
	for(unsigned int i = 0; i < nr_lwr_joints; ++i){
		euroc_c2_msgs::Limits &limits = move_along_joint_path_srv.request.joint_limits[i];
		limits.max_velocity = 20 * M_PI / 180.0; // 20 degrees per second
		limits.max_acceleration = 400 * M_PI / 180.0;
	}

	// current_configuration will hold our current joint position data extracted from the measured telemetry
	euroc_c2_msgs::Configuration current_configuration;
	current_configuration.q.resize(nr_lwr_joints);
	// Wait for the subscribed topics to be published by the euroc simulator to avoid race conditions
	std::cout << "Waiting for topics to be published..." << std::endl;
	ros::topic::waitForMessage<euroc_c2_msgs::Telemetry>(telemetry);
	std::cout << "have topics." << std::endl;

	// Get the current configuration from the telemetry message
	for(unsigned int i = 0; i < nr_lwr_joints; ++i){
		std::vector<std::string> &joint_names = _telemetry.joint_names;
		unsigned int telemetry_index = std::find(joint_names.begin(), joint_names.end(), lwr_joints[i]) - joint_names.begin();
		current_configuration.q[i] = _telemetry.measured.position[telemetry_index];
	}

	// Select the next desired position of the tcp from the target zone poses and fill
	// the search inverse kinematic solution request with the current configuration as
	// start configuration and the desired position
	geometry_msgs::Pose &desired_pose = pose;
	search_ik_solution_srv.request.start = current_configuration;
	search_ik_solution_srv.request.tcp_frame = desired_pose;

	// Call the search inverse kinematic solution service and check for errors
	search_ik_solution_client.call(search_ik_solution_srv);
	std::string &search_error_message = search_ik_solution_srv.response.error_message;
	if(!search_error_message.empty()){
		ROS_ERROR("Search IK Solution failed: %s", search_error_message.c_str());
	}

	// Extract the solution configuration from the response and fill it into the path of the move request
	euroc_c2_msgs::Configuration &solution_configuration = search_ik_solution_srv.response.solution;
	std::vector<euroc_c2_msgs::Configuration> &path = move_along_joint_path_srv.request.path;
	path[0] = solution_configuration;

	// Call the move request and check for errors
	std::cout << "calling move_along_joint_path towards next target_zone" << std::endl;
	move_along_joint_path_client.call(move_along_joint_path_srv);
	//	std::string &move_error_message = move_along_joint_path_srv.response.error_message;
	//	if(!move_error_message.empty()){
	//		std::cout << "Move failed: " + move_error_message << std::endl;
	//	}

}

// ROS Callback for the telemetry topic
void ROSinterface::on_telemetry(const euroc_c2_msgs::Telemetry &telemetry){
	_telemetry = telemetry;
	for (int i=0;i<12;i++){
		jointNames[i] = QString::fromStdString(_telemetry.joint_names[i]);
		measuredPositions[i] = _telemetry.measured.position[i];
		measuredForces[i] = _telemetry.measured.torque[i];
		measuredExternalForces[i] = _telemetry.measured.external_torque[i];
	}

	emit emitMeasuredValues(jointNames,measuredPositions,measuredForces,measuredExternalForces);
}
