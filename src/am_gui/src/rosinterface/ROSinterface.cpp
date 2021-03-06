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
												  system_limits_(12),
												  scenes_(1),
												  QObject(parent)
{
	//Wait for the simulator services and wait for them to be available:
	task_selector_ = "/euroc_c2_task_selector";
	start_simulator_= (task_selector_ + "/start_simulator");
	stop_simulator_= (task_selector_ + "/stop_simulator");
	list_scenes_ = (task_selector_ + "/list_scenes");

	euroc_c2_interface_ = "/euroc_interface_node";
	telemetry_ = euroc_c2_interface_ + "/telemetry";
	move_along_joint_path_ = euroc_c2_interface_ + "/move_along_joint_path";
	timing_along_joint_path_ = euroc_c2_interface_ + "/get_timing_along_joint_path";
	search_ik_solution_ = euroc_c2_interface_ + "/search_ik_solution";

	next_object_ = (euroc_c2_interface_ + "/request_next_object");
	set_stop_conditions_ = (euroc_c2_interface_ + "/set_stop_conditions");
	enable_servo_mode_ = (euroc_c2_interface_ + "/enable_servo_mode");
	set_servo_target_ = (euroc_c2_interface_ + "/set_servo_target");

	get_fk_ = (euroc_c2_interface_ + "/get_forward_kinematics");

	ros::service::waitForService(list_scenes_,ros::Duration(3.0));
	ros::service::waitForService(start_simulator_,ros::Duration(3.0));
	ros::service::waitForService(stop_simulator_,ros::Duration(3.0));


	move_along_joint_path_client_ = nh.serviceClient<euroc_c2_msgs::MoveAlongJointPath>(move_along_joint_path_);
	timing_along_joint_path_client_ = nh.serviceClient<euroc_c2_msgs::GetTimingAlongJointPath>(timing_along_joint_path_);
	search_ik_solution_client_ = nh.serviceClient<euroc_c2_msgs::SearchIkSolution>(search_ik_solution_);
	next_object_client_ = nh.serviceClient<euroc_c2_msgs::RequestNextObject>(next_object_);
	set_stop_conditions_client_ = nh.serviceClient<euroc_c2_msgs::SetStopConditions>(set_stop_conditions_);
	enable_servo_mode_client_ = nh.serviceClient<euroc_c2_msgs::EnableServoMode>(enable_servo_mode_);
	set_servo_target_client_ =  nh.serviceClient<euroc_c2_msgs::SetServoTarget>(set_servo_target_);
	get_fk_client_ = nh.serviceClient<euroc_c2_msgs::GetForwardKinematics>(get_fk_);

	list_scenes_client_     = nh.serviceClient<euroc_c2_msgs::ListScenes>(list_scenes_);
	start_simulator_client_ = nh.serviceClient<euroc_c2_msgs::StartSimulator>(start_simulator_);
	stop_simulator_client_  = nh.serviceClient<euroc_c2_msgs::StopSimulator>(stop_simulator_);


	telemetry_subscriber_ = nh.subscribe(telemetry_, 1, &ROSinterface::on_telemetry,this);

	commanded_configuration_.q.resize(12);

	if (!getSceneList())
		ROS_WARN("No scenes found.");

	speed_percentage_ = 30;

}


ROSinterface::~ROSinterface()
{

}

ROSinterface* ROSinterface::getInstance()
{
	if (m_ROSinterface == NULL)
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

void ROSinterface::callNextObject()
{
	if (ros::service::waitForService(next_object_,ros::Duration(1.0)))
	{
		next_object_client_.call(next_object_srv_);
		// The error_message field of each service response indicates whether an error occured. An empty string indicates success
		std::string &ls_error_message = next_object_srv_.response.error_message;
		if(!ls_error_message.empty()){
			ROS_ERROR("Request next Object failed: %s", ls_error_message.c_str());
		}
	}
	else
		ROS_WARN("Next Object Service has not been advertised.");

}

bool ROSinterface::getSceneList()
{
	/* The list scenes service returns all available scenes / tasks in the EuRoC C2 Simulation Challenge.
	     Each scene has a name and a description specified in yaml format.  */
	list_scenes_client_.call(list_scenes_srv_);

	// The error_message field of each service response indicates whether an error occured. An empty string indicates success
	std::string &ls_error_message = list_scenes_srv_.response.error_message;
	if(!ls_error_message.empty()){
		ROS_ERROR("List scenes failed: %s", ls_error_message.c_str());
		return false;
	}
	else
	{
		// Let's print the names of the received scenes
		std::cout << "[AM] Found the following scenes for the EuRoC C2 Simulation:" << std::endl;

		scenes_.resize(list_scenes_srv_.response.scenes.size());
		scenes_ = list_scenes_srv_.response.scenes;
		for(unsigned int i = 0; i < scenes_.size(); ++i){
			euroc_c2_msgs::Scene &scene = scenes_[i];
			std::cout << " - " << scene.name << std::endl;
		}
		return true;
	}

}

void ROSinterface::callStartSimulator(std::string task)
{
	euroc_c2_msgs::StartSimulator start_simulator_srv;
	start_simulator_srv.request.user_id = "AM-Robotics";

	start_simulator_srv.request.scene_name = task;

	start_simulator_client_.call(start_simulator_srv);

	// Check the response for errors
	std::string &error_message = start_simulator_srv.response.error_message;
	if(!error_message.empty()){
		ROS_ERROR("Starting the simulator failed: %s", error_message.c_str());
	}
}

void ROSinterface::callStopSimulator()
{

	euroc_c2_msgs::StopSimulator stop_simulator_srv;
	stop_simulator_client_.call(stop_simulator_srv);

}

void ROSinterface::callMoveToTargetPose(geometry_msgs::Pose target_pose)
{
	pose_ = target_pose;
	if (!getLimits())
		ROS_WARN("No Limits found.");
	if (getIKSolution7DOF() && getLimits())
		moveToTarget = boost::thread(&ROSinterface::moveToTargetCB,this);
	else
		ROS_WARN("IK Solution not found. Skipping Target.");

}

void ROSinterface::callSetCustomGoalConfiguration(double* commanded_joint_positions)
{
	if (ros::service::waitForService(move_along_joint_path_,ros::Duration(1.0)))
	{
		move_along_joint_path_srv_.request.joint_names = joint_names_; // Select all lwr joints
		move_along_joint_path_srv_.request.path.resize(1); // Our path has only one waypoint

		getUrdfConf();
		if (!getLimits())
			ROS_WARN("No Limits found.");
		// Initialize the velocity and acceleration limits of the joints
		move_along_joint_path_srv_.request.joint_limits.resize(12);
		for(unsigned int i = 0; i < 12; ++i){
			euroc_c2_msgs::Limits &limits = move_along_joint_path_srv_.request.joint_limits[i];
			limits.max_velocity = system_limits_[i].vel_limit; // 20 degrees per second
			limits.max_acceleration = system_limits_[i].acc_limit;
		}
		move_along_joint_path_srv_.request.joint_limits[0].max_acceleration = table_axis1_limit_.max_acceleration;
		move_along_joint_path_srv_.request.joint_limits[0].max_velocity     = table_axis1_limit_.max_velocity;
		move_along_joint_path_srv_.request.joint_limits[1].max_acceleration = table_axis2_limit_.max_acceleration;
		move_along_joint_path_srv_.request.joint_limits[1].max_velocity     = table_axis2_limit_.max_velocity;

		for (int ii=0;ii<7;ii++)
		{
			move_along_joint_path_srv_.request.joint_limits[ii+2].max_acceleration = joint_limits_[ii].max_acceleration;
			move_along_joint_path_srv_.request.joint_limits[ii+2].max_velocity     = joint_limits_[ii].max_velocity;
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
	else
		ROS_WARN("Service move to target configuration has not been advertised yet.");
}



bool ROSinterface::getIKSolution7DOF()
{
	if (ros::service::waitForService(move_along_joint_path_,ros::Duration(1.0)) &&
			ros::service::waitForService(search_ik_solution_,ros::Duration(1.0)))
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
		move_along_joint_path_srv_.request.path.resize(1);           // Our path has only one waypoint
		// Initialize the velocity and acceleration limits of the joints
		move_along_joint_path_srv_.request.joint_limits.resize(nr_lwr_joints);
		for(unsigned int i = 0; i < nr_lwr_joints; ++i){
			move_along_joint_path_srv_.request.joint_limits[i].max_acceleration = joint_limits_[i].max_acceleration;
			move_along_joint_path_srv_.request.joint_limits[i].max_velocity = joint_limits_[i].max_velocity;
		}

		// current_configuration will hold our current joint position data extracted from the measured telemetry
		euroc_c2_msgs::Configuration current_configuration;
		current_configuration.q.resize(nr_lwr_joints);
		// Wait for the subscribed topics to be published by the euroc simulator to avoid race conditions
		if (ros::topic::waitForMessage<euroc_c2_msgs::Telemetry>(telemetry_,ros::Duration(10))==NULL){
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
		search_ik_solution_client_.call(search_ik_solution_srv_);
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
	else
		return false;
}


void ROSinterface::moveToTargetCB()
{
	getTimingAlongJointPath();

	// Call the move request and check for errors
	move_along_joint_path_client_.call(move_along_joint_path_srv_);


	std::string &move_error_message = move_along_joint_path_srv_.response.error_message;
	if(!move_error_message.empty()){
		std::cout << "Move failed: " + move_error_message << std::endl;
	}

}

void ROSinterface::sendCurrentCfgOnce()
{
	emit emitCurrentCfgOnce(joint_names_,measured_positions_);
}

void ROSinterface::callGetFK()
{
	euroc_c2_msgs::Configuration current_cfg;
	geometry_msgs::Pose current_pose;

	current_cfg.q.resize(7);
	for (int i=0;i<7;i++)
		current_cfg.q[i] = measured_positions_[i+2];

	get_fk_srv_.request.configuration = current_cfg;
	get_fk_client_.call(get_fk_srv_);

	// The error_message field of each service response indicates whether an error occured. An empty string indicates success
	std::string &ls_error_message = get_fk_srv_.response.error_message;
	if(!ls_error_message.empty()){
		ROS_ERROR("Get FK failed: %s", ls_error_message.c_str());
	}
	else
	{
		current_pose = get_fk_srv_.response.ee_frame;
		emit emitFK(current_pose);
	}

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

void ROSinterface::getUrdfConf()
{

	std::string robotType;
	std::string urdf_robot = ros::package::getPath("am_robot_model");
	std::string gripper_urdf = urdf_robot;
	std::stringstream joint_name;

	urdf_robot.append("/kuka_lwr/kuka_lwr_mod.urdf");
	gripper_urdf.append("/pg70_gripper/pg70_gripper_mod.urdf");


	system_limits_[0].pos_limit_0 =  -1.84;
	system_limits_[0].pos_limit_1 =   1.84;
	system_limits_[0].vel_limit   =  0.1; //TODO 0.05; //TODO 0.5;
	system_limits_[0].acc_limit   =  5; //TODO 0.5; //TODO 0.5*(120.0/17.6);  // mass LWR + gripper = 17.6kg
	system_limits_[1] = system_limits_[0];

	if (!model_robot_.initFile(urdf_robot)){
		ROS_WARN("Failed to parse KUKA lwr urdf file.");
	}
	else{
		for (int i=2;i<9;i++){
			joint_name.str("joint");
			joint_name.seekp(0, std::ios_base::end);
			joint_name << (i-1);
			system_limits_[i].pos_limit_0 = (model_robot_.getJoint(joint_name.str()))->limits->lower;
			system_limits_[i].pos_limit_1 = (model_robot_.getJoint(joint_name.str()))->limits->upper;
			system_limits_[i].vel_limit   = (model_robot_.getJoint(joint_name.str()))->limits->velocity;
			system_limits_[i].acc_limit   = (model_robot_.getJoint(joint_name.str()))->limits->effort;
		}
	}
	if (!model_gripper_.initFile(gripper_urdf)){
		ROS_WARN("Failed to parse gripper urdf file.");
	}
	else{
		joint_name.str("joint_before_finger2");
		system_limits_[9].pos_limit_0 = (model_gripper_.getJoint(joint_name.str()))->limits->lower;
		system_limits_[9].pos_limit_1 = (model_gripper_.getJoint(joint_name.str()))->limits->upper * 2.0;
		system_limits_[9].vel_limit   = (model_gripper_.getJoint(joint_name.str()))->limits->velocity;
		system_limits_[9].acc_limit   = (model_gripper_.getJoint(joint_name.str()))->limits->effort;
	}

	for (int i=10;i<12;i++)
	{
		system_limits_[i].pos_limit_0 = -M_PI;
		system_limits_[i].pos_limit_1 = M_PI;
		system_limits_[i].vel_limit   = 0.1; //TODO 1.7453; // 20 degrees per second
		system_limits_[i].acc_limit   = 5; //TODO 400 * M_PI / 180.0;
	}

}

void ROSinterface::getTimingAlongJointPath()
{
	if (ros::service::waitForService(timing_along_joint_path_,ros::Duration(1.0)))
	{
		//TODO expected TIMING with getTimingAlongJointPath
		timing_along_joint_path_srv_.request.joint_names = move_along_joint_path_srv_.request.joint_names;

		int n_joints = timing_along_joint_path_srv_.request.joint_names.size();

		//timing_along_joint_path_srv_.request.start_pose =
		timing_along_joint_path_srv_.request.path = move_along_joint_path_srv_.request.path;
		timing_along_joint_path_srv_.request.joint_limits = move_along_joint_path_srv_.request.joint_limits;

		// current_configuration will hold our current joint position data extracted from the measured telemetry
		euroc_c2_msgs::Configuration current_configuration;
		current_configuration.q.resize(n_joints);
		// Wait for the subscribed topics to be published by the euroc simulator to avoid race conditions
		if (ros::topic::waitForMessage<euroc_c2_msgs::Telemetry>(telemetry_,ros::Duration(10))==NULL){
			ROS_WARN("No telemetry message received.");
		}

		int ii=0;
		// Get the current configuration from the telemetry message
		for(unsigned int i = 0; i < n_joints; ++i){
			std::vector<std::string> &joint_names = _telemetry.joint_names;
			unsigned int telemetry_index = std::find(joint_names.begin(), joint_names.end(), timing_along_joint_path_srv_.request.joint_names[ii]) - joint_names.begin();
			current_configuration.q[i] = _telemetry.measured.position[telemetry_index];
			ii++;
		}
		timing_along_joint_path_srv_.request.start_pose = current_configuration;

		timing_along_joint_path_client_.call(timing_along_joint_path_srv_);

		std::vector<ros::Time> time;
		time.resize(timing_along_joint_path_srv_.response.time_at_via_point.size());
		time = timing_along_joint_path_srv_.response.time_at_via_point;

		for (int i=0;i<time.size();i++)
			std::cout << "Time at Path Point "<<i<<": "<<time[i].sec<<","<<time[i].nsec<<"s"<<std::endl;
	}
	else
		ROS_WARN("Timing service has not been advertised.");
}

void ROSinterface::callSetStopConditions(std::vector<std::string> names,std::vector<std::string> operators,std::vector<double> values)
{
	if (ros::service::waitForService(set_stop_conditions_,ros::Duration(1.0)))
	{
		std::vector<euroc_c2_msgs::StopCondition> stop_conditions;
		stop_conditions.resize(names.size());
		for (int i=0;i<values.size();i++)
		{
			if (names[i].compare("gripper") == 0)
				stop_conditions[i].condition_type = "tool_force_threshold";
			else
				stop_conditions[i].condition_type = "joint_ext_torque_threshold";
			stop_conditions[i].condition_operator = operators[i];
			stop_conditions[i].joint_name = names[i];
			stop_conditions[i].threshold = values[i];
		}
		set_stop_conditions_srv_.request.conditions = stop_conditions;

		if (set_stop_conditions_client_.call(set_stop_conditions_srv_))
			ROS_INFO("Set Stop successfully condition called");
		else
			ROS_WARN("Setting stop condition not successful");

		std::string &sc_error_message = set_stop_conditions_srv_.response.error_message;
		if(!sc_error_message.empty()){
			std::cout << "Setting Stop Conditions failed: " + sc_error_message << std::endl;
		}
		else
			ROS_INFO("Stop condition has been set.");
	}
	else
		ROS_WARN("Set stop conditions service has not been advertised yet.");

}

void ROSinterface::callEnableServoMode(bool enable)
{
	if (ros::service::waitForService(enable_servo_mode_,ros::Duration(1.0)))
	{
		enable_servo_mode_srv_.request.servo_mode_active = enable;

		if (enable_servo_mode_client_.call(enable_servo_mode_srv_))
			ROS_INFO("Servo Mode Service successfully called.");
		else
			ROS_WARN("Setting enable servo mode not successful");

		std::string &servo_error_message = enable_servo_mode_srv_.response.error_message;
		if(!servo_error_message.empty()){
			std::cout << "Servo Mode Service failed: " + servo_error_message << std::endl;
		}
		else
			ROS_INFO("Servo Mode set.");

		if (ros::service::waitForService(set_servo_target_,ros::Duration(1.0)) && enable)
		{
			getUrdfConf();
			setServoTarget = boost::thread(&ROSinterface::sendServoTargetCB,this);
		}
		else if (!enable && ros::service::waitForService(set_servo_target_,ros::Duration(1.0)))
		{			setServoTarget.interrupt();
		std::cout << "end servo mode"<<std::endl;
		}
		else
			ROS_WARN("Send servo target service not applicable.");

	}
	else
		ROS_WARN("[Service] enable servo mode has not been advertised.");

}

void ROSinterface::sendServoTargetCB()
{
	while(true)
	{
		commanded_configuration_.q = _telemetry.measured.position;
		commanded_configuration_.q[servoing_joint_no_] += servoing_value_*0.001*system_limits_[servoing_joint_no_].vel_limit;

		set_servo_target_srv_.request.joint_names = _telemetry.joint_names;
		set_servo_target_srv_.request.target      = commanded_configuration_;
		set_servo_target_client_.call(set_servo_target_srv_);
		std::string &sm_error_message = set_servo_target_srv_.response.error_message;
		if(!sm_error_message.empty()){
			ROS_WARN("Setting Stop Conditions failed: %s", sm_error_message.c_str());
			break;
		}
		//ros::Duration(0.05).sleep();
	}
}

void ROSinterface::callSetCommandedConfiguration(int* joint_no,int* value)
{
	servoing_joint_no_ = joint_no[0];
	servoing_value_    = value[0];
}

bool ROSinterface::getLimits()
{
	//! Setting max velocities, getting from parameter server
	try
	{
		joint_limits_.resize(7);
		if (!ros::param::get("/two_axes_speed_limit_0",table_axis1_limit_.max_velocity))
			table_axis1_limit_.max_velocity = 0.5;
		if (!ros::param::get("/two_axes_speed_limit_1",table_axis2_limit_.max_velocity))
			table_axis2_limit_.max_velocity = 0.5;
		if (!ros::param::get("/joint_speed_limit_0",joint_limits_[0].max_velocity)){
			for (int ii=0;ii<7;ii++)
				joint_limits_[ii].max_velocity = 20 * M_PI / 180.0;
		}
		else
		{
			ros::param::get("/joint_speed_limit_1",joint_limits_[1].max_velocity);
			ros::param::get("/joint_speed_limit_2",joint_limits_[2].max_velocity);
			ros::param::get("/joint_speed_limit_3",joint_limits_[3].max_velocity);
			ros::param::get("/joint_speed_limit_4",joint_limits_[4].max_velocity);
			ros::param::get("/joint_speed_limit_5",joint_limits_[5].max_velocity);
			ros::param::get("/joint_speed_limit_6",joint_limits_[6].max_velocity);
		}
		if (!ros::param::get("/gripper_speed_limit",gripper_limit_.max_velocity))
			gripper_limit_.max_velocity = 0.5;
	}
	catch (...)
	{
		ROS_ERROR("GET LIMITS aborted. Setting default values.");
		table_axis1_limit_.max_velocity = 0.5;
		table_axis2_limit_.max_velocity = 0.5;
		for (int ii=0;ii<7;ii++)
			joint_limits_[ii].max_velocity = 20 * M_PI / 180.0;
		gripper_limit_.max_velocity = 0.5;
	}

	//! Setting max accelerations
	table_axis1_limit_.max_acceleration = 0.2;//2.0;
	table_axis2_limit_.max_acceleration = 0.2;//2.0;
	for (int ii=0;ii<7;ii++)
		joint_limits_[ii].max_acceleration = 200 * M_PI / 180.0;
	joint_limits_[5].max_acceleration = 30 * M_PI / 180.0;

	gripper_limit_.max_acceleration = 2.0;

	table_axis1_limit_.max_velocity *= (double)speed_percentage_*0.01;
	table_axis2_limit_.max_velocity *= (double)speed_percentage_*0.01;
	gripper_limit_.max_velocity     *= (double)speed_percentage_*0.01;
	for (int ii=0;ii<7;ii++)
		joint_limits_[ii].max_velocity *= (double)speed_percentage_*0.01;


	return true;

}

void ROSinterface::setSpeedPercentage(int sp)
{
	speed_percentage_ = sp;
}
