/*
 * MotionPlanning.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: euroc_admin
 */

#include "MotionPlanning.h"

MotionPlanning::MotionPlanning():
goalPose_server_(nh_, "goalPoseAction", boost::bind(&MotionPlanning::executeGoalPose_CB, this, _1),false),
goalPose_action_name_("goalPoseAction"),
time_at_path_points_(1)
{
	euroc_c2_interface_ = "/euroc_interface_node";
	telemetry_ = euroc_c2_interface_ + "/telemetry";
	move_along_joint_path_ = euroc_c2_interface_ + "/move_along_joint_path";
	timing_along_joint_path_ = euroc_c2_interface_ + "/get_timing_along_joint_path";
	search_ik_solution_ = euroc_c2_interface_ + "/search_ik_solution";
	get_dk_solution_ = euroc_c2_interface_ + "/get_forward_kinematics";

	move_along_joint_path_client_   = nh_.serviceClient<euroc_c2_msgs::MoveAlongJointPath>(move_along_joint_path_);
	timing_along_joint_path_client_ = nh_.serviceClient<euroc_c2_msgs::GetTimingAlongJointPath>(timing_along_joint_path_);
	search_ik_solution_client_      = nh_.serviceClient<euroc_c2_msgs::SearchIkSolution>(search_ik_solution_);
	get_dk_solution_client_         = nh_.serviceClient<euroc_c2_msgs::GetForwardKinematics>(get_dk_solution_);
	state_observer_client_          = nh_.serviceClient<am_msgs::CallSetStopConditions>("CallSetStopConditions_srv");  //TODO remove (by Anna)

	joint_limits_.resize(7);

	goalPose_server_.start();
	msg_info("goalPose action server started.");


	check_poses_service_ = nh_.advertiseService("CheckPoses_srv", &MotionPlanning::return_poses_valid,this);

	feedback_frequency_ = 2;

	mtt_=OPEN;
	called = false;
}

MotionPlanning::~MotionPlanning() {}

void MotionPlanning::executeGoalPose_CB(const am_msgs::goalPoseGoal::ConstPtr &goal)
{
	msg_info("In execution call");

	goal_pose_goal_ = goal;
	speed_percentage_ = goal_pose_goal_->speed_percentage;
	inter_steps_ = goal_pose_goal_->inter_steps;

	if (speed_percentage_ <= 0 || speed_percentage_ >100)
		speed_percentage_ = 40;

	getLimits();

	ros::Rate feedback_rate(feedback_frequency_);

	if (!getTelemetry()){
		msg_error("getTelemetry: An Error happened here.");
		goalPose_result_.reached_goal = false;
		goalPose_server_.setPreempted(goalPose_result_,"Got no telemetry.");
	}
	switch (goal->planning_algorithm)
	{
	case STANDARD_IK_7DOF:

		ROS_WARN("STANDARD IK 7DOF planning mode chosen.");

		if (!getIKSolution7DOF())
		{
			msg_error("No IK Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No IK Solution found.");
			break;
		}

		getTimingAlongJointPath();

		starting_time_ = ros::Time::now().toSec();
		getGoalPose_Feedback();

		goalPose_server_.publishFeedback(goalPose_feedback_);

		moveToTarget = boost::thread(&MotionPlanning::moveToTargetCB,this);

		mtt_=RUNNING;
		//while (!goalPose_result_.reached_goal)
		while(mtt_==RUNNING)
		{
			getGoalPose_Feedback();
			goalPose_server_.publishFeedback(goalPose_feedback_);

			feedback_rate.sleep();
		}

		moveToTarget.detach();
		//if (goalPose_feedback_.execution_time >= (goalPose_feedback_.estimated_motion_time+2.0))
		if(mtt_==FINISHED)
		{
			goalPose_result_.reached_goal = true;
			goalPose_server_.setSucceeded(goalPose_result_, "Goal configuration has been reached");
		}
		else
		{
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"Something strange happened.");
		}

		break;

	case HOMING_7DOF:
		ROS_WARN("HOMING 7DOF planning mode chosen.");

		if (!setReset7DOF())
			msg_error("Problem at HOMING");
		getTimingAlongJointPath();
		starting_time_ = ros::Time::now().toSec();
		getGoalPose_Feedback();

		goalPose_server_.publishFeedback(goalPose_feedback_);

		moveToTarget = boost::thread(&MotionPlanning::moveToTargetCB,this);

		mtt_=RUNNING;
		//while (!goalPose_result_.reached_goal)
		while(mtt_==RUNNING)
		{
			getGoalPose_Feedback();
			goalPose_server_.publishFeedback(goalPose_feedback_);

			feedback_rate.sleep();
		}

		moveToTarget.detach();
		//if (goalPose_feedback_.execution_time >= (goalPose_feedback_.estimated_motion_time+2.0))
		if(mtt_==FINISHED)
		{
			goalPose_result_.reached_goal = true;
			goalPose_server_.setSucceeded(goalPose_result_, "Goal configuration has been reached");
		}
		else
		{
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"Something strange happend");
		}


		break;

	default:
		break;
	}

}

void MotionPlanning::getGoalPose_Feedback()
{
	goalPose_feedback_.estimated_motion_time = estimated_motion_time_;
	goalPose_feedback_.execution_time = ros::Time::now().toSec() - starting_time_;

}



bool MotionPlanning::getIKSolution7DOF()
{
	try
	{
		if (ros::service::waitForService(move_along_joint_path_,ros::Duration(10.0)) &&
				ros::service::waitForService(search_ik_solution_,ros::Duration(10.0)))
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

			// TODO: Read from telemetry data
			move_along_joint_path_srv_.request.joint_names = lwr_joints; // Select all lwr joints
			move_along_joint_path_srv_.request.path.resize(1);           // Our path has only one waypoint
			// Initialize the velocity and acceleration limits of the joints
			move_along_joint_path_srv_.request.joint_limits.resize(nr_lwr_joints);

			for(unsigned int i = 0; i < nr_lwr_joints; i++){
				move_along_joint_path_srv_.request.joint_limits[i] = joint_limits_[i];
			}

			// current_configuration will hold our current joint position data extracted from the measured telemetry
			current_configuration_.q.resize(nr_lwr_joints);


			// Get the current configuration from the telemetry message
			for(unsigned int i = 0; i < nr_lwr_joints; ++i){
				std::vector<std::string> &joint_names = (_telemetry.joint_names);
				unsigned int telemetry_index = std::find(joint_names.begin(), joint_names.end(), lwr_joints[i]) - joint_names.begin();
				current_configuration_.q[i] = _telemetry.measured.position[telemetry_index];
			}

			if(inter_steps_==0)
			{

				// Select the next desired position of the tcp from the target zone poses and fill
				// the search inverse kinematic solution request with the current configuration as
				// start configuration and the desired position
				search_ik_solution_srv_.request.start = current_configuration_;
				search_ik_solution_srv_.request.tcp_frame = goal_pose_goal_->goal_pose;

				// Call the search inverse kinematic solution service and check for errors
				search_ik_solution_client_.call(search_ik_solution_srv_);
				std::string &search_error_message = search_ik_solution_srv_.response.error_message;
				if(!search_error_message.empty()){
					msg_error("Search IK Solution failed: %s", search_error_message.c_str());
					return false;
				}

				// Extract the solution configuration from the response and fill it into the path of the move request
				euroc_c2_msgs::Configuration &solution_configuration = search_ik_solution_srv_.response.solution;
				std::vector<euroc_c2_msgs::Configuration> &path = move_along_joint_path_srv_.request.path;
				path[0] = solution_configuration;
			}
			else
			{
				get_dk_solution_srv_.request.configuration = current_configuration_;
				get_dk_solution_client_.call(get_dk_solution_srv_);

				// The error_message field of each service response indicates whether an error occured. An empty string indicates success
				std::string &ls_error_message = get_dk_solution_srv_.response.error_message;
				if(!ls_error_message.empty()){
					ROS_ERROR("Get DK failed: %s", ls_error_message.c_str());
					return false;
				}
				else
				{
					geometry_msgs::Pose cur_pose = get_dk_solution_srv_.response.ee_frame;

					double delta_x=goal_pose_goal_->goal_pose.position.x - cur_pose.position.x;
					double delta_y=goal_pose_goal_->goal_pose.position.y - cur_pose.position.y;
					double delta_z=goal_pose_goal_->goal_pose.position.z - cur_pose.position.z;

					std::vector<geometry_msgs::Pose> all_poses;
					all_poses.push_back(cur_pose);

					//necessary to get goal orientation!
					cur_pose.orientation=goal_pose_goal_->goal_pose.orientation;
					for(uint16_t ii=0;ii<inter_steps_;ii++)
					{
						cur_pose.position.x+=(double)(delta_x/inter_steps_);
						cur_pose.position.y+=(double)(delta_y/inter_steps_);
						cur_pose.position.z+=(double)(delta_z/inter_steps_);

						all_poses.push_back(cur_pose);

						ROS_INFO("pose %d: [%4.3f %4.3f %4.3f]",ii,
								 cur_pose.position.x,cur_pose.position.y,cur_pose.position.z);
					}

					for(uint16_t ii=1;ii<all_poses.size();ii++)
					{
						search_ik_solution_srv_.request.start = current_configuration_;
						search_ik_solution_srv_.request.tcp_frame = all_poses[ii];

						// Call the search inverse kinematic solution service and check for errors
						search_ik_solution_client_.call(search_ik_solution_srv_);
						std::string &search_error_message = search_ik_solution_srv_.response.error_message;
						if(!search_error_message.empty()){
							msg_error("Search IK Solution failed: %s", search_error_message.c_str());
							return false;
						}

						std::vector<euroc_c2_msgs::Configuration> &path = move_along_joint_path_srv_.request.path;
						path.push_back(search_ik_solution_srv_.response.solution);
					}
				}
			}

			return true;
		}
		else
			return false;
	}
	catch (...)
	{
		msg_error("MotionPlanning::Error at searchforIK service.");
		return false;
	}
	return true;
}

bool MotionPlanning::setReset7DOF()
{
	try
	{
		if (ros::service::waitForService(move_along_joint_path_,ros::Duration(10.0)))
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

			// TODO: Read from telemetry data
			move_along_joint_path_srv_.request.joint_names = lwr_joints; // Select all lwr joints
			move_along_joint_path_srv_.request.path.resize(1);           // Our path has only one waypoint
			// Initialize the velocity and acceleration limits of the joints
			move_along_joint_path_srv_.request.joint_limits.resize(nr_lwr_joints);
			for(unsigned int i = 0; i < nr_lwr_joints; i++){
				move_along_joint_path_srv_.request.joint_limits[i] = joint_limits_[i];
			}

			// current_configuration will hold our current joint position data extracted from the measured telemetry
			euroc_c2_msgs::Configuration commanded_configuration;
			commanded_configuration.q.resize(nr_lwr_joints);

			// Get the current configuration from the telemetry message
			for(unsigned int i = 0; i < nr_lwr_joints; ++i){
				commanded_configuration.q[i] = 0.0;
			}

			// Extract the solution configuration from the response and fill it into the path of the move request
			std::vector<euroc_c2_msgs::Configuration> &path = move_along_joint_path_srv_.request.path;
			path[0] = commanded_configuration;

			return true;
		}
		else
			return false;
	}
	catch (...)
	{
		msg_error("MotionPlanning::Error at HOMING service.");
		return false;
	}
	return true;
}

void MotionPlanning::getTimingAlongJointPath()
{
	if (ros::service::waitForService(timing_along_joint_path_,ros::Duration(1.0)))
	{
		//TODO expected TIMING with getTimingAlongJointPath
		timing_along_joint_path_srv_.request.joint_names = move_along_joint_path_srv_.request.joint_names;

		int n_joints = timing_along_joint_path_srv_.request.joint_names.size();

		//timing_along_joint_path_srv_.request.start_pose =
		timing_along_joint_path_srv_.request.path = move_along_joint_path_srv_.request.path;
		timing_along_joint_path_srv_.request.joint_limits = move_along_joint_path_srv_.request.joint_limits;

		timing_along_joint_path_srv_.request.start_pose = current_configuration_;

		timing_along_joint_path_client_.call(timing_along_joint_path_srv_);

		time_at_path_points_.resize(timing_along_joint_path_srv_.response.time_at_via_point.size());
		time_at_path_points_ = timing_along_joint_path_srv_.response.time_at_via_point;

		estimated_motion_time_ = 0.0;

		for (int i=0;i<time_at_path_points_.size();i++)
		{
			estimated_motion_time_ += time_at_path_points_[i].toSec();
		}
	}
	else
		msg_warn("Timing service has not been advertised.");
}

bool MotionPlanning::getTelemetry()
{
	_telemetry = *(ros::topic::waitForMessage<euroc_c2_msgs::Telemetry>(telemetry_,ros::Duration(10.0)));
	if (&_telemetry==NULL)
	{
		msg_warn("No telemetry message received.");
		return false;
	}
	else
		return true;
}

void MotionPlanning::moveToTargetCB()
{
	// Call the move request and check for errors
	if(!move_along_joint_path_client_.call(move_along_joint_path_srv_)) //TODO CHange back to original
	{
	  move_along_joint_path_srv_.response.error_message = "Failed to call client to move along joint path";
	        msg_error("Failed to call client to move along joint path.");
	        return;
	}
	// Print out stop reason
	std::string joint = "joint ";
	std::string tool = "tool force";
	std::string ext_torque = "ext_torque ";
	std::string is = "is ";
	std::string StopReason = move_along_joint_path_srv_.response.stop_reason;

	if( (StopReason != "path finished") && (StopReason != "path finished (already at target)") )
	{
		if (StopReason == "")
		{
			msg_error("%s", move_along_joint_path_srv_.response.error_message.c_str());
			return;
		}
		else
		{
			msg_info("Stop Reason: %s", StopReason.c_str());

			if (StopReason.find(joint) != std::string::npos)
			{
				std::string joint_no = StopReason.substr(StopReason.find(joint) + joint.size(), 1);

				//joint_name no is given number plus 1 (in simulation joints start from 0)
				call_set_stop_cond_srv_.request.joint_nbr = std::atoi(joint_no.c_str());

				//testing atof functionality
				std::string string_torque = StopReason.substr(StopReason.find(ext_torque) + ext_torque.size(), StopReason.find(" ")); // until next space
				call_set_stop_cond_srv_.request.current_value = std::atof(string_torque.c_str());
			}
			else if (StopReason.find(tool) != std::string::npos)
			{
				call_set_stop_cond_srv_.request.joint_nbr = 7; // gripper
				std::string string_force = StopReason.substr(StopReason.find(is) + is.size(), StopReason.find(" "));
				call_set_stop_cond_srv_.request.current_value = std::atof(string_force.c_str());
			}
			else
			{
				msg_error("unknown stop reason!");
			}

			msg_error("Stop reason: %s", move_along_joint_path_srv_.response.stop_reason.c_str());
			call_set_stop_cond_srv_.request.level = 1; // level 1 = limits eased
		}

		if(!state_observer_client_.call(call_set_stop_cond_srv_))
		{
			msg_error("Failed to call client to set stop conditions.");
			return;
		}

		//check if max limits reached in stop cond. serv.
		if(call_set_stop_cond_srv_.response.limit_reached)
		{
			goalPose_result_.error_reason = MAX_LIMIT_REACHED;
		}
		else
		{
			goalPose_result_.error_reason = STOP_COND;
		}
	}
	else
	{
		call_set_stop_cond_srv_.request.level = 0;
		if(!state_observer_client_.call(call_set_stop_cond_srv_))
		{
			msg_error("Failed to call client to set stop conditions.");
			return;
		}
	}
	// above by Anna

	std::string &move_error_message = move_along_joint_path_srv_.response.error_message;
	if(!move_error_message.empty()){
		msg_error("Move failed...");
		std::cout << "Move failed: " + move_error_message << std::endl;
	}
	mtt_=FINISHED;
}

bool MotionPlanning::return_poses_valid(am_msgs::CheckPoses::Request &req, am_msgs::CheckPoses::Response &res)
{
	uint16_t nr_poses = req.poses.size();
	ROS_INFO("in check poses service call:");

	for(uint16_t ii=0;ii<nr_poses;ii++)
	{
		try
		{
			if(ros::service::waitForService(search_ik_solution_,ros::Duration(10.0)))
			{
				// current_configuration will hold our current joint position data extracted from the measured telemetry
				current_configuration_.q.resize(7);

				// Select the next desired position of the tcp from the target zone poses and fill
				// the search inverse kinematic solution request with the current configuration as
				// start configuration and the desired position
				search_ik_solution_srv_.request.start = current_configuration_;
				search_ik_solution_srv_.request.tcp_frame = req.poses[ii];

				search_ik_solution_client_.call(search_ik_solution_srv_);
				std::string &search_error_message = search_ik_solution_srv_.response.error_message;
				if(!search_error_message.empty()){
					msg_error("Search IK Solution failed: %s", search_error_message.c_str());

					res.valid=false;
					return false;
				}
			}
		} catch(...) {
			msg_error("failed to find service search ik-solution");
			return false;
		}

	}

	res.valid=true;
	ROS_INFO("finished check poses service call.");

	return true;
}

bool MotionPlanning::getLimits()
{
	//! Setting max velocities, getting from parameter server
	try
	{
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
		msg_error("GET LIMITS aborted. Setting default values.");
		table_axis1_limit_.max_velocity = 0.5;
		table_axis2_limit_.max_velocity = 0.5;
		for (int ii=0;ii<7;ii++)
			joint_limits_[ii].max_velocity = 20 * M_PI / 180.0;
		gripper_limit_.max_velocity = 0.5;
	}

	//! Setting max accelerations
	table_axis1_limit_.max_acceleration = 2.0;
	table_axis2_limit_.max_acceleration = 2.0;
	for (int ii=0;ii<7;ii++)
		joint_limits_[ii].max_acceleration = 400 * M_PI / 180.0;
	gripper_limit_.max_acceleration = 2.0;

	table_axis1_limit_.max_velocity *= (double)speed_percentage_*0.01;
	table_axis2_limit_.max_velocity *= (double)speed_percentage_*0.01;
	gripper_limit_.max_velocity     *= (double)speed_percentage_*0.01;
	for (int ii=0;ii<7;ii++)
		joint_limits_[ii].max_velocity *= (double)speed_percentage_*0.01;


	return true;

}
