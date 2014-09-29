/*
 * MotionPlanning.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: euroc_admin
 */

#include "MotionPlanning.h"
#define STANDARD_IK_7DOF 0

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

	move_along_joint_path_client_   = nh_.serviceClient<euroc_c2_msgs::MoveAlongJointPath>(move_along_joint_path_);
	timing_along_joint_path_client_ = nh_.serviceClient<euroc_c2_msgs::GetTimingAlongJointPath>(timing_along_joint_path_);
	search_ik_solution_client_      = nh_.serviceClient<euroc_c2_msgs::SearchIkSolution>(search_ik_solution_);

	goalPose_server_.start();
	msg_info("goalPose action server started.");


	check_poses_service_ = nh_.advertiseService("CheckPoses_srv", &MotionPlanning::return_poses_valid,this);

	feedback_frequency_ = 2;

	mtt_=OPEN;

}

MotionPlanning::~MotionPlanning() {}

void MotionPlanning::executeGoalPose_CB(const am_msgs::goalPoseGoal::ConstPtr &goal)
{
	msg_info("In execution call");

	goal_pose_goal_ = goal;
	speed_percentage_ = goal_pose_goal_->speed_percentage;

	if (speed_percentage_ <= 0 || speed_percentage_ >100)
		speed_percentage_ = 50;

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
			for(unsigned int i = 0; i < nr_lwr_joints; ++i){
				euroc_c2_msgs::Limits &limits = move_along_joint_path_srv_.request.joint_limits[i];
				limits.max_velocity = 20 * M_PI / 180.0; // 20 degrees per second
				limits.max_acceleration = 400 * M_PI / 180.0;
			}

			// current_configuration will hold our current joint position data extracted from the measured telemetry
			current_configuration_.q.resize(nr_lwr_joints);


			// Get the current configuration from the telemetry message
			for(unsigned int i = 0; i < nr_lwr_joints; ++i){
				std::vector<std::string> &joint_names = (_telemetry.joint_names);
				unsigned int telemetry_index = std::find(joint_names.begin(), joint_names.end(), lwr_joints[i]) - joint_names.begin();
				current_configuration_.q[i] = _telemetry.measured.position[telemetry_index];
			}

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
			for(unsigned int i = 0; i < nr_lwr_joints; ++i){
				euroc_c2_msgs::Limits &limits = move_along_joint_path_srv_.request.joint_limits[i];
				limits.max_velocity = 20 * M_PI / 180.0; // 20 degrees per second
				limits.max_acceleration = 400 * M_PI / 180.0;
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
	move_along_joint_path_client_.call(move_along_joint_path_srv_);

	std::string &move_error_message = move_along_joint_path_srv_.response.error_message;
	if(!move_error_message.empty()){
		msg_error("Move failed.");
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
