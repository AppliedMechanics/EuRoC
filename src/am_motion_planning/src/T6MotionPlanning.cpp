/*
 * T6MotionPlanning.cpp
 *
 *  Created on: Nov 3, 2014
 *      Author: euroc_admin
 */

#include <T6MotionPlanning.h>

T6MotionPlanning::T6MotionPlanning():
MotionPlanning::MotionPlanning(),
T6_goalPose_server_(nh_, "T6goalPoseAction", boost::bind(&T6MotionPlanning::executeGoalPose_CB, this, _1),false)
{
	// TODO Auto-generated constructor stub

}

T6MotionPlanning::~T6MotionPlanning() {
	// TODO Auto-generated destructor stub
}

void T6MotionPlanning::executeGoalPose_CB(const am_msgs::goalPoseGoal::ConstPtr &goal)
{

	ros::param::get("/skip_vision", skip_vision_);

	goal_pose_goal_ = goal;
	speed_percentage_ = goal_pose_goal_->speed_percentage;
	inter_steps_ = goal_pose_goal_->inter_steps;
	planning_frame_ = goal_pose_goal_->planning_frame;
	ROS_WARN("Chosen Planning Frame :%s",planning_frame_.c_str());

	//--------------------------------------------------------------------------------------
	//! get kinematic limits
	getLimits();


	//--------------------------------------------------------------------------------------
	//! set Planning Frame
	//! If planning frame is given in GP_TCP frame, the LWR_TCP frame is calculated
	//! If planning frame is chosen as LWR TCP, the GP TCP Pose needs to be calculated
	if (!planning_frame_.compare(LWR_TCP) || !planning_frame_.compare(GP_TCP)){
		if (!transformToTCPFrame(planning_frame_)){
			msg_error("Transformation to LWR frame not successful.");

			goalPose_result_.reached_goal = false;
			goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
			goalPose_server_.setPreempted(goalPose_result_,"Got no telemetry.");

			return;
		}
	}
	else if (goal_pose_goal_->planning_algorithm==MOVE_IT_2DOF){
		goal_pose_GPTCP_  = goal_pose_goal_->goal_pose;
		goal_pose_LWRTCP_ = goal_pose_goal_->goal_pose;
	}
	else if (goal_pose_goal_->planning_algorithm!=HOMING_7DOF)
		msg_error("Planning frame not properly defined.");

	//--------------------------------------------------------------------------------------
	//! get telemetry
	if (!getTelemetry()){
		msg_error("getTelemetry: An Error happened here.");
		goalPose_result_.reached_goal = false;
		goalPose_result_.error_reason = fsm::SIM_SRV_NA;
		goalPose_server_.setPreempted(goalPose_result_,"Got no telemetry.");

		return;
	}

	//--------------------------------------------------------------------------------------
	//! Plan
	switch (goal->planning_algorithm)
	{

	case HOMING_7DOF:
		ROS_INFO("HOMING 7DOF planning mode chosen.");
		if (!euroc_setReset7DOF())
		{
			msg_error("No IK Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No IK Solution found.");
			return;
		}
		break;

	case HOMING_MOVE_IT_2DOF:
		ROS_INFO("HOMING MOVEIT 2DOF planning mode chosen. - Goal tolerance = 0.5!");
		group = group_2DOF;
		joint_model_group_ = joint_model_group_2DOF_;
		if (!MoveIt_homing())
		{
			msg_error("No Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No Solution found.");
			return;
		}
		break;

	case HOMING_MOVE_IT_7DOF:
		ROS_INFO("HOMING MOVEIT 7DOF planning mode chosen.");
		group = group_7DOF;
		joint_model_group_ = joint_model_group_7DOF_;
		if (!MoveIt_homing())
		{
			msg_error("No Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No Solution found.");
			return;
		}
		break;

	case HOMING_MOVE_IT_9DOF:
		ROS_INFO("HOMING MOVEIT 9DOF planning mode chosen.");
		group = group_9DOF;
		joint_model_group_ = joint_model_group_9DOF_;
		if (!MoveIt_homing())
		{
			msg_error("No Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No Solution found.");
			return;
		}
		break;

	case STANDARD_IK_7DOF:

		ROS_INFO("STANDARD IK 7DOF planning mode chosen.");

		//! Transform goal pose to LWR0 Base frame
		if (!transformToLWRBase())
		{
			msg_warn("Transformation to LWR0 Base failed.");

			//			goalPose_result_.reached_goal = false;
			//			goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
			//			goalPose_server_.setPreempted(goalPose_result_,"Transformation to LWR0 Base failed.");
			//			return;
		}
		//! Find IK solution
		if (!euroc_getIKSolution7DOF())
		{
			msg_error("No IK Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No IK Solution found.");
			return;
		}
		break;
		//------------------------------------------------------------------------------------------------
	case (MOVE_IT_2DOF)		:
	case (MOVE_IT_7DOF)		:
	case (MOVE_IT_9DOF)		:
	ROS_WARN("Planning mode based on MoveIt! chosen.");

	//define groups
	if(goal->planning_algorithm == MOVE_IT_9DOF)
	{
		ROS_INFO("Choosed 9DOF");
		group = group_9DOF;
		joint_model_group_ = joint_model_group_9DOF_;
		// setting joint state target via the searchIKSolution srv is not considered
		max_setTarget_attempts_ = 4;

	}
	else if(goal->planning_algorithm == MOVE_IT_7DOF)
	{
		ROS_INFO("Choosed 7DOF");
		group = group_7DOF;
		joint_model_group_ = joint_model_group_7DOF_;
		// in case of unsuccessful planning,
		// the planning target is set as a joint state goal via the searchIKSolution srv
		max_setTarget_attempts_ = 5;

	}
	else if(goal->planning_algorithm == MOVE_IT_2DOF)
	{
		ROS_INFO("Choosed 2DOF");
		group = group_2DOF;
		joint_model_group_ = joint_model_group_2DOF_;

		// in case of unsuccessful planning,
		// the planning target is set as a joint state goal via the searchIKSolution srv
		max_setTarget_attempts_ = 5;

	}
	else
	{
		msg_error("Unknown move group name.");
		goalPose_result_.reached_goal = false;
		goalPose_server_.setPreempted(goalPose_result_,"Unknown move group name.");
		break;
	}

	//set target algorithm
	current_setTarget_algorithm_ = SINGLE_POSE_TARGET;
	// get moveit solution
	if (!MoveIt_getSolution())
	{
		msg_error("No MoveIT Solution found.");
		goalPose_result_.reached_goal = false;
		goalPose_server_.setPreempted(goalPose_result_,"No MoveIT Solution found.");
		return;
	}
	break;

	//------------------------------------------------------------------------------------------------
	case (MOVE_IT_7DOF_MOVE_TO_OBJECT)		:
	case (MOVE_IT_9DOF_MOVE_TO_OBJECT)		:
	{
		ROS_WARN("Planning mode based on MoveIt! chosen.");

		//define groups
		if(goal->planning_algorithm == MOVE_IT_9DOF_MOVE_TO_OBJECT)
		{
			ROS_INFO("Choosed 9DOF");
			group = group_9DOF;
			joint_model_group_ = joint_model_group_9DOF_;

			// setting joint state target via the searchIKSolution srv is not considered
			max_setTarget_attempts_ = 4;

		}
		else if(goal->planning_algorithm == MOVE_IT_7DOF_MOVE_TO_OBJECT)
		{
			ROS_INFO("Choosed 7DOF");
			group = group_7DOF;
			joint_model_group_ = joint_model_group_7DOF_;

			ROS_INFO_STREAM(group->getEndEffectorLink());


			// in case of unsuccessful planning,
			// the planning target is set as a joint state goal via the searchIKSolution srv
			max_setTarget_attempts_ = 5;

		}
		else
		{
			msg_error("Unknown move group name.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"Unknown move group name.");
			break;
		}


		//set target algorithm
		current_setTarget_algorithm_ = SINGLE_POSE_TARGET;

		//to save goal pose
		geometry_msgs::Pose goal_pose_GPTCP_old_ = goal_pose_GPTCP_;

		std::vector< geometry_msgs::Pose > waypoints_;
		//compute waypoints
		computeWayPoints(waypoints_);

		//initialize move group with current state
		MoveIt_initializeMoveGroup();

		planned_path_.clear();

		for(uint64_t ii = 0; ii < waypoints_.size(); ii++)
		{
			goal_pose_GPTCP_ = waypoints_.at(ii);

			// get moveit solution
			if (!MoveIt_getSolutionNoInitialize())
			{
				msg_error("No MoveIT Solution found for waypoint %d.",ii);
				goalPose_result_.reached_goal = false;
				goalPose_server_.setPreempted(goalPose_result_,"No MoveIT Solution found.");

				if(ii == waypoints_.size()-1)
				{
					msg_error("No MoveIT Solution found goal pose",ii);
					goalPose_result_.reached_goal = false;
					goalPose_server_.setPreempted(goalPose_result_,"No MoveIT Solution found.");
					return;

				}
			}

			// set start state to last waypoint
			if(planned_path_.empty() != true)
			{
				moveit_msgs::RobotState start_state;
				start_state.joint_state.name = group->getActiveJoints();
				start_state.joint_state.position = planned_path_.back().q;
				start_state.joint_state.velocity.resize(group->getActiveJoints().size());
				for(uint64_t jj = 0; jj < start_state.joint_state.velocity.size(); jj++)
					start_state.joint_state.velocity.at(jj) = 0.0;

				group->setStartState(start_state);
			}

		}
		goal_pose_GPTCP_ = goal_pose_GPTCP_old_;

		// fill move along joint path
		move_along_joint_path_srv_.request.joint_names = group->getActiveJoints();

		move_along_joint_path_srv_.request.path.resize(planned_path_.size()-1);
		for (unsigned idx = 0; idx < move_along_joint_path_srv_.request.path.size(); ++idx)
		{
			move_along_joint_path_srv_.request.path[idx] = planned_path_[idx+1];
		}

		// set the joint limits (velocities/accelerations) of the move along joint path service request
		setMoveRequestJointLimits();
		// set the TCP limits of the move along joint path service
		setMoveRequestTCPLimits();


		break;
	}
	//------------------------------------------------------------------------------------------------
	case (MOVE_IT_JT_9DOF):
					ROS_WARN("Given JT based on MoveIt! chosen.");
	current_setTarget_algorithm_ = JOINT_VALUE_TARGET_9DOF;
	group = group_9DOF;
	joint_model_group_ = joint_model_group_9DOF_;
	// setting joint state target via the searchIKSolution srv is not considered
	max_setTarget_attempts_ = 3;
	if (!MoveIt_getSolution())
	{
		msg_error("No MoveIT Solution found.");
		goalPose_result_.reached_goal = false;
		goalPose_server_.setPreempted(goalPose_result_,"No MoveIT Solution found.");
		return;
	}
	break;

	default:
		msg_warn("unkown Mode in MotionPlanning!");
		return;
	}
	//------------------------------------------------------------------------------------------------
	//! get timing along path
	getTimingAlongJointPath();
	//------------------------------------------------------------------------------------------------

	//! Feedback
	ros::Rate feedback_rate(feedback_frequency_);

	starting_time_ = ros::Time::now().toSec();
	getGoalPose_Feedback();

	goalPose_server_.publishFeedback(goalPose_feedback_);

	//------------------------------------------------------------------------------------------------
	ROS_WARN("Estimated Motion Time: %f",estimated_motion_time_);
//	if (estimated_motion_time_<goal_pose_goal_->allowed_time || goal_pose_goal_->allowed_time<1)
	moveToTarget = boost::thread(&T6MotionPlanning::moveToTargetCB,this);


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



}
