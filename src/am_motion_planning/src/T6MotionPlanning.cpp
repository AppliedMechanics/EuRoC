/*
 * T6MotionPlanning.cpp
 *
 *  Created on: Nov 3, 2014
 *      Author: euroc_admin
 */

#include <T6MotionPlanning.h>

T6MotionPlanning::T6MotionPlanning():
MotionPlanning::MotionPlanning(),
T6_goalPose_server_(nh_, "T6goalPoseAction", boost::bind(&T6MotionPlanning::T6executeGoalPose_CB, this, _1),false)
{
	// TODO Auto-generated constructor stub
	T6_goalPose_server_.start();
}

T6MotionPlanning::~T6MotionPlanning() {
	// TODO Auto-generated destructor stub
}

void T6MotionPlanning::T6executeGoalPose_CB(const am_msgs::goalPoseGoal::ConstPtr &goal)
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
	case MOVE_T6:
		ROS_INFO("MotionPlanning: MOVE_T6 choosen");

		//fill code

		break;
	case HOMING_T6:
		ROS_INFO("MotionPlanning: HOMING_T6 choosen");

		// fill code

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
