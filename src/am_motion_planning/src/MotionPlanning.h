/*
 * MotionPlanning.h
 *
 *  Created on: Jul 28, 2014
 *      Author: euroc_admin
 */

#ifndef MOTIONPLANNING_H_
#define MOTIONPLANNING_H_

#include <ros/ros.h>

// AM
#include <actionlib/server/simple_action_server.h>
#include <am_msgs/goalPoseAction.h>

// EUROC
#include <euroc_c2_msgs/MoveAlongJointPath.h>
#include <euroc_c2_msgs/GetTimingAlongJointPath.h>
#include <euroc_c2_msgs/SearchIkSolution.h>

#include <euroc_c2_msgs/Telemetry.h>

#include <boost/thread.hpp>


class MotionPlanning {
public:
	MotionPlanning();
	virtual ~MotionPlanning();
	void while_motion(const ros::TimerEvent& e);

private:
	ros::NodeHandle nh_;

	//! Actionlib related functions
	//! Actionlib Action Server for p2pAction
	actionlib::SimpleActionServer<am_msgs::goalPoseAction> goalPose_server_;
	std::string goalPose_action_name_;

	// create messages that are used to published feedback/result
	am_msgs::goalPoseFeedback goalPose_feedback_;
	am_msgs::goalPoseResult   goalPose_result_;

	//! This function is executed, when a new goal goalPoseAction is received
	void executeGoalPose_CB(const am_msgs::goalPoseGoal::ConstPtr &goal);
	void getGoalPose_Feedback();

	//! Services
	ros::ServiceClient move_along_joint_path_client_;
	ros::ServiceClient timing_along_joint_path_client_;
	ros::ServiceClient search_ik_solution_client_;

	ros::Subscriber telemetry_subscriber_;

	ros::Timer feedback_timer_;

	std::string euroc_c2_interface_;
	std::string move_along_joint_path_;
	std::string timing_along_joint_path_;
	std::string search_ik_solution_;
	std::string telemetry_;

	euroc_c2_msgs::SearchIkSolution search_ik_solution_srv_;
	euroc_c2_msgs::MoveAlongJointPath move_along_joint_path_srv_;
	euroc_c2_msgs::GetTimingAlongJointPath timing_along_joint_path_srv_;

	euroc_c2_msgs::Configuration commanded_configuration_;
	euroc_c2_msgs::Configuration current_configuration_;

	euroc_c2_msgs::Telemetry _telemetry;

	am_msgs::goalPoseGoal::ConstPtr goal_pose_goal_;

	//! Variables
	std::vector<ros::Time> time_at_path_points_;
	double estimated_motion_time_;
	double starting_time_;

	double feedback_frequency_;

	bool getIKSolution7DOF();
	void getTimingAlongJointPath();
	bool getTelemetry();

	boost::thread moveToTarget;
	void moveToTargetCB();


};

#endif /* MOTIONPLANNING_H_ */
