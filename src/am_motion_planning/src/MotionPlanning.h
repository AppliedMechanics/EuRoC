/*
 * MotionPlanning.h
 *
 *  Created on: Jul 28, 2014
 *      Author: euroc_admin
 */

#ifndef MOTIONPLANNING_H_
#define MOTIONPLANNING_H_
#include <cstdlib>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>

// AM
#include <actionlib/server/simple_action_server.h>
#include <am_msgs/goalPoseAction.h>
#include <am_msgs/CheckPoses.h>
#include <am_msgs/AttachObject.h>
#include <am_msgs/DetachObject.h>
#include <config.hpp>
#include <utils.hpp>

// MOVEIT
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

//#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/RobotState.h>

// EUROC
#include <euroc_c2_msgs/MoveAlongJointPath.h>
#include <euroc_c2_msgs/GetTimingAlongJointPath.h>
#include <euroc_c2_msgs/SearchIkSolution.h>
#include <euroc_c2_msgs/GetForwardKinematics.h>

#include <euroc_c2_msgs/Telemetry.h>
#include <am_msgs/CallSetStopConditions.h>
#include <am_msgs/GetStaticTFData.h>

#include <boost/thread.hpp>


#define MOVEIT


class MotionPlanning {
public:
	MotionPlanning();
	virtual ~MotionPlanning();

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


	ros::ServiceServer check_poses_service_;
	// ROS-service function to check ik solution for poses
	bool return_poses_valid(am_msgs::CheckPoses::Request &req, am_msgs::CheckPoses::Response &res);

	//! Services
	ros::ServiceClient move_along_joint_path_client_;
	ros::ServiceClient timing_along_joint_path_client_;
	ros::ServiceClient search_ik_solution_client_;
	ros::ServiceClient get_dk_solution_client_;
	ros::ServiceClient state_observer_client_;
	ros::ServiceClient get_static_tf_data_client_;

	ros::Subscriber telemetry_subscriber_;

	ros::Timer feedback_timer_;

	std::string euroc_c2_interface_;
	std::string move_along_joint_path_;
	std::string timing_along_joint_path_;
	std::string search_ik_solution_;
	std::string telemetry_;
	std::string get_dk_solution_;

	euroc_c2_msgs::SearchIkSolution search_ik_solution_srv_;
	euroc_c2_msgs::MoveAlongJointPath move_along_joint_path_srv_;
	euroc_c2_msgs::GetTimingAlongJointPath timing_along_joint_path_srv_;
	euroc_c2_msgs::GetForwardKinematics get_dk_solution_srv_;

	euroc_c2_msgs::Configuration commanded_configuration_;
	euroc_c2_msgs::Configuration current_configuration_;

	euroc_c2_msgs::Telemetry _telemetry;

	am_msgs::goalPoseGoal::ConstPtr goal_pose_goal_;
	geometry_msgs::Pose goal_pose_GPTCP_;
	geometry_msgs::Pose goal_pose_LWRTCP_;


	am_msgs::CallSetStopConditions call_set_stop_cond_srv_;
	am_msgs::GetStaticTFData get_static_tf_data_srv_;


// MOVEIT
#ifdef MOVEIT

	move_group_interface::MoveGroup *group_7DOF;
	move_group_interface::MoveGroup *group_9DOF;
	move_group_interface::MoveGroup *group;

	planning_scene_monitor::PlanningSceneMonitor *planning_scene_monitor;

	std::vector<euroc_c2_msgs::Configuration> planned_path_;

	bool getMoveItSolution();
	void setPlanningConstraints();
	bool setPlanningTarget(planning_target_type_t target_type);


	ros::ServiceServer attach_object_service_;
	// ROS-service function to attach a gripped object
	bool return_object_attached(am_msgs::AttachObject::Request &req, am_msgs::AttachObject::Response &res);

	ros::ServiceServer detach_object_service_;
	// ROS_service function to detach a release object
	bool return_object_detached(am_msgs::DetachObject::Request &req, am_msgs::DetachObject::Response &res);

#endif



	std::vector<euroc_c2_msgs::Limits> joint_limits_;
	euroc_c2_msgs::Limits gripper_limit_;
	euroc_c2_msgs::Limits table_axis1_limit_;
	euroc_c2_msgs::Limits table_axis2_limit_;
	//! Variables
	std::vector<ros::Time> time_at_path_points_;
	double estimated_motion_time_;
	double starting_time_;

	double feedback_frequency_;
	bool called;  // Keeps track of whether set_stop_conditions service has been called and evaluates as true after first call has been made

	uint32_t speed_percentage_;
	uint32_t inter_steps_;
	std::string planning_frame_;

	bool getIKSolution7DOF();
	void getTimingAlongJointPath();
	bool getTelemetry();
	bool getLimits();
	bool setReset7DOF();
	bool transformToTCPFrame(std::string frame);
	bool transformToLWRBase();

	void setMoveRequestJointLimits();
	void setMoveRequestTCPLimits();

	void moveToTargetCB();
	boost::thread moveToTarget;
	uint8_t mtt_;


};

#endif /* MOTIONPLANNING_H_ */
