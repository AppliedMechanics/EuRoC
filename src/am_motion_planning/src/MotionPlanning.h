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
#include <string>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>


// EUROC
#include <euroc_c2_msgs/MoveAlongJointPath.h>
#include <euroc_c2_msgs/GetTimingAlongJointPath.h>
#include <euroc_c2_msgs/SearchIkSolution.h>
#include <euroc_c2_msgs/GetForwardKinematics.h>

#include <euroc_c2_msgs/Telemetry.h>
#include <am_msgs/CallSetStopConditions.h>

#include <boost/thread.hpp>

// OCTOMAP
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <octomap_server/OctomapServer.h>


#define MOVEIT
//#undef MOVEIT

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

// MOVEIT
#ifdef MOVEIT

	std::vector<std::string> ompl_planners;

	move_group_interface::MoveGroup *group_7DOF;
	move_group_interface::MoveGroup *group_9DOF;
	move_group_interface::MoveGroup *group;

	robot_model_loader::RobotModelLoader robot_model_loader_;
	robot_model::RobotModelPtr kinematic_model_;

	robot_state::RobotStatePtr kinematic_state_;

	const robot_state::JointModelGroup* joint_model_group_7DOF_;
	const robot_state::JointModelGroup* joint_model_group_9DOF_;
	const robot_state::JointModelGroup* joint_model_group_;

	planning_scene_monitor::PlanningSceneMonitor* planning_scene_monitor;


	unsigned max_setTarget_attempts_;
	moveit::planning_interface::MoveGroup::Plan motion_plan_;

	std::vector<euroc_c2_msgs::Configuration> planned_path_;

	bool getMoveItSolution();
	void setPlanningConstraints();
	bool setPlanningTarget(unsigned algorithm);
	bool valid_kdl_ik(geometry_msgs::Pose& pose, short unsigned int& priority);
	moveit::planning_interface::PlanningSceneInterface PlanInterOcto;


	ros::ServiceServer attach_object_service_;
	// ROS-service function to attach a gripped object
	bool return_object_attached(am_msgs::AttachObject::Request &req, am_msgs::AttachObject::Response &res);

	ros::ServiceServer detach_object_service_;
	// ROS_service function to detach a release object
	bool return_object_detached(am_msgs::DetachObject::Request &req, am_msgs::DetachObject::Response &res);

	// OCTOMAP
	// Topic
	ros::Subscriber octomap_subscriber_;
	std::string octomap2_;
	octomap_msgs::Octomap _octomap;
	// Service
	ros::Publisher octo_pub;
	ros::ServiceClient octomap_client_;
	std::string octomap_;
	octomap_msgs::GetOctomap octomap_srv_;
	octomap_msgs::Octomap _octree;

	octomap_msgs::Octomap bmap_msg;

	moveit_msgs::PlanningScene planning_scene_octo;


	bool getOctomap();

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

	int active_task_nr_;
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

	bool valid_euroc_ik(geometry_msgs::Pose& pose, short unsigned int& priority);


};

#endif /* MOTIONPLANNING_H_ */
