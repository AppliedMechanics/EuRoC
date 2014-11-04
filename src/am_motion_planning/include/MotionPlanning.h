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
#include <string>
#include <boost/thread.hpp>
#include <std_msgs/Bool.h>

// AM
#include <actionlib/server/simple_action_server.h>
#include <am_msgs/goalPoseAction.h>
#include <am_msgs/CheckPoses.h>
#include <am_msgs/ObjState.h>
#include <am_msgs/CallSetStopConditions.h>
#include <config.hpp>
#include <fsm_state.hpp>
#include <utils.hpp>

// MOVEIT
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/Constraints.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection/collision_robot.h>	// probably required for self collision checking
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include <moveit/move_group/move_group_capability.h>




// GEOMETRY
#include <geometry_msgs/Vector3.h>
#include <shape_msgs/SolidPrimitive.h>

// EUROC
#include <euroc_c2_msgs/MoveAlongJointPath.h>
#include <euroc_c2_msgs/GetTimingAlongJointPath.h>
#include <euroc_c2_msgs/SearchIkSolution.h>
#include <euroc_c2_msgs/GetForwardKinematics.h>
#include <euroc_c2_msgs/Telemetry.h>

// OCTOMAP
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_ros/conversions.h>
#include <octomap_server/OctomapServer.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/BoundingBoxQuery.h>






struct ObjectInformation
{
	int nr_shapes;
	std::vector<int> shape_types;
	std::vector<geometry_msgs::Pose> shape_poses;
	std::vector<geometry_msgs::Vector3> shape_sizes;
	std::vector<double> shape_lengths;
	std::vector<double> shape_radii;
	geometry_msgs::Pose obj_pose;
};

struct TargetZoneInformation
{
	int obj_nr;
	double x;
	double y;
	double r;
};





class MotionPlanning {
public:
	MotionPlanning();
	virtual ~MotionPlanning();

protected:
	ros::NodeHandle nh_;

	//! Actionlib related functions
	//! Actionlib Action Server for p2pAction
	actionlib::SimpleActionServer<am_msgs::goalPoseAction> goalPose_server_;
	std::string goalPose_action_name_;

	// create messages that are used to published feedback/result
	am_msgs::goalPoseFeedback goalPose_feedback_;
	am_msgs::goalPoseResult   goalPose_result_;

	// object state message subscriber
	ros::Subscriber obj_state_sub_;

	void object_manager_get_object_state_cb(const am_msgs::ObjState::ConstPtr& msg);
	std::vector<am_msgs::ObjState> obj_state_;
	bool obj_data_loaded_;

	//! This function is executed, when a new goal goalPoseAction is received
	void executeGoalPose_CB(const am_msgs::goalPoseGoal::ConstPtr &goal);
	void getGoalPose_Feedback();

	virtual void executeGoalPoseT5(){msg_error("T5 call in MotioPlanning class. Did you set up the T5MotionPlanning class?");};
	virtual void executeGoalPoseT6(){msg_error("T6 call in MotioPlanning class. Did you set up the T6MotionPlanning class?");};

	void moveToTargetCB();

	ros::ServiceServer check_poses_service_;

	// ROS-service function to check ik solution for poses
	bool return_poses_valid(am_msgs::CheckPoses::Request &req, am_msgs::CheckPoses::Response &res);

	//! Services
	ros::ServiceClient move_along_joint_path_client_;
	ros::ServiceClient timing_along_joint_path_client_;
	ros::ServiceClient search_ik_solution_client_;
	ros::ServiceClient get_dk_solution_client_;
	ros::ServiceClient state_observer_client_;


	//! Subscribers
	ros::Subscriber telemetry_subscriber_;

	ros::Timer feedback_timer_;

	//! Strings
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


	am_msgs::CallSetStopConditions call_set_stop_cond_srv_;
	am_msgs::goalPoseGoal::ConstPtr goal_pose_goal_;


	geometry_msgs::Pose goal_pose_GPTCP_;
	geometry_msgs::Pose goal_pose_LWRTCP_;




	// MOVEIT
	std::vector<std::string> ompl_planners;

	move_group_interface::MoveGroup *group_2DOF;
	move_group_interface::MoveGroup *group_7DOF;
	move_group_interface::MoveGroup *group_9DOF;
	move_group_interface::MoveGroup *group;

	robot_model_loader::RobotModelLoader robot_model_loader_;
	robot_model::RobotModelPtr kinematic_model_;

	robot_state::RobotState* kinematic_state_;

	const robot_state::JointModelGroup* joint_model_group_2DOF_;
	const robot_state::JointModelGroup* joint_model_group_7DOF_;
	const robot_state::JointModelGroup* joint_model_group_9DOF_;
	const robot_state::JointModelGroup* joint_model_group_;

	planning_scene_monitor::PlanningSceneMonitor* planning_scene_monitor;


	unsigned max_setTarget_attempts_;
	moveit::planning_interface::MoveGroup::Plan motion_plan_;

	std::vector<euroc_c2_msgs::Configuration> planned_path_;


	moveit_msgs::PlanningScene planning_scene_;
	std::vector<moveit_msgs::CollisionObject> collision_objects_;




	bool MoveIt_homing();
	bool MoveIt_getSolution();
	//! get straight line in taskspace
	bool MoveIt_getSolutionInTaskSpace();
	//bool MoveIt_getSolution2();
	bool MoveIt_getSolutionNoInitialize();
	bool MoveIt_initializeMoveGroup();
	bool MoveIt_valid_kdl_ik(geometry_msgs::Pose& pose, short unsigned int& priority);


	bool euroc_getIKSolution7DOF();
	bool euroc_setReset7DOF();
	bool euroc_valid_euroc_ik(geometry_msgs::Pose& pose, short unsigned int& priority);

	bool octomap_manager_getOctomap();
	bool octomap_manager_cleanupOctomap();

	//! compute Waypoints for tcp to follow an straight line
	bool computeWayPoints(std::vector< geometry_msgs::Pose > &waypoints_);
	bool setPlanningTarget(unsigned algorithm);
	bool getTelemetry();
	bool getLimits();

	bool transformToTCPFrame(std::string frame);
	bool transformToLWRBase();

	bool object_manager_objectExists(int obj_index);
	void object_manager_createObject(int obj_index, geometry_msgs::Pose obj_pose);
	void object_manager_readObjectDataFromParamServer(int obj_index, ObjectInformation& obj_info);
	void object_manager_addObjectToWorld(int obj_index);
	void object_manager_removeObjectFromWorld(int obj_index);
	void object_manager_attachObject(int obj_index);
	void object_manager_detachObject(int obj_index);
	void object_manager_addObjectToTargetZone(int obj_index);

	void setShapePositions(int obj_index, geometry_msgs::Pose obj_pose);//wird zur ZEit nicht verwendet
	void initializePlanningScene();

	double getTargetObjectHeight(int obj_index);
	double getTargetObjectRadius(int obj_index);
	geometry_msgs::Pose getTargetObjectPose(int obj_index);
	void readTargetZoneDataFromParamServer(int obj_index, TargetZoneInformation& tz_info);
	sensor_msgs::JointState getCurrentJointState();




	void getTimingAlongJointPath();
	euroc_c2_msgs::Configuration getCurrentConfiguration();
	void setMoveRequestJointLimits();
	void setMoveRequestTCPLimits();



	unsigned current_setTarget_algorithm_;


	//! OCTOMAP
	// used for publishing planning scene
	ros::Publisher planning_scene_diff_publisher_;
	// used for calling service
	ros::ServiceClient octomap_client_;
	// used for storing path
	std::string octomap_;
	// used for storing data received from service
	octomap_msgs::GetOctomap octomap_srv_;
	// used for storing octomap data
	octomap_msgs::Octomap _octree;
	// used to manually load binary octomap.bt
	octomap::OcTree octree_file;
	// Static scene for pantilt + table
	moveit_msgs::PlanningScene static_scene_;

	//!client to remove area in octomap
	ros::ServiceClient cleanup_octomap_client_;
	octomap_msgs::BoundingBoxQuery cleanup_octomap_srv_;

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
	bool skip_vision_;
	int active_task_nr_;
	uint32_t speed_percentage_;
	uint32_t inter_steps_;
	std::string planning_frame_;
	boost::thread moveToTarget;
	uint8_t mtt_;



};

#endif /* MOTIONPLANNING_H_ */
