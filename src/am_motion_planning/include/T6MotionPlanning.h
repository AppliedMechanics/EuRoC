/*
 * T6MotionPlanning.h
 *
 *  Created on: Nov 3, 2014
 *      Author: euroc_admin
 */

#ifndef T6MOTIONPLANNING_H_
#define T6MOTIONPLANNING_H_

#include <T5MotionPlanning.h>

#include <am_msgs/GripperControl.h>

class T6MotionPlanning: public T5MotionPlanning {
public:
	T6MotionPlanning();
	virtual ~T6MotionPlanning();

private:
	//!client for gripper-service
	ros::ServiceClient gripper_control_client_;
	am_msgs::GripperControl gripper_control_srv_;
	uint8_t gripper_close_state_;

	bool executeGoalPoseT6();
	bool T6_MoveIt_move_to_object();
	bool T6_grap_object();
	bool T6_MoveIt_move_object_safe();

	// moveit functions for task 6 -> without octomap
	bool T6_MoveIt_initializeMoveGroup();
	bool T6_MoveIt_getSolution();

	void initializeConveyorBelt();


	//task 6
	bool T6_getBeltSpeed();
	bool T6_getNewPose(geometry_msgs::Pose& pose);
	bool T6_dynHoming();
	bool T6_calc_goalHoming_TCP();
	bool T6_calc_goalHoming_schlitten();
	bool T6_calcTarget();
	bool T6_belt_up(geometry_msgs::Pose& pose);
	//! get param from parameterserver
	void getParam();

	geometry_msgs::Pose goal_config_homing_T6;  // dyn homing
	geometry_msgs::Pose goal_config_homing_T6_schlitten; // dyn homing
	geometry_msgs::Pose goal_target_pos; // ziel position
	geometry_msgs::Point target_pos;
	geometry_msgs::Point drop_center_point;
	geometry_msgs::Vector3 mdl;
	geometry_msgs::Vector3 mdl_norm;
	geometry_msgs::Vector3 drop_deviation;

	//total number of objects
	int n_obj_ges_;
	// number of dropped objects
	int n_objects_;
	// start speed belt
	float start_speed_;
	// end speed
	float end_speed_;
	// vel of belt belt
	float belt_speed_;

	double T6_puffer_pose;
	// time stamp
	double time_stamp_in_;
	// rendez-vous time
	double t_rdv;
	//Winkel um z achse
	double winkel;
	//!	radius of targetzone
	double target_zone_radius_;
	//! target zone tolerance
	double tolerance_;
	//! height over target zone
	double tolerance_height_;
	//! height over belt for itermediate goal position
	double height_over_belt_;
	//! values for intermediate goal position
	double standard_distance_dcp_;//start distance
	double standard_distance_dcp_schlitten_;//start distance schlitten
	double gradient_distance_dcp_;//gradient of distance
	double standard_distance_dcp_planar_axis_;//standard distance for linear axis

	// zwischen winkel und quart
	tf::Quaternion q_tmp;




};

#endif /* T6MOTIONPLANNING_H_ */
