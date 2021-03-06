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
#include <tf/transform_broadcaster.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>


class T6MotionPlanning: public T5MotionPlanning {
public:
	T6MotionPlanning();
	virtual ~T6MotionPlanning();

private:
	//!client for gripper-service
	ros::ServiceClient gripper_control_client_;
	am_msgs::GripperControl gripper_control_srv_;
	uint8_t gripper_close_state_;

	ros::Subscriber nr_obj_sub_;
	void get_nr_obj_cb(const std_msgs::Int16::ConstPtr &msg){n_objects_=msg->data;ROS_INFO("got nr_obj: %d",n_objects_);};

	ros::Subscriber obj_mass_sub_;
	void get_obj_mass_cb(const std_msgs::Float64::ConstPtr &msg){obj_mass_=msg->data;ROS_INFO("got obj mass: %f",obj_mass_);};


	bool executeGoalPoseT6();
	bool T6_executeStd();
	bool T6_MoveIt_move_to_object();
	bool T6_grap_object();
	bool T6_MoveIt_move_object_safe();
	bool T6_MoveIt_initializeMoveGroup();
	bool T6_MoveIt_getSolution();
	bool T6_move_2DoF_Euroc();//get movement as a combination of 2DoF + Euroc
	bool T6_MoveIt_getSolution_2DOF();
	bool T6_moveSchlitten();
	bool T6_moveToTarget();
	bool T6_MoveIt_homing();
	bool T6_euroc_getIKSolution7DOF();//kombination mit moveit homing
	bool T6_transformToLWRTCPFrame(const geometry_msgs::Pose tmp_goal_pose_GPTCP, geometry_msgs::Pose &tmp_goal_pose_LWRTCP);

	void T6_initializeConveyorBelt();
	bool T6_open_gripper();


	//task 6
	bool T6_getBeltSpeed();
	bool T6_getNewPose(geometry_msgs::Pose& pose);
	bool T6_calc_goalHoming_TCP(geometry_msgs::Pose& pose);
	bool T6_calc_goalHoming_schlitten(geometry_msgs::Pose &goal_config_homing_T6);

	//! get schlitten position
	bool T6_getSchlittenPosition(std::vector<double> &schlitten_position);
	//! get current GPTCP pose in world frame
	bool T6_getCurrentGPTCP_ORIGIN(geometry_msgs::Pose &current_pose);
	//! get current LWRTCP pose in lwr_0 frame
	bool T6_getCurrentLWRTCP_LWR_0(geometry_msgs::Pose &current_pose);
	//! get current joint position
	bool T6_getJointPosition(const std::string joint_name, double& joint_position);
	//! calc Target position
	bool T6_calcTarget();
	//! get param from parameterserver
	void T6_getParam();
	// helper function
	void T6_zWinkel();
	//! transform goal pose to relativ pose to lwr base
	geometry_msgs::Pose T6_transformLWRBase(const geometry_msgs::Pose goal_config_homing_T6,
											const geometry_msgs::Pose goal_config_homing_T6_schlitten);

	//! helper functions from grasping for homing pose
	tf::Matrix3x3 T6_get_rotationmatrixfromaxis(tf::Vector3 axis, double angle_rad);
	void T6_set_orientation_from_axes(geometry_msgs::Pose &tmp_pose, tf::Vector3 x_axis, tf::Vector3 y_axis, tf::Vector3 z_axis);

	//! compute the target of the LWR base for moving parallel along the belt for grabbing the object
	/*!
	 * if the base lies outside the table, the distance is reduced incrementally
	*/
	bool T6_setTargetForParallelMotion(std::vector<double> tmp_pos, std::vector<double>senk, std::vector<double>par);

	geometry_msgs::Point target_pos;
	geometry_msgs::Point drop_center_point;
	geometry_msgs::Vector3 mdl;
	geometry_msgs::Vector3 mdl_norm;
	geometry_msgs::Vector3 drop_deviation;

	geometry_msgs::Pose save_goal_pose_GPTCP_;
	geometry_msgs::Pose save_goal_pose_LWRTCP_;


	 // dyn homing
	geometry_msgs::Pose goal_target_pos; // ziel position -> jetzt ueber statemachine


	geometry_msgs::Pose gripping_pose_;
	geometry_msgs::Pose over_belt_pose_origin_;
	geometry_msgs::Pose over_belt_pose_transformed_;
	geometry_msgs::Pose homing_belt_schlitten_pose_;
	geometry_msgs::Pose homing_belt_euroc_pose_origin_;
	geometry_msgs::Pose homing_belt_euroc_pose_transformed_;
	geometry_msgs::Pose senkrecht_belt_pose_;


	//total number of objects
	int n_obj_ges_;
	// number of dropped objects
	int n_objects_;
	// obj mass
	double obj_mass_;
	// start speed belt
	float start_speed_;
	// end speed
	float end_speed_;
	// vel of belt belt
	float belt_speed_;

	double home_faktor_;

	double T6_puffer_pose_;
	// time stamp
	double time_stamp_in_;
	// rendez-vous time
	double t_rdv;
	//Winkel um z achse
	double winkel_kl; // zu y-Achse
	double winkel_gr; // zu x_achse
 // tatsächlicher Drehwinkel
	double winkel_roll;
	double winkel_pitch;
	double winkel_yaw;
	//!	radius of targetzone
	double target_zone_radius_;
	//! tolerance for GP_TCP for TargetZone
	double tolerance_7DoF_position_;
	//! changes tolerance_7DoF_position_based on radius of target_zone
	double factor_tolerance_7DoF_position_;
	double tolerance_7DoF_orientation_;
	//! height over target zone
	double tolerance_height_;
	//! security margin for target area
	double target_zone_radius_security_;
	//! distance from robot to target center for schlitten endposition
	double target_distance_;
	//! height over belt for itermediate goal position
	double height_over_belt_;
	double height_over_belt_tuning_;
	//! values for intermediate goal position
	double standard_distance_dcp_schlitten_;//start distance schlitten
	double gradient_distance_dcp_;//gradient of distance
	double standard_distance_dcp_planar_axis_;//standard distance for linear axis

	// zwischen winkel und quart
	tf::Quaternion q_goalHoming;
	tf::Quaternion q_belt;

	//new solution based on grasping node
	tf::Vector3 conveyor_belt_move_direction_and_length;
	double l_belt;
	double gripping_angle_deg_;
	double gripping_angle_rad_;

	//! visualization in rviz
	void T6_send_poses_to_tf_broadcaster(geometry_msgs::Pose input_pose,bool frame);
	tf::Transform T6_pose_to_transform(geometry_msgs::Pose input_pose);
	tf::TransformBroadcaster br;
	double pose_counter;

	double senk_;
	double par_;



};

#endif /* T6MOTIONPLANNING_H_ */
