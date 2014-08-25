/*
 * GraspPose.hpp
 *
 *  Created on: Aug 13, 2014
 *      Author: euroc_student
 */

#ifndef GRASPPOSE_H_
#define GRASPPOSE_H_

//ros includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//am msgs
#include <am_msgs/Object.h>
#include <am_msgs/GetGraspPose.h>

//am includes
#include <tf_rot.hpp>
#include <config.hpp>


class GraspPose {
public:
	// Constructor
	GraspPose();
	// Standard Destructor
	virtual ~GraspPose();
	// ROS-service function
	bool return_grasp_pose(am_msgs::GetGraspPose::Request &req, am_msgs::GetGraspPose::Response &res);
	// Transform for frame of LWR-TCP
	tf::Transform transform_gripper;
	// Transform for frame of object (received in service request)
	tf::Transform transform_object;

private:
	// ------------------------- Class Properties -----------------------------
	// "Object" as defined in ros message
	am_msgs::Object object_;
	// Object height
	double object_height_;
	// Object mass
	double object_mass_;
	// Waiting height of gripper before approaching the object ("towards table")
	double waiting_height_;
	// Safety margin between object and gripper waiting position (waiting_height_ = object_height_ + safety_margin_obj2wait_)
	double safety_margin_obj2wait_;
	// Grasp width: distance between gripper yaws when gripper is closed
	double grasp_width_;
	// Shape positions r_02shape: vectors from origin of 0-frame to origin of shape, given transform into 0-coordinate frame
	std::vector<boost::array<double,3> > r_02shape_0_;
	// Object center of mass
	geometry_msgs::Point object_CoM_;
	// index of shape which is closest to CoM; this will be the shape where the gripper will close
	unsigned int idx_shape_CoM_;
	// Flat copy of object quaternion
	double quat_02obj_[4]; // 1st element is w
	// Rotation matrix corresponding to quat_02obj_
	double A_obj20_[9];
	// shape_height: highest point of each shape of the object (important to compute object height)
	std::vector<double> shape_height_;
	// LWR-TCP target pose
	geometry_msgs::Pose LWRTCP_target_pose_;
	// GP-TCP2 target pose
	geometry_msgs::Pose GPTCP_target_pose_;

	// Auxiliary variable for control outputs; in the constructor, set this variable to FALSE to avoid control outputs at the terminal
	bool ctrl_outp_;
	// z-axis of 0-frame
	double ez_0_0_[3];
	// Height of gripper jaws
	double gripper_height_ ;
	// Desired safety distance between gripper and object/table (needed when checking for collision of gripper with object/table)
	double safety_distance_;

	// ------------------------- Class Methods ----------------------------
	// "set_object_data_()" sets the properties "object_", "quat_02obj", "A_obj20_", and calls method "compute_abs_shape_positions_()"
	void set_object_data_(am_msgs::Object);
	// "compute_abs_shape_positions_()" sets the property "r_02shape_0_"
	void compute_abs_shape_positions_();
	// "compute_object_height_()" sets the properties "object_height_" and "waiting_height_"
	void compute_object_height_();
	// "compute_object_CoM_()" sets the properties "object_CoM_" and "object_mass_"
	void compute_object_CoM_();
	// "compute_idx_shape_CoM_()" sets the property "idx_shape_CoM_"
	void compute_idx_shape_CoM_();
	// "compute_grasp_pose_(prio)" computes the desired gripper pose in the gripper frame
	void compute_grasp_pose_(unsigned int);
	// "transform_grasp_pose_GPTCP_2_LWRTCP_()" returns the desired gripper pose transformed into the LWR-TCP frame
	geometry_msgs::Pose transform_grasp_pose_GPTCP_2_LWRTCP_();
	// "get_base_vectors_" extracts the base vectors from a rotation matrix and returns them in a vecotr list
	std::vector<boost::array<double,3> > get_base_vectors_( double*, bool );
	// "cross_product_(a,b,c)" computes the vector cross product a x b = c (c is the variable to be set)
	void cross_product_( double*, double*, double* );
	// "matrix_multiply_(A,x,b)" computes A*x = b (b is the variable to be set)
	void matrix_multiply_( double*, double*, double* );
	// "matrix_transpose_(A,At)" computes At = A' (At is the variable to be set)
	void matrix_transpose_( double*, double* );
	// "get_grasp_pose_(prio)" calls the method "compute_grasp_pose_(prio)"
	geometry_msgs::Pose get_grasp_pose_(unsigned int);
};

#endif /* GRASPPOSE_H_ */
