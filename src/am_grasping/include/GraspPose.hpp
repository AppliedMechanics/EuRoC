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
	GraspPose();
	virtual ~GraspPose();
	bool return_grasp_pose(am_msgs::GetGraspPose::Request &req, am_msgs::GetGraspPose::Response &res);
	tf::Transform transform_gripper;
	tf::Transform transform_object;

private:
	// ------------------------- Class Properties -----------------------------
	am_msgs::Object object_;
	double object_height_;
	double object_mass_;
	double waiting_height_;
	double safety_margin_obj2wait_;
	double grasp_width_;
	std::vector<boost::array<double,3> > r_02shape_0_;
	geometry_msgs::Point object_CoM_;
	unsigned int idx_shape_CoM_;
	double quat_02obj_[4]; // 1st element is w
	double A_obj20_[9];    // Rotation matrix corresponding to quat_02obj_
	std::vector<double> shape_height_;
	geometry_msgs::Pose LWRTCP_target_pose_;// LWR-TCP target pose
	geometry_msgs::Pose GPTCP_target_pose_;// GP-TCP2 target pose

	bool ctrl_outp_;
	double ez_0_0_[3];
	double gripper_height_ ;// Height of gripper jaws
	double safety_distance_;// Needed when checking

	// ------------------------- Class Methods ----------------------------
	void set_object_data_(am_msgs::Object);
	void compute_abs_shape_positions_();
	void compute_object_height_();
	void compute_object_CoM_();
	void compute_idx_shape_CoM_();
	void compute_grasp_pose_(unsigned int);
	geometry_msgs::Pose transform_grasp_pose_GPTCP_2_LWRTCP_();
	std::vector<boost::array<double,3> > get_base_vectors_( double*, bool );
	void cross_product_( double*, double*, double* );
	void matrix_multiply_( double*, double*, double* );
	void matrix_transpose_( double*, double* );
	geometry_msgs::Pose get_grasp_pose_(unsigned int);
};

#endif /* GRASPPOSE_H_ */
