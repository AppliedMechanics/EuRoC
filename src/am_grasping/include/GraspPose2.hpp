/*
 * GraspPose2.hpp
 *
 *  Created on: Aug 13, 2014
 *      Author: Christoph Schuetz (based on Niko's Code)
 */

#ifndef GRASPPOSE2_H_
#define GRASPPOSE2_H_

//ros includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>

//am msgs
#include <am_msgs/Object.h>
#include <am_msgs/GetGraspPose.h>

//am includes
#include <config.hpp>
#include <utils.hpp>
#include <tf_rot.hpp>

class GraspPose2 {
public:
	// Constructor
	GraspPose2();
	// Standard Destructor
	virtual ~GraspPose2();
	// ROS-service function
	bool return_grasp_pose(am_msgs::GetGraspPose::Request &req, am_msgs::GetGraspPose::Response &res);
	// Transform for frame of LWR-TCP
	tf::Transform transform_gripper;
	// Transform for frame of object (received in service request)
	tf::Transform transform_object;
	// Transform shapes
	std::vector<tf::Transform> b_transform_shapes_;
	std::vector<tf::Transform> o_transform_shapes_;

	bool first_called;
private:
	// ------------------------- Class Properties -----------------------------
	// "Object" as defined in ros message
	am_msgs::Object object_;
	// Object height
	double object_height_;
	// Mass of object
	double object_mass_;
	// Grasp width
	double grasp_width_;
	//!relative vector from TCP -> Object CoM
	geometry_msgs::Vector3 r_tcp_com_;
	//!relative vector from GP -> Object CoM
	geometry_msgs::Vector3 r_gp_com_;
	//!target zone offset (x,y)
	geometry_msgs::Vector3 r_target_offset_;

	// Index of shape closest to COM
	int8_t com_idx_;
	int8_t high_idx_;
	// Center of shape
	tf::Vector3 shape_com_;
	tf::Vector3 conn_;
	// Center of object mass
	tf::Vector3 b_object_com_;
	tf::Vector3 o_object_com_;

	// Rotation Matrix Grasp Shape
	tf::Matrix3x3 dcm_object_;
	// Rotation Matrix Grasp Pose
	tf::Matrix3x3 dcm_grasp_;

	std::vector<double> dist_COM_shape_;

	geometry_msgs::Pose GPTCP_target_pose_;
	geometry_msgs::Pose LWRTCP_target_pose_;
	double gripper_height_;
	double grip_safety_dist_;

	// ------------------------- Class Methods ----------------------------
	// "set_object_data_()" sets the properties "object_", "quat_02obj", "A_obj20_", and calls method "compute_abs_shape_positions_()"
	void set_object_data_(am_msgs::Object);
	// "compute_abs_shape_positions_()" sets the property "r_02shape_0_"
	void compute_abs_shape_poses_();
	// "compute_object_height_()" sets the properties "object_height_" and "waiting_height_"
	void compute_object_height_();
	// "compute_object_CoM_()" sets the properties "object_CoM_" and "object_mass_"
	void compute_object_CoM_();
	// "compute_idx_shape_CoM_()" sets the property "idx_shape_CoM_"
	void compute_idx_shape_CoM_();
	// "compute_grasp_pose_(prio)" computes the desired gripper pose in the gripper frame
	void compute_grasp_pose_();
	// "transform_grasp_pose_GPTCP_2_LWRTCP_()" returns the desired gripper pose transformed into the LWR-TCP frame

	void transform_grasp_pose_GPTCP_2_LWRTCP_();

};

#endif /* GRASPPOSE2_H_ */
