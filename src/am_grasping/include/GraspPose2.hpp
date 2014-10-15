/*
 * GraspPose2.hpp
 *
 *  Created on: Aug 13, 2014
 *      Author: Christoph Schuetz (based on Niko's Code)
 */

#ifndef GRASPPOSE2_H_
#define GRASPPOSE2_H_

//common includes
#include <math.h>

//ros includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>

//am msgs
#include <am_msgs/Object.h>
#include <am_msgs/GetGraspPose.h>
#include <am_msgs/CheckPoses.h>

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
	// Transform for frame of object (received in service request)
	tf::Transform transform_object;
	//transform Broadcaster
	tf::TransformBroadcaster br;

	//!node handle for this node
	ros::NodeHandle n;

	//!client to check poses service from motion planning
	ros::ServiceClient check_poses_client_;
	am_msgs::CheckPoses check_poses_srv_;


	//stamped Transform from GPTCP to LWRTCP
	tf::StampedTransform transform_GPTCP_2_LWRTCP_;

	// Transform shapes
	std::vector<tf::Transform> b_transform_shapes_;
	std::vector<tf::Transform> o_transform_shapes_;

private:
	// ------------------------- Class Properties -----------------------------
	// "Object" as defined in ros message
	am_msgs::Object object_;
	am_msgs::TargetZone target_zone_;
	// type of object (handle, cube, cylinder)
	uint8_t object_type_;
	// Mass of object
	double object_mass_;
	//dimensions of the bounding box
	double bbox_x_, bbox_y_, bbox_z_;
	//! pose of the object at the beginning
	uint16_t object_pose_type_;

	// Index of shape closest to COM
	int8_t com_idx_;
	std::vector<double> dist_COM_shape_;
	// Center of shape
	tf::Vector3 shape_com_;
	tf::Vector3 conn_;
	// Center of object mass
	tf::Vector3 b_object_com_;
	tf::Vector3 o_object_com_;

	std::vector<geometry_msgs::Pose> GPTCP_object_grip_pose;
	std::vector<geometry_msgs::Pose> LWRTCP_object_grip_pose;
	std::vector<geometry_msgs::Pose> GPTCP_object_safe_pose;
	std::vector<geometry_msgs::Pose> LWRTCP_object_safe_pose;
	std::vector<geometry_msgs::Pose> GPTCP_object_vision_pose;
	std::vector<geometry_msgs::Pose> LWRTCP_object_vision_pose;
	std::vector<uint16_t> object_skip_vision;
	std::vector<uint16_t> grip_pose_type;
	std::vector<double> object_grasp_width;
	std::vector<geometry_msgs::Pose> GPTCP_target_place_pose;
	std::vector<geometry_msgs::Pose> LWRTCP_target_place_pose;
	std::vector<geometry_msgs::Pose> GPTCP_target_safe_pose;
	std::vector<geometry_msgs::Pose> LWRTCP_target_safe_pose;
	std::vector<geometry_msgs::Pose> GPTCP_target_vision_pose;
	std::vector<geometry_msgs::Pose> LWRTCP_target_vision_pose;
	std::vector<uint16_t> target_skip_vision;
	std::vector<uint16_t> place_pose_type;
	std::vector<geometry_msgs::Vector3> object_grip_r_tcp_com;
	std::vector<geometry_msgs::Vector3> object_grip_r_gp_com;
	std::vector<geometry_msgs::Vector3> object_grip_r_gp_obj;

	double gripping_angle_deg_;
	double gripping_angle_rad_;
	double gripper_maxwidth_;
	double gripper_height_;
	double gripper_finger_width;
	double grip_safety_dist_;
	double vertical_handle_safe_planar_offset;
	double vertical_handle_safe_z_offset;
	double place_falling_dist_;
	double vision_distance_object_height_cube_;
	double vision_distance_object_height_cylinder_;
	double vision_distance_object_height_handle_;
	double pi;

	// ------------------------- Class Methods ----------------------------
	// "set_object_data_()" sets the properties "object_", "quat_02obj", "A_obj20_", and calls method "compute_abs_shape_positions_()"
	void set_object_data_(am_msgs::Object, am_msgs::TargetZone target_zone);
	// "compute_abs_shape_positions_()" sets the property "r_02shape_0_"
	void compute_abs_shape_poses_();
	// "compute_object_height_()" sets the properties "object_height_" and "waiting_height_"
	void compute_object_CoM_();
	// "compute_idx_shape_CoM_()" sets the property "idx_shape_CoM_"
	void compute_idx_shape_CoM_();
	// "compute_bounding_box()" calculates the bounding box of the object in the object frame
	void compute_bounding_box_();
	// "compute_grasp_pose_(prio)" computes the desired gripper pose in the gripper frame
	void compute_grasp_poses_();
	// "get_transform_GPTCP_2_LWRTCP()" returns the transform between GPTCP and LWRTCP
	bool get_transform_GPTCP_2_LWRTCP();
	// "transform_pose_GPTCP_2_LWRTCP_()" returns the desired gripper poses transformed into the LWR-TCP frame
	geometry_msgs::Pose transform_pose_GPTCP_2_LWRTCP_(geometry_msgs::Pose GPTCP_pose);
	// "compute_relative_vectors_()" computes the relative vectors (object<->gripper) for object_grip_poses
	void compute_relative_vectors_();
	// "set_orientation_from_axes()" computes the orientation quaternion from the given axes x, y, z
	void set_orientation_from_axes(geometry_msgs::Pose &tmp_pose, tf::Vector3 x_axis, tf::Vector3 y_axis, tf::Vector3 z_axis);
	// "sort_poses()" checks if the computed poses are valid and sorts them according to their priority (with IK from motion planning)
	void sort_poses();
	// "print_results()" gives the whole bunch of poses as output
	void print_results();
	// "send_poses_to_tf_broadcaster()" sends the whole poses to the tf broadcaster (for rviz)
	void send_poses_to_tf_broadcaster();
	// "get_rotationmatrixfromaxis()" returns a transformation matrix to rotate around an axis (with given angle)
	tf::Matrix3x3 get_rotationmatrixfromaxis(tf::Vector3 axis, double angle);

	//old stuff
	//geometry_msgs::Pose GPTCP_target_pose_;
	//geometry_msgs::Pose LWRTCP_target_pose_;
	//!relative vector from TCP -> Object CoM
	//geometry_msgs::Vector3 r_tcp_com_;
	//!relative vector from GP -> Object CoM
	//geometry_msgs::Vector3 r_gp_com_;
	//!target zone offset (x,y)
	//geometry_msgs::Vector3 r_target_offset_;
	// Object height
	//double object_height_;
	//int8_t high_idx_;
	// Grasp width
	//double grasp_width_;
	//void compute_grasp_pose_();
	// "compute_grasp_poses_" computes the desired gripper pose in the gripper frame
	//void compute_object_height_();
	// "compute_object_CoM_()" sets the properties "object_CoM_" and "object_mass_"
	// "transform_grasp_pose_GPTCP_2_LWRTCP_()" returns the desired gripper pose transformed into the LWR-TCP frame
	//void transform_grasp_pose_GPTCP_2_LWRTCP_();
	// Rotation Matrix Grasp Shape
	//tf::Matrix3x3 dcm_object_;
	// Rotation Matrix Grasp Pose
	//tf::Matrix3x3 dcm_grasp_;
};

#endif /* GRASPPOSE2_H_ */
