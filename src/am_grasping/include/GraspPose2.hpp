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
#include <am_msgs/ConveyorBelt.h>

//am includes
#include <config.hpp>
#include <utils.hpp>
#include <tf_rot.hpp>

struct puzzle_box{
  int num_free_face_pairs;

  bool obj_x_free;
  bool obj_y_free;
  bool obj_z_free;

  bool obj_xpos_free;
  bool obj_xneg_free;
  bool obj_ypos_free;
  bool obj_yneg_free;
  bool obj_zpos_free;
  bool obj_zneg_free;
};

typedef enum {
	POSE_XUP=0,
	POSE_XDOWN,
	POSE_YUP,
	POSE_YDOWN,
	POSE_ZUP,
	POSE_ZDOWN
} pose_orientation_type_t;

class GraspPose2 {
public:
	// Constructor
	GraspPose2();
	// Standard Destructor
	virtual ~GraspPose2();
	// ROS-service function
	bool return_grasp_pose(am_msgs::GetGraspPose::Request &req, am_msgs::GetGraspPose::Response &res);
	// Transform for frame of object (received in service request)
	tf::Transform transform_object, transform_puzzlefixture;

	//!node handle for this node
	ros::NodeHandle n;
	tf::TransformBroadcaster br;

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
	std::string error_message;
	uint16_t task_number_;
	am_msgs::Object object_;
	am_msgs::TargetZone target_zone_;
	geometry_msgs::Pose rel_target_pose_, abs_target_pose_;
	geometry_msgs::Pose puzzle_fixture_pose_;
	tf::Vector3 conveyor_belt_move_direction_and_length;

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
	tf::Vector3 b_object_com_, o_object_com_;
	tf::Vector3 b_object_center_, o_object_center_;
	std::vector<puzzle_box> puzzle_boxes;
	bool puzzle_push_in_x_, puzzle_push_in_y_;
	double puzzle_push_in_x_distance_, puzzle_push_in_y_distance_;
	double puzzle_boxsize;
    geometry_msgs::Pose emptyPose;

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
	std::vector<geometry_msgs::Pose> GPTCP_push_safe_pose;
	std::vector<geometry_msgs::Pose> LWRTCP_push_safe_pose;
	std::vector<geometry_msgs::Pose> GPTCP_push_target_pose;
	std::vector<geometry_msgs::Pose> LWRTCP_push_target_pose;
	std::vector<geometry_msgs::Vector3> object_grip_r_tcp_com;

	uint8_t task5_pose_counter_;
	double gripping_angle_deg_;
	double gripping_angle_rad_;
	double gripping_angleT6_deg_;
	double gripping_angleT6_rad_;
	double gripper_maxwidth_;
	double gripper_finger_height_;
	double gripper_finger_width;
	double gripper_width_;
	double gripper_height_;
	double grip_safety_dist_;
	double vertical_handle_safe_planar_offset;
	double vertical_handle_safe_z_offset;
	double place_falling_dist_;
	double place_falling_distT6_;
	double gripping_finger_overlapT5_;
	double vision_distance_object_height_cube_;
	double vision_distance_object_height_cylinder_;
	double vision_distance_object_height_handle_;
	double pi;

	// ------------------------- Class Methods ----------------------------
	// "reset_grasping_node()" resets everything to default values
	void reset_grasping_node();
	// "get_info_from_parameterserver()" gets various infos from the parameter server and stores it locally
	void get_info_from_parameterserver();
	// "set_object_data_()" sets the object and determines its type
	void set_object_data_(am_msgs::Object);
	// "correct_object_alignment()" corrects the object pose orientation
	void correct_object_alignment();
	// "compute_abs_shape_positions_()" sets the property "r_02shape_0_"
	void compute_abs_shape_poses_();
	// "compute_object_CoM_()" computes the center of mass of the object
	void compute_object_CoM_();
	// "compute_object_center_()" computes the center of the object (only shpae-origins, no density)
	void compute_object_center_();
	// "compute_idx_shape_CoM_()" sets the property "idx_shape_CoM_"
	void compute_idx_shape_CoM_();
	// "compute_bounding_box()" calculates the bounding box of the object in the object frame
	void compute_bounding_box_();
	// "does_puzzle_part_lie_flat()" detects wheter the puzzle part is standing vertical (false) or flat on the floor (true)
	bool does_puzzle_part_lie_flat();
	// "correct_puzzle_part_rotation()" rotates the object if its symmetric to avoid mistakes
	void correct_puzzle_part_rotation();
	// "compute_puzzle_free_sides_()" calculates the free sides of the boxes of a puzzle part in world-coordinates
	void compute_puzzle_free_sides_();
	// "compute_grasp_poses_()" computes the desired gripper pose in the gripper frame
	void compute_grasp_poses_();
	// "compute_grasp_posesT5_" computes the desired gripper pose in the gripper frame for Task 5
	void compute_grasp_posesT5_();
	// "compute_grasp_posesT6_" computes the desired gripper pose in the gripper frame for Task 6
	void compute_grasp_posesT6_();
	// "transform_poses_to_LWRTCP" compute the LWRTCP poses out of the GPTCP poses
	void transform_poses_to_LWRTCP();
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
	// "pose_to_transform()" transforms the given geometry_msgs::Pose object to a tf::transform object
	tf::Transform pose_to_transform(geometry_msgs::Pose input_pose);
	// "transform_to_pose()" transforms the given tf::transform object to a geometry_msgs::Pose object
	geometry_msgs::Pose transform_to_pose(tf::Transform input_transform);
};

#endif /* GRASPPOSE2_H_ */
