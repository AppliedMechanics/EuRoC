/*
 * GetGraspPose_server.cpp
 *
 *  Created on: Jul 25, 2014
 *      Author: Nikolas Tekles
 */

#include "ros/ros.h"
#include <am_msgs/GetGraspPose.h>
#include <cmath>
#include <iostream>
#include <tf_rot.hpp>
#include <numeric>

// Distance from LWR-TCP to GP-TCP in z0-direction
const double dr_tcp=0.173; // Need to use tf-Trafo later for Transformation: LWR-TCP -> GP -> GP-TCP
// Height of gripper jaws
const double gripper_height = 0.06;

// Auxiliary functions
void cross_product(double* vec1, double* vec2, double* vec1_x_vec2)
{
	vec1_x_vec2[0] = vec1[1]*vec2[2]-vec1[2]*vec2[1];
	vec1_x_vec2[1] = vec1[2]*vec2[0]-vec1[0]*vec2[2];
	vec1_x_vec2[2] = vec1[0]*vec2[1]-vec1[1]*vec2[0];
}

// ROS-Service Function
bool return_grasp_pose(am_msgs::GetGraspPose::Request &req, am_msgs::GetGraspPose::Response &res)
{

	// Declare LWR-TCP target pose
	geometry_msgs::Pose LWRTCP_target_pose;
	// Declare GP-TCP2 target pose
	geometry_msgs::Pose GPTCP_target_pose;

	// Control output
	std::cout << "Received object name: " << req.object.name << std::endl;
	std::cout << "Received value for object's number of shapes: " << req.object.nr_shapes << std::endl;
	std::cout << "Received object pose: " << std::endl;
		std::cout << "x: " << req.object.abs_pose.position.x << std::endl;
		std::cout << "y: " << req.object.abs_pose.position.y << std::endl;
		std::cout << "z: " << req.object.abs_pose.position.z << std::endl;
	std::cout << "Received object quaternion: " << std::endl;
		std::cout << "x: " << req.object.abs_pose.orientation.x << std::endl;
		std::cout << "y: " << req.object.abs_pose.orientation.y << std::endl;
		std::cout << "z: " << req.object.abs_pose.orientation.z << std::endl;
		std::cout << "w: " << req.object.abs_pose.orientation.w << std::endl;

	// CoG coordinates relative to absolute object pose
	double object_CoG[3]     = {0.0, 0.0, 0.0}; // Relative position of object-CoG w.r.t abs_pose (in obj-FoR)
	double object_CoG_abs[3] = {0.0, 0.0, 0.0}; // Absolute position of object-CoG (in 0-FoR)
	double shape_masses[3] = {0.0, 0.0, 0.0};
	double object_mass = 0.0;
	for(unsigned int idx_shape=0; idx_shape<req.object.nr_shapes; idx_shape++ )
	{
		am_msgs::Shape* current_shape = &req.object.shape[idx_shape];
		double shape_volume = 0.0;
		if( !(*current_shape).type.compare("box") )
		{
			shape_volume = (*current_shape).size[0] * (*current_shape).size[1] * (*current_shape).size[2];
			std::cout << "Determined shape: " <<  (*current_shape).type << std::endl;
		}
		else if( !(*current_shape).type.compare("cylinder") )
		{
			shape_volume = M_PI*(*current_shape).radius*(*current_shape).radius*(*current_shape).length;
			std::cout << "Determined shape: " <<  (*current_shape).type << std::endl;
		}
		std::cout << "Determined volume of shape " <<  (*current_shape).type << " :" << shape_volume << std::endl;
		shape_masses[idx_shape] = (*current_shape).density * shape_volume;
		std::cout << "Received density of shape " <<  (*current_shape).type << " :" << (*current_shape).density << std::endl;
		std::cout << "Determined mass of shape " <<  (*current_shape).type << " :" << shape_masses[idx_shape] << std::endl;
		object_mass += shape_masses[idx_shape];
		object_CoG[0] += current_shape->pose.position.x * shape_masses[idx_shape];
		object_CoG[1] += current_shape->pose.position.y * shape_masses[idx_shape];
		object_CoG[2] += current_shape->pose.position.z * shape_masses[idx_shape];
		std::cout << "current_shape->pose.position.z=" << current_shape->pose.position.z << std::endl;
		std::cout << "req.object.=" << req.object.abs_pose.position.z << std::endl;
	}
	if( object_mass == 0.0 )
		ROS_ERROR("Mass of object could not be computed");

	object_CoG[0] = object_CoG[0] / object_mass;
	object_CoG[1] = object_CoG[1] / object_mass;
	object_CoG[2] = object_CoG[2] / object_mass;

	object_CoG_abs[0] = object_CoG[0] + req.object.abs_pose.position.x;
	object_CoG_abs[1] = object_CoG[1] + req.object.abs_pose.position.y;
	object_CoG_abs[2] = object_CoG[2] + req.object.abs_pose.position.z;

	// Control output
	std::cout << "Computed object-CoG (0-FoR): " << std::endl;
	std::cout << "x: " << object_CoG_abs[0] << std::endl;
	std::cout << "y: " << object_CoG_abs[1] << std::endl;
	std::cout << "z: " << object_CoG_abs[2] << std::endl;


	// Need to modify the code later for objects with several subshapes
	if( req.object.nr_shapes > 1)
		ROS_ERROR("Algorithm is currently not capable of computing Grasp Pose for combined objects");
	unsigned int idx_shape = 0;


	// Convert object-quaternion to rotation matrix
	double a_tf[9];
	double q_tf[4];
	q_tf[0]=req.object.shape[idx_shape].pose.orientation.w;
	q_tf[1]=req.object.shape[idx_shape].pose.orientation.x;
	q_tf[2]=req.object.shape[idx_shape].pose.orientation.y;
	q_tf[3]=req.object.shape[idx_shape].pose.orientation.z;
	quat2dcm(q_tf,a_tf);

	// Extract base-vectors
	std::vector<boost::array<double,3> > obj_base_vectors(3);
	for(unsigned i = 0; i < 3; i++) {
		std::cout << "Base vector " << i+1 << std::endl;
		for(unsigned j = 0; j < 3; j++) {
			obj_base_vectors.at(i).at(j) = a_tf[j + i*3];
			std::cout << a_tf[j + i*3] << std::endl;
		}
	}
	// Is shape orientation absolute? Do I need another rotation?


	// Find object-axis parallel to z-axis of table coordinate frame:
	// ei_0_obj = arg( max_j(abs( ez_0_0' * ej_0_obj )) )
	double ez_0_0[3] = {0.0,0.0,1.0};
	double dot_prod = 0.0, abs_dot_prod_max = 0.0, sgn_dot_prod_max;
	unsigned int idx_dot_prod_max = 0;
	double object_height = 0.0;
	for(unsigned ii=0; ii<3; ii++) {
		// Compute vector dot-product
		for(unsigned jj=0; jj<3; jj++)
			dot_prod += ez_0_0[jj] * obj_base_vectors.at(ii)[jj];
		// Find maximum absolute value of dot products
		if(fabs(dot_prod) > abs_dot_prod_max) {
			abs_dot_prod_max = fabs(dot_prod);
			idx_dot_prod_max = ii;
			// Determine sign of dot product
			if( dot_prod > 0.0 )
				sgn_dot_prod_max = 1.0;
			else if( dot_prod < 0.0 )
				sgn_dot_prod_max = -1.0;
			else
				ROS_ERROR("Signs of all dot products are zero - check if object-frame has orthogonal base-vectors");
		}
		dot_prod = 0.0;
	}
	// Determine object-height
	if( !req.object.shape[idx_shape].type.compare("box")) {
		object_height = req.object.shape[idx_shape].size[idx_dot_prod_max];
	}
	else if(!req.object.shape[idx_shape].type.compare("cylinder")) {
		if(idx_dot_prod_max == 2)
			object_height = req.object.shape[idx_shape].length;
		else
			object_height = req.object.shape[idx_shape].radius;
	}

	// Search among other two-object axis to determine gripper x-axis
	double ex_0_GP[3] = { 0.0, 0.0, 0.0 };
	for (unsigned i = 0; i < 3; i++) {
		if (i != idx_dot_prod_max) { // Search among other axes
			if (!req.object.shape[idx_shape].type.compare("cylinder")) {
				// Distinguish between the two cases cylinder pointing upwards and cylinder lying horizontally
				if (idx_dot_prod_max == 2) { // cylinder pointing upwards
					// This must be replaced, e.g. with old gripper x-axis (coz no rotation is necessary)
					ex_0_GP[0] = 1.0; // any direction
					ex_0_GP[1] = 0.0;
					ex_0_GP[2] = 0.0;
				} else { // Cylinder lying horizontally
						 // desired gripper x-axis is perpendicular to object z-axis and to table z-axis
					cross_product(&obj_base_vectors.at(2)[0], ez_0_0, ex_0_GP);
					// might have to change vector orientation later to avoid unnecessary rotations
				}
			} else if (!req.object.shape[idx_shape].type.compare("box")) {
				// loop through dimensions and find smaller of the two dimension (this way the algorithm can also handle not only square cross-sections but also rectangular cross-sections)
				std::vector<double> box_dims = req.object.shape[idx_shape].size;
				double min_length = 100.0;
				unsigned int idx_min_length = 4;
				for (unsigned j = 0; j < 3; j++) {
					if ((j != idx_dot_prod_max) && (min_length >= box_dims[j])) {
						idx_min_length = j;
						min_length = box_dims[j];
					}
				}
				if (min_length > 99.9)
					ROS_ERROR("Could not find smallest length of red_cube");
				if (idx_min_length == 4)
					ROS_ERROR("Could not determine idx for smallest length");
				ex_0_GP[0] = obj_base_vectors.at(idx_min_length)[0];
				ex_0_GP[1] = obj_base_vectors.at(idx_min_length)[1];
				ex_0_GP[2] = obj_base_vectors.at(idx_min_length)[2];
			}
		}
	}


	// Control output
	 std::cout << "Determined desired gripper x-axis:" << std::endl;
	 for( unsigned i = 0; i<3; i++ )
	 std::cout << ex_0_GP[i] << std::endl;

	// Set desired gripper z-axis (pointing downwards)
	double ez_0_GP[3] = { 0.0, 0.0, -1.0 };
	// Control output
	 std::cout << "Determined desired gripper z-axis:" << std::endl;
	 for( unsigned i = 0; i<3; i++ )
	 std::cout << ez_0_GP[i] << std::endl;

	// Compute desired gripper y-axis from cross product
	double ey_0_GP[3];
	cross_product(ez_0_GP, ex_0_GP, ey_0_GP);

	// Control output
	 std::cout << "Determined desired gripper y-axis:" << std::endl;
	 for( unsigned i = 0; i<3; i++ )
	 std::cout << ey_0_GP[i] << std::endl;



	//---------------------------------------------------------------------------------------//
	//                            Set GP-TCP target position                                 //
	//---------------------------------------------------------------------------------------//
	double ez_GP_0[3] = {0.0, 0.0, -1.0};
	GPTCP_target_pose.position.x = object_CoG_abs[0];
	GPTCP_target_pose.position.y = object_CoG_abs[1];
	// Add z-distance between x_0-y_0 plane and GP-TCP
	// Watch out!!! Assuming that desired GP-orientation points towards table, i.e. ez_GP_0 = - ez_0_0
	// (would need transformation of ez_GP_0 into 0-FoR later)
	// Default z-distance (ensures that CoG is between gripper-jaws)
	double dz_0GPTCP = gripper_height / 2.0 + object_CoG_abs[2];
	// Check for collision with ground / object
	double safety_factor = 1.1;
	if( dz_0GPTCP < safety_factor * object_height )
		dz_0GPTCP = safety_factor * object_height;
	if( dz_0GPTCP < safety_factor * gripper_height )
		dz_0GPTCP = safety_factor * gripper_height;

	if( object_CoG_abs[2] < dz_0GPTCP - gripper_height )
		ROS_WARN("CoG not between gripper jaws");

	GPTCP_target_pose.position.z = dz_0GPTCP;
	// Transformation: GP-TCP -> LWR-TCP (must be replaced later with call of tf-trafo service)
	LWRTCP_target_pose.position.x = GPTCP_target_pose.position.x;
	LWRTCP_target_pose.position.y = GPTCP_target_pose.position.y;
	LWRTCP_target_pose.position.z = GPTCP_target_pose.position.z + dr_tcp;



	//---------------------------------------------------------------------------------------//
	//                            Set GP-TCP target orientation                              //
	//---------------------------------------------------------------------------------------//
	// Save base-vectors in rotation matrix (loop through matrix entries columnwise)
	double r_tf[9];
	for (unsigned i = 0; i < 3; i++) {
		r_tf[i]     = ex_0_GP[i];
		r_tf[3 + i] = ey_0_GP[i];
		r_tf[6 + i] = ez_0_GP[i];
	}
	// Transform rotation matrix to quaternion
	double temp_quat[4];
	dcm2quat(r_tf, temp_quat);
	// Copy quaternion
	geometry_msgs::Quaternion target_orientation;
	target_orientation.w = temp_quat[0];
	target_orientation.x = temp_quat[1];
	target_orientation.y = temp_quat[2];
	target_orientation.z = temp_quat[3];

	LWRTCP_target_pose.orientation = target_orientation;

	res.grasp_pose = LWRTCP_target_pose;

	std::cout << "Desired LWR-TCP position: " << std::endl;
			std::cout << "x: " << res.grasp_pose.position.x << std::endl;
			std::cout << "y: " << res.grasp_pose.position.y << std::endl;
			std::cout << "z: " << res.grasp_pose.position.z << std::endl;
	std::cout << "Desired gripper quaternion: " << std::endl;
			std::cout << "x: " << res.grasp_pose.orientation.x << std::endl;
			std::cout << "y: " << res.grasp_pose.orientation.y << std::endl;
			std::cout << "z: " << res.grasp_pose.orientation.z << std::endl;
			std::cout << "w: " << res.grasp_pose.orientation.w << std::endl;

	uint status = 1;

	if (status)
	{
		res.error_message = "";
		return true;
	}
	else
	{
		res.error_message = "Failed to return TCP-pose";
		return false;
	}
}

// Main Function -- Server-Node
int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_grasp_pose_srv_node"); //
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("GraspPose_srv", return_grasp_pose);
  ROS_INFO("Ready to calculate Grasp-Pose.");
  ros::spin();

  return 0;
}
