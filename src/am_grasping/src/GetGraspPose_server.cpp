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
#include <tf/transform_broadcaster.h>
#include <config.hpp>

// Distance from LWR-TCP to GP-TCP in z0-direction
const double dr_tcp=0.173; // Need to use tf-Trafo later for Transformation: LWR-TCP -> GP -> GP-TCP
// Height of gripper jaws
const double gripper_height = 0.06;

tf::Transform transform_0;
tf::Transform transform_gripper;
tf::Transform transform_object;

//##########################################################################################//
//                              Auxiliary functions                                         //
//##########################################################################################//
void cross_product(double* vec1, double* vec2, double* vec1_x_vec2) {
	vec1_x_vec2[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
	vec1_x_vec2[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
	vec1_x_vec2[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

void matrix_times_vector(double* A, double* b, double* r) {
	// Compute A*b=r assuming that A is a 9-element array with
	// A = {A_11 A_12 A_13, A_21 A_22 A_23, A_31 A_32 A_33};
	for (unsigned row = 0; row < 3; row++) {
		r[row] = 0;
		for (unsigned col = 0; col < 3; col++)
			r[row] += A[row * 3 + col] * b[col];
	}
}

void matrix_multiply(double* M1, double* M2, double* Result) {
	for (unsigned row = 0; row < 3; row++) {
		for (unsigned col = 0; col < 3; col++) {
			Result[row * 3 + col] = 0;
			for (unsigned i = 0; i < 3; i++)
				Result[row * 3 + col] += M1[row * 3 + i] * M2[i * 3 + col];
		}
	}
}

void matrix_transpose(double* A, double* At) {
	unsigned int idx_At[9] = { 0, 3, 6, 1, 4, 7, 2, 5, 8 };
	for (unsigned i = 0; i < 9; i++)
		At[idx_At[i]] = A[i];
}

void rotate_vec_k2i(double* v_k, double* quat, double* v_i) {
	// Compute v_i = A_ik * v_k, where A_ik is represented by a quaternion q_ik
	// Convert object-quaternion to rotation matrix
	double A_ik[9];
	double A_ki[9];
	quat2dcm(quat,A_ki);
	matrix_transpose(A_ki,A_ik);
	matrix_times_vector( A_ik, v_k, v_i  );
}

std::vector<boost::array<double,3> > get_base_vectors( double* A_k20, unsigned int verbose ) {
	// Extract base-vectors
	std::vector<boost::array<double,3> > base_vectors(3);
	for(unsigned i = 0; i < 3; i++) {
		if(verbose)
			std::cout << "Base vector " << i+1 << std::endl;
		for(unsigned j = 0; j < 3; j++) {
			base_vectors.at(i).at(j) = A_k20[i + j*3];
			if(verbose)
				std::cout << A_k20[i + j*3] << std::endl;
		}
	}

	return base_vectors;
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

	std::cout << "Test of matrix_transpose: " << std::endl;
	double A[9] = {1,2,3,4,5,6,7,8,9};
	double At[9];
	matrix_transpose(A,At);
		std::cout << "A: " << std::endl;
		for(unsigned int i=0; i<3; i++)
			{
				for(unsigned int j=0; j<3; j++)
					std::cout << A[i*3 + j] << "  ";
				std::cout << std::endl;
			}
		std::cout << "At: " << std::endl;
		for(unsigned int i=0; i<3; i++)
		{
			for(unsigned int j=0; j<3; j++)
				std::cout << At[i*3 + j] << "  ";
			std::cout << std::endl;
		}

	// CoG coordinates relative to absolute object pose
	double object_CoG[3]     = {0.0, 0.0, 0.0}; // Relative position of object-CoG w.r.t abs_pose (in obj-FoR)
	double object_CoG_abs[3] = {0.0, 0.0, 0.0}; // Absolute position of object-CoG (in 0-FoR)
	double shape_masses[3]   = {0.0, 0.0, 0.0};
	double object_mass = 0.0;
	// relative vector from reference point of shape-FoR to reference point of object-FoR, given in object-FoR
	double r_obj2shape_obj[3];
	// relative vector from reference point of shape-FoR to reference point of object-FoR, given in 0-FoR
	double r_obj2shape_0[3];
	// relative vector from reference point of 0-FoR to reference point of object-FoR, given in 0-FoR
	std::vector<boost::array<double,3> > r_02shape_0(req.object.nr_shapes);
	// quaternion rotation vector describing rotation from 0-FoR to obj-FoR
	double quat_02obj[4] = {req.object.abs_pose.orientation.w,req.object.abs_pose.orientation.x,\
			req.object.abs_pose.orientation.y,req.object.abs_pose.orientation.z};
	// Rotation matrix converting a vector from the object frame into the 0-frame
	double A_obj20[9]; // A_0.obj
	quat2dcm(quat_02obj,A_obj20);
	// Rotation matrix converting a vector from the shape frame into the object frame
	double A_shape2obj[9]; // A_obj.shape
	// Rotation matrix converting a vector from the object frame into the 0-frame
	double A_shape20[9]; // A_0.shape
	// object height
	double object_height = 0.0;
	//
	double q_obj2shape[4];
	for(unsigned int idx_shape=0; idx_shape<req.object.nr_shapes; idx_shape++ )
	{
		// Compute volume & mass of each shape
		// -----------------------------------
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

		// Compute object mass
		// -------------------
		object_mass += shape_masses[idx_shape];

		// Compute shape's absolute position
		// ---------------------------------
		r_obj2shape_obj[0] = current_shape->pose.position.x;
		r_obj2shape_obj[1] = current_shape->pose.position.y;
		r_obj2shape_obj[2] = current_shape->pose.position.z;

		// Rotate relative position from obj to shape (given in obj-frame) into the inertial frame
		rotate_vec_k2i(r_obj2shape_obj, quat_02obj,r_obj2shape_0);
		// Compute absolute position of shape
		r_02shape_0.at(idx_shape).at(0) = r_obj2shape_0[0] + req.object.abs_pose.position.x;
		r_02shape_0.at(idx_shape).at(1) = r_obj2shape_0[1] + req.object.abs_pose.position.y;
		r_02shape_0.at(idx_shape).at(2) = r_obj2shape_0[2] + req.object.abs_pose.position.z;

		// control output
		std::cout << "Vector r_obj2shape of shape " << (*current_shape).type << ", in obj-FoR:" << std::endl;
		std::cout << "x: " << r_obj2shape_obj[0] << std::endl;
		std::cout << "y: " << r_obj2shape_obj[1] << std::endl;
		std::cout << "z: " << r_obj2shape_obj[2] << std::endl;
		std::cout << "Vector r_obj2shape of shape " << (*current_shape).type << ", in 0-FoR:" << std::endl;
		std::cout << "x: " << r_obj2shape_0[0] << std::endl;
		std::cout << "y: " << r_obj2shape_0[1] << std::endl;
		std::cout << "z: " << r_obj2shape_0[2] << std::endl;
		std::cout << "Vector r_02shape of shape " << (*current_shape).type << ", in 0-FoR:" << std::endl;
		std::cout << "x: " << r_02shape_0.at(idx_shape).at(0) << std::endl;
		std::cout << "y: " << r_02shape_0.at(idx_shape).at(1) << std::endl;
		std::cout << "z: " << r_02shape_0.at(idx_shape).at(2) << std::endl;


		object_CoG[0] += r_02shape_0.at(idx_shape).at(0) * shape_masses[idx_shape];
		object_CoG[1] += r_02shape_0.at(idx_shape).at(1) * shape_masses[idx_shape];
		object_CoG[2] += r_02shape_0.at(idx_shape).at(2) * shape_masses[idx_shape];

		// Compute object height
		// ---------------------
		if( r_02shape_0.at(idx_shape).at(2) > object_height ) {
			// Find shape axis pointing "upwards" (along ez_0)
			double dot_prod_max = -10.0;
			for(unsigned i=0; i<3; i++) {

			}
		}
		q_obj2shape[0] = current_shape->pose.orientation.w;
		q_obj2shape[1] = current_shape->pose.orientation.x;
		q_obj2shape[2] = current_shape->pose.orientation.y;
		q_obj2shape[3] = current_shape->pose.orientation.z;
		//
		quat2dcm(q_obj2shape,A_shape2obj);
		matrix_multiply(A_obj20,A_shape2obj,A_shape20);

	}
	if( object_mass == 0.0 )
		ROS_ERROR("Mass of object could not be computed");

	object_CoG[0] = object_CoG[0] / object_mass;
	object_CoG[1] = object_CoG[1] / object_mass;
	object_CoG[2] = object_CoG[2] / object_mass;

	object_CoG_abs[0] = object_CoG[0];
	object_CoG_abs[1] = object_CoG[1];
	object_CoG_abs[2] = object_CoG[2];

	// Control output
	std::cout << "Computed object-CoG (0-FoR): " << std::endl;
	std::cout << "x: " << object_CoG_abs[0] << std::endl;
	std::cout << "y: " << object_CoG_abs[1] << std::endl;
	std::cout << "z: " << object_CoG_abs[2] << std::endl;


	// For objects with several shapes, find shape closest to object-CoG
	unsigned int idx_shape_CoG;
	if( req.object.nr_shapes > 1) {
		double dr[3];
		double length_dr;
		double length_dr_min = 100.0;
		unsigned int idx_min = 0;
		for(unsigned idx_shape=0; idx_shape<req.object.nr_shapes; idx_shape++) {
			length_dr = 0.0;
			for(unsigned i=0; i<3; i++) {
				dr[i] = r_02shape_0.at(idx_shape).at(i) - object_CoG_abs[i];
				length_dr += dr[i]*dr[i];
			}
			length_dr = sqrt(length_dr);
			if(length_dr < length_dr_min) {
				idx_min = idx_shape;
				length_dr_min = length_dr;
			}
			// Control output
			std::cout << "Distance between ref. point of shape " << req.object.shape[idx_shape].type << " and object-CoG:" << std::endl;
			std::cout << "x: " << dr[0] << std::endl;
			std::cout << "y: " << dr[1] << std::endl;
			std::cout << "z: " << dr[2] << std::endl;
		}
		idx_shape_CoG = idx_min;
		std::cout << "idx_min=" << idx_min << std::endl;
	}
	else
		idx_shape_CoG = 0;


	// Convert object-quaternion to rotation matrix
	q_obj2shape[0]=req.object.shape[idx_shape_CoG].pose.orientation.w;
	q_obj2shape[1]=req.object.shape[idx_shape_CoG].pose.orientation.x;
	q_obj2shape[2]=req.object.shape[idx_shape_CoG].pose.orientation.y;
	q_obj2shape[3]=req.object.shape[idx_shape_CoG].pose.orientation.z;
	quat2dcm(q_obj2shape,A_shape2obj);
	// Control Output
	std::cout << "q_obj2shape = " << std::endl;
	for (unsigned int i = 0; i < 4; i++)
		std::cout << q_obj2shape[i] << std::endl;
	std::cout << "A_obj.shape = " << std::endl;
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++)
			std::cout << A_shape2obj[i * 3 + j] << "     ";
		std::cout << std::endl;
	}
	//
	// Control Output
	std::cout << "q_02obj = " << std::endl;
	for (unsigned int i = 0; i < 4; i++)
		std::cout << quat_02obj[i] << std::endl;
	std::cout << "A_0.obj = " << std::endl;
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++)
			std::cout << A_obj20[i * 3 + j] << "     ";
		std::cout << std::endl;
	}
	//
	matrix_multiply(A_obj20,A_shape2obj,A_shape20);
	std::cout << "A_0.shape = " << std::endl;
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++)
			std::cout << A_shape20[i * 3 + j] << "     ";
		std::cout << std::endl;
	}
	//

	// Extract base-vectors
	std::vector<boost::array<double,3> > obj_base_vectors(3);
	for(unsigned i = 0; i < 3; i++) {
		std::cout << "Base vector " << i+1 << std::endl;
		for(unsigned j = 0; j < 3; j++) {
			obj_base_vectors.at(i).at(j) = A_shape20[i + j*3];
			std::cout << A_shape20[i + j*3] << std::endl;
		}
	}

	// Find object-axis parallel to z-axis of table coordinate frame:
	// ei_0_obj = arg( max_j(abs( ez_0_0' * ej_0_obj )) )
	double ez_0_0[3] = {0.0,0.0,1.0};
	double dot_prod = 0.0, abs_dot_prod_max = 0.0, sgn_dot_prod_max;
	unsigned int idx_dot_prod_max = 0;
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
	std::cout << "idx_dot_prod_max = " << idx_dot_prod_max << std::endl;

	// Determine object-height
	if( !req.object.shape[idx_shape_CoG].type.compare("box")) {
		object_height = req.object.shape[idx_shape_CoG].size[idx_dot_prod_max];
	}
	else if(!req.object.shape[idx_shape_CoG].type.compare("cylinder")) {
		if(idx_dot_prod_max == 2)
			object_height = req.object.shape[idx_shape_CoG].length;
		else
			object_height = 0.05; // req.object.shape[idx_shape_CoG].radius;
	}
	std::cout << "object height = " << object_height << std::endl;

	// Search among other two-object axis to determine gripper x-axis
	double ex_0_GP[3] = { 0.0, 0.0, 0.0 };
	for (unsigned i = 0; i < 3; i++) {
		if (i != idx_dot_prod_max) { // Search among other axes
			if (!req.object.shape[idx_shape_CoG].type.compare("cylinder")) {
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
			} else if (!req.object.shape[idx_shape_CoG].type.compare("box")) {
				// loop through dimensions and find smaller of the two dimension (this way the algorithm can also handle not only square cross-sections but also rectangular cross-sections)
				std::vector<double> box_dims = req.object.shape[idx_shape_CoG].size;
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
	GPTCP_target_pose.position.x = object_CoG_abs[0];
	GPTCP_target_pose.position.y = object_CoG_abs[1];
	// Add z-distance between x_0-y_0 plane and GP-TCP
	// Watch out!!! Assuming that desired GP-orientation points towards table, i.e. ez_GP_0 = - ez_0_0
	// (would need transformation of ez_GP_0 into 0-FoR later)
	// Default z-distance (ensures that CoG is between gripper-jaws)
	double dz_0GPTCP = gripper_height / 2.0 + object_CoG_abs[2];
	// Check for collision with ground / object
	double safety_distance = 5.0 / 1000.0;
	if( dz_0GPTCP < safety_distance + object_height )
		dz_0GPTCP = safety_distance + object_height;
	if( dz_0GPTCP < safety_distance + gripper_height )
		dz_0GPTCP = safety_distance + gripper_height;

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
	// Save base-vectors in rotation matrix
	double A_GP20[9]; // A_0.GP
	double A_02GP[9]; // A_GP.0
	for(unsigned i = 0; i < 3; i++) {
		A_GP20[i*3]     = ex_0_GP[i];
		A_GP20[i*3 + 1] = ey_0_GP[i];
		A_GP20[i*3 + 2] = ez_0_GP[i];
	}
	std::cout << "A_GP20 = " << std::endl;
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++)
			std::cout << A_GP20[i * 3 + j] << "     ";
		std::cout << std::endl;
	}
	matrix_transpose(A_GP20,A_02GP);
	std::cout << "A_02GP = " << std::endl;
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++)
			std::cout << A_02GP[i * 3 + j] << "     ";
		std::cout << std::endl;
	}
	// Transform rotation matrix to quaternion
	double q_02GP[4];
	dcm2quat(A_02GP, q_02GP);
	// Copy quaternion
	geometry_msgs::Quaternion target_orientation;
	target_orientation.w =  q_02GP[0];
	target_orientation.x =  q_02GP[1];
	target_orientation.y =  q_02GP[2];
	target_orientation.z =  q_02GP[3];

	LWRTCP_target_pose.orientation = target_orientation;

	res.grasp_pose = LWRTCP_target_pose;
	std::cout << LWRTCP_target_pose << std::endl;

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

	transform_0.setOrigin(tf::Vector3(0,0,0));
	transform_0.setRotation(tf::Quaternion(0,0,0,1));

	transform_gripper.setOrigin(tf::Vector3(res.grasp_pose.position.x,res.grasp_pose.position.y,res.grasp_pose.position.z));
	transform_gripper.setRotation(tf::Quaternion(res.grasp_pose.orientation.x,res.grasp_pose.orientation.y,res.grasp_pose.orientation.z,res.grasp_pose.orientation.w));

	transform_object.setOrigin(tf::Vector3(req.object.abs_pose.position.x,req.object.abs_pose.position.y,req.object.abs_pose.position.z));
	tf::Quaternion q;
	q.setW(req.object.abs_pose.orientation.w);
	q.setX(req.object.abs_pose.orientation.x);
	q.setY(req.object.abs_pose.orientation.y);
	q.setZ(req.object.abs_pose.orientation.z);
	transform_object.setRotation(q);

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

  static tf::TransformBroadcaster br;
  while (ros::ok())
  {
	  //br.sendTransform(tf::StampedTransform(transform_0, ros::Time::now(), ORIGIN, "0"));
	  br.sendTransform(tf::StampedTransform(transform_gripper, ros::Time::now(), ORIGIN, "grasp pose"));
	  br.sendTransform(tf::StampedTransform(transform_object, ros::Time::now(), ORIGIN, "object pose"));
	  ros::Duration(0.5).sleep();
	  ros::spinOnce();
  }

  //ros::spin();

  return 0;
}
