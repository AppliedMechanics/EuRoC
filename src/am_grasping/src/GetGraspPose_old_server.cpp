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

class ObjectProperties{
	double length_, radius_, volume_;
	std::string object_shape_;
	std::vector<double> box_dimensions_;
public:
	ObjectProperties();
	void set_object_dimensions (double, double);
	void set_object_shape (const std::string&);
	std::vector<double> get_object_dimensions ();
	std::string get_object_shape ();
	void compute_object_volume ();
	double get_object_volume ();
};
// Class constructor
ObjectProperties::ObjectProperties()
{
	length_ = -1;
	radius_ = -1;
	box_dimensions_.resize(3);
	box_dimensions_.at(0) = -1;
	box_dimensions_.at(1) = -1;
	box_dimensions_.at(2) = -1;
	volume_   = -1;
	object_shape_ = "Empty";
}
// Class methods
void ObjectProperties::set_object_shape (const std::string& shape_name)
{
	if( !shape_name.compare("box") )
		object_shape_ = "box";
	else if( !shape_name.compare("cylinder") )
	{
		ROS_INFO("Obj shape before being set: '%s'", object_shape_.c_str() );
		object_shape_ = "cylinder";
		ROS_INFO("Obj shape after being set: '%s'", object_shape_.c_str() );
	}
	else
		ROS_ERROR("Object shape %s not supported. Choose box or cylinder", shape_name.c_str() );
}

void ObjectProperties::set_object_dimensions (double length, double radius)
{
	if( !object_shape_.compare("cylinder") )
	{
		length_ = length;
		radius_ = radius;
		this->compute_object_volume();
	}
	else if( !object_shape_.compare("box") )
	{
		ROS_INFO("In ObjectGeometry::set_object_dimensions: Cannot set parameters 'length' and 'radius' for object type 'box'");
	}
}
std::string ObjectProperties::get_object_shape ()
{
	return object_shape_;
}

std::vector<double> ObjectProperties::get_object_dimensions ()
{
	if( !object_shape_.compare("box") )
		return box_dimensions_;
	else if( !object_shape_.compare("cylinder") )
	{
		std::vector<double> cyl_dims(2);
		cyl_dims.at(0) = length_;
		cyl_dims.at(1) = radius_;
		return cyl_dims;
	}
	else
	{
		ROS_ERROR("In ObjectProperties::get_object_dimensions: Unknown object shape.");
		std::vector<double> temp(1);
		temp.at(0) = -1.0;
		return temp;
	}

}

void ObjectProperties::compute_object_volume ()
{
	if( !object_shape_.compare("cylinder") )
		volume_ = M_PI * pow(radius_,2) * length_;
	else if( !object_shape_.compare("box") )
		volume_ = -1; // TODO
	else
	{
		ROS_INFO("comp_volume not implemented for obj shape '%s'", object_shape_.c_str() );
	}
}

double ObjectProperties::get_object_volume()
{
	return volume_;
}

// Declaration of Auxiliary Functions
std::vector<double> compute_CoG_position( std::vector<geometry_msgs::Pose>& obj_poses, std::vector<ObjectProperties>& obj_props );
void cross_product(double* vec1, double* vec2, double* result);

// ROS-Service Function
bool return_grasp_pose(am_msgs::GetGraspPose::Request &req, am_msgs::GetGraspPose::Response &res)
{
	// Read Request variables: ...
	//			... object type, ....
	std::string obj_type = req.object.shape[0].type;
	//			... object pose
	geometry_msgs::Pose obj_pose = req.object.abs_pose;

	// Declare response variable (target pose)
	geometry_msgs::Pose TCP_target_pose;

	// Auxiliary Variables
	uint status = 1;

	// Identify object parameters & geometry
	double density, mass;
	ObjectProperties obj_props_temp;void cross_product(double* vec1, double* vec2, double* result);

	if( !obj_type.compare("cylinder") )
	{
		density = 19302.0;     // Unit [kg/m^3]
		double length = 0.1;   // Unit [m]
		double radius = 0.02;  // Unit [m]
		obj_props_temp.set_object_shape("cylinder");
		obj_props_temp.set_object_dimensions(length, radius);
		ROS_INFO("Object shape recognized as cylinder.");
		mass = density * obj_props_temp.get_object_volume();
		ROS_INFO("Object mass computed: %.2f.", mass);
	}
	else
	{
		//			... object dimensions, ...
		double obj_dim [3];
		obj_dim[0] = req.object.shape[0].size[0];
		obj_dim[1] = req.object.shape[0].size[1];
		obj_dim[2] = req.object.shape[0].size[2];


		ROS_INFO("Error somewhere");
		ROS_INFO("obj_type is %s", obj_type.c_str() );
	}

	// Compute position of center of gravity
	std::vector<geometry_msgs::Pose> obj_poses(1);
	obj_poses.at(0) = obj_pose;
	std::vector<ObjectProperties> obj_props(1);
	obj_props.at(0) = obj_props_temp;
	std::vector<double> CoG_position = compute_CoG_position(obj_poses, obj_props);

	// Convert object-quaternion to rotation matrix
	double a_tf[9];
	double q_tf[4];
	q_tf[0]=obj_pose.orientation.w;
	q_tf[1]=obj_pose.orientation.x;
	q_tf[2]=obj_pose.orientation.y;
	q_tf[3]=obj_pose.orientation.z;
	quat2dcm(q_tf,a_tf);
	// Extract base vectors of obj-frame represented in the 0-frame
	unsigned int idx_x[3] = {0,1,2};
	unsigned int idx_y[3] = {3,4,5};
	unsigned int idx_z[3] = {6,7,8};


	std::vector<double> ex_0_obj(3); // x-axis direction of object frame given in 0-frame
	std::vector<double> ey_0_obj(3); // y-axis direction of object frame given in 0-frame
	std::vector<double> ez_0_obj(3); // z-axis direction of object frame given in 0-frame
	for(unsigned i = 0; i < 3; i++)
	{
		ex_0_obj[i] = a_tf[idx_x[i]];
		ey_0_obj[i] = a_tf[idx_y[i]];
		ez_0_obj[i] = a_tf[idx_z[i]];
	}

	double temp[3];
	std::vector<boost::array<double,3> > obj_base_vectors(3);
	for(unsigned i = 0; i < 3; i++)
	{
		//std::cout << "Base vector " << i+1 << std::endl;
		for(unsigned j = 0; j < 3; j++)
		{
			obj_base_vectors.at(i).at(j) = a_tf[j + i*3];
			//temp[j] = a_tf[j + i*3];
			//std::cout << temp[j] << std::endl;
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

		//std::cout << "Base-Vector " << ii+1 << " has dot-Product " << dot_prod << std::endl;

		dot_prod = 0.0;
	}

	// std::cout << "idx of dot product max: " << idx_dot_prod_max << std::endl;

	// Search among other two-object axis to determine gripper x-axis
	double ex_0_GP[3] = {0.0, 0.0, 0.0};
	for( unsigned i=0; i<3; i++ )
	{
		if( i != idx_dot_prod_max )
		{
			if( !obj_type.compare("green_cylinder") ) {
				// Distinguish between the two cases cylinder pointing upwards and cylinder lying horizontally
				if( idx_dot_prod_max == 2 ) { // cylinder pointing upwards
					// This must be replaced, e.g. with old gripper x-axis (coz no rotation is necessary)
					ex_0_GP[0] = 1.0;
					ex_0_GP[1] = 0.0;
					ex_0_GP[2] = 0.0;
				}
				else { // Cylinder lying horizontally
					// desired gripper x-axis is perpendicular to object z-axis and to table z-axis
					cross_product(&obj_base_vectors.at(2)[0],ez_0_0,ex_0_GP);
					/*double* vec1 = &obj_base_vectors.at(2)[0];
					double* vec2 = ez_0_0;
					double* vec1_x_vec2 = ex_0_GP;
					vec1_x_vec2[0] = vec1[1]*vec2[2]-vec1[2]*vec2[1];
					vec1_x_vec2[1] = vec1[2]*vec2[0]-vec1[0]*vec2[2];
					vec1_x_vec2[2] = vec1[0]*vec2[1]-vec1[1]*vec2[0];*/
					// might have to change vector orientation later to avoid unnecessary rotations
					// std::cout << "Vec1" << std::endl;
					// for (unsigned k = 0; k < 3; k++)
					// 	std::cout << vec1[k] << std::endl;
					// std::cout << "Vec2" << std::endl;
					// for (unsigned k = 0; k < 3; k++)
					// 	std::cout << vec2[k] << std::endl;
					// std::cout << "Result of cross product" << std::endl;
					// for( unsigned k=0; k<3; k++ )
					// 	std::cout << vec1_x_vec2[k] << std::endl;
				}
			} else if( !obj_type.compare("red_cube") ) {
				ROS_INFO("Error: Target Pose for red cube not implemented yet");
				// loop through dimensions and find smaller of the two dimension
				std::vector<double> box_dims = obj_props.at(0).get_object_dimensions();
				double min_length = 100.0;
				unsigned int idx_min_length = 4;
				for( unsigned j=0; j>3; j++ ){
					if( (j != idx_dot_prod_max) && (min_length > box_dims[j]) )
						idx_min_length = j;
				}
				if( min_length > 99.0 )
					ROS_ERROR("Could not find smallest length of red_cube");
				if( idx_min_length == 4 )
					ROS_ERROR("Could not determine idx for smallest length");
				ex_0_GP[0] = obj_base_vectors.at(idx_min_length)[0];
				ex_0_GP[1] = obj_base_vectors.at(idx_min_length)[1];
				ex_0_GP[2] = obj_base_vectors.at(idx_min_length)[2];
			}
		}
	}
	/*// Control output
	std::cout << "Determined desired gripper x-axis:" << std::endl;
	for( unsigned i = 0; i<3; i++ )
		std::cout << ex_0_GP[i] << std::endl;*/


	// Set desired gripper z-axis (pointing downwards)
	double ez_0_GP[3] = {0.0, 0.0, -1.0};
/*	// Control output
	std::cout << "Determined desired gripper z-axis:" << std::endl;
	for( unsigned i = 0; i<3; i++ )
		std::cout << ez_0_GP[i] << std::endl;*/

	// Compute desired gripper y-axis from cross product
	double ey_0_GP[3];
	cross_product(ez_0_GP,ex_0_GP,ey_0_GP);
/*	// Call function later
	double* vec2 = ex_0_GP;
	double* vec1 = ez_0_GP;
	double* vec1_x_vec2 = ey_0_GP;
	vec1_x_vec2[0] = vec1[1]*vec2[2]-vec1[2]*vec2[1];
	vec1_x_vec2[1] = vec1[2]*vec2[0]-vec1[0]*vec2[2];
	vec1_x_vec2[2] = vec1[0]*vec2[1]-vec1[1]*vec2[0];*/

	/*// Control output
	std::cout << "Determined desired gripper y-axis:" << std::endl;
	for( unsigned i = 0; i<3; i++ )
		std::cout << ey_0_GP[i] << std::endl;*/

	// Save base-vectors in rotation matrix (loop through matrix entries columnwise)
	double r_tf[9];
	for( unsigned i=0; i<3; i++ )
		r_tf[i]   = ex_0_GP[i];
	for( unsigned i=0; i<3; i++ )
		r_tf[3+i] = ey_0_GP[i];
	for( unsigned i=0; i<3; i++ )
		r_tf[6+i] = ez_0_GP[i];

	// Transform rotation matrix to quaternion
	double temp_quat[4];
	dcm2quat( r_tf, temp_quat);
	// Copy quaternion
	geometry_msgs::Quaternion target_orientation;
	target_orientation.w = temp_quat[0];
	target_orientation.x = temp_quat[1];
	target_orientation.y = temp_quat[2];
	target_orientation.z = temp_quat[3];

	TCP_target_pose.orientation = target_orientation;

	// Add half of the gripper height in TCP_(z) direction
//
//	TCP_target_pose.position.x = CoG_position[0];
//	TCP_target_pose.position.y = CoG_position[1];
//	TCP_target_pose.position.z = CoG_position[2];
	TCP_target_pose.position = req.object.abs_pose.position;

	res.grasp_pose = TCP_target_pose;

	if(status){
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

// Auxiliary functions
std::vector<double> compute_CoG_position(std::vector<geometry_msgs::Pose>& obj_poses, std::vector<ObjectProperties>& obj_props )
{
	// ------------------------------------------------------------------------------------
	//                    -- Use conventions for coordinate frames --
	// For boxes:
	//		coordinate frame reference point: 	geometric center of the cube
	//		coordinate frame axes:				any Cartesian reference frame wit axes perpendicular on the faces of the box
	// For cylinders:
	//		coordinate frame z-axis:			cylinder's rotational axis
	//		coordinate frame reference point: 	on z-axis at half of the cylinder's height
	// ------------------------------------------------------------------------------------

	std::vector<double> CoG_position(3);
	if( !( obj_props[0].get_object_shape() ).compare("box") )
	{
		CoG_position.at(0) = 0.0;
		CoG_position.at(1) = 0.0;
		CoG_position.at(2) = 0.0;
	}
	else if( !( obj_props[0].get_object_shape() ).compare("cylinder") )
	{
		CoG_position.at(0) = 0.0;
		CoG_position.at(1) = 0.0;
		CoG_position.at(2) = 0.0;
	}

	ROS_INFO("Return value of get_object_shape: %s", (obj_props[0].get_object_shape()).c_str() );

	return CoG_position;
}

void cross_product(double* vec1, double* vec2, double* vec1_x_vec2)
{
	vec1_x_vec2[0] = vec1[1]*vec2[2]-vec1[2]*vec2[1];
	vec1_x_vec2[1] = vec1[2]*vec2[0]-vec1[0]*vec2[2];
	vec1_x_vec2[2] = vec1[0]*vec2[1]-vec1[1]*vec2[0];
}
