/*
 * GetGraspPose.cpp
 *
 *  Created on: Aug 13, 2014
 *      Author: euroc_student
 */

#include <GraspPose.hpp>


GraspPose::GraspPose()
{
	// Initialize object properties
	object_height_ = 0.0;
	object_CoM_.x = 1000.0; // some large number - needed later for error handling
	object_CoM_.y = 1000.0;
	object_CoM_.z = 1000.0;
	idx_shape_CoM_ = 0;

	// Safety margin between highest point of object and waiting position
	safety_margin_obj2wait_ = 0.1;

	// Geometry Data
	ez_0_0_[0]=0.0;
	ez_0_0_[1]=0.0;
	ez_0_0_[2]=1.0;
	gripper_height_ = 0.06;
	safety_distance_= 5.0 / 1000.0;

	// Auxiliary variable for debugging
	ctrl_outp_ = true;
}

GraspPose::~GraspPose() {
	// TODO Auto-generated destructor stub
}

void GraspPose::set_object_data_(am_msgs::Object object) {
	// assign class property "object_"
	object_        = object;
	// quaternion rotation vector describing rotation from 0-FoR to obj-FoR
	quat_02obj_[0] = object.abs_pose.orientation.w;
	quat_02obj_[1] = object.abs_pose.orientation.x;
	quat_02obj_[2] = object.abs_pose.orientation.y;
	quat_02obj_[3] = object.abs_pose.orientation.z;
	// Initialize transformation matrix
	quat2dcm(quat_02obj_,A_obj20_);
	// Allocate memory for r_02shape_0_
	boost::array<double,3> temp = {1000.0, 1000.0, 1000.0};
	for(unsigned idx_shape=0; idx_shape<object_.nr_shapes; idx_shape++)
		r_02shape_0_.push_back(temp);
	// Resize shape_height_
	shape_height_.resize(object_.nr_shapes);
	// Extract position data of sub-shapes
	compute_abs_shape_positions_();
}

// ROS-Service Function
bool GraspPose::return_grasp_pose(am_msgs::GetGraspPose::Request &req, am_msgs::GetGraspPose::Response &res)
{

	// Control output
	if( ctrl_outp_ ) {
		std::cout << "---------------- Control output of GraspPose ------------------" << std::endl;
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
	}

	// Only Prio-1 grasp poses are currently implemented
	unsigned int prio = 1;

	// Update object data
	set_object_data_(req.object);
	// Compute grasp pose
	res.grasp_pose = get_grasp_pose_(prio);

	// Update TF-info for gripper
	transform_gripper.setOrigin(tf::Vector3(res.grasp_pose.position.x,res.grasp_pose.position.y,res.grasp_pose.position.z));
	transform_gripper.setRotation(tf::Quaternion(res.grasp_pose.orientation.x,res.grasp_pose.orientation.y,res.grasp_pose.orientation.z,res.grasp_pose.orientation.w));
	// Update TF-info for object
	transform_object.setOrigin(tf::Vector3(req.object.abs_pose.position.x,req.object.abs_pose.position.y,req.object.abs_pose.position.z));
	tf::Quaternion q;
	q.setW(req.object.abs_pose.orientation.w);
	q.setX(req.object.abs_pose.orientation.x);
	q.setY(req.object.abs_pose.orientation.y);
	q.setZ(req.object.abs_pose.orientation.z);
	transform_object.setRotation(q);
	// Assign other response variables
	res.grasp_width = grasp_width_;
	res.waiting_height = waiting_height_;


	uint status = 1;
	if (status) {
		res.error_message = "";
		return true;
	} else {
		res.error_message = "Failed to return TCP-pose";
		return false;
	}
}

void GraspPose::compute_abs_shape_positions_() {
	// relative vector from reference point of shape-FoR to reference point of object-FoR, given in object-FoR
	tf::Vector3 r_obj2shape_obj(0.0,0.0,0.0);
	// relative vector from reference point of shape-FoR to reference point of object-FoR, given in 0-FoR
	tf::Vector3 r_obj2shape_0(0.0,0.0,0.0);
	// Quaternion for rotation of shape w.r.t object
	double q_obj2shape[4];
	// Pointer at current shape
	am_msgs::Shape* current_shape;
	// Auxiliary quaternion for trafo
	tf::Quaternion q_temp;
	// Trafo from object frame to world-FoR
	tf::Transform tf_k0;
	for( unsigned int idx_shape=0; idx_shape<object_.nr_shapes; idx_shape++ ) {
		current_shape = &object_.shape[idx_shape];
		// Compute shape's absolute position
		// ---------------------------------
		r_obj2shape_obj.setX(current_shape->pose.position.x);
		r_obj2shape_obj.setY(current_shape->pose.position.y);
		r_obj2shape_obj.setZ(current_shape->pose.position.z);

		// Transform relative position from obj to shape (given in obj-frame) into the inertial frame
		q_temp.setW(object_.abs_pose.orientation.w);
		q_temp.setX(object_.abs_pose.orientation.x);
		q_temp.setY(object_.abs_pose.orientation.y);
		q_temp.setZ(object_.abs_pose.orientation.z);
		tf_k0.setRotation(q_temp);
		r_obj2shape_0 = tf_k0(r_obj2shape_obj);

		// Compute absolute position of shape
		r_02shape_0_.at(idx_shape).at(0) = r_obj2shape_0.getX() + object_.abs_pose.position.x;
		r_02shape_0_.at(idx_shape).at(1) = r_obj2shape_0.getY() + object_.abs_pose.position.y;
		r_02shape_0_.at(idx_shape).at(2) = r_obj2shape_0.getZ() + object_.abs_pose.position.z;
		// control output
		if(ctrl_outp_) {
			std::cout << "Vector r_obj2shape of shape " << (*current_shape).type << ", in obj-FoR:" << std::endl;
			std::cout << "x: " << r_obj2shape_obj[0] << std::endl;
			std::cout << "y: " << r_obj2shape_obj[1] << std::endl;
			std::cout << "z: " << r_obj2shape_obj[2] << std::endl;
			std::cout << "Vector r_obj2shape of shape " << (*current_shape).type << ", in 0-FoR (2):" << std::endl;
			std::cout << "x: " << r_obj2shape_0.getX() << std::endl;
			std::cout << "y: " << r_obj2shape_0.getY() << std::endl;
			std::cout << "z: " << r_obj2shape_0.getZ() << std::endl;
			std::cout << "Vector r_02shape of shape " << (*current_shape).type << ", in 0-FoR:" << std::endl;
			std::cout << "x: " << r_02shape_0_.at(idx_shape).at(0) << std::endl;
			std::cout << "y: " << r_02shape_0_.at(idx_shape).at(1) << std::endl;
			std::cout << "z: " << r_02shape_0_.at(idx_shape).at(2) << std::endl;
		}
	}
}

void GraspPose::compute_object_CoM_() {
	if( r_02shape_0_.at(0).at(0)>500 )
		ROS_ERROR("Call method 'compute_abs_shape_positions_()' before 'compute_object_CoM_()'");

	// Reinitialize object_CoM_
	object_CoM_.x = 0.0;
	object_CoM_.y = 0.0;
	object_CoM_.z = 0.0;

	double object_mass_ = 0.0;
	double shape_masses[object_.nr_shapes];
	am_msgs::Shape* current_shape;
	// Dot product auxiliary variables
	double dot_prod, abs_dot_prod_max;
	unsigned int idx_dot_prod_max = 0;

	for(unsigned int idx_shape=0; idx_shape<object_.nr_shapes; idx_shape++ ) {
		// Compute volume & mass of each shape
		// -----------------------------------
		current_shape = &object_.shape[idx_shape];
		double shape_volume = 0.0;
		if( !current_shape->type.compare("box") ) {
			shape_volume = current_shape->size[0] * current_shape->size[1] * current_shape->size[2];
		}
		else if( !current_shape->type.compare("cylinder") ) {
			shape_volume = M_PI*current_shape->radius*current_shape->radius*(*current_shape).length;
			if(ctrl_outp_)
				std::cout << "Shape No " << idx_shape+1 << " : " <<  (*current_shape).type << std::endl;
		}
		if(ctrl_outp_) {
			std::cout << "Shape No " << idx_shape+1 << " : " <<  current_shape->type << std::endl;
			std::cout << "Determined volume of shape " <<  current_shape->type << " :" << shape_volume << std::endl;
			std::cout << "Received density of shape No " << idx_shape+1 << " : " << current_shape->density << std::endl;
			std::cout << "Determined mass of shape No " << idx_shape+1 << " : " << shape_masses[idx_shape] << std::endl;
		}
		shape_masses[idx_shape] = current_shape->density * shape_volume;
		// Compute object mass
		// -------------------
		object_mass_ += shape_masses[idx_shape];

		object_CoM_.x += r_02shape_0_.at(idx_shape).at(0) * shape_masses[idx_shape];
		object_CoM_.y += r_02shape_0_.at(idx_shape).at(1) * shape_masses[idx_shape];
		object_CoM_.z += r_02shape_0_.at(idx_shape).at(2) * shape_masses[idx_shape];
	}

	if( object_mass_ == 0.0 )
		ROS_ERROR("Mass of object could not be computed");

	object_CoM_.x = object_CoM_.x / object_mass_;
	object_CoM_.y = object_CoM_.y / object_mass_;
	object_CoM_.z = object_CoM_.z / object_mass_;

	// Control output
	if(ctrl_outp_) {
		std::cout << "Computed object-CoM (0-FoR): " << std::endl;
		std::cout << "x: " << object_CoM_.x << std::endl;
		std::cout << "y: " << object_CoM_.y << std::endl;
		std::cout << "z: " << object_CoM_.z << std::endl;
	}
}

void GraspPose::compute_idx_shape_CoM_() {
	if( object_CoM_.x>500 )
			ROS_ERROR("Call method 'compute_object_CoM_()' before 'compute_idx_shape_CoM_()'");

	// For objects with several shapes, find shape closest to object-CoM
	if( object_.nr_shapes > 1) {
		double dr[3];
		double CoM_temp[3] = {object_CoM_.x, object_CoM_.y, object_CoM_.z};
		double length_dr;
		double length_dr_min = 100.0;
		unsigned int idx_min = 0;
		for(unsigned idx_shape=0; idx_shape<object_.nr_shapes; idx_shape++) {
			length_dr = 0.0;
			for(unsigned i=0; i<3; i++) {
				dr[i] = r_02shape_0_.at(idx_shape).at(i) - CoM_temp[i];
				length_dr += dr[i]*dr[i];
			}
			length_dr = sqrt(length_dr);
			if(length_dr < length_dr_min) {
				idx_min = idx_shape;
				length_dr_min = length_dr;
			}
			// Control output
			if(ctrl_outp_) {
				std::cout << "Distance between ref. point of shape at CoM (Type: " << object_.shape[idx_shape].type << ") and object-CoM:" << std::endl;
				std::cout << "x: " << dr[0] << std::endl;
				std::cout << "y: " << dr[1] << std::endl;
				std::cout << "z: " << dr[2] << std::endl;
			}
		}
		idx_shape_CoM_ = idx_min;
	}
	else
		idx_shape_CoM_ = 0;
}

void GraspPose::compute_object_height_() {
	if( r_02shape_0_.at(0).at(0)>500 )
			ROS_ERROR("Call method 'compute_abs_shape_positions_()' before 'compute_object_height_()'");

	am_msgs::Shape* current_shape;
	// Quaternion for rotation of shape w.r.t object
	double q_obj2shape[4];
	// Rotation matrix converting a vector from the shape frame into the object frame
	double A_shape2obj[9]; // A_obj.shape
	// Rotation matrix converting a vector from the object frame into the 0-frame
	double A_shape20[9]; // A_0.shape
	// base vectors of shape-FoR w.r.t 0-FoR
	std::vector<boost::array<double,3> > shape_base_vectors(object_.nr_shapes);
	double dot_prod, abs_dot_prod_max;
	unsigned int idx_dot_prod_max = 0;

	// Reset object_height_
	object_height_ = 0.0;

	for(unsigned int idx_shape=0; idx_shape<object_.nr_shapes; idx_shape++ ) {
		// Compute object height
		// ---------------------
		current_shape = &object_.shape[idx_shape];
		// First, determine height of current shape
		// Find shape-axis pointing "upwards" (along ez_0)
		//		1) Get base vectors of current shape written in 0-FoR
		q_obj2shape[0] = current_shape->pose.orientation.w;
		q_obj2shape[1] = current_shape->pose.orientation.x;
		q_obj2shape[2] = current_shape->pose.orientation.y;
		q_obj2shape[3] = current_shape->pose.orientation.z;
		quat2dcm(q_obj2shape,A_shape2obj);
		matrix_multiply_(A_obj20_,A_shape2obj,A_shape20);
		shape_base_vectors = get_base_vectors_(A_shape20,0);
		//		2) Find shape base-vector which has largest dot-product with ez_0_0
		abs_dot_prod_max = -10.0;
		for(unsigned i=0; i<3; i++) {
			dot_prod = 0.0;
			for(unsigned j=0; j<3; j++)
				dot_prod += ez_0_0_[j] * shape_base_vectors.at(i).at(j);
			if(fabs(dot_prod)>abs_dot_prod_max){
				abs_dot_prod_max = fabs(dot_prod);
				idx_dot_prod_max = i;
			}
		}
		// Determine shape height
		if( !current_shape->type.compare("box") )
			shape_height_.at(idx_shape) = r_02shape_0_.at(idx_shape).at(2) + (current_shape->size[idx_dot_prod_max] / 2.0) * fabs(shape_base_vectors.at(idx_dot_prod_max).at(2));
		else if( !current_shape->type.compare("cylinder") ) {
			if( idx_dot_prod_max == 2 )
				shape_height_.at(idx_shape) = r_02shape_0_.at(idx_shape).at(2) + (current_shape->length / 2.0) * fabs(shape_base_vectors.at(idx_dot_prod_max).at(2));
			else
				shape_height_.at(idx_shape) = r_02shape_0_.at(idx_shape).at(2) + current_shape->radius * fabs(shape_base_vectors.at(idx_dot_prod_max).at(2));
		}
		// Control output
		std::cout << "Height of shape " << current_shape->type << ": " << shape_height_.at(idx_shape) << std::endl;
		if(object_height_ < shape_height_.at(idx_shape))
			object_height_ = shape_height_.at(idx_shape);

	}
	waiting_height_ = object_height_ + safety_margin_obj2wait_;
}

void GraspPose::compute_grasp_pose_(unsigned int prio) {
	if( prio != 1)
		ROS_ERROR("Currently only Prio-1 pose is implemented");

	compute_abs_shape_positions_();
	compute_object_CoM_();
	compute_idx_shape_CoM_();
	compute_object_height_();

	// Rotation matrix converting a vector from the shape frame into the object frame
	double A_shape2obj[9]; // A_obj.shape
	// Rotation matrix converting a vector from the object frame into the 0-frame
	double A_shape20[9]; // A_0.shape
	// object height
	double object_height = 0.0;
	// shape heights
	double shape_height[object_.nr_shapes];
	// safety-margin between object height & waiting height
	double safety_margin_obj2wait = 0.1;
	// Quaternion for rotation of shape w.r.t object
	double q_obj2shape[4];

	double dot_prod, abs_dot_prod_max;
	unsigned int idx_dot_prod_max = 0;

	// Convert object-quaternion to rotation matrix
	q_obj2shape[0] = object_.shape[idx_shape_CoM_].pose.orientation.w;
	q_obj2shape[1] = object_.shape[idx_shape_CoM_].pose.orientation.x;
	q_obj2shape[2] = object_.shape[idx_shape_CoM_].pose.orientation.y;
	q_obj2shape[3] = object_.shape[idx_shape_CoM_].pose.orientation.z;
	quat2dcm(q_obj2shape,A_shape2obj);
	// Compute rotation matrix A_0.shape
	matrix_multiply_(A_obj20_,A_shape2obj,A_shape20);

	if(ctrl_outp_) {
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
			std::cout << quat_02obj_[i] << std::endl;
		std::cout << "A_0.obj = " << std::endl;
		for (unsigned int i = 0; i < 3; i++) {
			for (unsigned int j = 0; j < 3; j++)
				std::cout << A_obj20_[i * 3 + j] << "     ";
			std::cout << std::endl;
		}

		// Control Output
		std::cout << "A_0.shape = " << std::endl;
		for (unsigned int i = 0; i < 3; i++) {
			for (unsigned int j = 0; j < 3; j++)
				std::cout << A_shape20[i * 3 + j] << "     ";
			std::cout << std::endl;
		}
	}

	// Extract base-vectors
	std::vector<boost::array<double,3> > shape_at_CoM_base_vectors(3);
	shape_at_CoM_base_vectors = get_base_vectors_( A_shape20, 0 );


	// Find shape-axis parallel to z-axis of table coordinate frame:
	// ei_0_obj = arg( max_j(abs( ez_0_0' * ej_0_obj )) )
	double sgn_dot_prod_max;
	for(unsigned i=0; i<3; i++) {
		// Compute vector dot-product
		for(unsigned j=0; j<3; j++)
			dot_prod += ez_0_0_[j] * shape_at_CoM_base_vectors.at(i)[j];
		// Find maximum absolute value of dot products
		if(fabs(dot_prod) > abs_dot_prod_max) {
			abs_dot_prod_max = fabs(dot_prod);
			idx_dot_prod_max = i;
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

	// Search among other two-object axis to determine gripper x-axis
	double ex_0_GP[3] = { 0.0, 0.0, 0.0 };
	for (unsigned i = 0; i < 3; i++) {
		if (i != idx_dot_prod_max) { // Search among other axes
			if (!object_.shape[idx_shape_CoM_].type.compare("cylinder")) {
				// Distinguish between the two cases cylinder pointing upwards and cylinder lying horizontally
				if (idx_dot_prod_max == 2) { // cylinder pointing upwards
					// This must be replaced, e.g. with old gripper x-axis (coz no rotation is necessary)
					ex_0_GP[0] = 1.0; // any direction
					ex_0_GP[1] = 0.0;
					ex_0_GP[2] = 0.0;
				} else { // Cylinder lying horizontally
					// desired gripper x-axis is perpendicular to object z-axis and to table z-axis
					cross_product_(&shape_at_CoM_base_vectors.at(2)[0], (double*)ez_0_0_, ex_0_GP);
					// might have to change vector orientation later to avoid unnecessary rotations
				}
				// In either case grasp_width = 2*radius
				grasp_width_ = 2.0 * object_.shape[idx_shape_CoM_].radius;
			} else if (!object_.shape[idx_shape_CoM_].type.compare("box")) {
				// loop through dimensions and find smaller of the two dimension (this way the algorithm can also handle non-square cross-sections)
				double min_length = 100.0;
				unsigned int idx_min_length = 4;
				for (unsigned j = 0; j < 3; j++) {
					if ((j != idx_dot_prod_max) && (min_length >= object_.shape[idx_shape_CoM_].size[j])) {
						idx_min_length = j;
						min_length = object_.shape[idx_shape_CoM_].size[j];
					}
				}
				if (min_length > 99.9)
					ROS_ERROR("Could not find smallest length of red_cube");
				if (idx_min_length == 4)
					ROS_ERROR("Could not determine idx for smallest length");
				grasp_width_ = min_length;
				ex_0_GP[0] = shape_at_CoM_base_vectors.at(idx_min_length)[0];
				ex_0_GP[1] = shape_at_CoM_base_vectors.at(idx_min_length)[1];
				ex_0_GP[2] = shape_at_CoM_base_vectors.at(idx_min_length)[2];
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
	cross_product_(ez_0_GP, ex_0_GP, ey_0_GP);

	// Control output
	std::cout << "Determined desired gripper y-axis:" << std::endl;
	for( unsigned i = 0; i<3; i++ )
		std::cout << ey_0_GP[i] << std::endl;

	//---------------------------------------------------------------------------------------//
	//                            Set GP-TCP target position                                 //
	//---------------------------------------------------------------------------------------//
	GPTCP_target_pose_.position.x = object_CoM_.x;
	GPTCP_target_pose_.position.y = object_CoM_.y;
	// Add z-distance between x_0-y_0 plane and GP-TCP
	// Watch out!!! Assuming that desired GP-orientation points towards table, i.e. ez_GP_0 = - ez_0_0
	// (would need transformation of ez_GP_0 into 0-FoR later)
	// Default z-distance (ensures that CoM is between gripper-jaws)

	double dz_0GPTCP = gripper_height_ / 2.0 + object_CoM_.z;
	if( dz_0GPTCP < safety_distance_ + object_height_ )
		dz_0GPTCP = safety_distance_ + object_height_;
	if( dz_0GPTCP < safety_distance_ + gripper_height_ )
		dz_0GPTCP = safety_distance_ + gripper_height_;

	if( object_CoM_.z < dz_0GPTCP - gripper_height_ )
		ROS_WARN("CoM not between gripper jaws");

	GPTCP_target_pose_.position.z = dz_0GPTCP;


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

	matrix_transpose_(A_GP20,A_02GP);
	if(ctrl_outp_) {
		std::cout << "A_GP20 = " << std::endl;
		for (unsigned int i = 0; i < 3; i++) {
			for (unsigned int j = 0; j < 3; j++)
				std::cout << A_GP20[i * 3 + j] << "     ";
			std::cout << std::endl;
		}
		std::cout << "A_02GP = " << std::endl;
		for (unsigned int i = 0; i < 3; i++) {
			for (unsigned int j = 0; j < 3; j++)
				std::cout << A_02GP[i * 3 + j] << "     ";
			std::cout << std::endl;
		}
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

	GPTCP_target_pose_.orientation = target_orientation;

	std::cout << "-- GPTCP_target_pose_:" << std::endl;
	std::cout << GPTCP_target_pose_ << std::endl;
	//---------------------------------------------------------------------------------------//
	//               Transform target pose from GPTCP-FoR to LWRTCP-FoR                      //
	//---------------------------------------------------------------------------------------//
	LWRTCP_target_pose_ = transform_grasp_pose_GPTCP_2_LWRTCP_();

	/// Delete this code
	std::cout << "Using tf's: " << std::endl;
	std::cout << "LWRTCP_target_pose.position.x=" << LWRTCP_target_pose_.position.x << std::endl;
	std::cout << "LWRTCP_target_pose.position.y=" << LWRTCP_target_pose_.position.y << std::endl;
	std::cout << "LWRTCP_target_pose.position.z=" << LWRTCP_target_pose_.position.z << std::endl;

	const double dr_tcp=0.173; // For debugging of tf-Trafo
	std::cout << "Using brain: " << std::endl;
	std::cout << "LWRTCP_target_pose.position.x=" << GPTCP_target_pose_.position.x << std::endl;
	std::cout << "LWRTCP_target_pose.position.y=" << GPTCP_target_pose_.position.y << std::endl;
	std::cout << "LWRTCP_target_pose.position.z=" << GPTCP_target_pose_.position.z + dr_tcp << std::endl;
    /// end delete

	std::cout << "Desired LWR-TCP position: " << std::endl;
	std::cout << "x: " << LWRTCP_target_pose_.position.x << std::endl;
	std::cout << "y: " << LWRTCP_target_pose_.position.y << std::endl;
	std::cout << "z: " << LWRTCP_target_pose_.position.z << std::endl;
	std::cout << "Desired gripper quaternion: " << std::endl;
	std::cout << "x: " << LWRTCP_target_pose_.orientation.x << std::endl;
	std::cout << "y: " << LWRTCP_target_pose_.orientation.y << std::endl;
	std::cout << "z: " << LWRTCP_target_pose_.orientation.z << std::endl;
	std::cout << "w: " << LWRTCP_target_pose_.orientation.w << std::endl;
}

geometry_msgs::Pose GraspPose::transform_grasp_pose_GPTCP_2_LWRTCP_() {
	tf::TransformListener tf_listener;
	tf::StampedTransform transform_GPTCP_2_LWRTCP;

	tf::Stamped<tf::Vector3> GPTCP_2_target_vec;
	GPTCP_2_target_vec.setX(GPTCP_target_pose_.position.x);
	GPTCP_2_target_vec.setY(GPTCP_target_pose_.position.y);
	GPTCP_2_target_vec.setZ(GPTCP_target_pose_.position.z);
	ros::Time now = ros::Time::now();
	try{
		tf_listener.waitForTransform(LWR_TCP,GP_TCP,now,ros::Duration(2.0));
		tf_listener.lookupTransform(LWR_TCP,GP_TCP,ros::Time(0),transform_GPTCP_2_LWRTCP);
		ROS_INFO("Listening to transform was successful");
	}
	catch(...){
		ROS_ERROR("Listening to transform was not successful");
	}
	tf::Vector3 ret_vec = transform_GPTCP_2_LWRTCP(GPTCP_2_target_vec);

	geometry_msgs::Pose LWRTCP_pose;
	LWRTCP_pose.position.x = ret_vec.getX();
	LWRTCP_pose.position.y = ret_vec.getY();
	LWRTCP_pose.position.z = ret_vec.getZ();

	// TODO : use tf to transform quaternion
	LWRTCP_pose.orientation.w =  GPTCP_target_pose_.orientation.w;
	LWRTCP_pose.orientation.x =  GPTCP_target_pose_.orientation.x;
	LWRTCP_pose.orientation.y =  GPTCP_target_pose_.orientation.y;
	LWRTCP_pose.orientation.z =  GPTCP_target_pose_.orientation.z;

	return LWRTCP_pose;
}

geometry_msgs::Pose GraspPose::get_grasp_pose_(unsigned int prio) {
	if( prio != 1)
		ROS_ERROR("Currently only Prio-1 pose is implemented");
	compute_grasp_pose_(prio);
	return LWRTCP_target_pose_;
}

void GraspPose::cross_product_(double* vec1, double* vec2, double* vec1_x_vec2) {
	vec1_x_vec2[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
	vec1_x_vec2[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
	vec1_x_vec2[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
}


void GraspPose::matrix_multiply_(double* M1, double* M2, double* Result) {
	for (unsigned row = 0; row < 3; row++) {
		for (unsigned col = 0; col < 3; col++) {
			Result[row * 3 + col] = 0;
			for (unsigned i = 0; i < 3; i++)
				Result[row * 3 + col] += M1[row * 3 + i] * M2[i * 3 + col];
		}
	}
}

void GraspPose::matrix_transpose_(double* A, double* At) {
	unsigned int idx_At[9] = { 0, 3, 6, 1, 4, 7, 2, 5, 8 };
	for (unsigned i = 0; i < 9; i++)
		At[idx_At[i]] = A[i];
}


std::vector<boost::array<double,3> > GraspPose::get_base_vectors_( double* A_k20, bool verbose ) {
	if(verbose){
		std::cout << "-------------------------------------" << std::endl;
		std::cout << "Control Output of 'get_base_vectors'" << std::endl;
	}
	// Copy columns from rotational matrix into a vector of boost arrays
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

	if(verbose)
		std::cout << "-------------------------------------" << std::endl;

	return base_vectors;
}


