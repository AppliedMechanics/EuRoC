/*
 * GetGraspPose2.cpp
 *
 *  Created on: Aug 13, 2014
 *      Author: euroc_student
 */

#include <GraspPose2.hpp>


GraspPose2::GraspPose2()
{
	gripper_maxwidth_=0.06;	//Linear axis goes from 0 to 0.07, so there is a minimum safety distance of 0.01 between finger and object.
	gripper_height_=0.06;
	gripper_finger_width=0.03;
	grip_safety_dist_=0.01;
	vertical_handle_safe_z_offset=0.01;
	vision_distance_object_height_cube_=0.2;
	vision_distance_object_height_cylinder_=0.2;
	vision_distance_object_height_handle_=0.2;
	place_falling_dist_=0.01;
}

GraspPose2::~GraspPose2() {
	// TODO Auto-generated destructor stub
}

void GraspPose2::set_object_data_(am_msgs::Object object, am_msgs::TargetZone target_zone) {
	// assign class property "object_"
	object_ = object;
	target_zone_ = target_zone;

	dist_COM_shape_.resize(object_.nr_shapes);
	b_transform_shapes_.resize(object_.nr_shapes);
	o_transform_shapes_.resize(object_.nr_shapes);

	object_type_=OBJECT_UNKNOWN;
	if(object_.nr_shapes==1 && (!object_.shape[0].type.compare("cylinder")))
	{
		object_type_=OBJECT_CYLINDER;
	}
	if(object_.nr_shapes==1 && (!object_.shape[0].type.compare("box")))
	{
		object_type_=OBJECT_CUBE;
	}
	if(object_.nr_shapes==3)
	{
		object_type_=OBJECT_HANDLE;
	}
}

// ROS-Service Function
bool GraspPose2::return_grasp_pose(am_msgs::GetGraspPose::Request &req, am_msgs::GetGraspPose::Response &res)
{

	// Control output

	// Only Prio-1 grasp poses are currently implemented
	unsigned int prio = 1;

	// Update object data
	set_object_data_(req.object, req.target_zone);
	compute_object_CoM_();
	compute_idx_shape_CoM_();
	compute_abs_shape_poses_();
	compute_object_height_();

	//old
	compute_grasp_pose_();

	//new
	compute_grasp_poses_();

	//old
	transform_grasp_pose_GPTCP_2_LWRTCP_();

	//new
	transform_GPTCP_2_LWRTCP_=get_transform_GPTCP_2_LWRTCP();
	LWRTCP_object_grip_pose.clear();
	LWRTCP_object_safe_pose.clear();
	LWRTCP_object_vision_pose.clear();
	LWRTCP_target_place_pose.clear();
	LWRTCP_target_safe_pose.clear();
	LWRTCP_target_vision_pose.clear();
	for(int ii=0; ii<GPTCP_object_grip_pose.size();ii++)
		{ LWRTCP_object_grip_pose.push_back(transform_pose_GPTCP_2_LWRTCP_(GPTCP_object_grip_pose[ii])); }
	for(int ii=0; ii<GPTCP_object_safe_pose.size();ii++)
		{ LWRTCP_object_safe_pose.push_back(transform_pose_GPTCP_2_LWRTCP_(GPTCP_object_safe_pose[ii])); }
	for(int ii=0; ii<GPTCP_object_vision_pose.size();ii++)
		{ LWRTCP_object_vision_pose.push_back(transform_pose_GPTCP_2_LWRTCP_(GPTCP_object_vision_pose[ii])); }
	for(int ii=0; ii<GPTCP_target_place_pose.size();ii++)
		{ LWRTCP_target_place_pose.push_back(transform_pose_GPTCP_2_LWRTCP_(GPTCP_target_place_pose[ii])); }
	for(int ii=0; ii<GPTCP_target_safe_pose.size();ii++)
		{ LWRTCP_target_safe_pose.push_back(transform_pose_GPTCP_2_LWRTCP_(GPTCP_target_safe_pose[ii])); }
	for(int ii=0; ii<GPTCP_target_vision_pose.size();ii++)
		{ LWRTCP_target_vision_pose.push_back(transform_pose_GPTCP_2_LWRTCP_(GPTCP_target_vision_pose[ii])); }
	compute_relative_vectors_();

	//print some infos
	ROS_INFO("Nr of shape %i",object_.nr_shapes);
	ROS_INFO("CoM part of shape %i",com_idx_);

	// Save everything to response
	res.object_mass = object_mass_;
	res.object_grip_pose=LWRTCP_object_grip_pose;
	res.object_safe_pose=LWRTCP_object_safe_pose;
	res.object_vision_pose=LWRTCP_object_vision_pose;
	res.object_pose_type=object_pose_type;
	res.object_grasp_width=object_grasp_width;
	res.target_place_pose=LWRTCP_target_place_pose;
	res.target_safe_pose=LWRTCP_target_safe_pose;
	res.target_vision_pose=LWRTCP_target_vision_pose;
	res.target_pose_type=target_pose_type;
	res.object_grip_r_tcp_com=object_grip_r_tcp_com;
	res.object_grip_r_gp_com=object_grip_r_gp_com;
	res.object_grip_r_gp_obj=object_grip_r_gp_obj;

	//Print results of service call
	print_results();

	if (!first_called)
		first_called =true;

	return true;
}

void GraspPose2::compute_abs_shape_poses_()
{
	for (int ii=0;ii<object_.nr_shapes;ii++)
	{
		b_transform_shapes_[ii].setOrigin(tf::Vector3(object_.shape[ii].pose.position.x,
				object_.shape[ii].pose.position.y,
				object_.shape[ii].pose.position.z));
		b_transform_shapes_[ii].setRotation(tf::Quaternion(object_.shape[ii].pose.orientation.x,
				object_.shape[ii].pose.orientation.y,
				object_.shape[ii].pose.orientation.z,
				object_.shape[ii].pose.orientation.w));
		o_transform_shapes_[ii] = transform_object*b_transform_shapes_[ii];
		std::cout << "-------------- Shape "<<ii<<"------------"<<std::endl;
		std::cout << "[ "<<o_transform_shapes_[ii].getOrigin().getX()<<" "<<o_transform_shapes_[ii].getOrigin().getY()<<" "<<
				o_transform_shapes_[ii].getOrigin().getZ()<<" ]"<<std::endl;
	}
	std::cout << "-------------- END SHAPES ------------"<<std::endl;
}

void GraspPose2::compute_object_CoM_() {

	object_mass_ = 0;
	b_object_com_.setValue(0,0,0);
	double shape_mass = 0;

	for (int ii=0;ii<object_.nr_shapes;ii++)
	{
		shape_com_.setX(object_.shape[ii].pose.position.x);
		shape_com_.setY(object_.shape[ii].pose.position.y);
		shape_com_.setZ(object_.shape[ii].pose.position.z);

		if(!object_.shape[ii].type.compare("cylinder"))
		{
			shape_mass = M_PI*object_.shape[ii].radius*object_.shape[ii].radius*object_.shape[ii].length*object_.shape[ii].density;
		}
		else if (!object_.shape[ii].type.compare("box"))
		{
			shape_mass = object_.shape[ii].size[0]*object_.shape[ii].size[1]*object_.shape[ii].size[2]*object_.shape[ii].density;
		}
		else
		{
			msg_error("Undefined type.");
		}

		object_mass_ += shape_mass;
		b_object_com_ += shape_mass*shape_com_;
	}
	b_object_com_ = 1.0/object_mass_ * b_object_com_;

	// Transform com to world coordinates
	transform_object.setOrigin(tf::Vector3(object_.abs_pose.position.x,object_.abs_pose.position.y,object_.abs_pose.position.z));
	transform_object.setRotation(tf::Quaternion(object_.abs_pose.orientation.x,
			object_.abs_pose.orientation.y,
			object_.abs_pose.orientation.z,
			object_.abs_pose.orientation.w));
	o_object_com_ = transform_object(b_object_com_);

	//! Checked with blue handle, Task v1
	std::cout<<"Object CoM"<<o_object_com_[0]<<" "<<o_object_com_[1]<<" "<<o_object_com_[2]<<std::endl;
	ROS_INFO("Object Mass: %f kg",object_mass_);
}

void GraspPose2::compute_idx_shape_CoM_() {
	// Calculate Object shape idx where to grasp...
	if (object_.nr_shapes > 1)
	{
		com_idx_ = -1;
		for (int ii=0;ii<object_.nr_shapes;ii++)
		{
			shape_com_.setX(object_.shape[ii].pose.position.x);
			shape_com_.setY(object_.shape[ii].pose.position.y);
			shape_com_.setZ(object_.shape[ii].pose.position.z);
			conn_ = b_object_com_ - shape_com_;
			dist_COM_shape_[ii] = conn_.length();

			if(!object_.shape[ii].type.compare("cylinder"))
			{
				if ((conn_.getX()*conn_.getX()+conn_.getY()*conn_.getY()<=object_.shape[ii].radius*object_.shape[ii].radius) &&
						(am_abs(conn_.getZ()) <= 0.5*object_.shape[ii].length))
				{
					if (com_idx_ <0)
						com_idx_ = ii;
					else
						msg_warn("CoM part of two shapes??");
				}
			}
			else if (!object_.shape[ii].type.compare("box"))
			{
				if ((am_abs(conn_.getX())<=0.5*object_.shape[ii].size[0]) &&
					(am_abs(conn_.getY())<=0.5*object_.shape[ii].size[1]) &&
					(am_abs(conn_.getZ())<=0.5*object_.shape[ii].size[2]))
				{
					if (com_idx_ <0)
						com_idx_ = ii;
					else
						msg_warn("CoM part of two shapes??");
				}
			}
			else
			{
				msg_error("Undefined type.");
			}

//			if(!object_.shape[ii].type.compare("cylinder"))
//			{
//				if (conn_.closestAxis() != 2)
//					dist_COM_shape_[ii] = conn_.length() - (0.5*object_.shape[ii].length);
//				else
//					dist_COM_shape_[ii] = conn_.length() - object_.shape[ii].radius;
//			}
//			else if (!object_.shape[ii].type.compare("box"))
//			{
//				dist_COM_shape_[ii] = conn_.length() - 0.5*object_.shape[ii].size[conn_.closestAxis()];
//			}
//			else
//				msg_error("Undefined type.");
//			if (dist_COM_shape_[ii]<0)
//			{
//				if (com_idx_ <0)
//					com_idx_ = ii;
//				else
//					msg_warn("CoM part of two shapes??");
//			}
		}
		if (com_idx_ <0)
		{
			msg_warn("CoM outside of a shape? Calculating closest shape...");
			int min_idx=0;
			for (int jj = 1;jj<object_.nr_shapes;jj++){
				if (dist_COM_shape_[jj]<dist_COM_shape_[jj-1])
					min_idx = jj;
			}
			com_idx_ = min_idx;
		}
	}
	else {
		com_idx_=0;
		dist_COM_shape_[0] = 0.0;
	}
}

void GraspPose2::compute_object_height_() {

	object_height_ = 0;
	double tmp_oh = 0;
	double dot_product;
	tf::Vector3 z_axis(0,0,1);


	for (int ii=0;ii<object_.nr_shapes;ii++)
	{
		if(!object_.shape[ii].type.compare("cylinder"))
		{
			dot_product = o_transform_shapes_[ii].getBasis().getColumn(2).dot(z_axis);
			std::cout<<"x-axis "<<o_transform_shapes_[ii].getBasis().getColumn(0).getX()<<" "<<o_transform_shapes_[ii].getBasis().getColumn(0).getY()<<" "<<o_transform_shapes_[ii].getBasis().getColumn(0).getZ()<<std::endl;
			std::cout<<"y-axis "<<o_transform_shapes_[ii].getBasis().getColumn(1).getX()<<" "<<o_transform_shapes_[ii].getBasis().getColumn(1).getY()<<" "<<o_transform_shapes_[ii].getBasis().getColumn(1).getZ()<<std::endl;
			std::cout<<"z-axis "<<o_transform_shapes_[ii].getBasis().getColumn(2).getX()<<" "<<o_transform_shapes_[ii].getBasis().getColumn(2).getY()<<" "<<o_transform_shapes_[ii].getBasis().getColumn(2).getZ()<<std::endl;
			std::cout<<"dot product "<<o_transform_shapes_[ii].getBasis().getColumn(2).dot(z_axis)<<" "<<dot_product<<std::endl;
			if (dot_product > -0.7 && dot_product < 0.7){
				tmp_oh = o_transform_shapes_[ii].getOrigin().getZ() + object_.shape[ii].radius;
				msg_info("Cylinder is horizontal.");
			}
			else
			{
				tmp_oh = o_transform_shapes_[ii].getOrigin().getZ() + (object_.shape[ii].length*0.5);
				msg_info("Cylinder is standing upwards.");
			}
		}
		else if (!object_.shape[ii].type.compare("box"))
		{
			tmp_oh = o_transform_shapes_[ii].getOrigin().getZ() + 0.5*object_.shape[ii].size[o_transform_shapes_[ii].getRotation().getAxis().closestAxis()];
		}
		else
			msg_error("Undefined type.");

		if (tmp_oh > object_height_){
			object_height_ = tmp_oh;
			high_idx_ = ii;
		}
	}
	ROS_INFO("object height is %f",object_height_);
}

void GraspPose2::compute_grasp_pose_() {

	double dot_product;
	tf::Vector3 z_axis(0,0,1);
	tf::Vector3 x_obj, y_obj, z_obj;
	tf::Vector3 z_grp, x_grp, y_grp;

	x_obj = o_transform_shapes_[com_idx_].getBasis().getColumn(0);
	y_obj = o_transform_shapes_[com_idx_].getBasis().getColumn(1);
	z_obj = o_transform_shapes_[com_idx_].getBasis().getColumn(2);

	GPTCP_target_pose_.position.x = o_object_com_.getX();
	GPTCP_target_pose_.position.y = o_object_com_.getY();
	if (high_idx_ == com_idx_)
		GPTCP_target_pose_.position.z = object_height_ - 0.5*gripper_height_ + grip_safety_dist_;
	else
		GPTCP_target_pose_.position.z = o_object_com_.getZ() + grip_safety_dist_;

	if (GPTCP_target_pose_.position.z<0.04)
		GPTCP_target_pose_.position.z = 0.04;

	double roll,pitch,yaw,dummy1,dummy2;
	tf::Matrix3x3 dcm;
	tf::Quaternion q_tmp;
	bool cyl_vert = false;


	if(!object_.shape[com_idx_].type.compare("cylinder"))
	{
		dot_product = z_obj.dot(z_axis);
		grasp_width_ = object_.shape[com_idx_].radius*2.0;
		if (dot_product > -0.7 && dot_product < 0.7)
		{
			ROS_INFO("horizontal cylinder");
			//dcm.setRotation(o_transform_shapes_[com_idx_].getRotation());
			//! Revised Version
			// Set z pointing down, set y along z of object, calculate x
			z_grp = -1.0*z_axis;
			y_grp = z_obj;
			x_grp = y_grp.cross(z_grp);

			cyl_vert = true;
		}
		else //! Cylinder is horizontal
		{
			ROS_INFO("vertical cylinder");
			//			roll = 0;
			//			pitch = M_PI;
			//			yaw = 0.0; //TODO!
			//			yaw = atan2(am_abs(GPTCP_target_pose_.position.y),am_abs(GPTCP_target_pose_.position.x));
			z_grp = -1.0*z_axis;
			y_grp.setX(GPTCP_target_pose_.position.x);
			y_grp.setY(GPTCP_target_pose_.position.y);
			y_grp.setZ(0);
			x_grp = y_grp.cross(z_grp);
		}
//		ROS_INFO("Cylinder RPY = %f,%f,%f",roll,pitch,yaw);
	}
	else if (!object_.shape[com_idx_].type.compare("box"))
	{
		//		dcm.setRotation(o_transform_shapes_[com_idx_].getRotation());
		//		std::cout<<"x-axis "<<o_transform_shapes_[com_idx_].getBasis().getColumn(0).getX()<<" "<<o_transform_shapes_[com_idx_].getBasis().getColumn(0).getY()<<" "<<o_transform_shapes_[com_idx_].getBasis().getColumn(0).getZ()<<std::endl;
		//		std::cout<<"y-axis "<<o_transform_shapes_[com_idx_].getBasis().getColumn(1).getX()<<" "<<o_transform_shapes_[com_idx_].getBasis().getColumn(1).getY()<<" "<<o_transform_shapes_[com_idx_].getBasis().getColumn(1).getZ()<<std::endl;
		//		std::cout<<"z-axis "<<o_transform_shapes_[com_idx_].getBasis().getColumn(2).getX()<<" "<<o_transform_shapes_[com_idx_].getBasis().getColumn(2).getY()<<" "<<o_transform_shapes_[com_idx_].getBasis().getColumn(2).getZ()<<std::endl;
		//		if (am_abs(o_transform_shapes_[com_idx_].getBasis().getColumn(0).dot(z_axis))>0.9)
		//		{
		//			ROS_INFO("x_axis pointing up");
		//			dcm.getRPY(yaw,dummy1,dummy2);
		//		}
		//		if (am_abs(o_transform_shapes_[com_idx_].getBasis().getColumn(1).dot(z_axis))>0.9)
		//		{
		//			ROS_INFO("y_axis pointing up");
		//			dcm.getRPY(dummy1,yaw,dummy2);
		//		}
		//		if (am_abs(o_transform_shapes_[com_idx_].getBasis().getColumn(2).dot(z_axis))>0.9)
		//		{
		//			ROS_INFO("z_axis pointing up");
		//			dcm.getRPY(dummy1,dummy2,yaw);
		//		}
		z_grp = -1.0*z_axis;
		if (am_abs(x_obj.dot(z_axis))>0.9)
		{
			ROS_INFO("x_axis pointing up");
			if (object_.shape[com_idx_].size[1]>=object_.shape[com_idx_].size[2])
			{
				grasp_width_ = object_.shape[com_idx_].size[2];
				y_grp = y_obj;
			}
			else
			{
				grasp_width_ = object_.shape[com_idx_].size[1];
				y_grp = z_obj;
			}
		}
		if (am_abs(y_obj.dot(z_axis))>0.9)
		{
			ROS_INFO("y_axis pointing up");
			if (object_.shape[com_idx_].size[0]>=object_.shape[com_idx_].size[2])
			{
				grasp_width_ = object_.shape[com_idx_].size[2];
				y_grp = x_obj;
			}
			else
			{
				grasp_width_ = object_.shape[com_idx_].size[0];
				y_grp = z_obj;
			}
		}
		if (am_abs(z_obj.dot(z_axis))>0.9)
		{
			ROS_INFO("z_axis pointing up");
			if (object_.shape[com_idx_].size[0]>=object_.shape[com_idx_].size[1])
			{
				grasp_width_ = object_.shape[com_idx_].size[1];
				y_grp = x_obj;
			}
			else
			{
				grasp_width_ = object_.shape[com_idx_].size[0];
				y_grp = y_obj;
			}
		}
		x_grp = y_grp.cross(z_grp);

	}

	//! Choose roll, pitch dependent on position
	//	if (!cyl_vert)
	//	{
	//		if (am_abs(GPTCP_target_pose_.position.x)>am_abs(GPTCP_target_pose_.position.y))
	//		{
	//			roll = 0;
	//			if (GPTCP_target_pose_.position.x > 0)
	//				pitch = M_PI;
	//			else
	//				pitch = -M_PI;
	//		}
	//		else
	//		{
	//			pitch = 0;
	//			if (GPTCP_target_pose_.position.y > 0)
	//				roll = -M_PI;
	//			else
	//				roll = M_PI;
	//		}
	//	}

	//dcm.setValue(x_grp.getX(),x_grp.getY(),x_grp.getZ(),y_grp.getX(),y_grp.getY(),y_grp.getZ(),z_grp.getX(),z_grp.getY(),z_grp.getZ());
	//transposed matrix seems to be the right one :)
	dcm.setValue(x_grp.getX(),y_grp.getX(),z_grp.getX(),x_grp.getY(),y_grp.getY(),z_grp.getY(), x_grp.getZ(),y_grp.getZ(),z_grp.getZ());
	dcm.getRPY(roll,pitch,yaw);
	ROS_INFO("RPY = %f,%f,%f",roll,pitch,yaw);

	//! Choose yaw always  between [-PI/2;+PI/2]
	if (yaw<-M_PI_2)
		yaw+=M_PI;
	else if (yaw>M_PI_2)
		yaw-=M_PI;

	ROS_INFO("RPY = %f,%f,%f",roll,pitch,yaw);

	q_tmp.setRPY(roll,pitch,yaw);
	GPTCP_target_pose_.orientation.x = q_tmp.getX();
	GPTCP_target_pose_.orientation.y = q_tmp.getY();
	GPTCP_target_pose_.orientation.z = q_tmp.getZ();
	GPTCP_target_pose_.orientation.w = q_tmp.getW();
}

void GraspPose2::compute_grasp_poses_()
{
	geometry_msgs::Pose tmp_GPTCP_pose, tmp2_GPTCP_pose;
	double dot_product;
	tf::Vector3 x_axis(1,0,0), y_axis(0,1,0), z_axis(0,0,1);
	tf::Vector3 x_obj, y_obj, z_obj;
	tf::Vector3 x_grp, y_grp, z_grp;
	tf::Vector3 o_grip_point_;
	tf::Vector3 tmp_vec;
	double min_distance_to_object;
	tf::Vector3 safe_to_grip_dir;


	uint8_t upward_pointing_axis;
	uint8_t handle_cylinder_idx;
	uint8_t handle_box1_idx;
	uint8_t handle_box2_idx;

	//clear all existing poses
	GPTCP_object_grip_pose.clear();
	GPTCP_object_safe_pose.clear();
	GPTCP_object_vision_pose.clear();
	object_pose_type.clear();
	object_grasp_width.clear();
	GPTCP_target_place_pose.clear();
	GPTCP_target_safe_pose.clear();
	GPTCP_target_vision_pose.clear();
	target_pose_type.clear();

	//=======FOR TESTING: STORE OLD POSES IN 0=========
	tmp_GPTCP_pose=GPTCP_target_pose_;
	tmp_GPTCP_pose.position.z=GPTCP_target_pose_.position.z+0.02;
	GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
	tmp_GPTCP_pose.position.z=GPTCP_target_pose_.position.z+0.05;
	GPTCP_object_safe_pose.push_back(tmp_GPTCP_pose);
	tmp_GPTCP_pose.position.z=GPTCP_target_pose_.position.z+0.4;
	GPTCP_object_vision_pose.push_back(tmp_GPTCP_pose);
	if(object_type_==OBJECT_CUBE)
	{
		object_grasp_width.push_back(object_.shape[0].size[0]);
		object_pose_type.push_back(OBJECT_POSE_CUBE_Z_UP);
	}
	if(object_type_==OBJECT_CYLINDER)
	{
		object_grasp_width.push_back(object_.shape[0].radius*2);
		object_pose_type.push_back(OBJECT_POSE_CYLINDER_VERTICAL);
	}
	if(object_type_==OBJECT_HANDLE)
	{
		object_grasp_width.push_back(object_.shape[0].radius*2);
		object_pose_type.push_back(OBJECT_POSE_HANDLE_HORIZONTAL);
	}
	tmp_GPTCP_pose=GPTCP_target_pose_;
	tmp_GPTCP_pose.position = target_zone_.position;
	//tmp_GPTCP_pose.position.x -= r_gp_curobj_.x;
	//tmp_GPTCP_pose.position.y -= r_gp_curobj_.y;
	tmp_GPTCP_pose.position.z=GPTCP_target_pose_.position.z+0.03;
	tmp_GPTCP_pose.orientation.x=1;
	tmp_GPTCP_pose.orientation.y=0;
	tmp_GPTCP_pose.orientation.z=0;
	tmp_GPTCP_pose.orientation.w=0;
	GPTCP_target_place_pose.push_back(tmp_GPTCP_pose);
	tmp_GPTCP_pose.position.z=GPTCP_target_pose_.position.z+0.1;
	GPTCP_target_safe_pose.push_back(tmp_GPTCP_pose);
	tmp_GPTCP_pose.position.z=GPTCP_target_pose_.position.z+0.3;
	GPTCP_target_vision_pose.push_back(tmp_GPTCP_pose);
	if(object_type_==OBJECT_CUBE)
	{
		target_pose_type.push_back(OBJECT_POSE_CUBE_Z_UP);
	}
	if(object_type_==OBJECT_CYLINDER)
	{
		target_pose_type.push_back(OBJECT_POSE_CYLINDER_VERTICAL);
	}
	if(object_type_==OBJECT_HANDLE)
	{
		target_pose_type.push_back(OBJECT_POSE_HANDLE_HORIZONTAL_CYLINDER);
	}
	//====================================================

	switch (object_type_)
	{
		case OBJECT_CUBE:
			//get axes of object
			x_obj = o_transform_shapes_[0].getBasis().getColumn(0);
			y_obj = o_transform_shapes_[0].getBasis().getColumn(1);
			z_obj = o_transform_shapes_[0].getBasis().getColumn(2);
			//=================================================================
			//--------------------------GRIPPING OBJECT------------------------
			//=================================================================
			//find upward pointing axis
			dot_product = x_obj.dot(z_axis);
			if (am_abs(dot_product) > 0.9)
				upward_pointing_axis=0;			//x-axis points up
			dot_product = y_obj.dot(z_axis);
			if (am_abs(dot_product) > 0.9)
				upward_pointing_axis=1;			//y-axis points up
			dot_product = z_obj.dot(z_axis);
			if (am_abs(dot_product) > 0.9)
				upward_pointing_axis=2;			//z-axis points up
			tmp_GPTCP_pose.position.x = o_object_com_.getX();
			tmp_GPTCP_pose.position.y = o_object_com_.getY();
			tmp_GPTCP_pose.position.z = o_object_com_.getZ()+0.5*gripper_height_;
			//avoid collision with floor
			if (tmp_GPTCP_pose.position.z < (gripper_height_+grip_safety_dist_))
			{
				tmp_GPTCP_pose.position.z = gripper_height_+grip_safety_dist_;
			}
			//avoid collision with object
			if (tmp_GPTCP_pose.position.z < (o_object_com_.getZ()+0.5*object_.shape[0].size[upward_pointing_axis]+grip_safety_dist_))
			{
				tmp_GPTCP_pose.position.z=o_object_com_.getZ()+0.5*object_.shape[0].size[upward_pointing_axis]+grip_safety_dist_;
			}

			z_grp = -1.0*z_axis;
			if (upward_pointing_axis==0) 		//x-axis points up
			{
				object_alignment=OBJECT_POSE_CUBE_X_UP;
				//possibility 1+2
				if (object_.shape[0].size[1]<=gripper_maxwidth_)
				{
					//possibility 1
					y_grp = z_obj;
					y_grp.setZ(0);	//only use rotation around vertical axis
					x_grp = y_grp.cross(z_grp);
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

					GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z+=gripper_height_;
					GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
					GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);
					object_pose_type.push_back(object_alignment);
					object_grasp_width.push_back(object_.shape[0].size[1]);

					//possibility 2
					y_grp = -z_obj;
					y_grp.setZ(0);	//only use rotation around vertical axis
					x_grp = y_grp.cross(z_grp);
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

					GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z+=gripper_height_;
					GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
					GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);
					object_pose_type.push_back(object_alignment);
					object_grasp_width.push_back(object_.shape[0].size[1]);
				}
				//possibility 3+4
				if (object_.shape[0].size[2]<=gripper_maxwidth_)
				{
					//possibility 3
					y_grp = y_obj;
					y_grp.setZ(0);	//only use rotation around vertical axis
					x_grp = y_grp.cross(z_grp);
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

					GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z+=gripper_height_;
					GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
					GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);
					object_pose_type.push_back(object_alignment);
					object_grasp_width.push_back(object_.shape[0].size[2]);

					//possibility 4
					y_grp = -y_obj;
					x_grp = y_grp.cross(z_grp);
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

					GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z+=gripper_height_;
					GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
					GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);
					object_pose_type.push_back(object_alignment);
					object_grasp_width.push_back(object_.shape[0].size[2]);
				}
			}
			if (upward_pointing_axis==1) 		//y-axis points up
			{
				object_alignment=OBJECT_POSE_CUBE_Y_UP;
				//possibility 5+6
				if (object_.shape[0].size[0]<=gripper_maxwidth_)
				{
					//possibility 5
					y_grp = z_obj;
					y_grp.setZ(0);	//only use rotation around vertical axis
					x_grp = y_grp.cross(z_grp);
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

					GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z+=gripper_height_;
					GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
					GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);
					object_pose_type.push_back(object_alignment);
					object_grasp_width.push_back(object_.shape[0].size[0]);

					//possibility 6
					y_grp = -z_obj;
					y_grp.setZ(0);	//only use rotation around vertical axis
					x_grp = y_grp.cross(z_grp);
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

					GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z+=gripper_height_;
					GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
					GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);
					object_pose_type.push_back(object_alignment);
					object_grasp_width.push_back(object_.shape[0].size[0]);
				}
				//possibility 7+8
				if (object_.shape[0].size[2]<=gripper_maxwidth_)
				{
					//possibility 7
					y_grp = x_obj;
					y_grp.setZ(0);	//only use rotation around vertical axis
					x_grp = y_grp.cross(z_grp);
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

					GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z+=gripper_height_;
					GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
					GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);
					object_pose_type.push_back(object_alignment);
					object_grasp_width.push_back(object_.shape[0].size[2]);

					//possibility 8
					y_grp = -x_obj;
					y_grp.setZ(0);	//only use rotation around vertical axis
					x_grp = y_grp.cross(z_grp);
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

					GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z+=gripper_height_;
					GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
					GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);
					object_pose_type.push_back(object_alignment);
					object_grasp_width.push_back(object_.shape[0].size[2]);
				}
			}
			if (upward_pointing_axis==2) 		//z-axis points up
			{
				object_alignment=OBJECT_POSE_CUBE_Z_UP;
				//possibility 9+10
				if (object_.shape[0].size[0]<=gripper_maxwidth_)
				{
					//possibility 9
					y_grp = y_obj;
					y_grp.setZ(0);	//only use rotation around vertical axis
					x_grp = y_grp.cross(z_grp);
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

					GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z+=gripper_height_;
					GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
					GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);
					object_pose_type.push_back(object_alignment);
					object_grasp_width.push_back(object_.shape[0].size[0]);

					//possibility 10
					y_grp = -y_obj;
					y_grp.setZ(0);	//only use rotation around vertical axis
					x_grp = y_grp.cross(z_grp);
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

					GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z+=gripper_height_;
					GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
					GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);
					object_pose_type.push_back(object_alignment);
					object_grasp_width.push_back(object_.shape[0].size[0]);
				}
				//possibility 11+12
				if (object_.shape[0].size[1]<=gripper_maxwidth_)
				{
					//possibility 11
					y_grp = x_obj;
					y_grp.setZ(0);	//only use rotation around vertical axis
					x_grp = y_grp.cross(z_grp);
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

					GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z+=gripper_height_;
					GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
					GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);
					object_pose_type.push_back(object_alignment);
					object_grasp_width.push_back(object_.shape[0].size[1]);

					//possibility 12
					y_grp = -x_obj;
					y_grp.setZ(0);	//only use rotation around vertical axis
					x_grp = y_grp.cross(z_grp);
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

					GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z+=gripper_height_;
					GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
					GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);
					object_pose_type.push_back(object_alignment);
					object_grasp_width.push_back(object_.shape[0].size[1]);
				}
			}

			//=================================================================
			//-----------------------PLACING ON TARGET ZONE--------------------
			//=================================================================
			tmp_GPTCP_pose.position = target_zone_.position;
			z_grp = -1.0*z_axis;
			target_alignment=object_alignment;

			//same code as for gripping...
			tmp_GPTCP_pose.position.z = o_object_com_.getZ()+0.5*gripper_height_;
			if (tmp_GPTCP_pose.position.z < (gripper_height_+grip_safety_dist_))
			{
				tmp_GPTCP_pose.position.z = gripper_height_+grip_safety_dist_;
			}
			if (tmp_GPTCP_pose.position.z < (o_object_com_.getZ()+0.5*object_.shape[0].size[upward_pointing_axis]+grip_safety_dist_))
			{
				tmp_GPTCP_pose.position.z=o_object_com_.getZ()+0.5*object_.shape[0].size[upward_pointing_axis]+grip_safety_dist_;
			}
			//...and then add a small distance
			tmp_GPTCP_pose.position.z+=place_falling_dist_;

			//calculate all 4 possibilities
			for(uint8_t ii=1; ii<=4; ii++)
			{
				if(ii==1)	//possibility 1
					y_grp=y_axis;
				if(ii==2)	//possibility 2
					y_grp=-y_axis;
				if(ii==3)	//possibility 3
					y_grp=x_axis;
				if(ii==4)	//possibility 4
					y_grp=-x_axis;
				x_grp = y_grp.cross(z_grp);
				set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);
				GPTCP_target_place_pose.push_back(tmp_GPTCP_pose);
				tmp2_GPTCP_pose=tmp_GPTCP_pose;
				tmp2_GPTCP_pose.position.z+=gripper_height_;
				GPTCP_target_safe_pose.push_back(tmp2_GPTCP_pose);
				tmp2_GPTCP_pose=tmp_GPTCP_pose;
				tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
				GPTCP_target_vision_pose.push_back(tmp2_GPTCP_pose);
				target_pose_type.push_back(target_alignment);
			}
			break;
		case OBJECT_CYLINDER:
			//get axes of object
			x_obj = o_transform_shapes_[0].getBasis().getColumn(0);
			y_obj = o_transform_shapes_[0].getBasis().getColumn(1);
			z_obj = o_transform_shapes_[0].getBasis().getColumn(2);
			//=================================================================
			//--------------------------GRIPPING OBJECT------------------------
			//=================================================================
			//detect cylinder alignment
			dot_product = z_obj.dot(z_axis);
			if (dot_product > -0.7 && dot_product < 0.7)
			{
				//the cylinder is rolling, the object has to be skipped, so don't calculate poses
				object_alignment=OBJECT_POSE_CYLINDER_HORIZONTAL;
				return;
			}
			else
			{
				object_alignment=OBJECT_POSE_CYLINDER_VERTICAL;
			}
			tmp_GPTCP_pose.position.x = o_object_com_.getX();
			tmp_GPTCP_pose.position.y = o_object_com_.getY();
			tmp_GPTCP_pose.position.z = o_object_com_.getZ()+0.5*gripper_height_;
			//avoid collision with floor
			if (tmp_GPTCP_pose.position.z < (gripper_height_+grip_safety_dist_))
			{
				tmp_GPTCP_pose.position.z = gripper_height_+grip_safety_dist_;
			}
			//avoid collision with object
			if (tmp_GPTCP_pose.position.z < (o_object_com_.getZ()+0.5*object_.shape[0].length+grip_safety_dist_))
			{
				tmp_GPTCP_pose.position.z=o_object_com_.getZ()+0.5*object_.shape[0].length+grip_safety_dist_;
			}

			z_grp = -1.0*z_axis;
			//calculate 4 possibilities
			for(uint8_t ii=1; ii<=4; ii++)
			{
				if(ii==1)	//possibility 1
					y_grp=y_axis;
				if(ii==2)	//possibility 2
					y_grp=-y_axis;
				if(ii==3)	//possibility 3
					y_grp=x_axis;
				if(ii==4)	//possibility 4
					y_grp=-x_axis;
				x_grp = y_grp.cross(z_grp);
				set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);
				GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
				tmp2_GPTCP_pose=tmp_GPTCP_pose;
				tmp2_GPTCP_pose.position.z+=gripper_height_;
				GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);
				tmp2_GPTCP_pose=tmp_GPTCP_pose;
				tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cylinder_;
				GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);
				object_pose_type.push_back(object_alignment);
				object_grasp_width.push_back(2.0*object_.shape[0].radius);
			}
			//=================================================================
			//-----------------------PLACING ON TARGET ZONE--------------------
			//=================================================================
			tmp_GPTCP_pose.position = target_zone_.position;
			z_grp = -1.0*z_axis;
			target_alignment=object_alignment;

			//same code as for gripping...
			tmp_GPTCP_pose.position.z = o_object_com_.getZ()+0.5*gripper_height_;
			if (tmp_GPTCP_pose.position.z < (gripper_height_+grip_safety_dist_))
			{
				tmp_GPTCP_pose.position.z = gripper_height_+grip_safety_dist_;
			}
			if (tmp_GPTCP_pose.position.z < (o_object_com_.getZ()+0.5*object_.shape[0].length+grip_safety_dist_))
			{
				tmp_GPTCP_pose.position.z=o_object_com_.getZ()+0.5*object_.shape[0].length+grip_safety_dist_;
			}
			//...and then add a small distance
			tmp_GPTCP_pose.position.z+=place_falling_dist_;

			//calculate all 4 possibilities
			for(uint8_t ii=1; ii<=4; ii++)
			{
				if(ii==1)	//possibility 1
					y_grp=y_axis;
				if(ii==2)	//possibility 2
					y_grp=-y_axis;
				if(ii==3)	//possibility 3
					y_grp=x_axis;
				if(ii==4)	//possibility 4
					y_grp=-x_axis;
				x_grp = y_grp.cross(z_grp);
				set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);
				GPTCP_target_place_pose.push_back(tmp_GPTCP_pose);
				tmp2_GPTCP_pose=tmp_GPTCP_pose;
				tmp2_GPTCP_pose.position.z+=gripper_height_;
				GPTCP_target_safe_pose.push_back(tmp2_GPTCP_pose);
				tmp2_GPTCP_pose=tmp_GPTCP_pose;
				tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cylinder_;
				GPTCP_target_vision_pose.push_back(tmp2_GPTCP_pose);
				target_pose_type.push_back(target_alignment);
			}
			break;
		case OBJECT_HANDLE:
			//get shape indices
			handle_cylinder_idx=0;
			handle_box1_idx=0;
			handle_box2_idx=0;
			for(uint8_t ii=0; ii<object_.nr_shapes; ii++)
			{
				if (!object_.shape[ii].type.compare("cylinder"))
				{ handle_cylinder_idx=ii; }
			}
			if(handle_cylinder_idx==0)
			{
				handle_box1_idx=1;
				handle_box2_idx=2;
			}
			if(handle_cylinder_idx==1)
			{
				handle_box1_idx=0;
				handle_box2_idx=2;
			}
			if(handle_cylinder_idx==2)
			{
				handle_box1_idx=0;
				handle_box2_idx=1;
			}
			//get axes of object
			x_obj = o_transform_shapes_[handle_cylinder_idx].getBasis().getColumn(0);
			y_obj = o_transform_shapes_[handle_cylinder_idx].getBasis().getColumn(1);
			z_obj = o_transform_shapes_[handle_cylinder_idx].getBasis().getColumn(2);

			//get grip_point (nearest point of cylinder axis to COM of object)
			//first calculate the connecting vector of the object and the shape COM
			tmp_vec=o_object_com_-o_transform_shapes_[handle_cylinder_idx].getOrigin();
			//then project it to the z-axis of the cylinder
			double projected_distance;
			projected_distance=tmp_vec.dot(z_obj)/z_obj.length();
			//and limit it to avoid collisions with the cubes
			if(projected_distance>0.5*object_.shape[handle_cylinder_idx].length-0.5*gripper_finger_width-grip_safety_dist_)
			{
				projected_distance=0.5*object_.shape[handle_cylinder_idx].length-0.5*gripper_finger_width-grip_safety_dist_;
			}
			if(projected_distance<-(0.5*object_.shape[handle_cylinder_idx].length-0.5*gripper_finger_width-grip_safety_dist_))
			{
				projected_distance=-(0.5*object_.shape[handle_cylinder_idx].length-0.5*gripper_finger_width-grip_safety_dist_);
			}
			o_grip_point_=o_transform_shapes_[handle_cylinder_idx].getOrigin();
			o_grip_point_=o_grip_point_+projected_distance*z_obj/z_obj.length();

			//=================================================================
			//--------------------------GRIPPING OBJECT------------------------
			//=================================================================
			//detect handle alignment
			dot_product = z_obj.dot(z_axis);
			if (dot_product > -0.7 && dot_product < 0.7)
			{
				//handle horizontal
				object_alignment=OBJECT_POSE_HANDLE_HORIZONTAL;
			}
			else
			{
				//handle vertical
				object_alignment=OBJECT_POSE_HANDLE_VERTICAL;
			}
			if(object_alignment==OBJECT_POSE_HANDLE_HORIZONTAL)
			{
				z_grp = -1.0*z_axis;
				tmp_GPTCP_pose.position.x = o_grip_point_.getX();
				tmp_GPTCP_pose.position.y = o_grip_point_.getY();
				tmp_GPTCP_pose.position.z = o_grip_point_.getZ()+0.5*gripper_height_;
				//avoid collision with floor
				if (tmp_GPTCP_pose.position.z < (gripper_height_+grip_safety_dist_))
				{
					tmp_GPTCP_pose.position.z = gripper_height_+grip_safety_dist_;
				}
				//avoid collision with object
				if (tmp_GPTCP_pose.position.z < (o_grip_point_.getZ()+object_.shape[handle_cylinder_idx].radius+grip_safety_dist_))
				{
					tmp_GPTCP_pose.position.z=o_grip_point_.getZ()+object_.shape[handle_cylinder_idx].radius+grip_safety_dist_;
				}

				//possibility 1
				y_grp = z_obj;
				y_grp.setZ(0);	//only use rotation around vertical axis
				x_grp = y_grp.cross(z_grp);
				set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

				GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
				tmp2_GPTCP_pose=tmp_GPTCP_pose;
				tmp2_GPTCP_pose.position.z+=gripper_height_;
				GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);
				tmp2_GPTCP_pose=tmp_GPTCP_pose;
				tmp2_GPTCP_pose.position.x=o_transform_shapes_[handle_cylinder_idx].getOrigin().getX();
				tmp2_GPTCP_pose.position.y=o_transform_shapes_[handle_cylinder_idx].getOrigin().getY();
				tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_handle_;
				GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);
				object_pose_type.push_back(object_alignment);
				object_grasp_width.push_back(2.0*object_.shape[handle_cylinder_idx].radius);

				//possibility 2
				y_grp = -z_obj;
				y_grp.setZ(0);	//only use rotation around vertical axis
				x_grp = y_grp.cross(z_grp);
				set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

				GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
				tmp2_GPTCP_pose=tmp_GPTCP_pose;
				tmp2_GPTCP_pose.position.z+=gripper_height_;
				GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);
				tmp2_GPTCP_pose=tmp_GPTCP_pose;
				tmp2_GPTCP_pose.position.x=o_transform_shapes_[handle_cylinder_idx].getOrigin().getX();
				tmp2_GPTCP_pose.position.y=o_transform_shapes_[handle_cylinder_idx].getOrigin().getY();
				tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_handle_;
				GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);
				object_pose_type.push_back(object_alignment);
				object_grasp_width.push_back(2.0*object_.shape[handle_cylinder_idx].radius);
			}
			if(object_alignment==OBJECT_POSE_HANDLE_VERTICAL)
			{
				min_distance_to_object=object_.shape[handle_cylinder_idx].radius+grip_safety_dist_;
				tmp_GPTCP_pose.position.z = o_grip_point_.getZ();

				//possibility 1
				y_grp=-z_axis;
				z_grp=x_axis;
				z_grp.setZ(0);	//only use rotation around vertical axis
				x_grp = y_grp.cross(z_grp);
				set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

				ROS_INFO("x-axis-object: [%f %f %f]",x_obj.getX(),x_obj.getY(),x_obj.getZ());
				ROS_INFO("y-axis-object: [%f %f %f]",y_obj.getX(),y_obj.getY(),y_obj.getZ());
				ROS_INFO("z-axis-object: [%f %f %f]",z_obj.getX(),z_obj.getY(),z_obj.getZ());
				ROS_INFO("x-axis-gripper: [%f %f %f]",x_grp.getX(),x_grp.getY(),x_grp.getZ());
				ROS_INFO("y-axis-gripper: [%f %f %f]",y_grp.getX(),y_grp.getY(),y_grp.getZ());
				ROS_INFO("z-axis-gripper: [%f %f %f]",z_grp.getX(),z_grp.getY(),z_grp.getZ());

				ROS_INFO("gripper quat (x,y,z,w): [%f %f %f %f]",tmp_GPTCP_pose.orientation.x,
						tmp_GPTCP_pose.orientation.y,tmp_GPTCP_pose.orientation.z,tmp_GPTCP_pose.orientation.w);

				safe_to_grip_dir=z_grp/z_grp.length();
				if (min_distance_to_object > 0.5*gripper_height_)
				{
					tmp_vec=o_grip_point_-safe_to_grip_dir*min_distance_to_object;
				}
				else
				{
					tmp_vec=o_grip_point_-safe_to_grip_dir*0.5*gripper_height_;
				}

				tmp_GPTCP_pose.position.x=tmp_vec.getX();
				tmp_GPTCP_pose.position.y=tmp_vec.getY();
				tmp_GPTCP_pose.position.z=tmp_vec.getZ();
//				tmp_GPTCP_pose.orientation.x=-0.5;
//				tmp_GPTCP_pose.orientation.y=0.5;
//				tmp_GPTCP_pose.orientation.z=-0.5;
				GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
				tmp2_GPTCP_pose=tmp_GPTCP_pose;
//				tmp_vec=tmp_vec-safe_to_grip_dir*gripper_height_;
//				tmp_vec.setZ(tmp_vec.getZ()+0.01);
//				tmp2_GPTCP_pose.position.x=tmp_vec.getX();
//				tmp2_GPTCP_pose.position.y=tmp_vec.getY();
//				tmp2_GPTCP_pose.position.z=tmp_vec.getZ();
				GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);
//				z_grp=-z_axis;
//				y_grp=-y_obj;
//				y_grp.setZ(0);	//only use rotation around vertical axis
//				x_grp = y_grp.cross(z_grp);
//				set_orientation_from_axes(tmp2_GPTCP_pose,x_grp,y_grp,z_grp);
//
//				tmp2_GPTCP_pose.position.x=o_transform_shapes_[handle_cylinder_idx].getOrigin().getX();
//				tmp2_GPTCP_pose.position.y=o_transform_shapes_[handle_cylinder_idx].getOrigin().getY();
//				tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_handle_;
				GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);
				object_pose_type.push_back(object_alignment);
				object_grasp_width.push_back(2.0*object_.shape[handle_cylinder_idx].radius);
			}
			//=================================================================
			//-----------------------PLACING ON TARGET ZONE--------------------
			//=================================================================
			//TODO



			break;
		default:
			break;
	}
}

void GraspPose2::transform_grasp_pose_GPTCP_2_LWRTCP_() {

	tf::TransformListener tf_listener, tf_listener_obj;
	tf::StampedTransform transform_GPTCP_2_LWRTCP;
	tf::Transform tf_tmp,tf_tmp2;

	ros::Time now = ros::Time::now();
	try{
		tf_listener.waitForTransform(LWR_TCP,GP_TCP,now,ros::Duration(2.0));
		tf_listener.lookupTransform(LWR_TCP,GP_TCP,ros::Time(0),transform_GPTCP_2_LWRTCP);
		ROS_INFO("Listening to transform was successful");
	}
	catch(...){
		ROS_ERROR("Listening to transform was not successful");
	}
	tf_tmp.setOrigin(tf::Vector3(GPTCP_target_pose_.position.x,
			GPTCP_target_pose_.position.y,
			GPTCP_target_pose_.position.z));
	tf_tmp.setRotation(tf::Quaternion(GPTCP_target_pose_.orientation.x,
			GPTCP_target_pose_.orientation.y,
			GPTCP_target_pose_.orientation.z,
			GPTCP_target_pose_.orientation.w));

	transform_gripper = tf_tmp;

	tf_tmp2 = transform_GPTCP_2_LWRTCP*tf_tmp;

	LWRTCP_target_pose_.position.x = tf_tmp2.getOrigin().getX();
	LWRTCP_target_pose_.position.y = tf_tmp2.getOrigin().getY();
	LWRTCP_target_pose_.position.z = tf_tmp2.getOrigin().getZ();

	LWRTCP_target_pose_.orientation.x = tf_tmp2.getRotation().getX();
	LWRTCP_target_pose_.orientation.y = tf_tmp2.getRotation().getY();
	LWRTCP_target_pose_.orientation.z = tf_tmp2.getRotation().getZ();
	LWRTCP_target_pose_.orientation.w = tf_tmp2.getRotation().getW();


	//calculate relative vector (Object-CoM / LWR-TCP)
	r_tcp_com_.x = o_object_com_.x()-LWRTCP_target_pose_.position.x;
	r_tcp_com_.y = o_object_com_.y()-LWRTCP_target_pose_.position.y;
	r_tcp_com_.z = o_object_com_.z()-LWRTCP_target_pose_.position.z;

	//calculate relative vector (Object-CoM / GP-TCP)
	r_gp_com_.x = r_tcp_com_.x-transform_GPTCP_2_LWRTCP.getOrigin().getX();
	r_gp_com_.y = r_tcp_com_.y-transform_GPTCP_2_LWRTCP.getOrigin().getY();
	r_gp_com_.z = r_tcp_com_.z-transform_GPTCP_2_LWRTCP.getOrigin().getZ();

	//! Vector from Gripper tcp to object frame
	r_target_offset_.x = object_.abs_pose.position.x - LWRTCP_target_pose_.position.x;
	r_target_offset_.y = object_.abs_pose.position.y - LWRTCP_target_pose_.position.y;
	r_target_offset_.z = object_.abs_pose.position.z - LWRTCP_target_pose_.position.z;
	//	ROS_INFO("r_tcp_com: [%f %f %f]",r_tcp_com_.x,r_tcp_com_.y,r_tcp_com_.z);
	//	ROS_INFO("r_gp_com:  [%f %f %f]",r_gp_com_.x,r_gp_com_.y,r_gp_com_.z);

}

tf::StampedTransform GraspPose2::get_transform_GPTCP_2_LWRTCP()
{
	tf::TransformListener tf_listener;
	tf::StampedTransform tmp_transform_GPTCP_2_LWRTCP;

	ros::Time now = ros::Time::now();
	try{
		tf_listener.waitForTransform(LWR_TCP,GP_TCP,now,ros::Duration(2.0));
		tf_listener.lookupTransform(LWR_TCP,GP_TCP,ros::Time(0),tmp_transform_GPTCP_2_LWRTCP);
		ROS_INFO("Listening to transform was successful");
	}
	catch(...)
	{
		msg_error("Listening to transform was not successful");
	}
	return tmp_transform_GPTCP_2_LWRTCP;
}

geometry_msgs::Pose GraspPose2::transform_pose_GPTCP_2_LWRTCP_(geometry_msgs::Pose GPTCP_pose)
{
	geometry_msgs::Pose LWRTCP_pose;
	tf::Transform tf_tmp,tf_tmp2;

	tf_tmp.setOrigin(tf::Vector3(GPTCP_pose.position.x,GPTCP_pose.position.y,GPTCP_pose.position.z));
	tf_tmp.setRotation(tf::Quaternion(GPTCP_pose.orientation.x,GPTCP_pose.orientation.y,GPTCP_pose.orientation.z,GPTCP_pose.orientation.w));

	transform_gripper = tf_tmp;

	tf_tmp2 = transform_GPTCP_2_LWRTCP_*tf_tmp;

	LWRTCP_pose.position.x = tf_tmp2.getOrigin().getX();
	LWRTCP_pose.position.y = tf_tmp2.getOrigin().getY();
	LWRTCP_pose.position.z = tf_tmp2.getOrigin().getZ();
	LWRTCP_pose.orientation.x = tf_tmp2.getRotation().getX();
	LWRTCP_pose.orientation.y = tf_tmp2.getRotation().getY();
	LWRTCP_pose.orientation.z = tf_tmp2.getRotation().getZ();
	LWRTCP_pose.orientation.w = tf_tmp2.getRotation().getW();

	return LWRTCP_pose;
}

void GraspPose2::compute_relative_vectors_()
{
	object_grip_r_tcp_com.clear();
	object_grip_r_gp_com.clear();
	object_grip_r_gp_obj.clear();
	object_grip_r_tcp_com.resize(LWRTCP_object_grip_pose.size());
	object_grip_r_gp_com.resize(LWRTCP_object_grip_pose.size());
	object_grip_r_gp_obj.resize(LWRTCP_object_grip_pose.size());

	for(int ii=0; ii<LWRTCP_object_grip_pose.size();ii++)
	{
		//calculate relative vector (LWR-TCP -> Object-CoM)
		object_grip_r_tcp_com[ii].x = o_object_com_.x()-LWRTCP_object_grip_pose[ii].position.x;
		object_grip_r_tcp_com[ii].y = o_object_com_.y()-LWRTCP_object_grip_pose[ii].position.y;
		object_grip_r_tcp_com[ii].z = o_object_com_.z()-LWRTCP_object_grip_pose[ii].position.z;

		//calculate relative vector (GP-TCP -> Object-CoM)
		object_grip_r_gp_com[ii].x = object_grip_r_tcp_com[ii].x-transform_GPTCP_2_LWRTCP_.getOrigin().getX();
		object_grip_r_gp_com[ii].y = object_grip_r_tcp_com[ii].y-transform_GPTCP_2_LWRTCP_.getOrigin().getY();
		object_grip_r_gp_com[ii].z = object_grip_r_tcp_com[ii].z-transform_GPTCP_2_LWRTCP_.getOrigin().getZ();

		//calculate vector from gripper tcp to object frame
		object_grip_r_gp_obj[ii].x = object_.abs_pose.position.x - LWRTCP_object_grip_pose[ii].position.x;
		object_grip_r_gp_obj[ii].y = object_.abs_pose.position.y - LWRTCP_object_grip_pose[ii].position.y;
		object_grip_r_gp_obj[ii].z = object_.abs_pose.position.z - LWRTCP_object_grip_pose[ii].position.z;
	}
}

void GraspPose2::set_orientation_from_axes(geometry_msgs::Pose &tmp_pose, tf::Vector3 x_axis, tf::Vector3 y_axis, tf::Vector3 z_axis)
{
	double roll,pitch,yaw;
	tf::Matrix3x3 dcm;
	tf::Quaternion q_tmp;

//	dcm.setValue(x_axis.getX(),x_axis.getY(),x_axis.getZ(),y_axis.getX(),y_axis.getY(),y_axis.getZ(),
//				 z_axis.getX(),z_axis.getY(),z_axis.getZ());
	//transposed matrix seems to be the right one :)
	dcm.setValue(x_axis.getX(),y_axis.getX(),z_axis.getX(),x_axis.getY(),y_axis.getY(),z_axis.getY(),
				 x_axis.getZ(),y_axis.getZ(),z_axis.getZ());
	dcm.getRPY(roll,pitch,yaw);
	q_tmp.setRPY(roll,pitch,yaw);

	tmp_pose.orientation.x = q_tmp.getX();
	tmp_pose.orientation.y = q_tmp.getY();
	tmp_pose.orientation.z = q_tmp.getZ();
	tmp_pose.orientation.w = q_tmp.getW();
}

void GraspPose2::print_results()
{
	std::vector<geometry_msgs::Pose> tmpPoseArray;
	std::vector<geometry_msgs::Vector3> tmpVector3Array;

	ROS_INFO("===============================================");
	ROS_INFO("---------------BEGIN RESULTS-------------------");
	ROS_INFO("===============================================");

	if(object_type_==OBJECT_CYLINDER)
	{
		ROS_INFO("object_type: CYLINDER");
		ROS_INFO("radius: %f | length: %f",object_.shape[0].radius,object_.shape[0].length);
	}
	if(object_type_==OBJECT_CUBE)
	{
		ROS_INFO("object_type: CUBE");
		ROS_INFO("size: [%f %f %f]",object_.shape[0].size[0],object_.shape[0].size[1],object_.shape[0].size[2]);
	}
	if(object_type_==OBJECT_HANDLE)
	{
		ROS_INFO("object_type: HANDLE");
		for(uint8_t ii=0; ii<object_.nr_shapes; ii++)
		{
			if(!object_.shape[ii].type.compare("box"))
			{
				ROS_INFO("shape[%d]: box - size: [%f %f %f]",ii,object_.shape[ii].size[0],object_.shape[ii].size[1],object_.shape[ii].size[2]);
			}
			if(!object_.shape[ii].type.compare("cylinder"))
			{
				ROS_INFO("shape[%d]: cylinder - radius: %f | length: %f",ii,object_.shape[ii].radius,object_.shape[ii].length);
			}
		}
	}
	ROS_INFO("object_mass: %fkg",object_mass_);

	tmpPoseArray.clear();
	tmpPoseArray=LWRTCP_object_grip_pose;
	ROS_INFO("LWRTCP_object_grip_pose: PoseNr | pos: (X Y Z) | ori: (X Y Z W)");
	for(uint8_t ii=0; ii<tmpPoseArray.size(); ii++)
	{
		ROS_INFO(" [%d] | [%4.3f %4.3f %4.3f] | [%4.3f %4.3f %4.3f %4.3f]",ii,tmpPoseArray[ii].position.x,
				tmpPoseArray[ii].position.y,tmpPoseArray[ii].position.z,tmpPoseArray[ii].orientation.x,tmpPoseArray[ii].orientation.y,
				tmpPoseArray[ii].orientation.z,tmpPoseArray[ii].orientation.w);
	}
	tmpPoseArray.clear();
	tmpPoseArray=LWRTCP_object_safe_pose;
	ROS_INFO("LWRTCP_object_safe_pose: PoseNr | pos: (X Y Z) | ori: (X Y Z W)");
	for(uint8_t ii=0; ii<tmpPoseArray.size(); ii++)
	{
		ROS_INFO(" [%d] | [%4.3f %4.3f %4.3f] | [%4.3f %4.3f %4.3f %4.3f]",ii,tmpPoseArray[ii].position.x,
				tmpPoseArray[ii].position.y,tmpPoseArray[ii].position.z,tmpPoseArray[ii].orientation.x,tmpPoseArray[ii].orientation.y,
				tmpPoseArray[ii].orientation.z,tmpPoseArray[ii].orientation.w);
	}
	tmpPoseArray.clear();
	tmpPoseArray=LWRTCP_object_vision_pose;
	ROS_INFO("LWRTCP_object_vision_pose: PoseNr | pos: (X Y Z) | ori (X Y Z W)");
	for(uint8_t ii=0; ii<tmpPoseArray.size(); ii++)
	{
		ROS_INFO(" [%d] | [%4.3f %4.3f %4.3f] | [%4.3f %4.3f %4.3f %4.3f]",ii,tmpPoseArray[ii].position.x,
				tmpPoseArray[ii].position.y,tmpPoseArray[ii].position.z,tmpPoseArray[ii].orientation.x,tmpPoseArray[ii].orientation.y,
				tmpPoseArray[ii].orientation.z,tmpPoseArray[ii].orientation.w);
	}
	tmpPoseArray.clear();
	tmpPoseArray=LWRTCP_target_place_pose;
	ROS_INFO("LWRTCP_target_place_pose: PoseNr | pos: (X Y Z) | ori: (X Y Z W)");
	for(uint8_t ii=0; ii<tmpPoseArray.size(); ii++)
	{
		ROS_INFO(" [%d] | [%4.3f %4.3f %4.3f] | [%4.3f %4.3f %4.3f %4.3f]",ii,tmpPoseArray[ii].position.x,
				tmpPoseArray[ii].position.y,tmpPoseArray[ii].position.z,tmpPoseArray[ii].orientation.x,tmpPoseArray[ii].orientation.y,
				tmpPoseArray[ii].orientation.z,tmpPoseArray[ii].orientation.w);
	}
	tmpPoseArray.clear();
	tmpPoseArray=LWRTCP_target_safe_pose;
	ROS_INFO("LWRTCP_target_safe_pose: PoseNr | pos: (X Y Z) | ori: (X Y Z W)");
	for(uint8_t ii=0; ii<tmpPoseArray.size(); ii++)
	{
		ROS_INFO(" [%d] | [%4.3f %4.3f %4.3f] | [%4.3f %4.3f %4.3f %4.3f]",ii,tmpPoseArray[ii].position.x,
				tmpPoseArray[ii].position.y,tmpPoseArray[ii].position.z,tmpPoseArray[ii].orientation.x,tmpPoseArray[ii].orientation.y,
				tmpPoseArray[ii].orientation.z,tmpPoseArray[ii].orientation.w);
	}
	tmpPoseArray.clear();
	tmpPoseArray=LWRTCP_target_vision_pose;
	ROS_INFO("LWRTCP_target_vision_pose: PoseNr | pos: (X Y Z) | ori: (X Y Z W)");
	for(uint8_t ii=0; ii<tmpPoseArray.size(); ii++)
	{
		ROS_INFO(" [%d] | [%4.3f %4.3f %4.3f] | [%4.3f %4.3f %4.3f %4.3f]",ii,tmpPoseArray[ii].position.x,
				tmpPoseArray[ii].position.y,tmpPoseArray[ii].position.z,tmpPoseArray[ii].orientation.x,tmpPoseArray[ii].orientation.y,
				tmpPoseArray[ii].orientation.z,tmpPoseArray[ii].orientation.w);
	}
	ROS_INFO("object_pose_type: PoseNr | type");
	for(uint8_t ii=0; ii<object_pose_type.size(); ii++)
	{
		switch(object_pose_type[ii])
		{
			case OBJECT_POSE_CUBE_X_UP:
				ROS_INFO(" [%d] | OBJECT_POSE_CUBE_X_UP",ii);
				break;
			case OBJECT_POSE_CUBE_Y_UP:
				ROS_INFO(" [%d] | OBJECT_POSE_CUBE_Y_UP",ii);
				break;
			case OBJECT_POSE_CUBE_Z_UP:
				ROS_INFO(" [%d] | OBJECT_POSE_CUBE_Z_UP",ii);
				break;
			case OBJECT_POSE_CYLINDER_VERTICAL:
				ROS_INFO(" [%d] | OBJECT_POSE_CYLINDER_VERTICAL",ii);
				break;
			case OBJECT_POSE_CYLINDER_HORIZONTAL:
				ROS_INFO(" [%d] | OBJECT_POSE_CYLINDER_HORIZONTAL",ii);
				break;
			case OBJECT_POSE_HANDLE_HORIZONTAL:
				ROS_INFO(" [%d] | OBJECT_POSE_HANDLE_HORIZONTAL",ii);
				break;
			case OBJECT_POSE_HANDLE_VERTICAL:
				ROS_INFO(" [%d] | OBJECT_POSE_HANDLE_VERTICAL",ii);
				break;
		}
	}
	ROS_INFO("target_pose_type: PoseNr | type");
	for(uint8_t ii=0; ii<target_pose_type.size(); ii++)
	{
		switch(target_pose_type[ii])
		{
			case OBJECT_POSE_CUBE_X_UP:
				ROS_INFO(" [%d] | OBJECT_POSE_CUBE_X_UP",ii);
				break;
			case OBJECT_POSE_CUBE_Y_UP:
				ROS_INFO(" [%d] | OBJECT_POSE_CUBE_Y_UP",ii);
				break;
			case OBJECT_POSE_CUBE_Z_UP:
				ROS_INFO(" [%d] | OBJECT_POSE_CUBE_Z_UP",ii);
				break;
			case OBJECT_POSE_CYLINDER_VERTICAL:
				ROS_INFO(" [%d] | OBJECT_POSE_CYLINDER_VERTICAL",ii);
				break;
			case OBJECT_POSE_CYLINDER_HORIZONTAL:
				ROS_INFO(" [%d] | OBJECT_POSE_CYLINDER_HORIZONTAL",ii);
				break;
			case OBJECT_POSE_HANDLE_HORIZONTAL_CYLINDER:
				ROS_INFO(" [%d] | OBJECT_POSE_HANDLE_HORIZONTAL_CYLINDER",ii);
				break;
			case OBJECT_POSE_HANDLE_HORIZONTAL_BOX_1:
				ROS_INFO(" [%d] | OBJECT_POSE_HANDLE_HORIZONTAL_BOX_1",ii);
				break;
			case OBJECT_POSE_HANDLE_HORIZONTAL_BOX_2:
				ROS_INFO(" [%d] | OBJECT_POSE_HANDLE_HORIZONTAL_BOX_2",ii);
				break;
			case OBJECT_POSE_HANDLE_VERTICAL_BOX_1_UP:
				ROS_INFO(" [%d] | OBJECT_POSE_HANDLE_VERTICAL_BOX_1_UP",ii);
				break;
			case OBJECT_POSE_HANDLE_VERTICAL_BOX_2_UP:
				ROS_INFO(" [%d] | OBJECT_POSE_HANDLE_VERTICAL_BOX_2_UP",ii);
				break;
		}
	}
	ROS_INFO("object_grasp_width: PoseNr | width");
	for(uint8_t ii=0; ii<object_grasp_width.size(); ii++)
	{
		ROS_INFO(" [%d] | %f",ii,object_grasp_width[ii]);
	}
	tmpVector3Array.clear();
	tmpVector3Array=object_grip_r_tcp_com;
	ROS_INFO("object_grip_r_tcp_com: PoseNr | vector: (X Y Z)");
	for(uint8_t ii=0; ii<tmpVector3Array.size(); ii++)
	{
		ROS_INFO(" [%d] | %f %f %f",ii,tmpVector3Array[ii].x,tmpVector3Array[ii].y,
				tmpVector3Array[ii].z);
	}
	tmpVector3Array.clear();
	tmpVector3Array=object_grip_r_gp_com;
	ROS_INFO("object_grip_r_gp_com: PoseNr | vector: (X Y Z)");
	for(uint8_t ii=0; ii<tmpVector3Array.size(); ii++)
	{
		ROS_INFO("- [%d] | %f %f %f",ii,tmpVector3Array[ii].x,tmpVector3Array[ii].y,
				tmpVector3Array[ii].z);
	}
	tmpVector3Array.clear();
	tmpVector3Array=object_grip_r_gp_obj;
	ROS_INFO("object_grip_r_gp_obj: PoseNr | vector: (X Y Z)");
	for(uint8_t ii=0; ii<tmpVector3Array.size(); ii++)
	{
		ROS_INFO(" [%d] | %f %f %f",ii,tmpVector3Array[ii].x,tmpVector3Array[ii].y,
				tmpVector3Array[ii].z);
	}
	ROS_INFO("===============================================");
	ROS_INFO("-----------------END RESULTS-------------------");
	ROS_INFO("===============================================");
}
