/*
 * GetGraspPose2.cpp
 *
 *  Created on: Aug 13, 2014
 *      Author: euroc_student
 */

#include <GraspPose2.hpp>


GraspPose2::GraspPose2()
{

}

GraspPose2::~GraspPose2() {
	// TODO Auto-generated destructor stub
}

void GraspPose2::set_object_data_(am_msgs::Object object) {
	// assign class property "object_"
	object_        = object;

	dist_COM_shape_.resize(object_.nr_shapes);
	b_transform_shapes_.resize(object_.nr_shapes);
	o_transform_shapes_.resize(object_.nr_shapes);

	gripper_height_=0.06;
	grip_safety_dist_=0.02;
}

// ROS-Service Function
bool GraspPose2::return_grasp_pose(am_msgs::GetGraspPose::Request &req, am_msgs::GetGraspPose::Response &res)
{

	// Control output

	// Only Prio-1 grasp poses are currently implemented
	unsigned int prio = 1;

	// Update object data
	set_object_data_(req.object);
	compute_object_CoM_();
	compute_idx_shape_CoM_();
	compute_abs_shape_poses_();
	compute_object_height_();
	compute_grasp_pose_();
	transform_grasp_pose_GPTCP_2_LWRTCP_();

	ROS_INFO("Nr of shape %i",object_.nr_shapes);
	ROS_INFO("CoM part of shape %i",com_idx_);
	// Compute grasp pose
	res.grasp_pose = LWRTCP_target_pose_;
	res.grasp_width = grasp_width_;
	res.waiting_height = object_height_+0.2;
	res.object_mass = object_mass_;
	res.r_tcp_com = r_tcp_com_;
	res.r_gp_com = r_gp_com_;
	res.r_gp_obj = r_target_offset_;

	if (!first_called)
		first_called =true;

	return true;
}

void GraspPose2::compute_abs_shape_poses_() {

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
	std::cout << "-------------- END ShAPES ------------"<<std::endl;
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
			shape_mass = M_PI*object_.shape[ii].radius*object_.shape[ii].radius*object_.shape[ii].length*object_.shape[ii].density;
		else if (!object_.shape[ii].type.compare("box"))
			shape_mass = object_.shape[ii].size[0]*object_.shape[ii].size[1]*object_.shape[ii].size[2]*object_.shape[ii].density;
		else
			msg_error("Undefined type.");

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

			if(!object_.shape[ii].type.compare("cylinder"))
			{
				if (conn_.closestAxis() != 2)
					dist_COM_shape_[ii] = conn_.length() - (0.5*object_.shape[ii].length);
				else
					dist_COM_shape_[ii] = conn_.length() - object_.shape[ii].radius;
			}
			else if (!object_.shape[ii].type.compare("box"))
			{
				dist_COM_shape_[ii] = conn_.length() - 0.5*object_.shape[ii].size[conn_.closestAxis()];
			}
			else
				msg_error("Undefined type.");
			if (dist_COM_shape_[ii]<0){
				if (com_idx_ <0)
					com_idx_ = ii;
				else
					msg_warn("CoM part of two shapes??");
			}
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

	GPTCP_target_pose_.position.x = o_object_com_.getX();
	GPTCP_target_pose_.position.y = o_object_com_.getY();
	if (high_idx_ == com_idx_)
		GPTCP_target_pose_.position.z = object_height_ - 0.5*gripper_height_ + grip_safety_dist_;
	else
		GPTCP_target_pose_.position.z = o_object_com_.getZ() + grip_safety_dist_;

	double roll,pitch,yaw,dummy1,dummy2;
	tf::Matrix3x3 dcm;
	tf::Quaternion q_tmp;



	if(!object_.shape[com_idx_].type.compare("cylinder"))
	{
		dot_product = o_transform_shapes_[com_idx_].getBasis().getColumn(2).dot(z_axis);
		grasp_width_ = object_.shape[com_idx_].radius*2.0;
		if (dot_product > -0.7 && dot_product < 0.7) // cylinder is vertical
		{
			dcm.setRotation(o_transform_shapes_[com_idx_].getRotation());
			dcm.getRPY(dummy1,dummy2,yaw);
		}
		else //! Cylinder is horizontal
		{
			roll = 0;
			pitch = M_PI;
			yaw = 0.0; //TODO!
			yaw = atan2(am_abs(GPTCP_target_pose_.position.y),am_abs(GPTCP_target_pose_.position.x));
		}

	}
	else if (!object_.shape[com_idx_].type.compare("box"))
	{
		dcm.setRotation(o_transform_shapes_[com_idx_].getRotation());
		std::cout<<"x-axis "<<o_transform_shapes_[com_idx_].getBasis().getColumn(0).getX()<<" "<<o_transform_shapes_[com_idx_].getBasis().getColumn(0).getY()<<" "<<o_transform_shapes_[com_idx_].getBasis().getColumn(0).getZ()<<std::endl;
		std::cout<<"y-axis "<<o_transform_shapes_[com_idx_].getBasis().getColumn(1).getX()<<" "<<o_transform_shapes_[com_idx_].getBasis().getColumn(1).getY()<<" "<<o_transform_shapes_[com_idx_].getBasis().getColumn(1).getZ()<<std::endl;
		std::cout<<"z-axis "<<o_transform_shapes_[com_idx_].getBasis().getColumn(2).getX()<<" "<<o_transform_shapes_[com_idx_].getBasis().getColumn(2).getY()<<" "<<o_transform_shapes_[com_idx_].getBasis().getColumn(2).getZ()<<std::endl;
		if (am_abs(o_transform_shapes_[com_idx_].getBasis().getColumn(0).dot(z_axis))>0.9)
		{
			ROS_INFO("x_axis pointing up");
			dcm.getRPY(yaw,dummy1,dummy2);
		}
		if (am_abs(o_transform_shapes_[com_idx_].getBasis().getColumn(1).dot(z_axis))>0.9)
		{
			ROS_INFO("y_axis pointing up");
			dcm.getRPY(dummy1,yaw,dummy2);
		}
		if (am_abs(o_transform_shapes_[com_idx_].getBasis().getColumn(2).dot(z_axis))>0.9)
		{
			ROS_INFO("z_axis pointing up");
			dcm.getRPY(dummy1,dummy2,yaw);
		}

		grasp_width_ = object_.shape[com_idx_].size[0];
	}

	//! Choose roll, pitch dependent on position
	if (am_abs(GPTCP_target_pose_.position.x)>am_abs(GPTCP_target_pose_.position.y))
	{
		roll = 0;
		if (GPTCP_target_pose_.position.x > 0)
			pitch = M_PI;
		else
			pitch = -M_PI;
	}
	else
	{
		pitch = 0;
		if (GPTCP_target_pose_.position.y > 0)
			roll = -M_PI;
		else
			roll = M_PI;
	}

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
