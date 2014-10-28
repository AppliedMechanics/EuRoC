/*
 * GetGraspPose2.cpp
 *
 *  Created on: Aug 13, 2014
 *      Author: euroc_student
 */

#include <GraspPose2.hpp>

GraspPose2::GraspPose2()
{
	//pi=3.1415926535897932384626433832795028841971693993751058;
	pi=3.1415927;
	gripper_maxwidth_=0.06;	//Linear axis goes from 0 to 0.07, so there is a minimum safety distance of 0.01 between finger and object.
	gripper_height_=0.06;
	gripper_finger_width=0.03;
	grip_safety_dist_=0.01;
	vertical_handle_safe_planar_offset=0.05;
	vertical_handle_safe_z_offset=0.05;
	vision_distance_object_height_cube_=0.2;
	vision_distance_object_height_cylinder_=0.2;
	vision_distance_object_height_handle_=0.2;
	place_falling_dist_=0.01;
	place_falling_distT6_=0.1;
	gripping_angle_deg_=45;
	gripping_angle_rad_=gripping_angle_deg_/180.0*(pi);
	gripping_angleT6_deg_=45;
	gripping_angleT6_rad_=gripping_angleT6_deg_/180.0*(pi);
}

GraspPose2::~GraspPose2() {
	// TODO Auto-generated destructor stub
}

void GraspPose2::set_object_data_(am_msgs::Object object, am_msgs::TargetZone target_zone)
{
	ROS_INFO("set_object_data_() called");
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

bool GraspPose2::return_grasp_pose(am_msgs::GetGraspPose::Request &req, am_msgs::GetGraspPose::Response &res)
{
	ROS_INFO("GraspPose_srv called and running...");
	set_object_data_(req.object, req.target_zone);
	compute_object_CoM_();
	compute_idx_shape_CoM_();
	compute_abs_shape_poses_();
	compute_bounding_box_();
	compute_grasp_poses_();


	ROS_INFO("transforming poses from GPTCP to LWRTCP frame");
	if (!get_transform_GPTCP_2_LWRTCP())
		msg_error("Could not get transform GPTCP --> LWRTCP");
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

	//throw invalid poses away and sort according to priority
	sort_poses();

	//compute the relative vectors
	compute_relative_vectors_();

	// Save everything to response
	res.object_mass = object_mass_;
	res.object_grip_pose=GPTCP_object_grip_pose;
	res.object_safe_pose=GPTCP_object_safe_pose;
	res.object_vision_pose=GPTCP_object_vision_pose;
	res.object_skip_vision=object_skip_vision;
	res.grip_pose_type=grip_pose_type;
	res.object_grasp_width=object_grasp_width;
	res.target_place_pose=GPTCP_target_place_pose;
	res.target_safe_pose=GPTCP_target_safe_pose;
	res.target_vision_pose=GPTCP_target_vision_pose;
	res.target_skip_vision=target_skip_vision;
	res.place_pose_type=place_pose_type;
	res.object_grip_r_tcp_com=object_grip_r_tcp_com;
	res.object_grip_r_gp_com=object_grip_r_gp_com;
	res.object_grip_r_gp_obj=object_grip_r_gp_obj;

//	int sz=res.target_place_pose.size();
//	ROS_INFO("target_place_pose.size(): %d",sz);
//	sz=res.target_safe_pose.size();
//	ROS_INFO("target_safe_pose.size(): %d",sz);
//	sz=res.target_vision_pose.size();
//	ROS_INFO("target_vision_pose.size(): %d",sz);

	//Print results of service call
	print_results();

	//send the whole bunch of poses to the tf broadcaster (for rviz)
	send_poses_to_tf_broadcaster();

	ROS_INFO("GraspPose_srv finished successfully");
	return true;
}

bool GraspPose2::return_grasp_poseT6(am_msgs::GetGraspPoseT6::Request &req, am_msgs::GetGraspPoseT6::Response &res)
{
	ROS_INFO("GraspPoseT6_srv called and running...");
	set_object_data_(req.object, req.target_zone);
	conveyor_belt=req.conveyor_belt;

	compute_object_CoM_();
	compute_idx_shape_CoM_();
	compute_abs_shape_poses_();
	compute_bounding_box_();
	compute_grasp_posesT6_();

	ROS_INFO("transforming poses from GPTCP to LWRTCP frame");
	if (!get_transform_GPTCP_2_LWRTCP())
		msg_error("Could not get transform GPTCP --> LWRTCP");

	LWRTCP_object_grip_pose.clear();
	LWRTCP_object_safe_pose.clear();
	LWRTCP_object_vision_pose.clear();
	LWRTCP_target_place_pose.clear();
	LWRTCP_target_safe_pose.clear();
	LWRTCP_target_vision_pose.clear();

	for(int ii=0; ii<GPTCP_object_grip_pose.size();ii++)
			{ LWRTCP_object_grip_pose.push_back(transform_pose_GPTCP_2_LWRTCP_(GPTCP_object_grip_pose[ii])); }
	for(int ii=0; ii<GPTCP_target_place_pose.size();ii++)
		{ LWRTCP_target_place_pose.push_back(transform_pose_GPTCP_2_LWRTCP_(GPTCP_target_place_pose[ii])); }

	//compute the relative vectors
	compute_relative_vectors_();

	// Save everything to response
	res.object_mass = object_mass_;
	res.object_grip_pose=GPTCP_object_grip_pose[0];
	res.object_grasp_width=object_grasp_width[0];
	res.target_place_pose=GPTCP_target_place_pose;
	res.object_grip_r_tcp_com=object_grip_r_tcp_com[0];
	res.object_grip_r_gp_com=object_grip_r_gp_com[0];
	res.object_grip_r_gp_obj=object_grip_r_gp_obj[0];

	//Print results of service call
	print_results();

	//send the whole bunch of poses to the tf broadcaster (for rviz)
	send_poses_to_tf_broadcaster();

	ROS_INFO("GraspPoseT6_srv finished successfully");
	return true;
}

void GraspPose2::compute_abs_shape_poses_()
{
	ROS_INFO("compute_abs_shape_poses_() called");
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
	}
}

void GraspPose2::compute_object_CoM_()
{
	ROS_INFO("compute_object_CoM_() called");
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
}

void GraspPose2::compute_idx_shape_CoM_()
{
	ROS_INFO("compute_idx_shape_CoM_() called");

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

void GraspPose2::compute_bounding_box_()
{
	ROS_INFO("compute_bounding_box_() called");
	double maxX, maxY;

	bbox_x_=0;
	bbox_y_=0;
	bbox_z_=0;

	switch(object_type_)
	{
		case OBJECT_CUBE:
			bbox_x_=object_.shape[0].size[0];
			bbox_y_=object_.shape[0].size[1];
			bbox_z_=object_.shape[0].size[2];
			break;
		case OBJECT_CYLINDER:
			bbox_x_=object_.shape[0].radius;
			bbox_y_=object_.shape[0].radius;
			bbox_z_=object_.shape[0].length;
			break;
		case OBJECT_HANDLE:
			maxX=0;
			maxY=0;
			for (uint8_t ii=0; ii<object_.nr_shapes; ii++)
			{
				if(!object_.shape[ii].type.compare("cylinder"))
				{
					if(object_.shape[ii].radius>maxX)
					{ maxX=object_.shape[ii].radius; }
					if(object_.shape[ii].radius>maxY)
					{ maxY=object_.shape[ii].radius; }
					bbox_z_=bbox_z_+object_.shape[ii].length;
				}
				else if (!object_.shape[ii].type.compare("box"))
				{
					if(object_.shape[ii].size[0]>maxX)
					{ maxX=object_.shape[ii].size[0]; }
					if(object_.shape[ii].size[1]>maxY)
					{ maxY=object_.shape[ii].size[1]; }
					bbox_z_=bbox_z_+object_.shape[ii].size[2];
				}
				else
				{
					//ignore this unknown shape
				}
			}
			bbox_x_=maxX;
			bbox_y_=maxY;
			break;
	}
}

void GraspPose2::compute_grasp_poses_()
{
	ROS_INFO("compute_grasp_poses_() called");
	geometry_msgs::Pose tmp_GPTCP_pose, tmp2_GPTCP_pose;
	double dot_product, dot_product2;
	tf::Vector3 x_axis(1,0,0), y_axis(0,1,0), z_axis(0,0,1);
	tf::Vector3 x_obj, y_obj, z_obj;
	tf::Vector3 x_grp, y_grp, z_grp;
	tf::Vector3 o_grip_point_;
	tf::Vector3 tmp_vec, tmp2_vec;
	tf::Vector3 target_vec;
	double min_distance_to_object;
	tf::Vector3 safe_to_grip_dir;
	tf::Matrix3x3 tmpRotation;
	bool gripper_big_enough;

	uint8_t upward_pointing_axis;
	uint8_t handle_cylinder_idx;
	uint8_t handle_box1_idx;
	uint8_t handle_box2_idx;
	uint8_t handle_grip_box_idx;
	uint8_t handle_posz_box_idx;
	uint8_t handle_negz_box_idx;
	uint8_t handle_place_downsidebox_idx;
	uint16_t grip_pose_type_;
	uint16_t place_pose_type_;
	double grasp_width;
	double object_height_;
	double handle_grippingdistance_cylinder;
	double handle_grippingdistance_box;

	//clear all existing poses
	GPTCP_object_grip_pose.clear();
	GPTCP_object_safe_pose.clear();
	GPTCP_object_vision_pose.clear();
	grip_pose_type.clear();
	object_grasp_width.clear();
	GPTCP_target_place_pose.clear();
	GPTCP_target_safe_pose.clear();
	GPTCP_target_vision_pose.clear();
	place_pose_type.clear();

	switch (object_type_)
	{
		case OBJECT_CUBE:
			//get axes of object
			x_obj = o_transform_shapes_[0].getBasis().getColumn(0);
			y_obj = o_transform_shapes_[0].getBasis().getColumn(1);
			z_obj = o_transform_shapes_[0].getBasis().getColumn(2);
			//=================================================================
			//--------------------------GRIPPING CUBE--------------------------
			//=================================================================
			//find upward pointing axis

			//default: x-axis points up
			upward_pointing_axis=0;
			object_pose_type_=OBJECT_POSE_CUBE_X_UP;
			object_height_=bbox_x_;

			dot_product = y_obj.dot(z_axis);
			if (am_abs(dot_product) > 0.9)
			{
				upward_pointing_axis=1;			//y-axis points up
				object_pose_type_=OBJECT_POSE_CUBE_Y_UP;
				object_height_=bbox_y_;
			}
			dot_product = z_obj.dot(z_axis);
			if (am_abs(dot_product) > 0.9)
			{
				upward_pointing_axis=2;			//z-axis points up
				object_pose_type_=OBJECT_POSE_CUBE_Z_UP;
				object_height_=bbox_z_;
			}

			if (upward_pointing_axis==0) 		//x-axis points up
			{
				for(uint8_t ii=1; ii<=8; ii++)
				{
					gripper_big_enough=false;
					if((ii==1 || ii==2 || ii==5 || ii==6) && object_.shape[0].size[1]<=gripper_maxwidth_)
					{
						gripper_big_enough=true;
						grasp_width=object_.shape[0].size[1];
					}
					if((ii==3 || ii==4 || ii==7 || ii==8) && object_.shape[0].size[2]<=gripper_maxwidth_)
					{
						gripper_big_enough=true;
						grasp_width=object_.shape[0].size[2];
					}
					if(ii<=4 && gripper_big_enough==true)
					{
						//"90 degree case"
						grip_pose_type_=GRIP_POSE_CUBE_X_UP;
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
						if(ii==1)	//possibility 1
							y_grp = z_obj;
						if(ii==2)	//possibility 2
							y_grp = -z_obj;
						if(ii==3)	//possibility 3
							y_grp = y_obj;
						if(ii==4)	//possibility 4
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

						grip_pose_type.push_back(grip_pose_type_);
						object_grasp_width.push_back(grasp_width);
					}
					if(ii>=5 && ii<=8 && gripper_big_enough==true)
					{
						//"custom angle case"
						z_grp = -1.0*z_axis;
						if(ii==5)	//possibility 5
							y_grp = z_obj;
						if(ii==6)	//possibility 6
							y_grp = -z_obj;
						if(ii==7)	//possibility 7
							y_grp = y_obj;
						if(ii==8)	//possibility 8
							y_grp = -y_obj;
						if(ii==5 || ii==6)
							grip_pose_type_=GRIP_POSE_CUBE_X_UP_45byY;
						if(ii==7 || ii==8)
							grip_pose_type_=GRIP_POSE_CUBE_X_UP_45byZ;

						y_grp.setZ(0);	//only use rotation around vertical axis
						x_grp = y_grp.cross(z_grp);
						set_orientation_from_axes(tmp2_GPTCP_pose,x_grp,y_grp,z_grp);
						tmp2_GPTCP_pose.position.x=o_object_com_.getX();
						tmp2_GPTCP_pose.position.y=o_object_com_.getY();
						tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
						GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);

						tmpRotation=get_rotationmatrixfromaxis(x_grp,gripping_angle_rad_);
						y_grp=tmpRotation*y_grp;
						z_grp=tmpRotation*z_grp;
						set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);
						safe_to_grip_dir=z_grp/z_grp.length();
						tmp_vec=o_object_com_;
						tmp_vec=tmp_vec+0.5*object_.shape[0].size[upward_pointing_axis]*z_axis;
						if(ii==5)
							tmp_vec=tmp_vec+0.5*object_.shape[0].size[2]*z_obj;
						if(ii==6)
							tmp_vec=tmp_vec-0.5*object_.shape[0].size[2]*z_obj;
						if(ii==7)
							tmp_vec=tmp_vec+0.5*object_.shape[0].size[1]*y_obj;
						if(ii==8)
							tmp_vec=tmp_vec-0.5*object_.shape[0].size[1]*y_obj;
						tmp_vec=tmp_vec-grip_safety_dist_*safe_to_grip_dir;
						//avoid collision with floor
						if(tmp_vec.getZ()<gripper_height_*cos(gripping_angle_rad_)+0.5*gripper_finger_width*sin(gripping_angle_rad_)+grip_safety_dist_)
							tmp_vec.setZ(gripper_height_*cos(gripping_angle_rad_)+0.5*gripper_finger_width*sin(gripping_angle_rad_)+grip_safety_dist_);
						tmp_GPTCP_pose.position.x = tmp_vec.getX();
						tmp_GPTCP_pose.position.y = tmp_vec.getY();
						tmp_GPTCP_pose.position.z = tmp_vec.getZ();
						GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);

						tmp2_vec=tmp_vec-gripper_height_*safe_to_grip_dir;
						tmp2_GPTCP_pose=tmp_GPTCP_pose;
						tmp2_GPTCP_pose.position.x = tmp2_vec.getX();
						tmp2_GPTCP_pose.position.y = tmp2_vec.getY();
						tmp2_GPTCP_pose.position.z = tmp2_vec.getZ();
						GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);

						grip_pose_type.push_back(grip_pose_type_);
						object_grasp_width.push_back(grasp_width);
					}
				}
			}
			if (upward_pointing_axis==1) 		//y-axis points up
			{
				for(uint8_t ii=1; ii<=8; ii++)
				{
					gripper_big_enough=false;
					if((ii==1 || ii==2 || ii==5 || ii==6) && object_.shape[0].size[0]<=gripper_maxwidth_)
					{
						gripper_big_enough=true;
						grasp_width=object_.shape[0].size[0];
					}
					if((ii==3 || ii==4 || ii==7 || ii==8) && object_.shape[0].size[2]<=gripper_maxwidth_)
					{
						gripper_big_enough=true;
						grasp_width=object_.shape[0].size[2];
					}
					if(ii<=4 && gripper_big_enough==true)
					{
						//"90 degree case"
						grip_pose_type_=GRIP_POSE_CUBE_Y_UP;
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
						if(ii==1)	//possibility 1
							y_grp = z_obj;
						if(ii==2)	//possibility 2
							y_grp = -z_obj;
						if(ii==3)	//possibility 3
							y_grp = x_obj;
						if(ii==4)	//possibility 4
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

						grip_pose_type.push_back(grip_pose_type_);
						object_grasp_width.push_back(grasp_width);
					}
					if(ii>=5 && ii<=8 && gripper_big_enough==true)
					{
						//"custom angle case"
						z_grp = -1.0*z_axis;
						if(ii==5)	//possibility 5
							y_grp = z_obj;
						if(ii==6)	//possibility 6
							y_grp = -z_obj;
						if(ii==7)	//possibility 7
							y_grp = x_obj;
						if(ii==8)	//possibility 8
							y_grp = -x_obj;
						if(ii==5 || ii==6)
							grip_pose_type_=GRIP_POSE_CUBE_Y_UP_45byX;
						if(ii==7 || ii==8)
							grip_pose_type_=GRIP_POSE_CUBE_Y_UP_45byZ;

						y_grp.setZ(0);	//only use rotation around vertical axis
						x_grp = y_grp.cross(z_grp);
						set_orientation_from_axes(tmp2_GPTCP_pose,x_grp,y_grp,z_grp);
						tmp2_GPTCP_pose.position.x=o_object_com_.getX();
						tmp2_GPTCP_pose.position.y=o_object_com_.getY();
						tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
						GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);

						tmpRotation=get_rotationmatrixfromaxis(x_grp,gripping_angle_rad_);
						y_grp=tmpRotation*y_grp;
						z_grp=tmpRotation*z_grp;
						set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);
						safe_to_grip_dir=z_grp/z_grp.length();
						tmp_vec=o_object_com_;
						tmp_vec=tmp_vec+0.5*object_.shape[0].size[upward_pointing_axis]*z_axis;
						if(ii==5)
							tmp_vec=tmp_vec+0.5*object_.shape[0].size[2]*z_obj;
						if(ii==6)
							tmp_vec=tmp_vec-0.5*object_.shape[0].size[2]*z_obj;
						if(ii==7)
							tmp_vec=tmp_vec+0.5*object_.shape[0].size[0]*x_obj;
						if(ii==8)
							tmp_vec=tmp_vec-0.5*object_.shape[0].size[0]*x_obj;
						tmp_vec=tmp_vec-grip_safety_dist_*safe_to_grip_dir;
						//avoid collision with floor
						if(tmp_vec.getZ()<gripper_height_*cos(gripping_angle_rad_)+0.5*gripper_finger_width*sin(gripping_angle_rad_)+grip_safety_dist_)
							tmp_vec.setZ(gripper_height_*cos(gripping_angle_rad_)+0.5*gripper_finger_width*sin(gripping_angle_rad_)+grip_safety_dist_);
						tmp_GPTCP_pose.position.x = tmp_vec.getX();
						tmp_GPTCP_pose.position.y = tmp_vec.getY();
						tmp_GPTCP_pose.position.z = tmp_vec.getZ();
						GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);

						tmp2_vec=tmp_vec-gripper_height_*safe_to_grip_dir;
						tmp2_GPTCP_pose=tmp_GPTCP_pose;
						tmp2_GPTCP_pose.position.x = tmp2_vec.getX();
						tmp2_GPTCP_pose.position.y = tmp2_vec.getY();
						tmp2_GPTCP_pose.position.z = tmp2_vec.getZ();
						GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);

						grip_pose_type.push_back(grip_pose_type_);
						object_grasp_width.push_back(grasp_width);
					}
				}
			}
			if (upward_pointing_axis==2) 		//z-axis points up
			{
				for(uint8_t ii=1; ii<=8; ii++)
				{
					gripper_big_enough=false;
					if((ii==1 || ii==2 || ii==5 || ii==6) && object_.shape[0].size[0]<=gripper_maxwidth_)
					{
						gripper_big_enough=true;
						grasp_width=object_.shape[0].size[0];
					}
					if((ii==3 || ii==4 || ii==7 || ii==8) && object_.shape[0].size[1]<=gripper_maxwidth_)
					{
						gripper_big_enough=true;
						grasp_width=object_.shape[0].size[1];
					}
					if(ii<=4 && gripper_big_enough==true)
					{
						//"90 degree case"
						grip_pose_type_=GRIP_POSE_CUBE_Z_UP;
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
						if(ii==1)	//possibility 1
							y_grp = y_obj;
						if(ii==2)	//possibility 2
							y_grp = -y_obj;
						if(ii==3)	//possibility 3
							y_grp = x_obj;
						if(ii==4)	//possibility 4
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

						grip_pose_type.push_back(grip_pose_type_);
						object_grasp_width.push_back(grasp_width);
					}
					if(ii>=5 && ii<=8 && gripper_big_enough==true)
					{
						//"custom angle case"
						z_grp = -1.0*z_axis;
						if(ii==5)	//possibility 5
							y_grp = y_obj;
						if(ii==6)	//possibility 6
							y_grp = -y_obj;
						if(ii==7)	//possibility 7
							y_grp = x_obj;
						if(ii==8)	//possibility 8
							y_grp = -x_obj;
						if(ii==5 || ii==6)
							grip_pose_type_=GRIP_POSE_CUBE_Z_UP_45byX;
						if(ii==7 || ii==8)
							grip_pose_type_=GRIP_POSE_CUBE_Z_UP_45byY;

						y_grp.setZ(0);	//only use rotation around vertical axis
						x_grp = y_grp.cross(z_grp);
						set_orientation_from_axes(tmp2_GPTCP_pose,x_grp,y_grp,z_grp);
						tmp2_GPTCP_pose.position.x=o_object_com_.getX();
						tmp2_GPTCP_pose.position.y=o_object_com_.getY();
						tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
						GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);

						tmpRotation=get_rotationmatrixfromaxis(x_grp,gripping_angle_rad_);
						y_grp=tmpRotation*y_grp;
						z_grp=tmpRotation*z_grp;
						set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);
						safe_to_grip_dir=z_grp/z_grp.length();
						tmp_vec=o_object_com_;
						tmp_vec=tmp_vec+0.5*object_.shape[0].size[upward_pointing_axis]*z_axis;
						if(ii==5)
							tmp_vec=tmp_vec+0.5*object_.shape[0].size[1]*y_obj;
						if(ii==6)
							tmp_vec=tmp_vec-0.5*object_.shape[0].size[1]*y_obj;
						if(ii==7)
							tmp_vec=tmp_vec+0.5*object_.shape[0].size[0]*x_obj;
						if(ii==8)
							tmp_vec=tmp_vec-0.5*object_.shape[0].size[0]*x_obj;
						tmp_vec=tmp_vec-grip_safety_dist_*safe_to_grip_dir;
						//avoid collision with floor
						if(tmp_vec.getZ()<gripper_height_*cos(gripping_angle_rad_)+0.5*gripper_finger_width*sin(gripping_angle_rad_)+grip_safety_dist_)
							tmp_vec.setZ(gripper_height_*cos(gripping_angle_rad_)+0.5*gripper_finger_width*sin(gripping_angle_rad_)+grip_safety_dist_);
						tmp_GPTCP_pose.position.x = tmp_vec.getX();
						tmp_GPTCP_pose.position.y = tmp_vec.getY();
						tmp_GPTCP_pose.position.z = tmp_vec.getZ();
						GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);

						tmp2_vec=tmp_vec-gripper_height_*safe_to_grip_dir;
						tmp2_GPTCP_pose=tmp_GPTCP_pose;
						tmp2_GPTCP_pose.position.x = tmp2_vec.getX();
						tmp2_GPTCP_pose.position.y = tmp2_vec.getY();
						tmp2_GPTCP_pose.position.z = tmp2_vec.getZ();
						GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);

						grip_pose_type.push_back(grip_pose_type_);
						object_grasp_width.push_back(grasp_width);
					}
				}
			}

			//=================================================================
			//--------------------PLACING CUBE ON TARGET ZONE------------------
			//=================================================================
			//calculate all 12 possibilities
			for(uint8_t ii=1; ii<=12; ii++)
			{
				if(ii<=4)
				{
					//"90 degree case"
					switch(object_pose_type_)
					{
						case OBJECT_POSE_CUBE_X_UP:
							place_pose_type_=PLACE_POSE_CUBE_X_UP;
							break;
						case OBJECT_POSE_CUBE_Y_UP:
							place_pose_type_=PLACE_POSE_CUBE_Y_UP;
							break;
						case OBJECT_POSE_CUBE_Z_UP:
							place_pose_type_=PLACE_POSE_CUBE_Z_UP;
							break;
					}
					tmp_GPTCP_pose.position = target_zone_.position;
					z_grp = -1.0*z_axis;
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

					place_pose_type.push_back(place_pose_type_);
				}
				if(ii>=5 && ii<=12)
				{
					//"custom angle case"
					z_grp = -1.0*z_axis;
					if(ii==5 || ii==9)	//possibility 5+9
						y_grp=y_axis;
					if(ii==6 || ii==10)	//possibility 6+10
						y_grp=-y_axis;
					if(ii==7 || ii==11)	//possibility 7+11
						y_grp=x_axis;
					if(ii==8 || ii==12)	//possibility 8+12
						y_grp=-x_axis;
					if(ii>=5 && ii<=8)
					{
						switch(object_pose_type_)
						{
							case OBJECT_POSE_CUBE_X_UP:
								place_pose_type_=PLACE_POSE_CUBE_X_UP_45byY;
								break;
							case OBJECT_POSE_CUBE_Y_UP:
								place_pose_type_=PLACE_POSE_CUBE_Y_UP_45byX;
								break;
							case OBJECT_POSE_CUBE_Z_UP:
								place_pose_type_=PLACE_POSE_CUBE_Z_UP_45byX;
								break;
						}
					}
					if(ii>=9 && ii<=12)
					{
						switch(object_pose_type_)
						{
							case OBJECT_POSE_CUBE_X_UP:
								place_pose_type_=PLACE_POSE_CUBE_X_UP_45byZ;
								break;
							case OBJECT_POSE_CUBE_Y_UP:
								place_pose_type_=PLACE_POSE_CUBE_Y_UP_45byZ;
								break;
							case OBJECT_POSE_CUBE_Z_UP:
								place_pose_type_=PLACE_POSE_CUBE_Z_UP_45byY;
								break;
						}
					}
					x_grp = y_grp.cross(z_grp);
					set_orientation_from_axes(tmp2_GPTCP_pose,x_grp,y_grp,z_grp);
					tmp2_GPTCP_pose.position=target_zone_.position;
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cube_;
					GPTCP_target_vision_pose.push_back(tmp2_GPTCP_pose);
					tmpRotation=get_rotationmatrixfromaxis(x_grp,gripping_angle_rad_);
					y_grp=tmpRotation*y_grp;
					z_grp=tmpRotation*z_grp;
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

					safe_to_grip_dir=z_grp/z_grp.length();
					tmp_vec.setX(target_zone_.position.x);
					tmp_vec.setY(target_zone_.position.y);
					tmp_vec.setZ(o_object_com_.getZ());
					tmp_vec=tmp_vec+0.5*object_.shape[0].size[upward_pointing_axis]*z_axis;
					switch(object_pose_type_)
					{
						case OBJECT_POSE_CUBE_X_UP:
							if(ii==5)
								tmp_vec=tmp_vec+0.5*object_.shape[0].size[2]*y_axis;
							if(ii==6)
								tmp_vec=tmp_vec-0.5*object_.shape[0].size[2]*y_axis;
							if(ii==7)
								tmp_vec=tmp_vec+0.5*object_.shape[0].size[2]*x_axis;
							if(ii==8)
								tmp_vec=tmp_vec-0.5*object_.shape[0].size[2]*x_axis;
							if(ii==9)
								tmp_vec=tmp_vec+0.5*object_.shape[0].size[1]*y_axis;
							if(ii==10)
								tmp_vec=tmp_vec-0.5*object_.shape[0].size[1]*y_axis;
							if(ii==11)
								tmp_vec=tmp_vec+0.5*object_.shape[0].size[1]*x_axis;
							if(ii==12)
								tmp_vec=tmp_vec-0.5*object_.shape[0].size[1]*x_axis;
							break;
						case OBJECT_POSE_CUBE_Y_UP:
							if(ii==5)
								tmp_vec=tmp_vec+0.5*object_.shape[0].size[2]*y_axis;
							if(ii==6)
								tmp_vec=tmp_vec-0.5*object_.shape[0].size[2]*y_axis;
							if(ii==7)
								tmp_vec=tmp_vec+0.5*object_.shape[0].size[2]*x_axis;
							if(ii==8)
								tmp_vec=tmp_vec-0.5*object_.shape[0].size[2]*x_axis;
							if(ii==9)
								tmp_vec=tmp_vec+0.5*object_.shape[0].size[0]*y_axis;
							if(ii==10)
								tmp_vec=tmp_vec-0.5*object_.shape[0].size[0]*y_axis;
							if(ii==11)
								tmp_vec=tmp_vec+0.5*object_.shape[0].size[0]*x_axis;
							if(ii==12)
								tmp_vec=tmp_vec-0.5*object_.shape[0].size[0]*x_axis;
							break;
						case OBJECT_POSE_CUBE_Z_UP:
							if(ii==5)
								tmp_vec=tmp_vec+0.5*object_.shape[0].size[1]*y_axis;
							if(ii==6)
								tmp_vec=tmp_vec-0.5*object_.shape[0].size[1]*y_axis;
							if(ii==7)
								tmp_vec=tmp_vec+0.5*object_.shape[0].size[1]*x_axis;
							if(ii==8)
								tmp_vec=tmp_vec-0.5*object_.shape[0].size[1]*x_axis;
							if(ii==9)
								tmp_vec=tmp_vec+0.5*object_.shape[0].size[0]*y_axis;
							if(ii==10)
								tmp_vec=tmp_vec-0.5*object_.shape[0].size[0]*y_axis;
							if(ii==11)
								tmp_vec=tmp_vec+0.5*object_.shape[0].size[0]*x_axis;
							if(ii==12)
								tmp_vec=tmp_vec-0.5*object_.shape[0].size[0]*x_axis;
							break;
					}
					tmp_vec=tmp_vec-grip_safety_dist_*safe_to_grip_dir;
					//avoid collision with floor
					if(tmp_vec.getZ()<gripper_height_*cos(gripping_angle_rad_)+0.5*gripper_finger_width*sin(gripping_angle_rad_)+grip_safety_dist_)
						tmp_vec.setZ(gripper_height_*cos(gripping_angle_rad_)+0.5*gripper_finger_width*sin(gripping_angle_rad_)+grip_safety_dist_);
					tmp_vec.setZ(tmp_vec.getZ()+place_falling_dist_);
					tmp_GPTCP_pose.position.x = tmp_vec.getX();
					tmp_GPTCP_pose.position.y = tmp_vec.getY();
					tmp_GPTCP_pose.position.z = tmp_vec.getZ();
					GPTCP_target_place_pose.push_back(tmp_GPTCP_pose);

					tmp2_vec=tmp_vec-gripper_height_*safe_to_grip_dir;
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.x = tmp2_vec.getX();
					tmp2_GPTCP_pose.position.y = tmp2_vec.getY();
					tmp2_GPTCP_pose.position.z = tmp2_vec.getZ();
					GPTCP_target_safe_pose.push_back(tmp2_GPTCP_pose);

					place_pose_type.push_back(place_pose_type_);
				}
			}
			break;
		case OBJECT_CYLINDER:
			//get axes of object
			x_obj = o_transform_shapes_[0].getBasis().getColumn(0);
			y_obj = o_transform_shapes_[0].getBasis().getColumn(1);
			z_obj = o_transform_shapes_[0].getBasis().getColumn(2);
			//=================================================================
			//-------------------------GRIPPING CYLINDER-----------------------
			//=================================================================
			//detect cylinder alignment
			dot_product = z_obj.dot(z_axis);
			if (dot_product > -0.7 && dot_product < 0.7)
			{
				//the cylinder is rolling, the object has to be skipped, so don't calculate poses
				object_pose_type_=OBJECT_POSE_CYLINDER_HORIZONTAL;
				object_height_=bbox_x_;
				return;
			}
			else
			{
				object_pose_type_=OBJECT_POSE_CYLINDER_VERTICAL;

				place_pose_type_=PLACE_POSE_CYLINDER_VERTICAL;
				object_height_=bbox_z_;
			}
			//calculate 8 possibilities
			for(uint8_t ii=1; ii<=8; ii++)
			{
				z_grp = -1.0*z_axis;
				if(ii==1 || ii==5)	//possibility 1+5
					y_grp=y_axis;
				if(ii==2 || ii==6)	//possibility 2+6
					y_grp=-y_axis;
				if(ii==3 || ii==7)	//possibility 3+7
					y_grp=x_axis;
				if(ii==4 || ii==8)	//possibility 4+8
					y_grp=-x_axis;
				x_grp = y_grp.cross(z_grp);
				if(ii<=4)
				{
					grip_pose_type_=GRIP_POSE_CYLINDER_VERTICAL;
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);
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
					GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);

					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z+=gripper_height_;
					GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);

					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cylinder_;
					GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);

					grip_pose_type.push_back(grip_pose_type_);
					object_grasp_width.push_back(2.0*object_.shape[0].radius);
				}
				if(ii>=5 && ii<=8)
				{
					grip_pose_type_=GRIP_POSE_CYLINDER_VERTICAL_45;
					set_orientation_from_axes(tmp2_GPTCP_pose,x_grp,y_grp,z_grp);
					tmp2_GPTCP_pose.position.x=o_object_com_.getX();
					tmp2_GPTCP_pose.position.y=o_object_com_.getY();
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cylinder_;
					GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);

					tmpRotation=get_rotationmatrixfromaxis(x_grp,gripping_angle_rad_);
					y_grp=tmpRotation*y_grp;
					z_grp=tmpRotation*z_grp;
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);
					safe_to_grip_dir=z_grp/z_grp.length();
					tmp_vec=o_object_com_;
					tmp_vec=tmp_vec+0.5*object_.shape[0].length*z_axis;
					if(ii==5)
						tmp_vec=tmp_vec+object_.shape[0].radius*y_axis;
					if(ii==6)
						tmp_vec=tmp_vec-object_.shape[0].radius*y_axis;
					if(ii==7)
						tmp_vec=tmp_vec+object_.shape[0].radius*x_axis;
					if(ii==8)
						tmp_vec=tmp_vec-object_.shape[0].radius*x_axis;
					tmp_vec=tmp_vec-grip_safety_dist_*safe_to_grip_dir;
					//avoid collision with floor
					if(tmp_vec.getZ()<gripper_height_*cos(gripping_angle_rad_)+0.5*gripper_finger_width*sin(gripping_angle_rad_)+grip_safety_dist_)
						tmp_vec.setZ(gripper_height_*cos(gripping_angle_rad_)+0.5*gripper_finger_width*sin(gripping_angle_rad_)+grip_safety_dist_);
					tmp_GPTCP_pose.position.x = tmp_vec.getX();
					tmp_GPTCP_pose.position.y = tmp_vec.getY();
					tmp_GPTCP_pose.position.z = tmp_vec.getZ();
					GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);

					tmp2_vec=tmp_vec-gripper_height_*safe_to_grip_dir;
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.x = tmp2_vec.getX();
					tmp2_GPTCP_pose.position.y = tmp2_vec.getY();
					tmp2_GPTCP_pose.position.z = tmp2_vec.getZ();
					GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);

					grip_pose_type.push_back(grip_pose_type_);
					object_grasp_width.push_back(2.0*object_.shape[0].radius);
				}
			}
			//=================================================================
			//------------------PLACING CYLINDER ON TARGET ZONE----------------
			//=================================================================
			//calculate all 8 possibilities
			for(uint8_t ii=1; ii<=8; ii++)
			{
				z_grp = -1.0*z_axis;
				if(ii==1 || ii==5)	//possibility 1+5
					y_grp=y_axis;
				if(ii==2 || ii==6)	//possibility 2+6
					y_grp=-y_axis;
				if(ii==3 || ii==7)	//possibility 3+7
					y_grp=x_axis;
				if(ii==4 || ii==8)	//possibility 4+8
					y_grp=-x_axis;
				x_grp = y_grp.cross(z_grp);
				if(ii<=4)
				{
					place_pose_type_=PLACE_POSE_CYLINDER_VERTICAL;
					tmp_GPTCP_pose.position = target_zone_.position;
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
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);
					GPTCP_target_place_pose.push_back(tmp_GPTCP_pose);

					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z+=gripper_height_;
					GPTCP_target_safe_pose.push_back(tmp2_GPTCP_pose);

					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cylinder_;
					GPTCP_target_vision_pose.push_back(tmp2_GPTCP_pose);

					place_pose_type.push_back(place_pose_type_);
				}
				if(ii>=5 && ii<=8)
				{
					place_pose_type_=PLACE_POSE_CYLINDER_VERTICAL_45;
					set_orientation_from_axes(tmp2_GPTCP_pose,x_grp,y_grp,z_grp);
					tmp2_GPTCP_pose.position=target_zone_.position;
					tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_cylinder_;
					GPTCP_target_vision_pose.push_back(tmp2_GPTCP_pose);
					tmpRotation=get_rotationmatrixfromaxis(x_grp,gripping_angle_rad_);
					y_grp=tmpRotation*y_grp;
					z_grp=tmpRotation*z_grp;
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

					safe_to_grip_dir=z_grp/z_grp.length();
					tmp_vec.setX(target_zone_.position.x);
					tmp_vec.setY(target_zone_.position.y);
					tmp_vec.setZ(o_object_com_.getZ());
					tmp_vec=tmp_vec+0.5*object_.shape[0].length*z_axis;
					if(ii==5)
						tmp_vec=tmp_vec+object_.shape[0].radius*y_axis;
					if(ii==6)
						tmp_vec=tmp_vec-object_.shape[0].radius*y_axis;
					if(ii==7)
						tmp_vec=tmp_vec+object_.shape[0].radius*x_axis;
					if(ii==8)
						tmp_vec=tmp_vec-object_.shape[0].radius*x_axis;
					tmp_vec=tmp_vec-grip_safety_dist_*safe_to_grip_dir;
					//avoid collision with floor
					if(tmp_vec.getZ()<gripper_height_*cos(gripping_angle_rad_)+0.5*gripper_finger_width*sin(gripping_angle_rad_)+grip_safety_dist_)
						tmp_vec.setZ(gripper_height_*cos(gripping_angle_rad_)+0.5*gripper_finger_width*sin(gripping_angle_rad_)+grip_safety_dist_);
					tmp_vec.setZ(tmp_vec.getZ()+place_falling_dist_);
					tmp_GPTCP_pose.position.x = tmp_vec.getX();
					tmp_GPTCP_pose.position.y = tmp_vec.getY();
					tmp_GPTCP_pose.position.z = tmp_vec.getZ();
					GPTCP_target_place_pose.push_back(tmp_GPTCP_pose);

					tmp2_vec=tmp_vec-gripper_height_*safe_to_grip_dir;
					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp2_GPTCP_pose.position.x = tmp2_vec.getX();
					tmp2_GPTCP_pose.position.y = tmp2_vec.getY();
					tmp2_GPTCP_pose.position.z = tmp2_vec.getZ();
					GPTCP_target_safe_pose.push_back(tmp2_GPTCP_pose);

					place_pose_type.push_back(place_pose_type_);
				}
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
			//default: handle_cylinder_idx==0
			handle_box1_idx=1;
			handle_box2_idx=2;

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

			//detect handle alignment
			dot_product = z_obj.dot(z_axis);
			if (dot_product > -0.7 && dot_product < 0.7)
			{
				//handle horizontal
				dot_product2 = x_obj.dot(z_axis);
				if (am_abs(dot_product) > 0.9)	//x-axis points up
				{
					object_pose_type_=OBJECT_POSE_HANDLE_HORIZONTAL_XUP;
					object_height_=bbox_x_;
					upward_pointing_axis=0;
				}
				else							//y-axis points up
				{
					object_pose_type_=OBJECT_POSE_HANDLE_HORIZONTAL_YUP;
					object_height_=bbox_y_;
					upward_pointing_axis=1;
				}
			}
			else
			{
				//handle vertical
				if (dot_product > 0)	//z-axis points upwards
				{
					object_pose_type_=OBJECT_POSE_HANDLE_VERTICAL_ZUP;
					object_height_=bbox_z_;
					upward_pointing_axis=2;
				}
				else					//z-axis points downwards
				{
					object_pose_type_=OBJECT_POSE_HANDLE_VERTICAL_ZDOWN;
					object_height_=bbox_z_;
					upward_pointing_axis=2;
				}
			}

			//Detect which box is in positive z-direction of cylinder shape
			if(b_transform_shapes_[handle_box1_idx].getOrigin().getZ()>b_transform_shapes_[handle_box2_idx].getOrigin().getZ())
			{
				handle_posz_box_idx=handle_box1_idx;
				handle_negz_box_idx=handle_box2_idx;
			}
			else
			{
				handle_posz_box_idx=handle_box2_idx;
				handle_negz_box_idx=handle_box1_idx;
			}

			//=================================================================
			//--------------------------GRIPPING HANDLE------------------------
			//=================================================================
			if(object_pose_type_==OBJECT_POSE_HANDLE_HORIZONTAL_XUP || object_pose_type_==OBJECT_POSE_HANDLE_HORIZONTAL_YUP)
			{
				z_grp = -1.0*z_axis;
				tmp_GPTCP_pose.position.x = o_transform_shapes_[handle_cylinder_idx].getOrigin().getX();
				tmp_GPTCP_pose.position.y = o_transform_shapes_[handle_cylinder_idx].getOrigin().getY();
				tmp_GPTCP_pose.position.z = o_transform_shapes_[handle_cylinder_idx].getOrigin().getZ()+0.5*gripper_height_;
				//avoid collision with floor
				if (tmp_GPTCP_pose.position.z < (gripper_height_+grip_safety_dist_))
				{
					tmp_GPTCP_pose.position.z = gripper_height_+grip_safety_dist_;
				}
				//avoid collision with object
				if (tmp_GPTCP_pose.position.z < (o_transform_shapes_[handle_cylinder_idx].getOrigin().getZ()+object_.shape[handle_cylinder_idx].radius+grip_safety_dist_))
				{
					tmp_GPTCP_pose.position.z=o_transform_shapes_[handle_cylinder_idx].getOrigin().getZ()+object_.shape[handle_cylinder_idx].radius+grip_safety_dist_;
				}
				handle_grippingdistance_cylinder=tmp_GPTCP_pose.position.z-o_object_com_.getZ();

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

				if(object_pose_type_==OBJECT_POSE_HANDLE_HORIZONTAL_XUP)
				{
					grip_pose_type_=GRIP_POSE_HANDLE_CYLINDER_ZEQX_YPOSZ;
				}
				else
				{
					grip_pose_type_=GRIP_POSE_HANDLE_CYLINDER_ZEQY_YPOSZ;
				}
				grip_pose_type.push_back(grip_pose_type_);
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
				if(object_pose_type_==OBJECT_POSE_HANDLE_HORIZONTAL_XUP)
				{
					tmp2_GPTCP_pose.position.z=bbox_x_+vision_distance_object_height_handle_;
				}
				else
				{
					tmp2_GPTCP_pose.position.z=bbox_y_+vision_distance_object_height_handle_;
				}
				GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);

				if(object_pose_type_==OBJECT_POSE_HANDLE_HORIZONTAL_XUP)
				{
					grip_pose_type_=GRIP_POSE_HANDLE_CYLINDER_ZEQX_YNEGZ;
				}
				else
				{
					grip_pose_type_=GRIP_POSE_HANDLE_CYLINDER_ZEQY_YNEGZ;
				}
				grip_pose_type.push_back(grip_pose_type_);
				object_grasp_width.push_back(2.0*object_.shape[handle_cylinder_idx].radius);

				//possibility 3-10
				for(uint8_t ii=3; ii<=10; ii++)
				{
					if(ii==3)	//possibility 3
					{
						grip_pose_type_=GRIP_POSE_HANDLE_BOX1_ZEQX_YPOSZ;
						handle_grip_box_idx=handle_box1_idx;
						y_grp = z_obj;
						grasp_width=object_.shape[handle_grip_box_idx].size[1];
					}
					if(ii==4)	//possibility 4
					{
						grip_pose_type_=GRIP_POSE_HANDLE_BOX1_ZEQX_YNEGZ;
						handle_grip_box_idx=handle_box1_idx;
						y_grp = -z_obj;
						grasp_width=object_.shape[handle_grip_box_idx].size[1];
					}
					if(ii==5)	//possibility 5
					{
						grip_pose_type_=GRIP_POSE_HANDLE_BOX2_ZEQX_YPOSZ;
						handle_grip_box_idx=handle_box2_idx;
						y_grp = z_obj;
						grasp_width=object_.shape[handle_grip_box_idx].size[1];
					}
					if(ii==6)	//possibility 6
					{
						grip_pose_type_=GRIP_POSE_HANDLE_BOX2_ZEQX_YNEGZ;
						handle_grip_box_idx=handle_box2_idx;
						y_grp = -z_obj;
						grasp_width=object_.shape[handle_grip_box_idx].size[1];
					}
					if(ii==7)	//possibility 7
					{
						grip_pose_type_=GRIP_POSE_HANDLE_BOX1_ZEQY_YPOSZ;
						handle_grip_box_idx=handle_box1_idx;
						y_grp = z_obj;
						grasp_width=object_.shape[handle_grip_box_idx].size[0];
					}
					if(ii==8)	//possibility 8
					{
						grip_pose_type_=GRIP_POSE_HANDLE_BOX1_ZEQY_YNEGZ;
						handle_grip_box_idx=handle_box1_idx;
						y_grp = -z_obj;
						grasp_width=object_.shape[handle_grip_box_idx].size[0];
					}
					if(ii==9)	//possibility 9
					{
						grip_pose_type_=GRIP_POSE_HANDLE_BOX2_ZEQY_YPOSZ;
						handle_grip_box_idx=handle_box2_idx;
						y_grp = z_obj;
						grasp_width=object_.shape[handle_grip_box_idx].size[0];
					}
					if(ii==10)	//possibility 10
					{
						grip_pose_type_=GRIP_POSE_HANDLE_BOX2_ZEQY_YNEGZ;
						handle_grip_box_idx=handle_box2_idx;
						y_grp = -z_obj;
						grasp_width=object_.shape[handle_grip_box_idx].size[0];
					}

					if(grasp_width<=gripper_maxwidth_ &&
							((ii>=3 && ii<=6 && object_pose_type_==OBJECT_POSE_HANDLE_HORIZONTAL_XUP) ||
							 (ii>=7 && ii<=10 && object_pose_type_==OBJECT_POSE_HANDLE_HORIZONTAL_YUP)))
					{
						tmp_GPTCP_pose.position.x = o_transform_shapes_[handle_grip_box_idx].getOrigin().getX();
						tmp_GPTCP_pose.position.y = o_transform_shapes_[handle_grip_box_idx].getOrigin().getY();
						tmp_GPTCP_pose.position.z = o_transform_shapes_[handle_grip_box_idx].getOrigin().getZ()+0.5*gripper_height_;

						//avoid collision with floor
						if (tmp_GPTCP_pose.position.z < (gripper_height_+grip_safety_dist_))
						{
							tmp_GPTCP_pose.position.z = gripper_height_+grip_safety_dist_;
						}
						//avoid collision with object
						if (tmp_GPTCP_pose.position.z < (o_transform_shapes_[handle_grip_box_idx].getOrigin().getZ()+0.5*object_.shape[handle_grip_box_idx].size[upward_pointing_axis]+grip_safety_dist_))
						{
							tmp_GPTCP_pose.position.z=o_transform_shapes_[handle_grip_box_idx].getOrigin().getZ()+0.5*object_.shape[handle_grip_box_idx].size[upward_pointing_axis]+grip_safety_dist_;
						}
						handle_grippingdistance_box=tmp_GPTCP_pose.position.z-o_object_com_.getZ();

						y_grp.setZ(0);	//only use rotation around vertical axis
						x_grp = y_grp.cross(z_grp);
						set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);
						GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);

						tmp2_GPTCP_pose=tmp_GPTCP_pose;
						tmp2_GPTCP_pose.position.z+=gripper_height_;
						GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);

						tmp2_GPTCP_pose=tmp_GPTCP_pose;
						tmp2_GPTCP_pose.position.x = o_object_com_.getX();
						tmp2_GPTCP_pose.position.y = o_object_com_.getY();
						tmp2_GPTCP_pose.position.z=object_height_+vision_distance_object_height_handle_;
						GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);

						grip_pose_type.push_back(grip_pose_type_);
						object_grasp_width.push_back(grasp_width);
					}
				}
			}
			if(object_pose_type_==OBJECT_POSE_HANDLE_VERTICAL_ZUP || object_pose_type_==OBJECT_POSE_HANDLE_VERTICAL_ZDOWN)
			{
				//possibility 1 to 8
				for(uint8_t ii=1; ii<=8; ii++)
				{
					//possibility ii
					switch (ii)
					{
						case 1:
							y_grp=-z_axis;
							z_grp=y_axis;
							grip_pose_type_=GRIP_POSE_HANDLE_CYLINDER_ZEQY_YNEGZ;
							break;
						case 2:
							y_grp=-z_axis;
							z_grp=-x_axis;
							grip_pose_type_=GRIP_POSE_HANDLE_CYLINDER_ZEQX_YNEGZ;
							break;
						case 3:
							y_grp=-z_axis;
							z_grp=-y_axis;
							grip_pose_type_=GRIP_POSE_HANDLE_CYLINDER_ZEQY_YNEGZ;
							break;
						case 4:
							y_grp=-z_axis;
							z_grp=x_axis;
							grip_pose_type_=GRIP_POSE_HANDLE_CYLINDER_ZEQX_YNEGZ;
							break;
						case 5:
							y_grp=z_axis;
							z_grp=y_axis;
							grip_pose_type_=GRIP_POSE_HANDLE_CYLINDER_ZEQY_YPOSZ;
							break;
						case 6:
							y_grp=z_axis;
							z_grp=-x_axis;
							grip_pose_type_=GRIP_POSE_HANDLE_CYLINDER_ZEQX_YPOSZ;
							break;
						case 7:
							y_grp=z_axis;
							z_grp=-y_axis;
							grip_pose_type_=GRIP_POSE_HANDLE_CYLINDER_ZEQY_YPOSZ;
							break;
						case 8:
							y_grp=z_axis;
							z_grp=x_axis;
							grip_pose_type_=GRIP_POSE_HANDLE_CYLINDER_ZEQX_YPOSZ;
							break;
					}

					z_grp.setZ(0);	//only use rotation around vertical axis
					x_grp = y_grp.cross(z_grp);
					set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

					safe_to_grip_dir=z_grp/z_grp.length();
					min_distance_to_object=object_.shape[handle_cylinder_idx].radius+grip_safety_dist_;
					if (min_distance_to_object > 0.5*gripper_height_)
					{
						tmp_vec=o_transform_shapes_[handle_cylinder_idx].getOrigin()-safe_to_grip_dir*min_distance_to_object;
						handle_grippingdistance_cylinder=min_distance_to_object;
					}
					else
					{
						tmp_vec=o_transform_shapes_[handle_cylinder_idx].getOrigin()-safe_to_grip_dir*0.5*gripper_height_;
						handle_grippingdistance_cylinder=0.5*gripper_height_;
					}

					tmp_GPTCP_pose.position.x=tmp_vec.getX();
					tmp_GPTCP_pose.position.y=tmp_vec.getY();
					tmp_GPTCP_pose.position.z=tmp_vec.getZ();
					GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);

					tmp2_GPTCP_pose=tmp_GPTCP_pose;
					tmp_vec=tmp_vec-safe_to_grip_dir*(gripper_height_+vertical_handle_safe_planar_offset);
					tmp_vec.setZ(tmp_vec.getZ()+vertical_handle_safe_z_offset);
					tmp2_GPTCP_pose.position.x=tmp_vec.getX();
					tmp2_GPTCP_pose.position.y=tmp_vec.getY();
					tmp2_GPTCP_pose.position.z=tmp_vec.getZ();
					GPTCP_object_safe_pose.push_back(tmp2_GPTCP_pose);

					z_grp=-z_axis;
					switch (ii)
					{
						case 1:
							y_grp=-y_axis;
							break;
						case 2:
							y_grp=x_axis;
							break;
						case 3:
							y_grp=y_axis;
							break;
						case 4:
							y_grp=-x_axis;
							break;
						case 5:
							y_grp=y_axis;
							break;
						case 6:
							y_grp=-x_axis;
							break;
						case 7:
							y_grp=-y_axis;
							break;
						case 8:
							y_grp=x_axis;
							break;
					}

					y_grp.setZ(0);	//only use rotation around vertical axis
					x_grp = y_grp.cross(z_grp);
					set_orientation_from_axes(tmp2_GPTCP_pose,x_grp,y_grp,z_grp);

					tmp2_GPTCP_pose.position.x=o_transform_shapes_[handle_cylinder_idx].getOrigin().getX();
					tmp2_GPTCP_pose.position.y=o_transform_shapes_[handle_cylinder_idx].getOrigin().getY();
					tmp2_GPTCP_pose.position.z=bbox_z_+vision_distance_object_height_handle_;
					GPTCP_object_vision_pose.push_back(tmp2_GPTCP_pose);

					grip_pose_type.push_back(grip_pose_type_);
					object_grasp_width.push_back(2.0*object_.shape[handle_cylinder_idx].radius);
				}
			}
			//=================================================================
			//--------------------PLACING HANDLE ON TARGET ZONE----------------
			//=================================================================
			target_vec.setX(target_zone_.position.x);
			target_vec.setY(target_zone_.position.y);

			//possibility 1 to 16
			for(uint8_t ii=1; ii<=16; ii++)
			{
				//possibility ii
				if(ii<=8)
				{
					place_pose_type_=PLACE_POSE_HANDLE_CYLINDER_YPOSZ_VERTICAL;
				}
				else
				{
					place_pose_type_=PLACE_POSE_HANDLE_CYLINDER_YNEGZ_VERTICAL;
				}
				switch (ii)
				{
					case 1:
						y_grp=-z_axis;
						z_grp=y_axis;
						handle_place_downsidebox_idx=handle_posz_box_idx;
						break;
					case 2:
						y_grp=-z_axis;
						z_grp=-x_axis;
						handle_place_downsidebox_idx=handle_posz_box_idx;
						break;
					case 3:
						y_grp=-z_axis;
						z_grp=-y_axis;
						handle_place_downsidebox_idx=handle_posz_box_idx;
						break;
					case 4:
						y_grp=-z_axis;
						z_grp=x_axis;
						handle_place_downsidebox_idx=handle_posz_box_idx;
						break;
					case 5:
						y_grp=z_axis;
						z_grp=y_axis;
						handle_place_downsidebox_idx=handle_negz_box_idx;
						break;
					case 6:
						y_grp=z_axis;
						z_grp=-x_axis;
						handle_place_downsidebox_idx=handle_negz_box_idx;
						break;
					case 7:
						y_grp=z_axis;
						z_grp=-y_axis;
						handle_place_downsidebox_idx=handle_negz_box_idx;
						break;
					case 8:
						y_grp=z_axis;
						z_grp=x_axis;
						handle_place_downsidebox_idx=handle_negz_box_idx;
						break;
					case 9:
						y_grp=-z_axis;
						z_grp=y_axis;
						handle_place_downsidebox_idx=handle_negz_box_idx;
						break;
					case 10:
						y_grp=-z_axis;
						z_grp=-x_axis;
						handle_place_downsidebox_idx=handle_negz_box_idx;
						break;
					case 11:
						y_grp=-z_axis;
						z_grp=-y_axis;
						handle_place_downsidebox_idx=handle_negz_box_idx;
						break;
					case 12:
						y_grp=-z_axis;
						z_grp=x_axis;
						handle_place_downsidebox_idx=handle_negz_box_idx;
						break;
					case 13:
						y_grp=z_axis;
						z_grp=y_axis;
						handle_place_downsidebox_idx=handle_posz_box_idx;
						break;
					case 14:
						y_grp=z_axis;
						z_grp=-x_axis;
						handle_place_downsidebox_idx=handle_posz_box_idx;
						break;
					case 15:
						y_grp=z_axis;
						z_grp=-y_axis;
						handle_place_downsidebox_idx=handle_posz_box_idx;
						break;
					case 16:
						y_grp=z_axis;
						z_grp=x_axis;
						handle_place_downsidebox_idx=handle_posz_box_idx;
						break;
				}
				z_grp.setZ(0);	//only use rotation around vertical axis
				x_grp = y_grp.cross(z_grp);
				set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);
				target_vec.setZ(0.5*object_.shape[handle_cylinder_idx].length+object_.shape[handle_place_downsidebox_idx].size[2]);

				//consider gripping offset
				safe_to_grip_dir=z_grp/z_grp.length();
				tmp_vec=target_vec-safe_to_grip_dir*handle_grippingdistance_cylinder;

				tmp_GPTCP_pose.position.x=tmp_vec.getX();
				tmp_GPTCP_pose.position.y=tmp_vec.getY();
				tmp_GPTCP_pose.position.z=tmp_vec.getZ()+place_falling_dist_;
				GPTCP_target_place_pose.push_back(tmp_GPTCP_pose);

				tmp2_GPTCP_pose=tmp_GPTCP_pose;
				tmp_vec=tmp_vec-safe_to_grip_dir*(gripper_height_+vertical_handle_safe_planar_offset);
				tmp_vec.setZ(tmp_vec.getZ()+vertical_handle_safe_z_offset);
				tmp2_GPTCP_pose.position.x=tmp_vec.getX();
				tmp2_GPTCP_pose.position.y=tmp_vec.getY();
				tmp2_GPTCP_pose.position.z=tmp_vec.getZ();
				GPTCP_target_safe_pose.push_back(tmp2_GPTCP_pose);

				z_grp=-z_axis;
				switch (ii)
				{
					case 1:
					case 9:
						y_grp=-y_axis;
						break;
					case 2:
					case 10:
						y_grp=x_axis;
						break;
					case 3:
					case 11:
						y_grp=y_axis;
						break;
					case 4:
					case 12:
						y_grp=-x_axis;
						break;
					case 5:
					case 13:
						y_grp=y_axis;
						break;
					case 6:
					case 14:
						y_grp=-x_axis;
						break;
					case 7:
					case 15:
						y_grp=-y_axis;
						break;
					case 8:
					case 16:
						y_grp=x_axis;
						break;
				}
				y_grp.setZ(0);	//only use rotation around vertical axis
				x_grp = y_grp.cross(z_grp);
				set_orientation_from_axes(tmp2_GPTCP_pose,x_grp,y_grp,z_grp);
				tmp2_GPTCP_pose.position.x=target_zone_.position.x;
				tmp2_GPTCP_pose.position.y=target_zone_.position.y;
				tmp2_GPTCP_pose.position.z=bbox_z_+vision_distance_object_height_handle_;
				GPTCP_target_vision_pose.push_back(tmp2_GPTCP_pose);

				place_pose_type.push_back(place_pose_type_);
			}

			//possibility 17 to 32
			for(uint8_t ii=17; ii<=32; ii++)
			{
				//possibility ii
				if(ii>=17 && ii<=20)
				{
					place_pose_type_=PLACE_POSE_HANDLE_BOX1_YPOSZ_VERTICAL;
					handle_grip_box_idx=handle_box1_idx;
					if(handle_posz_box_idx==handle_box1_idx)
					{
						y_grp=z_axis;
					}
					else
					{
						y_grp=-z_axis;
					}
				}
				if(ii>=21 && ii<=24)
				{
					place_pose_type_=PLACE_POSE_HANDLE_BOX1_YNEGZ_VERTICAL;
					handle_grip_box_idx=handle_box1_idx;
					if(handle_posz_box_idx==handle_box1_idx)
					{
						y_grp=-z_axis;
					}
					else
					{
						y_grp=z_axis;
					}
				}
				if(ii>=25 && ii<=28)
				{
					place_pose_type_=PLACE_POSE_HANDLE_BOX2_YPOSZ_VERTICAL;
					handle_grip_box_idx=handle_box2_idx;
					if(handle_posz_box_idx==handle_box2_idx)
					{
						y_grp=z_axis;
					}
					else
					{
						y_grp=-z_axis;
					}
				}
				if(ii>=29 && ii<=32)
				{
					place_pose_type_=PLACE_POSE_HANDLE_BOX2_YNEGZ_VERTICAL;
					handle_grip_box_idx=handle_box2_idx;
					if(handle_posz_box_idx==handle_box2_idx)
					{
						y_grp=-z_axis;
					}
					else
					{
						y_grp=z_axis;
					}
				}
				switch (ii)
				{
					case 17:
					case 21:
					case 25:
					case 29:
						z_grp=y_axis;
						break;
					case 18:
					case 22:
					case 26:
					case 30:
						z_grp=-x_axis;
						break;
					case 19:
					case 23:
					case 27:
					case 31:
						z_grp=-y_axis;
						break;
					case 20:
					case 24:
					case 28:
					case 32:
						z_grp=x_axis;
						break;
				}
				z_grp.setZ(0);	//only use rotation around vertical axis
				x_grp = y_grp.cross(z_grp);
				set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);
				target_vec.setZ(bbox_z_-0.5*object_.shape[handle_grip_box_idx].size[2]);

				//consider gripping offset
				safe_to_grip_dir=z_grp/z_grp.length();
				tmp_vec=target_vec-safe_to_grip_dir*handle_grippingdistance_box;

				tmp_GPTCP_pose.position.x=tmp_vec.getX();
				tmp_GPTCP_pose.position.y=tmp_vec.getY();
				tmp_GPTCP_pose.position.z=tmp_vec.getZ()+place_falling_dist_;
				GPTCP_target_place_pose.push_back(tmp_GPTCP_pose);

				tmp2_GPTCP_pose=tmp_GPTCP_pose;
				tmp2_GPTCP_pose.position.z=tmp_GPTCP_pose.position.z+0.5*gripper_finger_width+
										   0.5*object_.shape[handle_grip_box_idx].size[2]+grip_safety_dist_;
				GPTCP_target_safe_pose.push_back(tmp2_GPTCP_pose);

				z_grp=-z_axis;
				switch (ii)
				{
					case 17:
					case 21:
					case 25:
					case 29:
						y_grp=-y_axis;
						break;
					case 18:
					case 22:
					case 26:
					case 30:
						y_grp=x_axis;
						break;
					case 19:
					case 23:
					case 27:
					case 31:
						y_grp=y_axis;
						break;
					case 20:
					case 24:
					case 28:
					case 32:
						y_grp=-x_axis;
						break;
				}
				y_grp.setZ(0);	//only use rotation around vertical axis
				x_grp = y_grp.cross(z_grp);
				set_orientation_from_axes(tmp2_GPTCP_pose,x_grp,y_grp,z_grp);
				tmp2_GPTCP_pose.position.x=target_zone_.position.x;
				tmp2_GPTCP_pose.position.y=target_zone_.position.y;
				tmp2_GPTCP_pose.position.z=bbox_z_+vision_distance_object_height_handle_;
				GPTCP_target_vision_pose.push_back(tmp2_GPTCP_pose);

				place_pose_type.push_back(place_pose_type_);
			}
			break;
		default:
			break;
	}
}

void GraspPose2::compute_grasp_posesT6_()
{
	ROS_INFO("compute_grasp_posesT6_() called");
	geometry_msgs::Pose tmp_GPTCP_pose;
	tf::Vector3 x_axis(1,0,0), y_axis(0,1,0), z_axis(0,0,1);
	tf::Vector3 x_grp, y_grp, z_grp;
	tf::Vector3 tmp_vec;
	tf::Vector3 safe_to_grip_dir;
	tf::Matrix3x3 tmpRotation;

	//clear all existing poses
	GPTCP_object_grip_pose.clear();
	GPTCP_object_safe_pose.clear();
	GPTCP_object_vision_pose.clear();
	grip_pose_type.clear();
	object_grasp_width.clear();
	GPTCP_target_place_pose.clear();
	GPTCP_target_safe_pose.clear();
	GPTCP_target_vision_pose.clear();
	place_pose_type.clear();

	object_pose_type_=OBJECT_POSE_CUBE_TASK6;

	//=================================================================
	//--------------------------GRIPPING CUBE--------------------------
	//=================================================================
	z_grp = -1.0*z_axis;
	y_grp.setX(conveyor_belt.move_direction_and_length.x);
	y_grp.setY(conveyor_belt.move_direction_and_length.y);
	y_grp.setZ(conveyor_belt.move_direction_and_length.z);
	y_grp=y_grp/y_grp.length();
	y_grp.setZ(0);	//only use rotation around vertical axis
	x_grp = y_grp.cross(z_grp);

	tmpRotation=get_rotationmatrixfromaxis(x_grp,gripping_angleT6_rad_);
	y_grp=tmpRotation*y_grp;
	z_grp=tmpRotation*z_grp;
	set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);

	safe_to_grip_dir=z_grp/z_grp.length();
	tmp_vec=o_object_com_;
	tmp_vec.setZ(tmp_vec.getZ()-0.5*bbox_z_);
	tmp_vec=tmp_vec-gripper_height_*safe_to_grip_dir;
	tmp_vec.setZ(tmp_vec.getZ()+0.5*gripper_finger_width*sin(gripping_angleT6_rad_)+grip_safety_dist_);

	tmp_GPTCP_pose.position.x = tmp_vec.getX();
	tmp_GPTCP_pose.position.y = tmp_vec.getY();
	tmp_GPTCP_pose.position.z = tmp_vec.getZ();
	GPTCP_object_grip_pose.push_back(tmp_GPTCP_pose);
	grip_pose_type.push_back(GRIP_POSE_CUBE_TASK6);
	object_grasp_width.push_back(bbox_x_);	//assuming, that its really a cube

	//=================================================================
	//--------------------PLACING CUBE ON TARGET ZONE------------------
	//=================================================================
	tmp_GPTCP_pose.position = target_zone_.position;
	z_grp = -1.0*z_axis;
	tmp_GPTCP_pose.position.z=tmp_GPTCP_pose.position.z+gripper_height_+grip_safety_dist_+place_falling_distT6_;
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

		GPTCP_target_place_pose.push_back(tmp_GPTCP_pose);
		place_pose_type.push_back(PLACE_POSE_CUBE_TASK6);
	}
}

bool GraspPose2::get_transform_GPTCP_2_LWRTCP()
{
	tf::TransformListener tf_listener;
//	tf::StampedTransform tmp_transform_GPTCP_2_LWRTCP;

	ros::Time now = ros::Time(0);
	try{
		tf_listener.waitForTransform(LWR_TCP,GP_TCP,now,ros::Duration(2.0));
		tf_listener.lookupTransform(LWR_TCP,GP_TCP,now,transform_GPTCP_2_LWRTCP_);
		ROS_INFO("Listening to transform was successful");
	}
	catch(...)
	{
		msg_error("Listening to transform was not successful");
	}

	return true;
}

geometry_msgs::Pose GraspPose2::transform_pose_GPTCP_2_LWRTCP_(geometry_msgs::Pose GPTCP_pose)
{
	geometry_msgs::Pose LWRTCP_pose;
	tf::Transform tf_tmp1, tf_tmp2;

	tf_tmp1.setOrigin(tf::Vector3(GPTCP_pose.position.x,GPTCP_pose.position.y,GPTCP_pose.position.z));
	tf_tmp1.setRotation(tf::Quaternion(GPTCP_pose.orientation.x,GPTCP_pose.orientation.y,GPTCP_pose.orientation.z,GPTCP_pose.orientation.w));

	tf_tmp2.mult(tf_tmp1,transform_GPTCP_2_LWRTCP_.inverse());

	LWRTCP_pose.position.x 		= tf_tmp2.getOrigin().getX();
	LWRTCP_pose.position.y 		= tf_tmp2.getOrigin().getY();
	LWRTCP_pose.position.z 		= tf_tmp2.getOrigin().getZ();
	LWRTCP_pose.orientation.x 	= tf_tmp2.getRotation().getX();
	LWRTCP_pose.orientation.y 	= tf_tmp2.getRotation().getY();
	LWRTCP_pose.orientation.z 	= tf_tmp2.getRotation().getZ();
	LWRTCP_pose.orientation.w 	= tf_tmp2.getRotation().getW();
	return LWRTCP_pose;
}

void GraspPose2::compute_relative_vectors_()
{
	ROS_INFO("compute_relative_vectors_() called");
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
		object_grip_r_gp_com[ii].x = 0;
		object_grip_r_gp_com[ii].y = 0;
		object_grip_r_gp_com[ii].z = 0;
//		object_grip_r_gp_com[ii].x = object_grip_r_tcp_com[ii].x-transform_GPTCP_2_LWRTCP_.getOrigin().getX();
//		object_grip_r_gp_com[ii].y = object_grip_r_tcp_com[ii].y-transform_GPTCP_2_LWRTCP_.getOrigin().getY();
//		object_grip_r_gp_com[ii].z = object_grip_r_tcp_com[ii].z-transform_GPTCP_2_LWRTCP_.getOrigin().getZ();
//
//		//calculate vector from gripper tcp to object frame
		object_grip_r_gp_obj[ii].x = 0;
		object_grip_r_gp_obj[ii].y = 0;
		object_grip_r_gp_obj[ii].z = 0;
//		object_grip_r_gp_obj[ii].x = object_.abs_pose.position.x - LWRTCP_object_grip_pose[ii].position.x;
//		object_grip_r_gp_obj[ii].y = object_.abs_pose.position.y - LWRTCP_object_grip_pose[ii].position.y;
//		object_grip_r_gp_obj[ii].z = object_.abs_pose.position.z - LWRTCP_object_grip_pose[ii].position.z;
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

void GraspPose2::sort_poses()
{
	ROS_INFO("sort_poses() called");
	std::vector<uint16_t> object_grip_prio, object_safe_prio, object_vision_prio;
	std::vector<uint16_t> target_place_prio, target_safe_prio, target_vision_prio;
	bool swappeditems, doswap;
	int priosum1, priosum2;

	//check the service call
	if(ros::service::exists("CheckPoses_srv",false)==true)
	{
	    check_poses_client_ = n.serviceClient<am_msgs::CheckPoses>("CheckPoses_srv");
	}
	else
	{
		msg_error("Error. check_poses_client_ not available");
		return;
	}

	//check object_grip poses
	check_poses_srv_.request.poses=LWRTCP_object_grip_pose;
	if(!check_poses_client_.call(check_poses_srv_))
	{
		msg_error("Error. failed to call check_poses_client_");
		return;
	}
	else
	{
		object_grip_prio=check_poses_srv_.response.priority;
	}
	if(check_poses_srv_.request.poses.size()!=check_poses_srv_.response.priority.size())
	{
		msg_error("Error. check_poses_client_ returned a vector of invalid size");
		return;
	}

	//check object_safe poses
	check_poses_srv_.request.poses=LWRTCP_object_safe_pose;
	if(!check_poses_client_.call(check_poses_srv_))
	{
		msg_error("Error. failed to call check_poses_client_");
		return;
	}
	else
	{
		object_safe_prio=check_poses_srv_.response.priority;
	}
	if(check_poses_srv_.request.poses.size()!=check_poses_srv_.response.priority.size())
	{
		msg_error("Error. check_poses_client_ returned a vector of invalid size");
		return;
	}

	//check object_vision poses
	check_poses_srv_.request.poses=LWRTCP_object_vision_pose;
	if(!check_poses_client_.call(check_poses_srv_))
	{
		msg_error("Error. failed to call check_poses_client_");
		return;
	}
	else
	{
		object_vision_prio=check_poses_srv_.response.priority;
	}
	if(check_poses_srv_.request.poses.size()!=check_poses_srv_.response.priority.size())
	{
		msg_error("Error. check_poses_client_ returned a vector of invalid size");
		return;
	}

	//check target_place poses
	check_poses_srv_.request.poses=LWRTCP_target_place_pose;
	if(!check_poses_client_.call(check_poses_srv_))
	{
		msg_error("Error. failed to call check_poses_client_");
		return;
	}
	else
	{
		target_place_prio=check_poses_srv_.response.priority;
	}
	if(check_poses_srv_.request.poses.size()!=check_poses_srv_.response.priority.size())
	{
		msg_error("Error. check_poses_client_ returned a vector of invalid size");
		return;
	}

	//check target_safe poses
	check_poses_srv_.request.poses=LWRTCP_target_safe_pose;
	if(!check_poses_client_.call(check_poses_srv_))
	{
		msg_error("Error. failed to call check_poses_client_");
		return;
	}
	else
	{
		target_safe_prio=check_poses_srv_.response.priority;
	}
	if(check_poses_srv_.request.poses.size()!=check_poses_srv_.response.priority.size())
	{
		msg_error("Error. check_poses_client_ returned a vector of invalid size");
		return;
	}

	//check target_vision poses
	check_poses_srv_.request.poses=LWRTCP_target_vision_pose;
	if(!check_poses_client_.call(check_poses_srv_))
	{
		msg_error("Error. failed to call check_poses_client_");
		return;
	}
	else
	{
		target_vision_prio=check_poses_srv_.response.priority;
	}
	if(check_poses_srv_.request.poses.size()!=check_poses_srv_.response.priority.size())
	{
		msg_error("Error. check_poses_client_ returned a vector of invalid size");
		return;
	}

	//print priority
	ROS_INFO("priority: (0 means invalid)");
	ROS_INFO("Pose-Nr. | object_grip | object_safe | object_vision");
	for (uint8_t ii=0; ii<LWRTCP_object_grip_pose.size(); ii++)
	{
		ROS_INFO("[%d] | %d | %d | %d",ii,object_grip_prio[ii], object_safe_prio[ii], object_vision_prio[ii]);
	}
	ROS_INFO("Pose-Nr. | target_place | target_safe | target_vision");
	for (uint8_t ii=0; ii<LWRTCP_target_place_pose.size(); ii++)
	{
		ROS_INFO("[%d] | %d | %d | %d",ii,target_place_prio[ii], target_safe_prio[ii], target_vision_prio[ii]);
	}

	//remove invalid object poses
	object_skip_vision.clear();
	object_skip_vision.resize(LWRTCP_object_grip_pose.size(),0);
	int jj=0;
	while(jj<LWRTCP_object_grip_pose.size())
	{
		if((object_grip_prio[jj]==0) || (object_safe_prio[jj]==0))
		{
			object_grip_prio.erase(object_grip_prio.begin()+jj);
			object_safe_prio.erase(object_safe_prio.begin()+jj);
			object_vision_prio.erase(object_vision_prio.begin()+jj);
			LWRTCP_object_grip_pose.erase(LWRTCP_object_grip_pose.begin()+jj);
			LWRTCP_object_safe_pose.erase(LWRTCP_object_safe_pose.begin()+jj);
			LWRTCP_object_vision_pose.erase(LWRTCP_object_vision_pose.begin()+jj);
			GPTCP_object_grip_pose.erase(GPTCP_object_grip_pose.begin()+jj);
			GPTCP_object_safe_pose.erase(GPTCP_object_safe_pose.begin()+jj);
			GPTCP_object_vision_pose.erase(GPTCP_object_vision_pose.begin()+jj);
			object_skip_vision.erase(object_skip_vision.begin()+jj);
			grip_pose_type.erase(grip_pose_type.begin()+jj);
			object_grasp_width.erase(object_grasp_width.begin()+jj);
		}
		else
		{
			if(object_vision_prio[jj]==0)
			{
				object_skip_vision[jj]=1;
			}
			jj++;
		}
	}

	//remove invalid target poses
	target_skip_vision.clear();
	target_skip_vision.resize(LWRTCP_target_place_pose.size(),0);
	jj=0;
	while(jj<LWRTCP_target_place_pose.size())
	{
		if((target_place_prio[jj]==0) || (target_safe_prio[jj]==0))
		{
			target_place_prio.erase(target_place_prio.begin()+jj);
			target_safe_prio.erase(target_safe_prio.begin()+jj);
			target_vision_prio.erase(target_vision_prio.begin()+jj);
			LWRTCP_target_place_pose.erase(LWRTCP_target_place_pose.begin()+jj);
			LWRTCP_target_safe_pose.erase(LWRTCP_target_safe_pose.begin()+jj);
			LWRTCP_target_vision_pose.erase(LWRTCP_target_vision_pose.begin()+jj);
			GPTCP_target_place_pose.erase(GPTCP_target_place_pose.begin()+jj);
			GPTCP_target_safe_pose.erase(GPTCP_target_safe_pose.begin()+jj);
			GPTCP_target_vision_pose.erase(GPTCP_target_vision_pose.begin()+jj);
			target_skip_vision.erase(target_skip_vision.begin()+jj);
			place_pose_type.erase(place_pose_type.begin()+jj);
		}
		else
		{
			if(target_vision_prio[jj]==0)
			{
				target_skip_vision[jj]=1;
			}
			jj++;
		}
	}

	//sort the gripping poses according to their priority (lowest index has highest priority)
	swappeditems=true;
	int iimax=0;
	while(swappeditems==true)
	{
		swappeditems=false;
		iimax=LWRTCP_object_grip_pose.size()-1;
		for(int ii=0; ii<iimax; ii++)
		{
			doswap=false;
			if(object_vision_prio[ii]==0 && object_vision_prio[ii+1]>0)
			{
				//prefer pose-sequences with possible vision poses
				doswap=true;
			}
			else
			{
				//calculate an average priority over the pose-sequence
				priosum1=object_grip_prio[ii]+object_safe_prio[ii]+object_vision_prio[ii];
				priosum2=object_grip_prio[ii+1]+object_safe_prio[ii+1]+object_vision_prio[ii+1];
				if(priosum2>priosum1)
				{
					doswap=true;
				}
			}
			if(doswap==true)
			{
				std::swap(object_grip_prio[ii],object_grip_prio[ii+1]);
				std::swap(object_safe_prio[ii],object_safe_prio[ii+1]);
				std::swap(object_vision_prio[ii],object_vision_prio[ii+1]);
				std::swap(LWRTCP_object_grip_pose[ii],LWRTCP_object_grip_pose[ii+1]);
				std::swap(LWRTCP_object_safe_pose[ii],LWRTCP_object_safe_pose[ii+1]);
				std::swap(LWRTCP_object_vision_pose[ii],LWRTCP_object_vision_pose[ii+1]);
				std::swap(GPTCP_object_grip_pose[ii],GPTCP_object_grip_pose[ii+1]);
				std::swap(GPTCP_object_safe_pose[ii],GPTCP_object_safe_pose[ii+1]);
				std::swap(GPTCP_object_vision_pose[ii],GPTCP_object_vision_pose[ii+1]);
				std::swap(object_skip_vision[ii],object_skip_vision[ii+1]);
				std::swap(grip_pose_type[ii],grip_pose_type[ii+1]);
				std::swap(object_grasp_width[ii],object_grasp_width[ii+1]);
				swappeditems=true;
			}
		}
	}

	//sort the placing poses according to their priority (lowest index has highest priority)
	swappeditems=true;
	iimax=0;
	while(swappeditems==true)
	{
		swappeditems=false;
		iimax=LWRTCP_target_place_pose.size()-1;
		for(int ii=0; ii<iimax; ii++)
		{
			doswap=false;
			if(target_vision_prio[ii]==0 && target_vision_prio[ii+1]>0)
			{
				//prefer pose-sequences with possible vision poses
				doswap=true;
			}
			else
			{
				//calculate an average priority over the pose-sequence
				priosum1=target_place_prio[ii]+target_safe_prio[ii]+target_vision_prio[ii];
				priosum2=target_place_prio[ii+1]+target_safe_prio[ii+1]+target_vision_prio[ii+1];
				if(priosum2>priosum1)
				{
					doswap=true;
				}
			}
			if(doswap==true)
			{
				std::swap(target_place_prio[ii],target_place_prio[ii+1]);
				std::swap(target_safe_prio[ii],target_safe_prio[ii+1]);
				std::swap(target_vision_prio[ii],target_vision_prio[ii+1]);
				std::swap(LWRTCP_target_place_pose[ii],LWRTCP_target_place_pose[ii+1]);
				std::swap(LWRTCP_target_safe_pose[ii],LWRTCP_target_safe_pose[ii+1]);
				std::swap(LWRTCP_target_vision_pose[ii],LWRTCP_target_vision_pose[ii+1]);
				std::swap(GPTCP_target_place_pose[ii],GPTCP_target_place_pose[ii+1]);
				std::swap(GPTCP_target_safe_pose[ii],GPTCP_target_safe_pose[ii+1]);
				std::swap(GPTCP_target_vision_pose[ii],GPTCP_target_vision_pose[ii+1]);
				std::swap(target_skip_vision[ii],target_skip_vision[ii+1]);
				std::swap(place_pose_type[ii],place_pose_type[ii+1]);
				swappeditems=true;
			}
		}
	}
}

void GraspPose2::print_results()
{
	std::vector<geometry_msgs::Pose> tmpPoseArray;
	std::vector<geometry_msgs::Vector3> tmpVector3Array;

	ROS_INFO("===============================================");
	ROS_INFO("------------BEGIN GRASPING RESULTS-------------");
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
				ROS_INFO("shape[%d]: box - size: [%4.3f %4.3f %4.3f] | o_orig: [%4.3f %4.3f %4.3f]",ii,object_.shape[ii].size[0],object_.shape[ii].size[1],
						object_.shape[ii].size[2],o_transform_shapes_[ii].getOrigin().getX(),o_transform_shapes_[ii].getOrigin().getY(),o_transform_shapes_[ii].getOrigin().getZ());
			}
			if(!object_.shape[ii].type.compare("cylinder"))
			{
				ROS_INFO("shape[%d]: cylinder - radius: %4.3f | length: %4.3f | o_orig: [%4.3f %4.3f %4.3f]",ii,object_.shape[ii].radius,object_.shape[ii].length,
						o_transform_shapes_[ii].getOrigin().getX(),o_transform_shapes_[ii].getOrigin().getY(),o_transform_shapes_[ii].getOrigin().getZ());
			}
		}
	}
	ROS_INFO("bounding box dimensions (x y z): %4.3f %4.3f %4.3f",bbox_x_,bbox_y_,bbox_z_);
	switch(object_pose_type_)
	{
		case OBJECT_POSE_CUBE_X_UP:
			ROS_INFO("object_pose_type_: OBJECT_POSE_CUBE_X_UP");
			break;
		case OBJECT_POSE_CUBE_Y_UP:
			ROS_INFO("object_pose_type_: OBJECT_POSE_CUBE_Y_UP");
			break;
		case OBJECT_POSE_CUBE_Z_UP:
			ROS_INFO("object_pose_type_: OBJECT_POSE_CUBE_Z_UP");
			break;
		case OBJECT_POSE_CYLINDER_HORIZONTAL:
			ROS_INFO("object_pose_type_: OBJECT_POSE_CYLINDER_HORIZONTAL");
			break;
		case OBJECT_POSE_CYLINDER_VERTICAL:
			ROS_INFO("object_pose_type_: OBJECT_POSE_CYLINDER_VERTICAL");
			break;
		case OBJECT_POSE_HANDLE_HORIZONTAL_XUP:
			ROS_INFO("object_pose_type_: OBJECT_POSE_HANDLE_HORIZONTAL_XUP");
			break;
		case OBJECT_POSE_HANDLE_HORIZONTAL_YUP:
			ROS_INFO("object_pose_type_: OBJECT_POSE_HANDLE_HORIZONTAL_YUP");
			break;
		case OBJECT_POSE_HANDLE_VERTICAL_ZUP:
			ROS_INFO("object_pose_type_: OBJECT_POSE_HANDLE_VERTICAL_ZUP");
			break;
		case OBJECT_POSE_HANDLE_VERTICAL_ZDOWN:
			ROS_INFO("object_pose_type_: OBJECT_POSE_HANDLE_VERTICAL_ZDOWN");
			break;
		default:
			ROS_INFO("object_pose_type_: UNKNOWN");
			break;
	}

	tf::Matrix3x3 dcm;
	tf::Quaternion q_tmp;
	double tmp_Roll, tmp_Pitch, tmp_Yaw;

	q_tmp.setX(object_.abs_pose.orientation.x);
	q_tmp.setY(object_.abs_pose.orientation.y);
	q_tmp.setZ(object_.abs_pose.orientation.z);
	q_tmp.setW(object_.abs_pose.orientation.w);
	dcm = tf::Matrix3x3(q_tmp);
	dcm.getRPY(tmp_Roll, tmp_Pitch, tmp_Yaw);

	ROS_INFO("object absolute pose position: x=%4.3f y=%4.3f z=%4.3f",object_.abs_pose.position.x,object_.abs_pose.position.y,object_.abs_pose.position.z);
	ROS_INFO("object absolute pose orientation: x=%4.3f y=%4.3f z=%4.3f w=%4.3f",object_.abs_pose.orientation.x,object_.abs_pose.orientation.y,object_.abs_pose.orientation.z,object_.abs_pose.orientation.w,tmp_Roll,tmp_Pitch,tmp_Yaw);
	ROS_INFO("                                  (roll=%4.3f pitch=%4.3f yaw=%4.3f)",tmp_Roll,tmp_Pitch,tmp_Yaw);
	ROS_INFO("b_object_com_ = [%4.3f %4.3f %4.3f]",b_object_com_[0],b_object_com_[1],b_object_com_[2]);
	ROS_INFO("o_object_com_ = [%4.3f %4.3f %4.3f]",o_object_com_[0],o_object_com_[1],o_object_com_[2]);
	ROS_INFO("object_mass: %fkg",object_mass_);

	tmpPoseArray.clear();
	tmpPoseArray=LWRTCP_object_grip_pose;
	ROS_INFO("LWRTCP_object_grip_pose: PoseNr | pos (XYZ) | ori (XYZW)");
	for(uint8_t ii=0; ii<tmpPoseArray.size(); ii++)
	{
		ROS_INFO(" [%d] | [%4.3f %4.3f %4.3f] | [%4.3f %4.3f %4.3f %4.3f]",ii,tmpPoseArray[ii].position.x,
				tmpPoseArray[ii].position.y,tmpPoseArray[ii].position.z,tmpPoseArray[ii].orientation.x,tmpPoseArray[ii].orientation.y,
				tmpPoseArray[ii].orientation.z,tmpPoseArray[ii].orientation.w);
	}
	tmpPoseArray.clear();
	tmpPoseArray=LWRTCP_object_safe_pose;
	ROS_INFO("LWRTCP_object_safe_pose: PoseNr | pos (XYZ) | ori (XYZW)");
	for(uint8_t ii=0; ii<tmpPoseArray.size(); ii++)
	{
		ROS_INFO(" [%d] | [%4.3f %4.3f %4.3f] | [%4.3f %4.3f %4.3f %4.3f]",ii,tmpPoseArray[ii].position.x,
				tmpPoseArray[ii].position.y,tmpPoseArray[ii].position.z,tmpPoseArray[ii].orientation.x,tmpPoseArray[ii].orientation.y,
				tmpPoseArray[ii].orientation.z,tmpPoseArray[ii].orientation.w);
	}
	tmpPoseArray.clear();
	tmpPoseArray=LWRTCP_object_vision_pose;
	ROS_INFO("LWRTCP_object_vision_pose: PoseNr | pos (XYZ) | ori (XYZW) | skip");
	for(uint8_t ii=0; ii<tmpPoseArray.size(); ii++)
	{
		ROS_INFO(" [%d] | [%4.3f %4.3f %4.3f] | [%4.3f %4.3f %4.3f %4.3f] | %d",ii,tmpPoseArray[ii].position.x,
				tmpPoseArray[ii].position.y,tmpPoseArray[ii].position.z,tmpPoseArray[ii].orientation.x,tmpPoseArray[ii].orientation.y,
				tmpPoseArray[ii].orientation.z,tmpPoseArray[ii].orientation.w,object_skip_vision[ii]);
	}
	tmpPoseArray.clear();
	tmpPoseArray=LWRTCP_target_place_pose;
	ROS_INFO("LWRTCP_target_place_pose: PoseNr | pos (XYZ) | ori (XYZW) | prio");
	for(uint8_t ii=0; ii<tmpPoseArray.size(); ii++)
	{
		ROS_INFO(" [%d] | [%4.3f %4.3f %4.3f] | [%4.3f %4.3f %4.3f %4.3f]",ii,tmpPoseArray[ii].position.x,
				tmpPoseArray[ii].position.y,tmpPoseArray[ii].position.z,tmpPoseArray[ii].orientation.x,tmpPoseArray[ii].orientation.y,
				tmpPoseArray[ii].orientation.z,tmpPoseArray[ii].orientation.w);
	}
	tmpPoseArray.clear();
	tmpPoseArray=LWRTCP_target_safe_pose;
	ROS_INFO("LWRTCP_target_safe_pose: PoseNr | pos (XYZ) | ori (XYZW)");
	for(uint8_t ii=0; ii<tmpPoseArray.size(); ii++)
	{
		ROS_INFO(" [%d] | [%4.3f %4.3f %4.3f] | [%4.3f %4.3f %4.3f %4.3f]",ii,tmpPoseArray[ii].position.x,
				tmpPoseArray[ii].position.y,tmpPoseArray[ii].position.z,tmpPoseArray[ii].orientation.x,tmpPoseArray[ii].orientation.y,
				tmpPoseArray[ii].orientation.z,tmpPoseArray[ii].orientation.w);
	}
	tmpPoseArray.clear();
	tmpPoseArray=LWRTCP_target_vision_pose;
	ROS_INFO("LWRTCP_target_vision_pose: PoseNr | pos (XYZ) | ori (XYZW) | skip");
	for(uint8_t ii=0; ii<tmpPoseArray.size(); ii++)
	{
		ROS_INFO(" [%d] | [%4.3f %4.3f %4.3f] | [%4.3f %4.3f %4.3f %4.3f] | %d",ii,tmpPoseArray[ii].position.x,
				tmpPoseArray[ii].position.y,tmpPoseArray[ii].position.z,tmpPoseArray[ii].orientation.x,tmpPoseArray[ii].orientation.y,
				tmpPoseArray[ii].orientation.z,tmpPoseArray[ii].orientation.w,target_skip_vision[ii]);
	}
	ROS_INFO("grip_pose_type: PoseNr | type");
	for(uint8_t ii=0; ii<grip_pose_type.size(); ii++)
	{
		switch(grip_pose_type[ii])
		{
			case GRIP_POSE_CUBE_X_UP:
				ROS_INFO(" [%d] | GRIP_POSE_CUBE_X_UP",ii);
				break;
			case GRIP_POSE_CUBE_X_UP_45byY:
				ROS_INFO(" [%d] | GRIP_POSE_CUBE_X_UP_45byY",ii);
				break;
			case GRIP_POSE_CUBE_X_UP_45byZ:
				ROS_INFO(" [%d] | GRIP_POSE_CUBE_X_UP_45byZ",ii);
				break;
			case GRIP_POSE_CUBE_Y_UP:
				ROS_INFO(" [%d] | GRIP_POSE_CUBE_Y_UP",ii);
				break;
			case GRIP_POSE_CUBE_Y_UP_45byX:
				ROS_INFO(" [%d] | GRIP_POSE_CUBE_Y_UP_45byX",ii);
				break;
			case GRIP_POSE_CUBE_Y_UP_45byZ:
				ROS_INFO(" [%d] | GRIP_POSE_CUBE_Y_UP_45byZ",ii);
				break;
			case GRIP_POSE_CUBE_Z_UP:
				ROS_INFO(" [%d] | GRIP_POSE_CUBE_Z_UP",ii);
				break;
			case GRIP_POSE_CUBE_Z_UP_45byX:
				ROS_INFO(" [%d] | GRIP_POSE_CUBE_Z_UP_45byX",ii);
				break;
			case GRIP_POSE_CUBE_Z_UP_45byY:
				ROS_INFO(" [%d] | GRIP_POSE_CUBE_Z_UP_45byY",ii);
				break;
			case GRIP_POSE_CUBE_TASK6:
				ROS_INFO(" [%d] | GRIP_POSE_CUBE_TASK6",ii);
				break;
			case GRIP_POSE_CYLINDER_VERTICAL:
				ROS_INFO(" [%d] | GRIP_POSE_CYLINDER_VERTICAL",ii);
				break;
			case GRIP_POSE_CYLINDER_VERTICAL_45:
				ROS_INFO(" [%d] | GRIP_POSE_CYLINDER_VERTICAL_45",ii);
				break;
			case GRIP_POSE_HANDLE_CYLINDER_ZEQX_YPOSZ:
				ROS_INFO(" [%d] | GRIP_POSE_HANDLE_CYLINDER_ZEQX_YPOSZ",ii);
				break;
			case GRIP_POSE_HANDLE_CYLINDER_ZEQX_YNEGZ:
				ROS_INFO(" [%d] | GRIP_POSE_HANDLE_CYLINDER_ZEQX_YNEGZ",ii);
				break;
			case GRIP_POSE_HANDLE_CYLINDER_ZEQY_YPOSZ:
				ROS_INFO(" [%d] | GRIP_POSE_HANDLE_CYLINDER_ZEQY_YPOSZ",ii);
				break;
			case GRIP_POSE_HANDLE_CYLINDER_ZEQY_YNEGZ:
				ROS_INFO(" [%d] | GRIP_POSE_HANDLE_CYLINDER_ZEQY_YNEGZ",ii);
				break;
			case GRIP_POSE_HANDLE_BOX1_ZEQX_YPOSZ:
				ROS_INFO(" [%d] | GRIP_POSE_HANDLE_BOX1_ZEQX_YPOSZ",ii);
				break;
			case GRIP_POSE_HANDLE_BOX1_ZEQX_YNEGZ:
				ROS_INFO(" [%d] | GRIP_POSE_HANDLE_BOX1_ZEQX_YNEGZ",ii);
				break;
			case GRIP_POSE_HANDLE_BOX1_ZEQY_YPOSZ:
				ROS_INFO(" [%d] | GRIP_POSE_HANDLE_BOX1_ZEQY_YPOSZ",ii);
				break;
			case GRIP_POSE_HANDLE_BOX1_ZEQY_YNEGZ:
				ROS_INFO(" [%d] | GRIP_POSE_HANDLE_BOX1_ZEQY_YNEGZ",ii);
				break;
			case GRIP_POSE_HANDLE_BOX2_ZEQX_YPOSZ:
				ROS_INFO(" [%d] | GRIP_POSE_HANDLE_BOX2_ZEQX_YPOSZ",ii);
				break;
			case GRIP_POSE_HANDLE_BOX2_ZEQX_YNEGZ:
				ROS_INFO(" [%d] | GRIP_POSE_HANDLE_BOX2_ZEQX_YNEGZ",ii);
				break;
			case GRIP_POSE_HANDLE_BOX2_ZEQY_YPOSZ:
				ROS_INFO(" [%d] | GRIP_POSE_HANDLE_BOX2_ZEQY_YPOSZ",ii);
				break;
			case GRIP_POSE_HANDLE_BOX2_ZEQY_YNEGZ:
				ROS_INFO(" [%d] | GRIP_POSE_HANDLE_BOX2_ZEQY_YNEGZ",ii);
				break;
			default:
				ROS_INFO("grip_pose_type_: UNKNOWN");
				break;
		}
	}
	ROS_INFO("place_pose_type: PoseNr | type");
	for(uint8_t ii=0; ii<place_pose_type.size(); ii++)
	{
		switch(place_pose_type[ii])
		{
			case PLACE_POSE_CUBE_X_UP:
				ROS_INFO(" [%d] | PLACE_POSE_CUBE_X_UP",ii);
				break;
			case PLACE_POSE_CUBE_X_UP_45byY:
				ROS_INFO(" [%d] | PLACE_POSE_CUBE_X_UP_45byY",ii);
				break;
			case PLACE_POSE_CUBE_X_UP_45byZ:
				ROS_INFO(" [%d] | PLACE_POSE_CUBE_X_UP_45byZ",ii);
				break;
			case PLACE_POSE_CUBE_Y_UP:
				ROS_INFO(" [%d] | PLACE_POSE_CUBE_Y_UP",ii);
				break;
			case PLACE_POSE_CUBE_Y_UP_45byX:
				ROS_INFO(" [%d] | PLACE_POSE_CUBE_Y_UP_45byX",ii);
				break;
			case PLACE_POSE_CUBE_Y_UP_45byZ:
				ROS_INFO(" [%d] | PLACE_POSE_CUBE_Y_UP_45byZ",ii);
				break;
			case PLACE_POSE_CUBE_Z_UP:
				ROS_INFO(" [%d] | PLACE_POSE_CUBE_Z_UP",ii);
				break;
			case PLACE_POSE_CUBE_Z_UP_45byX:
				ROS_INFO(" [%d] | PLACE_POSE_CUBE_Z_UP_45byX",ii);
				break;
			case PLACE_POSE_CUBE_Z_UP_45byY:
				ROS_INFO(" [%d] | PLACE_POSE_CUBE_Z_UP_45byY",ii);
				break;
			case PLACE_POSE_CUBE_TASK6:
				ROS_INFO(" [%d] | PLACE_POSE_CUBE_TASK6",ii);
				break;
			case PLACE_POSE_CYLINDER_VERTICAL:
				ROS_INFO(" [%d] | PLACE_POSE_CYLINDER_VERTICAL",ii);
				break;
			case PLACE_POSE_CYLINDER_VERTICAL_45:
				ROS_INFO(" [%d] | PLACE_POSE_CYLINDER_VERTICAL_45",ii);
				break;
			case PLACE_POSE_HANDLE_CYLINDER_YPOSZ_VERTICAL:
				ROS_INFO(" [%d] | PLACE_POSE_HANDLE_CYLINDER_YPOSZ_VERTICAL",ii);
				break;
			case PLACE_POSE_HANDLE_CYLINDER_YNEGZ_VERTICAL:
				ROS_INFO(" [%d] | PLACE_POSE_HANDLE_CYLINDER_YNEGZ_VERTICAL",ii);
				break;
			case PLACE_POSE_HANDLE_BOX1_YPOSZ_VERTICAL:
				ROS_INFO(" [%d] | PLACE_POSE_HANDLE_BOX1_YPOSZ_VERTICAL",ii);
				break;
			case PLACE_POSE_HANDLE_BOX1_YNEGZ_VERTICAL:
				ROS_INFO(" [%d] | PLACE_POSE_HANDLE_BOX1_YNEGZ_VERTICAL",ii);
				break;
			case PLACE_POSE_HANDLE_BOX2_YPOSZ_VERTICAL:
				ROS_INFO(" [%d] | PLACE_POSE_HANDLE_BOX2_YPOSZ_VERTICAL",ii);
				break;
			case PLACE_POSE_HANDLE_BOX2_YNEGZ_VERTICAL:
				ROS_INFO(" [%d] | PLACE_POSE_HANDLE_BOX2_YNEGZ_VERTICAL",ii);
				break;
			default:
				ROS_INFO("place_pose_type_: UNKNOWN");
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
	ROS_INFO("object_grip_r_tcp_com: PoseNr | vector: (XYZ)");
	for(uint8_t ii=0; ii<tmpVector3Array.size(); ii++)
	{
		ROS_INFO(" [%d] | %f %f %f",ii,tmpVector3Array[ii].x,tmpVector3Array[ii].y,
				tmpVector3Array[ii].z);
	}
	tmpVector3Array.clear();
	tmpVector3Array=object_grip_r_gp_com;
	ROS_INFO("object_grip_r_gp_com: PoseNr | vector: (XYZ)");
	for(uint8_t ii=0; ii<tmpVector3Array.size(); ii++)
	{
		ROS_INFO("- [%d] | %f %f %f",ii,tmpVector3Array[ii].x,tmpVector3Array[ii].y,
				tmpVector3Array[ii].z);
	}
	tmpVector3Array.clear();
	tmpVector3Array=object_grip_r_gp_obj;
	ROS_INFO("object_grip_r_gp_obj: PoseNr | vector: (XYZ)");
	for(uint8_t ii=0; ii<tmpVector3Array.size(); ii++)
	{
		ROS_INFO(" [%d] | %f %f %f",ii,tmpVector3Array[ii].x,tmpVector3Array[ii].y,
				tmpVector3Array[ii].z);
	}
	ROS_INFO("===============================================");
	ROS_INFO("------------END GRASPING RESULTS---------------");
	ROS_INFO("===============================================");
}

void GraspPose2::send_poses_to_tf_broadcaster()
{
	ROS_INFO("send_poses_to_tf_broadcaster() called");
	tf::TransformBroadcaster br;

	tf::Vector3 tmp_Origin;
	tf::Quaternion tmp_Rotation;
	tf::Transform tmp_transform;
	tf::StampedTransform tmp_stampedtransform;
	std::stringstream posename;

	br.sendTransform(tf::StampedTransform(transform_object, ros::Time::now(), ORIGIN, "object_"));

	for (uint16_t ii=0; ii<LWRTCP_object_grip_pose.size(); ii++)
	{
		//send grip pose
		tmp_Origin.setX(LWRTCP_object_grip_pose[ii].position.x);
		tmp_Origin.setY(LWRTCP_object_grip_pose[ii].position.y);
		tmp_Origin.setZ(LWRTCP_object_grip_pose[ii].position.z);
		tmp_Rotation.setX(LWRTCP_object_grip_pose[ii].orientation.x);
		tmp_Rotation.setY(LWRTCP_object_grip_pose[ii].orientation.y);
		tmp_Rotation.setZ(LWRTCP_object_grip_pose[ii].orientation.z);
		tmp_Rotation.setW(LWRTCP_object_grip_pose[ii].orientation.w);

		tmp_transform.setOrigin(tmp_Origin);
		tmp_transform.setRotation(tmp_Rotation);
		posename.str("");
		posename << "object_grip_" << ii;
		tmp_stampedtransform=tf::StampedTransform(tmp_transform,ros::Time::now(),ORIGIN,posename.str().c_str());
		br.sendTransform(tmp_stampedtransform);
	}
	for (uint16_t ii=0; ii<LWRTCP_object_safe_pose.size(); ii++)
	{
		//send safe pose
		tmp_Origin.setX(LWRTCP_object_safe_pose[ii].position.x);
		tmp_Origin.setY(LWRTCP_object_safe_pose[ii].position.y);
		tmp_Origin.setZ(LWRTCP_object_safe_pose[ii].position.z);
		tmp_Rotation.setX(LWRTCP_object_safe_pose[ii].orientation.x);
		tmp_Rotation.setY(LWRTCP_object_safe_pose[ii].orientation.y);
		tmp_Rotation.setZ(LWRTCP_object_safe_pose[ii].orientation.z);
		tmp_Rotation.setW(LWRTCP_object_safe_pose[ii].orientation.w);

		tmp_transform.setOrigin(tmp_Origin);
		tmp_transform.setRotation(tmp_Rotation);
		posename.str("");
		posename << "object_safe_" << ii;
		tmp_stampedtransform=tf::StampedTransform(tmp_transform,ros::Time::now(),ORIGIN,posename.str().c_str());
		br.sendTransform(tmp_stampedtransform);
	}
	for (uint16_t ii=0; ii<LWRTCP_object_vision_pose.size(); ii++)
		{
		//send vision pose
		tmp_Origin.setX(LWRTCP_object_vision_pose[ii].position.x);
		tmp_Origin.setY(LWRTCP_object_vision_pose[ii].position.y);
		tmp_Origin.setZ(LWRTCP_object_vision_pose[ii].position.z);
		tmp_Rotation.setX(LWRTCP_object_vision_pose[ii].orientation.x);
		tmp_Rotation.setY(LWRTCP_object_vision_pose[ii].orientation.y);
		tmp_Rotation.setZ(LWRTCP_object_vision_pose[ii].orientation.z);
		tmp_Rotation.setW(LWRTCP_object_vision_pose[ii].orientation.w);

		tmp_transform.setOrigin(tmp_Origin);
		tmp_transform.setRotation(tmp_Rotation);
		posename.str("");
		posename << "object_vision_" << ii;
		tmp_stampedtransform=tf::StampedTransform(tmp_transform,ros::Time::now(),ORIGIN,posename.str().c_str());
		br.sendTransform(tmp_stampedtransform);
	}

	for (uint16_t ii=0; ii<LWRTCP_target_place_pose.size(); ii++)
	{
		//send grip pose
		tmp_Origin.setX(LWRTCP_target_place_pose[ii].position.x);
		tmp_Origin.setY(LWRTCP_target_place_pose[ii].position.y);
		tmp_Origin.setZ(LWRTCP_target_place_pose[ii].position.z);
		tmp_Rotation.setX(LWRTCP_target_place_pose[ii].orientation.x);
		tmp_Rotation.setY(LWRTCP_target_place_pose[ii].orientation.y);
		tmp_Rotation.setZ(LWRTCP_target_place_pose[ii].orientation.z);
		tmp_Rotation.setW(LWRTCP_target_place_pose[ii].orientation.w);

		tmp_transform.setOrigin(tmp_Origin);
		tmp_transform.setRotation(tmp_Rotation);
		posename.str("");
		posename << "target_place_" << ii;
		tmp_stampedtransform=tf::StampedTransform(tmp_transform,ros::Time::now(),ORIGIN,posename.str().c_str());
		br.sendTransform(tmp_stampedtransform);
	}
	for (uint16_t ii=0; ii<LWRTCP_target_safe_pose.size(); ii++)
	{
		//send safe pose
		tmp_Origin.setX(LWRTCP_target_safe_pose[ii].position.x);
		tmp_Origin.setY(LWRTCP_target_safe_pose[ii].position.y);
		tmp_Origin.setZ(LWRTCP_target_safe_pose[ii].position.z);
		tmp_Rotation.setX(LWRTCP_target_safe_pose[ii].orientation.x);
		tmp_Rotation.setY(LWRTCP_target_safe_pose[ii].orientation.y);
		tmp_Rotation.setZ(LWRTCP_target_safe_pose[ii].orientation.z);
		tmp_Rotation.setW(LWRTCP_target_safe_pose[ii].orientation.w);

		tmp_transform.setOrigin(tmp_Origin);
		tmp_transform.setRotation(tmp_Rotation);
		posename.str("");
		posename << "target_safe_" << ii;
		tmp_stampedtransform=tf::StampedTransform(tmp_transform,ros::Time::now(),ORIGIN,posename.str().c_str());
		br.sendTransform(tmp_stampedtransform);
	}
	for (uint16_t ii=0; ii<LWRTCP_target_vision_pose.size(); ii++)
	{
		//send vision pose
		tmp_Origin.setX(LWRTCP_target_vision_pose[ii].position.x);
		tmp_Origin.setY(LWRTCP_target_vision_pose[ii].position.y);
		tmp_Origin.setZ(LWRTCP_target_vision_pose[ii].position.z);
		tmp_Rotation.setX(LWRTCP_target_vision_pose[ii].orientation.x);
		tmp_Rotation.setY(LWRTCP_target_vision_pose[ii].orientation.y);
		tmp_Rotation.setZ(LWRTCP_target_vision_pose[ii].orientation.z);
		tmp_Rotation.setW(LWRTCP_target_vision_pose[ii].orientation.w);

		tmp_transform.setOrigin(tmp_Origin);
		tmp_transform.setRotation(tmp_Rotation);
		posename.str("");
		posename << "target_vision_" << ii;
		tmp_stampedtransform=tf::StampedTransform(tmp_transform,ros::Time::now(),ORIGIN,posename.str().c_str());
		br.sendTransform(tmp_stampedtransform);
	}
}

tf::Matrix3x3 GraspPose2::get_rotationmatrixfromaxis(tf::Vector3 axis, double angle_rad)
{
	tf::Vector3 normalized_axis;
	tf::Matrix3x3 tmpMat;
	double n1, n2, n3;
	double xx, xy, xz, yx, yy, yz, zx, zy, zz;

	//set identity as default
	xx=1; xy=0;	xz=0;
	yx=0; yy=1; yz=0;
	zx=0; zy=0; zz=1;

	if (axis.length()>0)
	{
		normalized_axis = 1.0/(axis.length())*axis;
		n1=normalized_axis.getX();
		n2=normalized_axis.getY();
		n3=normalized_axis.getZ();
		xx=n1*n1*(1-cos(angle_rad))+cos(angle_rad);
		xy=n1*n2*(1-cos(angle_rad))-n3*sin(angle_rad);
		xz=n1*n3*(1-cos(angle_rad))+n2*sin(angle_rad);
		yx=n2*n1*(1-cos(angle_rad))+n3*sin(angle_rad);
		yy=n2*n2*(1-cos(angle_rad))+cos(angle_rad);
		yz=n2*n3*(1-cos(angle_rad))-n1*sin(angle_rad);
		zx=n3*n1*(1-cos(angle_rad))-n2*sin(angle_rad);
		zy=n3*n2*(1-cos(angle_rad))+n1*sin(angle_rad);
		zz=n3*n3*(1-cos(angle_rad))+cos(angle_rad);
	}

	tmpMat.setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
	return tmpMat;
}
