#include <euroc_input.hpp>
#include <tf_rot.hpp>
#include <utils.hpp>
#include <iostream>
#include <string>
#include <sstream>

#undef DBG_OUT


EurocInput::EurocInput():
	task_nr_(1),
	nr_objects_(0),
	active_object_(-1),
	nr_zones_(0),
	active_zone_(-1),
	time_limit_(3000),
	nr_sensors(0)
{

}
EurocInput::~EurocInput()
{

}

int EurocInput::parse_yaml_file(std::string task_yaml_description, const uint16_t task_nr)
{
	//save active task number
	task_nr_=task_nr;
	// Parse the explanation/description of the task from the yaml string
	std::stringstream yaml_stream(task_yaml_description);


	YAML::Parser parser(yaml_stream);
	YAML::Node task_description_node;
	if(!parser.GetNextDocument(task_description_node))
	{
	  msg_error("failed to get next document!");
	  return -1;
	}


	task_description_node["description"] >> description_;
	task_description_node["log_filename"] >> log_filename_;
	task_description_node["time_limit"] >> time_limit_;

	const YAML::Node* mast_of_cam = task_description_node.FindValue("mast_of_cam");
	if(!mast_of_cam)
	{
		ROS_ERROR("EurocInput: mast_of_cam not found in task_description_node");
		return -1;
	}
	const YAML::Node* base_pose = mast_of_cam->FindValue("base_pose");
	double rpy[3];
	double a_tf[9];
	double q_tf[4];
	tf::Quaternion q_tf_ros;

	try{
		(*base_pose)[0] >> base_pose_.position.x;
		(*base_pose)[1] >> base_pose_.position.y;
		(*base_pose)[2] >> base_pose_.position.z;
		(*base_pose)[3] >> rpy[0];
		(*base_pose)[4] >> rpy[1];
		(*base_pose)[5] >> rpy[2];
		q_tf_ros.setRPY(rpy[0],rpy[1],rpy[2]);
		base_pose_.orientation.w=q_tf_ros.getW();
		base_pose_.orientation.x=q_tf_ros.getX();
		base_pose_.orientation.y=q_tf_ros.getY();
		base_pose_.orientation.z=q_tf_ros.getZ();
//		rpy2quat(rpy,a_tf,q_tf);
//		base_pose_.orientation.w=q_tf[0];
//		base_pose_.orientation.x=q_tf[1];
//		base_pose_.orientation.y=q_tf[2];
//		base_pose_.orientation.z=q_tf[3];
	}
	catch(YAML::Exception e){
		ROS_ERROR("EurocInput: YAML Error in base_pose");
		return -1;
	}

	const YAML::Node* pan_tilt_base = mast_of_cam->FindValue("pan_tilt_base");
	try{
		(*pan_tilt_base)[0] >> pan_tilt_base_.position.x;
		(*pan_tilt_base)[1] >> pan_tilt_base_.position.y;
		(*pan_tilt_base)[2] >> pan_tilt_base_.position.z;
		(*pan_tilt_base)[3] >> rpy[0];
		(*pan_tilt_base)[4] >> rpy[1];
		(*pan_tilt_base)[5] >> rpy[2];
		q_tf_ros.setRPY(rpy[0],rpy[1],rpy[2]);
		pan_tilt_base_.orientation.w=q_tf_ros.getW();
		pan_tilt_base_.orientation.x=q_tf_ros.getX();
		pan_tilt_base_.orientation.y=q_tf_ros.getY();
		pan_tilt_base_.orientation.z=q_tf_ros.getZ();
//		rpy2quat(rpy,a_tf,q_tf);
//		pan_tilt_base_.orientation.w=q_tf[0];
//		pan_tilt_base_.orientation.x=q_tf[1];
//		pan_tilt_base_.orientation.y=q_tf[2];
//		pan_tilt_base_.orientation.z=q_tf[3];
	}
	catch(YAML::Exception e){
		ROS_ERROR("EurocInput: YAML Error in pan_filt_base");
		return -1;
	}

	const YAML::Node* speed_limit = mast_of_cam->FindValue("speed_limit");
	try{
		(*speed_limit)[0] >> speed_limit_[0];
		(*speed_limit)[1] >> speed_limit_[1];
	}
	catch(YAML::Exception e){
		ROS_ERROR("EurocInput: YAML Error in speed_limit");
		return -1;
	}

	//######################################################################################
	//######################################################################################
	//Objects:
	const YAML::Node *objects = task_description_node.FindValue("objects");
	if(!objects){
	    ROS_ERROR("EurocInput: Objects not found in task description");
	    return -1;
	  }

	nr_objects_ = objects->size();
	obj_finished_.resize(nr_objects_,0);
	unsigned int ii = 0;
	am_msgs::Object tmp_obj;
	//objects_.resize(nr_objects);
	for(YAML::Iterator it = objects->begin(); it != objects->end(); ++it)
	{
	    const YAML::Node* obj;
	    try{
	    obj = &(it.second());
	    }catch(YAML::Exception e){
	      ROS_ERROR("Obj error");
	      return -1;
	    }

	    try{
	    	(it.first()) >> tmp_obj.name;
	    }catch(YAML::Exception e){
	    	ROS_ERROR("EurocInput: Obj error");
	    	return -1;
	    }

	    const YAML::Node* col = obj->FindValue("color");
	    if(!col){
	    	ROS_ERROR("EurocInput: Color not found in object");
		    return -1;
	    }
	    try {
	    	(*col) >> tmp_obj.color;
	    }catch(YAML::Exception e){
	    	ROS_ERROR("EurocInput: Obj error");
			return -1;
		}

	    if(task_nr_ != 5)
		{
			const YAML::Node* des = obj->FindValue("description");
			if(!des){
				ROS_ERROR("EurocInput: description not found in object");
				return -1;
			}
			try {
				*des >> tmp_obj.description;
			}catch(YAML::Exception e){
				ROS_ERROR("EurocInput: Obj error");
				return -1;
			}
	    }

	    if((task_nr_ != 5) && (task_nr_ != 6))
	    {
	    	try {
                const YAML::Node* sur = obj->FindValue("surface_material");

                if(!sur){
                  ROS_ERROR("EurocInput: surface material not found in object");
                  return -1;
                }
                else
                {
                  try {
                    *sur >> tmp_obj.surface_material;
                  }catch(YAML::Exception e)
                  {
                    ROS_ERROR("EurocInput: Obj error");
                    return -1;
                  }
                }
			}catch(YAML::Exception e) {
			  ROS_ERROR("surface material not found in shape");
			  return -1;
			}
	    }


	    const YAML::Node* shapes = obj->FindValue("shape");
	    if(!shapes){
	      ROS_ERROR("Shape not found in object");
	      return -1;
	    }
		tmp_obj.nr_shapes = shapes->size();
		unsigned int jj = 0;
		tmp_obj.shape.resize(tmp_obj.nr_shapes);
		for (YAML::Iterator it2 = shapes->begin(); it2 != shapes->end();++it2)
		{
			try{
				const YAML::Node* type = (*it2).FindValue("type");
				*type >> tmp_obj.shape[jj].type;
			}catch(YAML::Exception e) {
				ROS_ERROR("Type not found in shape");
				return -1;
			}

			if(tmp_obj.shape[jj].type == "cylinder")
			{
				try{
					const YAML::Node* leng = (*it2).FindValue("length");
					*leng >> tmp_obj.shape[jj].length;
					const YAML::Node* rad = (*it2).FindValue("radius");
					*rad >> tmp_obj.shape[jj].radius;
				} catch(YAML::Exception e) {
					ROS_ERROR("dimensions not found in shape");
					return -1;
				}
				tmp_obj.shape[jj].size.resize(3,0);
			}
			else if(tmp_obj.shape[jj].type == "box")
			{
				tmp_obj.shape[jj].size.resize(3);
				try{
					const YAML::Node* size = (*it2).FindValue("size");
					(*size)[0] >> tmp_obj.shape[jj].size[0];
					(*size)[1] >> tmp_obj.shape[jj].size[1];
					(*size)[2] >> tmp_obj.shape[jj].size[2];
				}catch(YAML::Exception e) {
					ROS_ERROR("dimensions not found in shape");
					return -1;
				}
				tmp_obj.shape[jj].length=0;
				tmp_obj.shape[jj].radius=0;
			}
			else
			{
				ROS_ERROR("Unknown type : %s",tmp_obj.shape[jj].type.c_str());
				return -1;
			}


			try{
				const YAML::Node* sh_pose = (*it2).FindValue("pose");
				(*sh_pose)[0] >> tmp_obj.shape[jj].pose.position.x;
				(*sh_pose)[1] >> tmp_obj.shape[jj].pose.position.y;
				(*sh_pose)[2] >> tmp_obj.shape[jj].pose.position.z;
				(*sh_pose)[3] >> rpy[0];
				(*sh_pose)[4] >> rpy[1];
				(*sh_pose)[5] >> rpy[2];
				q_tf_ros.setRPY(rpy[0],rpy[1],rpy[2]);
				tmp_obj.shape[jj].pose.orientation.w=q_tf_ros.getW();
				tmp_obj.shape[jj].pose.orientation.x=q_tf_ros.getX();
				tmp_obj.shape[jj].pose.orientation.y=q_tf_ros.getY();
				tmp_obj.shape[jj].pose.orientation.z=q_tf_ros.getZ();
//				rpy2quat(rpy,a_tf,q_tf);
//				tmp_obj.shape[jj].pose.orientation.w=q_tf[0];
//				tmp_obj.shape[jj].pose.orientation.x=q_tf[1];
//				tmp_obj.shape[jj].pose.orientation.y=q_tf[2];
//				tmp_obj.shape[jj].pose.orientation.z=q_tf[3];
			}catch(YAML::Exception e) {
				ROS_ERROR("Pose not found in shape");
				return -1;
			}


			try{
				const YAML::Node* dens = (*it2).FindValue("density");
				(*dens) >> tmp_obj.shape[jj].density;
			}catch(YAML::Exception e) {
				msg_warn("density not found in shape");

				tmp_obj.shape[jj].density=7850;
				//return -1;
			}

			jj++;

		} /* for (YAML::Iterator it2 = shapes->begin(); it2 != shapes->end();++it2) */


		objects_.push_back(tmp_obj);
	    ii++;
	  } /*for(YAML::Iterator it = objects->begin(); it != objects->end(); ++it) */

#ifdef DBG_OUT
	  for(unsigned kk=0;kk<nr_objects_;kk++)
	  {
		  ROS_INFO("======================");
		  ROS_INFO("Object %d:",kk);
		  print_object(&objects_[kk]);
	  }
	  ROS_INFO("");
#endif

	//######################################################################################
	//######################################################################################
	//Target zones:
    if(task_nr_ != 5)
    {
		const YAML::Node *target_zones = task_description_node.FindValue("target_zones");
		if(!target_zones){
		ROS_ERROR("Target zones not found in task description");
		return -1;
		}
		am_msgs::TargetZone tmp_zone;
		ii=0;
		nr_zones_ = target_zones->size();
		for(YAML::Iterator it = target_zones->begin(); it != target_zones->end(); ++it)
		{

			const YAML::Node* zone;
			try{
			  zone = &(it.second());
			}catch(YAML::Exception e){
			  ROS_ERROR("Zone error");
			  return -1;
			}
			const YAML::Node* target_position = zone->FindValue("target_position");
			if(!target_position){
			  ROS_ERROR("Target position not found in target zone");
			  return -1;
			}
			try{
			  (*target_position)[0] >> tmp_zone.position.x;
			  (*target_position)[1] >> tmp_zone.position.y;
			  (*target_position)[2] >> tmp_zone.position.z;
			}catch(YAML::Exception e){
			  ROS_ERROR("YAML_Exception while parsing target zone position: %s", e.msg.c_str());
			  return -1;
			}

			const YAML::Node* dist = zone->FindValue("max_distance");
			if(!dist){
				ROS_ERROR("max_distance not found in target_zone");
				return -1;
			}
			try{
				(*dist) >> tmp_zone.max_distance;
			}catch(YAML::Exception e){
			  ROS_ERROR("YAML_Exception while parsing target zone max distance: %s", e.msg.c_str());
			  return -1;
			}

			const YAML::Node* exp_obj = zone->FindValue("expected_object");
			if(!exp_obj){
				ROS_ERROR("expected_object not found in target_zone");
				return -1;
			}
			try{
				(*exp_obj) >> tmp_zone.expected_object;
			}catch(YAML::Exception e){
			  ROS_ERROR("YAML_Exception while parsing target zone expected object: %s", e.msg.c_str());
			  return -1;
			}

			target_zones_.push_back(tmp_zone);
			ii++;

		}

#ifdef DBG_OUT
		for(unsigned kk=0;kk<nr_zones_;kk++)
		{
		  ROS_INFO("======================");
		  ROS_INFO("Target zone %d:",kk);
		  ROS_INFO("Expected object: %s",target_zones_[kk].expected_object.c_str());
		  ROS_INFO("Target position: [%f %f %f]",target_zones_[kk].position.x,
				  target_zones_[kk].position.y,target_zones_[kk].position.z);
		  ROS_INFO("Max distance: %f",target_zones_[kk].max_distance);
		}
		ROS_INFO("");
#endif
    }

	//######################################################################################
	//######################################################################################
	// Robot:
	const YAML::Node* robot = task_description_node.FindValue("robot");
	if (!robot)
	{
		ROS_ERROR("EurocInput: robot not found in task_description_node");
		return -1;
	}
	const YAML::Node* rob_pose = robot->FindValue("pose");
	try {
		(*rob_pose)[0] >> robot_.pose.position.x;
		(*rob_pose)[1] >> robot_.pose.position.y;
		(*rob_pose)[2] >> robot_.pose.position.z;
		(*rob_pose)[3] >> rpy[0];
		(*rob_pose)[4] >> rpy[1];
		(*rob_pose)[5] >> rpy[2];
		q_tf_ros.setRPY(rpy[0],rpy[1],rpy[2]);
		robot_.pose.orientation.w=q_tf_ros.getW();
		robot_.pose.orientation.x=q_tf_ros.getX();
		robot_.pose.orientation.y=q_tf_ros.getY();
		robot_.pose.orientation.z=q_tf_ros.getZ();
//		rpy2quat(rpy, a_tf, q_tf);
//		robot_.pose.orientation.w = q_tf[0];
//		robot_.pose.orientation.x = q_tf[1];
//		robot_.pose.orientation.y = q_tf[2];
//		robot_.pose.orientation.z = q_tf[3];
	} catch (YAML::Exception e) {
		ROS_ERROR("EurocInput: YAML Error in robot pose");
		return -1;
	}

	const YAML::Node* rob_lim = robot->FindValue("speed_limit");
	try {
		robot_.speed_limit.resize(7);
		for(unsigned ii=0;ii<7;ii++)
		{
			(*rob_lim)[ii] >> robot_.speed_limit[ii];
		}
	} catch (YAML::Exception e) {
		ROS_ERROR("EurocInput: YAML Error in robot speed limit");
		return -1;
	}

	const YAML::Node* grip_lim = robot->FindValue("gripper_speed_limit");
	try {
			(*grip_lim) >> robot_.gripper_speed_limit;
	} catch (YAML::Exception e) {
		ROS_ERROR("EurocInput: YAML Error in gripper speed limit");
		return -1;
	}

	const YAML::Node* grip_pose = robot->FindValue("gripper_pose");
	try {
		(*grip_pose)[0] >> robot_.gripper_pose.position.x;
		(*grip_pose)[1] >> robot_.gripper_pose.position.y;
		(*grip_pose)[2] >> robot_.gripper_pose.position.z;
		(*grip_pose)[3] >> rpy[0];
		(*grip_pose)[4] >> rpy[1];
		(*grip_pose)[5] >> rpy[2];
		q_tf_ros.setRPY(rpy[0],rpy[1],rpy[2]);
		robot_.gripper_pose.orientation.w=q_tf_ros.getW();
		robot_.gripper_pose.orientation.x=q_tf_ros.getX();
		robot_.gripper_pose.orientation.y=q_tf_ros.getY();
		robot_.gripper_pose.orientation.z=q_tf_ros.getZ();
//		rpy2quat(rpy, a_tf, q_tf);
//		robot_.gripper_pose.orientation.w = q_tf[0];
//		robot_.gripper_pose.orientation.x = q_tf[1];
//		robot_.gripper_pose.orientation.y = q_tf[2];
//		robot_.gripper_pose.orientation.z = q_tf[3];
	} catch (YAML::Exception e) {
		ROS_ERROR("EurocInput: YAML Error in gripper pose");
		return -1;
	}

	const YAML::Node* tcp_pose = robot->FindValue("gripper_tcp");
	try {
		(*tcp_pose)[0] >> robot_.gripper_tcp.position.x;
		(*tcp_pose)[1] >> robot_.gripper_tcp.position.y;
		(*tcp_pose)[2] >> robot_.gripper_tcp.position.z;
		(*tcp_pose)[3] >> rpy[0];
		(*tcp_pose)[4] >> rpy[1];
		(*tcp_pose)[5] >> rpy[2];
		q_tf_ros.setRPY(rpy[0],rpy[1],rpy[2]);
		robot_.gripper_tcp.orientation.w=q_tf_ros.getW();
		robot_.gripper_tcp.orientation.x=q_tf_ros.getX();
		robot_.gripper_tcp.orientation.y=q_tf_ros.getY();
		robot_.gripper_tcp.orientation.z=q_tf_ros.getZ();
//		rpy2quat(rpy, a_tf, q_tf);
//		robot_.gripper_tcp.orientation.w = q_tf[0];
//		robot_.gripper_tcp.orientation.x = q_tf[1];
//		robot_.gripper_tcp.orientation.y = q_tf[2];
//		robot_.gripper_tcp.orientation.z = q_tf[3];
	} catch (YAML::Exception e) {
		ROS_ERROR("EurocInput: YAML Error in gripper tcp pose");
		return -1;
	}

	const YAML::Node* table = task_description_node.FindValue("two_axes_table");
	if (!table)
	{
		ROS_ERROR("EurocInput: two_axes_table not found in task_description_node");
		return -1;
	}
	const YAML::Node* table_lim = table->FindValue("speed_limit");
	try {
		robot_.two_axes_speed_limit.resize(2);
		(*table_lim)[0] >> robot_.two_axes_speed_limit[0];
		(*table_lim)[1] >> robot_.two_axes_speed_limit[1];
	}
	catch (YAML::Exception e) {
		ROS_ERROR("EurocInput: YAML Error in two_axes speed limit");
		return -1;
	}

	//######################################################################################
	//######################################################################################
	//Sensors:
	const YAML::Node* sensors = task_description_node.FindValue("sensors");
	if (!sensors)
	{
		ROS_ERROR("EurocInput: sensors not found in task_description_node");
		return -1;
	}
	nr_sensors = sensors->size();
	ii = 0;
	am_msgs::Sensor tmp_sens;
	for(YAML::Iterator it = sensors->begin(); it != sensors->end(); ++it)
	{
		const YAML::Node* sens;
		try{
		  sens = &(it.second());
		}catch(YAML::Exception e){
		  ROS_ERROR("EurocInput: Sensor error");
		  return -1;
		}

		(it.first()) >> tmp_sens.name;

		const YAML::Node* upd_rate = sens->FindValue("update_rate");
		if(!upd_rate){
		  ROS_ERROR("EurocInput: Update_rate not found in sensor");
		  return -1;
		}
		try{
			(*upd_rate) >> tmp_sens.update_rate;
		}catch(YAML::Exception e){
		  ROS_ERROR("EurocInput: YAML_Exception while parsing sensor update rate: %s", e.msg.c_str());
		  return -1;
		}

		const YAML::Node* sens_pose = sens->FindValue("pose");
		if(!sens_pose)
		{
			const YAML::Node* rel = sens->FindValue("relative_pose");
			if(!rel){
				ROS_ERROR("EurocInput: rel_pose not found in sensor");
				return -1;
			}
			const YAML::Node* rel_pose = rel->FindValue("pose");
			if(!rel_pose){
				ROS_ERROR("EurocInput: pose not found in sensor");
				return -1;
			}

			try {
				(*rel_pose)[0] >> tmp_sens.pose.position.x;
				(*rel_pose)[1] >> tmp_sens.pose.position.y;
				(*rel_pose)[2] >> tmp_sens.pose.position.z;
				(*rel_pose)[3] >> rpy[0];
				(*rel_pose)[4] >> rpy[1];
				(*rel_pose)[5] >> rpy[2];
				q_tf_ros.setRPY(rpy[0],rpy[1],rpy[2]);
				tmp_sens.pose.orientation.w=q_tf_ros.getW();
				tmp_sens.pose.orientation.x=q_tf_ros.getX();
				tmp_sens.pose.orientation.y=q_tf_ros.getY();
				tmp_sens.pose.orientation.z=q_tf_ros.getZ();
//				rpy2quat(rpy, a_tf, q_tf);
//				tmp_sens.pose.orientation.w = q_tf[0];
//				tmp_sens.pose.orientation.x = q_tf[1];
//				tmp_sens.pose.orientation.y = q_tf[2];
//				tmp_sens.pose.orientation.z = q_tf[3];
			} catch (YAML::Exception e) {
				ROS_ERROR("EurocInput: YAML Error in sensor pose");
				return -1;
			}

			const YAML::Node* parent = rel->FindValue("from");
			if(!parent){
				ROS_ERROR("EurocInput: parent (from) not found in sensor");
				return -1;
			}
			try {
				*parent >> tmp_sens.parent;
			} catch (YAML::Exception e) {
				ROS_ERROR("EurocInput: YAML Error in sensor parent (from)");
				return -1;
			}
		}
		else
		{
			try {
				(*sens_pose)[0] >> tmp_sens.pose.position.x;
				(*sens_pose)[1] >> tmp_sens.pose.position.y;
				(*sens_pose)[2] >> tmp_sens.pose.position.z;
				(*sens_pose)[3] >> rpy[0];
				(*sens_pose)[4] >> rpy[1];
				(*sens_pose)[5] >> rpy[2];
				q_tf_ros.setRPY(rpy[0],rpy[1],rpy[2]);
				tmp_sens.pose.orientation.w=q_tf_ros.getW();
				tmp_sens.pose.orientation.x=q_tf_ros.getX();
				tmp_sens.pose.orientation.y=q_tf_ros.getY();
				tmp_sens.pose.orientation.z=q_tf_ros.getZ();
//				rpy2quat(rpy, a_tf, q_tf);
//				tmp_sens.pose.orientation.w = q_tf[0];
//				tmp_sens.pose.orientation.x = q_tf[1];
//				tmp_sens.pose.orientation.y = q_tf[2];
//				tmp_sens.pose.orientation.z = q_tf[3];
			} catch (YAML::Exception e) {
				ROS_ERROR("EurocInput: YAML Error in sensor pose");
				return -1;
			}
			tmp_sens.parent = "";
		}

		const YAML::Node* cam = sens->FindValue("camera");
		if(!cam) {
			ROS_ERROR("EurocInput: camera not found in sensor");
			return -1;
		}

		const YAML::Node* fov = cam->FindValue("horizontal_fov");
		if(!fov) {
			ROS_ERROR("EurocInput: horizontal_fov not found in camera");
			return -1;
		}
		try {
			(*fov) >> tmp_sens.camera.horizontal_fov;
		} catch (YAML::Exception e) {
			ROS_ERROR("EurocInput: YAML Error in camera horizontal fov");
			return -1;
		}

		const YAML::Node* image = cam->FindValue("image");
		if(!image) {
			ROS_ERROR("EurocInput: image not found in camera");
			return -1;
		}
		const YAML::Node* width = image->FindValue("width");
		if(!width){
			ROS_ERROR("EurocInput: width not found in image");
			return -1;
		}
		unsigned int tmp;
		try{
			(*width) >> tmp_sens.camera.image_width;
		} catch (YAML::Exception e) {
			ROS_ERROR("EurocInput: YAML Error in camera image width");
			return -1;
		}

		const YAML::Node* height = image->FindValue("height");
		if(!height){
			ROS_ERROR("EurocInput: height not found in image");
			return -1;
		}
		try{
			(*height) >> tmp_sens.camera.image_height;
		} catch (YAML::Exception e) {
			ROS_ERROR("EurocInput: YAML Error in camera image height");
			return -1;
		}

		sensors_.push_back(tmp_sens);
	}

	//######################################################################################
	//######################################################################################
	//Puzzle:
	geometry_msgs::Pose tmp_pose;
	if(task_nr_ == 5)
	{
		const YAML::Node* puzzle = task_description_node.FindValue("puzzle_fixture");
		if (!puzzle)
		{
			ROS_ERROR("EurocInput: puzzle_fixture not found in task_description_node");
			return -1;
		}
		else
		{
			const YAML::Node* fix_pose = puzzle->FindValue("pose");
			if (!fix_pose)
			{
				ROS_ERROR("EurocInput: pose not found in puzzle_fixture");
				return -1;
			}
			try {
				(*fix_pose)[0] >> tmp_pose.position.x;
				(*fix_pose)[1] >> tmp_pose.position.y;
				(*fix_pose)[2] >> tmp_pose.position.z;
				(*fix_pose)[3] >> rpy[0];
				(*fix_pose)[4] >> rpy[1];
				(*fix_pose)[5] >> rpy[2];
				q_tf_ros.setRPY(rpy[0],rpy[1],rpy[2]);
				tmp_pose.orientation.w=q_tf_ros.getW();
				tmp_pose.orientation.x=q_tf_ros.getX();
				tmp_pose.orientation.y=q_tf_ros.getY();
				tmp_pose.orientation.z=q_tf_ros.getZ();

				fixture_pose_=tmp_pose;
			} catch (YAML::Exception e) {
				ROS_ERROR("EurocInput: YAML Error in puzzle pose");
				return -1;
			}
		}

		const YAML::Node* rel_puzzle = task_description_node.FindValue("relative_puzzle_part_target_poses");
		if (!rel_puzzle)
		{
			ROS_ERROR("EurocInput: relative_puzzle_part_targt_poses not found in task_description_node");
			return -1;
		}
		else
		{
			puzzle_target_poses_.resize(nr_objects_);
			for(uint16_t ii=0;ii<nr_objects_;ii++)
			{
				const YAML::Node* puzzle_fix = rel_puzzle->FindValue(objects_[ii].name.c_str());
				if (!puzzle_fix)
				{
					msg_error("EurocInput: object %s not found in puzzle_fixture",objects_[ii].name.c_str());
					return -1;
				}
				try {
					(*puzzle_fix)[0] >> tmp_pose.position.x;
					(*puzzle_fix)[1] >> tmp_pose.position.y;
					(*puzzle_fix)[2] >> tmp_pose.position.z;
					(*puzzle_fix)[3] >> rpy[0];
					(*puzzle_fix)[4] >> rpy[1];
					(*puzzle_fix)[5] >> rpy[2];
					q_tf_ros.setRPY(rpy[0],rpy[1],rpy[2]);
					tmp_pose.orientation.w=q_tf_ros.getW();
					tmp_pose.orientation.x=q_tf_ros.getX();
					tmp_pose.orientation.y=q_tf_ros.getY();
					tmp_pose.orientation.z=q_tf_ros.getZ();

					puzzle_target_poses_[ii]=tmp_pose;

				} catch (YAML::Exception e) {
					ROS_ERROR("EurocInput: YAML Error in puzzle pose");
					return -1;
				}
			}
		}

#ifdef DBG_OUT
		//print puzzle info:
		ROS_INFO("Fixture pose: [%3.2f %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f]",
				  fixture_pose_.position.x,fixture_pose_.position.y,fixture_pose_.position.z,
				  fixture_pose_.orientation.w,fixture_pose_.orientation.x,fixture_pose_.orientation.y,
				  fixture_pose_.orientation.z);

		for(uint16_t ii=0;ii<nr_objects_;ii++)
		{
			ROS_INFO("Object %s pose: [%3.2f %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f]",
					objects_[ii].name.c_str(),
					  puzzle_target_poses_[ii].position.x,puzzle_target_poses_[ii].position.y,puzzle_target_poses_[ii].position.z,
					  puzzle_target_poses_[ii].orientation.w,puzzle_target_poses_[ii].orientation.x,puzzle_target_poses_[ii].orientation.y,
					  puzzle_target_poses_[ii].orientation.z);
		}

#endif
	} //if(task_nr_ == 5)

	//######################################################################################
	//######################################################################################
	//Conveyor Belt:
	geometry_msgs::Vector3 tmp_vec;
	if(task_nr_ == 6)
	{
		const YAML::Node* belt = task_description_node.FindValue("conveyor_belt");
		if (!belt)
		{
			ROS_ERROR("EurocInput: conveyor_belt not found in task_description_node");
			return -1;
		}
		else
		{
			const YAML::Node* dir = belt->FindValue("move_direction_and_length");
			if (!dir)
			{
				ROS_ERROR("EurocInput: move_direction_and_length not found in conveyor belt");
				return -1;
			}
			try {
				(*dir)[0] >> conv_belt_.move_direction_and_length.x;
				(*dir)[1] >> conv_belt_.move_direction_and_length.y;
				(*dir)[2] >> conv_belt_.move_direction_and_length.z;

			} catch (YAML::Exception e) {
				ROS_ERROR("EurocInput: YAML Error in conveyor belt");
				return -1;
			}

			const YAML::Node* drop = belt->FindValue("drop_center_point");
			if (!drop)
			{
				ROS_ERROR("EurocInput: drop_center_point not found in conveyor belt");
				return -1;
			}
			try {
				(*drop)[0] >> conv_belt_.drop_center_point.x;
				(*drop)[1] >> conv_belt_.drop_center_point.y;
				(*drop)[2] >> conv_belt_.drop_center_point.z;

			} catch (YAML::Exception e) {
				ROS_ERROR("EurocInput: YAML Error in conveyor belt");
				return -1;
			}

			const YAML::Node* dev = belt->FindValue("drop_deviation");
			if (!dev)
			{
				ROS_ERROR("EurocInput: drop_deviation not found in conveyor belt");
				return -1;
			}
			try {
				(*dev)[0] >> conv_belt_.drop_deviation.x;
				(*dev)[1] >> conv_belt_.drop_deviation.y;
				(*dev)[2] >> conv_belt_.drop_deviation.z;

			} catch (YAML::Exception e) {
				ROS_ERROR("EurocInput: YAML Error in conveyor belt");
				return -1;
			}

			const YAML::Node* speed_b = belt->FindValue("start_speed");
			if (!speed_b)
			{
				ROS_ERROR("EurocInput: start_speed not found in conveyor belt");
				return -1;
			}
			try {
				(*speed_b) >> conv_belt_.start_speed;

			} catch (YAML::Exception e) {
				ROS_ERROR("EurocInput: YAML Error in conveyor belt");
				return -1;
			}

			const YAML::Node* speed_e = belt->FindValue("end_speed");
			if (!speed_e)
			{
				ROS_ERROR("EurocInput: end_speed not found in conveyor belt");
				return -1;
			}
			try {
				(*speed_e) >> conv_belt_.end_speed;

			} catch (YAML::Exception e) {
				ROS_ERROR("EurocInput: YAML Error in conveyor belt");
				return -1;
			}

			const YAML::Node* nr_obj = belt->FindValue("n_objects");
			if (!nr_obj)
			{
				ROS_ERROR("EurocInput: n_objects not found in conveyor belt");
				return -1;
			}
			try {
				(*nr_obj) >> conv_belt_.n_objects;

			} catch (YAML::Exception e) {
				ROS_ERROR("EurocInput: YAML Error in conveyor belt");
				return -1;
			}

			const YAML::Node* obj = belt->FindValue("object_template");
			if (!obj)
			{
				ROS_ERROR("EurocInput: object_template not found in conveyor belt");
				return -1;
			}
			try {
				*obj >> conv_belt_.object_template;

			} catch (YAML::Exception e) {
				ROS_ERROR("EurocInput: YAML Error in conveyor belt");
				return -1;
			}
		}

		am_msgs::Object tmp_obj=objects_[0];
		nr_objects_=conv_belt_.n_objects;
		objects_.resize(nr_objects_);
		for(uint16_t ii=0;ii<nr_objects_;ii++)
		{
			objects_[ii]=tmp_obj;
		}

#ifdef DBG_OUT
		ROS_INFO("Conveyor Belt yaml result:");
		ROS_INFO("move dir and length: [%f %f %f]",conv_belt_.move_direction_and_length.x,
				conv_belt_.move_direction_and_length.z,conv_belt_.move_direction_and_length.y);
		ROS_INFO("drop center point: [%f %f %f]",conv_belt_.drop_center_point.x,
				conv_belt_.drop_center_point.y,conv_belt_.drop_center_point.z);
		ROS_INFO("drop deviation: [%f %f %f]",conv_belt_.drop_deviation.x,
				conv_belt_.drop_deviation.y,conv_belt_.drop_deviation.z);
		ROS_INFO("speed: start %f, end %f",conv_belt_.start_speed,conv_belt_.end_speed);
		ROS_INFO("number of objects: %d",conv_belt_.n_objects);
#endif
	} //task_nr == 6

	//######################################################################################
	//######################################################################################

	return 0;
}

void EurocInput::select_new_object()
{
	uint16_t actualindex;
	actualindex=active_object_;

	//Find next object which is not finsihed. Start to search right after the
	//active object. If all are finished remain at the actual object.
	for(uint16_t ii=0; ii<nr_objects_; ii++)
	{
		actualindex++;
		if(actualindex<0 || actualindex>=nr_objects_)
		{
			actualindex=0;
		}
		if(obj_finished_[actualindex]==0)
		{
			active_object_=actualindex;
			//...and determine the target zone
			std::string obj_name=objects_[active_object_].name;
			for(uint16_t jj=0;jj<nr_zones_;jj++)
			{
				if(!obj_name.compare(target_zones_[jj].expected_object.c_str()))
				{
					active_zone_=jj;
				}
			}

			//...and stop searching
			break;
		}
	}
}

am_msgs::Object EurocInput::get_active_object()
{
	if(active_object_<0 || active_object_ >= nr_objects_)
	{
		msg_error("EurocInput: get_active_object() failed. Index out of range.");
		am_msgs::Object empty_obj;
		return empty_obj;
	}
	else
	{
		return objects_[active_object_];
	}
}

am_msgs::TargetZone EurocInput::get_active_target_zone()
{
	if(active_zone_<0 || active_zone_ >= nr_zones_)
	{
		if(task_nr_ != 5)
			msg_error("EurocInput: get_active_target_zone() failed. Index out of range.");

		am_msgs::TargetZone empty_zone;
		return empty_zone;
	}
	else
	{
		return target_zones_[active_zone_];
	}
}

void EurocInput::save_objects_to_parameter_server(ros::NodeHandle& n, bool show_log_messages)
{
	n.setParam("nr_objects_", nr_objects_);
	if(show_log_messages)
	{ ROS_INFO("saved nr_objects_=%d to parameter server.", nr_objects_); }

	std::stringstream parname;
	uint8_t objecttype;
	uint8_t shapetype;
	uint8_t intnrshapes;

	for(uint16_t ii=0; ii<nr_objects_; ii++)
	{
		objecttype=OBJECT_UNKNOWN;
		if(objects_[ii].nr_shapes==1)
		{
			if(objects_[ii].shape[0].type == "cylinder")
			{
				objecttype=OBJECT_CYLINDER;
			}
			if(objects_[ii].shape[0].type == "box")
			{
				objecttype=OBJECT_CUBE;
			}
		}
		if(objects_[ii].nr_shapes==3)
		{
			objecttype=OBJECT_HANDLE;
		}

		//saving object name
		parname.str("");
		parname << "object_" << ii << "_name_";
		n.setParam(parname.str(), objects_[ii].name);
		if(show_log_messages)
		{ ROS_INFO("saved %s=%s to parameter server.", parname.str().c_str(), objects_[ii].name.c_str()); }

		//saving object type
		parname.str("");
		parname << "object_" << ii << "_type_";
		n.setParam(parname.str(), objecttype);
		if(show_log_messages)
		{ ROS_INFO("saved %s=%d to parameter server.", parname.str().c_str(),objecttype); }

		//saving object color
		parname.str("");
		parname << "object_" << ii << "_color_";
		n.setParam(parname.str(), objects_[ii].color);
		if(show_log_messages)
		{ ROS_INFO("saved %s=%s to parameter server.", parname.str().c_str(), objects_[ii].color.c_str()); }

		//saving object shapes
		intnrshapes=objects_[ii].nr_shapes;
		parname.str("");
		parname << "object_" << ii << "_nr_shapes_";
		n.setParam(parname.str(), intnrshapes);
		if(show_log_messages)
		{ ROS_INFO("saved %s=%d to parameter server.", parname.str().c_str(), intnrshapes); }

		for(uint16_t jj=0; jj<objects_[ii].shape.size(); jj++)
		{
			if(objects_[ii].shape[jj].type == "cylinder")
			{
				shapetype=SHAPE_CYLINDER;
				parname.str("");
				parname << "object_" << ii  << "_shape_" << jj << "_type_";
				n.setParam(parname.str(), shapetype);
				if(show_log_messages)
				{ ROS_INFO("saved %s=%d to parameter server.", parname.str().c_str(), shapetype); }

				parname.str("");
				parname << "object_" << ii << "_shape_" << jj << "_length_";
				n.setParam(parname.str(), objects_[ii].shape[jj].length);
				if(show_log_messages)
				{ ROS_INFO("saved %s=%3.2f to parameter server.", parname.str().c_str(), objects_[ii].shape[jj].length); }

				parname.str("");
				parname << "object_" << ii << "_shape_" << jj << "_radius_";
				n.setParam(parname.str(), objects_[ii].shape[jj].radius);
				if(show_log_messages)
				{ ROS_INFO("saved %s=%3.2f to parameter server.", parname.str().c_str(), objects_[ii].shape[jj].radius); }
			}
			else if(objects_[ii].shape[jj].type == "box")
			{
				shapetype=SHAPE_BOX;
				parname.str("");
				parname << "object_" << ii << "_shape_" << jj << "_type_";
				n.setParam(parname.str(), shapetype);
				if(show_log_messages)
				{ ROS_INFO("saved %s=%d to parameter server.", parname.str().c_str(), shapetype); }

				for(uint16_t kk=0; kk<objects_[ii].shape[jj].size.size(); kk++)
				{
					parname.str("");
					parname << "object_" << ii << "_shape_" << jj << "_size_" << kk << "_";
					n.setParam(parname.str(), objects_[ii].shape[jj].size[kk]);
					if(show_log_messages)
					{ ROS_INFO("saved %s=%3.2f to parameter server.", parname.str().c_str(), objects_[ii].shape[jj].size[kk]); }
				}
			}
			else
			{
				shapetype=SHAPE_UNKNOWN;
				parname.str("");
				parname << "object_" << ii << "_shape_" << jj << "_type_";
				n.setParam(parname.str(), shapetype);
				if(show_log_messages)
				{ ROS_INFO("saved %s=%d to parameter server.", parname.str().c_str(), shapetype); }
			}

			//saving shape pose
			parname.str("");
			parname << "object_" << ii  << "_shape_" << jj << "_pose_position_x_";
			n.setParam(parname.str(), objects_[ii].shape[jj].pose.position.x);
			if(show_log_messages)
			{ ROS_INFO("saved %s=%3.2f to parameter server.", parname.str().c_str(), objects_[ii].shape[jj].pose.position.x); }

			parname.str("");
			parname << "object_" << ii  << "_shape_" << jj << "_pose_position_y_";
			n.setParam(parname.str(), objects_[ii].shape[jj].pose.position.y);
			if(show_log_messages)
			{ ROS_INFO("saved %s=%3.2f to parameter server.", parname.str().c_str(), objects_[ii].shape[jj].pose.position.y); }

			parname.str("");
			parname << "object_" << ii  << "_shape_" << jj << "_pose_position_z_";
			n.setParam(parname.str(), objects_[ii].shape[jj].pose.position.z);
			if(show_log_messages)
			{ ROS_INFO("saved %s=%3.2f to parameter server.", parname.str().c_str(), objects_[ii].shape[jj].pose.position.z); }

			parname.str("");
			parname << "object_" << ii  << "_shape_" << jj << "_pose_orientation_w_";
			n.setParam(parname.str(), objects_[ii].shape[jj].pose.orientation.w);
			if(show_log_messages)
			{ ROS_INFO("saved %s=%3.2f to parameter server.", parname.str().c_str(), objects_[ii].shape[jj].pose.orientation.w); }

			parname.str("");
			parname << "object_" << ii  << "_shape_" << jj << "_pose_orientation_x_";
			n.setParam(parname.str(), objects_[ii].shape[jj].pose.orientation.x);
			if(show_log_messages)
			{ ROS_INFO("saved %s=%3.2f to parameter server.", parname.str().c_str(), objects_[ii].shape[jj].pose.orientation.x); }

			parname.str("");
			parname << "object_" << ii  << "_shape_" << jj << "_pose_orientation_y_";
			n.setParam(parname.str(), objects_[ii].shape[jj].pose.orientation.y);
			if(show_log_messages)
			{ ROS_INFO("saved %s=%3.2f to parameter server.", parname.str().c_str(), objects_[ii].shape[jj].pose.orientation.y); }

			parname.str("");
			parname << "object_" << ii  << "_shape_" << jj << "_pose_orientation_z_";
			n.setParam(parname.str(), objects_[ii].shape[jj].pose.orientation.z);
			if(show_log_messages)
			{ ROS_INFO("saved %s=%3.2f to parameter server.", parname.str().c_str(), objects_[ii].shape[jj].pose.orientation.z); }
		}
	}
}

void EurocInput::save_target_zone_to_parameter_server(ros::NodeHandle& n, bool show_log_messages)
{
	std::stringstream parname;

	for(uint16_t ii=0;ii<nr_zones_;ii++)
	{
		parname.str("");
		parname << "target_zone_" << ii << "_x_";
		n.setParam(parname.str(), target_zones_[ii].position.x);
		parname.str("");
		parname << "target_zone_" << ii << "_y_";
		n.setParam(parname.str(), target_zones_[ii].position.y);
		parname.str("");
		parname << "target_zone_" << ii << "_radius_";
		n.setParam(parname.str(), target_zones_[ii].max_distance);

		uint16_t obj_nr;
		for(uint16_t jj=0;jj<nr_objects_;jj++)
		{
			if(!target_zones_[ii].expected_object.compare(objects_[jj].name.c_str()))
			{
				obj_nr=jj;
				break;
			}
		}
		parname.str("");
		parname << "target_zone_" << ii << "_obj_nr_";
		n.setParam(parname.str(), obj_nr);
	}
}

void EurocInput::save_robot_to_parameter_server(ros::NodeHandle& n, bool show_log_messages)
{
	std::stringstream parname;

	for(uint16_t ii=0;ii<robot_.speed_limit.size();ii++)
	{
		parname.str("");
		parname << "joint_speed_limit_" << ii;
		n.setParam(parname.str(), robot_.speed_limit[ii]);
		if(show_log_messages)
		{ ROS_INFO("saved %s=%3.2f to parameter server.", parname.str().c_str(), robot_.speed_limit[ii]);  }
	}

	parname.str("gripper_speed_limit");
	n.setParam(parname.str(), robot_.gripper_speed_limit);
	if(show_log_messages)
	{ ROS_INFO("saved %s=%3.2f to parameter server.", parname.str().c_str(), robot_.gripper_speed_limit);  }

	for(uint16_t ii=0;ii<robot_.two_axes_speed_limit.size();ii++)
	{
		parname.str("");
		parname << "two_axes_speed_limit_" << ii;
		n.setParam(parname.str(), robot_.two_axes_speed_limit[ii]);
		if(show_log_messages)
		{ ROS_INFO("saved %s=%3.2f to parameter server.", parname.str().c_str(), robot_.two_axes_speed_limit[ii]);  }
	}
}

void EurocInput::print_object(am_msgs::Object*obj)
{
	ROS_INFO("Name: %s",obj->name.c_str());
	ROS_INFO("description: %s",obj->description.c_str());
	ROS_INFO("color: %s",obj->color.c_str());
	ROS_INFO("Pose: [%3.2f %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f]",
			  obj->abs_pose.position.x,obj->abs_pose.position.y,obj->abs_pose.position.z,
			  obj->abs_pose.orientation.w,obj->abs_pose.orientation.x,obj->abs_pose.orientation.y,
			  obj->abs_pose.orientation.z);
	ROS_INFO("Nr. of shapes: %d",obj->nr_shapes);
	for(unsigned zz=0;zz<obj->nr_shapes;zz++)
	{
		ROS_INFO("------------------------");
		ROS_INFO("shape %d: type: %s",zz,obj->shape[zz].type.c_str());
		ROS_INFO("density: %f",obj->shape[zz].density);
		if(obj->shape[zz].type == "cylinder")
			ROS_INFO("Length: %3.2f, Radius: %3.2f",obj->shape[zz].length,
				  obj->shape[zz].radius);
		else if (obj->shape[zz].type == "box")
			ROS_INFO("Size: [%3.2f %3.2f %3.2f]",obj->shape[zz].size[0],obj->shape[zz].size[1]
									 ,obj->shape[zz].size[2]);
		else
			ROS_INFO("Unknown!!!");
		ROS_INFO("Relative pose: [%3.2f %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f]",
				obj->shape[zz].pose.position.x,obj->shape[zz].pose.position.y,
				obj->shape[zz].pose.position.z,obj->shape[zz].pose.orientation.w,obj->shape[zz].pose.orientation.x,
				obj->shape[zz].pose.orientation.y,obj->shape[zz].pose.orientation.z);
	}
}

void EurocInput::set_active_object_finished()
{
	if(active_object_<0 || active_object_ >= nr_objects_)
	{
		msg_error("EurocInput: set_active_object_finished() failed. Index out of range.");
	}
	else
	{
		obj_finished_[active_object_] = 1;
	}
}

bool EurocInput::is_active_object_last_object()
{
	if(active_object_ == nr_objects_-1)
		return true;
	else
		return false;
}

bool EurocInput::all_finished()
{
	uint16_t cnt=0;
	for(uint16_t ii=0; ii<nr_objects_; ii++)
	{
		if(obj_finished_[ii]==1)
			cnt++;
	}
	if(cnt==nr_objects_)
		return true;
	else
		return false;
}

void EurocInput::reset()
{
	task_nr_=0;
	description_="";
	log_filename_="";
	objects_.clear();
	nr_objects_=0;
	active_object_=-1;
	obj_finished_.clear();
	target_zones_.clear();
	nr_zones_=0;
	active_zone_=0;
	sensors_.clear();
	nr_sensors=0;
	puzzle_target_poses_.clear();
}
