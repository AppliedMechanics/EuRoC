#include <euroc_input.hpp>
#include <tf_rot.hpp>
#include <iostream>

#define DBG_OUT

#define msg_error(format,...) \
			ROS_ERROR("%s - %d: "format,__FUNCTION__,__LINE__,##__VA_ARGS__)

EurocInput::EurocInput()
{

}

int EurocInput::parse_yaml_file(std::string task_yaml_description)
{
	// Parse the explanation/description of the task from the yaml string
	std::stringstream yaml_stream(task_yaml_description);


	YAML::Parser parser(yaml_stream);
	YAML::Node task_description_node;
	parser.GetNextDocument(task_description_node);

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
	try{
		(*base_pose)[0] >> base_pose_.position.x;
		(*base_pose)[1] >> base_pose_.position.y;
		(*base_pose)[2] >> base_pose_.position.z;
		(*base_pose)[3] >> rpy[0];
		(*base_pose)[4] >> rpy[1];
		(*base_pose)[5] >> rpy[2];
		rpy2quat(rpy,a_tf,q_tf);
		base_pose_.orientation.w=q_tf[0];
		base_pose_.orientation.x=q_tf[1];
		base_pose_.orientation.y=q_tf[2];
		base_pose_.orientation.z=q_tf[3];
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
		rpy2quat(rpy,a_tf,q_tf);
		pan_tilt_base_.orientation.w=q_tf[0];
		pan_tilt_base_.orientation.x=q_tf[1];
		pan_tilt_base_.orientation.y=q_tf[2];
		pan_tilt_base_.orientation.z=q_tf[3];
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

	nr_objects = objects->size();
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

		const YAML::Node* sur = obj->FindValue("surface_material");
		if(!des){
			ROS_ERROR("EurocInput: surface material not found in object");
		    return -1;
		}
		try {
			*sur >> tmp_obj.surface_material;
		}catch(YAML::Exception e){
	    	ROS_ERROR("EurocInput: Obj error");
	    	return -1;
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
				rpy2quat(rpy,a_tf,q_tf);
				tmp_obj.shape[jj].pose.orientation.w=q_tf[0];
				tmp_obj.shape[jj].pose.orientation.x=q_tf[1];
				tmp_obj.shape[jj].pose.orientation.y=q_tf[2];
				tmp_obj.shape[jj].pose.orientation.z=q_tf[3];
			}catch(YAML::Exception e) {
				ROS_ERROR("Pose not found in shape");
				return -1;
			}
			jj++;

		} /* for (YAML::Iterator it2 = shapes->begin(); it2 != shapes->end();++it2) */

		objects_.push_back(tmp_obj);
	    ii++;
	  } /*for(YAML::Iterator it = objects->begin(); it != objects->end(); ++it) */

#ifdef DBG_OUT
	  for(unsigned kk=0;kk<nr_objects;kk++)
	  {
		  ROS_INFO("======================");
		  ROS_INFO("Object %d:",kk);
		  ROS_INFO("Name: %s",objects_[kk].name.c_str());
		  ROS_INFO("description: %s",objects_[kk].description.c_str());
		  ROS_INFO("color: %s",objects_[kk].color.c_str());
		  ROS_INFO("Pose: [%3.2f %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f]",
					  objects_[kk].abs_pose.position.x,objects_[kk].abs_pose.position.y,objects_[kk].abs_pose.position.z,
					  objects_[kk].abs_pose.orientation.w,objects_[kk].abs_pose.orientation.x,objects_[kk].abs_pose.orientation.y,
					  objects_[kk].abs_pose.orientation.z);
		  ROS_INFO("Nr. of shapes: %d",objects_[kk].nr_shapes);
		  for(unsigned zz=0;zz<objects_[kk].nr_shapes;zz++)
		  {
			  ROS_INFO("");
			  ROS_INFO("shape %d: type: %s",zz,objects_[kk].shape[zz].type.c_str());
			  if(objects_[kk].shape[zz].type == "cylinder")
				  ROS_INFO("Length: %3.2f, Radius: %3.2f",objects_[kk].shape[zz].length,
						  objects_[kk].shape[zz].radius);
			  else if (objects_[kk].shape[zz].type == "box")
				  ROS_INFO("Size: [%3.2f %3.2f %3.2f]",objects_[kk].shape[zz].size[0],objects_[kk].shape[zz].size[1]
											 ,objects_[kk].shape[zz].size[2]);
			  else
				  ROS_INFO("Unknown!!!");
			  ROS_INFO("Relative pose: [%3.2f %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f]",
					  objects_[kk].shape[zz].pose.position.x,objects_[kk].shape[zz].pose.position.y,
					  objects_[kk].shape[zz].pose.position.z,objects_[kk].shape[zz].pose.orientation.w,objects_[kk].shape[zz].pose.orientation.x,
					  objects_[kk].shape[zz].pose.orientation.y,objects_[kk].shape[zz].pose.orientation.z);
		  }
	  }
#endif

	//######################################################################################
	//######################################################################################
	//Target zones:
	const YAML::Node *target_zones = task_description_node.FindValue("target_zones");
	  if(!target_zones){
	    ROS_ERROR("Target zones not found in task description");
	    return -1;
	  }
	  am_msgs::TargetZone tmp_zone;
	  ii=0;
	  nr_zones = target_zones->size();
	  for(YAML::Iterator it = target_zones->begin(); it != target_zones->end(); ++it) {

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
	  for(unsigned kk=0;kk<nr_zones;kk++)
	  {
		  ROS_INFO("======================");
		  ROS_INFO("Target zone %d:",kk);
		  ROS_INFO("Expected object: %s",target_zones_[kk].expected_object.c_str());
		  ROS_INFO("Target position: [%f %f %f]",target_zones_[kk].position.x,
				  target_zones_[kk].position.y,target_zones_[kk].position.z);
		  ROS_INFO("Max distance: %f",target_zones_[kk].max_distance);
	  }
#endif

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
		rpy2quat(rpy, a_tf, q_tf);
		robot_.pose.orientation.w = q_tf[0];
		robot_.pose.orientation.x = q_tf[1];
		robot_.pose.orientation.y = q_tf[2];
		robot_.pose.orientation.z = q_tf[3];
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
		rpy2quat(rpy, a_tf, q_tf);
		robot_.gripper_pose.orientation.w = q_tf[0];
		robot_.gripper_pose.orientation.x = q_tf[1];
		robot_.gripper_pose.orientation.y = q_tf[2];
		robot_.gripper_pose.orientation.z = q_tf[3];
	} catch (YAML::Exception e) {
		ROS_ERROR("EurocInput: YAML Error in gripper pose");
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
				rpy2quat(rpy, a_tf, q_tf);
				tmp_sens.pose.orientation.w = q_tf[0];
				tmp_sens.pose.orientation.x = q_tf[1];
				tmp_sens.pose.orientation.y = q_tf[2];
				tmp_sens.pose.orientation.z = q_tf[3];
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
				rpy2quat(rpy, a_tf, q_tf);
				tmp_sens.pose.orientation.w = q_tf[0];
				tmp_sens.pose.orientation.x = q_tf[1];
				tmp_sens.pose.orientation.y = q_tf[2];
				tmp_sens.pose.orientation.z = q_tf[3];
			} catch (YAML::Exception e) {
				ROS_ERROR("EurocInput: YAML Error in sensor pose");
				return -1;
			}
			tmp_sens.parent = -1;
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

	return 0;
}
