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
	obj_state_.resize(nr_objects_,EIN_OBJ_INIT);

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
	    else
	    {
	    	tmp_obj.description="";
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
	    else
	    {
	    	tmp_obj.surface_material="";
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

			if((task_nr_ != 5) && (task_nr_ != 6))
			{
				try{
					const YAML::Node* dens = (*it2).FindValue("density");
					(*dens) >> tmp_obj.shape[jj].density;
				}catch(YAML::Exception e) {
					msg_warn("density not found in shape");

					tmp_obj.shape[jj].density=7850;
					//return -1;
				}
			}
			else
			{
				tmp_obj.shape[jj].density=7850;
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

int EurocInput::sort_objects(std::vector<uint16_t> target_zone_occupied)
{
	msg_info("EurocInput: sort_objects() started");
	//ret: -1 -> error happend, ret:0 -> sort successful,continue, ret=1 -> problem with targetzone
	if(task_nr_==6)
	{
		obj_queue_t temp_obj;
		obj_queue_.clear();
		for(uint16_t ii=0;ii<nr_objects_;ii++)
		{
			temp_obj.action=EIN_PLACE;
			temp_obj.data=&objects_[0];
			temp_obj.obj_idx=0;
			temp_obj.target_zone_idx=0;
			temp_obj.target_zone_occupied=0;

			obj_queue_.push_back(temp_obj);
		}
	}
	else if(task_nr_==5)
	{
		ROS_INFO("calling order puzzle pieces:");
		order_of_puzzle_pieces();

		obj_queue_t temp_obj;
		obj_queue_.clear();
		for(uint16_t ii=0;ii<nr_objects_;ii++)
		{
			temp_obj.action=EIN_PLACE;
			temp_obj.data=&objects_[puzzle_order_[ii].part_index];
			temp_obj.obj_idx=puzzle_order_[ii].part_index;
			temp_obj.target_zone_idx=0;
			temp_obj.target_zone_occupied=0;

			obj_queue_.push_back(temp_obj);
		}
	}
	else
	{
		//copy target zone data
		if(target_zones_.size() != target_zone_occupied.size())
		{
			msg_error("EurocInput: sort_objects() failed. Targetzone sizes dont match");
			return -1;
		}

		//!target_zone index for objects
		std::vector<uint16_t> target_zone_idx_;
		target_zone_idx_.resize(nr_objects_);
		//!target zone of object is occupied
		std::vector<uint16_t> target_zone_occupied_;
		target_zone_occupied_.resize(nr_objects_);

		//find target zone number and target_zone state for each object
		uint16_t nr_occupied=0;
		for(uint16_t oo=0;oo<nr_objects_;oo++)
		{
			std::string obj_name=objects_[oo].name;
			for(uint16_t zz=0;zz<nr_zones_;zz++)
			{
				if(!obj_name.compare(target_zones_[zz].expected_object.c_str()))
				{
					target_zone_idx_[oo]=zz;
					if(target_zone_occupied[zz]==1)
					{
						target_zone_occupied_[oo]=1;
						nr_occupied++;
					}
				}
			}
		} //for(uint16_t oo=0;oo<nr_objects_;oo++)

		//1. sort objects by type (put handle-bar at the end)
		obj_queue_.clear();
		obj_queue_t temp_obj;
		for(unsigned ii=0;ii<nr_objects_;ii++)
		{
			temp_obj.obj_idx=ii;
			temp_obj.data = &objects_[ii];
			temp_obj.action=EIN_PLACE;
			temp_obj.target_zone_occupied=target_zone_occupied_[ii];
			temp_obj.target_zone_idx=target_zone_idx_[ii];
			if(objects_[ii].nr_shapes==1 && task_nr_!= 5)
			{
				obj_queue_.insert(obj_queue_.begin(),temp_obj);
			}
			else
			{
				obj_queue_.push_back(temp_obj);
			}
		}

		//2. sort objects by target zone state
		if(nr_occupied<2)
		{
			for(unsigned ii=0;ii<nr_objects_;ii++)
			{
				if(obj_queue_[ii].target_zone_occupied==1)
				{
					temp_obj=obj_queue_[ii];
					obj_queue_.erase(obj_queue_.begin()+ii);
					obj_queue_.push_back(temp_obj);
					break;
				}
			}
		}
		else if(nr_occupied==2)
		{
			uint16_t cnt=0;
			for(unsigned ii=0;ii<obj_state_.size();ii++)
				if(obj_state_[ii]==EIN_OBJ_LOCATED)
					cnt++;
			//ROS_INFO("cnt=%d, obj_state_.size()=%d",cnt,obj_state_.size());

			if(cnt!=obj_state_.size())
			{
				ROS_INFO("2 Target zones are occupied! -> Try to locate all objects first");
				ROS_INFO("Objects:");
				for(unsigned ii=0;ii<objects_.size();ii++)
				{
					ROS_INFO("# %d : Object %s",ii,objects_[ii].name.c_str());
					ROS_INFO("Targetzone (%d) occupied: %s",target_zone_idx_[ii],target_zone_occupied_[ii] ? "true":"false");
					switch(obj_state_[ii])
					{
					case EIN_OBJ_INIT:
						ROS_INFO("state: EIN_OBJ_INIT");
						break;
					case EIN_OBJ_LOCATED:
						ROS_INFO("state: EIN_OBJ_LOCATED");
						break;
					case EIN_OBJ_PARKING:
						ROS_INFO("state: EIN_OBJ_PARKING");
						break;
					case EIN_OBJ_FINISHED:
						ROS_INFO("state: EIN_OBJ_FINISHED");
						break;
					case EIN_OBJ_UNCERTAIN:
						ROS_INFO("state: EIN_OBJ_UNCERTAIN");
						break;
					default:
						ROS_ERROR("action unknown!!");
						return -1;
					}
				}
				return 1;
			}
			else
			{
				//msg_info("implement me!!");
				//vector of objects on zones, indizes are stored in a tupel
				std::vector<std::vector<int16_t> >obj_idx_on_zone_;
				//def: (obj idx,target zone idx)
				std::vector<int16_t> tupel;
				tupel.resize(2);


				//find occupied zones
				for(uint16_t ii=0;ii<nr_zones_;ii++)
				{
					if(target_zone_occupied[ii])
					{
						double rad2=pow(target_zones_[ii].max_distance,2);
						//find object on that zone
						for(uint16_t oo=0;oo<nr_objects_;oo++)
						{
							double dist2=pow(objects_[oo].abs_pose.position.x-target_zones_[ii].position.x,2)+
									pow(objects_[oo].abs_pose.position.y-target_zones_[ii].position.y,2);

							//object is lying on that zone
							if(dist2<rad2)
							{
								tupel[0]=oo;
								tupel[1]=ii;
								obj_idx_on_zone_.push_back(tupel);

								if(target_zone_idx_[oo]==ii)
								{
									ROS_INFO("object %d is already in its target zone (%d)!!",oo,ii);
									obj_state_[oo]=EIN_OBJ_FINISHED;
								}
								break;
							}
						}
					}
				} //for(uint16_t ii=0;ii<nr_zones_;ii++)
				ROS_INFO("############################################");
				ROS_INFO("Result for 2 Objects on 2 target_zones:");
				ROS_INFO(" ");
				ROS_INFO("obj_idx_on_zone_(0,0)=%d | obj_idx_on_zone_(0,1)=%d ",obj_idx_on_zone_[0][0],obj_idx_on_zone_[0][1]);
				ROS_INFO("obj_idx_on_zone_(1,0)=%d | obj_idx_on_zone_(1,1)=%d ",obj_idx_on_zone_[1][0],obj_idx_on_zone_[1][1]);
				ROS_INFO(" ");
				ROS_INFO("Interpretation:");
				for(uint16_t ii=0;ii<obj_idx_on_zone_.size();ii++)
				{
					ROS_INFO("Object # %d on TargetZone %d",obj_idx_on_zone_[ii][0],obj_idx_on_zone_[ii][1]);
					//quatsch!
					//ROS_INFO("Targetzone belongs to Object # %d",target_zone_idx_[obj_idx_on_zone_[ii][0]]);
				}

				//delete occupied objects from queue
				for(uint16_t ii=0;ii<obj_queue_.size();ii++)
				{
					if(obj_queue_[ii].target_zone_occupied==0)
					{
						temp_obj=obj_queue_[ii];
						obj_queue_.clear();
						obj_queue_.push_back(temp_obj);
						break;
					}
				}

				//find objects with one shape (= not handlebar)
				int cnt_1shape=0;
				int handle_idx=0;
				for(uint16_t ii=0;ii<obj_idx_on_zone_.size();ii++)
				{
					if(objects_[obj_idx_on_zone_[ii][0]].nr_shapes==1)
					{
						cnt_1shape++;
					}
					else
					{
						handle_idx=obj_idx_on_zone_[ii][0];
						obj_idx_on_zone_.erase(obj_idx_on_zone_.begin()+ii);
					}
				}

				if(cnt_1shape==2)
				{
					uint16_t box_idx;
					if( strcmp(objects_[obj_idx_on_zone_[0][0]].shape[0].type.c_str(),"box") == 0)
						box_idx=0;
					else
						box_idx=1;

					temp_obj.obj_idx=obj_idx_on_zone_[box_idx][0];
					temp_obj.data=&objects_[obj_idx_on_zone_[box_idx][0]];
					temp_obj.action=EIN_PARKING;
					temp_obj.target_zone_idx=target_zone_idx_[obj_idx_on_zone_[box_idx][1]];
					temp_obj.target_zone_occupied=1;
					temp_obj.target_zone_obj_idx=1-box_idx;

					obj_queue_.push_back(temp_obj);

					temp_obj.obj_idx=obj_idx_on_zone_[1-box_idx][0];
					temp_obj.data=&objects_[obj_idx_on_zone_[1-box_idx][0]];
					temp_obj.action=EIN_PLACE;
					temp_obj.target_zone_idx=target_zone_idx_[obj_idx_on_zone_[1-box_idx][1]];
					temp_obj.target_zone_occupied=0;
					temp_obj.target_zone_obj_idx=-1;

					obj_queue_.push_back(temp_obj);

					temp_obj.obj_idx=obj_idx_on_zone_[box_idx][0];
					temp_obj.data=&objects_[obj_idx_on_zone_[box_idx][0]];
					temp_obj.action=EIN_PLACE_FROM_PARKING;
					temp_obj.target_zone_idx=target_zone_idx_[obj_idx_on_zone_[box_idx][1]];
					temp_obj.target_zone_occupied=0;
					temp_obj.target_zone_obj_idx=-1;

					obj_queue_.push_back(temp_obj);
				}
				else if(cnt_1shape==1)
				{
					temp_obj.obj_idx=obj_idx_on_zone_[1-handle_idx][0];
					temp_obj.data=&objects_[obj_idx_on_zone_[1-handle_idx][0]];
					temp_obj.action=EIN_PARKING;
					temp_obj.target_zone_idx=target_zone_idx_[obj_idx_on_zone_[1-handle_idx][1]];
					temp_obj.target_zone_occupied=1;
					temp_obj.target_zone_obj_idx=handle_idx;

					obj_queue_.push_back(temp_obj);

					temp_obj.obj_idx=handle_idx;
					temp_obj.data=&objects_[obj_idx_on_zone_[handle_idx][0]];
					temp_obj.action=EIN_PLACE;
					temp_obj.target_zone_idx=target_zone_idx_[obj_idx_on_zone_[handle_idx][1]];
					temp_obj.target_zone_occupied=0;
					temp_obj.target_zone_obj_idx=-1;

					obj_queue_.push_back(temp_obj);

					temp_obj.obj_idx=obj_idx_on_zone_[1-handle_idx][0];
					temp_obj.data=&objects_[obj_idx_on_zone_[1-handle_idx][0]];
					temp_obj.action=EIN_PLACE_FROM_PARKING;
					temp_obj.target_zone_idx=target_zone_idx_[obj_idx_on_zone_[1-handle_idx][1]];
					temp_obj.target_zone_occupied=0;
					temp_obj.target_zone_obj_idx=-1;

					obj_queue_.push_back(temp_obj);
				}
				else
					msg_error("dont know what to do...");
			}
		}
		else
			msg_error("WTF");

		//print object queue
		ROS_INFO("Object queue:");
		for(unsigned ii=0;ii<obj_queue_.size();ii++)
		{
			uint16_t obj_idx=obj_queue_[ii].obj_idx;
			ROS_INFO("# %d : Object %s",obj_idx,objects_[obj_idx].name.c_str());
			ROS_INFO("Targetzone (%d) occupied: %s",target_zone_idx_[obj_idx],target_zone_occupied_[obj_idx] ? "true":"false");
			switch(obj_queue_[ii].action)
			{
			case EIN_PLACE:
				ROS_INFO("action: EIN_PLACE");
				break;
			case EIN_PARKING:
				ROS_INFO("action: EIN_PARKING");
				break;
			case EIN_PLACE_FROM_PARKING:
				ROS_INFO("action: EIN_PLACE_FROM_PARKING");
				break;
			default:
				ROS_ERROR("action unknown!!");
				return -1;
			}
		}

	} //end task_nr_!=5/6

	msg_info("EurocInput: sort_objects() finished");

	return 0;
}

void EurocInput::select_new_object()
{
	ROS_INFO("Select new object in euroc-input:");

	if(obj_queue_.size()==0)
	{
		msg_error("object queue is empty!");
		return;
	}

	if(obj_state_[obj_queue_[0].obj_idx] == EIN_OBJ_INIT)
	{
		msg_info("EurocInput: select_new_object() move to next object");
		obj_queue_t temp_obj=obj_queue_[0];

		obj_queue_.erase(obj_queue_.begin());
		obj_queue_.push_back(temp_obj);
	}
	else
	{
		//move current object to the end of obj_queue
		msg_info("current object is not finished -> move it to the end of object queue");

		obj_queue_t temp_obj=obj_queue_[0];
		obj_state_[obj_queue_[0].obj_idx] = EIN_OBJ_UNCERTAIN;

		obj_queue_.erase(obj_queue_.begin());
		obj_queue_.push_back(temp_obj);
	}

	if(obj_queue_[0].action==EIN_PARKING)
		obj_state_[obj_queue_[0].obj_idx]=EIN_OBJ_PARKING;
}

am_msgs::Object EurocInput::get_active_object()
{
	ROS_INFO("Get active object in euroc-input");

	if(obj_queue_.size()==0)
	{
		msg_error("EurocInput: get_active_object() failed. Index out of range.");
		am_msgs::Object empty_obj;
		return empty_obj;
	}
	else
	{
		return *obj_queue_[0].data;
	}
}

geometry_msgs::Pose EurocInput::get_active_target_pose()
{
        ROS_INFO("Get active targetpose in euroc-input");

        geometry_msgs::Pose empty_Pose;

    	if(task_nr_ != 5)
    	{
    		return empty_Pose;
    	}
        if(obj_queue_.size()==0)
        {
            msg_error("EurocInput: get_active_target_pose() failed. Index out of range (obj_queue_).");
            return empty_Pose;
        }
        else
        {
          if(puzzle_target_poses_.size()>obj_queue_[0].obj_idx)
          {
                return puzzle_target_poses_[obj_queue_[0].obj_idx];
          }
          else
          {
                msg_error("EurocInput: get_active_target_pose() failed. Index out of range (puzzle_target_poses_).");
                            return empty_Pose;
          }
        }
}

am_msgs::TargetZone EurocInput::get_active_target_zone()
{
	ROS_INFO("Get active targetzone in euroc-input");

	if((obj_queue_.size()==0) ||(task_nr_ == 5))
	{
		if(task_nr_ != 5)
			msg_error("EurocInput: get_active_target_zone() failed. Index out of range.");

		am_msgs::TargetZone empty_zone;
		return empty_zone;
	}
	else
	{
		if(obj_queue_[0].action == EIN_PARKING)
		{
			ROS_INFO("action = parking -> place object next to target zone");
			am_msgs::TargetZone temp_zone=target_zones_[obj_queue_[0].target_zone_idx];
			if(temp_zone.position.x>=0)
				temp_zone.position.x-=3*temp_zone.max_distance;
			else
				temp_zone.position.x+=3*temp_zone.max_distance;
			if(temp_zone.position.y>=0)
				temp_zone.position.y-=3*temp_zone.max_distance;
			else
				temp_zone.position.y+=3*temp_zone.max_distance;

			return temp_zone;
		}
		else
			return target_zones_[obj_queue_[0].target_zone_idx];
	}
}

void EurocInput::get_all_zones(std::vector<am_msgs::TargetZone> *tz)
{
	tz->resize(nr_zones_);
	*tz=target_zones_;
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
	if(task_nr_==5)
		return;

	std::stringstream parname;

	n.setParam("target_zone_nr_",nr_zones_);
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

void EurocInput::save_fixture_to_parameter_server(ros::NodeHandle& n, bool show_log_messages)
{
    if(task_nr_==5)
    {
            ROS_INFO("saving fixture pose to parameter server...");
            n.setParam("fixture_pose_position_x_",fixture_pose_.position.x);
            n.setParam("fixture_pose_position_y_",fixture_pose_.position.y);
            n.setParam("fixture_pose_position_z_",fixture_pose_.position.z);
            n.setParam("fixture_pose_orientation_x_",fixture_pose_.orientation.x);
            n.setParam("fixture_pose_orientation_y_",fixture_pose_.orientation.y);
            n.setParam("fixture_pose_orientation_z_",fixture_pose_.orientation.z);
            n.setParam("fixture_pose_orientation_w_",fixture_pose_.orientation.w);
    }
}

void EurocInput::save_conveyorbelt_to_parameter_server(ros::NodeHandle& n, bool show_log_messages)
{
    if(task_nr_==6)
    {
            ROS_INFO("saving conveyor_belt to parameter server...");
            n.setParam("conveyorbelt_move_direction_and_length_x_",conv_belt_.move_direction_and_length.x);
            n.setParam("conveyorbelt_move_direction_and_length_y_",conv_belt_.move_direction_and_length.y);
            n.setParam("conveyorbelt_move_direction_and_length_z_",conv_belt_.move_direction_and_length.z);

            n.setParam("conveyorbelt_drop_center_point_x_",conv_belt_.drop_center_point.x);
            n.setParam("conveyorbelt_drop_center_point_y_",conv_belt_.drop_center_point.y);
            n.setParam("conveyorbelt_drop_center_point_z_",conv_belt_.drop_center_point.z);

            n.setParam("conveyorbelt_drop_deviation_x_",conv_belt_.drop_deviation.x);
            n.setParam("conveyorbelt_drop_deviation_y_",conv_belt_.drop_deviation.y);
            n.setParam("conveyorbelt_drop_deviation_z_",conv_belt_.drop_deviation.z);

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
	if(obj_queue_.size()==0)
	{
		msg_error("EurocInput: set_active_object_finished() failed. obj_queue_.size()==0");
	}
	else
	{
		if(obj_queue_[0].action == EIN_PARKING)
		{
			obj_state_[obj_queue_[0].obj_idx] = EIN_OBJ_LOCATED;
		}
		else
			obj_state_[obj_queue_[0].obj_idx] = EIN_OBJ_FINISHED;
		obj_queue_.erase(obj_queue_.begin());


		if(obj_queue_[0].action==EIN_PARKING)
			obj_state_[obj_queue_[0].obj_idx]=EIN_OBJ_PARKING;
	}
}

void EurocInput::set_object_pose(geometry_msgs::Pose abs_pose)
{
	if(obj_queue_.size()==0)
	{
		msg_error("Object queue is empty!");
		return;
	}
	else
	{
		obj_queue_[0].data->abs_pose=abs_pose;
		if(obj_state_[obj_queue_[0].obj_idx] == EIN_OBJ_INIT)
			obj_state_[obj_queue_[0].obj_idx] = EIN_OBJ_LOCATED;
		return;
	}
}

void EurocInput::set_object_pose(geometry_msgs::Pose abs_pose, uint16_t idx)
{
	if(objects_.size()==0)
	{
		msg_error("Object vector is empty!");
		return;
	}
	else
	{
		objects_[idx].abs_pose=abs_pose;
		obj_state_[idx] = EIN_OBJ_LOCATED;
		return;
	}
}

bool EurocInput::is_active_object_last_object()
{
	if(obj_queue_.size() == 1)
		return true;
	else
		return false;
}

bool EurocInput::all_finished()
{
	uint16_t cnt=0;
	for(uint16_t ii=0; ii<nr_objects_; ii++)
	{
		if(obj_state_[ii]==EIN_OBJ_FINISHED)
			cnt++;
	}

	//print object queue
//	ROS_INFO("Object queue:");
//	for(unsigned ii=0;ii<obj_queue_.size();ii++)
//	{
//		uint16_t obj_idx=obj_queue_[ii].obj_idx;
//		ROS_INFO("# %d : Object %s",ii,objects_[obj_idx].name.c_str());
//		ROS_INFO("Targetzone (%d) occupied: %s",obj_queue_[ii].target_zone_idx,obj_queue_[ii].target_zone_occupied ? "true":"false");
//		switch(obj_queue_[ii].action)
//		{
//		case EIN_PLACE:
//			ROS_INFO("action: EIN_PLACE");
//			break;
//		case EIN_PARKING:
//			ROS_INFO("action: EIN_PARKING");
//			break;
//		case EIN_PLACE_FROM_PARKING:
//			ROS_INFO("action: EIN_PLACE_FROM_PARKING");
//			break;
//		default:
//			ROS_ERROR("action unknown!!");
//			return -1;
//		}
//	}
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
	target_zones_.clear();
	obj_state_.clear();
	obj_queue_.clear();
	nr_zones_=0;
	active_zone_=0;
	sensors_.clear();
	nr_sensors=0;
	puzzle_target_poses_.clear();
}

void EurocInput::order_of_puzzle_pieces()
{
  bool fix_x_below = false;
  bool fix_y_below = false;

  bool order_found = false;

  bool fix_x_above = false;
  bool fix_y_above = false;

  bool all_fix_x_below = true;
  bool all_fix_y_below = true;
  bool all_fix_x_above = true;
  bool all_fix_y_above = true;

  int search_until = 0; // will be used in the back-up check for first order of object placement
  int nr_objects_after[nr_objects_]; //how many objects will be placed AFTER the current object

  for(int i=0; i<nr_objects_; i++)
  {
    nr_objects_after[i] = 0;
    //    ROS_INFO("_____________________________________________________");
    //    ROS_INFO("object %d :", i);
    //    ROS_INFO("relative x: %f", puzzle_target_poses_[i].position.x);
    //    ROS_INFO("relative y: %f", puzzle_target_poses_[i].position.y);
  }
  //  ROS_INFO("_____________________________________________________");

  puzzle_order_.resize(nr_objects_);
  //vector of objects --> choose an object for comparison
  for (int fix_object = 0; fix_object < nr_objects_; fix_object++)
  {

    // compare fix_object with all other objects
    for (int comp_object = 0; comp_object < nr_objects_; comp_object++)
    {
      order_found = false;

      if (fix_object < comp_object)
      {

        //vector of shapes within a specific object --> choose a shape for comparison
        for (int fix_shape = 0; fix_shape < objects_[fix_object].nr_shapes; fix_shape++)
        {
          fix_x_below = false;
          fix_y_below = false;

          // compare fix_shape (of fix_object) with all shapes of comp_object
          for (int comp_shape1 = 0; comp_shape1 < objects_[comp_object].nr_shapes; comp_shape1++)
          {
            // for fix_shape: x smaller & y the same as any shape of comp_object?
            if (int(100*(puzzle_target_poses_[fix_object].position.x + objects_[fix_object].shape[fix_shape].pose.position.x+0.000005))
                < int(100*(puzzle_target_poses_[comp_object].position.x + objects_[comp_object].shape[comp_shape1].pose.position.x+0.000005))
                && int(100*(puzzle_target_poses_[fix_object].position.y + objects_[fix_object].shape[fix_shape].pose.position.y+0.000005))
                == int(100*(puzzle_target_poses_[comp_object].position.y + objects_[comp_object].shape[comp_shape1].pose.position.y+0.000005)))
            {
              fix_x_below = true; //one of the shapes of fix_object is directly below the same shape of comp_object
              break;
            }
          }

          for (int comp_shape2 = 0; comp_shape2 < objects_[comp_object].nr_shapes; comp_shape2++)
          {
            // for fix_shape: x the same & y smaller than any shape of comp_object?
            if (int(100*(puzzle_target_poses_[fix_object].position.x + objects_[fix_object].shape[fix_shape].pose.position.x+0.000005))
                == int(100*(puzzle_target_poses_[comp_object].position.x + objects_[comp_object].shape[comp_shape2].pose.position.x+0.000005))
                && int(100*(puzzle_target_poses_[fix_object].position.y + objects_[fix_object].shape[fix_shape].pose.position.y+0.000005))
                < int(100*(puzzle_target_poses_[comp_object].position.y + objects_[comp_object].shape[comp_shape2].pose.position.y+0.000005)))
            {
              fix_y_below = true; //one of the shapes of fix_object is on the right of the same shape of comp_object
              break;
            }
          }

          if(fix_x_below && fix_y_below)
          {
            //            ROS_INFO("piece %d must be placed before piece %d",fix_object, comp_object);
            //            ROS_INFO("the latter has a shape on the left and on top of one of the shapes of the former");
            nr_objects_after[fix_object]++;
            order_found = true;
            //Now leave current for loop and proceed with comparing fix_object with a new comp_object!
            break; // --> leaving fix_shape loop
          } // try to find the above scenario for all shapes of fix object before testing for other possibilities.
        }// end fix_shape

        if (!order_found)
        {
          for (int comp_shape3 = 0; comp_shape3 < objects_[comp_object].nr_shapes; comp_shape3++)
          {
            fix_x_above = false;
            fix_y_above = false;

            for (int fix_shape1 = 0; fix_shape1 < objects_[fix_object].nr_shapes; fix_shape1++)
            {
              // if fix_object has one of its shapes above a shape of comp_object
              if (int(100*(puzzle_target_poses_[fix_object].position.x + objects_[fix_object].shape[fix_shape1].pose.position.x+0.000005))
                  > int(100*(puzzle_target_poses_[comp_object].position.x + objects_[comp_object].shape[comp_shape3].pose.position.x+0.000005))
                  && int(100*(puzzle_target_poses_[fix_object].position.y + objects_[fix_object].shape[fix_shape1].pose.position.y+0.000005))
                  == int(100*(puzzle_target_poses_[comp_object].position.y + objects_[comp_object].shape[comp_shape3].pose.position.y+0.000005)))
              {
                fix_x_above = true;
                break; // --> leave fix_shape1
              }
            }

            for (int fix_shape2 = 0; fix_shape2 < objects_[fix_object].nr_shapes; fix_shape2++)
            {
              // if there is another shape of fix_object to the left of the original fix_shape
              if (int(100*(puzzle_target_poses_[fix_object].position.x + objects_[fix_object].shape[fix_shape2].pose.position.x+0.000005))
                  == int(100*(puzzle_target_poses_[comp_object].position.x + objects_[comp_object].shape[comp_shape3].pose.position.x+0.000005))
                  && int(100*(puzzle_target_poses_[fix_object].position.y + objects_[fix_object].shape[fix_shape2].pose.position.y+0.000005))
                  > int(100*(puzzle_target_poses_[comp_object].position.y + objects_[comp_object].shape[comp_shape3].pose.position.y+0.000005)))
              {
                fix_y_above = true;
                break; // -->leave fix_shape2
              }
            }

            if(fix_x_above && fix_y_above)
            {
              //              ROS_INFO("piece %d must be placed after piece %d",fix_object, comp_object);
              //              ROS_INFO("The former has a shape on top and on the left of one of the shapes of the latter.");
              nr_objects_after[comp_object]++;
              order_found = true;
              break; // --> leaving comp_shape3
            }

          } // end comp_shape3

          if(!order_found)
          {
            all_fix_x_below = true;
            all_fix_y_below = true;
            all_fix_x_above = true;
            all_fix_y_above = true;

            for (int fix_shape4 = 0; fix_shape4 < objects_[fix_object].nr_shapes; fix_shape4++)
            {
              for (int comp_shape4 = 0; comp_shape4 < objects_[comp_object].nr_shapes; comp_shape4++)
              {
                if(int(100*(puzzle_target_poses_[fix_object].position.x + objects_[fix_object].shape[fix_shape4].pose.position.x+0.000005))
                    >= int(100*(puzzle_target_poses_[comp_object].position.x + objects_[comp_object].shape[comp_shape4].pose.position.x+0.000005)))
                {
                  all_fix_x_below = false;
                }
                // all pieces of fix_object on top of comp_object? -> if not, allfixxabove = false
                if(int(100*(puzzle_target_poses_[fix_object].position.x + objects_[fix_object].shape[fix_shape4].pose.position.x+0.000005))
                    <= int(100*(puzzle_target_poses_[comp_object].position.x + objects_[comp_object].shape[comp_shape4].pose.position.x+0.000005)))
                {
                  all_fix_x_above = false;
                }

                if(int(100*(puzzle_target_poses_[fix_object].position.y + objects_[fix_object].shape[fix_shape4].pose.position.y+0.000005))
                    >= int(100*(puzzle_target_poses_[comp_object].position.y + objects_[comp_object].shape[comp_shape4].pose.position.y+0.000005)))
                {
                  all_fix_y_below = false;
                }
                // all pieces of fix_object on the left of comp_object? -> if not, allfixyabove = false
                if(int(100*(puzzle_target_poses_[fix_object].position.y + objects_[fix_object].shape[fix_shape4].pose.position.y+0.000005))
                    <= int(100*(puzzle_target_poses_[comp_object].position.y + objects_[comp_object].shape[comp_shape4].pose.position.y+0.000005)))
                {
                  all_fix_y_above = false;
                }
              }
            }

            if(all_fix_x_below)
            {
              bool x_below = false;
              for(int fix_shape5 = 0; fix_shape5 < objects_[fix_object].nr_shapes; fix_shape5++)
              {
                for(int comp_shape5 = 0; comp_shape5 < objects_[comp_object].nr_shapes; comp_shape5++)
                {
                  if(int(100*(puzzle_target_poses_[fix_object].position.y + objects_[fix_object].shape[fix_shape5].pose.position.y+0.000005))
                      == int(100*(puzzle_target_poses_[comp_object].position.y + objects_[comp_object].shape[comp_shape5].pose.position.y+0.000005)))
                  {
                    x_below = true;
                  }
                }
              }
              if(x_below)
              {
                //                ROS_INFO("piece %d must be placed before piece %d",fix_object, comp_object);
                //                ROS_INFO("the latter has all its shapes above the former");
                nr_objects_after[fix_object]++;
                order_found = true;
              }
            }
            else if(all_fix_x_above)
            {
              bool x_above = false;
              for(int fix_shape5 = 0; fix_shape5 < objects_[fix_object].nr_shapes; fix_shape5++)
              {
                for(int comp_shape5 = 0; comp_shape5 < objects_[comp_object].nr_shapes; comp_shape5++)
                {
                  if(int(100*(puzzle_target_poses_[fix_object].position.y + objects_[fix_object].shape[fix_shape5].pose.position.y+0.000005))
                      == int(100*(puzzle_target_poses_[comp_object].position.y + objects_[comp_object].shape[comp_shape5].pose.position.y+0.000005)))
                  {
                    x_above = true;
                  }
                }
              }
              if(x_above)
              {
                //                ROS_INFO("piece %d must be placed after piece %d",fix_object, comp_object);
                //                ROS_INFO("the former has all its shapes above the latter");
                nr_objects_after[comp_object]++;
                order_found = true;
              }
            }
            else if(all_fix_y_below)
            {
              bool y_below = false;
              for(int fix_shape5 = 0; fix_shape5 < objects_[fix_object].nr_shapes; fix_shape5++)
              {
                for(int comp_shape5 = 0; comp_shape5 < objects_[comp_object].nr_shapes; comp_shape5++)
                {
                  if(int(100*(puzzle_target_poses_[fix_object].position.x + objects_[fix_object].shape[fix_shape5].pose.position.x+0.000005))
                      == int(100*(puzzle_target_poses_[comp_object].position.x + objects_[comp_object].shape[comp_shape5].pose.position.x+0.000005)))
                  {
                    y_below = true;
                  }
                }
              }
              if(y_below)
              {
                //                ROS_INFO("piece %d must be placed before piece %d",fix_object, comp_object);
                //                ROS_INFO("the latter has all its shapes on the left of the former");
                nr_objects_after[fix_object]++;
                order_found = true;
              }
            }
            else if(all_fix_y_above)
            {
              bool y_above = false;
              for(int fix_shape5 = 0; fix_shape5 < objects_[fix_object].nr_shapes; fix_shape5++)
              {
                for(int comp_shape5 = 0; comp_shape5 < objects_[comp_object].nr_shapes; comp_shape5++)
                {
                  if(int(100*(puzzle_target_poses_[fix_object].position.x + objects_[fix_object].shape[fix_shape5].pose.position.x+0.000005))
                      == int(100*(puzzle_target_poses_[comp_object].position.x + objects_[comp_object].shape[comp_shape5].pose.position.x+0.000005)))
                  {
                    y_above = true;
                  }
                }
              }
              if(y_above)
              {
                //                ROS_INFO("piece %d must be placed after piece %d",fix_object, comp_object);
                //                ROS_INFO("the former has all its shapes on the left of the latter");
                nr_objects_after[comp_object]++;
                order_found = true;
              }
            }


            if (!order_found)
            {
              if (sqrt(puzzle_target_poses_[fix_object].position.x * puzzle_target_poses_[fix_object].position.x
                       + puzzle_target_poses_[fix_object].position.y * puzzle_target_poses_[fix_object].position.y)
                       < sqrt(puzzle_target_poses_[comp_object].position.x * puzzle_target_poses_[comp_object].position.x
                              + puzzle_target_poses_[comp_object].position.y * puzzle_target_poses_[comp_object].position.y))
              {
                //                ROS_INFO("piece %d must be placed before piece %d",fix_object, comp_object);
                //                ROS_INFO("object %d is closer to the fixture origin than object %d", fix_object, comp_object);
                nr_objects_after[fix_object]++;
              }
              else if (sqrt(puzzle_target_poses_[fix_object].position.x * puzzle_target_poses_[fix_object].position.x
                            + puzzle_target_poses_[fix_object].position.y * puzzle_target_poses_[fix_object].position.y)
                            > sqrt(puzzle_target_poses_[comp_object].position.x * puzzle_target_poses_[comp_object].position.x
                                   + puzzle_target_poses_[comp_object].position.y * puzzle_target_poses_[comp_object].position.y))
              {
                //                ROS_INFO("object %d is farther away from the fixture origin than object %d.", fix_object, comp_object);
                //                ROS_INFO("piece %d must be placed after piece %d",fix_object, comp_object);
                nr_objects_after[comp_object]++;
              }
              else
              {
                // if nothing else remains, simply place first the object with a smaller number in its name
                if(fix_object < comp_object)
                {
                  nr_objects_after[fix_object]++;
                  //                  ROS_INFO("piece %d must be placed before piece %d",fix_object, comp_object);
                }
                else
                {
                  nr_objects_after[comp_object]++;
                  //                  ROS_INFO("piece %d must be placed after piece %d",fix_object, comp_object);
                }
              }
            }
          } // end 2nd if(!order_found)
        } // end 1st if(!order_found)

      } // end if fix_object != comp_object
    } // end comp_object


    // construct the first version of the order in which the objects should be placed
    puzzle_order_[(nr_objects_ - 1) - nr_objects_after[fix_object]].part_index = fix_object;

  }// end fix_object

  //----------------------------------------------------------------------------------
  //go through first version of object order and check for blocking

  bool any_fix_x_below = false;
  bool any_fix_x_above = false;
  bool any_organised_object_y_above_fix = false;
  bool any_organised_object_y_above_comp = false;


  for (int fix_object = 0; fix_object < nr_objects_; fix_object++)
  {
    for (int comp_object = 0; comp_object < nr_objects_; comp_object++)
    {
      any_fix_x_below = false;
      any_fix_x_above = false;
      any_organised_object_y_above_fix = false;
      any_organised_object_y_above_comp = false;

      if (fix_object < comp_object)
      {

        for (int fix_shape = 0; fix_shape < objects_[fix_object].nr_shapes; fix_shape++)
        {
          for (int comp_shape1 = 0; comp_shape1 < objects_[comp_object].nr_shapes; comp_shape1++)
          {
            // for fix_shape: x smaller & y the same as any shape of comp_object?
            if (int(100*(puzzle_target_poses_[fix_object].position.x + objects_[fix_object].shape[fix_shape].pose.position.x+0.000005))
                < int(100*(puzzle_target_poses_[comp_object].position.x + objects_[comp_object].shape[comp_shape1].pose.position.x+0.000005))
                && int(100*(puzzle_target_poses_[fix_object].position.y + objects_[fix_object].shape[fix_shape].pose.position.y+0.000005))
                == int(100*(puzzle_target_poses_[comp_object].position.y + objects_[comp_object].shape[comp_shape1].pose.position.y+0.000005)))
            {
              any_fix_x_below = true; // if fix_object has any of its shapes under a shape of comp_object
            }
          }
        }

        for (int comp_shape3 = 0; comp_shape3 < objects_[comp_object].nr_shapes; comp_shape3++)
        {
          for (int fix_shape1 = 0; fix_shape1 < objects_[fix_object].nr_shapes; fix_shape1++)
          {
            // if fix_object has one of its shapes above a shape of comp_object
            if (int(100*(puzzle_target_poses_[fix_object].position.x + objects_[fix_object].shape[fix_shape1].pose.position.x+0.000005))
                > int(100*(puzzle_target_poses_[comp_object].position.x + objects_[comp_object].shape[comp_shape3].pose.position.x+0.000005))
                && int(100*(puzzle_target_poses_[fix_object].position.y + objects_[fix_object].shape[fix_shape1].pose.position.y+0.000005))
                == int(100*(puzzle_target_poses_[comp_object].position.y + objects_[comp_object].shape[comp_shape3].pose.position.y+0.000005)))
            {
              any_fix_x_above = true;
            }
          }
        }

        for (int i = 0; i < nr_objects_; i++)
        {
          if (puzzle_order_[i].part_index == fix_object)
          {
            search_until = i;
          }
        }

        for(int organised_object = 0; organised_object < search_until; organised_object++)
        {
          if (puzzle_order_[organised_object].part_index != comp_object)
          {
            for(int fix_shape4 = 0; fix_shape4 < objects_[organised_object].nr_shapes; fix_shape4++)
            {
              for(int fix_shape3 = 0; fix_shape3 < objects_[fix_object].nr_shapes; fix_shape3++)
              {
                if(int(100*(puzzle_target_poses_[fix_object].position.x + objects_[fix_object].shape[fix_shape3].pose.position.x+0.000005))
                    == int(100*(puzzle_target_poses_[puzzle_order_[organised_object].part_index].position.x + objects_[puzzle_order_[organised_object].part_index].shape[fix_shape4].pose.position.x+0.000005))
                    && int(100*(puzzle_target_poses_[fix_object].position.y + objects_[fix_object].shape[fix_shape3].pose.position.y+0.000005))
                    < int(100*(puzzle_target_poses_[puzzle_order_[organised_object].part_index].position.y + objects_[puzzle_order_[organised_object].part_index].shape[fix_shape4].pose.position.y+0.000005)))
                {
                  any_organised_object_y_above_fix = true;
                }
              }
              for(int comp_shape4 = 0; comp_shape4 < objects_[comp_object].nr_shapes; comp_shape4++)
              {
                //                ROS_INFO("org_object %d, comp_object %d. fix_object %d ", puzzle_order_[organised_object].part_index, comp_object, fix_object);
                if(int(100*(puzzle_target_poses_[comp_object].position.x + objects_[comp_object].shape[comp_shape4].pose.position.x+0.000005))
                    == int(100*(puzzle_target_poses_[puzzle_order_[organised_object].part_index].position.x + objects_[puzzle_order_[organised_object].part_index].shape[fix_shape4].pose.position.x+0.000005))
                    && int(100*(puzzle_target_poses_[comp_object].position.y + objects_[comp_object].shape[comp_shape4].pose.position.y+0.000005))
                    < int(100*(puzzle_target_poses_[puzzle_order_[organised_object].part_index].position.y + objects_[puzzle_order_[organised_object].part_index].shape[fix_shape4].pose.position.y+0.000005)))
                {
                  any_organised_object_y_above_comp = true;
                }
              }
            }
            if (any_organised_object_y_above_fix && any_fix_x_below)
            {
              //              ROS_INFO("piece %d must be placed before piece %d",fix_object, comp_object);
              //              ROS_INFO("the former is blocked by an organised object %d and the latter", puzzle_order_[organised_object].part_index);
              nr_objects_after[fix_object]++;
              nr_objects_after[comp_object]--;
            }
            else if(any_organised_object_y_above_comp && any_fix_x_above)
            {
              //              ROS_INFO("piece %d must be placed after piece %d",fix_object, comp_object);
              //              ROS_INFO("the latter is blocked by an organised object %d and the former", puzzle_order_[organised_object].part_index);
              nr_objects_after[fix_object]--;
              nr_objects_after[comp_object]++;
            }
          }
        }
      }
    }

    // construct the second version of the order in which the objects should be placed
    //    ROS_INFO("%d objects have to be placed after object %d", nr_objects_after[fix_object], fix_object);
    puzzle_order_[(nr_objects_ - 1) - nr_objects_after[fix_object]].part_index = fix_object;

  }// end fix_object

  //----------------------------------------------------------------------------------

  ROS_INFO("The right order:");
  for(int k=0; k < nr_objects_; k++)
  {
    try
    {
      ROS_INFO("piece %d", puzzle_order_[k].part_index);
    }
    catch (...)
    {
      ROS_ERROR("Failed to print out right order");
    }
  }

  //---------------------------------------------------------------------------------

  // The manner in which the objects need to be placed: (fist in x then in y or vice versa?)
  // if object touches x axis but not y axis --> first move in y
  // if order doesn't matter --> first in x

  bool only_x = true;
  bool only_y = true;
  bool no_blocks_x = true;
  bool no_blocks_y = true;

  //  // look at first object --> moving it against fixture wall in one direction first
  //  for (int shape = 0; shape < objects_[puzzle_order_[0].part_index].nr_shapes; shape++)
  //  {
  //    // if object has a shape at the x axis --> doesn't only touch y axis
  //    if ((puzzle_target_poses_[puzzle_order_[0].part_index].position.x + objects_[puzzle_order_[0].part_index].shape[shape].pose.position.x) < 0.03)
  //    {
  //      only_y = false;
  //    }
  //    // if object has a shape at the y axis --> doesn't only touch x axis
  //    if ((puzzle_target_poses_[puzzle_order_[0].part_index].position.y + objects_[puzzle_order_[0].part_index].shape[shape].pose.position.y) < 0.03)
  //    {
  //      only_x = false;
  //    }
  //  }
  //  if (only_x && !only_y) // only touches x axis -->first move in y
  //  {
  //    puzzle_order_[0].x_first = false;
  //  }
  //  else
  //  {
  //    puzzle_order_[0].x_first = true;
  //  }

  for (int fix_object = 0; fix_object < nr_objects_; fix_object++)
  {
    only_x = true;
    only_y = true;

    for (int shape = 0; shape < objects_[puzzle_order_[fix_object].part_index].nr_shapes; shape++)
    {
      // if object has a shape at the y axis --> doesn't only touch x axis
      if ((puzzle_target_poses_[puzzle_order_[fix_object].part_index].position.x + objects_[puzzle_order_[fix_object].part_index].shape[shape].pose.position.x) < 0.03)
      {
        ROS_INFO("Object %d touches y axis", puzzle_order_[fix_object].part_index);
        only_x = false;
      }
      // if object has a shape at the x axis --> doesn't only touch y axis
      if ((puzzle_target_poses_[puzzle_order_[fix_object].part_index].position.y + objects_[puzzle_order_[fix_object].part_index].shape[shape].pose.position.y) < 0.03)
      {
        ROS_INFO("Object %d touches x axis", puzzle_order_[fix_object].part_index);
        only_y = false;
      }
    }
    if (only_x && !only_y) // only touches x axis -->first move in y
    {
      ROS_INFO("Object %d only touches x axis", puzzle_order_[fix_object].part_index);
      puzzle_order_[fix_object].x_first = false;
    }
    else if ((!only_x && only_y) || (!only_x && !only_y)) //only touches y axis OR touches both
    {
      ROS_INFO("Object %d touches y and x axis or only y", puzzle_order_[fix_object].part_index);
      puzzle_order_[fix_object].x_first = true;
    }
    else
    {
      ROS_INFO("Object %d does not touch any axis", puzzle_order_[fix_object].part_index);
      bool no_blocks_x = true; // no blocks x = other objects don't have shapes with greater x value and equal y value
      bool no_blocks_y = true;

      for (int i = 0; i < nr_objects_; i++)
      {
        // search where in puzzle_order_ fix_object is located
        // --> only the objects placed before fix_object matter when placing it
        if (puzzle_order_[i].part_index == puzzle_order_[fix_object].part_index)
        {
          search_until = i;
        }
      }

      for(int organised_object = 0; organised_object < search_until; organised_object++)
      {
        if (puzzle_order_[organised_object].part_index != puzzle_order_[fix_object].part_index)
        {
          for (int fix_shape = 0; fix_shape < objects_[puzzle_order_[fix_object].part_index].nr_shapes; fix_shape++)
          {
            for (int organised_shape = 0; organised_shape < objects_[puzzle_order_[organised_object].part_index].nr_shapes; organised_shape++)
            {
              if(int(100*(puzzle_target_poses_[puzzle_order_[fix_object].part_index].position.x + objects_[puzzle_order_[fix_object].part_index].shape[fix_shape].pose.position.x+0.000005))
                  < int(100*(puzzle_target_poses_[puzzle_order_[organised_object].part_index].position.x + objects_[puzzle_order_[organised_object].part_index].shape[organised_shape].pose.position.x+0.000005))
                  && int(100*(puzzle_target_poses_[puzzle_order_[fix_object].part_index].position.y + objects_[puzzle_order_[fix_object].part_index].shape[fix_shape].pose.position.y+0.000005))
                  == int(100*(puzzle_target_poses_[puzzle_order_[organised_object].part_index].position.y + objects_[puzzle_order_[organised_object].part_index].shape[organised_shape].pose.position.y+0.000005)))
              {
                no_blocks_x = false;
              }
              //          ROS_INFO("fix %d, organised %d: ", puzzle_order_[fix_object].part_index, puzzle_order_[organised_object].part_index);
              //          ROS_INFO("x: %d & %d: ", int(100*(puzzle_target_poses_[puzzle_order_[fix_object].part_index].position.x + objects_[puzzle_order_[fix_object].part_index].shape[fix_shape].pose.position.x+0.000005)), int(100*(puzzle_target_poses_[puzzle_order_[organised_object].part_index].position.x + objects_[puzzle_order_[organised_object].part_index].shape[organised_shape].pose.position.x+0.000005)));
              //          ROS_INFO("y: %d & %d: ", int(100*(puzzle_target_poses_[puzzle_order_[fix_object].part_index].position.y + objects_[puzzle_order_[fix_object].part_index].shape[fix_shape].pose.position.y+0.000005)), int(100*(puzzle_target_poses_[puzzle_order_[organised_object].part_index].position.y + objects_[puzzle_order_[organised_object].part_index].shape[organised_shape].pose.position.y+0.000005)));
              if(int(100*(puzzle_target_poses_[puzzle_order_[fix_object].part_index].position.x + objects_[puzzle_order_[fix_object].part_index].shape[fix_shape].pose.position.x+0.000005))
                  == int(100*(puzzle_target_poses_[puzzle_order_[organised_object].part_index].position.x + objects_[puzzle_order_[organised_object].part_index].shape[organised_shape].pose.position.x+0.000005))
                  && int(100*(puzzle_target_poses_[puzzle_order_[fix_object].part_index].position.y + objects_[puzzle_order_[fix_object].part_index].shape[fix_shape].pose.position.y+0.000005))
                  < int(100*(puzzle_target_poses_[puzzle_order_[organised_object].part_index].position.y + objects_[puzzle_order_[organised_object].part_index].shape[organised_shape].pose.position.y+0.000005)))
              {
                //              ROS_INFO("fix %d, organised %d, !no_blocks_y", puzzle_order_[fix_object].part_index, puzzle_order_[organised_object].part_index);
                no_blocks_y = false;
              }
            }
          }
        }
      }

      if (no_blocks_y && !no_blocks_x)
      {
        //      ROS_INFO("fix_object %d no_blocks_y && !no_blocks_x", puzzle_order_[fix_object].part_index);
        puzzle_order_[fix_object].x_first = false;
      }
      else if (!no_blocks_y && !no_blocks_x)
      {
        ROS_INFO("fix_object %d THIS IS AN ERROR", puzzle_order_[fix_object].part_index);
      }
      else
      {
        puzzle_order_[fix_object].x_first = true;
      }
    }
  }

  //---------------------------------------------------------------------------------

  for(int i = 0; i < nr_objects_; i++)
  {
    if(puzzle_order_[i].x_first)
    {
      ROS_INFO("push piece %d first in x direction", puzzle_order_[i].part_index);
    }
    else
    {
      ROS_INFO("push piece %d first in y direction", puzzle_order_[i].part_index);
    }
  }
}
