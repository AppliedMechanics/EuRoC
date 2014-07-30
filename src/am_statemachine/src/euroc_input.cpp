#include <euroc_input.hpp>
#include <tf_rot.hpp>
#include <iostream>

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

	const YAML::Node *objects = task_description_node.FindValue("objects");
	if(!objects){
	    ROS_ERROR("Target zones not found in task description");
	    return -1;
	  }

	unsigned int nr_objects = objects->size();
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

	    const YAML::Node* col = obj->FindValue("color");
	    if(!col){
	    	ROS_ERROR("Color not found in object");
		    return -1;
	    }
	    (*col) >> tmp_obj.color;
	    std::cout<<"color: "<<tmp_obj.color<<std::endl;


	    const YAML::Node* des = obj->FindValue("description");
	    if(!des){
			ROS_ERROR("description not found in object");
		    return -1;
		}
		*des >> tmp_obj.description;
		std::cout<<"description:"<<tmp_obj.description<<"\n";


		const YAML::Node* sur = obj->FindValue("surface_material");
		if(!des){
			ROS_ERROR("surface material not found in object");
		    return -1;
		}
		*sur >> tmp_obj.surface_material;
		std::cout<<"surface_material:"<<tmp_obj.surface_material<<"\n";


	    const YAML::Node* shapes = obj->FindValue("shape");
	    if(!shapes){
	      ROS_ERROR("Shape not found in object");
	      return -1;
	    }
		unsigned int nr_shapes = shapes->size();
		unsigned int jj = 0;
		tmp_obj.shape.resize(nr_shapes);
		std::cout << "number of shapes:" << nr_shapes << std::endl;
		for (YAML::Iterator it2 = shapes->begin(); it2 != shapes->end();++it2)
		{
			try{
				const YAML::Node* type = (*it2).FindValue("type");
				*type >> tmp_obj.shape[jj].type;
				std::cout<<"test "<<tmp_obj.shape[jj].type<<std::endl;
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
					tmp_obj.shape[jj].size.push_back(-1);
				} catch(YAML::Exception e) {
					ROS_ERROR("dimensions not found in shape");
					return -1;
				}
				std::cout<<"Ist ein Cylinder mit l="<<tmp_obj.shape[jj].length
						 <<" und r= "<<tmp_obj.shape[jj].radius<<" !!!!"<<std::endl;
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
				std::cout<<"Ist eine Box mit Groesse: ["<<tmp_obj.shape[jj].size[0]<<" "<<tmp_obj.shape[jj].size[0]<<" "
						<<tmp_obj.shape[jj].size[0]<<"]"<<std::endl;
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
			(*speed_limit)[ii] >> robot_.speed_limit[ii];
		}
	} catch (YAML::Exception e) {
		ROS_ERROR("EurocInput: YAML Error in robot speed limit");
		return -1;
	}

	const YAML::Node* grip_lim = robot->FindValue("gripper_speed_limit");
	try {
			(*speed_limit) >> robot_.gripper_speed_limit;
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
		ROS_ERROR("EurocInput: YAML Error in robot speed limit");
		return -1;
	}

	return 0;
}
