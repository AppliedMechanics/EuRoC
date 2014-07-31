#include <statemachine.hpp>

Statemachine* Statemachine::instance_ = 0x0;

Statemachine::Statemachine():
	    scenes_(1),
		task_active_(false),
		sim_running_(false),
		nr_scenes_(0),
		active_scene_(-1)
{
	ein_=new EurocInput();
	//==============================================
	//state:
	state_.sub.one = REQUEST_TASK;
	//==============================================

	task_selector_ = "/euroc_c2_task_selector";
	list_scenes_ = (task_selector_ + "/list_scenes");
	start_simulator_ = (task_selector_ + "/start_simulator");
	stop_simulator_ = (task_selector_ + "/stop_simulator");
	euroc_c2_interface_ = "/euroc_interface_node";
	save_log_ = euroc_c2_interface_ + "/save_log";

}

Statemachine::~Statemachine()
{
	if(sim_running_)
	{
		stop_sim();
	}
}

int Statemachine::init_sm()
{
	ros::service::waitForService(list_scenes_,ros::Duration(5.0));
	ros::service::waitForService(start_simulator_,ros::Duration(5.0));
	ros::service::waitForService(stop_simulator_,ros::Duration(5.0));

	list_scenes_client_ = node_.serviceClient<euroc_c2_msgs::ListScenes>(list_scenes_);
	start_simulator_client_ = node_.serviceClient<euroc_c2_msgs::StartSimulator>(start_simulator_);
	stop_simulator_client_ = node_.serviceClient<euroc_c2_msgs::StopSimulator>(stop_simulator_);
	save_log_client_ = node_.serviceClient<euroc_c2_msgs::SaveLog>(save_log_);

	get_grasp_pose_client_ = node_.serviceClient<am_msgs::GetGraspPose>("get_grasp_pose");

	return 0;
}

int Statemachine::tick()
{

	switch(state_.sub.one)
	{
	case REQUEST_TASK:
		return request_task();
	case START_SIM:
		return start_sim();
	case PARSE_YAML:
		return parse_yaml_file();
	case SOLVE_TASK:
		switch(state_.sub.two)
		{
		case LOCATE_OBJECT:
			return locate_object();
		case GET_GRASPING_POSE:
			return get_grasping_pose();
		case MOVE_TO_OBJECT:
		case MOVE_TO_TARGET_ZONE:
		default:
			return solve_task();
		}
	case STOP_SIM:
		return stop_sim();
	default:
		msg_error("Unknown STATE!!!");
		return -1;
	}
}

int Statemachine::request_task()
{
	list_scenes_client_.call(list_scenes_srv_);

	// The error_message field of each service response indicates whether an error occured. An empty string indicates success
	std::string &ls_error_message = list_scenes_srv_.response.error_message;
	if(!ls_error_message.empty()){
		ROS_ERROR("List scenes failed: %s", ls_error_message.c_str());
		return -1;
	}
	else
	{
		// Let's print the names of the received scenes
		ROS_INFO("[AM] Found the following scenes for the EuRoC C2 Simulation:");

		scenes_.resize(list_scenes_srv_.response.scenes.size());
		scenes_ = list_scenes_srv_.response.scenes;

		nr_scenes_ = scenes_.size();
		for(unsigned int ii = 0; ii < nr_scenes_; ++ii){
			euroc_c2_msgs::Scene &scene = scenes_[ii];
			ROS_INFO("[%2d] - %s", ii,scene.name.c_str());
		}
	}

	std::cout<<"Choose task: ";
	int blub;
	std::cin>>blub;
	active_scene_=blub;

	ROS_INFO("Your Choice: %d", active_scene_);
	task_active_=true;

	//==============================================
	//state:
	state_.sub.one = START_SIM;
	//==============================================
	return 0;
}

int Statemachine::start_sim()
{
	ROS_INFO("Statemachine: Starting Server");

	start_simulator_srv_.request.user_id = "am-robotics";
	start_simulator_srv_.request.scene_name = scenes_[active_scene_].name;
	start_simulator_client_.call(start_simulator_srv_);

	// Check the response for errors
	std::string &error_message = start_simulator_srv_.response.error_message;
	if(!error_message.empty()){
		ROS_ERROR("Statemachine: Starting the simulator failed: %s", error_message.c_str());
		return -1;
	}

	ros::service::waitForService(save_log_,ros::Duration(5.0));

	//==============================================
	//state:
	state_.sub.one = PARSE_YAML;
	//==============================================

	return 0;
}

int Statemachine::parse_yaml_file()
{
	ROS_INFO("Statemachine: Parsing YAML-file");

	// The start simulator service returns a description of the selected task in yaml format
	std::string &task_yaml_description = start_simulator_srv_.response.description_yaml;


	if(-1==ein_->parse_yaml_file(task_yaml_description))
	{
		ROS_ERROR("Statemachine: Error in parse_yaml_file()");
	}


	//==============================================
	//state:
	state_.sub.one = SOLVE_TASK;
	state_.sub.two = LOCATE_OBJECT;
	//==============================================

	return 0;
}

int Statemachine::solve_task()
{
	//blub

	//==============================================
	//state:
	state_.sub.one = STOP_SIM;
	//==============================================
	return 0;
}

int Statemachine::stop_sim()
{
	ROS_INFO("Statemachine: Saving log");
	save_log_client_.call(save_log_srv_);

	ROS_INFO("Statemachine: Stopping server");

	// The stop simulator callback ends the current simulation
	stop_simulator_client_.call(stop_simulator_srv_);


	//==============================================
	//state:
	state_.sub.one = FINISHED;
	//==============================================

	return 0;
}

int Statemachine::locate_object()
{
	ein_->get_object(&cur_obj_);

	//hier dann vision aufrufen
	cur_obj_.abs_pose.position.x=0.3;
	cur_obj_.abs_pose.position.y=0.3;
	cur_obj_.abs_pose.position.z=0.1;

	//==============================================
	//state:
	state_.sub.one = SOLVE_TASK;
	state_.sub.two = GET_GRASPING_POSE;
	//==============================================

	return 0;
}

int Statemachine::get_grasping_pose()
{
	get_grasp_pose_srv_.request.object=cur_obj_;

	if(get_grasp_pose_client_.call(get_grasp_pose_srv_))
	{
		geometry_msgs::Pose *pose= &get_grasp_pose_srv_.response.grasp_pose;
		ROS_INFO("received pose: [%3.2f %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f]",
				pose->position.x,pose->position.y,pose->position.z,pose->orientation.w,
				pose->orientation.x,pose->orientation.y,pose->orientation.z);
	}
	else
	{
		msg_error("Failed to call get_grasp_pose service.");
	}

	//==============================================
	//state:
	state_.sub.one = SOLVE_TASK;
	state_.sub.two = MOVE_TO_OBJECT;
	//==============================================
	return 0;
}

int Statemachine::move_to_object()
{

	//==============================================
	//state:
	state_.sub.one = SOLVE_TASK;
	state_.sub.two = MOVE_TO_TARGET_ZONE;
	//==============================================
	return 0;
}
