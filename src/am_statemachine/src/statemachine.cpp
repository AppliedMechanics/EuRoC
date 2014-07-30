#include <statemachine.hpp>

Statemachine* Statemachine::instance = 0x0;

Statemachine::Statemachine()
{
	ein=new EurocInput();

	task_selector = "/euroc_c2_task_selector";
	list_scenes = (task_selector + "/list_scenes");
	start_simulator= (task_selector + "/start_simulator");
	stop_simulator= (task_selector + "/stop_simulator");
	euroc_c2_interface = "/euroc_interface_node";
	save_log = euroc_c2_interface + "/save_log";

	ros::service::waitForService(list_scenes,ros::Duration(3.0));
	ros::service::waitForService(start_simulator,ros::Duration(3.0));
	ros::service::waitForService(stop_simulator,ros::Duration(3.0));

	list_scenes_client = node.serviceClient<euroc_c2_msgs::ListScenes>(list_scenes);
	start_simulator_client = node.serviceClient<euroc_c2_msgs::StartSimulator>(start_simulator);
	stop_simulator_client = node.serviceClient<euroc_c2_msgs::StopSimulator>(stop_simulator);
	save_log_client = node.serviceClient<euroc_c2_msgs::SaveLog>(save_log);
}

Statemachine::~Statemachine()
{
	stop_sim();
}

int Statemachine::parse_yaml_file()
{
	// The start simulator service returns a description of the selected task in yaml format
	std::string &task_yaml_description = start_simulator_srv.response.description_yaml;


	if(-1==ein->parse_yaml_file(task_yaml_description))
	{
		ROS_ERROR("Statemachine: Error in parse_yaml_file()");
	}

	return 0;
}

int Statemachine::start_sim()
{
	start_simulator_srv.request.user_id = "am-robotics";
	start_simulator_srv.request.scene_name = "task2_v1_1";
	start_simulator_client.call(start_simulator_srv);

	// Check the response for errors
	std::string &error_message = start_simulator_srv.response.error_message;
	if(!error_message.empty()){
		ROS_ERROR("Statemachine: Starting the simulator failed: %s", error_message.c_str());
		return -1;
	}

	ros::service::waitForService(save_log,ros::Duration(20.0));

	return 0;
}

int Statemachine::stop_sim()
{
	ROS_INFO("Statemachine: Saving log");
	save_log_client.call(save_log_srv);

	ROS_INFO("Statemachine: Stopping server");

	// The stop simulator callback ends the current simulation
	euroc_c2_msgs::StopSimulator stop_simulator_srv;
	stop_simulator_client.call(stop_simulator_srv);

	return 0;
}
