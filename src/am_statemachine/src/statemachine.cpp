#include "statemachine.hpp"


Statemachine::Statemachine()
{
  task_selector("/euroc_c2_task_selector");
  list_scenes = (task_selector + "/list_scenes");
  start_simulator= (task_selector + "/start_simulator");
  stop_simulator= (task_selector + "/stop_simulator");
  ros::service::waitForService(list_scenes);
  ros::service::waitForService(start_simulator);
  ros::service::waitForService(stop_simulator);
  list_scenes_client = node.serviceClient<euroc_c2_msgs::ListScenes>(list_scenes);
  start_simulator_client = node.serviceClient<euroc_c2_msgs::StartSimulator>(start_simulator);
  stop_simulator_client = node.serviceClient<euroc_c2_msgs::StopSimulator>(stop_simulator);
}

int Statemachine::parse_yaml_file()
{
	// The start simulator service returns a description of the selected task in yaml format
	std::string &task_yaml_description = start_simulator_srv.response.description_yaml;

	// Parse the explanation/description of the task from the yaml string
	std::stringstream yaml_stream(task_yaml_description);
	YAML::Parser parser(yaml_stream);
	YAML::Node task_description_node;
	parser.GetNextDocument(task_description_node);
	std::string task_description;
	task_description_node["description"] >> task_description;

	return 0;
}
