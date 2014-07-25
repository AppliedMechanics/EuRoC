#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

// Includes for parsing yaml data
#include <yaml-cpp/yaml.h>

#include <euroc_c2_msgs/SaveLog.h>

class Statemachine
{
private:
	Statemachine();
	Statemachine(const Statemachine&);
	~Statemachine();

	static Statemachine* statemachine_m;

	//Get the node handle for this node
	ros::NodeHandle node;

	const std::string task_selector;
	const std::string list_scenes;
	const std::string start_simulator;
	const std::string stop_simulator;
	ros::ServiceClient list_scenes_client;
	ros::ServiceClient start_simulator_client;
	ros::ServiceClient stop_simulator_client;

	euroc_c2_msgs::StartSimulator start_simulator_srv;
public:
	static Statemachine* getInstance();

	int handle();

	int parse_yaml_file();
};

inline static Statemachine* Statemachine::getInstance()
{
	if(statemachine_m==NULL)
	{
		statemachine_m=new Statemachine();
	}
	return statemachine_m;
}
