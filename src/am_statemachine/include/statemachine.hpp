#ifndef __STATEMACHINE_HPP__
#define __STATEMACHINE_HPP__

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

//euroc includes
#include <euroc_c2_msgs/SaveLog.h>
#include <euroc_c2_msgs/ListScenes.h>
#include <euroc_c2_msgs/StartSimulator.h>
#include <euroc_c2_msgs/StopSimulator.h>

#include <euroc_input.hpp>

class Statemachine
{
private:
	Statemachine();
	//!private copy contructor ...
	Statemachine(const Statemachine&);
	~Statemachine();

	//!statemachine instance (singleton)
	static Statemachine* instance;

	//!input container class
	EurocInput *ein;

	//Get the node handle for this node
	ros::NodeHandle node;

	std::string task_selector;
	std::string list_scenes;
	std::string start_simulator;
	std::string stop_simulator;
	std::string euroc_c2_interface;
	std::string save_log;
	ros::ServiceClient list_scenes_client;
	ros::ServiceClient start_simulator_client;
	ros::ServiceClient stop_simulator_client;
	ros::ServiceClient save_log_client;

	euroc_c2_msgs::StartSimulator start_simulator_srv;
	euroc_c2_msgs::SaveLog save_log_srv;
public:
	//!singleton implementation
	static Statemachine* get_instance();

	int handle();

	int parse_yaml_file();
	int start_sim();
	int stop_sim();
};

inline Statemachine* Statemachine::get_instance()
{
	if(instance==NULL)
	{
		instance=new Statemachine();
	}
	return instance;
};

#endif //__STATEMACHINE_HPP__
