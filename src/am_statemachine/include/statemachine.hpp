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

#include <am_msgs/Object.h>
#include <am_msgs/TargetZone.h>
#include <am_msgs/GetGraspPose.h>

#include <utils.hpp>
#include <fsm_state.hpp>
#include <euroc_input.hpp>

class Statemachine
{

private:
	Statemachine();
	//!private copy contructor ...
	Statemachine(const Statemachine&);
	~Statemachine();

	//!statemachine instance (singleton)
	static Statemachine* instance_;

	//!input container class
	EurocInput *ein_;

	//!state variable
	fsm_state_t state_;

	//Get the node handle for this node
	ros::NodeHandle node_;

	std::string task_selector_;
	std::string list_scenes_;
	std::string start_simulator_;
	std::string stop_simulator_;
	std::string euroc_c2_interface_;
	std::string save_log_;
	ros::ServiceClient list_scenes_client_;
	ros::ServiceClient start_simulator_client_;
	ros::ServiceClient stop_simulator_client_;
	ros::ServiceClient save_log_client_;

	ros::ServiceClient get_grasp_pose_client_;
	am_msgs::GetGraspPose get_grasp_pose_srv_;

	euroc_c2_msgs::StartSimulator start_simulator_srv_;
	euroc_c2_msgs::StopSimulator stop_simulator_srv_;
	euroc_c2_msgs::SaveLog save_log_srv_;
	euroc_c2_msgs::ListScenes list_scenes_srv_;
	std::vector<euroc_c2_msgs::Scene> scenes_;


	bool task_active_;
	bool sim_running_;
	uint8_t nr_scenes_;
	int8_t active_scene_;

	uint64_t counter;

	am_msgs::Object cur_obj_;
	uint8_t cur_obj_index;
	am_msgs::TargetZone cur_zone_;

public:
	//!singleton implementation
	static Statemachine* get_instance();

	int init_sm();
	int tick();

	fsm_state_t get_state()	{ return state_; };

private:
	int request_task();
	int start_sim();
	int parse_yaml_file();
	int solve_task();
	int stop_sim();

	int locate_object();
	int get_grasping_pose();
	int move_to_object();
	int move_to_target_zone();
};

inline Statemachine* Statemachine::get_instance()
{
	if(instance_==NULL)
	{
		instance_=new Statemachine();
	}
	return instance_;
};

#endif //__STATEMACHINE_HPP__
