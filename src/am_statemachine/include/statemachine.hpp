#ifndef __STATEMACHINE_HPP__
#define __STATEMACHINE_HPP__

//ros includes
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
//general includes
#include <sstream>
#include <boost/thread.hpp>

//euroc includes
#include <euroc_c2_msgs/SaveLog.h>
#include <euroc_c2_msgs/ListScenes.h>
#include <euroc_c2_msgs/StartSimulator.h>
#include <euroc_c2_msgs/StopSimulator.h>

//am msgs
#include <am_msgs/Object.h>
#include <am_msgs/TargetZone.h>
#include <am_msgs/GetGraspPose.h>
#include <am_msgs/goalPoseAction.h>
#include <am_msgs/GripperControl.h>
#include <am_msgs/VisionAction.h>

//am includes
#include <utils.hpp>
#include <config.hpp>
#include <fsm_state.hpp>
#include <euroc_input.hpp>

typedef actionlib::SimpleActionClient<am_msgs::goalPoseAction> Client;

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
	fsm::fsm_state_t state_;

	//!node handle for this node
	ros::NodeHandle node_;
	//!callback queue
	ros::CallbackQueue my_queue_;

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

	actionlib::SimpleActionClient<am_msgs::goalPoseAction> motion_planning_action_client_;
	actionlib::SimpleActionClient<am_msgs::VisionAction> vision_action_client_;
	ros::ServiceClient get_grasp_pose_client_;
	am_msgs::GetGraspPose get_grasp_pose_srv_;
	ros::ServiceClient gripper_control_client_;
	am_msgs::GripperControl gripper_control_srv_;

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
	void execute();
	int tick();

	fsm::fsm_state_t get_state()	{ return state_; };

private:
	int request_task();
	void request_task_cb();
	uint8_t request_task_state_;
	boost::thread lsc_;
	int start_sim();
	void start_sim_cb();
	uint8_t start_sim_state_;
	int parse_yaml_file();
	int solve_task();
	int stop_sim();
	void stop_sim_cb();
	uint8_t stop_sim_state_;

	void grip_cb();
	uint8_t grip_state_;
	int locate_object();
	int get_grasping_pose();
	int move_to_object();
	int grip_object();
	int move_to_target_zone();

	void mto_done(const actionlib::SimpleClientGoalState& state,
				  const am_msgs::goalPoseResultConstPtr& result);
	void mto_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);
	void mto_active();
	void mttz_done(const actionlib::SimpleClientGoalState& state,
				  const am_msgs::goalPoseResultConstPtr& result);
	uint8_t mto_;
	uint8_t mttz_;
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
