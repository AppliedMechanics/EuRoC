#ifndef __STATEMACHINE_HPP__
#define __STATEMACHINE_HPP__

//ros includes
#include <ros/ros.h>
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

typedef actionlib::SimpleActionClient<am_msgs::VisionAction> visionClient;
typedef actionlib::SimpleActionClient<am_msgs::goalPoseAction> motionClient;

class StaticTFBroadcaster;

class Statemachine
{

private:
	//!Constructor
	Statemachine();
	//!private copy contructor ...
	Statemachine(const Statemachine&);
	~Statemachine();

	//!statemachine instance (singleton)
	static Statemachine* instance_;

	//!Static TF Broadcaster
	StaticTFBroadcaster* broadcaster_;
	ros::Timer br_timer_;

	//!input container class for yaml data
	EurocInput *ein_;

	//!state variable
	fsm::fsm_state_t state_;

	//!node handle for this node
	ros::NodeHandle node_;

	//!interface/client names
	std::string task_selector_;
	std::string list_scenes_;
	std::string start_simulator_;
	std::string stop_simulator_;
	std::string euroc_c2_interface_;
	std::string save_log_;


	//!Euroc client to access available scenes
	ros::ServiceClient list_scenes_client_;
	//!list scenes message
	euroc_c2_msgs::ListScenes list_scenes_srv_;
	//!Euroc client to start the simulator
	ros::ServiceClient start_simulator_client_;
	//!start simulator message
	euroc_c2_msgs::StartSimulator start_simulator_srv_;
	//!Euroc client to stop the simulator
	ros::ServiceClient stop_simulator_client_;
	//stop simulator message
	euroc_c2_msgs::StopSimulator stop_simulator_srv_;
	//!Euroc client to save the logfile
	ros::ServiceClient save_log_client_;
	//!save log message
	euroc_c2_msgs::SaveLog save_log_srv_;


	//!action client for motionplanning-node (am_motionplanning)
	actionlib::SimpleActionClient<am_msgs::goalPoseAction> motion_planning_action_client_;
	//!action client for vision-node (am_vision)
	actionlib::SimpleActionClient<am_msgs::VisionAction> vision_action_client_;
	//!client for grasp-service (am_grasping)
	ros::ServiceClient get_grasp_pose_client_;
	//!grasp message
	am_msgs::GetGraspPose get_grasp_pose_srv_;
	//!client for gripper-service
	ros::ServiceClient gripper_control_client_;
	//!gripper message
	am_msgs::GripperControl gripper_control_srv_;


	//!true if a task is actually running
	bool task_active_;
	//!true if simulation client is running
	bool sim_running_;


	//!internal counter for while loop in execute()
	uint64_t counter;


	//!list of all available scenes
	std::vector<euroc_c2_msgs::Scene> scenes_;
	//!number of scenes
	uint8_t nr_scenes_;
	//!active scene
	int8_t active_scene_;


	//!current object that is in progress
	am_msgs::Object cur_obj_;
	//!corresponding zone to this object
	am_msgs::TargetZone cur_zone_;


	//!goal queue for motion planning action server
	std::vector<am_msgs::goalPoseGoal> goal_queue;
	//!number of goals in the queue for a certain state
	uint8_t nr_goals_;
	//!active goal
	uint8_t active_goal_;

	//!thread for long lasting service calls
	boost::thread lsc_;

public:
	//!singleton implementation
	static Statemachine* get_instance();

	//!init statemachine
	int init_sm();
	//!execute statemachine
	void execute();

	//!returns current state
	fsm::fsm_state_t get_state()	{ return state_; };

private:
	//!main function that is called every timestep
	int tick();
	//! broadcast callback
	void brTimerCallback(const ros::TimerEvent& event);

	//functions for each state (all of them are called in tick() )
	//!request task state function
	int request_task();
	//!callback for request task state function
	void request_task_cb();
	//!state of the callback (OPEN,RUNNING,FINISHED)
	uint8_t request_task_state_;

	//!start sim state function
	int start_sim();
	//!callback for start sim state function (started as thread)
	void start_sim_cb();
	//!state of the callback (OPEN,RUNNING,FINISHED)
	uint8_t start_sim_state_;

	//!parse yaml file state function
	int parse_yaml_file();

	//!solve task state function (dummy)
	int solve_task();

	//!stop sim state function
	int stop_sim();
	//!callback for stop sim state function (started as thread)
	void stop_sim_cb();
	//!state of the callback (OPEN,RUNNING,FINISHED)
	uint8_t stop_sim_state_;

	//!callback for gripper service calls
	void grip_cb();
	//!state of gripper service call (OPEN,RUNNING,FINISHED)
	uint8_t grip_state_;

	//!locate object state function
	int locate_object();
	//!state of the vision (OPEN,RUNNING,FINISHED)
	uint8_t vision_state_;

	//!get grasping pose state function
	int get_grasping_pose();

	//!move to object state function
	int move_to_object();
	//!state of motion function (OPEN,RUNNING,FINISHED)
	uint8_t motion_state_;

	//!grip object state function
	int grip_object();

	//!move to target zone state function
	int move_to_target_zone();

	//!homing function (goto upright pose)
	int homing();

	//!vision action-client callbacks:
	void vision_done(const actionlib::SimpleClientGoalState& state,
			  	  	 const am_msgs::VisionResultConstPtr& result);
	void vision_feedback(const am_msgs::VisionFeedbackConstPtr feedback);
	//!motion action-client callbacks:
	void motion_done(const actionlib::SimpleClientGoalState& state,
				  const am_msgs::goalPoseResultConstPtr& result);
	void motion_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);
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
