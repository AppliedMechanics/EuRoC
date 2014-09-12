#ifndef __STATEMACHINE_HPP__
#define __STATEMACHINE_HPP__

//ros includes
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <tf/LinearMath/Quaternion.h>

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
#include <am_msgs/TakeImage.h>
#include <am_msgs/VisionAction.h>
#include <am_msgs/ObjectPickedUp.h>

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
	public:
		//!Constructor
		Statemachine();
		//!Destructor
		~Statemachine();

		//!true if a task is actually running
		bool task_active_;
		//!true if simulation client is running
		bool sim_running_;

	private:
		//!Static TF Broadcaster
		StaticTFBroadcaster* broadcaster_;

		//!input container class for yaml data
		EurocInput *ein_;

		//!state variable
		fsm::fsm_state_t state_;
		std::vector<fsm::fsm_state_t> state_queue;

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
		//!client for environment scanning
		ros::ServiceClient take_image_client_;
		//!explore environment service
		am_msgs::TakeImage take_image_srv_;

		//!state-observer interface
		ros::ServiceClient state_observer_client_;
		am_msgs::ObjectPickedUp obj_picked_up_srv_;


		//!internal counter for while loop in execute()
		uint64_t counter;

		//!list of all available scenes
		std::vector<euroc_c2_msgs::Scene> scenes_;
		//!number of scenes
		uint8_t nr_scenes_;
		//!active scene
		int8_t active_scene_;
		//!active task number (generated out of name (string))
		int8_t active_task_number_;

		//!current object that is in progress
		am_msgs::Object cur_obj_;
		//!corresponding zone to this object
		am_msgs::TargetZone cur_zone_;
		//!current object mass
		double cur_obj_mass_;
		geometry_msgs::Vector3 r_tcp_curobjcom_;
		geometry_msgs::Vector3 r_gp_curobjcom_;
		geometry_msgs::Vector3 r_gp_curobj_;

		//!goal queue for motion planning action server
		std::vector<am_msgs::goalPoseGoal> goal_queue;
		//!number of goals in the queue for a certain state
		uint8_t nr_goals_;
		//!active goal
		uint8_t active_goal_;
		//!has the active goal been reached?
		bool reached_active_goal_;

		//!thread for long lasting service calls
		boost::thread lsc_;

		bool skip_vision_;

	public:
		//!init statemachine
		int init_sm();

		//!returns current state
		fsm::fsm_state_t get_state()	{ return state_; };

		//! returns the name of the given state-tree as a string
		std::string get_state_name(fsm::fsm_state_t parstate);

		//!main function that is called every timestep
		int tick();

	private:
		//! move to next state
		void scheduler_next();
		//! make a new schedule, or modify the existing one
		void scheduler_schedule();
		//!print state queue to console
		void scheduler_printqueue();

		//!Make a stop to help with debugging (waits for console input)
		int pause();
		//!state of pause() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t pause_state_;

		//!request task from ROS-master
		int request_task();
		//!callback for request_task()
		void request_task_cb();
		//!state of request_task() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t request_task_state_;

		//!start simulator
		int start_sim();
		//!callback for start_sim()
		void start_sim_cb();
		//!state of start_sim() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t start_sim_state_;

		//!parse yaml file
		int parse_yaml_file();
		//!state of parse_yaml_file() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t parse_yaml_file_state_;

		//!stop simulator
		int stop_sim();
		//!callback for stop_sim()
		void stop_sim_cb();
		//!state of stop_sim() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t stop_sim_state_;

		//!Scan the scene
		int watch_scene();
		//!callback for watch_scene()
		void watch_scene_cb();
		//!state of watch_scene() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t watch_scene_state_;

		//!Explore the environment (initial part)
		int explore_environment_init();
		//!state of explore_environment_init() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t explore_environment_init_state_;

		//!Explore the environment (motion part)
		int explore_environment_motion();
		//!state of explore_environment_motion() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t explore_environment_motion_state_;
		//!callbacks for explore_environment_motion()
		void explore_environment_motion_done(const actionlib::SimpleClientGoalState& state,const am_msgs::goalPoseResultConstPtr& result);
		void explore_environment_motion_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);

		//!Explore the environment (image part)
		int explore_environment_image();
		//!callback for explore_environment_image()
		void explore_environment_image_cb();
		//!state of explore_environment_image() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t explore_environment_image_state_;

		//!locate object
		int locate_object();
		//!state of locate_object() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t locate_object_state_;
		//!callbacks for locate_object()
		void locate_object_done(const actionlib::SimpleClientGoalState& state,const am_msgs::VisionResultConstPtr& result);
		void locate_object_feedback(const am_msgs::VisionFeedbackConstPtr feedback);

		//!get grasping pose
		int get_grasping_pose();
		//!callback for get_grasping_pose()
		void get_grasping_pose_cb();
		//!state of get_grasping_pose() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t get_grasping_pose_state_;

		//!move to object above
		int move_to_object_above();
		//!state of move_to_object_above() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t move_to_object_above_state_;
		//!callbacks for move_to_object_above()
		void move_to_object_above_done(const actionlib::SimpleClientGoalState& state,const am_msgs::goalPoseResultConstPtr& result);
		void move_to_object_above_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);

		//!move to object
		int move_to_object();
		//!state of move_to_object() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t move_to_object_state_;
		//!callbacks for move_to_object()
		void move_to_object_done(const actionlib::SimpleClientGoalState& state,const am_msgs::goalPoseResultConstPtr& result);
		void move_to_object_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);

		//!check wether the object has been placed correctly
		int check_object_finished();
		//!state of check_object_finished() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t check_object_finished_state_;

		//!release the gripper
		int gripper_release();
		//!callback for gripper_release()
		void gripper_release_cb();
		//!state of gripper_release() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t gripper_release_state_;

		//!release the gripper
		int gripper_close();
		//!callback for gripper_close()
		void gripper_close_cb();
		//!state of gripper_close() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t gripper_close_state_;

		//!move to target zone above
		int move_to_target_zone_above();
		//!state of move_to_target_zone_above() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t move_to_target_zone_above_state_;
		//!callbacks for move_to_target_zone_above()
		void move_to_target_zone_above_done(const actionlib::SimpleClientGoalState& state,const am_msgs::goalPoseResultConstPtr& result);
		void move_to_target_zone_above_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);

		//!move to target zone
		int move_to_target_zone();
		//!state of move_to_target_zone() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t move_to_target_zone_state_;
		//!callbacks for move_to_target_zone()
		void move_to_target_zone_done(const actionlib::SimpleClientGoalState& state,const am_msgs::goalPoseResultConstPtr& result);
		void move_to_target_zone_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);

		//!homing function (goto upright pose)
		int homing();
		//!state of homing() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
		uint8_t homing_state_;
		//!callbacks of homing()
		void homing_done(const actionlib::SimpleClientGoalState& state,const am_msgs::goalPoseResultConstPtr& result);
		void homing_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);
};

//Methods of main programm
void exitstatemachine();
void signalHandler(int sig);

#endif //__STATEMACHINE_HPP__
