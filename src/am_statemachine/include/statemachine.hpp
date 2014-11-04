#ifndef __STATEMACHINE_HPP__
#define __STATEMACHINE_HPP__


//ros includes
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <actionlib/client/simple_action_client.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>
#include <octomap_msgs/BoundingBoxQuery.h>

//general includes
#include <sstream>
#include <boost/thread.hpp>

//euroc includes
#include <euroc_c2_msgs/SaveLog.h>
#include <euroc_c2_msgs/ListScenes.h>
#include <euroc_c2_msgs/StartSimulator.h>
#include <euroc_c2_msgs/StopSimulator.h>
#include <euroc_c2_msgs/SetObjectLoad.h>
#include <euroc_c2_msgs/RequestNextObject.h>

//am msgs
#include <am_msgs/Object.h>
#include <am_msgs/TargetZone.h>
#include <am_msgs/GetGraspPose.h>
#include <am_msgs/goalPoseAction.h>
#include <am_msgs/GripperControl.h>
#include <am_msgs/TakeImage.h>
#include <am_msgs/VisionAction.h>
#include <am_msgs/ObjectPickedUp.h>
#include <am_msgs/CheckPoses.h>
#include <am_msgs/ObjState.h>
#include <am_msgs/CheckZones.h>

//am includes
#include <utils.hpp>
#include <config.hpp>
#include <fsm_state.hpp>
#include <euroc_input.hpp>


typedef actionlib::SimpleActionClient<am_msgs::VisionAction> visionClient;
typedef actionlib::SimpleActionClient<am_msgs::goalPoseAction> motionClient;

class StaticTFBroadcaster;
class ExplorePoses;

class Statemachine
{
public:
	//!Constructor
	Statemachine();
	//!Destructor
	~Statemachine();

	//!init statemachine
	int init_sm();

	//!returns current state
	fsm::fsm_state_t get_state()	{ return state_; };

	//! returns the name of the given state-tree as a string
	std::string get_state_name(fsm::fsm_state_t parstate);

	//!main function that is called every timestep
	int tick();

	//!true if a task is actually running
	bool task_active_;
	//!true if simulation client is running
	bool sim_running_;

private:
	//!Static TF Broadcaster
	StaticTFBroadcaster* broadcaster_;

	//!Explore Poses
	ExplorePoses* explore_poses_;

	//!input container class for yaml data
	EurocInput *ein_;

	//!state variable
	fsm::fsm_state_t state_;
	std::vector<fsm::fsm_state_t> state_queue;

	//!speed modification due to stop condition event
	double speed_mod_;

	//! motion planning algorithm
	typedef struct{
		uint32_t object;
		uint32_t move_to_object;
		uint32_t target;
		uint32_t move_to_target;
		uint32_t move_to_target_zone;
		uint32_t homing;
	}planning_mode_t;

	planning_mode_t planning_mode_;

	//!node handle for this node
	ros::NodeHandle node_;

	//!interface/client names
	std::string task_selector_;
	std::string list_scenes_;
	std::string start_simulator_;
	std::string stop_simulator_;
	std::string euroc_c2_interface_;
	std::string save_log_;
	std::string set_object_load_;
	std::string next_object_;

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
	//!Euroc client to set object load
	ros::ServiceClient set_object_load_client_;
	//!object load message
	euroc_c2_msgs::SetObjectLoad set_object_load_srv_;
	//!Euroc client to request next object
	euroc_c2_msgs::RequestNextObject next_object_srv_;
	ros::ServiceClient next_object_client_;


	//!action client for motionplanning-node (am_motionplanning)
	actionlib::SimpleActionClient<am_msgs::goalPoseAction> *motion_planning_action_client_;
	actionlib::SimpleActionClient<am_msgs::goalPoseAction> *motion_planning_action_client_t6_;
	actionlib::SimpleActionClient<am_msgs::goalPoseAction> *motion_planning_action_client_std_;

	am_msgs::goalPoseResult motion_planning_result_;
	//!action client for vision-node (am_vision)
	actionlib::SimpleActionClient<am_msgs::VisionAction> *vision_action_client_;
	am_msgs::VisionResult vision_result_;
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
	//!client for target zone check
	ros::ServiceClient check_zones_client_;
	am_msgs::CheckZones check_zones_srv_;
	//!state-observer interface
	ros::ServiceClient state_observer_client_;
	am_msgs::ObjectPickedUp obj_picked_up_srv_;
	//!client to check poses service from motion planning
	ros::ServiceClient check_poses_client_;
	am_msgs::CheckPoses check_poses_srv_;

	//!client to remove area in octomap
	ros::ServiceClient rm_grasping_area_collision_client_;
	octomap_msgs::BoundingBoxQuery rm_grasping_area_collision_srv_;

	//!publisher for motion planning (object state)
	ros::Publisher obj_state_;
	am_msgs::ObjState obj_state_msg_;
	void publish_obj_state(uint16_t state);

	ros::Publisher reset_pub_;



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
	uint8_t cur_object_type_;
	//!corresponding zone to this object
	am_msgs::TargetZone cur_zone_;
	//!corresponding target pose to this object
	geometry_msgs::Pose cur_target_pose_;
	//!current object mass
	double cur_obj_mass_;
	//!robot has cur obj gripped
	bool cur_obj_gripped_;

	//!object counter for task 6
	uint16_t obj_counter_t6_;

	//grasping poses
	uint16_t selected_object_pose_;
	std::vector<geometry_msgs::Pose> object_grip_pose;
	std::vector<geometry_msgs::Pose> object_safe_pose;
	std::vector<geometry_msgs::Pose> object_vision_pose;
	std::vector<uint16_t> grip_pose_type;
	std::vector<uint16_t> object_skip_vision;
	std::vector<double> object_grasp_width;
	uint16_t selected_target_pose_;
	std::vector<geometry_msgs::Pose> target_place_pose;
	std::vector<geometry_msgs::Pose> target_safe_pose;
	std::vector<geometry_msgs::Pose> target_vision_pose;
	std::vector<uint16_t> place_pose_type;
	std::vector<uint16_t> target_skip_vision;
	std::vector<geometry_msgs::Pose> push_safe_pose;
	std::vector<geometry_msgs::Pose> push_target_pose;
	std::vector<geometry_msgs::Vector3> object_grip_r_tcp_com;

	//transformation from gp-tcp to object origin
	tf::Transform gp_obj_orig_;

	//!goal queue for motion planning action server
	std::vector<am_msgs::goalPoseGoal> goal_queue;
	std::vector<am_msgs::goalPoseGoal> explore_queue_;
	std::vector<am_msgs::VisionGoal>   vision_queue;
	//!number of goals in the queue for a certain state
	uint8_t nr_goals_;
	//!number of exp. poses
	uint8_t nr_exp_poses_;
	//!active goal
	uint8_t active_goal_;
	//!maximum explore poses
	uint8_t max_explore_poses_;
	//!Successful explore poses
	uint8_t explore_success_count_;
	//!Explore Pose Type
	uint8_t explore_pose_type_;

	//!has the active goal been reached?
	bool reached_active_goal_;

	//!thread for long lasting service calls
	boost::thread lsc_;

	bool skip_vision_;
	bool skip_motion_;
	bool pause_in_loop_;


	//! move to next state
	void scheduler_next();
	//! make a new schedule, or modify the existing one
	void scheduler_schedule();
	//!print state queue to console
	void scheduler_printqueue();
	//!skip object
	void scheduler_skip_object();
	//!skip explore environment
	void scheduler_skip_explore();
	//!properly change to next object
	void scheduler_next_object();
	//!schedule grasping sequence
	void scheduler_grasp_object(bool start);
	//!schedule placing sequence
	void scheduler_place_object(bool start);


	//!error handling for each state
	void scheduler_error_homing();
	void scheduler_error_move_to_object_vision();
	void scheduler_error_move_to_object_safe();
	void scheduler_error_move_to_object();
	void scheduler_error_move_to_target_zone_vision();
	void scheduler_error_move_to_target_zone_safe();
	void scheduler_error_move_to_target_zone();
	void scheduler_error_check_object_gripped();
	void scheduler_error_check_object_finished();
	void scheduler_error_gripper_close();
	void scheduler_error_gripper_release();
	void scheduler_error_watch_scene();
	void scheduler_error_explore_environment_motion();
	void scheduler_error_explore_environment_image();
	void scheduler_error_locate_object_global();

	//!state for setting the object load in gripper_close() and gripper_release()
	uint8_t set_object_load_state_;
	//!callback for gripper_release() and gripper_close()
	void set_object_load_cb();

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
	uint8_t watch_scene_counter_;

	//!Explore the environment (initial part)
	int explore_environment_init();
	//!state of explore_environment_init() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t explore_environment_init_state_;

	//!Explore the environment (motion part)
	int explore_environment_motion();
	//!callbacks for explore_environment_motion()
	void explore_environment_motion_done(const actionlib::SimpleClientGoalState& state,const am_msgs::goalPoseResultConstPtr& result);
	void explore_environment_motion_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);
	//!state of explore_environment_motion() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t explore_environment_motion_state_;

	//!Explore the environment (image part)
	int explore_environment_image();
	//!callback for explore_environment_image()
	void explore_environment_image_cb();
	//!state of explore_environment_image() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t explore_environment_image_state_;
	uint8_t explore_environment_image_counter_;

	//!Explore the environment (check results) -> objects in targetzones lying?
	int explore_environment_check();
	//!callback for explore_environment_check()
	void explore_environment_check_cb();
	//!state of explore_environment_check() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t explore_environment_check_state_;

	//!locate object in a global manner
	int locate_object_global();
	//!callbacks for locate_object_global()
	void locate_object_global_done(const actionlib::SimpleClientGoalState& state,const am_msgs::VisionResultConstPtr& result);
	void locate_object_global_feedback(const am_msgs::VisionFeedbackConstPtr feedback);
	//!state of locate_object_global() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t locate_object_global_state_;
	uint8_t locate_object_global_counter_;

	//!locate all objects in a global manner
	int locate_all_objects_global();
	//!callbacks for locate_all_objects_global()
	void locate_all_objects_global_done(const actionlib::SimpleClientGoalState& state,const am_msgs::VisionResultConstPtr& result);
	void locate_all_objects_global_feedback(const am_msgs::VisionFeedbackConstPtr feedback);
	//!state of locate_all_objects_global() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t locate_all_objects_global_state_;
	uint8_t locate_all_objects_global_counter_;

	//!locate object from short distance with TCP CAM
	int locate_object_close_range();
	//!callbacks for locate_object_close_range()
	void locate_object_close_range_done(const actionlib::SimpleClientGoalState& state,const am_msgs::VisionResultConstPtr& result);
	void locate_object_close_range_feedback(const am_msgs::VisionFeedbackConstPtr feedback);
	//!state of locate_object_close_range() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t locate_object_close_range_state_;

	//!get grasping pose
	int get_grasping_pose();
	//!callback for get_grasping_pose()
	void get_grasping_pose_cb();
	//!state of get_grasping_pose() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t get_grasping_pose_state_;
	//!get the pose-vectors from the grasping node an assign it to the local vectors (true if everything is fine, false otherwise)
	bool get_grasp_pose_get_vectors();
	//!find a feasible pose set
	int find_pose_set();

	//!get grasping pose (Task 5)
	int get_grasping_poseT5();
	//!callback for get_grasping_poseT5() (Task 5)
	void get_grasping_poseT5_cb();
	//!state of get_grasping_poseT5() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t get_grasping_poseT5_state_;

	//!get grasping pose (Task 6)
	int get_grasping_poseT6();
	//!callback for get_grasping_poseT6() (Task 6)
	void get_grasping_poseT6_cb();
	//!state of get_grasping_poseT6() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t get_grasping_poseT6_state_;

	//!move to object safe
	int move_to_object_safe();
	//!callbacks for move_to_object_safe()
	void move_to_object_safe_done(const actionlib::SimpleClientGoalState& state,const am_msgs::goalPoseResultConstPtr& result);
	void move_to_object_safe_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);
	//!state of move_to_object_safe() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t move_to_object_safe_state_;
	uint8_t move_to_object_safe_counter_;

	//!move to object vision
	int move_to_object_vision();
	//!callbacks for move_to_object_vision()
	void move_to_object_vision_done(const actionlib::SimpleClientGoalState& state,const am_msgs::goalPoseResultConstPtr& result);
	void move_to_object_vision_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);
	//!state of move_to_object_vision() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t move_to_object_vision_state_;
	uint8_t move_to_object_vision_counter_;

	//!move to object
	int move_to_object();
	//!callbacks for move_to_object()
	void move_to_object_done(const actionlib::SimpleClientGoalState& state,const am_msgs::goalPoseResultConstPtr& result);
	void move_to_object_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);
	//!state of move_to_object() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t move_to_object_state_;
	uint8_t move_to_object_counter_;

	//!move to object for task 6
	int move_to_object_t6();
	//!callbacks for move_to_object()
	void move_to_object_t6_done(const actionlib::SimpleClientGoalState& state,const am_msgs::goalPoseResultConstPtr& result);
	void move_to_object_t6_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);
	//!state of move_to_object() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t move_to_object_t6_state_;
	uint8_t move_to_object_t6_counter_;

	//!move to target zone for task 6
	int move_to_target_zone_t6();
	//!callbacks for move_to_target_zone_t6()
	void move_to_target_zone_t6_done(const actionlib::SimpleClientGoalState& state,const am_msgs::goalPoseResultConstPtr& result);
	void move_to_target_zone_t6_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);
	//!state of move_to_object() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t move_to_target_zone_t6_state_;
	uint8_t move_to_target_zone_t6_counter_;

	//!check whether the object has been placed correctly
	int check_object_finished();
	//!state of check_object_finished() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t check_object_finished_state_;
	//!callbacks for check_object_finished()
	void check_object_finished_done(const actionlib::SimpleClientGoalState& state,const am_msgs::VisionResultConstPtr& result);
	void check_object_finished_feedback(const am_msgs::VisionFeedbackConstPtr feedback);

	//!for task 6, request next object
	int new_object_t6();
	//!callback for new_object_t6()
	void new_object_t6_cb();
	//!state of new_object_t6() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t new_object_t6_state_;

	//!check whether the object has been gripped correctly
	int check_object_gripped();
	//!callback for check_object_gripped()
	void check_object_gripped_cb();
	//!state of check_object_gripped() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t check_object_gripped_state_;
	uint8_t check_object_gripped_counter_;

	//!release the gripper
	int gripper_release();
	//!callback for gripper_release()
	void gripper_release_cb();
	//!state of gripper_release() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t gripper_release_state_;
	uint8_t gripper_release_counter_;

	//!release the gripper
	int gripper_close();
	//!callback for gripper_close()
	void gripper_close_cb();
	//!state of gripper_close() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t gripper_close_state_;
	uint8_t gripper_close_counter_;

	//!move to target zone safe
	int move_to_target_zone_safe();
	//!callbacks for move_to_target_zone_safe()
	void move_to_target_zone_safe_done(const actionlib::SimpleClientGoalState& state,const am_msgs::goalPoseResultConstPtr& result);
	void move_to_target_zone_safe_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);
	//!state of move_to_target_zone_safe() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t move_to_target_zone_safe_state_;
	uint8_t move_to_target_zone_safe_counter_;

	//!move to target zone vision
	int move_to_target_zone_vision();
	//!callbacks for move_to_target_zone_vision()
	void move_to_target_zone_vision_done(const actionlib::SimpleClientGoalState& state,const am_msgs::goalPoseResultConstPtr& result);
	void move_to_target_zone_vision_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);
	//!state of move_to_target_zone_vision() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t move_to_target_zone_vision_state_;
	uint8_t move_to_target_zone_vision_counter_;

	//!move to target zone
	int move_to_target_zone();
	//!callbacks for move_to_target_zone()
	void move_to_target_zone_done(const actionlib::SimpleClientGoalState& state,const am_msgs::goalPoseResultConstPtr& result);
	void move_to_target_zone_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);
	//!state of move_to_target_zone() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t move_to_target_zone_state_;
	uint8_t move_to_target_zone_counter_;

	//!homing function (goto upright pose)
	int homing();
	//!callbacks of homing()
	void homing_done(const actionlib::SimpleClientGoalState& state,const am_msgs::goalPoseResultConstPtr& result);
	void homing_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback);
	//!state of homing() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t homing_state_;
	uint8_t homing_counter_;


	//utililty functions:
	int check_time();
	int wait();
	uint16_t wait_counter_;
	int reset();
	//! state of reset() (OPEN,RUNNING,FINISHED,FINISHEDWITHERRORS)
	uint8_t reset_state_;
	uint8_t reset_counter_;
};

//Methods of main programm
void exitstatemachine();
void signalHandler(int sig);

#endif //__STATEMACHINE_HPP__
