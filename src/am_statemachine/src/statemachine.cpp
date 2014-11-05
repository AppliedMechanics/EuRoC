#include <statemachine.hpp>
#include <cmath>
#include <StaticTFBroadcaster.h>
#include <explore_poses.h>

#define ONE_TASK //run only one task and then quit

Statemachine::Statemachine():
		scenes_(1),
		task_active_(false),
		sim_running_(false),
		speed_mod_(0),
		nr_scenes_(0),
		active_scene_(-1),
		active_goal_(0),
		max_explore_poses_(10),
		explore_success_count_(0),
		nr_goals_(0),
		nr_exp_poses_(0),
		explore_pose_type_(EXPLORE_STD_1),
		obj_counter_t6_(1),
		skip_vision_(false),
		skip_motion_(false),
		pause_in_loop_(0),
		reached_active_goal_(false),
		request_task_state_(OPEN),
		start_sim_state_(OPEN),
		set_object_load_state_(OPEN),
		pause_state_(OPEN),
		parse_yaml_file_state_(OPEN),
		stop_sim_state_(OPEN),
		watch_scene_state_(OPEN),
		watch_scene_counter_(0),
		explore_environment_init_state_(OPEN),
		explore_environment_motion_state_(OPEN),
		explore_environment_image_state_(OPEN),
		explore_environment_check_state_(OPEN),
		explore_environment_image_counter_(0),
		locate_object_global_state_(OPEN),
		locate_object_global_counter_(0),
		locate_all_objects_global_state_(OPEN),
		locate_all_objects_global_counter_(0),
		locate_object_close_range_state_(OPEN),
		check_object_finished_state_(OPEN),
		check_object_gripped_state_(OPEN),
		check_object_gripped_counter_(0),
		new_object_t6_state_(OPEN),
		get_grasping_pose_state_(OPEN),
		get_grasping_poseT5_state_(OPEN),
		get_grasping_poseT6_state_(OPEN),
		move_to_object_vision_state_(OPEN),
		move_to_object_vision_counter_(0),
		move_to_object_safe_state_(OPEN),
		move_to_object_safe_counter_(0),
		move_to_object_state_(OPEN),
		move_to_object_counter_(0),
		move_to_object_t6_state_(OPEN),
		move_to_object_t6_counter_(0),
		move_to_target_zone_t6_state_(OPEN),
		move_to_target_zone_t6_counter_(0),
		gripper_release_state_(OPEN),
		gripper_release_counter_(0),
		gripper_close_state_(OPEN),
		gripper_close_counter_(0),
		move_to_target_zone_safe_state_(OPEN),
		move_to_target_zone_safe_counter_(0),
		move_to_target_zone_vision_state_(OPEN),
		move_to_target_zone_vision_counter_(0),
		move_to_target_zone_state_(OPEN),
		move_to_target_zone_counter_(0),
		homing_state_(OPEN),
		homing_counter_(0),
		reset_state_(OPEN),
		reset_counter_(0)
{
	ein_=new EurocInput();
	broadcaster_ = new StaticTFBroadcaster();
	explore_poses_ = new ExplorePoses();

	vision_action_client_ = new actionlib::SimpleActionClient<am_msgs::VisionAction>("VisionAction", true);
	// set motion planning action client
	motion_planning_action_client_ = new actionlib::SimpleActionClient<am_msgs::goalPoseAction>("goalPoseAction", true);
	ROS_INFO("Wait 0.1s");
	boost::this_thread::sleep( boost::posix_time::milliseconds(100));


	cur_obj_gripped_=false;

	//==============================================
	//state:
	state_.sub.one = fsm::INITIAL_STATE;
	//==============================================

	task_selector_ = "/euroc_c2_task_selector";
	list_scenes_ = (task_selector_ + "/list_scenes");
	start_simulator_ = (task_selector_ + "/start_simulator");
	stop_simulator_ = (task_selector_ + "/stop_simulator");
	euroc_c2_interface_ = "/euroc_interface_node";
	save_log_ = euroc_c2_interface_ + "/save_log";
	set_object_load_ = euroc_c2_interface_ + "/set_object_load";
	next_object_ = (euroc_c2_interface_ + "/request_next_object");

	ros::param::get("/skip_vision",skip_vision_);
	ros::param::get("/skip_motion",skip_motion_);
	ros::param::get("/pause_in_loop",pause_in_loop_);

	//first object drops automatically after starting the simulator with task 6
	node_.setParam("object_counter_",obj_counter_t6_);

	planning_mode_.object	= STANDARD_IK_7DOF;
	planning_mode_.move_to_object	= STANDARD_IK_7DOF;
	planning_mode_.target	= STANDARD_IK_7DOF;
	planning_mode_.move_to_target_zone	= STANDARD_IK_7DOF;
	planning_mode_.homing	= STANDARD_IK_7DOF;

	obj_state_ = node_.advertise<am_msgs::ObjState>("obj_state", 1000);
	reset_pub_ = node_.advertise<std_msgs::Bool>("kill", 1000);
}

Statemachine::~Statemachine()
{
	ROS_INFO("destructor called");
	delete ein_;
	delete broadcaster_;
	delete explore_poses_;
	delete vision_action_client_;
	delete motion_planning_action_client_;

	if(sim_running_)
	{
		stop_sim();
	}
}

int Statemachine::init_sm()
{
	//Wait for all services
	ROS_INFO("waiting for services...");
	uint8_t CheckServiceCounter=0;
	bool FirstTimeLoop=true;
	while(CheckServiceCounter<3)	//Loop until all services are available
	{
		CheckServiceCounter=0;	//reset counter
		if(ros::service::exists(list_scenes_,false)==true)
		{ CheckServiceCounter++; }
		if(ros::service::exists(start_simulator_,false)==true)
		{ CheckServiceCounter++; }
		if(ros::service::exists(stop_simulator_,false)==true)
		{ CheckServiceCounter++; }
		if(CheckServiceCounter<3)
		{
			//if not all services are available...
			if(FirstTimeLoop==true)
			{
				if(CheckServiceCounter==0)
				{ ROS_INFO("none of the required services is available. Waiting for services..."); }
				else
				{ ROS_INFO("not all of the required services are available. Waiting for services..."); }
				FirstTimeLoop=false;
			}
			//wait for a specific time to reduce cpu usage
			boost::this_thread::sleep( boost::posix_time::milliseconds(100));
		}
	}
	ROS_INFO("all services are available.");

	//generating service clients
	list_scenes_client_ = node_.serviceClient<euroc_c2_msgs::ListScenes>(list_scenes_);
	start_simulator_client_ = node_.serviceClient<euroc_c2_msgs::StartSimulator>(start_simulator_);
	stop_simulator_client_ = node_.serviceClient<euroc_c2_msgs::StopSimulator>(stop_simulator_);
	save_log_client_ = node_.serviceClient<euroc_c2_msgs::SaveLog>(save_log_);
	get_grasp_pose_client_ = node_.serviceClient<am_msgs::GetGraspPose>("GraspPose_srv");
	gripper_control_client_ = node_.serviceClient<am_msgs::GripperControl>("GripperInterface");
	take_image_client_ = node_.serviceClient<am_msgs::TakeImage>("TakeImageService");
	check_zones_client_ = node_.serviceClient<am_msgs::CheckZones>("CheckZonesService");
	state_observer_client_ = node_.serviceClient<am_msgs::ObjectPickedUp>("ObjectPickedUp_srv");
	set_object_load_client_ = node_.serviceClient<euroc_c2_msgs::SetObjectLoad>(set_object_load_);
	next_object_client_ = node_.serviceClient<euroc_c2_msgs::RequestNextObject>(next_object_);

	//check_poses_client_ = node_.serviceClient<am_msgs::CheckPoses>("CheckPoses_srv");
	rm_grasping_area_collision_client_ = node_.serviceClient<octomap_msgs::BoundingBoxQuery>("/octomap_server/clear_bbx");
	//wait for all action servers
	ROS_INFO("waiting for action servers...");
	uint8_t CheckActionServerCounter=0;
	FirstTimeLoop=true;
	while(CheckActionServerCounter<2)	//Loop until all action servers are connected
	{
		CheckActionServerCounter=0;	//reset counter
		if(motion_planning_action_client_->isServerConnected())
		{ CheckActionServerCounter++; }
		if(vision_action_client_->isServerConnected())
		{ CheckActionServerCounter++; }
		if(CheckActionServerCounter<2)
		{
			//if not all action servers are connected...
			if(FirstTimeLoop==true)
			{
				if(CheckActionServerCounter==0)
				{ ROS_INFO("none of the required action servers is connected. Waiting for connection..."); }
				else
				{ ROS_INFO("not all of the required action servers are connected. Waiting for connection..."); }
				FirstTimeLoop=false;
			}
			//wait for a specific time to reduce cpu usage
			boost::this_thread::sleep( boost::posix_time::milliseconds(100));
		}
	}
	//following two lines don't work: see method description (statemachine is single threaded!)
	//motion_planning_action_client_->waitForServer(ros::Duration(5.0));
	//vision_action_client_->waitForServer(ros::Duration(5.0));
	ROS_INFO("all action servers are connected.");

	return 0;
}

std::string Statemachine::get_state_name(fsm::fsm_state_t parstate)
{
	switch(parstate.sub.one)
	{
	case fsm::INITIAL_STATE:
		return "INITIAL_STATE";
	case fsm::PAUSE:
		return "PAUSE";
	case fsm::REQUEST_TASK:
		return "REQUEST_TASK";
	case fsm::START_SIM:
		return "START_SIM";
	case fsm::PARSE_YAML:
		return "PARSE_YAML";
	case fsm::SCHEDULER:
		return "SCHEDULER";
	case fsm::WATCH_SCENE:
		return "WATCH_SCENE";
	case fsm::EXPLORE_ENVIRONMENT:
		switch(parstate.sub.two)
		{
		case fsm::PAUSE:
			return "EXPLORE_ENVIRONMENT->PAUSE";
		case fsm::HOMING:
			return "EXPLORE_ENVIRONMENT->HOMING";
		case fsm::EXPLORE_ENVIRONMENT_INIT:
			return "EXPLORE_ENVIRONMENT->EXPLORE_ENVIRONMENT_INIT";
		case fsm::EXPLORE_ENVIRONMENT_MOTION:
			return "EXPLORE_ENVIRONMENT->EXPLORE_ENVIRONMENT_MOTION";
		case fsm::EXPLORE_ENVIRONMENT_IMAGE:
			return "EXPLORE_ENVIRONMENT->EXPLORE_ENVIRONMENT_IMAGE";
		case fsm::EXPLORE_ENVIRONMENT_CHECK:
			return "EXPLORE_ENVIRONMENT->EXPLORE_ENVIRONMENT_CHECK";
		default:
			return "EXPLORE_ENVIRONMENT->default";
		}
		break;
		case fsm::SOLVE_TASK:
			switch(parstate.sub.two)
			{
			case fsm::PAUSE:
				return "SOLVE_TASK->PAUSE";
			case fsm::WAIT:
				return "SOLVE_TASK->WAIT";
			case fsm::SCHEDULER:
				return "SOLVE_TASK->SCHEDULER";
			case fsm::HOMING:
				return "SOLVE_TASK->HOMING";
			case fsm::LOCATE_OBJECT_GLOBAL:
				return "SOLVE_TASK->LOCATE_OBJECT_GLOBAL";
			case fsm::LOCATE_ALL_OBJECTS_GLOBAL:
				return "SOLVE_TASK->LOCATE_ALL_OBJECTS_GLOBAL";
			case fsm::LOCATE_OBJECT_CLOSE_RANGE:
				return "SOLVE_TASK->LOCATE_OBJECT_CLOSE_RANGE";
			case fsm::GET_GRASPING_POSE:
				return "SOLVE_TASK->GET_GRASPING_POSE";
			case fsm::GET_GRASPING_POSE_T5:
				return "SOLVE_TASK->GET_GRASPING_POSET5";
			case fsm::GET_GRASPING_POSE_T6:
				return "SOLVE_TASK->GET_GRASPING_POSET6";
			case fsm::GRIPPER_RELEASE:
				return "SOLVE_TASK->GRIPPER_RELEASE";
			case fsm::GRIPPER_CLOSE:
				return "SOLVE_TASK->GRIPPER_CLOSE";
			case fsm::MOVE_TO_OBJECT_VISION:
				return "SOLVE_TASK->MOVE_TO_OBJECT_VISION";
			case fsm::MOVE_TO_OBJECT_SAFE:
				return "SOLVE_TASK->MOVE_TO_OBJECT_SAFE";
			case fsm::MOVE_TO_TARGET_ZONE_VISION:
				return "SOLVE_TASK->MOVE_TO_TARGET_ZONE_VISION";
			case fsm::MOVE_TO_TARGET_ZONE_SAFE:
				return "SOLVE_TASK->MOVE_TO_TARGET_ZONE_SAFE";
			case fsm::MOVE_TO_OBJECT_T6:
				return "SOLVE_TASK->MOVE_TO_OBJECT_T6";
			case fsm::MOVE_TO_TARGET_ZONE_T6:
				return "SOLVE_TASK->MOVE_TO_TARGET_ZONE_T6";
			case fsm::CHECK_OBJECT_FINISHED:
				return "SOLVE_TASK->CHECK_OBJECT_FINISHED";
			case fsm::CHECK_OBJECT_GRIPPED:
				return "SOLVE_TASK->CHECK_OBJECT_GRIPPED";
			case fsm::NEW_OBJECT_T6:
				return "SOLVE_TASK->NEW_OBJECT_T6";
			case fsm::GRAB_OBJECT:
				switch(parstate.sub.three)
				{
				case fsm::PAUSE:
					return "SOLVE_TASK->GRAB_OBJECT->PAUSE";
				case fsm::MOVE_TO_OBJECT:
					return "SOLVE_TASK->GRAB_OBJECT->MOVE_TO_OBJECT";
				case fsm::GRIPPER_CLOSE:
					return "SOLVE_TASK->GRAB_OBJECT->GRIPPER_CLOSE";
				case fsm::MOVE_TO_OBJECT_SAFE:
					return "SOLVE_TASK->GRAB_OBJECT->MOVE_TO_OBJECTSAFE";
				default:
					return "SOLVE_TASK->GRAB_OBJECT->default";
				}
				break;
				case fsm::PLACE_OBJECT:
					switch(parstate.sub.three)
					{
					case fsm::PAUSE:
						return "SOLVE_TASK->PLACE_OBJECT->PAUSE";
					case fsm::MOVE_TO_TARGET_ZONE:
						return "SOLVE_TASK->PLACE_OBJECT->MOVE_TO_TARGET_ZONE";
					case fsm::GRIPPER_RELEASE:
						return "SOLVE_TASK->PLACE_OBJECT->GRIPPER_RELEASE";
					case fsm::MOVE_TO_TARGET_ZONE_SAFE:
						return "SOLVE_TASK->PLACE_OBJECT->MOVE_TO_TARGET_ZONE_SAFE";
					default:
						return "SOLVE_TASK->PLACE_OBJECT->default";
					}
					break;
					default:
						return "SOLVE_TASK->default";
			}
		break;
		case fsm::STOP_SIM:
			return "STOP_SIM";
		case fsm::RESET:
			return "RESET";
		default:
			return "default";
	}
}

void Statemachine::scheduler_next()
{
	ROS_INFO("Scheduler: scheduler_next() called");
	if(state_queue.size()==0)	//nothing planned next
	{
		//start scheduler to get new plan
		ROS_INFO("Scheduler: state queue is empty, starting scheduler...");
		scheduler_schedule();
	}
	else
	{
		//Set first element in queue to actual state
		state_=state_queue.at(0);
		//remove this element from queue
		state_queue.erase(state_queue.begin());
		ROS_INFO("Scheduler: new state: %s",get_state_name(state_).c_str());
	}
}

void Statemachine::scheduler_schedule()
{
	fsm::fsm_state_t temp_state;	//temporary state variable for scheduler

	ROS_INFO("Scheduler: scheduler_schedule() called");
	switch(state_.sub.one)
	{
	//------------------------------------------------------------------
	//--- SET UP THE STARTUP-PLAN --------------------------------------
	//------------------------------------------------------------------
	case fsm::INITIAL_STATE: //the first 4 states are always the same
		//clear the queue
		state_queue.clear();

		temp_state.sub.one=fsm::REQUEST_TASK;		state_queue.push_back(temp_state);
		temp_state.sub.one=fsm::START_SIM;			state_queue.push_back(temp_state);
		temp_state.sub.one=fsm::PARSE_YAML;			state_queue.push_back(temp_state);
		temp_state.sub.one=fsm::SCHEDULER;			state_queue.push_back(temp_state);
		scheduler_printqueue(); //print queue to console for debugging purposes
		break;

	case fsm::SCHEDULER: //make a schedule at beginning of a task

		//clear the queue
		state_queue.clear();

		if (!skip_vision_)
		{
			//schedule for task 1 to 4
			if(active_task_number_==1 || active_task_number_==2 || active_task_number_==3 ||
					active_task_number_==4 || active_task_number_==5)
			{
				temp_state.sub.one=fsm::WATCH_SCENE;						state_queue.push_back(temp_state);
				temp_state.sub.one=fsm::EXPLORE_ENVIRONMENT;
				temp_state.sub.two=fsm::HOMING;								state_queue.push_back(temp_state);
				temp_state.sub.two=fsm::EXPLORE_ENVIRONMENT_INIT;			state_queue.push_back(temp_state);
				for (int i=0; i<nr_exp_poses_; i++)
				{
					temp_state.sub.two=fsm::EXPLORE_ENVIRONMENT_MOTION;		state_queue.push_back(temp_state);
					temp_state.sub.two=fsm::EXPLORE_ENVIRONMENT_IMAGE;		state_queue.push_back(temp_state);
				}
				temp_state.sub.two=fsm::HOMING;								state_queue.push_back(temp_state);
				temp_state.sub.two=fsm::EXPLORE_ENVIRONMENT_CHECK;			state_queue.push_back(temp_state);
			}
			else if(active_task_number_==6) //schedule for task 5 and 6
			{
				//Nothing special right now
				temp_state.sub.one=fsm::EXPLORE_ENVIRONMENT;
				temp_state.sub.two=fsm::EXPLORE_ENVIRONMENT_CHECK;			state_queue.push_back(temp_state);
				temp_state.sub.two=fsm::HOMING;								state_queue.push_back(temp_state);
			}
		}
		else
		{
			std::vector<uint16_t> test;
			test.resize(ein_->get_nr_objects(),0);
			if(-1==ein_->sort_objects(test))
			{
				msg_error("sort_objects failed!");
			}
		}

		//Then make a new schedule inside the SOLVE_TASK
		temp_state.sub.one=fsm::SOLVE_TASK;
		temp_state.sub.two=fsm::SCHEDULER;								state_queue.push_back(temp_state);
		scheduler_printqueue(); //print queue to console for debugging purposes
		break;

	case fsm::PAUSE:
		if(pause_state_==FINISHEDWITHERROR)	//user wants to exit
		{
			state_queue.clear();	//throw plan away
			temp_state.sub.one=fsm::STOP_SIM;				state_queue.push_back(temp_state);
			pause_state_=FINISHED;
		}
		break;
	case fsm::REQUEST_TASK:
		if(request_task_state_==FINISHEDWITHERROR)	//something went wrong
		{
			//just start the state again
			ROS_INFO("Statemachine-Errorhandler: restarting REQUEST_TASK state");
			request_task_state_=OPEN;
		}
		break;
	case fsm::START_SIM:
		if(start_sim_state_==FINISHEDWITHERROR)	//something went wrong
		{
			//just start the state again
			ROS_INFO("Statemachine-Errorhandler: restarting state");
			start_sim_state_=OPEN;
		}
		break;
	case fsm::PARSE_YAML:
		if(parse_yaml_file_state_==FINISHEDWITHERROR)	//something went wrong
		{
			//just start the state again
			ROS_INFO("Statemachine-Errorhandler: restarting state");
			parse_yaml_file_state_=OPEN;
		}
		break;
	case fsm::STOP_SIM:
		if(stop_sim_state_==FINISHEDWITHERROR)	//something went wrong
		{
			//the statemachine was shutting down anyway, so just proceed...
			state_.sub.one = fsm::FINISHED;
			task_active_=false;
			sim_running_=false;
		}
		break;
	case fsm::WATCH_SCENE:
		if(watch_scene_state_==FINISHEDWITHERROR)	//something went wrong
		{
			scheduler_error_watch_scene();
		}
		break;
	case fsm::EXPLORE_ENVIRONMENT:
		switch(state_.sub.two)
		{
		case fsm::HOMING:
			if(homing_state_==FINISHEDWITHERROR)
			{
				scheduler_error_homing();
			}
			break;
		case fsm::EXPLORE_ENVIRONMENT_INIT:
			if(explore_environment_init_state_==FINISHEDWITHERROR)	//something went wrong
			{
				msg_error("Error in explore environment init -> skipping exploration");
				explore_environment_init_state_=OPEN;

				//skip explore environment:
				state_queue.clear();
				temp_state.sub.one=fsm::SOLVE_TASK;
				temp_state.sub.two=fsm::SCHEDULER;						state_queue.push_back(temp_state);
			}
			break;
		case fsm::EXPLORE_ENVIRONMENT_MOTION:
			if(explore_environment_motion_state_==FINISHEDWITHERROR)	//something went wrong
			{
				scheduler_error_explore_environment_motion();
			}
			break;
		case fsm::EXPLORE_ENVIRONMENT_IMAGE:
			if(explore_environment_image_state_==FINISHEDWITHERROR)	//something went wrong
			{
				scheduler_error_explore_environment_image();
			}
			break;
		case fsm::EXPLORE_ENVIRONMENT_CHECK:
			if(explore_environment_check_state_==FINISHEDWITHERROR)
			{
				msg_warn("error in explore_environment_check()");
			}
			break;
		default:
			ROS_INFO("Scheduler: Don't know what to do! (%s)",get_state_name(state_).c_str()); break;
		}
		break;
		case fsm::SOLVE_TASK:
			switch(state_.sub.two)
			{
			case fsm::SCHEDULER:
				if(ein_->all_finished()) //stop simulation, when all objects finished
				{
					//temp_state.sub.one=fsm::PAUSE;								state_queue.push_back(temp_state);
					temp_state.sub.one=fsm::STOP_SIM;						state_queue.push_back(temp_state);
				}
				else	//otherwise start with next object
				{
					//Get first object
					scheduler_next_object();

					//Following code can be modified according to specific object-properties
					temp_state.sub.one=fsm::SOLVE_TASK;
					if(pause_in_loop_==true)
					{
						temp_state.sub.two=fsm::PAUSE;					    state_queue.push_back(temp_state);
					}
					if(active_task_number_<=4)
					{
						temp_state.sub.two=fsm::LOCATE_OBJECT_GLOBAL;		state_queue.push_back(temp_state);
						temp_state.sub.two=fsm::GET_GRASPING_POSE;          state_queue.push_back(temp_state);
						if (!skip_motion_)
						{
							scheduler_grasp_object(EXECUTE_LATER);
							scheduler_place_object(EXECUTE_LATER);
						}
						else
						{
							ein_->set_active_object_finished();
							scheduler_next_object();
						}
					}
					else if(active_task_number_==5)
					{
						temp_state.sub.two=fsm::LOCATE_OBJECT_GLOBAL;		state_queue.push_back(temp_state);
						temp_state.sub.two=fsm::GET_GRASPING_POSE_T5;       state_queue.push_back(temp_state);
						if (!skip_motion_)
						{
							scheduler_grasp_object(EXECUTE_LATER);
							temp_state.sub.one=fsm::SOLVE_TASK;
							temp_state.sub.two=fsm::MOVE_TO_TARGET_ZONE_SAFE;		state_queue.insert(it,temp_state);
							temp_state.sub.two=fsm::PLACE_OBJECT;
							temp_state.sub.three=fsm::MOVE_TO_TARGET_ZONE;		    state_queue.insert(++it,temp_state);
							//scheduler_place_object(EXECUTE_LATER);
						}
						else
						{
							ein_->set_active_object_finished();
							scheduler_next_object();
						}
					}
					else //task_number 6
					{
						temp_state.sub.two=fsm::WAIT;						state_queue.push_back(temp_state);
						temp_state.sub.two=fsm::LOCATE_OBJECT_GLOBAL;		state_queue.push_back(temp_state);
						temp_state.sub.two=fsm::GET_GRASPING_POSE_T6;       state_queue.push_back(temp_state);
						temp_state.sub.two=fsm::MOVE_TO_OBJECT_T6;			state_queue.push_back(temp_state);
						temp_state.sub.two=fsm::MOVE_TO_TARGET_ZONE_T6;  	state_queue.push_back(temp_state);
						temp_state.sub.two=fsm::HOMING;						state_queue.push_back(temp_state);
						temp_state.sub.two=fsm::NEW_OBJECT_T6;				state_queue.push_back(temp_state);
					}
					temp_state.sub.two=fsm::SCHEDULER;						state_queue.push_back(temp_state);
				}
				scheduler_printqueue(); //print queue to console for debugging purposes
				break;

			case fsm::PAUSE:
				if(pause_state_==FINISHEDWITHERROR)	//user wants to exit
				{
					state_queue.clear();	//throw plan away
					temp_state.sub.one=fsm::STOP_SIM;						state_queue.push_back(temp_state);
					pause_state_=FINISHED;
				}
				break;

			case fsm::HOMING:
				if(homing_state_==FINISHEDWITHERROR)
				{
					scheduler_error_homing();
				}
				break;

			case fsm::LOCATE_OBJECT_GLOBAL:
				if(locate_object_global_state_==FINISHEDWITHERROR)
				{
					scheduler_error_locate_object_global();
				}
				break;

			case fsm::LOCATE_ALL_OBJECTS_GLOBAL:
				if(locate_all_objects_global_state_==FINISHEDWITHERROR)
				{
					scheduler_error_locate_all_objects_global();
				}
				break;

			case fsm::GET_GRASPING_POSE:
				if(get_grasping_pose_state_==FINISHEDWITHERROR)
				{
					//just skip that object
					ROS_INFO("Statemachine-Errorhandler: skipping object");
					get_grasping_pose_state_=OPEN;
					scheduler_skip_object();
				}
				break;
			case fsm::GET_GRASPING_POSE_T5:
				if(get_grasping_poseT5_state_==FINISHEDWITHERROR)
				{
					//just start the state again
					ROS_INFO("Statemachine-Errorhandler: skipping object");
					get_grasping_poseT5_state_=OPEN;
					scheduler_skip_object();
				}
				break;
			case fsm::GET_GRASPING_POSE_T6:
				if(get_grasping_poseT6_state_==FINISHEDWITHERROR)
				{
					//just start the state again
					ROS_INFO("Statemachine-Errorhandler: skipping object");
					get_grasping_poseT6_state_=OPEN;
					scheduler_skip_object();
				}
				break;
			case fsm::GRIPPER_RELEASE:
				if(gripper_release_state_==FINISHEDWITHERROR)
				{
					scheduler_error_gripper_release();
				}
				if(set_object_load_state_==FINISHEDWITHERROR)
				{
					//just start the state again
					ROS_INFO("Statemachine-Errorhandler: restarting state");
					set_object_load_state_=OPEN;
				}
				break;
			case fsm::GRIPPER_CLOSE:
				if(gripper_close_state_==FINISHEDWITHERROR)
				{
					scheduler_error_gripper_close();
				}
				if(set_object_load_state_==FINISHEDWITHERROR)
				{
					//just start the state again
					ROS_INFO("Statemachine-Errorhandler: restarting state");
					set_object_load_state_=OPEN;
				}
				break;

			case fsm::MOVE_TO_OBJECT_VISION:
				if(move_to_object_vision_state_==FINISHEDWITHERROR)
				{
					scheduler_error_move_to_object_vision();
				}
				break;

			case fsm::MOVE_TO_OBJECT_SAFE:
				if(move_to_object_safe_state_==FINISHEDWITHERROR)
				{
					scheduler_error_move_to_object_safe();
				}
				break;

			case fsm::MOVE_TO_OBJECT_T6:
				if(move_to_object_t6_state_==FINISHEDWITHERROR)
				{
					msg_warn("move_to_object_t6_state_==FINISHEDWITHERROR, IMPLEMENT ME");
					move_to_object_t6_state_=FINISHED;
				}
				break;

			case fsm::MOVE_TO_TARGET_ZONE_T6:
				if(move_to_target_zone_t6_state_==FINISHEDWITHERROR)
				{
					msg_warn("move_to_target_zone_t6_state_==FINISHEDWITHERROR, IMPLEMENT ME");
					move_to_target_zone_t6_state_=FINISHED;
				}
				break;

			case fsm::MOVE_TO_TARGET_ZONE_VISION:
				if(move_to_target_zone_vision_state_==FINISHEDWITHERROR)
				{
					scheduler_error_move_to_target_zone_vision();
				}
				break;
			case fsm::MOVE_TO_TARGET_ZONE_SAFE:
				if(move_to_target_zone_safe_state_==FINISHEDWITHERROR)
				{
					scheduler_error_move_to_target_zone_safe();
				}
				break;
			case fsm::CHECK_OBJECT_FINISHED:
				if(check_object_finished_state_==FINISHEDWITHERROR)
				{
					scheduler_error_check_object_finished();
				}
				break;
			case fsm::LOCATE_OBJECT_CLOSE_RANGE:
				if(locate_object_close_range_state_==FINISHEDWITHERROR)
				{
					//-----------------------
					ROS_INFO("Statemachine-Errorhandler: skipping this state...");
					locate_object_close_range_state_ =FINISHED;
					//-----------------------
				}
				break;
			case fsm::CHECK_OBJECT_GRIPPED:
				if(check_object_gripped_state_==FINISHEDWITHERROR)
				{
					scheduler_error_check_object_gripped();
				}
				break;
			case fsm::NEW_OBJECT_T6:
				if(new_object_t6_state_==FINISHEDWITHERROR)
				{
					msg_warn("Statemachine-Errorhandler: failed to call request object -> retry");
					new_object_t6_state_=OPEN;
				}
				break;
			case fsm::GRAB_OBJECT:
			{
				switch(state_.sub.three)
				{
				case fsm::MOVE_TO_OBJECT:
					if(move_to_object_state_==FINISHEDWITHERROR)
					{
						scheduler_error_move_to_object();
					}
					break;
				case fsm::GRIPPER_CLOSE:
					if(gripper_close_state_==FINISHEDWITHERROR)
					{
						scheduler_error_gripper_close();
					}
					if(set_object_load_state_==FINISHEDWITHERROR)
					{
						//just start the state again
						ROS_INFO("Statemachine-Errorhandler: restarting state");
						set_object_load_state_=OPEN;
					}
					break;
				case fsm::MOVE_TO_OBJECT_SAFE:
					if(move_to_object_safe_state_==FINISHEDWITHERROR)
					{
						scheduler_error_move_to_object_safe();
					}
					break;
				default:
					ROS_INFO("Scheduler: Don't know what to do! (%s)",get_state_name(state_).c_str()); break;
				}
				break;
			} //case fsm::GRAB_OBJECT

			case fsm::PLACE_OBJECT:
			{
				switch(state_.sub.three)
				{
				case fsm::MOVE_TO_TARGET_ZONE:
					if(move_to_target_zone_state_==FINISHEDWITHERROR)
					{
						scheduler_error_move_to_target_zone();
					}
					break;
				case fsm::GRIPPER_RELEASE:
					if(gripper_release_state_==FINISHEDWITHERROR)
					{
						scheduler_error_gripper_release();
					}
					if(set_object_load_state_==FINISHEDWITHERROR)
					{
						//just start the state again
						ROS_INFO("Statemachine-Errorhandler: restarting state");
						set_object_load_state_=OPEN;
					}
					break;
				case fsm::MOVE_TO_TARGET_ZONE_SAFE:
					if(move_to_target_zone_safe_state_==FINISHEDWITHERROR)
					{
						scheduler_error_move_to_target_zone_safe();
					}
					break;
				default:
					ROS_INFO("Scheduler: Don't know what to do! (%s)",get_state_name(state_).c_str()); break;
				}
				break;
			}//case fsm::PLACE_OBJECT

			default:
				ROS_INFO("Scheduler: Don't know what to do! (%s)",get_state_name(state_).c_str()); break;

			}
		break;
		default:
			ROS_INFO("Scheduler: Don't know what to do! (%s)",get_state_name(state_).c_str()); break;
	}
}

void Statemachine::scheduler_printqueue()
{
	ROS_INFO("Scheduler: actual state-queue:");
	for(int i=0; i<state_queue.size(); i++)
		ROS_INFO("[%d] %s",i,get_state_name(state_queue.at(i)).c_str());
}

void Statemachine::scheduler_skip_object()
{
	fsm::fsm_state_t temp_state;	//temporary state variable for scheduler

	if(ein_->is_active_object_last_object())
	{
		ROS_INFO("Scheduler: skipping object -> last object reached -> back to vision");

		state_queue.clear();
		temp_state.sub.one=fsm::SCHEDULER;			state_queue.push_back(temp_state);
		scheduler_next();
	}
	else
	{
		ROS_INFO("Scheduler: skipping object -> next object");
		ein_->select_new_object();
		scheduler_next_object();

		//clear the queue
		state_queue.clear();
		temp_state.sub.one=fsm::SOLVE_TASK;
		temp_state.sub.two=fsm::GRIPPER_RELEASE;		state_queue.push_back(temp_state);
		temp_state.sub.two=fsm::HOMING;					state_queue.push_back(temp_state);
		temp_state.sub.two=fsm::SCHEDULER;				state_queue.push_back(temp_state);
		scheduler_next();
	}
}

void Statemachine::scheduler_skip_explore()
{
	//delete explore sequence
	state_queue.clear();

	fsm::fsm_state_t temp_state;
	temp_state.sub.one=fsm::EXPLORE_ENVIRONMENT;
	temp_state.sub.two=fsm::EXPLORE_ENVIRONMENT_CHECK;	state_queue.push_back(temp_state);
	temp_state.sub.one=fsm::SOLVE_TASK;
	temp_state.sub.two=fsm::HOMING;						state_queue.push_back(temp_state);
	temp_state.sub.two=fsm::SCHEDULER;					state_queue.push_back(temp_state);
}

void Statemachine::scheduler_next_object()
{
	if(ein_->all_finished())
	{
		ROS_INFO("scheduler_next_object() and all objects finished!");
		fsm::fsm_state_t temp_state;
		temp_state.sub.one=fsm::SOLVE_TASK;
		temp_state.sub.two=fsm::SCHEDULER;

		state_queue.clear();
		state_queue.push_back(temp_state);
	}
	else
	{
		cur_obj_ = ein_->get_active_object();
		cur_zone_ = ein_->get_active_target_zone();
		if(active_task_number_==5)
			cur_target_pose_ = ein_->get_active_target_pose();

		ROS_INFO("new object-name: %s",cur_obj_.name.c_str());
		cur_object_type_=OBJECT_UNKNOWN;
		if(cur_obj_.nr_shapes==1 && (!cur_obj_.shape[0].type.compare("cylinder")))
		{
			cur_object_type_=OBJECT_CYLINDER;
			ROS_INFO("new object-type: CYLINDER");
		}
		if(cur_obj_.nr_shapes==1 && (!cur_obj_.shape[0].type.compare("box")))
		{
			cur_object_type_=OBJECT_CUBE;
			ROS_INFO("new object-type: CUBE");
		}
		if(cur_obj_.nr_shapes==3)
		{
			cur_object_type_=OBJECT_HANDLE;
			ROS_INFO("new object-type: HANDLE");
		}

		//reset speed modification
		speed_mod_=0;
	}
}

void Statemachine::scheduler_grasp_object(bool start)
{
	ROS_INFO("Scheduler: schedule grasp object sequence");

	fsm::fsm_state_t temp_state;	//temporary state variable for scheduler

	if(start)
	{
		std::vector<fsm::fsm_state_t>::iterator it = state_queue.begin();

		temp_state.sub.one=fsm::SOLVE_TASK;
		temp_state.sub.two=fsm::GRIPPER_RELEASE;				state_queue.insert(it,temp_state);
		temp_state.sub.two=fsm::MOVE_TO_OBJECT_VISION;			state_queue.insert(++it,temp_state);
		temp_state.sub.two=fsm::LOCATE_OBJECT_CLOSE_RANGE;		state_queue.insert(++it,temp_state);
		temp_state.sub.two=fsm::MOVE_TO_OBJECT_SAFE;			state_queue.insert(++it,temp_state);
		temp_state.sub.two=fsm::GRAB_OBJECT;
		temp_state.sub.three=fsm::MOVE_TO_OBJECT;			state_queue.insert(++it,temp_state);
		temp_state.sub.three=fsm::GRIPPER_CLOSE;			state_queue.insert(++it,temp_state);
		temp_state.sub.three=fsm::MOVE_TO_OBJECT_SAFE;		state_queue.insert(++it,temp_state);
		temp_state.sub.two=fsm::CHECK_OBJECT_GRIPPED;			state_queue.insert(++it,temp_state);
	}
	else
	{
		temp_state.sub.one=fsm::SOLVE_TASK;
		temp_state.sub.two=fsm::GRIPPER_RELEASE;				state_queue.push_back(temp_state);
		temp_state.sub.two=fsm::MOVE_TO_OBJECT_VISION;			state_queue.push_back(temp_state);
		temp_state.sub.two=fsm::LOCATE_OBJECT_CLOSE_RANGE;		state_queue.push_back(temp_state);
		temp_state.sub.two=fsm::MOVE_TO_OBJECT_SAFE;			state_queue.push_back(temp_state);
		temp_state.sub.two=fsm::GRAB_OBJECT;
		temp_state.sub.three=fsm::MOVE_TO_OBJECT;			state_queue.push_back(temp_state);
		temp_state.sub.three=fsm::GRIPPER_CLOSE;			state_queue.push_back(temp_state);
		temp_state.sub.three=fsm::MOVE_TO_OBJECT_SAFE;		state_queue.push_back(temp_state);
		temp_state.sub.two=fsm::CHECK_OBJECT_GRIPPED;			state_queue.push_back(temp_state);
	}
}

void Statemachine::scheduler_place_object(bool start)
{
	ROS_INFO("Scheduler: schedule place object sequence");

	fsm::fsm_state_t temp_state;	//temporary state variable for scheduler

	if(start)
	{
		std::vector<fsm::fsm_state_t>::iterator it = state_queue.begin();

		temp_state.sub.one=fsm::SOLVE_TASK;
		temp_state.sub.two=fsm::MOVE_TO_TARGET_ZONE_SAFE;		state_queue.insert(it,temp_state);
		temp_state.sub.two=fsm::PLACE_OBJECT;
		temp_state.sub.three=fsm::MOVE_TO_TARGET_ZONE;		    state_queue.insert(++it,temp_state);
		temp_state.sub.three=fsm::GRIPPER_RELEASE;			    state_queue.insert(++it,temp_state);
		temp_state.sub.three=fsm::MOVE_TO_TARGET_ZONE_SAFE;	    state_queue.insert(++it,temp_state);
		temp_state.sub.two=fsm::MOVE_TO_TARGET_ZONE_VISION;		state_queue.insert(++it,temp_state);
		temp_state.sub.two=fsm::CHECK_OBJECT_FINISHED;			state_queue.insert(++it,temp_state);
	}
	else
	{
		temp_state.sub.one=fsm::SOLVE_TASK;
		temp_state.sub.two=fsm::MOVE_TO_TARGET_ZONE_SAFE;		state_queue.push_back(temp_state);
		temp_state.sub.two=fsm::PLACE_OBJECT;
		temp_state.sub.three=fsm::MOVE_TO_TARGET_ZONE;		    state_queue.push_back(temp_state);
		temp_state.sub.three=fsm::GRIPPER_RELEASE;			    state_queue.push_back(temp_state);
		temp_state.sub.three=fsm::MOVE_TO_TARGET_ZONE_SAFE;	    state_queue.push_back(temp_state);
		temp_state.sub.two=fsm::MOVE_TO_TARGET_ZONE_VISION;		state_queue.push_back(temp_state);
		temp_state.sub.two=fsm::CHECK_OBJECT_FINISHED;			state_queue.push_back(temp_state);
	}
}

void Statemachine::scheduler_error_homing()
{
	switch(state_.sub.event_two)
	{
	case fsm::NO_DK_SOL:
	case fsm::MOTION_PLANNING_ERROR:
		msg_warn("Statemachine-Errorhandler: gen. motion planning error -> retry");
		//retry once, otherwise skip homing:
		homing_state_=OPEN;
		homing_counter_++;

		if(homing_counter_ > 1)
		{
			homing_counter_=0;
			scheduler_next();
		}
		break;

	case fsm::MAX_LIMIT_REACHED:
		homing_state_=OPEN;
		msg_error("IMPLEMENT ME!!!!!!!!");

		//temporary:
		if(state_.sub.one!=fsm::EXPLORE_ENVIRONMENT)
			scheduler_skip_object();
		else
			scheduler_next();
		break;

	case fsm::STOP_COND:
		msg_warn("Statemachine-Errorhandler: stop cond -> try slower");

		homing_state_=OPEN;
		if(speed_mod_< 0.66)
			speed_mod_+=0.33;
		else
			speed_mod_= 0.8;
		break;
	default:
		msg_error("Statemachine Errorhandler: Unknown error!");
		homing_state_=FINISHED;
		break;
	}
}
void Statemachine::scheduler_error_move_to_object_vision()
{
	//move_to_object_vision_state_ =OPEN;
	switch(state_.sub.event_two)
	{
	case fsm::NO_DK_SOL:
	case fsm::MOTION_PLANNING_ERROR:
		msg_warn("Statemachine-Errorhandler: gen. motion planning error -> retry");
		//retry:
		move_to_object_vision_state_=OPEN;
		move_to_object_vision_counter_++;

		if(move_to_object_vision_counter_ > 1)
		{
			move_to_object_vision_counter_=0;
			scheduler_next();
			//skip locate object close range
			scheduler_next();
			//skip get grasping pose
			scheduler_next();
		}
		break;

	case fsm::NO_IK_SOL:
		msg_warn("Statemachine-Errorhandler: no ik sol -> try next pose");

		if(selected_object_pose_ < object_vision_pose.size()-1)
		{
			move_to_object_vision_state_=OPEN;
			selected_object_pose_++;
		}
		else
		{
			//skip vision pose and try to grasp the object

			move_to_object_vision_state_=OPEN;
			selected_object_pose_=0;
			scheduler_next();
			//skip locate object close range
			scheduler_next();
			//skip get grasping pose
			scheduler_next();
		}
		break;

	case fsm::MAX_LIMIT_REACHED:
		move_to_object_vision_state_=OPEN;
		msg_error("IMPLEMENT ME!!!!!!!!");

		//temporary:
		scheduler_skip_object();
		break;

	case fsm::STOP_COND:
		msg_warn("Statemachine-Errorhandler: stop cond -> try slower");

		move_to_object_vision_state_=OPEN;
		if(speed_mod_< 0.66)
			speed_mod_+=0.33;
		else
			speed_mod_= 0.8;
		break;
	default:
		msg_error("Statemachine Errorhandler: Unknown error!");
		move_to_object_vision_state_=FINISHED;

		break;
	}
}
void Statemachine::scheduler_error_move_to_object_safe()
{
	//move_to_object_safe_state_ =OPEN;
	switch(state_.sub.event_two)
	{
	case fsm::NO_DK_SOL:
	case fsm::MOTION_PLANNING_ERROR:
		msg_warn("Statemachine-Errorhandler: gen. motion planning error -> retry");
		//retry:
		move_to_object_safe_state_=OPEN;
		move_to_object_safe_counter_++;

		if(move_to_object_safe_counter_ > 1)
		{
			move_to_object_safe_counter_=0;
			scheduler_skip_object();
		}
		break;

	case fsm::NO_IK_SOL:
		msg_warn("Statemachine-Errorhandler: no ik sol -> try next pose");

		if(selected_object_pose_ < object_safe_pose.size()-1)
		{
			move_to_object_safe_state_=OPEN;
			selected_object_pose_++;
		}
		else
		{
			//skip vision pose and try to grasp the object
			move_to_object_safe_state_=OPEN;
			scheduler_skip_object();
		}
		break;

	case fsm::MAX_LIMIT_REACHED:
		move_to_object_safe_state_=OPEN;
		msg_error("IMPLEMENT ME!!!!!!!!");

		//temporary:
		scheduler_skip_object();
		break;

	case fsm::STOP_COND:
		msg_warn("Statemachine-Errorhandler: stop cond -> try slower");

		move_to_object_safe_state_=OPEN;
		if(speed_mod_< 0.66)
			speed_mod_+=0.33;
		else
			speed_mod_= 0.8;
		break;
	default:
		msg_error("Statemachine Errorhandler: Unknown error!");
		move_to_object_safe_state_=FINISHED;
		break;
	}
}
void Statemachine::scheduler_error_move_to_target_zone_vision()
{
	//move_to_target_zone_vision_state_ =OPEN;
	switch(state_.sub.event_two)
	{
	case fsm::NO_DK_SOL:
	case fsm::MOTION_PLANNING_ERROR:
	case fsm::NO_IK_SOL:
		msg_warn("Statemachine-Errorhandler: gen. motion planning error -> retry");
		//retry:
		move_to_target_zone_vision_state_=OPEN;
		move_to_target_zone_vision_counter_++;

		if(move_to_target_zone_vision_counter_ > 1)
		{
			move_to_target_zone_vision_counter_=0;
			//skip final vision check
			check_object_finished_state_=FINISHED;
			scheduler_next();
		}
		break;

//	case fsm::NO_IK_SOL:
//		msg_warn("Statemachine-Errorhandler: no ik sol -> try next pose");
//
//		if(selected_target_pose_ < target_vision_pose.size()-1)
//		{
//			move_to_target_zone_vision_state_=OPEN;
//			selected_target_pose_++;
//		}
//		else
//		{
//			//skip vision pose
//			move_to_target_zone_vision_state_=OPEN;
//			//skip final vision check
//			check_object_finished_state_=FINISHED;
//			scheduler_next();
//		}
//		break;

	case fsm::MAX_LIMIT_REACHED:
		move_to_target_zone_vision_state_=OPEN;
		msg_error("IMPLEMENT ME!!!!!!!!");

		//temporary:
		scheduler_skip_object();
		break;

	case fsm::STOP_COND:
		msg_warn("Statemachine-Errorhandler: stop cond -> try slower");

		move_to_target_zone_vision_state_=OPEN;
		if(speed_mod_< 0.66)
			speed_mod_+=0.33;
		else
			speed_mod_= 0.8;
		break;
	default:
		msg_error("Statemachine Errorhandler: Unknown error!");
		move_to_target_zone_vision_state_=FINISHED;
		break;
	}
}
void Statemachine::scheduler_error_move_to_target_zone_safe()
{
	//move_to_target_zone_safe_state_ =OPEN;
	switch(state_.sub.event_two)
	{
	case fsm::NO_DK_SOL:
	case fsm::MOTION_PLANNING_ERROR:
		msg_warn("Statemachine-Errorhandler: gen. motion planning error -> retry");
		//retry:
		move_to_target_zone_safe_state_=OPEN;
		move_to_target_zone_safe_counter_++;

		if(move_to_target_zone_safe_counter_ > 1)
		{
			move_to_target_zone_safe_counter_=0;
			scheduler_skip_object();
		}
		break;

	case fsm::NO_IK_SOL:
		msg_warn("Statemachine-Errorhandler: no ik sol -> try next pose");

		if(selected_target_pose_ < target_safe_pose.size()-1)
		{
			move_to_target_zone_safe_state_=OPEN;
			selected_target_pose_++;
		}
		else
		{
			//skip vision pose and try to grasp the object
			move_to_target_zone_safe_state_=OPEN;
			scheduler_skip_object();
		}
		break;

	case fsm::MAX_LIMIT_REACHED:
		move_to_target_zone_safe_state_=OPEN;
		msg_error("IMPLEMENT ME!!!!!!!!");

		//temporary:
		scheduler_skip_object();
		break;

	case fsm::STOP_COND:
		msg_warn("Statemachine-Errorhandler: stop cond -> try slower");

		move_to_target_zone_safe_state_=OPEN;
		if(speed_mod_< 0.66)
			speed_mod_+=0.33;
		else
			speed_mod_= 0.8;
		break;
	default:
		msg_error("Statemachine Errorhandler: Unknown error!");
		move_to_target_zone_safe_state_=FINISHED;
		break;
	}
}
void Statemachine::scheduler_error_move_to_object()
{
	switch(state_.sub.event_three)
	{
	case fsm::NO_DK_SOL:
	case fsm::MOTION_PLANNING_ERROR:
	case fsm::NO_IK_SOL:
		msg_warn("Statemachine-Errorhandler: gen. motion planning error -> retry");
		//retry:
		move_to_object_state_=OPEN;
		move_to_object_counter_++;

		if(move_to_object_counter_ > 3)
		{
			move_to_object_counter_=0;
			//try to close the gripper
			move_to_object_state_=FINISHED;
		}
		break;

//	case fsm::NO_IK_SOL:
//		msg_warn("Statemachine-Errorhandler: no ik sol -> try next pose");
//
//		if(selected_object_pose_ < object_vision_pose.size()-1)
//		{
//			move_to_object_state_=OPEN;
//			selected_object_pose_++;
//		}
//		else
//		{
//			//skip vision pose and try to grasp the object
//			move_to_object_state_=FINISHED;
//		}
//		break;

	case fsm::MAX_LIMIT_REACHED:
		move_to_object_state_=FINISHED;
		msg_error("IMPLEMENT ME!!!!!!!!");

		break;

	case fsm::STOP_COND:
		msg_warn("Statemachine-Errorhandler: stop cond -> try slower");

		//not smart?!
		//		move_to_object_safe_state_=OPEN;
		//		if(speed_mod_ < 0.66)
		//			speed_mod_+=0.33;
		//		else
		//			move_to_object_safe_state_=FINISHED;

		//try to close the gripper
		move_to_object_state_=FINISHED;
		break;
	default:
		msg_error("Statemachine Errorhandler: Unknown error!");
		move_to_object_state_=FINISHED;
		break;
	}
}
void Statemachine::scheduler_error_move_to_target_zone()
{
	//move_to_target_zone_state_=OPEN;
	switch(state_.sub.event_three)
	{
	case fsm::NO_DK_SOL:
	case fsm::MOTION_PLANNING_ERROR:
		msg_warn("Statemachine-Errorhandler: gen. motion planning error -> retry");
		//retry:
		move_to_target_zone_state_=OPEN;
		move_to_target_zone_counter_++;

		if(move_to_target_zone_counter_ > 1)
		{
			move_to_target_zone_counter_=0;
			//try to close the gripper
			move_to_target_zone_state_=FINISHED;
		}
		break;

	case fsm::NO_IK_SOL:
		msg_warn("Statemachine-Errorhandler: no ik sol -> try next pose");

		if(selected_object_pose_ < object_vision_pose.size()-1)
		{
			move_to_target_zone_state_=OPEN;
			selected_object_pose_++;
		}
		else
		{
			//skip vision pose and try to grasp the object
			move_to_target_zone_state_=FINISHED;
		}
		break;

	case fsm::MAX_LIMIT_REACHED:
		move_to_target_zone_state_=FINISHED;
		msg_error("IMPLEMENT ME!!!!!!!!");

		break;

	case fsm::STOP_COND:
		msg_warn("Statemachine-Errorhandler: stop cond -> try slower");

		move_to_target_zone_state_=OPEN;
		if(speed_mod_ < 0.66)
			speed_mod_+=0.33;
		else
			move_to_target_zone_state_=FINISHED;
		break;

	default:
		msg_error("Statemachine Errorhandler: Unknown error!");
		move_to_target_zone_state_=FINISHED;
		break;
	}
}
void Statemachine::scheduler_error_check_object_gripped()
{
	//increase error counter
	check_object_gripped_counter_++;
	if(check_object_gripped_counter_ > 1)
		state_.sub.event_two = fsm::SKIP_OBJECT;

	switch(state_.sub.event_two)
	{
	case fsm::RETRY:
		msg_warn("Statemachine-Errorhandler: retry check");
		check_object_gripped_state_ =OPEN;
		//nop
		break;

	case fsm::OBJECT_LOST:
		scheduler_next();
		scheduler_grasp_object(EXECUTE_NOW);

		check_object_gripped_state_ = OPEN;
		msg_warn("Statemachine-Errorhandler: Object lost detected... retrying gripping routine");

		scheduler_printqueue(); //print queue to console for debugging purposes
		scheduler_next();
		break;

	case fsm::SKIP_OBJECT:
		msg_warn("Statemachine-Errorhandler: skip object");
		check_object_gripped_state_ = OPEN;
		scheduler_skip_object();
		break;

	default:
		msg_error("Statemachine Errorhandler: Unknown error!");
		check_object_gripped_state_ = FINISHED;
		break;
	}
}
void Statemachine::scheduler_error_check_object_finished()
{
	//skip current object and try next one

	msg_warn("Statemachine-Errorhandler: object not in target zone -> try next object");
	//and get next one
	ein_->select_new_object();
	scheduler_next_object();


	//publish object state for motion planning
	publish_obj_state(OBJ_NOT_LOCATED);

	//==============================================
	scheduler_next();
	//==============================================
	//reset state
	check_object_finished_state_=OPEN;
}
void Statemachine::scheduler_error_gripper_close()
{
	switch(state_.sub.event_three)
	{
	case fsm::SIM_SRV_NA:
	case fsm::GRIPPING_ERROR:

		msg_warn("Statemachine-Errorhandler: gripping error -> retry");
		gripper_close_counter_++;
		if(gripper_close_counter_ > 1)
		{
			gripper_close_counter_=0;
			gripper_close_state_=OPEN;

			scheduler_skip_object();
		}
		break;
	case fsm::STOP_COND:
		msg_warn("Statemachine-Errorhandler: stop cond -> try slower");
		//set object load and continue
		gripper_close_state_=FINISHED;
		break;
	default:
		msg_error("Statemachine Errorhandler: Unknown error!");
		gripper_close_state_=FINISHED;
		break;
	}
}
void Statemachine::scheduler_error_gripper_release()
{
	switch(state_.sub.event_three)
	{
	case fsm::SIM_SRV_NA:
	case fsm::GRIPPING_ERROR:
	{
		gripper_close_counter_++;
		gripper_release_state_=OPEN;
		if(gripper_close_counter_ > 1)
		{
			gripper_close_counter_=0;
			gripper_release_state_=FINISHED;

			//insert gripper release after move_to_target_zone_safe
			fsm::fsm_state_t temp_state;
			std::vector<fsm::fsm_state_t>::iterator it = state_queue.begin();

			temp_state.sub.one=fsm::SOLVE_TASK;
			temp_state.sub.two=fsm::PLACE_OBJECT;
			temp_state.sub.three=fsm::GRIPPER_RELEASE;			state_queue.insert(it+1,temp_state);
		}
		break;
	}
	case fsm::STOP_COND:
	{
		//set object load and continue
		gripper_release_state_=FINISHED;

		//insert gripper release after move_to_target_zone_safe
		fsm::fsm_state_t temp_state;
		std::vector<fsm::fsm_state_t>::iterator it = state_queue.begin();

		temp_state.sub.one=fsm::SOLVE_TASK;
		temp_state.sub.two=fsm::PLACE_OBJECT;
		temp_state.sub.three=fsm::GRIPPER_RELEASE;			state_queue.insert(it+1,temp_state);
		break;
	}
	default:
		msg_error("Statemachine Errorhandler: Unknown error!");
		gripper_release_state_=FINISHED;
		break;
	}
}
void Statemachine::scheduler_error_watch_scene()
{
	switch(state_.sub.event_one)
	{
	case fsm::DATA_ERROR:
		//try again (once)
		watch_scene_state_=OPEN;

		watch_scene_counter_++;
		if(watch_scene_counter_>1)
		{
			scheduler_next();
			watch_scene_counter_=0;
			msg_error("JUSTIN please plug in the cameras! =)");
		}
		break;

	case fsm::VISION_ERROR:
	case fsm::SIM_SRV_NA:
		watch_scene_state_=OPEN;
		scheduler_next();
		break;

	default:
		msg_error("Statemachine Errorhandler: Unknown error!");
		watch_scene_state_=FINISHED;
		break;
	}
}
void Statemachine::scheduler_error_explore_environment_motion()
{
	switch(state_.sub.event_two)
	{
	case fsm::NO_DK_SOL:
	case fsm::MOTION_PLANNING_ERROR:
	case fsm::MAX_LIMIT_REACHED:
	case fsm::NO_IK_SOL:
		//skip this explore pose
		explore_environment_motion_state_=OPEN;
		msg_warn("skip this explore pose and DO NOT insert homing");
		active_goal_++;
		scheduler_next();
#warning Homing for explore poses disabled
//		fsm::fsm_state_t temp_state;
//		temp_state.sub.one=fsm::SOLVE_TASK;
//		temp_state.sub.two=fsm::HOMING;			state_queue.insert(state_queue.begin(),temp_state);
		scheduler_next();
		break;

	case fsm::STOP_COND:
		msg_warn("Statemachine-Errorhandler: stop cond -> try slower");

		explore_environment_motion_state_=OPEN;
		if(speed_mod_< 0.66)
			speed_mod_+=0.33;
		else
			speed_mod_= 0.8;
		break;
	default:
		msg_error("Statemachine Errorhandler: Unknown error!");
		explore_environment_motion_state_=FINISHED;
		break;
	}
}
void Statemachine::scheduler_error_explore_environment_image()
{
	switch(state_.sub.event_one)
	{
	case fsm::DATA_ERROR:
		msg_warn("Statemachine-Errorhandler: data error -> try again once");
		//try again (once)
		explore_environment_image_state_=OPEN;

		explore_environment_image_counter_++;
		if(explore_environment_image_counter_>1)
		{
			scheduler_next();
			explore_environment_image_counter_=0;
			msg_error("JUSTIN please plug in the cameras! =)");
		}
		break;

	case fsm::VISION_ERROR:
	case fsm::SIM_SRV_NA:
		explore_environment_image_state_=OPEN;
		scheduler_next();
		break;

	default:
		msg_error("Statemachine Errorhandler: Unknown error!");
		explore_environment_image_state_=FINISHED;
		break;
	}
}
void Statemachine::scheduler_error_locate_object_global()
{
	switch(state_.sub.event_two)
	{
	case fsm::SKIP_OBJECT:
		msg_warn("Statemachine-Errorhandler: skip object");
		locate_object_global_state_=OPEN;
		locate_object_global_counter_=0;
		scheduler_skip_object();
		break;

	case fsm::POSE_NOT_FOUND:
		msg_warn("Statemachine-Errorhandler: pose not found -> decrease precision and try again");
		//try it again with lower precision
		locate_object_global_state_=OPEN;
		locate_object_global_counter_++;
		if(locate_object_global_counter_>3)
		{
			locate_object_global_counter_=0;
			scheduler_skip_object();
		}
		break;

	default:
		msg_error("Statemachine Errorhandler: Unknown error!");
		locate_object_global_state_=OPEN;
		scheduler_skip_object();
		break;
	}
}
void Statemachine::scheduler_error_locate_all_objects_global()
{
	msg_warn("Statemachine-Errorhandler: pose not found and try next object");
	//try it again with lower precision
	locate_all_objects_global_state_=RUNNING;
	reached_active_goal_=true;
}

int Statemachine::tick()
{
	counter++; 	//tick counter (only for debugging)

	//check simulation time if sim is running
	if(-1 == check_time())
	{
		msg_error("check time failed!");
	}

	switch(state_.sub.one)
	{
	case fsm::INITIAL_STATE:
		scheduler_schedule();	//Make an initial schedule
		scheduler_next();
		return 0;

	case fsm::PAUSE:
		return pause();

	case fsm::REQUEST_TASK:
		return request_task();

	case fsm::START_SIM:
		return start_sim();

	case fsm::PARSE_YAML:
		return parse_yaml_file();

	case fsm::SCHEDULER:
		scheduler_schedule();	//make a schedule depending on chosen task
		scheduler_next();		//jump to the first state in the queue
		return 0;

	case fsm::WATCH_SCENE:
		return watch_scene();

	case fsm::EXPLORE_ENVIRONMENT:
	{
		switch(state_.sub.two)
		{
		case fsm::PAUSE:
			return pause();

		case fsm::HOMING:
			return homing();

		case fsm::EXPLORE_ENVIRONMENT_INIT:
			return explore_environment_init();

		case fsm::EXPLORE_ENVIRONMENT_MOTION:
			return explore_environment_motion();

		case fsm::EXPLORE_ENVIRONMENT_IMAGE:
			return explore_environment_image();

		case fsm::EXPLORE_ENVIRONMENT_CHECK:
			return explore_environment_check();

		default:
			msg_error("Error. unknown state of level two (EXPLORE_ENVIRONMENT) in tick()");
			return -1;
		}
		break;
	}

	case fsm::SOLVE_TASK:
	{
		switch(state_.sub.two)
		{
		case fsm::PAUSE:
			return pause();

		case fsm::WAIT:
			return wait();

		case fsm::SCHEDULER:
			scheduler_schedule();	//make a schedule
			scheduler_next();		//jump to the first state in the queue
			return 0;

		case fsm::HOMING:
			return homing();

		case fsm::LOCATE_OBJECT_GLOBAL:
			return locate_object_global();

		case fsm::LOCATE_ALL_OBJECTS_GLOBAL:
			return locate_all_objects_global();

		case fsm::GET_GRASPING_POSE:
			return get_grasping_pose();

		case fsm::GET_GRASPING_POSE_T5:
			return get_grasping_poseT5();

		case fsm::GET_GRASPING_POSE_T6:
			return get_grasping_poseT6();

		case fsm::GRIPPER_RELEASE:
			return gripper_release();

		case fsm::GRIPPER_CLOSE:
			return gripper_close();

		case fsm::MOVE_TO_OBJECT_VISION:
			return move_to_object_vision();

		case fsm::MOVE_TO_OBJECT_SAFE:
			return move_to_object_safe();

		case fsm::MOVE_TO_OBJECT_T6:
			return move_to_object_t6();

		case fsm::MOVE_TO_TARGET_ZONE_T6:
			return move_to_target_zone_t6();

		case fsm::MOVE_TO_TARGET_ZONE_VISION:
			return move_to_target_zone_vision();

		case fsm::MOVE_TO_TARGET_ZONE_SAFE:
			return move_to_target_zone_safe();

		case fsm::CHECK_OBJECT_FINISHED:
			return check_object_finished();

		case fsm::CHECK_OBJECT_GRIPPED:
			return check_object_gripped();

		case fsm::NEW_OBJECT_T6:
			return new_object_t6();

		case fsm::LOCATE_OBJECT_CLOSE_RANGE:
			return locate_object_close_range();

		case fsm::GRAB_OBJECT:
		{
			switch(state_.sub.three)
			{
			case fsm::PAUSE:
				return pause();

			case fsm::MOVE_TO_OBJECT:
				return move_to_object();

			case fsm::GRIPPER_CLOSE:
				return gripper_close();

			case fsm::MOVE_TO_OBJECT_SAFE:
				return move_to_object_safe();

			default:
				msg_error("Error. unknown state of level three (SOLVE_TASK->GRAB_OBJECT) in tick()");
				return -1;
			}
			break;
		}

		case fsm::PLACE_OBJECT:
		{
			switch(state_.sub.three)
			{
			case fsm::PAUSE:
				return pause();

			case fsm::MOVE_TO_TARGET_ZONE:
				return move_to_target_zone();

			case fsm::GRIPPER_RELEASE:
				return gripper_release();

			case fsm::MOVE_TO_TARGET_ZONE_SAFE:
				return move_to_target_zone_safe();

			default:
				msg_error("Error. unknown state of level three (SOLVE_TASK->PLACE_OBJECT) in tick()");
				return -1;
			}
			break;
		}

		default:
			msg_error("Error. unknown state of level two (SOLVE_TASK) in tick()");
			return -1;
		}
		break;
	}

	case fsm::STOP_SIM:
		return stop_sim();

	case fsm::RESET:
		return reset();

	default:
		msg_error("Error. unknown state of level one in tick()");
		return -1;
	}
}

int Statemachine::pause()
{
	if(pause_state_==OPEN)
	{
		ROS_INFO("pause() called: OPEN");

		pause_state_=RUNNING;
		cur_obj_ = ein_->get_active_object();
		ROS_INFO("Paused. Current object: %s",cur_obj_.name.c_str());
		ROS_INFO("options:");
		ROS_INFO("--------");
		ROS_INFO("'0'->exit");
		ROS_INFO("'1'->continue");
		ROS_INFO("'2'->next object");
		ROS_INFO("'3'->next object (and set current one to finished)");
		std::cout<<"your choice: ";
		int something=0;
		std::cin>>something;

		switch(something)
		{
		case 0:
			pause_state_=FINISHEDWITHERROR;
			break;
		case 2:
			ein_->select_new_object();
			scheduler_next_object();
			pause_state_=OPEN;
			break;
		case 3:
			ein_->set_active_object_finished();
			scheduler_next_object();
			pause_state_=OPEN;
			break;
		case 1:
		default:
			pause_state_=FINISHED;
			break;
		}
	}
	else if(pause_state_==FINISHED)
	{
		ROS_INFO("pause() called: FINISHED");
		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		pause_state_=OPEN;
	}
	else if(pause_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("pause_state_() called: FINISHEDWITHERROR");
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::request_task_cb()
{
	ROS_INFO("request_task_cb() running");

	if(list_scenes_client_.exists())
	{
		if(list_scenes_client_.call(list_scenes_srv_))
		{
			request_task_state_=FINISHED;
		}
		else
		{
			msg_error("Error. call of list_scenes_client failed");
			request_task_state_=FINISHEDWITHERROR;
		}
	}
	else
	{
		msg_error("Error. list_scenes_client is not available");
		request_task_state_=FINISHEDWITHERROR;
	}

	ROS_INFO("request_task_cb() finished");
}

int Statemachine::request_task()
{
	if(request_task_state_==OPEN)
	{
		ROS_INFO("request_task() called: OPEN");

		request_task_state_=RUNNING;

		//wait for a specific time to let the service start up properly
		ROS_INFO("waiting for 2 seconds to let the list_scenes_-service start...");
		boost::this_thread::sleep( boost::posix_time::milliseconds(2000));
		ROS_INFO("...ok, let's go!");

		//lsc_ = boost::thread(&Statemachine::request_task_cb,this);
		request_task_cb();
	}
	else if(request_task_state_==FINISHED)//lsc_.timed_join(boost::posix_time::seconds(0.0)))
	{
		ROS_INFO("request_task() called: FINISHED");
		// The error_message field of each service response indicates whether an error occurred. An empty string indicates success
		std::string &ls_error_message = list_scenes_srv_.response.error_message;
		if(!ls_error_message.empty())
		{
			msg_error("Error. list_scenes_srv_ failed: %s", ls_error_message.c_str());
			request_task_state_=FINISHEDWITHERROR;
			return 0;
		}
		else
		{
			// Let's print the names of the received scenes
			ROS_INFO("Found the following scenes for the EuRoC C2 Simulation:");

			scenes_.resize(list_scenes_srv_.response.scenes.size());
			scenes_ = list_scenes_srv_.response.scenes;

			nr_scenes_ = scenes_.size();
			for(unsigned int ii = 0; ii < nr_scenes_; ++ii)
			{
				euroc_c2_msgs::Scene &scene = scenes_[ii];
				ROS_INFO("[%2d] - %s", ii,scene.name.c_str());
			}
		}

		//destroy thread
		//lsc_.detach();

#ifndef ONE_TASK
		//find scene for task_name
		std::string task_name_;
		ros::param::get("/task_name",task_name_);
		ROS_INFO("got task_name: %s",task_name_.c_str());

		for(uint16_t ii=0;ii<nr_scenes_;ii++)
		{
			if(strcmp(scenes_[ii].name.c_str(),task_name_.c_str())==0)
			{
				active_scene_=ii;
				break;
			}
		}
		ROS_INFO("Scene number: %d",active_scene_);
#else

		std::cout<<"choose task: ";
		int blub=8;
		std::cin>>blub;
		active_scene_=blub;

#endif
		//check which task has been chosen
		std::string strTaskName = scenes_[active_scene_].name;
		strTaskName=strTaskName.substr(0,5);
		std::transform(strTaskName.begin(), strTaskName.end(), strTaskName.begin(), ::tolower);

		if(strcmp(strTaskName.c_str(),"task1")==0)
		{ active_task_number_=1; }
		else if(strcmp(strTaskName.c_str(),"task2")==0)
		{ active_task_number_=2; }
		else if(strcmp(strTaskName.c_str(),"task3")==0)
		{ active_task_number_=3; }
		else if(strcmp(strTaskName.c_str(),"task4")==0)
		{ active_task_number_=4; }
		else if(strcmp(strTaskName.c_str(),"task5")==0)
		{ active_task_number_=5; }
		else if(strcmp(strTaskName.c_str(),"task6")==0)
		{ active_task_number_=6; }
		else
		{ active_task_number_=0; }
		if(active_task_number_==0)
		{
			ROS_INFO("scene number %d chosen", active_scene_);
			msg_error("Error. unknown task number. can't get it from name: %s",scenes_[active_scene_].name.c_str());
		}
		else
		{
			ROS_INFO("scene number %d chosen (task %d)", active_scene_,active_task_number_);
		}



		switch(active_task_number_)
		{
		case 1:
		case 2:
			planning_mode_.object	= MOVE_IT_7DOF;
			planning_mode_.move_to_object	= MOVE_IT_7DOF;//MOVE_IT_7DOF_MOVE_TO_OBJECT;//
			planning_mode_.target	= MOVE_IT_7DOF;
			planning_mode_.move_to_target_zone	= MOVE_IT_7DOF;//MOVE_IT_7DOF_MOVE_TO_OBJECT;//
			planning_mode_.homing	= HOMING_MOVE_IT_7DOF;
			explore_pose_type_ = EXPLORE_STD_2;
			nr_exp_poses_ = explore_poses_->size(explore_pose_type_);
			max_explore_poses_ = nr_exp_poses_;
			break;
		case 3:
			planning_mode_.object	= MOVE_IT_9DOF;
			planning_mode_.move_to_object	= MOVE_IT_9DOF;//MOVE_IT_9DOF_MOVE_TO_OBJECT;//
			planning_mode_.target	= MOVE_IT_9DOF;
			planning_mode_.move_to_target_zone	= MOVE_IT_9DOF;//MOVE_IT_9DOF_MOVE_TO_OBJECT;//
			planning_mode_.homing	= HOMING_MOVE_IT_7DOF;
			explore_pose_type_ = EXPLORE_STD_2;
			nr_exp_poses_ = explore_poses_->size(explore_pose_type_);
			max_explore_poses_ = nr_exp_poses_;
			break;
		case 4:
			planning_mode_.object	= MOVE_IT_9DOF;
			planning_mode_.move_to_object	= MOVE_IT_9DOF;//MOVE_IT_9DOF_MOVE_TO_OBJECT;//
			planning_mode_.target	= MOVE_IT_9DOF;
			planning_mode_.move_to_target_zone	= MOVE_IT_9DOF;//MOVE_IT_9DOF_MOVE_TO_OBJECT;//
			planning_mode_.homing	= HOMING_MOVE_IT_7DOF;
			explore_pose_type_ = EXPLORE_SNAKE;
			nr_exp_poses_ = explore_poses_->size(explore_pose_type_);
			max_explore_poses_ = 7;
			break;
		case 5:
		case 6:
#warning STATES UEBERARBEITEN
			planning_mode_.object	= MOVE_T6;
			planning_mode_.move_to_object	= MOVE_IT_9DOF;//MOVE_IT_9DOF_MOVE_TO_OBJECT;//
			planning_mode_.target	= MOVE_IT_9DOF_TARGET;
			planning_mode_.move_to_target_zone	= MOVE_IT_9DOF_TARGET;//MOVE_IT_9DOF_MOVE_TO_OBJECT;//
			planning_mode_.homing	= MOVE_T6;
			explore_pose_type_ = EXPLORE_STD_1;
			nr_exp_poses_ = explore_poses_->size(explore_pose_type_);
			max_explore_poses_ = nr_exp_poses_;


			//motion_planning_action_client_->waitForServer();
			break;
		}

		//save the task number to the parameter server of the ROS master
		try
		{
			node_.setParam("active_task_number_", active_task_number_);
			ROS_INFO("saved active_task_number_=%d to parameter server.", active_task_number_);
		}
		catch(...)
		{
			msg_error("Error. could not save active_task_number_ to parameter server.");
		}

		task_active_=true;

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		request_task_state_=OPEN;
	}
	else if(request_task_state_==FINISHEDWITHERROR)
	{
		//destroy thread
		//lsc_.detach();

		ROS_INFO("request_task() called: FINISHEDWITHERROR");
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::start_sim_cb()
{
	ROS_INFO("start_sim_cb() running");

	if(start_simulator_client_.exists())
	{
		if(start_simulator_client_.call(start_simulator_srv_))
		{
			start_sim_state_=FINISHED;
		}
		else
		{
			msg_error("Error. call of start_simulator_client_ failed");
			start_sim_state_=FINISHEDWITHERROR;
		}
	}
	else
	{
		msg_error("Error. start_simulator_client_ is not available");
		start_sim_state_=FINISHEDWITHERROR;
	}

	ROS_INFO("start_sim_cb() finished");

	ROS_INFO("waiting for 3 seconds to let the start_sim_-service start...");
	boost::this_thread::sleep( boost::posix_time::milliseconds(3000));
}

int Statemachine::start_sim()
{
	if(start_sim_state_==OPEN)
	{
		ROS_INFO("start_sim() called: OPEN");

		start_simulator_srv_.request.user_id = "oxofrmbl"; //"C2T14#4547344";
		start_simulator_srv_.request.scene_name = scenes_[active_scene_].name;
		start_sim_state_=RUNNING;
		//lsc_ = boost::thread(&Statemachine::start_sim_cb,this);
		start_sim_cb();
	}
	else if(start_sim_state_==FINISHED)
	{
		ROS_INFO("start_sim() called: FINISHED");
		// Check the response for errors
		std::string &error_message = start_simulator_srv_.response.error_message;
		if(!error_message.empty()){
			msg_error("Error. start_simulator_srv_ failed: %s", error_message.c_str());
			start_sim_state_=FINISHEDWITHERROR;
			return 0;
		}

		//destroy thread
		//lsc_.detach();

		sim_running_=true;
		ROS_INFO("simulator started");

		ROS_INFO("checking for save_log_ service...");
		bool FirstTimeLoop=true;
		bool SaveLogAvailable=false;
		while(!SaveLogAvailable)	//Loop until save log service is available
		{
			if(ros::service::exists(save_log_,false)==true)
			{ SaveLogAvailable=true; }
			if(FirstTimeLoop==true && SaveLogAvailable==false)
			{ ROS_INFO("save_log_ service is not available. Waiting for service..."); }
			FirstTimeLoop=false;
			//wait for a specific time to reduce cpu usage
			boost::this_thread::sleep( boost::posix_time::milliseconds(100));
		}
		ROS_INFO("save_log service is available.");
		//alternative with timeout:
		//ros::service::waitForService(save_log_,ros::Duration(5.0));

		msg_info("checking for action servers:");
		if(vision_action_client_==0x0)
		{
			msg_warn("vision_action_client_->isServerConnected=0!");
			//vor send goal -> isconnected abfragen, wenn nicht zerstören und neustarten!
			//delete vision_action_client_;
			vision_action_client_ = new actionlib::SimpleActionClient<am_msgs::VisionAction>("VisionAction", true);
			msg_warn("action client recreated, waiting for server");
			vision_action_client_->waitForServer();
		}
		else
		{
			msg_info("vision action server available.");
		}

		//check isConnected before send goal -> otherwise destroy and recreate!
		if(motion_planning_action_client_==0x0)
		{
			msg_warn("motion_planning_action_client_->isServerConnected=0!");

			//delete motion_planning_action_client_;
			motion_planning_action_client_ = new actionlib::SimpleActionClient<am_msgs::goalPoseAction>("goalPoseAction", true);
			msg_warn("motion planning action client recreated, waiting for server");
			motion_planning_action_client_->waitForServer();
			motion_planning_action_client_->cancelAllGoals();
		}
		else
		{
			msg_info("motion planning action server available.");
		}

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		start_sim_state_=OPEN;
	}
	else if(start_sim_state_==FINISHEDWITHERROR)
	{
		//destroy thread
		//lsc_.detach();

		ROS_INFO("start_sim() called: FINISHEDWITHERROR");
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

int Statemachine::parse_yaml_file()
{
	if(parse_yaml_file_state_==OPEN)
	{
		ROS_INFO("parse_yaml_file() called: OPEN");

		parse_yaml_file_state_=RUNNING;

		// The start simulator service returns a description of the selected task in yaml format
		std::string &task_yaml_description = start_simulator_srv_.response.description_yaml;

		//parse the yaml file -> content is saved in EurocInput-class
		if(-1==ein_->parse_yaml_file(task_yaml_description, active_task_number_))
		{
			msg_error("Error. parse_yaml_file() failed");
			parse_yaml_file_state_=FINISHEDWITHERROR;
			return 0;
		}
		ROS_INFO("parsing YAML-file finished");


		ROS_INFO("filling TF Broadcast information...");
		try
		{
			broadcaster_->fill_tf_information(ein_);
			ROS_INFO("filling TF info successful.");
		}
		catch (...)
		{
			msg_error("filling up TF information failed.");
			parse_yaml_file_state_=FINISHEDWITHERROR;
			return 0;
		}
		ROS_INFO("publishing static TF...");
		try
		{
			broadcaster_->publish_static_tf();
			ROS_INFO("publishing static TF successful.");
		}
		catch (...)
		{
			msg_error("publishing static TF failed.");
			parse_yaml_file_state_=FINISHEDWITHERROR;
			return 0;
		}

		ROS_INFO("saving object data to parameter server...");
		try
		{
			ein_->save_fixture_to_parameter_server(node_,false);
            ein_->save_conveyorbelt_to_parameter_server(node_,false);
			ein_->save_objects_to_parameter_server(node_,false);
			ein_->save_target_zone_to_parameter_server(node_,false);
			ein_->save_robot_to_parameter_server(node_,false);
		}
		catch(...)
		{
			msg_error("Error. could not save object and robot data to parameter server.");
		}

		parse_yaml_file_state_=FINISHED;
	}
	else if(parse_yaml_file_state_==FINISHED)
	{
		ROS_INFO("parse_yaml_file() called: FINISHED");
		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		parse_yaml_file_state_=OPEN;
	}
	else if(parse_yaml_file_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("parse_yaml_file() called: FINISHEDWITHERROR");
		scheduler_schedule(); //Call for Error-Handling
	}

	return 0;
}

int Statemachine::check_object_finished()
{
	if((target_skip_vision[selected_target_pose_]==1)||(ein_->get_active_object_state()==EurocInput::EIN_OBJ_PARKING)||
			(cur_obj_.nr_shapes>1))
	{
		check_object_finished_state_=FINISHED;
		ROS_INFO("skip check object in zone");
	}
	if(check_object_finished_state_==OPEN)
	{
		ROS_INFO("check_object_finished() called: OPEN");

		am_msgs::VisionGoal goal;
		goal.mode = CHECKING_FOR_OBJECT_IN_TARGET_ZONE;
		goal.object = cur_obj_;
		goal.sensors.resize(ein_->get_nr_sensors());
		for(uint16_t ii=0;ii<ein_->get_nr_sensors();ii++)
		{
			goal.sensors[ii]=ein_->get_sensors(ii);
		}
		goal.target_zone=ein_->get_active_target_zone();

		//send goal to vision-node.
		check_object_finished_state_=RUNNING;
		vision_action_client_->sendGoal(goal,
				boost::bind(&Statemachine::check_object_finished_done,this,_1,_2),
				visionClient::SimpleActiveCallback(),
				visionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::check_object_finished_feedback,this,_1));
		);
	}
	else if(check_object_finished_state_==FINISHED)
	{
		ROS_INFO("check_object_finished() called: FINISHED");

		//set current object finished
		ein_->set_active_object_finished();

		//and get next one
		scheduler_next_object();

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		check_object_finished_state_=OPEN;

	}
	else if(check_object_finished_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("check_object_finished() called: FINISHEDWITHERROR");
		scheduler_schedule(); //Call for Error-Handling
	}

	return 0;
}

void Statemachine::check_object_finished_feedback(const am_msgs::VisionFeedbackConstPtr feedback)
{
	ROS_INFO("check_object_finished_feedback() called");
}

void Statemachine::check_object_finished_done(const actionlib::SimpleClientGoalState& state,
		const am_msgs::VisionResultConstPtr& result)
{
	ROS_INFO("check_object_finished_done() called, state: %s",state.toString().c_str());

	ROS_INFO("Result: object detected: %s, object in zone: %s",
			result->object_detected ? "true":"false",result->object_in_zone ? "true":"false");

	switch(state.state_)
	{
	case actionlib::SimpleClientGoalState::SUCCEEDED:
		if(result->object_in_zone == true)
		{
			check_object_finished_state_=FINISHED;
			//TODO extend to 3 way case!
		}
		else
		{
			msg_error("Error. object is not in the target zone");
			check_object_finished_state_=FINISHEDWITHERROR;
		}
		break;
	case actionlib::SimpleClientGoalState::ACTIVE:
	case actionlib::SimpleClientGoalState::PENDING:
	case actionlib::SimpleClientGoalState::RECALLED:
		break;
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::PREEMPTED:
	case actionlib::SimpleClientGoalState::ABORTED:
		check_object_finished_state_=FINISHEDWITHERROR;
		break;
	default:
		break;
	}
}

void Statemachine::check_object_gripped_cb()
{
	ROS_INFO("check_object_gripped_cb() running");

	if(state_observer_client_.exists())
	{
		if(state_observer_client_.call(obj_picked_up_srv_))
		{
			check_object_gripped_state_=FINISHED;
		}
		else
		{
			msg_error("Error. call of state_observer_client_ failed");
			check_object_gripped_state_=FINISHEDWITHERROR;
		}
	}
	else
	{
		msg_error("Error. state_observer_client_ is not available");
		check_object_gripped_state_=FINISHEDWITHERROR;
	}

	ROS_INFO("check_object_gripped_cb() finished");
}

int Statemachine::check_object_gripped()
{
	if(check_object_gripped_state_==OPEN)
	{
		ROS_INFO("check_object_gripped() called: OPEN");

		obj_picked_up_srv_.request.ObjectMass=cur_obj_mass_;
		check_object_gripped_state_=RUNNING;

		//Wait to let the robot finish its movement (vibrations)
		ROS_INFO("waiting for 1 second to ensure a static robot...");
		ros::Duration waittime = ros::Duration(1, 0);
		waittime.sleep();
		lsc_ = boost::thread(&Statemachine::check_object_gripped_cb,this);
		//check_object_gripped_cb();
	}
	else if(check_object_gripped_state_==FINISHED)
	{
		ROS_INFO("check_object_gripped() called: FINISHED");

		//destroy thread
		lsc_.detach();

		// The error_message field of each service response indicates whether an error occurred. An empty string indicates success
		std::string &op_error_message = obj_picked_up_srv_.response.error_message;
		if(!op_error_message.empty())
		{
			msg_error("Error. obj_picked_up_srv_ failed: %s", op_error_message.c_str());
			check_object_gripped_state_=FINISHEDWITHERROR;

			state_.sub.event_two = fsm::RETRY;
			return 0;
		}
		else
		{
			if (obj_picked_up_srv_.response.GotObject==true)
			{
				ROS_INFO("state-observer says, that object is gripped: OK");
				cur_obj_gripped_=true;
			}
			else
			{
				ROS_INFO("state-observer says, that object is gripped: NOT OK");
				check_object_gripped_state_=FINISHEDWITHERROR;

				state_.sub.event_two = fsm::OBJECT_LOST;


				//publish object state for motion planning
				publish_obj_state(OBJ_LOCATED);
				cur_obj_gripped_=false;

				return 0;
			}
		}

		//==============================================
		scheduler_next();
		//==============================================
		//reset state and counter
		check_object_gripped_state_=OPEN;
		check_object_gripped_counter_=0;

	}
	else if(check_object_gripped_state_==FINISHEDWITHERROR)
	{
		//destroy thread
		lsc_.detach();

		ROS_INFO("check_object_gripped() called: FINISHEDWITHERROR");
		scheduler_schedule(); //Call for Error-Handling
	}

	return 0;
}


int Statemachine::new_object_t6()
{
	if(new_object_t6_state_==OPEN)
	{
		ROS_INFO("new_object_t6() called: OPEN");

		new_object_t6_state_=RUNNING;
		//lsc_ = boost::thread(&Statemachine::new_object_cb,this);
		if(!next_object_client_.call(next_object_srv_))
		{
			std::string &ls_error_message = next_object_srv_.response.error_message;
			msg_error("failed to call next object client: %s ",ls_error_message.c_str());

			new_object_t6_state_=FINISHEDWITHERROR;
		}
		else
		{
			new_object_t6_state_=FINISHED;
			//increase global object counter
			obj_counter_t6_++;
			//and publish it to the parameter server
			node_.setParam("object_counter_",obj_counter_t6_);
		}
	}
	else if(new_object_t6_state_==FINISHED)
	{
		ROS_INFO("new_object_t6() called: FINISHED");

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		new_object_t6_state_=OPEN;
	}
	else if(new_object_t6_state_==FINISHEDWITHERROR)
	{
		//destroy thread
		//lsc_.detach();

		ROS_INFO("new_object_t6() called: FINISHEDWITHERROR");
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::stop_sim_cb()
{
	ROS_INFO("stop_sim_cb() running");

	//save the log
	ROS_INFO("Saving log");
	if(save_log_client_.exists())
	{
		if(!save_log_client_.call(save_log_srv_))
		{
			msg_error("Error. call of save_log_client_ failed");
			stop_sim_state_=FINISHEDWITHERROR;
		}
	}
	else
	{
		msg_error("Error. save_log_client is not available");
		stop_sim_state_=FINISHEDWITHERROR;
	}

	// The stop simulator callback ends the current simulation
	ROS_INFO("stopping simulator");
	if(stop_simulator_client_.exists())
	{
		if(!stop_simulator_client_.call(stop_simulator_srv_))
		{
			msg_error("Error. call of stop_simulator_client_ failed");
			stop_sim_state_=FINISHEDWITHERROR;
		}
	}
	else
	{
		msg_error("Error. stop_simulator_client is not available");
		stop_sim_state_=FINISHEDWITHERROR;
	}

	if(stop_sim_state_==RUNNING)
	{
		stop_sim_state_=FINISHED; //otherwise there was an error
	}
	ROS_INFO("stop_sim_cb() finished");
}

int Statemachine::stop_sim()
{
	if(stop_sim_state_==OPEN)
	{
		ROS_INFO("stop_sim() called: OPEN");

		stop_sim_state_=RUNNING;
		//lsc_ = boost::thread(&Statemachine::stop_sim_cb,this);
		stop_sim_cb();
	}
	else if(stop_sim_state_==FINISHED)
	{
		ROS_INFO("stop_sim() called: FINISHED");

		ROS_INFO("simulator stopped");
		task_active_=false;
		sim_running_=false;

		//==============================================
		//Don't call scheduler_next()! The state has to be set manually to be able to quit in any situation.
#ifndef ONE_TASK
		state_.sub.one = fsm::FINISHED;
#else
		scheduler_next();
#endif
		//==============================================
		//reset state
		stop_sim_state_=OPEN;
	}
	else if(stop_sim_state_==FINISHEDWITHERROR)
	{
		//destroy thread
		//lsc_.detach();

		ROS_INFO("stop_sim() called: FINISHEDWITHERROR");
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::watch_scene_cb()
{
	ROS_INFO("watch_scene_cb() running");

	if(take_image_client_.exists())
	{
		if(take_image_client_.call(take_image_srv_))
		{
			watch_scene_state_=FINISHED;
		}
		else
		{
			msg_error("Error. call of take_image_client_ failed");
			watch_scene_state_=FINISHEDWITHERROR;
			state_.sub.event_one=take_image_srv_.response.error_reason;
		}
	}
	else
	{
		msg_error("Error. take_image_client_ is not available");
		watch_scene_state_=FINISHEDWITHERROR;
		state_.sub.event_one=fsm::SIM_SRV_NA;
	}

	ROS_INFO("watch_scene_cb() finished");
}

int Statemachine::watch_scene()
{
	if(watch_scene_state_==OPEN)
	{
		ROS_INFO("watch_scene() called: OPEN");

		take_image_srv_.request.camera = SCENE_CAM;
		watch_scene_state_=RUNNING;
		lsc_ = boost::thread(&Statemachine::watch_scene_cb,this);
		//watch_scene_cb();
	}
	else if(watch_scene_state_==FINISHED)
	{
		ROS_INFO("watch_scene() called: FINISHED");

		//destroy thread
		lsc_.detach();

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		watch_scene_state_=OPEN;
		watch_scene_counter_=0;
	}
	else if(watch_scene_state_==FINISHEDWITHERROR)
	{
		//destroy thread
		lsc_.detach();

		ROS_INFO("watch_scene() called: FINISHEDWITHERROR");
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

int Statemachine::explore_environment_init()
{
	if(explore_environment_init_state_==OPEN)
	{
		ROS_INFO("explore_environment_init() called: OPEN");

		explore_environment_init_state_=RUNNING;

//		if (active_task_number_ == 4){
//			explore_pose_type_ = EXPLORE_SNAKE;
//		}
//		else{
//			explore_pose_type_ = EXPLORE_STD_1;
//		}

		explore_environment_init_state_=FINISHED;
	}
	else if(explore_environment_init_state_==FINISHED)
	{
		ROS_INFO("explore_environment_init() called: FINISHED");

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		explore_environment_init_state_=OPEN;
	}
	else if(explore_environment_init_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("explore_environment_init() called: FINISHEDWITHERROR");
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::explore_environment_image_cb()
{
	ROS_INFO("explore_environment_image_cb() running");

	if(take_image_client_.exists())
	{
		if(take_image_client_.call(take_image_srv_))
		{
			explore_environment_image_state_=FINISHED;
		}
		else
		{
			msg_error("Error. call of take_image_client_ failed");
			explore_environment_image_state_=FINISHEDWITHERROR;
			state_.sub.event_one=take_image_srv_.response.error_reason;
		}
	}
	else
	{
		msg_error("Error. take_image_client_ is not available");
		explore_environment_image_state_=FINISHEDWITHERROR;
		state_.sub.event_one=fsm::SIM_SRV_NA;
	}

	explore_success_count_++;
	ROS_INFO("explore_environment_image_cb() finished");
}

int Statemachine::explore_environment_image()
{
	if(explore_environment_image_state_==OPEN)
	{
		ROS_INFO("explore_environment_image() called: OPEN");

		take_image_srv_.request.camera = TCP_CAM;
		take_image_srv_.request.sensors.resize(ein_->get_nr_sensors());
		for(uint16_t ii=0;ii<ein_->get_nr_sensors();ii++)
		{
			take_image_srv_.request.sensors[ii]=ein_->get_sensors(ii);
		}
		explore_environment_image_state_=RUNNING;
		lsc_ = boost::thread(&Statemachine::explore_environment_image_cb,this);
		//watch_scene_cb();
	}
	else if(explore_environment_image_state_==FINISHED)
	{
		ROS_INFO("explore_environment_image() called: FINISHED");

		//destroy thread
		lsc_.detach();

		if (explore_success_count_<max_explore_poses_)
		{
			//==============================================
			scheduler_next();
			//==============================================
			//reset state
			explore_environment_image_state_=OPEN;
			explore_environment_image_counter_=0;
		}
		else
		{
			// Original one... call at end
			msg_info("Maximum number of explore poses reached.");

			//==============================================
			scheduler_skip_explore();
			scheduler_next();
			//==============================================
			//reset state
			explore_environment_image_state_=OPEN;
			explore_environment_image_counter_=0;
		}
	}
	else if(explore_environment_image_state_==FINISHEDWITHERROR)
	{
		//destroy thread
		lsc_.detach();

		ROS_INFO("explore_environment_image() called: FINISHEDWITHERROR");
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

int Statemachine::explore_environment_motion()
{
	if(explore_environment_motion_state_==OPEN)
	{
		ROS_INFO("explore_environment_motion() called: OPEN");

		//send actual goal
		explore_environment_motion_state_=RUNNING;

		ROS_INFO("Active Goal: %i Successful poses: %i",active_goal_,explore_success_count_);
		motion_planning_action_client_->sendGoal(explore_poses_->getExploreGoalPose(active_goal_,explore_pose_type_, explore_success_count_),
				boost::bind(&Statemachine::explore_environment_motion_done,this,_1,_2),
				motionClient::SimpleActiveCallback(), //Statemachine::explore_environment_motion_active(),
				motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::explore_environment_motion_feedback,this,_1));
		);
	}
	else if(explore_environment_motion_state_==FINISHED)
	{
		ROS_INFO("explore_environment_motion() called: FINISHED");

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		explore_environment_motion_state_=OPEN;
	}
	else if(explore_environment_motion_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("explore_environment_motion() called: FINISHEDWITHERROR");
		state_.sub.event_two = motion_planning_result_.error_reason;
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::explore_environment_motion_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback)
{
	ROS_INFO("explore_environment_motion_feedback() called");
	ROS_WARN("Estimated Motion Time %f execution time %f",feedback->estimated_motion_time, feedback->execution_time);
}

void Statemachine::explore_environment_motion_done(const actionlib::SimpleClientGoalState& state,
		const am_msgs::goalPoseResultConstPtr& result)
{
	ROS_INFO("explore_environment_motion_done() called, state: %s",state.toString().c_str());

	switch(state.state_)
	{
	case actionlib::SimpleClientGoalState::SUCCEEDED:
		//increase active_goal counter
		active_goal_++;
		explore_environment_motion_state_=FINISHED;
		break;
	case actionlib::SimpleClientGoalState::ACTIVE:
	case actionlib::SimpleClientGoalState::PENDING:
	case actionlib::SimpleClientGoalState::RECALLED:
		break;
	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::PREEMPTED:
	case actionlib::SimpleClientGoalState::ABORTED:
		explore_environment_motion_state_=FINISHEDWITHERROR;
		motion_planning_result_ = *result;
		break;
	default:
		break;
	}
}

void Statemachine::explore_environment_check_cb()
{
	ROS_INFO("explore_environment_check_cb() running");

	ein_->get_all_zones(&check_zones_srv_.request.target_zones);

	if(check_zones_client_.call(check_zones_srv_))
	{
		explore_environment_check_state_=FINISHED;
	}
	else
	{
		msg_error("Error. call of check_zones_client_ failed");
		explore_environment_check_state_=FINISHEDWITHERROR;
	}

	ROS_INFO("explore_environment_check_cb() finished");
}

int Statemachine::explore_environment_check()
{
	if(explore_environment_check_state_==OPEN)
	{
		ROS_INFO("explore_environment_check() called: OPEN");

		explore_environment_check_state_=RUNNING;

		lsc_ = boost::thread(&Statemachine::explore_environment_check_cb,this);
	}
	else if((explore_environment_check_state_==FINISHED)||
			(explore_environment_check_state_==FINISHEDWITHERROR))
	{
		lsc_.detach();

		std::vector<uint16_t> test;
		if(explore_environment_check_state_==FINISHEDWITHERROR)
		{
			test.resize(ein_->get_nr_objects(),0);
			//test[1]=1;
		}
		else
			test=check_zones_srv_.response.zones_occupied;

		int16_t ret=ein_->sort_objects(test);
		if(ret==-1)
		{
			msg_error("sort_objects failed!");
		}
		else if(ret==1)
		{
			msg_warn("insert locate all objects global in state-queue");
			fsm::fsm_state_t temp_state;
			temp_state.sub.one=fsm::SOLVE_TASK;
			temp_state.sub.two=fsm::LOCATE_ALL_OBJECTS_GLOBAL;
			state_queue.insert(state_queue.begin(),temp_state);

			temp_state.sub.one=fsm::EXPLORE_ENVIRONMENT;
			temp_state.sub.two=fsm::EXPLORE_ENVIRONMENT_CHECK;
			state_queue.insert(state_queue.begin()+1,temp_state);
		}

		explore_environment_check_state_=OPEN;
		scheduler_next();
		scheduler_printqueue();

		ROS_INFO("explore_environment_check() called: FINISHED");
	}

	return 0;
}

int Statemachine::locate_object_global()
{
	if(locate_object_global_state_==OPEN)
	{
		ROS_INFO("locate_object_global() called: OPEN");

		//ROS_INFO("current object:");
		//ein_->print_object(&cur_obj_);
		if(ein_->get_active_object_state()==EurocInput::EIN_OBJ_LOCATED)
		{
			ROS_INFO("object %s already located, skip locate_object_global()",cur_obj_.name.c_str());
			locate_object_global_state_=FINISHED;
			return 0;
		}
		else
		{
			ROS_INFO("searching for: %s",cur_obj_.name.c_str());
		}

		if(vision_action_client_->isServerConnected()==0)
		{
			msg_warn("vision_action_client_->isServerConnected=0!");
			//vor send goal -> isconnected abfragen, wenn nicht zerstören und neustarten!
			delete vision_action_client_;
			vision_action_client_ = new actionlib::SimpleActionClient<am_msgs::VisionAction>("VisionAction", true);
			msg_warn("action client recreated, waiting for server");
			vision_action_client_->waitForServer();
		}

		am_msgs::VisionGoal goal;
		goal.mode = GLOBAL_POSE_ESTIMATION;
		goal.precision = locate_object_global_counter_;
		goal.object = cur_obj_;
		goal.sensors.resize(ein_->get_nr_sensors());
		for(uint16_t ii=0;ii<ein_->get_nr_sensors();ii++)
		{
			goal.sensors[ii]=ein_->get_sensors(ii);
		}
		goal.target_zone=ein_->get_active_target_zone();

		//send goal to vision-node.
		locate_object_global_state_=RUNNING;
		vision_action_client_->sendGoal(goal,
				boost::bind(&Statemachine::locate_object_global_done,this,_1,_2),
				visionClient::SimpleActiveCallback(),
				visionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::locate_object_global_feedback,this,_1));
		);
	}
	else if(locate_object_global_state_==FINISHED)
	{
		ROS_INFO("locate_object_global() called: FINISHED");

		//publish object state for motion planning
		publish_obj_state(OBJ_LOCATED);

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		locate_object_global_state_=OPEN;
		locate_object_global_counter_=0;
	}
	else if(locate_object_global_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("locate_object_global() called: FINISHEDWITHERROR");
		state_.sub.event_two = vision_result_.error_reason;
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::locate_object_global_feedback(const am_msgs::VisionFeedbackConstPtr feedback)
{
	ROS_INFO("locate_object_global_feedback() called");
}

void Statemachine::locate_object_global_done(const actionlib::SimpleClientGoalState& state,
		const am_msgs::VisionResultConstPtr& result)
{
	ROS_INFO("locate_object_global_done() called, state: %s",state.toString().c_str());

	switch(state.state_)
	{
	case actionlib::SimpleClientGoalState::SUCCEEDED:
		if(result->object_detected==true)
		{
			cur_obj_.abs_pose=result->abs_object_pose;
			ein_->set_object_pose(result->abs_object_pose, result->stamp);

			locate_object_global_state_=FINISHED;
		}
		else
		{
			msg_error("Error. vision node could not locate object");
			locate_object_global_state_=FINISHEDWITHERROR;
			vision_result_=*result;
			//todo: set event here to skip object
		}
		break;
	case actionlib::SimpleClientGoalState::ACTIVE:
	case actionlib::SimpleClientGoalState::PENDING:
	case actionlib::SimpleClientGoalState::RECALLED:
		break;
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::PREEMPTED:
	case actionlib::SimpleClientGoalState::ABORTED:
		locate_object_global_state_=FINISHEDWITHERROR;
		vision_result_ = *result;
		break;
	default:
		break;
	}
}

int Statemachine::locate_all_objects_global()
{
	if(locate_all_objects_global_state_==OPEN)
	{
		ROS_INFO("locate_all_objects_global() called: OPEN");

		if(vision_action_client_->isServerConnected()==0)
		{
			msg_warn("vision_action_client_->isServerConnected=0!");
			//vor send goal -> isconnected abfragen, wenn nicht zerstören und neustarten!
			delete vision_action_client_;
			vision_action_client_ = new actionlib::SimpleActionClient<am_msgs::VisionAction>("VisionAction", true);
			msg_warn("action client recreated, waiting for server");
			vision_action_client_->waitForServer();
		}

		cur_obj_=ein_->get_object(0);

		//send goals to motion-planning
		active_goal_=0;
		nr_goals_=ein_->get_nr_objects();
		reached_active_goal_=false;
		vision_queue.resize(nr_goals_);

		vision_queue[0].mode = GLOBAL_POSE_ESTIMATION;
		vision_queue[0].precision = locate_all_objects_global_counter_;
		vision_queue[0].object = cur_obj_;
		vision_queue[0].sensors.resize(ein_->get_nr_sensors());
		for(uint16_t ii=0;ii<ein_->get_nr_sensors();ii++)
		{
			vision_queue[0].sensors[ii]=ein_->get_sensors(ii);
		}
		vision_queue[0].target_zone=ein_->get_target_zone(0);

		for(uint16_t ii=1;ii<nr_goals_;ii++)
		{
			vision_queue[ii]=vision_queue[0];

			vision_queue[ii].object=ein_->get_object(ii);
			vision_queue[ii].target_zone=ein_->get_target_zone(ii);
		}

		//send goal to vision-node.
		ROS_INFO("searching for: %s",vision_queue[0].object.name.c_str());
		locate_all_objects_global_state_=RUNNING;
		vision_action_client_->sendGoal(vision_queue[0],
				boost::bind(&Statemachine::locate_all_objects_global_done,this,_1,_2),
				visionClient::SimpleActiveCallback(),
				visionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::locate_object_global_feedback,this,_1));
		);
	}
	else if (locate_all_objects_global_state_==RUNNING && reached_active_goal_==true)
	{
		reached_active_goal_=false;
		active_goal_++;


		if(active_goal_==nr_goals_)
		{
			locate_all_objects_global_state_=FINISHED;
			return 0;
		}
		else
		{
			ROS_INFO("locate_all_objects_global() called: RUNNING (remaining goals)");
		}

		ROS_INFO("searching for: %s",vision_queue[active_goal_].object.name.c_str());
		vision_action_client_->sendGoal(vision_queue[active_goal_],
						boost::bind(&Statemachine::locate_all_objects_global_done,this,_1,_2),
						visionClient::SimpleActiveCallback(),
						visionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::locate_object_global_feedback,this,_1));
		);
	}
	else if(locate_all_objects_global_state_==FINISHED)
	{
		ROS_INFO("locate_all_objects_global() called: FINISHED");

		//publish object state for motion planning
		//publish_obj_state(OBJ_LOCATED);

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		locate_all_objects_global_state_=OPEN;
		locate_all_objects_global_counter_=0;
	}
	else if(locate_all_objects_global_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("locate_all_objects_global() called: FINISHEDWITHERROR");
		state_.sub.event_two = vision_result_.error_reason;
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::locate_all_objects_global_feedback(const am_msgs::VisionFeedbackConstPtr feedback)
{
	ROS_INFO("locate_all_objects_global_feedback() called");
}

void Statemachine::locate_all_objects_global_done(const actionlib::SimpleClientGoalState& state,
		const am_msgs::VisionResultConstPtr& result)
{
	ROS_INFO("locate_all_objects_global_done() called, state: %s",state.toString().c_str());

	switch(state.state_)
	{
	case actionlib::SimpleClientGoalState::SUCCEEDED:
		if(result->object_detected==true)
		{
			reached_active_goal_=true;
			ein_->set_object_pose(result->abs_object_pose, active_goal_, result->stamp);
		}
		else
		{
			msg_error("Error. vision node could not locate object");
			locate_all_objects_global_state_=FINISHEDWITHERROR;
			vision_result_=*result;
			//todo: set event here to skip object
		}
		break;
	case actionlib::SimpleClientGoalState::ACTIVE:
	case actionlib::SimpleClientGoalState::PENDING:
	case actionlib::SimpleClientGoalState::RECALLED:
		break;
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::PREEMPTED:
	case actionlib::SimpleClientGoalState::ABORTED:
		locate_object_global_state_=FINISHEDWITHERROR;
		vision_result_ = *result;
		break;
	default:
		break;
	}
}

int Statemachine::locate_object_close_range()
{
	if(locate_object_close_range_state_==OPEN)
	{
		ROS_INFO("locate_object_close_range() called: OPEN");

		if(strcmp(cur_obj_.shape[0].type.c_str(),"box")!=0)
		{
			//skip locate object close range for objects other than boxes
			msg_info("cur_obj_.shape[0].type != box -> skipping locate object close range");
			locate_object_close_range_state_=FINISHED;
			vision_result_.object_detected=false;
		}

		//ROS_INFO("current object:");
		//ein_->print_object(&cur_obj_);
		ROS_INFO("searching for: %s",cur_obj_.name.c_str());

		am_msgs::VisionGoal goal;
		goal.mode = CLOSE_RANGE_POSE_ESTIMATION;
		goal.precision = 0;
		goal.object = cur_obj_;
		goal.sensors.resize(ein_->get_nr_sensors());
		for(uint16_t ii=0;ii<ein_->get_nr_sensors();ii++)
		{
			goal.sensors[ii]=ein_->get_sensors(ii);
		}
		goal.target_zone=ein_->get_active_target_zone();

		//send goal to vision-node.
		locate_object_close_range_state_=RUNNING;
		vision_action_client_->sendGoal(goal,
				boost::bind(&Statemachine::locate_object_close_range_done,this,_1,_2),
				visionClient::SimpleActiveCallback(),
				visionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::locate_object_close_range_feedback,this,_1));
		);
	}
	else if(locate_object_close_range_state_==FINISHED)
	{
		ROS_INFO("locate_object_close_range() called: FINISHED");

		if(vision_result_.object_detected==true)
		{
			cur_obj_.abs_pose=vision_result_.abs_object_pose;

			//calc grasping pose again, if there's a new object pose:
			fsm::fsm_state_t temp_state;
			temp_state.sub.one=fsm::SOLVE_TASK;
			if(active_task_number_<=4)
            {
              temp_state.sub.two=fsm::GET_GRASPING_POSE;               state_queue.insert(state_queue.begin(),temp_state);
            }
            else if(active_task_number_==5)
            {
              temp_state.sub.two=fsm::GET_GRASPING_POSE_T5;            state_queue.insert(state_queue.begin(),temp_state);
            }
            else
            {
              temp_state.sub.two=fsm::GET_GRASPING_POSE_T6;            state_queue.insert(state_queue.begin(),temp_state);
            }

		}
		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		locate_object_close_range_state_=OPEN;
	}
	else if(locate_object_close_range_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("locate_object_close_range() called: FINISHEDWITHERROR");
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::locate_object_close_range_feedback(const am_msgs::VisionFeedbackConstPtr feedback)
{
	ROS_INFO("locate_object_close_range_feedback() called");
}

void Statemachine::locate_object_close_range_done(const actionlib::SimpleClientGoalState& state,
		const am_msgs::VisionResultConstPtr& result)
{
	ROS_INFO("locate_object_close_range_done() called, state: %s",state.toString().c_str());

	switch(state.state_)
	{
	case actionlib::SimpleClientGoalState::SUCCEEDED:
		locate_object_close_range_state_=FINISHED;
		vision_result_=*result;
		break;
	case actionlib::SimpleClientGoalState::ACTIVE:
	case actionlib::SimpleClientGoalState::PENDING:
	case actionlib::SimpleClientGoalState::RECALLED:
		break;
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::PREEMPTED:
	case actionlib::SimpleClientGoalState::ABORTED:
		locate_object_close_range_state_=FINISHEDWITHERROR;
		break;
	default:
		break;
	}
}

void Statemachine::get_grasping_pose_cb()
{
	ROS_INFO("get_grasping_pose_cb() running");

	if(get_grasp_pose_client_.exists())
	{
		if(get_grasp_pose_client_.call(get_grasp_pose_srv_))
		{
			get_grasping_pose_state_=FINISHED;
		}
		else
		{
			msg_error("Error. call of get_grasp_pose_client_ failed");
			get_grasping_pose_state_=FINISHEDWITHERROR;
		}
	}
	else
	{
		msg_error("Error. get_grasp_pose_client_ is not available");
		get_grasping_pose_state_=FINISHEDWITHERROR;
	}



	ROS_INFO("get_grasping_pose_cb() finished");
}

int Statemachine::get_grasping_pose()
{
	if(get_grasping_pose_state_==OPEN)
	{
		ROS_INFO("get_grasping_pose() called: OPEN");
		get_grasp_pose_srv_.request.object=cur_obj_;
		get_grasp_pose_srv_.request.target_zone=cur_zone_;
		get_grasping_pose_state_=RUNNING;
		lsc_ = boost::thread(&Statemachine::get_grasping_pose_cb,this);
		//get_grasping_pose_cb();
	}
	else if(get_grasping_pose_state_==FINISHED)
	{
		ROS_INFO("get_grasping_pose() called: FINISHED");

		//destroy thread
		lsc_.detach();

		cur_obj_mass_=get_grasp_pose_srv_.response.object_mass;
		if(get_grasp_pose_get_vectors()==false)
		{
                  get_grasping_pose_state_=FINISHEDWITHERROR;
                  return 0;
		}

		if(-1==find_pose_set())
		{
			msg_error("no feasible pose set found!");
			get_grasping_pose_state_=FINISHEDWITHERROR;
			return 0;
		}
		ROS_INFO("Found pose set: ");
		ROS_INFO("selected grip pose: %d",selected_object_pose_);
		ROS_INFO("selected target pose: %d",selected_target_pose_);

		//
		if(object_skip_vision[selected_object_pose_]==1)
		{
			ROS_INFO("Skip vision=1 for this pose -> skip move to object vision!");

			uint16_t iter=state_queue.size()-2;
			for(uint16_t ii=0;ii<iter;ii++)
			{
				if((state_queue[ii].sub.one == fsm::SOLVE_TASK) &&
						(state_queue[ii].sub.two == fsm::MOVE_TO_OBJECT_VISION))
				{
					state_queue.erase(state_queue.begin()+ii);
					state_queue.erase(state_queue.begin()+ii);
					//state_queue.erase(state_queue.begin()+ii);
					break;
				}
			}
		}
		if(target_skip_vision[selected_target_pose_]==1)
		{
			ROS_INFO("Skip vision=1 for this pose -> skip move to target zone vision!");

			uint16_t iter=state_queue.size()-2;
			for(uint16_t ii=0;ii<iter;ii++)
			{
				if((state_queue[ii].sub.one == fsm::SOLVE_TASK) &&
						(state_queue[ii].sub.two == fsm::MOVE_TO_TARGET_ZONE_VISION))
				{
					state_queue.erase(state_queue.begin()+ii);
					//state_queue.erase(state_queue.begin()+ii);
					break;
				}
			}
		}


		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		get_grasping_pose_state_=OPEN;
	}
	else if(get_grasping_pose_state_==FINISHEDWITHERROR)
	{
		//destroy thread
		lsc_.detach();

		ROS_INFO("get_grasping_pose() called: FINISHEDWITHERROR");
		scheduler_schedule(); //Call for Error-Handling
	}

	return 0;
}

bool Statemachine::get_grasp_pose_get_vectors()
{
  //first clean up all vectors
  object_grip_pose.clear();
  object_safe_pose.clear();
  object_vision_pose.clear();
  grip_pose_type.clear();
  object_skip_vision.clear();
  object_grasp_width.clear();
  target_place_pose.clear();
  target_safe_pose.clear();
  target_vision_pose.clear();
  place_pose_type.clear();
  target_skip_vision.clear();
  push_safe_pose.clear();
  push_target_pose.clear();
  object_grip_r_tcp_com.clear();

  //then assign new vectors
  object_grip_pose=get_grasp_pose_srv_.response.object_grip_pose;
  object_safe_pose=get_grasp_pose_srv_.response.object_safe_pose;
  object_vision_pose=get_grasp_pose_srv_.response.object_vision_pose;
  grip_pose_type=get_grasp_pose_srv_.response.grip_pose_type;
  object_skip_vision=get_grasp_pose_srv_.response.object_skip_vision;
  object_grasp_width=get_grasp_pose_srv_.response.object_grasp_width;
  target_place_pose=get_grasp_pose_srv_.response.target_place_pose;
  target_safe_pose=get_grasp_pose_srv_.response.target_safe_pose;
  target_vision_pose=get_grasp_pose_srv_.response.target_vision_pose;
  place_pose_type=get_grasp_pose_srv_.response.place_pose_type;
  target_skip_vision=get_grasp_pose_srv_.response.target_skip_vision;
  push_safe_pose=get_grasp_pose_srv_.response.push_safe_pose;
  push_target_pose=get_grasp_pose_srv_.response.push_target_pose;
  object_grip_r_tcp_com=get_grasp_pose_srv_.response.object_grip_r_tcp_com;

  //print vector sizes
  ROS_INFO("Received pose vectors:");
  ROS_INFO("object_grip_pose.size()=%d",object_grip_pose.size());
  ROS_INFO("object_safe_pose.size()=%d",object_safe_pose.size());
  ROS_INFO("object_vision_pose.size()=%d",object_vision_pose.size());
  ROS_INFO("grip_pose_type.size()=%d",grip_pose_type.size());
  ROS_INFO("object_skip_vision.size()=%d",object_skip_vision.size());
  ROS_INFO("object_grasp_width.size()=%d",object_grasp_width.size());
  ROS_INFO("target_place_pose.size()=%d",target_place_pose.size());
  ROS_INFO("target_safe_pose.size()=%d",target_safe_pose.size());
  ROS_INFO("target_vision_pose.size()=%d",target_vision_pose.size());
  ROS_INFO("target_skip_vision.size()=%d",target_skip_vision.size());
  ROS_INFO("place_pose_type.size()=%d",place_pose_type.size());
  ROS_INFO("push_safe_pose.size()=%d",push_safe_pose.size());
  ROS_INFO("push_target_pose.size()=%d",push_target_pose.size());
  ROS_INFO("object_grip_r_tcp_com.size()=%d",object_grip_r_tcp_com.size());

  //check the vector sizes
  bool checksizes;
  checksizes=true;

  if (object_grip_pose.size()<1)
  {
          msg_error("Error. no object_grip_poses received");
          checksizes=false;
  }
  if (target_place_pose.size()<1)
  {
          msg_error("Error. no target_place_poses received");
          checksizes=false;
  }
//  if (push_target_pose.size()<1 && active_task_number_==5)
//  {
//          msg_error("Error. no push_target_poses received (but necessary for task 5)");
//          checksizes=false;
//  }

  if (object_safe_pose.size()!=object_grip_pose.size())
  {
          msg_error("Error. object_safe_pose has wrong size");
          checksizes=false;
  }
  if (object_vision_pose.size()!=object_grip_pose.size())
  {
          msg_error("Error. object_vision_pose has wrong size");
          checksizes=false;
  }
  if (object_skip_vision.size()!=object_grip_pose.size())
  {
          msg_error("Error. object_skip_vision has wrong size");
          checksizes=false;
  }
  if (grip_pose_type.size()!=object_grip_pose.size())
  {
          msg_error("Error. grip_pose_type has wrong size");
          checksizes=false;
  }
  if (object_grasp_width.size()!=object_grip_pose.size())
  {
          msg_error("Error. object_grasp_width has wrong size");
          checksizes=false;
  }
  if (target_safe_pose.size()!=target_place_pose.size())
  {
          msg_error("Error. target_safe_pose has wrong size");
          checksizes=false;
  }
  if (target_vision_pose.size()!=target_place_pose.size())
  {
          msg_error("Error. target_vision_pose has wrong size");
          checksizes=false;
  }
  if (place_pose_type.size()!=target_place_pose.size())
  {
          msg_error("Error. place_pose_type has wrong size");
          checksizes=false;
  }
  if (target_skip_vision.size()!=target_place_pose.size())
  {
          msg_error("Error. target_skip_vision has wrong size");
          checksizes=false;
  }
//  if (push_safe_pose.size()!=push_target_pose.size())
//  {
//          msg_error("Error. push_safe_pose has wrong size");
//          checksizes=false;
//  }
  if (object_grip_r_tcp_com.size()!=object_grip_pose.size())
  {
          msg_error("Error. object_grip_r_tcp_com has wrong size");
          checksizes=false;
  }

  return checksizes;
}

void Statemachine::get_grasping_poseT5_cb()
{
        ROS_INFO("get_grasping_poseT5_cb() running");

        if(get_grasp_pose_client_.exists())
        {
                if(get_grasp_pose_client_.call(get_grasp_pose_srv_))
                {
                        get_grasping_poseT5_state_=FINISHED;
                }
                else
                {
                        msg_error("Error. call of get_grasp_pose_client_ failed");
                        get_grasping_poseT5_state_=FINISHEDWITHERROR;
                }
        }
        else
        {
                msg_error("Error. get_grasp_pose_client_ is not available");
                get_grasping_poseT5_state_=FINISHEDWITHERROR;
        }
        ROS_INFO("get_grasping_poseT5_cb() finished");
}

int Statemachine::get_grasping_poseT5()
{
        if(get_grasping_poseT5_state_==OPEN)
        {
                ROS_INFO("get_grasping_poseT5() called: OPEN");
                get_grasp_pose_srv_.request.object=cur_obj_;
                get_grasp_pose_srv_.request.target_pose=cur_target_pose_;

                get_grasping_poseT5_state_=RUNNING;
                lsc_ = boost::thread(&Statemachine::get_grasping_poseT5_cb,this);
                //get_grasping_poseT5_cb();
        }
        else if(get_grasping_poseT5_state_==FINISHED)
        {
                ROS_INFO("get_grasping_poseT5() called: FINISHED");

                //destroy thread
                lsc_.detach();

                cur_obj_mass_=get_grasp_pose_srv_.response.object_mass;
                if(get_grasp_pose_get_vectors()==false)
                {
                  get_grasping_poseT5_state_=FINISHEDWITHERROR;
                  return 0;
                }

                selected_object_pose_=0;
                selected_target_pose_=0;

				if(object_skip_vision[selected_object_pose_]==1)
				{
					ROS_INFO("Skip vision=1 for this pose -> skip move to object vision!");

					uint16_t iter=state_queue.size()-2;
					for(uint16_t ii=0;ii<iter;ii++)
					{
						if((state_queue[ii].sub.one == fsm::SOLVE_TASK) &&
								(state_queue[ii].sub.two == fsm::MOVE_TO_OBJECT_VISION))
						{
							state_queue.erase(state_queue.begin()+ii);
							state_queue.erase(state_queue.begin()+ii);
							//state_queue.erase(state_queue.begin()+ii);
							break;
						}
					}
				}
				if(target_skip_vision[selected_target_pose_]==1)
				{
					ROS_INFO("Skip vision=1 for this pose -> skip move to target zone vision!");

					uint16_t iter=state_queue.size()-2;
					for(uint16_t ii=0;ii<iter;ii++)
					{
						if((state_queue[ii].sub.one == fsm::SOLVE_TASK) &&
								(state_queue[ii].sub.two == fsm::MOVE_TO_TARGET_ZONE_VISION))
						{
							state_queue.erase(state_queue.begin()+ii);
							//state_queue.erase(state_queue.begin()+ii);
							break;
						}
					}
				}

                //==============================================
                scheduler_next();
                //==============================================
                //reset state
                get_grasping_poseT5_state_=OPEN;
        }
        else if(get_grasping_poseT5_state_==FINISHEDWITHERROR)
        {
                //destroy thread
                lsc_.detach();

                ROS_INFO("get_grasping_poseT5() called: FINISHEDWITHERROR");
                scheduler_schedule(); //Call for Error-Handling
        }

        return 0;
}

void Statemachine::get_grasping_poseT6_cb()
{
        ROS_INFO("get_grasping_poseT6_cb() running");

        if(get_grasp_pose_client_.exists())
        {
                if(get_grasp_pose_client_.call(get_grasp_pose_srv_))
                {
                        get_grasping_poseT6_state_=FINISHED;
                }
                else
                {
                        msg_error("Error. call of get_grasp_pose_client_ failed");
                        get_grasping_poseT6_state_=FINISHEDWITHERROR;
                }
        }
        else
        {
                msg_error("Error. get_grasp_pose_client_ is not available");
                get_grasping_poseT6_state_=FINISHEDWITHERROR;
        }
        ROS_INFO("get_grasping_poseT6_cb() finished");
}

int Statemachine::get_grasping_poseT6()
{
        if(get_grasping_poseT6_state_==OPEN)
        {
                ROS_INFO("get_grasping_poseT6() called: OPEN");
                get_grasp_pose_srv_.request.object=cur_obj_;
                get_grasp_pose_srv_.request.target_zone=cur_zone_;
                get_grasping_poseT6_state_=RUNNING;
                lsc_ = boost::thread(&Statemachine::get_grasping_poseT6_cb,this);
                //get_grasping_poseT6_cb();
        }
        else if(get_grasping_poseT6_state_==FINISHED)
        {
                ROS_INFO("get_grasping_poseT6() called: FINISHED");

                //destroy thread
                lsc_.detach();

                cur_obj_mass_=get_grasp_pose_srv_.response.object_mass;
                if(get_grasp_pose_get_vectors()==false)
                {
                  get_grasping_poseT6_state_=FINISHEDWITHERROR;
                  return 0;
                }

                selected_object_pose_=0;
                selected_target_pose_=0;

                //==============================================
                scheduler_next();
                //==============================================
                //reset state
                get_grasping_poseT6_state_=OPEN;
        }
        else if(get_grasping_poseT6_state_==FINISHEDWITHERROR)
        {
                //destroy thread
                lsc_.detach();

                ROS_INFO("get_grasping_poseT6() called: FINISHEDWITHERROR");
                scheduler_schedule(); //Call for Error-Handling
        }

        return 0;
}

int Statemachine::find_pose_set()
{
	uint16_t obj_sz=object_grip_pose.size();
	uint16_t tag_sz=target_place_pose.size();

	switch (cur_object_type_)
	{
	case OBJECT_HANDLE:
		for(uint16_t oo=0;oo<obj_sz;oo++)
		{
			for(uint16_t tt=0;tt<tag_sz;tt++)
			{
				switch(grip_pose_type[oo])
				{
				case GRIP_POSE_HANDLE_CYLINDER_ZEQX_YPOSZ:
				case GRIP_POSE_HANDLE_CYLINDER_ZEQY_YPOSZ:
					if(place_pose_type[tt]==PLACE_POSE_HANDLE_CYLINDER_YPOSZ_VERTICAL)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				case GRIP_POSE_HANDLE_CYLINDER_ZEQX_YNEGZ:
				case GRIP_POSE_HANDLE_CYLINDER_ZEQY_YNEGZ:
					if(place_pose_type[tt]==PLACE_POSE_HANDLE_CYLINDER_YNEGZ_VERTICAL)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				case GRIP_POSE_HANDLE_BOX1_ZEQX_YPOSZ:
				case GRIP_POSE_HANDLE_BOX1_ZEQY_YPOSZ:
					if(place_pose_type[tt]==PLACE_POSE_HANDLE_BOX1_YPOSZ_VERTICAL)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				case GRIP_POSE_HANDLE_BOX1_ZEQX_YNEGZ:
				case GRIP_POSE_HANDLE_BOX1_ZEQY_YNEGZ:
					if(place_pose_type[tt]==PLACE_POSE_HANDLE_BOX1_YNEGZ_VERTICAL)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				case GRIP_POSE_HANDLE_BOX2_ZEQX_YPOSZ:
				case GRIP_POSE_HANDLE_BOX2_ZEQY_YPOSZ:
					if(place_pose_type[tt]==PLACE_POSE_HANDLE_BOX2_YPOSZ_VERTICAL)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				case GRIP_POSE_HANDLE_BOX2_ZEQX_YNEGZ:
				case GRIP_POSE_HANDLE_BOX2_ZEQY_YNEGZ:
					if(place_pose_type[tt]==PLACE_POSE_HANDLE_BOX2_YNEGZ_VERTICAL)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				default:
					break;
				}
			}
		}

		break;
	case OBJECT_CYLINDER:
		for(uint16_t oo=0;oo<obj_sz;oo++)
		{
			for(uint16_t tt=0;tt<tag_sz;tt++)
			{
				switch(grip_pose_type[oo])
				{
				case GRIP_POSE_CYLINDER_VERTICAL:
					if(place_pose_type[tt]==PLACE_POSE_CYLINDER_VERTICAL)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				case GRIP_POSE_CYLINDER_VERTICAL_45:
					if(place_pose_type[tt]==PLACE_POSE_CYLINDER_VERTICAL_45)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				default:
					break;
				}
			}
		}
		break;
	case OBJECT_CUBE:
		for(uint16_t oo=0;oo<obj_sz;oo++)
		{
			for(uint16_t tt=0;tt<tag_sz;tt++)
			{
				switch(grip_pose_type[oo])
				{
				case GRIP_POSE_CUBE_X_UP:
					if(place_pose_type[tt]==PLACE_POSE_CUBE_X_UP)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				case GRIP_POSE_CUBE_Y_UP:
					if(place_pose_type[tt]==PLACE_POSE_CUBE_Y_UP)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				case GRIP_POSE_CUBE_Z_UP:
					if(place_pose_type[tt]==PLACE_POSE_CUBE_Z_UP)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				case GRIP_POSE_CUBE_X_UP_45byY:
					if(place_pose_type[tt]==PLACE_POSE_CUBE_X_UP_45byY)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				case GRIP_POSE_CUBE_X_UP_45byZ:
					if(place_pose_type[tt]==PLACE_POSE_CUBE_X_UP_45byZ)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				case GRIP_POSE_CUBE_Y_UP_45byX:
					if(place_pose_type[tt]==PLACE_POSE_CUBE_Y_UP_45byX)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				case GRIP_POSE_CUBE_Y_UP_45byZ:
					if(place_pose_type[tt]==PLACE_POSE_CUBE_Y_UP_45byZ)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				case GRIP_POSE_CUBE_Z_UP_45byX:
					if(place_pose_type[tt]==PLACE_POSE_CUBE_Z_UP_45byX)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				case GRIP_POSE_CUBE_Z_UP_45byY:
					if(place_pose_type[tt]==PLACE_POSE_CUBE_Z_UP_45byY)
					{
						selected_object_pose_=oo;
						selected_target_pose_=tt;
						return 0;
					}
				default:
					break;
				}
			}
		}
		break;
	}

	return -1;
}

void Statemachine::set_object_load_cb()
{
	ROS_INFO("set_object_load_cb() running");

	if(set_object_load_client_.exists())
	{
		if(set_object_load_client_.call(set_object_load_srv_))
		{
			set_object_load_state_=FINISHED;
		}
		else
		{
			msg_error("Error. call of set_object_load_client_ failed");
			set_object_load_state_=FINISHEDWITHERROR;
		}
	}
	else
	{
		msg_error("Error. set_object_load_client_ is not available");
		set_object_load_state_=FINISHEDWITHERROR;
	}

	ROS_INFO("set_object_load_cb() finished");
}

void Statemachine::gripper_release_cb()
{
	ROS_INFO("gripper_release_cb() running");

	if(gripper_control_client_.exists())
	{
		if(gripper_control_client_.call(gripper_control_srv_))
		{
			gripper_release_state_=FINISHED;
		}
		else
		{
			msg_error("Error. call of gripper_control_client_ failed");
			gripper_release_state_=FINISHEDWITHERROR;
			state_.sub.event_three=gripper_control_srv_.response.error_reason;
		}
	}
	else
	{
		msg_error("Error. gripper_control_client_ is not available");
		gripper_release_state_=FINISHEDWITHERROR;
		state_.sub.event_three=fsm::SIM_SRV_NA;
	}

	ROS_INFO("gripper_release_cb() finished");
}

int Statemachine::gripper_release()
{
	if(gripper_release_state_==OPEN && set_object_load_state_==OPEN)
	{
		ROS_INFO("gripper_release() called: OPEN (release) OPEN (set object load)");

		gripper_control_srv_.request.gripping_mode = RELEASE;
		gripper_release_state_=RUNNING;
		lsc_ = boost::thread(&Statemachine::gripper_release_cb,this);
		//gripper_release_cb();
	}
	else if(gripper_release_state_==FINISHED && set_object_load_state_==OPEN)
	{
		ROS_INFO("gripper_release() called: FINISHED (release) OPEN (set object load)");

		//destroy thread
		lsc_.detach();

		if (object_grip_r_tcp_com.size()>0)
		{
			set_object_load_srv_.request.center_of_gravity = object_grip_r_tcp_com[selected_object_pose_];
		}
		else
		{
			geometry_msgs::Vector3 emptyvector;
			emptyvector.x=0;
			emptyvector.y=0;
			emptyvector.z=0;
			set_object_load_srv_.request.center_of_gravity = emptyvector;
		}
		set_object_load_srv_.request.mass = 0;
		set_object_load_state_=RUNNING;
		lsc_ = boost::thread(&Statemachine::set_object_load_cb,this);
		//set_object_load_cb();
	}
	else if(gripper_release_state_==FINISHED && set_object_load_state_==FINISHED)
	{
		ROS_INFO("gripper_release() called: FINISHED (release) FINISHED (set object load)");

		//destroy thread
		lsc_.detach();

		if(!set_object_load_srv_.response.error_message.empty())
		{
			msg_error("Error. failed to set object load! reason: %s",set_object_load_srv_.response.error_message.c_str());
			set_object_load_state_=FINISHEDWITHERROR;
			return 0;
		}
		else
		{
			ROS_INFO("object mass set to 0kg");
		}

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		gripper_release_state_=OPEN;
		set_object_load_state_=OPEN;
		gripper_release_counter_=0;
	}
	else if(gripper_release_state_==FINISHEDWITHERROR)
	{
		//destroy thread
		lsc_.detach();

		ROS_INFO("gripper_release() called: FINISHEDWITHERROR (release)");
		scheduler_schedule(); //Call for Error-Handling
	}
	else if(set_object_load_state_==FINISHEDWITHERROR)
	{
		//destroy thread
		lsc_.detach();

		ROS_INFO("gripper_release() called: FINISHEDWITHERROR (set object load)");
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::gripper_close_cb()
{
	ROS_INFO("gripper_close_cb() running");

	if(gripper_control_client_.exists())
	{
		if(gripper_control_client_.call(gripper_control_srv_))
		{
			gripper_close_state_=FINISHED;
		}
		else
		{
			msg_error("Error. call of gripper_control_client_ failed");
			gripper_close_state_=FINISHEDWITHERROR;
			state_.sub.event_three=gripper_control_srv_.response.error_reason;
		}
	}
	else
	{
		msg_error("Error. gripper_control_client_ is not available");
		gripper_close_state_=FINISHEDWITHERROR;
		state_.sub.event_three=fsm::SIM_SRV_NA;
	}

	ROS_INFO("gripper_close_cb() finished");
}

int Statemachine::gripper_close()
{
	if(gripper_close_state_==OPEN && set_object_load_state_==OPEN)
	{
		ROS_INFO("gripper_close() called: OPEN (close) OPEN (set object load)");

		//calculate estimated gripper forces over friction (friction-coefficient=0.1, safety-factor=1.5)
		//gripper_control_srv_.request.gripping_force = 1.5*(cur_obj_mass_*9.81)/(2*0.1);
		gripper_control_srv_.request.gripping_force = 1.5*(cur_obj_mass_*9.81)/(2*1);
		if (gripper_control_srv_.request.gripping_force>170)
		{
			gripper_control_srv_.request.gripping_force=170;
		}
		if(gripper_control_srv_.request.gripping_force<10)
		{
			gripper_control_srv_.request.gripping_force=10;
		}

		ROS_INFO("gripper force set to %3.2fN",gripper_control_srv_.request.gripping_force);
		if (object_grasp_width.size()>0)
		{
			gripper_control_srv_.request.object_width = object_grasp_width[selected_object_pose_];
			gripper_control_srv_.request.gripper_position = 0.0;  //TODO Delete?
		}
		else
		{
			gripper_control_srv_.request.object_width = object_grasp_width[selected_object_pose_]; //was 0
			gripper_control_srv_.request.gripper_position = 0.0; //TODO Delete?
		}
		gripper_control_srv_.request.gripping_mode = POSITION; // FF_FORCE;
		gripper_close_state_=RUNNING;
		lsc_ = boost::thread(&Statemachine::gripper_close_cb,this);
		//gripper_close_cb();
	}
	else if(gripper_close_state_==FINISHED && set_object_load_state_==OPEN)
	{
		ROS_INFO("gripper_close() called: FINISHED (close) OPEN (set object load)");

		//destroy thread
		lsc_.detach();

		ROS_INFO("waiting for 0.5 second to ensure a static robot...");
		ros::Duration waittime = ros::Duration(0.5, 0);
		waittime.sleep();

		if (object_grip_r_tcp_com.size()>0)
		{
			set_object_load_srv_.request.center_of_gravity = object_grip_r_tcp_com[selected_object_pose_];
			ROS_INFO("r_tcp_com=[%3.2f %3.2f %3.2f]^T",object_grip_r_tcp_com[selected_object_pose_].x,
					object_grip_r_tcp_com[selected_object_pose_].y,object_grip_r_tcp_com[selected_object_pose_].z);
		}
		else
		{
			msg_warn("object_grip_r_tcp_com.size()=0, setting object load with zero-vec");
			geometry_msgs::Vector3 emptyvector;
			emptyvector.x=0;
			emptyvector.y=0;
			emptyvector.z=0;
			set_object_load_srv_.request.center_of_gravity = emptyvector;
		}
		set_object_load_srv_.request.mass = cur_obj_mass_;
		set_object_load_state_=RUNNING;
		lsc_ = boost::thread(&Statemachine::set_object_load_cb,this);
		//set_object_load_cb();
	}
	else if(gripper_close_state_==FINISHED && set_object_load_state_==FINISHED)
	{
		ROS_INFO("gripper_close() called: FINISHED (close) FINISHED (set object load)");
		cur_obj_gripped_=true;

		//destroy thread
		lsc_.detach();

		if(!set_object_load_srv_.response.error_message.empty())
		{
			msg_error("Error. failed to set object load! reason: %s",set_object_load_srv_.response.error_message.c_str());
			set_object_load_state_=FINISHEDWITHERROR;
			return 0;
		}
		else
		{
			ROS_INFO("object mass set to %3.2fkg",cur_obj_mass_);
		}

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		gripper_close_state_=OPEN;
		set_object_load_state_=OPEN;
		gripper_close_counter_=0;
	}
	else if(gripper_close_state_==FINISHEDWITHERROR)
	{
		//destroy thread
		lsc_.detach();

		ROS_INFO("gripper_close() called: FINISHEDWITHERROR (close)");
		scheduler_schedule(); //Call for Error-Handling
	}
	else if(set_object_load_state_==FINISHEDWITHERROR)
	{
		//destroy thread
		lsc_.detach();

		ROS_INFO("gripper_close() called: FINISHEDWITHERROR (set object load)");
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

int Statemachine::move_to_object_safe()
{
	if(move_to_object_safe_state_==OPEN)
	{
		ROS_INFO("move_to_object_safe() called: OPEN");

		//publish object state for motion planning
		if(cur_obj_gripped_==true)
			publish_obj_state(OBJ_GRABED);

		//send goals to motion-planning
		active_goal_=0;
		nr_goals_=1;
		reached_active_goal_=false;
		goal_queue.resize(nr_goals_);

		goal_queue[0].goal_pose = object_safe_pose[selected_object_pose_];
		goal_queue[0].planning_algorithm = planning_mode_.object;
		goal_queue[0].planning_frame = GP_TCP;
		goal_queue[0].inter_steps = 0;
		goal_queue[0].allowed_time = 60.0;

		if(cur_obj_gripped_==true)
			goal_queue[0].speed_percentage = slow_moving_speed*(1-speed_mod_);
		else
			goal_queue[0].speed_percentage = std_moving_speed*(1-speed_mod_);


		//send first goal
		move_to_object_safe_state_=RUNNING;
		motion_planning_action_client_->sendGoal(goal_queue[0],
				boost::bind(&Statemachine::move_to_object_safe_done,this,_1,_2),
				motionClient::SimpleActiveCallback(), //Statemachine::move_to_object_safe_active(),
				motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::move_to_object_safe_feedback,this,_1));
		);
	}
	else if (move_to_object_safe_state_==RUNNING && reached_active_goal_==true)
	{
		reached_active_goal_=false;
		active_goal_++;


		if(active_goal_==nr_goals_)
		{
			move_to_object_safe_state_=FINISHED;
			return 0;
		}
		else
		{
			ROS_INFO("move_to_object_safe() called: RUNNING (remaining goals)");
		}

		motion_planning_action_client_->sendGoal(goal_queue[active_goal_],
				boost::bind(&Statemachine::move_to_object_safe_done,this,_1,_2),
				motionClient::SimpleActiveCallback(), //Statemachine::move_to_object_safe_active(),
				motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::move_to_object_safe_feedback,this,_1));
		);
	}
	else if(move_to_object_safe_state_==FINISHED)
	{
		ROS_INFO("move_to_object_safe() called: FINISHED");


		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		move_to_object_safe_state_=OPEN;
		//reset retry counter
		move_to_object_safe_counter_=0;
	}
	else if(move_to_object_safe_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("move_to_object_safe() called: FINISHEDWITHERROR");
		state_.sub.event_two = motion_planning_result_.error_reason;
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::move_to_object_safe_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback)
{
	ROS_INFO("move_to_object_safe_feedback() called");
}

void Statemachine::move_to_object_safe_done(const actionlib::SimpleClientGoalState& state,
		const am_msgs::goalPoseResultConstPtr& result)
{
	ROS_INFO("move_to_object_safe_done() called, state: %s",state.toString().c_str());

	switch(state.state_)
	{
	case actionlib::SimpleClientGoalState::SUCCEEDED:
		reached_active_goal_=true;
		break;
	case actionlib::SimpleClientGoalState::ACTIVE:
	case actionlib::SimpleClientGoalState::PENDING:
	case actionlib::SimpleClientGoalState::RECALLED:
		break;
	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::PREEMPTED:
	case actionlib::SimpleClientGoalState::ABORTED:
		move_to_object_safe_state_=FINISHEDWITHERROR;
		motion_planning_result_ = *result;
		break;
	default:
		break;
	}
}

int Statemachine::move_to_object_vision()
{
	if(move_to_object_vision_state_==OPEN)
	{
		ROS_INFO("move_to_object_vision() called: OPEN");

#if 0
		//send goals to motion-planning
		active_goal_=0;
		nr_goals_=1;
		reached_active_goal_=false;
		goal_queue.resize(nr_goals_);

		goal_queue[0].goal_pose = object_vision_pose[selected_object_pose_];
		goal_queue[0].planning_algorithm = planning_mode_.object;
		goal_queue[0].planning_frame = GP_TCP;
		goal_queue[0].inter_steps = 0;
		goal_queue[0].speed_percentage = std_moving_speed*(1-speed_mod_);
		goal_queue[0].allowed_time = 60.0;
#else
#warning Fuer task 3 eigentlich nicht noetig -> nur fuer test zwecke
		if((active_task_number_ == 3 || active_task_number_ == 4)&& (cur_obj_gripped_==false))
		{
			//send goals to motion-planning
			active_goal_=0;
			nr_goals_=3;
			reached_active_goal_=false;
			goal_queue.resize(nr_goals_);

			goal_queue[0].goal_pose = object_vision_pose[selected_object_pose_];
			goal_queue[0].planning_algorithm = HOMING_MOVE_IT_7DOF;
			goal_queue[0].planning_frame = GP_TCP;
			goal_queue[0].inter_steps = 0;
			goal_queue[0].speed_percentage = std_moving_speed*(1-speed_mod_);
			goal_queue[0].allowed_time = 60.0;

			goal_queue[1].goal_pose.position.x = object_vision_pose[selected_object_pose_].position.x;
			goal_queue[1].goal_pose.position.y = object_vision_pose[selected_object_pose_].position.y;
			goal_queue[1].goal_pose.position.z = 0.0;
			goal_queue[1].planning_algorithm = MOVE_IT_2DOF;
			goal_queue[1].planning_frame = LWR_0; // should not affect MP
			goal_queue[1].inter_steps = 0;
			goal_queue[1].speed_percentage = std_moving_speed*(1-speed_mod_);
			goal_queue[1].allowed_time = 60.0;

			goal_queue[2].goal_pose = object_vision_pose[selected_object_pose_];
			goal_queue[2].planning_algorithm = MOVE_IT_9DOF;
			goal_queue[2].planning_frame = GP_TCP;
			goal_queue[2].inter_steps = 0;
			goal_queue[2].speed_percentage = std_moving_speed*(1-speed_mod_);
			goal_queue[2].allowed_time = 60.0;
		}
		else
		{
			//send goals to motion-planning
			active_goal_=0;
			nr_goals_=1;
			reached_active_goal_=false;
			goal_queue.resize(nr_goals_);

			goal_queue[0].goal_pose = object_vision_pose[selected_object_pose_];
			goal_queue[0].planning_algorithm = planning_mode_.object;
			goal_queue[0].planning_frame = GP_TCP;
			goal_queue[0].inter_steps = 0;
			goal_queue[0].speed_percentage = std_moving_speed*(1-speed_mod_);
			goal_queue[0].allowed_time = 60.0;
		}
#endif

		//send first goal
		move_to_object_vision_state_=RUNNING;
		motion_planning_action_client_->sendGoal(goal_queue[0],
				boost::bind(&Statemachine::move_to_object_vision_done,this,_1,_2),
				motionClient::SimpleActiveCallback(), //Statemachine::move_to_object_vision_active(),
				motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::move_to_object_vision_feedback,this,_1));
		);
	}
	else if (move_to_object_vision_state_==RUNNING && reached_active_goal_==true)
	{
		reached_active_goal_=false;
		active_goal_++;


		if(active_goal_==nr_goals_)
		{
			move_to_object_vision_state_=FINISHED;
			return 0;
		}
		else
		{
			ROS_INFO("move_to_object_vision() called: RUNNING (remaining goals)");
		}

		motion_planning_action_client_->sendGoal(goal_queue[active_goal_],
				boost::bind(&Statemachine::move_to_object_vision_done,this,_1,_2),
				motionClient::SimpleActiveCallback(), //Statemachine::move_to_object_vision_active(),
				motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::move_to_object_vision_feedback,this,_1));
		);
	}
	else if(move_to_object_vision_state_==FINISHED)
	{
		ROS_INFO("move_to_object_vision() called: FINISHED");

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		move_to_object_vision_state_=OPEN;
		move_to_object_vision_counter_=0;
	}
	else if(move_to_object_vision_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("move_to_object_vision() called: FINISHEDWITHERROR");
		state_.sub.event_two = motion_planning_result_.error_reason;
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::move_to_object_vision_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback)
{
	ROS_INFO("move_to_object_vision_feedback() called");
}

void Statemachine::move_to_object_vision_done(const actionlib::SimpleClientGoalState& state,
		const am_msgs::goalPoseResultConstPtr& result)
{
	ROS_INFO("move_to_object_vision_done() called, state: %s",state.toString().c_str());

	switch(state.state_)
	{
	case actionlib::SimpleClientGoalState::SUCCEEDED:
		reached_active_goal_=true;
		break;
	case actionlib::SimpleClientGoalState::ACTIVE:
	case actionlib::SimpleClientGoalState::PENDING:
	case actionlib::SimpleClientGoalState::RECALLED:
		break;
	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::PREEMPTED:
	case actionlib::SimpleClientGoalState::ABORTED:
		move_to_object_vision_state_=FINISHEDWITHERROR;
		motion_planning_result_ = *result;
		break;
	default:
		break;
	}
}

int Statemachine::move_to_object()
{
	if(move_to_object_state_==OPEN)
	{
		ROS_INFO("move_to_object() called: OPEN");


		//publish object state for motion planning
		if(cur_obj_gripped_==false)
			publish_obj_state(OBJ_GRIPPING);

#warning remove area from octomap
		if (!skip_vision_){
			rm_grasping_area_collision_srv_.request.max.x = object_grip_pose[selected_object_pose_].position.x + 0.04;
			rm_grasping_area_collision_srv_.request.max.y = object_grip_pose[selected_object_pose_].position.y + 0.04;
			rm_grasping_area_collision_srv_.request.max.z = 1.0;
			rm_grasping_area_collision_srv_.request.min.x = object_grip_pose[selected_object_pose_].position.x - 0.04;
			rm_grasping_area_collision_srv_.request.min.y = object_grip_pose[selected_object_pose_].position.y - 0.04;
			rm_grasping_area_collision_srv_.request.min.z = 0.0;
			try{
				if (!rm_grasping_area_collision_client_.call(rm_grasping_area_collision_srv_))
					msg_warn("Grasping area clearing failed.");
			}catch (...){msg_error("Grasping area clearing failed.");}
		}
		//send goals to motion-planning
		active_goal_=0;
		nr_goals_=1;
		goal_queue.resize(nr_goals_);

		goal_queue[0].goal_pose = object_grip_pose[selected_object_pose_];

#warning TO_DISCUSS
		//goal_queue[0].planning_algorithm = STANDARD_IK_7DOF;
		goal_queue[0].planning_algorithm = planning_mode_.move_to_object;
		goal_queue[0].planning_frame = GP_TCP;
		goal_queue[0].inter_steps = std_inter_steps;
		goal_queue[0].speed_percentage = slow_moving_speed*(1-speed_mod_);
		goal_queue[0].allowed_time = 60.0;


		move_to_object_state_=RUNNING;
		motion_planning_action_client_->sendGoal(goal_queue[0],
				boost::bind(&Statemachine::move_to_object_done,this,_1,_2),
				motionClient::SimpleActiveCallback(), //Statemachine::move_to_object_active(),
				motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::move_to_object_feedback,this,_1));
		);
	}
	else if(move_to_object_state_==FINISHED)
	{
		ROS_INFO("move_to_object() called: FINISHED");

		//calculate transformation from gp tcp to object origin for grapsing with object_grip_pose[selected_object_pose_]
		tf::Transform gp;
		gp.setOrigin(tf::Vector3(goal_queue[0].goal_pose.position.x,
				goal_queue[0].goal_pose.position.y,goal_queue[0].goal_pose.position.z));
		gp.setRotation(tf::Quaternion(goal_queue[0].goal_pose.orientation.x,
				goal_queue[0].goal_pose.orientation.y,goal_queue[0].goal_pose.orientation.z,
				goal_queue[0].goal_pose.orientation.w));

		tf::Transform obj_orig;
		obj_orig.setOrigin(tf::Vector3(cur_obj_.abs_pose.position.x,
				cur_obj_.abs_pose.position.y,cur_obj_.abs_pose.position.z));
		obj_orig.setRotation(tf::Quaternion(cur_obj_.abs_pose.orientation.x,
				cur_obj_.abs_pose.orientation.y,cur_obj_.abs_pose.orientation.z,
				cur_obj_.abs_pose.orientation.w));

		gp_obj_orig_.mult(gp.inverse(),obj_orig);

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		move_to_object_state_=OPEN;
		move_to_object_counter_=0;
	}
	else if(move_to_object_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("move_to_object() called: FINISHEDWITHERROR");
		state_.sub.event_three = motion_planning_result_.error_reason;
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::move_to_object_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback)
{
	ROS_INFO("move_to_object_feedback() called");
}

void Statemachine::move_to_object_done(const actionlib::SimpleClientGoalState& state,
		const am_msgs::goalPoseResultConstPtr& result)
{
	ROS_INFO("move_to_object_done() called, state: %s",state.toString().c_str());

	switch(state.state_)
	{
	case actionlib::SimpleClientGoalState::SUCCEEDED:
		move_to_object_state_=FINISHED;
		break;
	case actionlib::SimpleClientGoalState::ACTIVE:
	case actionlib::SimpleClientGoalState::PENDING:
	case actionlib::SimpleClientGoalState::RECALLED:
		break;
	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::PREEMPTED:
	case actionlib::SimpleClientGoalState::ABORTED:
		move_to_object_state_=FINISHEDWITHERROR;
		motion_planning_result_ = *result;
		break;
	default:
		break;
	}
}

int Statemachine::move_to_object_t6()
{
	if(move_to_object_t6_state_==OPEN)
	{
		ROS_INFO("move_to_object_t6() called: OPEN");

		//send goals to motion-planning
		active_goal_=0;
		nr_goals_=1;
		goal_queue.resize(nr_goals_);

		goal_queue[0].goal_pose = object_grip_pose[selected_object_pose_];

#warning TO_DISCUSS
		//goal_queue[0].planning_algorithm = STANDARD_IK_7DOF;
		goal_queue[0].planning_algorithm = planning_mode_.move_to_object;
		goal_queue[0].planning_frame = GP_TCP;
		goal_queue[0].inter_steps = 0;
		goal_queue[0].speed_percentage = slow_moving_speed;
		goal_queue[0].allowed_time = 60.0;
		goal_queue[0].stamp = ein_->get_active_object_stamp();


		move_to_object_t6_state_=RUNNING;
		motion_planning_action_client_->sendGoal(goal_queue[0],
				boost::bind(&Statemachine::move_to_object_t6_done,this,_1,_2),
				motionClient::SimpleActiveCallback(), //Statemachine::move_to_object_active(),
				motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::move_to_object_feedback,this,_1));
		);
	}
	else if(move_to_object_t6_state_==FINISHED)
	{
		ROS_INFO("move_to_object_t6() called: FINISHED");

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		move_to_object_t6_state_=OPEN;
		move_to_object_t6_counter_=0;
	}
	else if(move_to_object_t6_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("move_to_object_t6() called: FINISHEDWITHERROR");
		state_.sub.event_two = motion_planning_result_.error_reason;
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::move_to_object_t6_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback)
{
	ROS_INFO("move_to_object_t6_feedback() called");
}

void Statemachine::move_to_object_t6_done(const actionlib::SimpleClientGoalState& state,
		const am_msgs::goalPoseResultConstPtr& result)
{
	ROS_INFO("move_to_object_t6_done() called, state: %s",state.toString().c_str());

	switch(state.state_)
	{
	case actionlib::SimpleClientGoalState::SUCCEEDED:
		move_to_object_t6_state_=FINISHED;
		break;
	case actionlib::SimpleClientGoalState::ACTIVE:
	case actionlib::SimpleClientGoalState::PENDING:
	case actionlib::SimpleClientGoalState::RECALLED:
		break;
	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::PREEMPTED:
	case actionlib::SimpleClientGoalState::ABORTED:
		move_to_object_t6_state_=FINISHEDWITHERROR;
		motion_planning_result_ = *result;
		break;
	default:
		break;
	}
}

int Statemachine::move_to_target_zone_safe()
{
	if(move_to_target_zone_safe_state_==OPEN)
	{
		ROS_INFO("move_to_target_zone_safe() called: OPEN");


#if 0
		//send goals to motion-planning
		active_goal_=0;
		nr_goals_=1;
		reached_active_goal_=false;
		goal_queue.resize(nr_goals_);

		goal_queue[0].goal_pose = target_safe_pose[selected_target_pose_];
		goal_queue[0].planning_algorithm = planning_mode_.target;
		goal_queue[0].planning_frame = GP_TCP;
		goal_queue[0].inter_steps = 0;
		goal_queue[0].speed_percentage = std_moving_speed*(1-speed_mod_);
		goal_queue[0].allowed_time = 60.0;
#else
#warning Fuer task 3 eigentlich nicht noetig -> nur fuer test zwecke
		if((active_task_number_ == 3 || active_task_number_ == 4) && (cur_obj_gripped_==true))
		{
			//send goals to motion-planning
			active_goal_=0;
			nr_goals_=3;
			reached_active_goal_=false;
			goal_queue.resize(nr_goals_);

			goal_queue[0].goal_pose = target_safe_pose[selected_target_pose_];
			goal_queue[0].planning_algorithm = HOMING_MOVE_IT_7DOF;
			goal_queue[0].planning_frame = GP_TCP;
			goal_queue[0].inter_steps = 0;
			goal_queue[0].speed_percentage = std_moving_speed*(1-speed_mod_);
			goal_queue[0].allowed_time = 60.0;

			goal_queue[1].goal_pose.position.x = target_safe_pose[selected_target_pose_].position.x;
			goal_queue[1].goal_pose.position.y = target_safe_pose[selected_target_pose_].position.y;
			goal_queue[1].goal_pose.position.z = 0.0;
			goal_queue[1].planning_algorithm = MOVE_IT_2DOF;
			goal_queue[1].planning_frame = LWR_0; // should not affect MP
			goal_queue[1].inter_steps = 0;
			goal_queue[1].speed_percentage = std_moving_speed;//*(1-speed_mod_);
			goal_queue[1].allowed_time = 60.0;

			goal_queue[2].goal_pose = target_safe_pose[selected_target_pose_];
			goal_queue[2].planning_algorithm = MOVE_IT_9DOF;
			goal_queue[2].planning_frame = GP_TCP;
			goal_queue[2].inter_steps = 0;
			goal_queue[2].speed_percentage = std_moving_speed*(1-speed_mod_);
			goal_queue[2].allowed_time = 60.0;
		}
		else
		{
			//send goals to motion-planning
			active_goal_=0;
			nr_goals_=1;
			reached_active_goal_=false;
			goal_queue.resize(nr_goals_);

			goal_queue[0].goal_pose = target_safe_pose[selected_target_pose_];
			goal_queue[0].planning_algorithm = planning_mode_.target;
			goal_queue[0].planning_frame = GP_TCP;
			goal_queue[0].inter_steps = 0;
			goal_queue[0].speed_percentage = std_moving_speed*(1-speed_mod_);
			goal_queue[0].allowed_time = 60.0;
		}
#endif


		move_to_target_zone_safe_state_=RUNNING;
		motion_planning_action_client_->sendGoal(goal_queue[0],
				boost::bind(&Statemachine::move_to_target_zone_safe_done,this,_1,_2),
				motionClient::SimpleActiveCallback(), //Statemachine::move_to_target_zone_safe_active(),
				motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::move_to_target_zone_safe_feedback,this,_1));
		);
	}
	else if(move_to_target_zone_safe_state_==RUNNING && reached_active_goal_==true)
	{
		reached_active_goal_=false;
		active_goal_++;
		if(active_goal_==nr_goals_)
		{
			move_to_target_zone_safe_state_=FINISHED;
			return 0;
		}
		else
		{
			ROS_INFO("move_to_target_zone_safe() called: RUNNING (remaining goals)");
		}

		motion_planning_action_client_->sendGoal(goal_queue[active_goal_],
				boost::bind(&Statemachine::move_to_target_zone_safe_done,this,_1,_2),
				motionClient::SimpleActiveCallback(), //Statemachine::move_to_target_zone_safe_active(),
				motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::move_to_target_zone_safe_feedback,this,_1));
		);
	}
	else if(move_to_target_zone_safe_state_==FINISHED)
	{
		ROS_INFO("move_to_target_zone_safe() called: FINISHED");


		//publish object state for motion planning
		if(cur_obj_gripped_==false)
			publish_obj_state(OBJ_FINISHED);

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		move_to_target_zone_safe_state_=OPEN;
		move_to_target_zone_safe_counter_=0;
	}
	else if(move_to_target_zone_safe_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("move_to_target_zone_safe() called: FINISHEDWITHERROR");
		state_.sub.event_two = motion_planning_result_.error_reason;
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::move_to_target_zone_safe_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback)
{
	ROS_INFO("move_to_target_zone_safe_feedback() called");
}

void Statemachine::move_to_target_zone_safe_done(const actionlib::SimpleClientGoalState& state,
		const am_msgs::goalPoseResultConstPtr& result)
{
	ROS_INFO("move_to_target_zone_safe_done() called, state: %s",state.toString().c_str());

	switch(state.state_)
	{
	case actionlib::SimpleClientGoalState::SUCCEEDED:
		reached_active_goal_=true;
		break;
	case actionlib::SimpleClientGoalState::ACTIVE:
	case actionlib::SimpleClientGoalState::PENDING:
	case actionlib::SimpleClientGoalState::RECALLED:
		break;
	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::PREEMPTED:
	case actionlib::SimpleClientGoalState::ABORTED:
		move_to_target_zone_safe_state_=FINISHEDWITHERROR;
		motion_planning_result_ = *result;
		break;
	default:
		break;
	}
}

int Statemachine::move_to_target_zone_vision()
{
	if(move_to_target_zone_vision_state_==OPEN)
	{
		ROS_INFO("move_to_target_zone_vision() called: OPEN");

		if((ein_->get_active_object_state()==EurocInput::EIN_OBJ_PARKING)
				|| (cur_obj_.nr_shapes>1))
		{
			move_to_target_zone_vision_state_=FINISHED;
			return 0;
		}


		//send goals to motion-planning
		active_goal_=0;
		nr_goals_=1;
		reached_active_goal_=false;
		goal_queue.resize(nr_goals_);

		goal_queue[0].goal_pose = target_vision_pose[selected_target_pose_];
		goal_queue[0].planning_algorithm = planning_mode_.target;
		goal_queue[0].planning_frame = GP_TCP;
		goal_queue[0].inter_steps = 0;
		goal_queue[0].speed_percentage = std_moving_speed*(1-speed_mod_);
		goal_queue[0].allowed_time = 60.0;


		move_to_target_zone_vision_state_=RUNNING;
		motion_planning_action_client_->sendGoal(goal_queue[0],
				boost::bind(&Statemachine::move_to_target_zone_vision_done,this,_1,_2),
				motionClient::SimpleActiveCallback(), //Statemachine::move_to_target_zone_vision_active(),
				motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::move_to_target_zone_vision_feedback,this,_1));
		);
	}
	else if(move_to_target_zone_vision_state_==RUNNING && reached_active_goal_==true)
	{
		reached_active_goal_=false;
		active_goal_++;
		if(active_goal_==nr_goals_)
		{
			move_to_target_zone_vision_state_=FINISHED;
			return 0;
		}
		else
		{
			ROS_INFO("move_to_target_zone_vision() called: RUNNING (remaining goals)");
		}

		motion_planning_action_client_->sendGoal(goal_queue[active_goal_],
				boost::bind(&Statemachine::move_to_target_zone_vision_done,this,_1,_2),
				motionClient::SimpleActiveCallback(), //Statemachine::move_to_target_zone_vision_active(),
				motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::move_to_target_zone_vision_feedback,this,_1));
		);
	}
	else if(move_to_target_zone_vision_state_==FINISHED)
	{
		ROS_INFO("move_to_target_zone_vision() called: FINISHED");

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		move_to_target_zone_vision_state_=OPEN;
		move_to_target_zone_vision_counter_=0;
	}
	else if(move_to_target_zone_vision_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("move_to_target_zone_vision() called: FINISHEDWITHERROR");
		state_.sub.event_two = motion_planning_result_.error_reason;
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::move_to_target_zone_vision_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback)
{
	ROS_INFO("move_to_target_zone_vision_feedback() called");
}

void Statemachine::move_to_target_zone_vision_done(const actionlib::SimpleClientGoalState& state,
		const am_msgs::goalPoseResultConstPtr& result)
{
	ROS_INFO("move_to_target_zone_vision_done() called, state: %s",state.toString().c_str());

	switch(state.state_)
	{
	case actionlib::SimpleClientGoalState::SUCCEEDED:
		reached_active_goal_=true;
		break;
	case actionlib::SimpleClientGoalState::ACTIVE:
	case actionlib::SimpleClientGoalState::PENDING:
	case actionlib::SimpleClientGoalState::RECALLED:
		break;
	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::PREEMPTED:
	case actionlib::SimpleClientGoalState::ABORTED:
		move_to_target_zone_vision_state_=FINISHEDWITHERROR;
		motion_planning_result_ = *result;
		break;
	default:
		break;
	}
}

int Statemachine::move_to_target_zone()
{
	if(move_to_target_zone_state_==OPEN)
	{
		ROS_INFO("move_to_target_zone() called: OPEN");

		//send goals to motion-planning
		active_goal_=0;
		nr_goals_=1;
		goal_queue.resize(nr_goals_);

		goal_queue[0].goal_pose = target_place_pose[selected_target_pose_];
#warning TO_DISCUSS
		//goal_queue[0].planning_algorithm = STANDARD_IK_7DOF;
		goal_queue[0].planning_algorithm = planning_mode_.move_to_target_zone;
		goal_queue[0].planning_frame = GP_TCP;
		goal_queue[0].inter_steps = std_inter_steps;
		goal_queue[0].speed_percentage = slow_moving_speed*(1-speed_mod_);
		goal_queue[0].allowed_time = 60.0;


		move_to_target_zone_state_=RUNNING;
		motion_planning_action_client_->sendGoal(goal_queue[0],
				boost::bind(&Statemachine::move_to_target_zone_done,this,_1,_2),
				motionClient::SimpleActiveCallback(), //Statemachine::move_to_target_zone_active(),
				motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::move_to_target_zone_feedback,this,_1));
		);
	}
	else if(move_to_target_zone_state_==FINISHED)
	{
		ROS_INFO("move_to_target_zone() called: FINISHED");

		geometry_msgs::Pose tmp_pose;
		//calculate transformation from gp tcp to object origin for grapsing with object_grip_pose[selected_object_pose_]
		tf::Transform gp;
		gp.setOrigin(tf::Vector3(goal_queue[0].goal_pose.position.x,
				goal_queue[0].goal_pose.position.y,goal_queue[0].goal_pose.position.z));
		gp.setRotation(tf::Quaternion(goal_queue[0].goal_pose.orientation.x,
				goal_queue[0].goal_pose.orientation.y,goal_queue[0].goal_pose.orientation.z,
				goal_queue[0].goal_pose.orientation.w));

		tf::Transform obj_orig;
		obj_orig.mult(gp,gp_obj_orig_);
		tmp_pose.position.x=obj_orig.getOrigin().getX();
		tmp_pose.position.y=obj_orig.getOrigin().getY();
		tmp_pose.position.z=obj_orig.getOrigin().getZ();
		tmp_pose.orientation.x=obj_orig.getRotation().getX();
		tmp_pose.orientation.y=obj_orig.getRotation().getY();
		tmp_pose.orientation.z=obj_orig.getRotation().getZ();
		tmp_pose.orientation.w=obj_orig.getRotation().getW();

		//gp_obj_orig_=
		ein_->set_object_pose(tmp_pose, ros::Time::now());

		//publish object state for motion planning
		publish_obj_state(OBJ_PLACED);
		cur_obj_gripped_=false;

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		move_to_target_zone_state_=OPEN;
		move_to_target_zone_counter_=0;
	}
	else if(move_to_target_zone_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("move_to_target_zone() called: FINISHEDWITHERROR");
		state_.sub.event_three = motion_planning_result_.error_reason;
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::move_to_target_zone_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback)
{
	ROS_INFO("move_to_target_zone_feedback() called");
}

void Statemachine::move_to_target_zone_done(const actionlib::SimpleClientGoalState& state,
		const am_msgs::goalPoseResultConstPtr& result)
{
	ROS_INFO("move_to_target_zone_done() called, state: %s",state.toString().c_str());

	switch(state.state_)
	{
	case actionlib::SimpleClientGoalState::SUCCEEDED:
		move_to_target_zone_state_=FINISHED;
		break;
	case actionlib::SimpleClientGoalState::ACTIVE:
	case actionlib::SimpleClientGoalState::PENDING:
	case actionlib::SimpleClientGoalState::RECALLED:
		break;
	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::PREEMPTED:
	case actionlib::SimpleClientGoalState::ABORTED:
		move_to_target_zone_state_=FINISHEDWITHERROR;
		motion_planning_result_ = *result;
		break;
	default:
		break;
	}
}

int Statemachine::move_to_target_zone_t6()
{
	if(move_to_target_zone_t6_state_==OPEN)
	{
		ROS_INFO("move_to_target_zone_t6() called: OPEN");

		//send goals to motion-planning
		active_goal_=0;
		nr_goals_=1;
		goal_queue.resize(nr_goals_);

		goal_queue[0].goal_pose = target_place_pose[selected_target_pose_];
#warning TO_DISCUSS
		//goal_queue[0].planning_algorithm = STANDARD_IK_7DOF;
		goal_queue[0].planning_algorithm = planning_mode_.move_to_target_zone;
		goal_queue[0].planning_frame = GP_TCP;
		goal_queue[0].inter_steps = std_inter_steps;
		goal_queue[0].speed_percentage = slow_moving_speed*(1-speed_mod_);
		goal_queue[0].allowed_time = 60.0;

		move_to_target_zone_t6_state_=RUNNING;
		motion_planning_action_client_->sendGoal(goal_queue[0],
				boost::bind(&Statemachine::move_to_target_zone_t6_done,this,_1,_2),
				motionClient::SimpleActiveCallback(), //Statemachine::move_to_target_zone_active(),
				motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::move_to_target_zone_feedback,this,_1));
		);
	}
	else if(move_to_target_zone_t6_state_==FINISHED)
	{
		ROS_INFO("move_to_target_zone_t6() called: FINISHED");

		geometry_msgs::Pose tmp_pose;
		//calculate transformation from gp tcp to object origin for grapsing with object_grip_pose[selected_object_pose_]
		tf::Transform gp;
		gp.setOrigin(tf::Vector3(goal_queue[0].goal_pose.position.x,
				goal_queue[0].goal_pose.position.y,goal_queue[0].goal_pose.position.z));
		gp.setRotation(tf::Quaternion(goal_queue[0].goal_pose.orientation.x,
				goal_queue[0].goal_pose.orientation.y,goal_queue[0].goal_pose.orientation.z,
				goal_queue[0].goal_pose.orientation.w));

		tf::Transform obj_orig;
		obj_orig.mult(gp,gp_obj_orig_);
		tmp_pose.position.x=obj_orig.getOrigin().getX();
		tmp_pose.position.y=obj_orig.getOrigin().getY();
		tmp_pose.position.z=obj_orig.getOrigin().getZ();
		tmp_pose.orientation.x=obj_orig.getRotation().getX();
		tmp_pose.orientation.y=obj_orig.getRotation().getY();
		tmp_pose.orientation.z=obj_orig.getRotation().getZ();
		tmp_pose.orientation.w=obj_orig.getRotation().getW();

		//gp_obj_orig_=
		ein_->set_object_pose(tmp_pose, ros::Time::now());

		//publish object state for motion planning
		publish_obj_state(OBJ_PLACED);
		cur_obj_gripped_=false;

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		move_to_target_zone_t6_state_=OPEN;
		move_to_target_zone_t6_counter_=0;
	}
	else if(move_to_target_zone_t6_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("move_to_target_zone_t6() called: FINISHEDWITHERROR");
		state_.sub.event_two = motion_planning_result_.error_reason;
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::move_to_target_zone_t6_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback)
{
	ROS_INFO("move_to_target_zone_t6_feedback() called");
}

void Statemachine::move_to_target_zone_t6_done(const actionlib::SimpleClientGoalState& state,
		const am_msgs::goalPoseResultConstPtr& result)
{
	ROS_INFO("move_to_target_zone_done_t6() called, state: %s",state.toString().c_str());

	switch(state.state_)
	{
	case actionlib::SimpleClientGoalState::SUCCEEDED:
		move_to_target_zone_t6_state_=FINISHED;
		break;
	case actionlib::SimpleClientGoalState::ACTIVE:
	case actionlib::SimpleClientGoalState::PENDING:
	case actionlib::SimpleClientGoalState::RECALLED:
		break;
	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::PREEMPTED:
	case actionlib::SimpleClientGoalState::ABORTED:
		move_to_target_zone_t6_state_=FINISHEDWITHERROR;
		motion_planning_result_ = *result;
		break;
	default:
		break;
	}
}

int Statemachine::homing()
{
	if(homing_state_==OPEN)
	{
		ROS_INFO("homing() called: OPEN");

		//send goals to motion-planning
#warning homing interferes with explore environment state and overwrites nr_goals_ and active_goal_
		//		active_goal_=0;
		//		nr_goals_=1;
		goal_queue.resize(1);

		goal_queue[0].planning_algorithm = planning_mode_.homing;
		goal_queue[0].inter_steps = 0;
		goal_queue[0].speed_percentage = std_moving_speed*(1-speed_mod_);
		goal_queue[0].allowed_time = 60.0;


		homing_state_=RUNNING;
		motion_planning_action_client_->sendGoal(goal_queue[0],
				boost::bind(&Statemachine::homing_done,this,_1,_2),
				motionClient::SimpleActiveCallback(), //Statemachine::homing_active(),
				motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::homing_feedback,this,_1));
		);
	}
	else if(homing_state_==FINISHED)
	{
		ROS_INFO("homing() called: FINISHED");

		//==============================================
		scheduler_next();
		//==============================================
		//reset state
		homing_state_=OPEN;
		homing_counter_=0;
	}
	else if(homing_state_==FINISHEDWITHERROR)
	{
		ROS_INFO("homing() called: FINISHEDWITHERROR");
		state_.sub.event_two = motion_planning_result_.error_reason;
		scheduler_schedule(); //Call for Error-Handling
	}
	return 0;
}

void Statemachine::homing_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback)
{
	ROS_INFO("homing_feedback() called");
}

void Statemachine::homing_done(const actionlib::SimpleClientGoalState& state,
		const am_msgs::goalPoseResultConstPtr& result)
{
	ROS_INFO("homing_done() called, state: %s",state.toString().c_str());

	switch(state.state_)
	{
	case actionlib::SimpleClientGoalState::SUCCEEDED:
		homing_state_=FINISHED;
		break;
	case actionlib::SimpleClientGoalState::ACTIVE:
	case actionlib::SimpleClientGoalState::PENDING:
	case actionlib::SimpleClientGoalState::RECALLED:
		break;
	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::PREEMPTED:
	case actionlib::SimpleClientGoalState::ABORTED:
		homing_state_=FINISHEDWITHERROR;
		motion_planning_result_ = *result;
		break;
	default:
		break;
	}
}

void Statemachine::publish_obj_state(uint16_t state)
{
	obj_state_msg_.obj_index=ein_->get_active_object_idx();
	obj_state_msg_.obj_state=state;

	switch(state)
	{
	case OBJ_NOT_LOCATED:
		ROS_INFO("Statemachine: publishing state OBJ_NOT_LOCATED for object %s",cur_obj_.name.c_str());
		break;
	case OBJ_LOCATED:
		obj_state_msg_.obj_pose=cur_obj_.abs_pose;
		ROS_INFO("Statemachine: publishing state OBJ_LOCATED for object %s",cur_obj_.name.c_str());
		break;
	case OBJ_GRIPPING:
		obj_state_msg_.obj_pose=cur_obj_.abs_pose;
		ROS_INFO("Statemachine: publishing state OBJ_GRIPPING for object %s",cur_obj_.name.c_str());
		break;
	case OBJ_GRABED:
		obj_state_msg_.obj_pose=cur_obj_.abs_pose;
		ROS_INFO("Statemachine: publishing state OBJ_GRABED for object %s",cur_obj_.name.c_str());
		break;
	case OBJ_PLACED:
		obj_state_msg_.obj_pose=target_place_pose[selected_target_pose_];
		ROS_INFO("Statemachine: publishing state OBJ_PLACED for object %s",cur_obj_.name.c_str());
		break;
	case OBJ_FINISHED:
	{
		geometry_msgs::Pose empty_pose;
		obj_state_msg_.obj_pose=empty_pose;
		ROS_INFO("Statemachine: publishing state OBJ_FINISHED for object %s",cur_obj_.name.c_str());
		break;
	}
	default:
		msg_error("Unknown object state !!!");
		break;
	}

	obj_state_.publish(obj_state_msg_);
}

int Statemachine::check_time()
{
	double t_act=(double)ros::Time::now().toSec();

	//
	static bool first=true;

	bool stop_needed=(sim_running_ && !(state_.sub.one==fsm::STOP_SIM || state_queue[0].sub.one==fsm::STOP_SIM));
	if(stop_needed && t_act >= ein_->get_time_limit())
	{
		if(first)
		{
			msg_info("%f seconds are over! -> soft reset, next state is STOP_SIM",ein_->get_time_limit());
			first=false;
		}
#ifndef ONE_TASK
		//soft shutdown of current task
		state_queue.clear();

		fsm::fsm_state_t temp_state;
		//state:
		temp_state.sub.one=fsm::STOP_SIM;
		state_queue.push_back(temp_state);
		//temp_state.sub.one=fsm::RESET;
		//state_queue.push_back(temp_state);
#endif
	}
	else if(sim_running_ && state_queue[0].sub.one==fsm::STOP_SIM && t_act >= 1.1*ein_->get_time_limit())
	{
#ifndef ONE_TASK
		msg_warn("%f seconds are over! -> hard reset, next state is STOP_SIM",1.1*ein_->get_time_limit());
		//hard shutdown of current task
		state_queue.clear();

		//hard shutdown of current task
		state_queue.clear();

		fsm::fsm_state_t temp_state;
		//state:
		state_.sub.one=fsm::STOP_SIM;

		reset();
#endif
	}

	return 0;
}

int Statemachine::reset()
{
	if(reset_state_==OPEN)
	{
		msg_info("reseting all variables in Statemachine and EurocInput!");

		speed_mod_=0;
		nr_scenes_=0;
		active_scene_=-1;
		active_goal_=0;
		nr_goals_=0;
		reached_active_goal_=false;
		request_task_state_=OPEN;
		start_sim_state_=OPEN;
		set_object_load_state_=OPEN;
		pause_state_=OPEN;
		parse_yaml_file_state_=OPEN;
		stop_sim_state_=OPEN;
		watch_scene_state_=OPEN;
		watch_scene_counter_=0;
		explore_environment_init_state_=OPEN;
		explore_environment_motion_state_=OPEN;
		explore_environment_image_state_=OPEN;
		explore_environment_image_counter_=0;
		locate_object_global_state_=OPEN;
		locate_object_global_counter_=0;
		locate_object_close_range_state_=OPEN;
		check_object_finished_state_=OPEN;
		check_object_gripped_state_=OPEN;
		check_object_gripped_counter_=0;
		new_object_t6_state_=OPEN;
		get_grasping_pose_state_=OPEN;
        get_grasping_poseT5_state_=OPEN;
        get_grasping_poseT6_state_=OPEN;
		move_to_object_vision_state_=OPEN;
		move_to_object_vision_counter_=0;
		move_to_object_safe_state_=OPEN;
		move_to_object_safe_counter_=0;
		move_to_object_state_=OPEN;
		move_to_object_counter_=0;
		move_to_object_t6_state_=OPEN;
		move_to_object_t6_counter_=0;
		gripper_release_state_=OPEN;
		gripper_release_counter_=0;
		gripper_close_state_=OPEN;
		gripper_close_counter_=0;
		move_to_target_zone_safe_state_=OPEN;
		move_to_target_zone_safe_counter_=0;
		move_to_target_zone_vision_state_=OPEN;
		move_to_target_zone_vision_counter_=0;
		move_to_target_zone_state_=OPEN;
		move_to_target_zone_counter_=0;
		move_to_target_zone_t6_state_=OPEN;
		move_to_target_zone_t6_counter_=0;
		homing_state_=OPEN;
		homing_counter_=0;
		wait_counter_=0;

		ein_->reset();
		lsc_.detach();


		//send reset message:
		std_msgs::Bool rst;
		rst.data=true;
		reset_pub_.publish(rst);

		delete vision_action_client_;
		vision_action_client_=0x0;
		delete motion_planning_action_client_;

		reset_state_=RUNNING;
	}
	else if(reset_state_==RUNNING)
	{
		reset_counter_++;

		if(reset_counter_ > 10)
		{
			reset_state_=FINISHED;
			reset_counter_=0;
		}
	}
	else if(reset_state_==FINISHED)
	{


		reset_state_=OPEN;

		//test neu:
		state_.sub.one = fsm::INITIAL_STATE;
		scheduler_schedule();
	}

	return 0;
}

int Statemachine::wait()
{
	if(wait_counter_==0)
		ROS_INFO("Statemachine waiting for %f seconds",wait_duration);

	wait_counter_++;

	if(wait_counter_>FREQ*wait_duration)
	{
		wait_counter_=0;
		scheduler_next();
	}
}
