#include <statemachine.hpp>
#include <cmath>
#include <StaticTFBroadcaster.h>

Statemachine* Statemachine::instance_ = 0x0;

Statemachine::Statemachine():
	    scenes_(1),
		task_active_(false),
		sim_running_(false),
		nr_scenes_(0),
		active_scene_(-1),
		motion_planning_action_client_("goalPoseAction", true),
		vision_action_client_("VisionAction", true),
		active_goal_(0),
		nr_goals_(0),
		request_task_state_(OPEN),
		start_sim_state_(OPEN),
		stop_sim_state_(OPEN),
		grip_state_(OPEN),
		vision_state_(OPEN),
		motion_state_(OPEN)
{
	ein_=new EurocInput();
	broadcaster_ = new StaticTFBroadcaster();

	//==============================================
	//state:
	state_.sub.one = fsm::REQUEST_TASK;
	//==============================================

	task_selector_ = "/euroc_c2_task_selector";
	list_scenes_ = (task_selector_ + "/list_scenes");
	start_simulator_ = (task_selector_ + "/start_simulator");
	stop_simulator_ = (task_selector_ + "/stop_simulator");
	euroc_c2_interface_ = "/euroc_interface_node";
	save_log_ = euroc_c2_interface_ + "/save_log";

}

Statemachine::~Statemachine()
{
	ROS_INFO("Killing Statemachine ...");
	if(sim_running_)
	{
		stop_sim();
	}

	//delete ein_;
	//delete state_obs_;
	//delete broadcaster_;

	clear_instance();
}

int Statemachine::init_sm()
{
	//wait for all services and servers
	ros::service::waitForService(list_scenes_,ros::Duration(5.0));
	ros::service::waitForService(start_simulator_,ros::Duration(5.0));
	ros::service::waitForService(stop_simulator_,ros::Duration(5.0));

	list_scenes_client_ = node_.serviceClient<euroc_c2_msgs::ListScenes>(list_scenes_);
	start_simulator_client_ = node_.serviceClient<euroc_c2_msgs::StartSimulator>(start_simulator_);
	stop_simulator_client_ = node_.serviceClient<euroc_c2_msgs::StopSimulator>(stop_simulator_);
	save_log_client_ = node_.serviceClient<euroc_c2_msgs::SaveLog>(save_log_);

	get_grasp_pose_client_ = node_.serviceClient<am_msgs::GetGraspPose>("GraspPose_srv");
	gripper_control_client_ = node_.serviceClient<am_msgs::GripperControl>("GripperInterface");

	ROS_INFO("Waiting for action server to start.");
	motion_planning_action_client_.waitForServer(ros::Duration(5.0));
	vision_action_client_.waitForServer(ros::Duration(5.0));
	ROS_INFO("Action server started.");

	return 0;
}

//void Statemachine::execute()
//{
//	ROS_INFO("Starting statemachine ...");
//	//update rate
//	ros::Rate loop_rate(FREQ);
//
//	//main loop:
//	double t_act=0;
//	ROS_INFO("Entering while-loop");
//	while(ros::ok() && !(state_.sub.one == fsm::FINISHED))
//	{
//
//	  //do state action ...
//	  if(-1==tick())
//	  {
//		  msg_error("Error in Statemachine::tick()");
//	  }
//
//	  //"tick"
//	  ros::spinOnce();
//
//	  //sleep for the remaining time (only during solve task)
//	  if(state_.sub.one == fsm::SOLVE_TASK)
//		  loop_rate.sleep();
//
//	  t_act=(double)ros::Time::now().toSec();
//
//	  printf("time= %f s \r",t_act);
//	  fflush(stdout);
//	}
//
//}

int Statemachine::tick()
{
	counter++;

	switch(state_.sub.one)
	{
	case fsm::REQUEST_TASK:
		return request_task();

	case fsm::START_SIM:
		return start_sim();

	case fsm::PARSE_YAML:
		return parse_yaml_file();

	case fsm::SOLVE_TASK:
		switch(state_.sub.two)
		{
		case fsm::LOCATE_OBJECT:
			return locate_object();

		case fsm::GET_GRASPING_POSE:
			return get_grasping_pose();

		case fsm::MOVE_TO_OBJECT:
			return move_to_object();

		case fsm::GRIP:
			return grip_object();

		case fsm::MOVE_TO_TARGET_ZONE:
			return move_to_target_zone();

		case fsm::HOMING:
			return homing();

		default:
			return solve_task();
		}
		break; //should not happen ?!

	case fsm::STOP_SIM:
		return stop_sim();

	default:
		msg_error("Unknown STATE!!!");
		return -1;
	}

}

void Statemachine::request_task_cb()
{
	ROS_INFO("In request_task_cb");
	request_task_state_=RUNNING;

	if(list_scenes_client_.exists())
		list_scenes_client_.call(list_scenes_srv_);
	else
		msg_error("list_scenes_client doesn't exist!");

	request_task_state_=FINISHED;
	ROS_INFO("Exiting request_task_cb");
}

int Statemachine::request_task()
{
	if(request_task_state_==OPEN)
	{
		//start the service call as thread and monitor request_task_state
		//lsc_ = boost::thread(&Statemachine::request_task_cb,this);
		request_task_cb();
	}
	else if(request_task_state_==FINISHED)//lsc_.timed_join(boost::posix_time::seconds(0.0)))
	{
		// The error_message field of each service response indicates whether an error occured. An empty string indicates success
		std::string &ls_error_message = list_scenes_srv_.response.error_message;
		if(!ls_error_message.empty()){
			ROS_ERROR("List scenes failed: %s", ls_error_message.c_str());
			return -1;
		}
		else
		{
			// Let's print the names of the received scenes
			ROS_INFO("[AM] Found the following scenes for the EuRoC C2 Simulation:");

			scenes_.resize(list_scenes_srv_.response.scenes.size());
			scenes_ = list_scenes_srv_.response.scenes;

			nr_scenes_ = scenes_.size();
			for(unsigned int ii = 0; ii < nr_scenes_; ++ii){
				euroc_c2_msgs::Scene &scene = scenes_[ii];
				ROS_INFO("[%2d] - %s", ii,scene.name.c_str());
			}
		}

		//destroy thread (should not be necessary?!)
		//lsc_.detach();


		std::cout<<"Choose task: ";
		int blub=8;
		//std::cin>>blub;
		active_scene_=blub;

		//ROS_INFO("Your Choice: %d", active_scene_);
		task_active_=true;


		//==============================================
		//state:
		state_.sub.one = fsm::START_SIM;
		//==============================================
	}
	return 0;
}

void Statemachine::start_sim_cb()
{
	start_sim_state_=RUNNING;

	if(start_simulator_client_.exists())
		start_simulator_client_.call(start_simulator_srv_);

	start_sim_state_=FINISHED;
}

int Statemachine::start_sim()
{
	if(start_sim_state_==OPEN)
	{
		ROS_INFO("Statemachine: Starting Server");

		start_simulator_srv_.request.user_id = "am-robotics";
		start_simulator_srv_.request.scene_name = scenes_[active_scene_].name;

		//lsc_ = boost::thread(&Statemachine::start_sim_cb,this);
		start_sim_cb();
	}
	else if(start_sim_state_==FINISHED)
	{
		// Check the response for errors
		std::string &error_message = start_simulator_srv_.response.error_message;
		if(!error_message.empty()){
			ROS_ERROR("Statemachine: Starting the simulator failed: %s", error_message.c_str());
			return -1;
		}

		//destroy thread
		//lsc_.detach();

		sim_running_=true;

		ROS_INFO("Statemachine: Server started");
		ros::service::waitForService(save_log_,ros::Duration(5.0));


		//==============================================
		//state:
		state_.sub.one = fsm::PARSE_YAML;
		//==============================================
	}
	return 0;
}

int Statemachine::parse_yaml_file()
{
	ROS_INFO("Statemachine: Parsing YAML-file");

	// The start simulator service returns a description of the selected task in yaml format
	std::string &task_yaml_description = start_simulator_srv_.response.description_yaml;

	//parse the yaml file -> content is saved in EurocInput-class
	if(-1==ein_->parse_yaml_file(task_yaml_description))
	{
		ROS_ERROR("Statemachine: Error in parse_yaml_file()");
	}

	ROS_INFO("Statemachine: Parsing YAML-file finished");

	ROS_INFO("Statemachine: Filling TF Broadcast information...");
	try {
		broadcaster_->fill_tf_information(ein_);
		ROS_INFO("Statemachine: Filling TF info successful.");
		broadcaster_->publish_static_tf();
	}
	catch (...)
	{
		msg_error("Filling up TF information failed.");
	}

	//==============================================
	//state:
	state_.sub.one = fsm::SOLVE_TASK;
	state_.sub.two = fsm::LOCATE_OBJECT;
	//==============================================

	return 0;
}

int Statemachine::solve_task()
{
	//do nothing ....
	//this should not be called!


	//==============================================
	//state:
	state_.sub.one = fsm::STOP_SIM;
	//==============================================
	return 0;
}

void Statemachine::stop_sim_cb()
{
	stop_sim_state_=RUNNING;

	//save the log
	ROS_INFO("Statemachine: Saving log");
	if(save_log_client_.exists())
		save_log_client_.call(save_log_srv_);
	else
		msg_error("save_log_client doesn't exist");

	ROS_INFO("Statemachine: Stopping server");

	// The stop simulator callback ends the current simulation
	if(stop_simulator_client_.exists())
		stop_simulator_client_.call(stop_simulator_srv_);
	else
		msg_error("stop_simulator_client doesn't exist");

	stop_sim_state_=FINISHED;
}
int Statemachine::stop_sim()
{
	if(stop_sim_state_==OPEN)
	{
		//lsc_ = boost::thread(&Statemachine::stop_sim_cb,this);
		stop_sim_cb();
	}
	else if(stop_sim_state_==FINISHED)
	{
		//lsc_.detach();

		sim_running_=false;

		//reset states:
		request_task_state_=OPEN;
		start_sim_state_=OPEN;
		stop_sim_state_=OPEN;

		//==============================================
		//state:
		state_.sub.one = fsm::FINISHED;
		//==============================================
	}

	return 0;
}

int Statemachine::locate_object()
{
	if(vision_state_==OPEN)
	{
		msg_info("Enter state");

		//hack: dont use blue handle!!
//		static bool first=true;
//		if(first)
//		{
//			ein_->get_object();
//			ein_->set_object_finished(); // jump to green cylinder
//			first=false;
//		}
		cur_obj_ = ein_->get_object();

		ein_->print_object(&cur_obj_);

		am_msgs::VisionGoal goal;
		goal.object = cur_obj_;
		goal.sensors.resize(ein_->get_nr_sensors());
		for(uint16_t ii=0;ii<ein_->get_nr_sensors();ii++)
		{
			goal.sensors[ii]=ein_->get_sensors(ii);
		}
		goal.target_zone=ein_->get_target_zone();

		//send goal to vision-node.
		//the callback function vision_done is registered to the action_done event
		vision_action_client_.sendGoal(goal,
										boost::bind(&Statemachine::vision_done,this,_1,_2),
										visionClient::SimpleActiveCallback(),
										visionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::vision_feedback,this,_1));
										);

		//set vision state to RUNNING and wait until it switches to FINISHED
		vision_state_=RUNNING;
	}
	else if(vision_state_==FINISHED)
	{
		//reset vision state
		vision_state_=OPEN;


		//==============================================
		//state:
		state_.sub.one = fsm::SOLVE_TASK;
		state_.sub.two = fsm::GET_GRASPING_POSE;
		//==============================================
	}

	return 0;
}

int Statemachine::get_grasping_pose()
{
	msg_info("Enter state");

	get_grasp_pose_srv_.request.object=cur_obj_;

	//call grasp service -> maybe call it in a seperate thread?!
	if(get_grasp_pose_client_.call(get_grasp_pose_srv_))
	{
		geometry_msgs::Pose *pose= &get_grasp_pose_srv_.response.grasp_pose;
		ROS_INFO("received pose: [%3.2f %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f]",
				pose->position.x,pose->position.y,pose->position.z,pose->orientation.w,
				pose->orientation.x,pose->orientation.y,pose->orientation.z);

		ein_->set_grasping_pose(get_grasp_pose_srv_.response.grasp_pose);
	}
	else
	{
		msg_error("Failed to call get_grasp_pose service.");
	}


	//==============================================
	//state:
	state_.sub.one = fsm::SOLVE_TASK;
	state_.sub.two = fsm::MOVE_TO_OBJECT;
	//==============================================
	return 0;
}

void Statemachine::grip_cb()
{
	//
	if(gripper_control_client_.exists())
	{
		if(gripper_control_client_.call(gripper_control_srv_))
		{
			if(gripper_control_srv_.request.gripping_mode==RELEASE)
				ROS_INFO("Gripper released");
			else
				ROS_INFO("Gripper closed");
		}
		else
			ROS_ERROR("Failed to call gripper control client");
	}

	grip_state_=FINISHED;
}

int Statemachine::move_to_object()
{
	if(grip_state_==OPEN)
	{
		msg_info("Enter state");


		//release gripper request
		gripper_control_srv_.request.gripping_mode = RELEASE;
		//start thread for gripper service call
		lsc_ = boost::thread(&Statemachine::grip_cb,this);
		grip_state_=RUNNING;
	}
	else if((motion_state_==OPEN) && (grip_state_==FINISHED))
	{
		//send goals to motion-planning
		active_goal_=0;
		nr_goals_=2;
		goal_queue.resize(nr_goals_);

		goal_queue[0].goal_pose = ein_->get_grasping_pose();
		goal_queue[0].goal_pose.position.z+=0.2;
		goal_queue[0].planning_algorithm = STANDARD_IK_7DOF;

		goal_queue[1]=goal_queue[0];
		goal_queue[1].goal_pose.position.z-=0.2;
		goal_queue[1].planning_algorithm = STANDARD_IK_7DOF;

		//send first goal
		motion_planning_action_client_.sendGoal(goal_queue[0],
												boost::bind(&Statemachine::motion_done,this,_1,_2),
												motionClient::SimpleActiveCallback(), //Statemachine::mto_active(),
												motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::mto2_feedback,this,_1));
												);
		//and set motion state to RUNNING
		motion_state_=RUNNING;
	}
	else if((motion_state_==RUNNING) && (grip_state_==FINISHED))
	{
		//request state from motion planning
		actionlib::SimpleClientGoalState state=motion_planning_action_client_.getState();
		//ROS_INFO("state: %s",state.toString().c_str());
		switch(state.state_)
		{
		case actionlib::SimpleClientGoalState::SUCCEEDED:
			motion_planning_action_client_.sendGoal(goal_queue[active_goal_],
													boost::bind(&Statemachine::motion_done,this,_1,_2),
													motionClient::SimpleActiveCallback(), //Statemachine::mto_active(),
													motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::mto2_feedback,this,_1));
													);
			break;
		case actionlib::SimpleClientGoalState::ACTIVE:
		case actionlib::SimpleClientGoalState::PENDING:
		case actionlib::SimpleClientGoalState::RECALLED:
			break;
		case actionlib::SimpleClientGoalState::REJECTED:
		case actionlib::SimpleClientGoalState::LOST:
			active_goal_--;
			break;
		case actionlib::SimpleClientGoalState::PREEMPTED:
		case actionlib::SimpleClientGoalState::ABORTED:
		default:
			break;
		}
	}
	else if((motion_state_==FINISHED) && (grip_state_==FINISHED))
	{
		lsc_.detach();
		grip_state_=OPEN;
		motion_state_=OPEN;


		//==============================================
		//state:
		state_.sub.one = fsm::SOLVE_TASK;
		state_.sub.two = fsm::GRIP;
		//==============================================
	}
	return 0;
}

int Statemachine::grip_object()
{
	if(grip_state_==OPEN)
	{
		msg_info("Enter state");

		gripper_control_srv_.request.gripping_force = 40;
		if(cur_obj_.name.compare("blue_handle"))
			gripper_control_srv_.request.object_width = 0.05;
		else
			gripper_control_srv_.request.object_width = 0.01;

		gripper_control_srv_.request.gripping_mode = FF_FORCE;

		lsc_ = boost::thread(&Statemachine::grip_cb,this);
		grip_state_=RUNNING;
	}
	else if(grip_state_==FINISHED)
	{
		lsc_.detach();
		grip_state_=OPEN;

		//==============================================
		//state:
		state_.sub.one = fsm::SOLVE_TASK;
		state_.sub.two = fsm::MOVE_TO_TARGET_ZONE;
		//==============================================
	}
	return 0;
}

int Statemachine::move_to_target_zone()
{
	if(motion_state_==OPEN)
	{
		msg_info("Enter state");

		active_goal_=0;
		nr_goals_=3;
		goal_queue.resize(nr_goals_);

		goal_queue[0].goal_pose = ein_->get_grasping_pose();
		goal_queue[0].goal_pose.position.z+=0.2;
		goal_queue[0].planning_algorithm = STANDARD_IK_7DOF;

//		goal_queue[1]=goal_queue[0];
//		goal_queue[1].goal_pose.position.z+=0.2;
//		goal_queue[1].planning_algorithm = STANDARD_IK_7DOF;
		goal_queue[1].planning_algorithm = HOMING_7DOF;

		cur_zone_ = ein_->get_target_zone();
		goal_queue[2].goal_pose.position = cur_zone_.position;

		goal_queue[2].goal_pose.position.z=goal_queue[0].goal_pose.position.z-0.2+0.005+0.05;
		goal_queue[2].goal_pose.orientation.x=1;
		goal_queue[2].goal_pose.orientation.y=0;
		goal_queue[2].goal_pose.orientation.z=0;
		goal_queue[2].goal_pose.orientation.w=0;
		goal_queue[2].planning_algorithm = STANDARD_IK_7DOF;

		motion_planning_action_client_.sendGoal(goal_queue[0],
												boost::bind(&Statemachine::motion_done,this,_1,_2),
												motionClient::SimpleActiveCallback(), //Statemachine::mto_active(),
												motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::mto2_feedback,this,_1));
												);
		motion_state_=RUNNING;
	}
	else if(motion_state_==RUNNING)
	{
		actionlib::SimpleClientGoalState state=motion_planning_action_client_.getState();
		//ROS_INFO("state: %s",state.toString().c_str());
		switch(state.state_)
		{
		case actionlib::SimpleClientGoalState::SUCCEEDED:
			motion_planning_action_client_.sendGoal(goal_queue[active_goal_],
													boost::bind(&Statemachine::motion_done,this,_1,_2),
													motionClient::SimpleActiveCallback(), //Statemachine::mto_active(),
													motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::mto2_feedback,this,_1));
													);
			break;
		case actionlib::SimpleClientGoalState::ACTIVE:
		case actionlib::SimpleClientGoalState::PENDING:
		case actionlib::SimpleClientGoalState::RECALLED:
		case actionlib::SimpleClientGoalState::REJECTED:
		case actionlib::SimpleClientGoalState::LOST:
		case actionlib::SimpleClientGoalState::PREEMPTED:
		case actionlib::SimpleClientGoalState::ABORTED:
		default:
			break;
		}
	}
	else if((motion_state_==FINISHED) && (grip_state_==OPEN))
	{
		gripper_control_srv_.request.gripping_mode = RELEASE;

		lsc_ = boost::thread(&Statemachine::grip_cb,this);
		grip_state_=RUNNING;

	}
	else if((motion_state_==FINISHED) && (grip_state_==FINISHED))
	{
		lsc_.detach();

		ein_->set_object_finished();

		grip_state_=OPEN;
		motion_state_=OPEN;

		//==============================================
		//state:

		state_.sub.one = fsm::SOLVE_TASK;
		state_.sub.two = fsm::HOMING;
		//==============================================
	}
	return 0;
}

int Statemachine::homing()
{
	if(motion_state_==OPEN)
	{
		msg_info("Enter state");

		active_goal_=0;
		nr_goals_=2;
		goal_queue.resize(nr_goals_);

		goal_queue[0].goal_pose.position = cur_zone_.position;
		goal_queue[0].goal_pose.position.z=0.4;
		goal_queue[0].goal_pose.orientation.x=1;
		goal_queue[0].goal_pose.orientation.y=0;
		goal_queue[0].goal_pose.orientation.z=0;
		goal_queue[0].goal_pose.orientation.w=0;
		goal_queue[0].planning_algorithm = STANDARD_IK_7DOF;

		goal_queue[1].planning_algorithm = HOMING_7DOF;
		motion_planning_action_client_.sendGoal(goal_queue[0],
												boost::bind(&Statemachine::motion_done,this,_1,_2),
												motionClient::SimpleActiveCallback(), //Statemachine::mto_active(),
												motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::mto2_feedback,this,_1));
												);

		motion_state_=RUNNING;
	}
	else if(motion_state_==RUNNING)
	{
		actionlib::SimpleClientGoalState state=motion_planning_action_client_.getState();
		//ROS_INFO("state: %s",state.toString().c_str());
		switch(state.state_)
		{
		case actionlib::SimpleClientGoalState::SUCCEEDED:
			motion_planning_action_client_.sendGoal(goal_queue[active_goal_],
													boost::bind(&Statemachine::motion_done,this,_1,_2),
													motionClient::SimpleActiveCallback(), //Statemachine::mto_active(),
													motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::mto2_feedback,this,_1));
													);
			break;
		case actionlib::SimpleClientGoalState::ACTIVE:
		case actionlib::SimpleClientGoalState::PENDING:
		case actionlib::SimpleClientGoalState::RECALLED:
		case actionlib::SimpleClientGoalState::REJECTED:
		case actionlib::SimpleClientGoalState::LOST:
		case actionlib::SimpleClientGoalState::PREEMPTED:
		case actionlib::SimpleClientGoalState::ABORTED:
		default:
			break;
		}
	}
	else if(motion_state_==FINISHED)
	{
		motion_state_=OPEN;
		//ros::Duration(1).sleep();

		//==============================================
		//state:
		if(ein_->all_finished()) //stop simulation, when all objects finished
		{
			state_.sub.one = fsm::STOP_SIM;
			state_.sub.two = 0;
			task_active_=false;
		}
		else
		{
			//otherwise start again with next object
			state_.sub.one = fsm::SOLVE_TASK;
			state_.sub.two = fsm::LOCATE_OBJECT;
		}
		//==============================================
	}
	return 0;
}

void Statemachine::vision_done(const actionlib::SimpleClientGoalState& state,
		 	 	 	 	 	   const am_msgs::VisionResultConstPtr& result)
{
	ROS_INFO("Vision action finished: %s",state.toString().c_str());

	switch(state.state_)
	{
	case actionlib::SimpleClientGoalState::SUCCEEDED:
		cur_obj_.abs_pose=result->abs_object_pose;
		vision_state_=FINISHED;
		break;
	case actionlib::SimpleClientGoalState::ACTIVE:
		break;
	case actionlib::SimpleClientGoalState::PENDING:
	case actionlib::SimpleClientGoalState::RECALLED:
	case actionlib::SimpleClientGoalState::REJECTED:
	case actionlib::SimpleClientGoalState::LOST:
	case actionlib::SimpleClientGoalState::PREEMPTED:
	case actionlib::SimpleClientGoalState::ABORTED:
	default:
		break;
	}

}
void Statemachine::vision_feedback(const am_msgs::VisionFeedbackConstPtr feedback)
{
	ROS_INFO("got feedback");
}
void Statemachine::motion_feedback(const am_msgs::goalPoseFeedbackConstPtr feedback)
{
	ROS_INFO("got feedback");
}
void Statemachine::motion_done(const actionlib::SimpleClientGoalState& state,
							 const am_msgs::goalPoseResultConstPtr& result)
{
	//state = motion_planning_action_client_.getState();
	ROS_INFO("Motion action finished: %s",state.toString().c_str());

	//increase active_goal counter
	active_goal_++;

	//resolve if there are remaining goals
	if(active_goal_==nr_goals_)
	{
		msg_info("all goals completed, motion_state is set to finished");
		motion_state_=FINISHED;
	}
	else
	{
		ROS_INFO("there are remaining goals, motion_state is set to running");
		motion_state_=RUNNING;
	}
}
