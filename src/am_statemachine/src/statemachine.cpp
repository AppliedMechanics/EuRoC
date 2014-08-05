#include <statemachine.hpp>

Statemachine* Statemachine::instance_ = 0x0;

Statemachine::Statemachine():
	    scenes_(1),
		task_active_(false),
		sim_running_(false),
		nr_scenes_(0),
		active_scene_(-1),
		motion_planning_action_client_("goalPoseAction", true),
		vision_action_client_("VisionAction", true),
		request_task_(OPEN),
		mto1_(OPEN),
		mto2_(OPEN),
		mttz_(OPEN)
{
	ein_=new EurocInput();
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
	if(sim_running_)
	{
		stop_sim();
	}
}

int Statemachine::init_sm()
{

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

void Statemachine::execute()
{
#if 0
	ros::CallbackQueue my_queue_;
	ros::AsyncSpinner spinner(2); // Use 4 threads

	ROS_INFO("before spinner::start");
	spinner.start();
	ROS_INFO("before waitForShutdown");
	//ros::waitForShutdown();

	ROS_INFO("Entering while-loop");
	while(ros::ok() && !(state_.sub.one==FINISHED))
	{
		 my_queue_.callAvailable(ros::WallDuration(0.01));
		 tick();

		 counter++;
	}
#else
	//update rate
	ros::Rate loop_rate(10);

	double start=(double)ros::Time::now().toSec();
	double t_act=0;
	while(ros::ok() && !(state_.sub.one == fsm::FINISHED))
	{

	  if(-1==tick())
	  {
		  msg_error("Error in Statemachine::tick()");
	  }

	  //"tick"
	  ros::spinOnce();

	  //sleep for the remaining time
	  loop_rate.sleep();

	  counter++;
	  t_act=(double)ros::Time::now().toSec()-start;

//	  std::cout<<"time: "<<t_act<<" s, counter: "<<counter<<std::endl;
	  printf("time= %f s \r",t_act);
	  fflush(stdout);
	}
#endif
}

int Statemachine::tick()
{

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
	//ROS_INFO("In request_task_cb");
	request_task_=RUNNING;

	if(list_scenes_client_.exists())
		list_scenes_client_.call(list_scenes_srv_);

	request_task_=FINISHED;
	//ROS_INFO("Exiting request_task_cb");
}

int Statemachine::request_task()
{
	if(request_task_==OPEN)
	{
		lsc_ = boost::thread(&Statemachine::request_task_cb,this);
	}
	else if(request_task_==FINISHED)//lsc_.timed_join(boost::posix_time::seconds(0.0)))
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
		lsc_.detach();


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
	start_sim_=RUNNING;

	//if(start_simulator_client_.exists())
		start_simulator_client_.call(start_simulator_srv_);

	start_sim_=FINISHED;
}

int Statemachine::start_sim()
{
	if(start_sim_==OPEN)
	{
		ROS_INFO("Statemachine: Starting Server");

		start_simulator_srv_.request.user_id = "am-robotics";
		start_simulator_srv_.request.scene_name = scenes_[active_scene_].name;

		lsc_ = boost::thread(&Statemachine::start_sim_cb,this);
	}
	else if(start_sim_==FINISHED)
	{
		// Check the response for errors
		std::string &error_message = start_simulator_srv_.response.error_message;
		if(!error_message.empty()){
			ROS_ERROR("Statemachine: Starting the simulator failed: %s", error_message.c_str());
			return -1;
		}

		//destroy thread
		lsc_.detach();
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


	if(-1==ein_->parse_yaml_file(task_yaml_description))
	{
		ROS_ERROR("Statemachine: Error in parse_yaml_file()");
	}

	ROS_INFO("Statemachine: Parsing YAML-file finished");

	//==============================================
	//state:
	state_.sub.one = fsm::SOLVE_TASK;
	state_.sub.two = fsm::LOCATE_OBJECT;
	//==============================================

	return 0;
}

int Statemachine::solve_task()
{
	//blub

	//==============================================
	//state:
	state_.sub.one = fsm::STOP_SIM;
	//==============================================
	return 0;
}

int Statemachine::stop_sim()
{
	ROS_INFO("Statemachine: Saving log");
	save_log_client_.call(save_log_srv_);

	ROS_INFO("Statemachine: Stopping server");

	// The stop simulator callback ends the current simulation
	stop_simulator_client_.call(stop_simulator_srv_);

	request_task_=OPEN;
	mto1_=OPEN;
	mto2_=OPEN;
	mttz_=OPEN;

	//==============================================
	//state:
	state_.sub.one = fsm::FINISHED;
	//==============================================

	return 0;
}

int Statemachine::locate_object()
{
	//hack: dont use blue handle!!
	static bool first=true;
	if(first)
	{
		ein_->get_object();
		ein_->set_object_finished();
		first=false;
	}
	cur_obj_ = ein_->get_object();
	ein_->set_object_finished();
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

	vision_action_client_.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = vision_action_client_.waitForResult(ros::Duration(300.0));
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = vision_action_client_.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());

		boost::shared_ptr<const am_msgs::VisionResult> result=vision_action_client_.getResult();
		cur_obj_.abs_pose=result->abs_object_pose;
	}
	else
		ROS_INFO("Action did not finish before the time out.");

	//actionlib::SimpleActionClient<am_msgs::VisionAction_<std::allocator<void> > >::ResultConstPtr {aka boost::shared_ptr<const am_msgs::VisionResult_<std::allocator<void> > >}’
	//to non-scalar type ‘am_msgs::VisionActionResult

	//==============================================
	//state:
	state_.sub.one = fsm::SOLVE_TASK;
	state_.sub.two = fsm::GET_GRASPING_POSE;
	//==============================================

	return 0;
}

int Statemachine::get_grasping_pose()
{
	get_grasp_pose_srv_.request.object=cur_obj_;

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

int Statemachine::move_to_object()
{
#if 0
	gripper_control_srv_.request.gripping_mode = RELEASE;

	if(gripper_control_client_.call(gripper_control_srv_))
	{
		ROS_INFO("Gripper released");
	}
	else
		ROS_ERROR("Failed to call gripper control client");


	am_msgs::goalPoseGoal goal;
	goal.goal_pose = ein_->get_grasping_pose();
	goal.goal_pose.position.z+=0.1;
	goal.planning_algorithm = 0;
	motion_planning_action_client_.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = motion_planning_action_client_.waitForResult(ros::Duration(300.0));
	if (finished_before_timeout)
	{
	    actionlib::SimpleClientGoalState state = motion_planning_action_client_.getState();
	    ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	    ROS_INFO("Action did not finish before the time out.");

	goal.goal_pose.position.z-=0.1;
		motion_planning_action_client_.sendGoal(goal);

		//wait for the action to return
		finished_before_timeout = motion_planning_action_client_.waitForResult(ros::Duration(300.0));
		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = motion_planning_action_client_.getState();
			ROS_INFO("Action finished: %s",state.toString().c_str());
		}
		else
			ROS_INFO("Action did not finish before the time out.");


		ros::Duration(1).sleep();

		//==============================================
		//state:
		state_.sub.one = fsm::SOLVE_TASK;
		state_.sub.two = fsm::GRIP;
		//==============================================
#else
	if(mto1_==OPEN)
	{
		gripper_control_srv_.request.gripping_mode = RELEASE;

		if(gripper_control_client_.call(gripper_control_srv_))
		{
			ROS_INFO("Gripper released");
		}
		else
			ROS_ERROR("Failed to call gripper control client");


		am_msgs::goalPoseGoal goal;
		goal.goal_pose = ein_->get_grasping_pose();
		goal.goal_pose.position.z+=0.1;
		goal.planning_algorithm = 0;
		motion_planning_action_client_.sendGoal(goal,
												boost::bind(&Statemachine::mto1_done,this,_1,_2), //Client::SimpleDoneCallback(), //
												Client::SimpleActiveCallback(), //Statemachine::mto_active(),
												Client::SimpleFeedbackCallback());

		//setFeedbackCallback(boost::bind(&MoveToFruitState::feedbackCb, this, _1));

		mto1_=RUNNING;
//	}
//	else if((mto1_==FINISHED)&&(mto2_==OPEN))
//	{
//		am_msgs::goalPoseGoal goal;
		goal.goal_pose = ein_->get_grasping_pose();
		motion_planning_action_client_.sendGoal(goal,
												boost::bind(&Statemachine::mto2_done,this,_1,_2),
												Client::SimpleActiveCallback(), //Statemachine::mto_active(),
												Client::SimpleFeedbackCallback());//boost::bind(&Statemachine::mto2_feedback,this,_1));

		mto2_=RUNNING;
	}
	else if(mto2_==FINISHED)
	{
		//actionlib::SimpleClientGoalState state = motion_planning_action_client_.getState();
		//ROS_INFO("Action finished: %s",state.toString().c_str());

//		am_msgs::goalPoseGoal goal;
//		goal.goal_pose = ein_->get_grasping_pose();
//		motion_planning_action_client_.sendGoal(goal,
//												boost::bind(&Statemachine::mto_done,this,_1,_2),
//												Client::SimpleActiveCallback(), //Statemachine::mto_active(),
//												Client::SimpleFeedbackCallback());
//
//		mto_=RUNNING;


		//ros::Duration(1).sleep();

		//==============================================
		//state:
		state_.sub.one = fsm::SOLVE_TASK;
		state_.sub.two = fsm::GRIP;
		//==============================================
	}
#endif
	return 0;
}

int Statemachine::grip_object()
{
	gripper_control_srv_.request.gripping_force = 30;
	gripper_control_srv_.request.object_width = 0.05;
	gripper_control_srv_.request.gripping_mode = FF_FORCE;

	if(gripper_control_client_.call(gripper_control_srv_))
	{
		ROS_INFO("Gripper closed");
	}
	else
		ROS_ERROR("Failed to call gripper control client");


	//==============================================
	//state:
	state_.sub.one = fsm::SOLVE_TASK;
	state_.sub.two = fsm::MOVE_TO_TARGET_ZONE;
	//==============================================
	return 0;
}

int Statemachine::move_to_target_zone()
{
#if 0
	am_msgs::TargetZone tmp_zone= ein_->get_target_zone();

	am_msgs::goalPoseGoal goal;
	goal.goal_pose.position = tmp_zone.position;
	goal.goal_pose.position.z+=0.4;
	goal.goal_pose.orientation.x=1;
	goal.goal_pose.orientation.y=0;
	goal.goal_pose.orientation.z=0;
	goal.goal_pose.orientation.w=0;
	motion_planning_action_client_.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = motion_planning_action_client_.waitForResult(ros::Duration(300.0));
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = motion_planning_action_client_.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
		ROS_INFO("Action did not finish before the time out.");

	gripper_control_srv_.request.gripping_mode = RELEASE;

	if(gripper_control_client_.call(gripper_control_srv_))
	{
		ROS_INFO("Gripper released");
	}
	else
		ROS_ERROR("Failed to call gripper control client");

	ein_->set_object_finished();

	//==============================================
	//state:
	if(1)//ein_->all_finished())
	{
		state_.sub.one = fsm::STOP_SIM;
		state_.sub.two = 0;
		task_active_=false;
	}
	else
	{
		state_.sub.one = fsm::SOLVE_TASK;
		state_.sub.two = fsm::LOCATE_OBJECT;
	}
	//==============================================
#else
	if(mttz_==OPEN)
	{
		am_msgs::TargetZone tmp_zone= ein_->get_target_zone();

		am_msgs::goalPoseGoal goal;
		goal.goal_pose.position = tmp_zone.position;
		goal.goal_pose.position.z+=0.4;
		goal.goal_pose.orientation.x=1;
		goal.goal_pose.orientation.y=0;
		goal.goal_pose.orientation.z=0;
		goal.goal_pose.orientation.w=0;

		motion_planning_action_client_.sendGoal(goal,
												boost::bind(&Statemachine::mttz_done,this,_1,_2),
												Client::SimpleActiveCallback(), //Statemachine::mto_active(),
												Client::SimpleFeedbackCallback());//boost::bind(&Statemachine::mto_feedback,this,_1,_2));
		mttz_=RUNNING;
	}
	else if(mttz_==FINISHED)
	{
		gripper_control_srv_.request.gripping_mode = RELEASE;

		if(gripper_control_client_.call(gripper_control_srv_))
		{
			ROS_INFO("Gripper released");
		}
		else
			ROS_ERROR("Failed to call gripper control client");

		ein_->set_object_finished();

		//==============================================
		//state:
		if(1)//ein_->all_finished())
		{
			state_.sub.one = fsm::STOP_SIM;
			state_.sub.two = 0;
			task_active_=false;
		}
		else
		{
			state_.sub.one = fsm::SOLVE_TASK;
			state_.sub.two = fsm::LOCATE_OBJECT;
		}
		//==============================================
	}
#endif
	return 0;
}

void Statemachine::mto_feedback(const am_msgs::goalPoseActionFeedbackConstPtr& feedback)
{
	ROS_INFO("got feedback");
}
void Statemachine::mto1_done(const actionlib::SimpleClientGoalState& state,
							 const am_msgs::goalPoseResultConstPtr& result)
{
	//state = motion_planning_action_client_.getState();
	ROS_INFO("Action finished: %s",state.toString().c_str());
	mto1_=FINISHED;
}
void Statemachine::mto2_done(const actionlib::SimpleClientGoalState& state,
							 const am_msgs::goalPoseResultConstPtr& result)
{
	//state = motion_planning_action_client_.getState();
	ROS_INFO("Action finished: %s",state.toString().c_str());
	mto2_=FINISHED;
}
void Statemachine::mto_active()
{
	ROS_INFO("Goal now active!");
}
void Statemachine::mttz_done(const actionlib::SimpleClientGoalState& state,
							const am_msgs::goalPoseResultConstPtr& result)
{
	//state = motion_planning_action_client_.getState();
	ROS_INFO("Action finished: %s",state.toString().c_str());
	mttz_=FINISHED;
}
