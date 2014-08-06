#include <statemachine.hpp>
#include <cmath>

Statemachine* Statemachine::instance_ = 0x0;

Statemachine::Statemachine():
	    scenes_(1),
		task_active_(false),
		sim_running_(false),
		nr_scenes_(0),
		active_scene_(-1),
		motion_planning_action_client_("goalPoseAction", true),
		vision_action_client_("VisionAction", true),
		goal1_sent_(false),
		goal2_sent_(false),
		goal3_sent_(false),
		request_task_state_(OPEN),
		start_sim_state_(OPEN),
		stop_sim_state_(OPEN),
		grip_state_(OPEN),
		vision_state_(OPEN),
		motion_state_(OPEN)
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
	//update rate
	ros::Rate loop_rate(FREQ);

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

	request_task_state_=FINISHED;
	ROS_INFO("Exiting request_task_cb");
}

int Statemachine::request_task()
{
	if(request_task_state_==OPEN)
	{
		lsc_ = boost::thread(&Statemachine::request_task_cb,this);
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

		lsc_ = boost::thread(&Statemachine::start_sim_cb,this);
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

void Statemachine::stop_sim_cb()
{
	stop_sim_state_=RUNNING;
	ROS_INFO("Statemachine: Saving log");
	if(save_log_client_.exists())
		save_log_client_.call(save_log_srv_);

	ROS_INFO("Statemachine: Stopping server");

	// The stop simulator callback ends the current simulation
	if(stop_simulator_client_.exists())
		stop_simulator_client_.call(stop_simulator_srv_);

	stop_sim_state_=FINISHED;
}
int Statemachine::stop_sim()
{
	if(stop_sim_state_==OPEN)
	{
		lsc_ = boost::thread(&Statemachine::stop_sim_cb,this);
	}
	else if(stop_sim_state_==FINISHED)
	{
		lsc_.detach();

		//==============================================
		//state:
		state_.sub.one = fsm::FINISHED;
		//==============================================
	}

	//reset states:
	request_task_state_=OPEN;
	start_sim_state_=OPEN;
	stop_sim_state_=OPEN;

	return 0;
}

int Statemachine::locate_object()
{
	if(vision_state_==OPEN)
	{
		msg_info("Enter state");

		//hack: dont use blue handle!!
		static bool first=true;
		if(first)
		{
			ein_->get_object();
			ein_->set_object_finished(); // jump to green cylinder
			first=false;
		}
		//ein_->get_object();
		//ein_->set_object_finished(); // jump to red cube
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

		vision_action_client_.sendGoal(goal,
										boost::bind(&Statemachine::vision_done,this,_1,_2),
										visionClient::SimpleActiveCallback(),
										visionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::vision_feedback,this,_1));
										);
		vision_state_=RUNNING;
	}
	else if(vision_state_==FINISHED)
	{
//		boost::shared_ptr<const am_msgs::VisionResult> result=vision_action_client_.getResult();
//		cur_obj_.abs_pose=result->abs_object_pose;

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
		am_msgs::goalPoseGoal goal;
		goal.goal_pose = ein_->get_grasping_pose();
		goal.goal_pose.position.z+=0.2;
		goal.planning_algorithm = STANDARD_IK_7DOF;
		if(!goal1_sent_)
		{
			motion_planning_action_client_.sendGoal(goal,
													motionClient::SimpleDoneCallback(),
													motionClient::SimpleActiveCallback(),
													boost::bind(&Statemachine::motion_feedback,this,_1) // Client::SimpleFeedbackCallback());
													);
			goal1_sent_=true;
		}
		else if(!goal2_sent_)
		{
			goal.goal_pose.position.z-=0.2;
			motion_planning_action_client_.sendGoal(goal,
													boost::bind(&Statemachine::motion_done,this,_1,_2),
													motionClient::SimpleActiveCallback(), //Statemachine::mto_active(),
													motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::mto2_feedback,this,_1));
													);

			goal2_sent_=true;
			motion_state_=RUNNING;
		}
	}
	else if((motion_state_==FINISHED) && (grip_state_==FINISHED))
	{
		lsc_.detach();
		grip_state_=OPEN;
		motion_state_=OPEN;

		goal1_sent_=false;
		goal2_sent_=false;

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

		gripper_control_srv_.request.gripping_force = 30;
		gripper_control_srv_.request.object_width = 0.05;
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

		am_msgs::goalPoseGoal goal;

		if(!goal1_sent_)
		{

			goal.goal_pose = ein_->get_grasping_pose();
			goal.goal_pose.position.z+=0.2;
			goal.planning_algorithm = STANDARD_IK_7DOF;
			motion_planning_action_client_.sendGoal(goal,
													motionClient::SimpleDoneCallback(),
													motionClient::SimpleActiveCallback(),
													motionClient::SimpleFeedbackCallback()
													);

			goal1_sent_=true;
		}
		else if(!goal2_sent_)
		{
			ros::spinOnce();
			ros::Duration(0.15).sleep();
			goal.planning_algorithm = HOMING_7DOF;
			//goal.goal_pose.position.z+=0.05;
			motion_planning_action_client_.sendGoal(goal,
													motionClient::SimpleDoneCallback(),
													motionClient::SimpleActiveCallback(), //Statemachine::mto_active(),
													motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::mto2_feedback,this,_1));
													);
			goal2_sent_=true;
		}
		else if(!goal3_sent_)
		{
			ros::spinOnce();
			ros::Duration(0.15).sleep();
			am_msgs::TargetZone tmp_zone= ein_->get_target_zone();
			goal.goal_pose.position = tmp_zone.position;
			goal.goal_pose.position.z=0.3;
			goal.goal_pose.orientation.x=1;
			goal.goal_pose.orientation.y=0;
			goal.goal_pose.orientation.z=0;
			goal.goal_pose.orientation.w=0;
			goal.planning_algorithm = STANDARD_IK_7DOF;

			motion_planning_action_client_.sendGoal(goal,
													boost::bind(&Statemachine::motion_done,this,_1,_2),
													motionClient::SimpleActiveCallback(), //Statemachine::mto_active(),
													motionClient::SimpleFeedbackCallback());//boost::bind(&Statemachine::mto_feedback,this,_1,_2));
			motion_state_=RUNNING;
			goal3_sent_=true;
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

		goal1_sent_=false;
		goal2_sent_=false;
		goal3_sent_=false;

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
			state_.sub.two = fsm::HOMING;
		}
		//==============================================
	}
	return 0;
}

int Statemachine::homing()
{
	if(motion_state_==OPEN)
	{
		msg_info("Enter state");

		//send goals to motion-planning
		am_msgs::goalPoseGoal goal;

		goal.planning_algorithm = HOMING_7DOF;
		motion_planning_action_client_.sendGoal(goal,
												boost::bind(&Statemachine::motion_done,this,_1,_2),
												motionClient::SimpleActiveCallback(), //Statemachine::mto_active(),
												motionClient::SimpleFeedbackCallback()//boost::bind(&Statemachine::mto2_feedback,this,_1));
												);

		motion_state_=RUNNING;
	}
	else if(motion_state_==FINISHED)
	{
		motion_state_=OPEN;
		//ros::Duration(1).sleep();

		//==============================================
		//state:
		state_.sub.one = fsm::SOLVE_TASK;
		state_.sub.two = fsm::LOCATE_OBJECT;
		//==============================================
	}
	return 0;
}

void Statemachine::vision_done(const actionlib::SimpleClientGoalState& state,
		 	 	 	 	 	   const am_msgs::VisionResultConstPtr& result)
{
	ROS_INFO("Vision action finished: %s",state.toString().c_str());

	cur_obj_.abs_pose=result->abs_object_pose;

	vision_state_=FINISHED;
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
	motion_state_=FINISHED;
}
