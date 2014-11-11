/*
 * MotionPlanning.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: euroc_admin
 */

#include <MotionPlanning.h>

MotionPlanning::MotionPlanning():
goalPose_server_(nh_, "goalPoseAction", boost::bind(&MotionPlanning::executeGoalPose_CB, this, _1),false),
goalPose_action_name_("goalPoseAction"),
time_at_path_points_(1),
active_task_nr_(1),
octree_file(0.01),
allowed_planning_time_(30),
collision_object_scaler_(1.0),
obj_data_loaded_(false)
//randomObjectAttached(false)
{
	euroc_c2_interface_ = "/euroc_interface_node";
	telemetry_ = euroc_c2_interface_ + "/telemetry";
	move_along_joint_path_ = euroc_c2_interface_ + "/move_along_joint_path";
	timing_along_joint_path_ = euroc_c2_interface_ + "/get_timing_along_joint_path";
	search_ik_solution_ = euroc_c2_interface_ + "/search_ik_solution";
	get_dk_solution_ = euroc_c2_interface_ + "/get_forward_kinematics";

	move_along_joint_path_client_   = nh_.serviceClient<euroc_c2_msgs::MoveAlongJointPath>(move_along_joint_path_);
	timing_along_joint_path_client_ = nh_.serviceClient<euroc_c2_msgs::GetTimingAlongJointPath>(timing_along_joint_path_);
	search_ik_solution_client_      = nh_.serviceClient<euroc_c2_msgs::SearchIkSolution>(search_ik_solution_);
	get_dk_solution_client_         = nh_.serviceClient<euroc_c2_msgs::GetForwardKinematics>(get_dk_solution_);
	state_observer_client_          = nh_.serviceClient<am_msgs::CallSetStopConditions>("CallSetStopConditions_srv");

	joint_limits_.resize(7);

	goalPose_server_.start();
	msg_info("goalPose action server started.");


	check_poses_service_ = nh_.advertiseService("CheckPoses_srv", &MotionPlanning::return_poses_valid,this);

	obj_state_sub_ = nh_.subscribe("obj_state", 1000, &MotionPlanning::object_manager_get_object_state_cb, this);

	feedback_frequency_ = 2;

	mtt_=OPEN;
	called = false;



	// vector of ompl planners
	ompl_planners.push_back("SBLkConfigDefault");                         //  0
	ompl_planners.push_back("ESTkConfigDefault");               //  1
	ompl_planners.push_back("LBKPIECEkConfigDefault_2DOF");     //  2
	ompl_planners.push_back("LBKPIECEkConfigDefault_7DOF");     //  3
	ompl_planners.push_back("LBKPIECEkConfigDefault_9DOF");     //  4
	ompl_planners.push_back("BKPIECEkConfigDefault");           //  5
	ompl_planners.push_back("KPIECEkConfigDefault");            //  6
	ompl_planners.push_back("RRTkConfigDefault");               //  7
	ompl_planners.push_back("RRTConnectkConfigDefault");        //  8
	ompl_planners.push_back("RRTstarkConfigDefault");           //  9
	ompl_planners.push_back("TRRTkConfigDefault");              // 10
	ompl_planners.push_back("PRMkConfigDefault");               // 11
	ompl_planners.push_back("PRMstarkConfigDefault");           // 12

	// table axes
	group_2DOF = new move_group_interface::MoveGroup("LWR_2DOF");
	group_2DOF->setEndEffectorLink("base_link");
#warning Also Goal Tolerance for homing 2DOF (isn't used till now')
	// planning algorithm for arm + table axes
	//    group_2DOF->setPlannerId(ompl_planners[7]); // 2
	group_2DOF->setNumPlanningAttempts(2);

	// arm group
	group_7DOF = new move_group_interface::MoveGroup("LWR_7DOF");
	group_7DOF->setEndEffectorLink("gripper_tcp");
	// planning algorithm for arm group
	//    group_7DOF->setPlannerId(ompl_planners[3]);
	group_7DOF->setNumPlanningAttempts(2);

	// arm + table axes
	group_9DOF = new move_group_interface::MoveGroup("LWR_9DOF");
	group_9DOF->setEndEffectorLink("gripper_tcp");
	// planning algorithm for arm + table axes
	//    group_9DOF->setPlannerId(ompl_planners[4]); // 4
	group_9DOF->setNumPlanningAttempts(3);

	// robot model loader
	robot_model_loader_ = robot_model_loader::RobotModelLoader("robot_description");
	kinematic_model_ = robot_model_loader_.getModel();
	kinematic_state_ = new robot_state::RobotState(kinematic_model_);

	joint_model_group_2DOF_ = kinematic_model_->getJointModelGroup("LWR_2DOF");
	joint_model_group_7DOF_ = kinematic_model_->getJointModelGroup("LWR_7DOF");
	joint_model_group_9DOF_ = kinematic_model_->getJointModelGroup("LWR_9DOF");

	group = group_9DOF;
	joint_model_group_ = joint_model_group_9DOF_;

	// planning scene monitor
	planning_scene_monitor = new planning_scene_monitor::PlanningSceneMonitor("robot_description");

	get_planning_scene_client_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>(planning_scene_monitor->DEFAULT_PLANNING_SCENE_SERVICE);

	// Octomap service client
	octomap_ = "/octomap_binary";
	octomap_client_                       = nh_.serviceClient<octomap_msgs::GetOctomap>(octomap_);
	cleanup_octomap_client_ = nh_.serviceClient<octomap_msgs::BoundingBoxQuery>("/octomap_server/clear_bbx");
	// Publisher
	planning_scene_diff_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

	current_setTarget_algorithm_ = SINGLE_POSE_TARGET;

}

MotionPlanning::~MotionPlanning()
{
	delete planning_scene_monitor;
	delete group_9DOF;
	delete group_7DOF;
	delete group_2DOF;
}

void MotionPlanning::executeGoalPose_CB(const am_msgs::goalPoseGoal::ConstPtr &goal)
{
	ros::param::get("/skip_vision", skip_vision_);
	ros::param::get("/active_task_number_", active_task_nr_);

	static bool first=true;
	if (first){
		// initialize Planning Scene -> add ground + pan tilt to environment
		initializePlanningScene();
		if (!octomap_manager_cleanupOctomap())
			msg_error("Error at cleaning up octomap.");
		first = false;
	}

	goal_pose_goal_ = goal;
	speed_percentage_ = goal_pose_goal_->speed_percentage;
	inter_steps_ = goal_pose_goal_->inter_steps;
	planning_frame_ = goal_pose_goal_->planning_frame;
	ROS_WARN("Chosen Planning Frame :%s",planning_frame_.c_str());

	// ROS_INFO_STREAM("Vision pose 1 "<<goal_pose_goal_->goal_pose.position);
	//	ROS_INFO_STREAM("goal received:");
	//	ROS_INFO_STREAM(goal_pose_goal_->goal_pose.position);

	//--------------------------------------------------------------------------------------
	//! get kinematic limits
	getLimits();

	//--------------------------------------------------------------------------------------
	//! set Planning Frame
	//! If planning frame is given in GP_TCP frame, the LWR_TCP frame is calculated
	//! If planning frame is chosen as LWR TCP, the GP TCP Pose needs to be calculated
	if (!planning_frame_.compare(LWR_TCP) || !planning_frame_.compare(GP_TCP)){
		if (!transformToTCPFrame(planning_frame_)){
			msg_error("Transformation to LWR frame not successful.");

			goalPose_result_.reached_goal = false;
			goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
			goalPose_server_.setPreempted(goalPose_result_,"Got no telemetry.");

			return;
		}
	}
	else if (goal_pose_goal_->planning_algorithm==MOVE_IT_2DOF){
		goal_pose_GPTCP_  = goal_pose_goal_->goal_pose;
		goal_pose_LWRTCP_ = goal_pose_goal_->goal_pose;
	}
	else if (goal_pose_goal_->planning_algorithm!=HOMING_7DOF && goal_pose_goal_->planning_algorithm!=T6_MOVE_IT_9DOF_BELTHOMING && goal_pose_goal_->planning_algorithm!=HOMING_MOVE_IT_7DOF)
		msg_error("Planning frame not properly defined.");

	//--------------------------------------------------------------------------------------
	//! get telemetry
	if (!getTelemetry()){
		msg_error("getTelemetry: An Error happened here.");
		goalPose_result_.reached_goal = false;
		goalPose_result_.error_reason = fsm::SIM_SRV_NA;
		goalPose_server_.setPreempted(goalPose_result_,"Got no telemetry.");

		return;
	}

	// ROS_INFO_STREAM("Vision pose 2 "<<goal_pose_GPTCP_.position);
	//! Decision about T6 or T5

	switch (active_task_nr_)
	{
	case 6:
		if ( !executeGoalPoseT6() )
		{
			msg_error("executeGoalPoseT6() failed");
			return;
		}
		break;
	case 5:
		if ( !executeGoalPoseStd())//executeGoalPoseT5() )
		{
			msg_error("executeGoalPoseT5() failed");
			return;
		}
		break;
	default:
		if ( !executeGoalPoseStd() )
		{
			msg_error("executeGoalPoseStd() failed");
			return;
		}
		break;
	}

	return;
}


bool MotionPlanning::executeGoalPoseStd()
{
	//--------------------------------------------------------------------------------------
	//! Plan
	switch (goal_pose_goal_->planning_algorithm)
	{

	case HOMING_7DOF:
		ROS_INFO("HOMING 7DOF planning mode chosen.");
		if (!euroc_setReset7DOF())
		{
			msg_error("No IK Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No IK Solution found.");
			return false;
		}
		break;

	case HOMING_MOVE_IT_2DOF:
		ROS_INFO("HOMING MOVEIT 2DOF planning mode chosen. - Goal tolerance = 0.5!");
		group = group_2DOF;
		joint_model_group_ = joint_model_group_2DOF_;
		if (!MoveIt_homing())
		{
			msg_error("No Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No Solution found.");
			return false;
		}
		break;

	case HOMING_MOVE_IT_7DOF:
		ROS_INFO("HOMING MOVEIT 7DOF planning mode chosen.");
		group = group_7DOF;
		joint_model_group_ = joint_model_group_7DOF_;
		int homing_counter;
		homing_counter = 0;

		while (!MoveIt_homing() && homing_counter<5)
		{homing_counter++;}
		if (homing_counter==5)
		{
			group = group_9DOF;
			joint_model_group_ = joint_model_group_9DOF_;
			if (!MoveIt_homing())
			{
				msg_error("No Solution found.");
				goalPose_result_.reached_goal = false;
				goalPose_server_.setPreempted(goalPose_result_,"No Solution found.");
				return false;
			}
		}
		break;

	case HOMING_MOVE_IT_9DOF:
		ROS_INFO("HOMING MOVEIT 9DOF planning mode chosen.");
		group = group_9DOF;
		joint_model_group_ = joint_model_group_9DOF_;
		if (!MoveIt_homing())
		{
			msg_error("No Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No Solution found.");
			return false;
		}
		break;

	case RANDOMPOSE_7DOF:
		ROS_INFO("RANDOMPOSE 7DOF chosen.");
		group = group_7DOF;
		joint_model_group_ = joint_model_group_7DOF_;
		max_setTarget_attempts_ = 10;
		if (!MoveIt_randomPose())
		{
			msg_error("No Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No Solution found.");
			return false;
		}
		break;
	case RANDOMPOSE_9DOF:
		ROS_INFO("RANDOMPOSE 9DOF chosen.");
		group = group_9DOF;
		joint_model_group_ = joint_model_group_9DOF_;
		max_setTarget_attempts_ = 10;
		if (!MoveIt_randomPose())
		{
			msg_error("No Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No Solution found.");
			return false;
		}
		break;
	case STANDARD_IK_7DOF:

		ROS_INFO("STANDARD IK 7DOF planning mode chosen.");

		//! Transform goal pose to LWR0 Base frame
		if (!transformToLWRBase())	{msg_warn("Transformation to LWR0 Base failed.");}
		//! Find IK solution
		if (!euroc_getIKSolution7DOF())
		{
			msg_error("No IK Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No IK Solution found.");
			return false;
		}
		break;
		//------------------------------------------------------------------------------------------------
	case (SWING_IN):

				swing_in_motion();
	break;
	case (MOVE_IT_2DOF):
	case (MOVE_IT_7DOF):
	case (MOVE_IT_9DOF):
	ROS_WARN("Planning mode based on MoveIt! chosen.");

	//define groups
	if(goal_pose_goal_->planning_algorithm == MOVE_IT_9DOF)
	{
		ROS_INFO("Choosed 9DOF");
		group = group_9DOF;
		joint_model_group_ = joint_model_group_9DOF_;
		// setting joint state target via the searchIKSolution srv is not considered
		max_setTarget_attempts_ = 6;

	}
	else if(goal_pose_goal_->planning_algorithm == MOVE_IT_7DOF)
	{
		ROS_INFO("7DOF was chosen");
		group = group_7DOF;
		joint_model_group_ = joint_model_group_7DOF_;
		// in case of unsuccessful planning,
		// the planning target is set as a joint state goal via the searchIKSolution srv
		max_setTarget_attempts_ = 7;

	}
	else if(goal_pose_goal_->planning_algorithm == MOVE_IT_2DOF)
	{
		ROS_INFO("2DOF was chosen");
		group = group_2DOF;
		joint_model_group_ = joint_model_group_2DOF_;

		// in case of unsuccessful planning,
		// the planning target is set as a joint state goal via the searchIKSolution srv
		max_setTarget_attempts_ = 6;

	}
	else
	{
		msg_error("Unknown move group name.");
		goalPose_result_.reached_goal = false;
		goalPose_server_.setPreempted(goalPose_result_,"Unknown move group name.");
		break;
	}

	//set target algorithm
	current_setTarget_algorithm_ = SINGLE_POSE_TARGET;
	// get moveit solution
	if (!MoveIt_getSolution())
	{
		msg_error("No MoveIT Solution found.");
		goalPose_result_.reached_goal = false;
		goalPose_server_.setPreempted(goalPose_result_,"No MoveIT Solution found.");
		return false;
	}
	break;

	case (MOVE_IT_JT_9DOF):

    								ROS_WARN("Given JT based on MoveIt! chosen.");
	current_setTarget_algorithm_ = JOINT_VALUE_TARGET_9DOF;
	group = group_9DOF;
	joint_model_group_ = joint_model_group_9DOF_;
	// setting joint state target via the searchIKSolution srv is not considered
	max_setTarget_attempts_ = 3;
	if (!MoveIt_getSolution())
	{
		msg_error("No MoveIT Solution found.");
		goalPose_result_.reached_goal = false;
		goalPose_server_.setPreempted(goalPose_result_,"No MoveIT Solution found.");
		return false;
	}
	break;

	default:
		msg_warn("unkown Mode in MotionPlanning!");
		return false;
	}
	//------------------------------------------------------------------------------------------------

	if(!executeStd())
	{
		msg_error("Execute goal std failed!");
		return false;
	}
	return true;
}

bool MotionPlanning::executeStd()
{
	//! get timing along path
	getTimingAlongJointPath();
	//------------------------------------------------------------------------------------------------

	//! Feedback
	ros::Rate feedback_rate(feedback_frequency_);

	starting_time_ = ros::Time::now().toSec();
	getGoalPose_Feedback();

	goalPose_server_.publishFeedback(goalPose_feedback_);

	if (move_along_joint_path_srv_.request.path.size()<1 || move_along_joint_path_srv_.request.path.size()>999)
	{
		goalPose_result_.reached_goal = false;
		goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
		goalPose_server_.setPreempted(goalPose_result_,"Path length <1 OR >999");
		return false;
	}
	else
	{
		//------------------------------------------------------------------------------------------------
		if (estimated_motion_time_<goal_pose_goal_->allowed_time || goal_pose_goal_->allowed_time<0.5)
		{
			moveToTarget = boost::thread(&MotionPlanning::moveToTargetCB,this);
			mtt_=RUNNING;
			//while (!goalPose_result_.reached_goal)
			while(mtt_==RUNNING)
			{
				getGoalPose_Feedback();
				goalPose_server_.publishFeedback(goalPose_feedback_);

				feedback_rate.sleep();
			}

			moveToTarget.detach();

			if(mtt_==FINISHED)
			{
				goalPose_result_.reached_goal = true;
				goalPose_server_.setSucceeded(goalPose_result_, "Goal configuration has been reached");
			}
			else
			{
				goalPose_result_.reached_goal = false;
				goalPose_server_.setPreempted(goalPose_result_,"Something strange happened.");
			}
		}
		else
		{
			msg_warn("Estimated Motion Time %f > allowed Time %f",estimated_motion_time_,goal_pose_goal_->allowed_time);
			goalPose_result_.reached_goal = false;
			goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
			goalPose_server_.setAborted(goalPose_result_,"Motion takes too long.");
			return false;
		}

	}
	return true;
}


void MotionPlanning::getGoalPose_Feedback()
{
	goalPose_feedback_.estimated_motion_time = estimated_motion_time_;
	goalPose_feedback_.execution_time = ros::Time::now().toSec() - starting_time_;

}



bool MotionPlanning::euroc_getIKSolution7DOF()
{
	try
	{
		if (ros::service::waitForService(move_along_joint_path_,ros::Duration(10.0)) &&
				ros::service::waitForService(search_ik_solution_,ros::Duration(10.0)))
		{
			// Populate a vector with all the lwr joint names
			const unsigned int nr_lwr_joints = 7;
			std::vector<std::string> lwr_joints(nr_lwr_joints);
			std::stringstream name;
			for(unsigned int i = 0; i < nr_lwr_joints; ++i){
				name.str("lwr_joint_");
				name.seekp(0, std::ios_base::end);
				name << (i + 1);
				lwr_joints[i] = name.str();
			}

			// TODO: Read from telemetry data
			move_along_joint_path_srv_.request.joint_names = lwr_joints; // Select all lwr joints
			move_along_joint_path_srv_.request.path.resize(1);           // Our path has only one waypoint

			for (unsigned int ii = 0; ii<move_along_joint_path_srv_.request.path.size();ii++)
				move_along_joint_path_srv_.request.path[ii].q.resize(nr_lwr_joints);

			// Initialize the velocity and acceleration limits of the joints
			move_along_joint_path_srv_.request.joint_limits.resize(nr_lwr_joints);

			for(unsigned int i = 0; i < nr_lwr_joints; i++){
				move_along_joint_path_srv_.request.joint_limits[i] = joint_limits_[i];
			}

			// current_configuration will hold our current joint position data extracted from the measured telemetry
			current_configuration_.q.resize(nr_lwr_joints);


			// Get the current configuration from the telemetry message
			for(unsigned int i = 0; i < nr_lwr_joints; ++i){
				std::vector<std::string> &joint_names = (_telemetry.joint_names);
				unsigned int telemetry_index = std::find(joint_names.begin(), joint_names.end(), lwr_joints[i]) - joint_names.begin();
				current_configuration_.q[i] = _telemetry.measured.position[telemetry_index];
			}

			if(inter_steps_==0)
			{

				// Select the next desired position of the tcp from the target zone poses and fill
				// the search inverse kinematic solution request with the current configuration as
				// start configuration and the desired position
				search_ik_solution_srv_.request.start = current_configuration_;
				search_ik_solution_srv_.request.tcp_frame = goal_pose_LWRTCP_;

				// Call the search inverse kinematic solution service and check for errors
				if(!search_ik_solution_client_.call(search_ik_solution_srv_))
				{
					msg_error("Search IK Solution call failed");

					goalPose_result_.error_reason = fsm::SIM_SRV_NA;
					return false;
				}
				std::string &search_error_message = search_ik_solution_srv_.response.error_message;
				if(!search_error_message.empty()){
					msg_error("Search IK Solution failed: %s", search_error_message.c_str());

					goalPose_result_.error_reason = fsm::NO_IK_SOL;
					return false;
				}

				// Extract the solution configuration from the response and fill it into the path of the move request
				euroc_c2_msgs::Configuration &solution_configuration = search_ik_solution_srv_.response.solution;
				std::vector<euroc_c2_msgs::Configuration> &path = move_along_joint_path_srv_.request.path;
				path[0] = solution_configuration;
			}
			else
			{
				get_dk_solution_srv_.request.configuration = current_configuration_;
				if(!get_dk_solution_client_.call(get_dk_solution_srv_))
				{
					msg_error("Search DK Solution call failed");

					goalPose_result_.error_reason = fsm::SIM_SRV_NA;
					return false;
				}

				// The error_message field of each service response indicates whether an error occured. An empty string indicates success
				std::string &ls_error_message = get_dk_solution_srv_.response.error_message;
				if(!ls_error_message.empty()){
					ROS_ERROR("Get DK failed: %s", ls_error_message.c_str());

					goalPose_result_.error_reason = fsm::NO_DK_SOL;
					return false;
				}
				else
				{
					geometry_msgs::Pose cur_pose = get_dk_solution_srv_.response.ee_frame;

					double delta_x=goal_pose_LWRTCP_.position.x - cur_pose.position.x;
					double delta_y=goal_pose_LWRTCP_.position.y - cur_pose.position.y;
					double delta_z=goal_pose_LWRTCP_.position.z - cur_pose.position.z;

					std::vector<geometry_msgs::Pose> all_poses;
					all_poses.push_back(cur_pose);

					//necessary to get goal orientation!
					cur_pose.orientation=goal_pose_LWRTCP_.orientation;
					for(uint16_t ii=0;ii<inter_steps_;ii++)
					{
						cur_pose.position.x+=(double)(delta_x/inter_steps_);
						cur_pose.position.y+=(double)(delta_y/inter_steps_);
						cur_pose.position.z+=(double)(delta_z/inter_steps_);

						all_poses.push_back(cur_pose);

						ROS_INFO("pose %d: [%4.3f %4.3f %4.3f]",ii,
								cur_pose.position.x,cur_pose.position.y,cur_pose.position.z);
					}

					for(uint16_t ii=1;ii<all_poses.size();ii++)
					{
						search_ik_solution_srv_.request.start = current_configuration_;
						search_ik_solution_srv_.request.tcp_frame = all_poses[ii];

						// Call the search inverse kinematic solution service and check for errors
						if(!search_ik_solution_client_.call(search_ik_solution_srv_))
						{
							msg_error("Search IK Solution call failed");

							goalPose_result_.error_reason = fsm::SIM_SRV_NA;
							return false;
						}
						std::string &search_error_message = search_ik_solution_srv_.response.error_message;
						if(!search_error_message.empty()){
							msg_error("Search IK Solution failed: %s", search_error_message.c_str());

							goalPose_result_.error_reason = fsm::NO_IK_SOL;
							return false;
						}

						std::vector<euroc_c2_msgs::Configuration> &path = move_along_joint_path_srv_.request.path;
						path.push_back(search_ik_solution_srv_.response.solution);
					}
				}
			}

			return true;
		}
		else
		{
			goalPose_result_.error_reason = fsm::SIM_SRV_NA;
			return false;
		}
	}
	catch (...)
	{
		msg_error("MotionPlanning::Error at searchforIK function.");
		goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
		return false;
	}
	return true;
}

bool MotionPlanning::euroc_setReset7DOF()
{
	try
	{
		if (ros::service::waitForService(move_along_joint_path_,ros::Duration(10.0)))
		{
			// Populate a vector with all the lwr joint names
			const unsigned int nr_lwr_joints = 7;
			std::vector<std::string> lwr_joints(nr_lwr_joints);
			std::stringstream name;
			for(unsigned int i = 0; i < nr_lwr_joints; ++i){
				name.str("lwr_joint_");
				name.seekp(0, std::ios_base::end);
				name << (i + 1);
				lwr_joints[i] = name.str();
			}

			// TODO: Read from telemetry data
			move_along_joint_path_srv_.request.joint_names = lwr_joints; // Select all lwr joints
			move_along_joint_path_srv_.request.path.resize(1);           // Our path has only one waypoint
			// Initialize the velocity and acceleration limits of the joints
			move_along_joint_path_srv_.request.joint_limits.resize(nr_lwr_joints);
			for(unsigned int i = 0; i < nr_lwr_joints; i++){
				move_along_joint_path_srv_.request.joint_limits[i] = joint_limits_[i];
			}

			// current_configuration will hold our current joint position data extracted from the measured telemetry
			euroc_c2_msgs::Configuration commanded_configuration;
			commanded_configuration.q.resize(nr_lwr_joints);

			// Get the current configuration from the telemetry message
			for(unsigned int i = 0; i < nr_lwr_joints; ++i){
				commanded_configuration.q[i] = 0.0;
			}

			// Extract the solution configuration from the response and fill it into the path of the move request
			std::vector<euroc_c2_msgs::Configuration> &path = move_along_joint_path_srv_.request.path;
			path[0] = commanded_configuration;

			return true;
		}
		else
		{
			goalPose_result_.error_reason = fsm::SIM_SRV_NA;
			return false;
		}
	}
	catch (...)
	{
		msg_error("MotionPlanning::Error at searchforIK function.");
		goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
		return false;
	}
	return true;
}

bool MotionPlanning::octomap_manager_getOctomap()
{

	if(skip_vision_ && active_task_nr_==4 && ros::service::waitForService(octomap_,ros::Duration(4.0)))
	{
		// manuelles reinladen
		octomap_client_.call(octomap_srv_);

		ROS_INFO("Loading Octomap manually from file.");
		octree_file.readBinary("../EUROC_SVN/trunk/misc/octomap_task41.bt");

		//hier wird der Programmpfad gespeichert
		char pfad[256];
		getcwd( pfad, 256);
		ROS_INFO_STREAM(pfad);

		// Convert binary file to octomap
		octomap_msgs::binaryMapToMsg(octree_file, _octree);
		if (_octree.data.empty())
		{
			ROS_WARN("No manual Octomap received!");
			return false;
		}
		else
		{
			ROS_WARN("Manual Octomap received!");
			return true;
		}
	}
	else if (!skip_vision_)
	{
		try
		{
			if (ros::service::waitForService(octomap_,ros::Duration(3.0)))
			{
				// call service
				octomap_client_.call(octomap_srv_);
				// calling octomap from octomap_service
				if (octomap_srv_.response.map.data.empty())
				{
					msg_warn("No Octomap received!");
					return false;
				}
				else
				{
					msg_info("Octomap received!");
					_octree = octomap_srv_.response.map;
					return true;
				}
			}
			else
			{
				msg_error("WaitForService Octomap failed.");
				return false;
			}

		}
		catch(...)
		{
			msg_error("Octomap service client failed.");
			return false;
		}

	}
	else
		return false;
}

bool MotionPlanning::octomap_manager_cleanupOctomap()
{
	try{
		if (!cleanup_octomap_client_.call(cleanup_octomap_srv_)){
			msg_warn("Octomap cleanup failed");
			return false;
		}
	}catch (...){msg_error("Cleanup Octomap failed."); return false;}

	return true;
}

bool MotionPlanning::MoveIt_homing()
{


	try
	{
		if (ros::service::waitForService(move_along_joint_path_,ros::Duration(10.0)))
		{

			//    Initiliazing MoveGroup
			if(!MoveIt_initializeMoveGroup())
			{
				return false;
			}

			bool setTarget_successful = false;
			setTarget_successful = setPlanningTarget(HOMING);


			if (setTarget_successful)
			{
				bool planning_successful = group->plan(motion_plan_);
				if (planning_successful)
				{
					ROS_INFO("Planning successful!");
					planned_path_.clear();

					// for each configuration of the trajectory except from the start configuration
					for (unsigned configIdx = 0; configIdx < motion_plan_.trajectory_.joint_trajectory.points.size(); ++configIdx)
					{
						// current configuration
						euroc_c2_msgs::Configuration current_config;

						// for each joint at the current configuration
						for (unsigned jointIdx = 0; jointIdx < group->getActiveJoints().size(); ++ jointIdx)
						{
							current_config.q.push_back(motion_plan_.trajectory_.joint_trajectory.points[configIdx].positions[jointIdx]);
						}
						planned_path_.push_back(current_config);
					}

					current_configuration_ = planned_path_[0];

					move_along_joint_path_srv_.request.joint_names = group->getActiveJoints();

					move_along_joint_path_srv_.request.path.resize(planned_path_.size()-1);
					for (unsigned idx = 0; idx < move_along_joint_path_srv_.request.path.size(); ++idx)
					{
						move_along_joint_path_srv_.request.path[idx] = planned_path_[idx+1];
					}

					// set the joint limits (velocities/accelerations) of the move along joint path service request
					setMoveRequestJointLimits();
					// set the TCP limits of the move along joint path service
					setMoveRequestTCPLimits();

					return true;
				}
			}

		}
		else      // if (!ros::service::waitForService(move_along_joint_path_,ros::Duration(10.0)))
		{
			goalPose_result_.error_reason = fsm::SIM_SRV_NA;
			return false;
		}

	} // end try

	catch(...)
	{
		msg_error("MotionPlanning::Error in MoveIt! Planning.");
		goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
		return false;
	}
	return true;



}

bool MotionPlanning::MoveIt_randomPose()
{

	try
	{
		if (ros::service::waitForService(move_along_joint_path_,ros::Duration(10.0)))
		{

			if(!MoveIt_initializeMoveGroup()){return false;}

			unsigned current_setTarget_attempt = 1;
			bool setTarget_successful = false;
			bool planning_successful = false;

			while (!planning_successful)
			{
				setTarget_successful = false;
				setTarget_successful = setPlanningTarget(RANDOM_TARGET);
				if (setTarget_successful)
				{

					planning_successful = group->plan(motion_plan_);

					if (planning_successful)
					{
						ROS_INFO("Planning successful!");
						planned_path_.clear();

						// for each configuration of the trajectory except from the start configuration
						for (unsigned configIdx = 0; configIdx < motion_plan_.trajectory_.joint_trajectory.points.size(); ++configIdx)
						{
							// current configuration
							euroc_c2_msgs::Configuration current_config;

							// for each joint at the current configuration
							for (unsigned jointIdx = 0; jointIdx < group->getActiveJoints().size(); ++ jointIdx)
							{
								current_config.q.push_back(motion_plan_.trajectory_.joint_trajectory.points[configIdx].positions[jointIdx]);
							}
							planned_path_.push_back(current_config);
						}

						current_configuration_ = planned_path_[0];

						move_along_joint_path_srv_.request.joint_names = group->getActiveJoints();

						move_along_joint_path_srv_.request.path.resize(planned_path_.size()-1);
						for (unsigned idx = 0; idx < move_along_joint_path_srv_.request.path.size(); ++idx)
						{
							move_along_joint_path_srv_.request.path[idx] = planned_path_[idx+1];
						}

						// set the joint limits (velocities/accelerations) of the move along joint path service request
						setMoveRequestJointLimits();
						// set the TCP limits of the move along joint path service
						setMoveRequestTCPLimits();

						return true;
					}
				}

				current_setTarget_attempt++;
				if (current_setTarget_attempt > max_setTarget_attempts_)
				{
					msg_error("MoveIt: No Motion Plan found!");
					goalPose_result_.error_reason = fsm::NO_IK_SOL;
					return false;
				}
			}


		}
		else      // if (!ros::service::waitForService(move_along_joint_path_,ros::Duration(10.0)))
		{
			goalPose_result_.error_reason = fsm::SIM_SRV_NA;
			return false;
		}

	} // end try

	catch(...)
	{
		msg_error("MotionPlanning::Error in MoveIt! Planning.");
		goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
		return false;
	}
	return true;

}


bool MotionPlanning::MoveIt_getSolution()
{

	try
	{
		if (ros::service::waitForService(move_along_joint_path_,ros::Duration(10.0)))
		{

			if(!MoveIt_initializeMoveGroup()){return false;}

			unsigned current_setTarget_attempt = 1;
			bool setTarget_successful = false;
			bool planning_successful = false;

			while (!planning_successful)
			{
				setTarget_successful = false;
				setTarget_successful = setPlanningTarget(current_setTarget_algorithm_);
				if (setTarget_successful)
				{
					//                                    ROS_INFO("Planning!");
					//                                    ROS_INFO_STREAM(group->getName());
					//                                    ROS_INFO_STREAM(group->getPoseReferenceFrame());
					planning_successful = group->plan(motion_plan_);

					if (planning_successful)
					{
						ROS_INFO("Planning successful!");
						planned_path_.clear();

						// for each configuration of the trajectory except from the start configuration
						for (unsigned configIdx = 0; configIdx < motion_plan_.trajectory_.joint_trajectory.points.size(); ++configIdx)
						{
							// current configuration
							euroc_c2_msgs::Configuration current_config;

							// for each joint at the current configuration
							for (unsigned jointIdx = 0; jointIdx < group->getActiveJoints().size(); ++ jointIdx)
							{
								current_config.q.push_back(motion_plan_.trajectory_.joint_trajectory.points[configIdx].positions[jointIdx]);
							}
							planned_path_.push_back(current_config);
						}

						current_configuration_ = planned_path_[0];

						move_along_joint_path_srv_.request.joint_names.clear();
						move_along_joint_path_srv_.request.joint_names = group->getActiveJoints();

						move_along_joint_path_srv_.request.path.clear();
						move_along_joint_path_srv_.request.path.resize(planned_path_.size()-1);
						for (unsigned idx = 0; idx < move_along_joint_path_srv_.request.path.size(); ++idx)
						{
							move_along_joint_path_srv_.request.path[idx] = planned_path_[idx+1];
						}

						// set the joint limits (velocities/accelerations) of the move along joint path service request
						setMoveRequestJointLimits();
						// set the TCP limits of the move along joint path service
						setMoveRequestTCPLimits();

						return true;
					}
				}

				current_setTarget_attempt++;
				if (current_setTarget_attempt > max_setTarget_attempts_)
				{
					msg_error("MoveIt: No Motion Plan found!");
					goalPose_result_.error_reason = fsm::NO_IK_SOL;
					return false;
				}

				current_setTarget_algorithm_++;
			}


		}
		else      // if (!ros::service::waitForService(move_along_joint_path_,ros::Duration(10.0)))
		{
			goalPose_result_.error_reason = fsm::SIM_SRV_NA;
			return false;
		}

	} // end try

	catch(...)
	{
		msg_error("MotionPlanning::Error in MoveIt! Planning.");
		goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
		return false;
	}
	return true;


}




bool MotionPlanning::MoveIt_initializeMoveGroup()
{


	// Load current robot state into planning scene
	planning_scene_.robot_state.joint_state = getCurrentJointState();
	planning_scene_.robot_state.is_diff = true;




	//	get_planning_scene_srv_.request.components.components = get_planning_scene_srv_.request.components.ROBOT_STATE_ATTACHED_OBJECTS;
	//	get_planning_scene_srv_.request.components.components = get_planning_scene_srv_.request.components.ROBOT_STATE;
	get_planning_scene_srv_.request.components.components
	= get_planning_scene_srv_.request.components.ROBOT_STATE
	+ get_planning_scene_srv_.request.components.ROBOT_STATE_ATTACHED_OBJECTS;

	if (ros::service::waitForService(planning_scene_monitor->DEFAULT_PLANNING_SCENE_SERVICE,ros::Duration(10.0)))
	{
		get_planning_scene_client_.call(get_planning_scene_srv_);

		get_planning_scene_srv_.response.scene.robot_state.joint_state = planning_scene_.robot_state.joint_state;

		group->setStartState(get_planning_scene_srv_.response.scene.robot_state);
	}


	else
	{
		ROS_ERROR("Setting start state failed");
		goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
		return false;
	}


	// print the planning interface description
	moveit_msgs::PlannerInterfaceDescription plintdesc;
	group->getInterfaceDescription(plintdesc);
	//    ROS_INFO_STREAM("Name of the planner interface: " << plintdesc.name);
	//    ROS_INFO("Names of the planner ID's within the interface:");
	//    for (unsigned idx = 0; idx < plintdesc.planner_ids.size(); ++idx)
	//    {
	//            ROS_INFO_STREAM(idx << ": " << plintdesc.planner_ids[idx]);
	//    }

	// print the planning reference frame
	//    ROS_INFO_STREAM("Planning frame:" << group->getPlanningFrame());
	//    // print the pose reference frame
	//    ROS_INFO_STREAM("Pose reference frame: " << group->getPoseReferenceFrame());
	//    // print default planning time
	//    ROS_INFO_STREAM("Default planning time: " << group->getPlanningTime() << " seconds.");
	group->setPlanningTime(allowed_planning_time_);
	//    ROS_INFO_STREAM("Planning time set to " << group->getPlanningTime() << " seconds.");
	//    // print the name of the end effector
	//    ROS_INFO_STREAM("End effector: " << group->getEndEffector());
	//    // print the name of the end effector link
	//    ROS_INFO_STREAM("End effector link: " << group->getEndEffectorLink());
	//==========================================================================================



	// get the robot model
	robot_model::RobotModelConstPtr robot_model = planning_scene_monitor->getRobotModel();
	const robot_model::JointModelGroup* joint_model_group_LWR = robot_model->getJointModelGroup("LWR_9DOF");
	//    ROS_WARN("JOINT BOUNDS");
	moveit::core::JointBoundsVector joint_bounds = joint_model_group_LWR->getActiveJointModelsBounds();




	//==========================================================================================
	// OCTOMAP
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	kinematic_model_ = robot_model_loader.getModel();

	// if Octomap received, then stored in _octree

	//	if(!skip_vision_)
	//	{
	//		if (octomap_manager_cleanupOctomap())
	//			msg_info("Octomap cleaned up");
	//	}

	if(octomap_manager_getOctomap())
	{
		// Arming a planning scene message with octomap
		_octree.header.frame_id =("/Origin");
		planning_scene_.world.octomap.octomap = _octree;
		// arming planning scene message with tf information
		planning_scene_.world.octomap.octomap.header.frame_id = "/Origin";
		planning_scene_.world.octomap.octomap.header.stamp = ros::Time::now();
		planning_scene_.world.octomap.header.frame_id = "/Origin";
		planning_scene_.world.octomap.header.stamp = ros::Time::now();
	}

	//	 Robot link padding
	//	planning_scene_.link_padding.resize(planning_scene_monitor->getRobotModel()->getLinkModelNames().size());
	//	for (unsigned i = 0; i < planning_scene_monitor->getRobotModel()->getLinkModelNames().size(); ++i)
	//	{
	//		planning_scene_.link_padding[i].link_name = planning_scene_monitor->getRobotModel()->getLinkModelNames()[i];
	//		if (!planning_scene_.link_padding[i].link_name.compare("finger1") || !planning_scene_.link_padding[i].link_name.compare("finger2"))
	//			planning_scene_.link_padding[i].padding = 0.0;
	//		else if (!planning_scene_.link_padding[i].link_name.compare("base"))
	//			planning_scene_.link_padding[i].padding = 0.01;
	//		else
	//			planning_scene_.link_padding[i].padding = 0.00;
	//	}

	// setting planning scene message to type diff
	planning_scene_.is_diff = true;

	// Publish msg on topic /planning_scene
	planning_scene_diff_publisher_.publish(planning_scene_);
	planning_scene_diff_publisher_.publish(static_scene_);

	//==========================================================================================

	return true;
}


euroc_c2_msgs::Configuration MotionPlanning::getCurrentConfiguration()
{
	euroc_c2_msgs::Configuration current_configuration;

	if (getTelemetry())
	{
		// Populate a vector with all the lwr joint names
		std::vector<std::string> lwr_joints = group->getActiveJoints();
		current_configuration.q.resize(group->getActiveJoints().size());

		// Get the current configuration from the telemetry message
		for(unsigned int i = 0; i < group->getActiveJoints().size(); ++i)
		{
			std::vector<std::string> &joint_names = (_telemetry.joint_names);
			unsigned int telemetry_index = std::find(joint_names.begin(), joint_names.end(), lwr_joints[i]) - joint_names.begin();
			current_configuration.q[i] = _telemetry.measured.position[telemetry_index];
		}
	}

	return current_configuration;
}

bool MotionPlanning::setPlanningTarget(unsigned algorithm)
{
	switch (algorithm) {

	case JOINT_VALUE_TARGET_KDL_IK_RELEASE_CONSTRAINT:
		group->setGoalJointTolerance(release_goal_joint_tolerance);
		group->setGoalOrientationTolerance(release_goal_orientation_tolerance);
		group->setGoalPositionTolerance(release_goal_position_tolerance);
	case JOINT_VALUE_TARGET_KDL_IK: {
		ROS_INFO("Setting a Joint value target obtained via the KDL IK.");

		if (algorithm!=JOINT_VALUE_TARGET_KDL_IK_RELEASE_CONSTRAINT)
		{
			group->setGoalJointTolerance(std_goal_joint_tolerance);
			group->setGoalOrientationTolerance(std_goal_orientation_tolerance);
			group->setGoalPositionTolerance(std_goal_position_tolerance);
		}
		if (goal_pose_goal_->planning_algorithm == MOVE_IT_2DOF)
			group->setGoalTolerance(twodof_goal_tolerance);
		// declare seed state
		std::vector<double> ik_seed_state;

		if (active_task_nr_ == 1
				|| active_task_nr_ == 2)
		{
			ik_seed_state.resize(joint_model_group_->getActiveJointModels().size());

			try
			{
				if(ros::service::waitForService(search_ik_solution_,ros::Duration(10.0)))
				{
					current_configuration_.q.resize(7);

					search_ik_solution_srv_.request.start = current_configuration_;
					search_ik_solution_srv_.request.tcp_frame = goal_pose_GPTCP_;

					search_ik_solution_client_.call(search_ik_solution_srv_);
					std::string &search_error_message = search_ik_solution_srv_.response.error_message;
					if(!search_error_message.empty())
					{
						ROS_WARN("No seed state found via searchIKSolution srv.");
						ROS_INFO("Setting seed state to current configuration...");
						ik_seed_state = current_configuration_.q;
					}
					else
					{
						ROS_INFO("Seed state found via searchIKSolution srv.");
						ik_seed_state = search_ik_solution_srv_.response.solution.q;
					}
				}
			}
			catch(...)
			{
				msg_error("failed to find service search ik-solution");
				return false;
			}
		}
		else if (active_task_nr_ == 3
				|| active_task_nr_ == 4
				|| active_task_nr_ == 5
				|| active_task_nr_ == 6)
		{
			ROS_INFO("Setting seed state to current configuration...");
			ik_seed_state.resize(joint_model_group_->getActiveJointModels().size());
			ik_seed_state = getCurrentConfiguration().q;
		}


		//          ROS_INFO("computing KDL IK...");
		std::vector<double> solution;
		moveit_msgs::MoveItErrorCodes error_code;
		if (!joint_model_group_->getSolverInstance()->getPositionIK(goal_pose_GPTCP_,
				ik_seed_state,
				solution,
				error_code))
		{
			ROS_WARN("KDL->getPositionIK() failed.");
			return false;
		}
		else
		{
			ROS_INFO("KDL->getPositionIK() successful.");

			// do self collision checking!
			ROS_INFO("Checking solution for self collision...-disabled because motion planning chrashes!");


			if (!group->setJointValueTarget(solution))
			{
				ROS_ERROR("Setting joint value target failed.");
				return false;
			}
			else
			{
				// TODO
				// check if state valid and collision free
				break;
			}
			//}
		}

		break;
	}

	case JOINT_VALUE_TARGET_EUROC_IK: {
		ROS_INFO(
				"Setting a joint value target obtained via the SearchIKSolution srv.");
		if(group->getActiveJoints().size() == 2)
		{
			ROS_ERROR("JOINT_VALUE_TARGET_EUROC_IK called for 2 DOF!");
			return false;
		}
		try {
			if (ros::service::waitForService(search_ik_solution_,
					ros::Duration(10.0))) {
				current_configuration_.q.resize(7);

				search_ik_solution_srv_.request.start = current_configuration_;
				search_ik_solution_srv_.request.tcp_frame = goal_pose_GPTCP_;

				search_ik_solution_client_.call(search_ik_solution_srv_);
				std::string &search_error_message =
						search_ik_solution_srv_.response.error_message;
				if (!search_error_message.empty()) {
					ROS_ERROR("Search IK solution failed.");
					return false;
				} else {
					ROS_INFO("Search IK solution successful.");
					if (!group->setJointValueTarget(
							search_ik_solution_srv_.response.solution.q)) {
						ROS_ERROR(
								"Setting obtained joint value target failed.");
						return false;
					}
				}
			}
		} catch (...) {
			msg_error("failed to find service search ik-solution");
			return false;
		}
		break;
	}
	case POSE_TARGET_RELEASE_CONSTRAINT:
		group->setGoalJointTolerance(release_goal_joint_tolerance);
		group->setGoalOrientationTolerance(release_goal_orientation_tolerance);
		group->setGoalPositionTolerance(release_goal_position_tolerance);
	case POSE_TARGET_EXP:
	case POSE_TARGET: {
		if (algorithm!=POSE_TARGET_RELEASE_CONSTRAINT)
		{
			group->setGoalJointTolerance(std_goal_joint_tolerance);
			group->setGoalOrientationTolerance(std_goal_orientation_tolerance);
			group->setGoalPositionTolerance(std_goal_position_tolerance);
		}
		if (goal_pose_goal_->planning_algorithm == MOVE_IT_2DOF)
			group->setGoalTolerance(twodof_goal_tolerance);
		ROS_INFO("Setting a pose target.");

		if (!group->setPoseTarget(goal_pose_GPTCP_)) {
			ROS_ERROR("Setting pose target failed.");
			return false;
		}
		break;
	}
	case SINGLE_POSE_RELEASE_CONSTRAINT:
		group->setGoalJointTolerance(release_goal_joint_tolerance);
		group->setGoalOrientationTolerance(release_goal_orientation_tolerance);
		group->setGoalPositionTolerance(release_goal_position_tolerance);
	case SINGLE_POSE_TARGET: {
		ROS_INFO("Setting a single pose target.");
		if (algorithm!=SINGLE_POSE_RELEASE_CONSTRAINT)
		{
			group->setGoalJointTolerance(std_goal_joint_tolerance);
			group->setGoalOrientationTolerance(std_goal_orientation_tolerance);
			group->setGoalPositionTolerance(std_goal_position_tolerance);
		}
		if (goal_pose_goal_->planning_algorithm == MOVE_IT_2DOF)
			group->setGoalTolerance(twodof_goal_tolerance);
		if (!group->setJointValueTarget(goal_pose_GPTCP_)) {
			ROS_ERROR("Setting pose target failed.");
			return false;
		}
		break;
	}
	case HOMING: {
		ROS_INFO("Setting a joint value target (HOMING).");

		group->setGoalJointTolerance(release_goal_joint_tolerance);

		std::vector<double> goal_config_homing;
		goal_config_homing.resize(group->getActiveJoints().size());
		for (uint64_t ii = 0; ii < group->getActiveJoints().size(); ii++) {
			goal_config_homing[ii] = 0.0;
		}

		if (!group->setJointValueTarget(goal_config_homing)) {
			ROS_ERROR("Setting homing joint value target failed.");
			return false;
		}

		break;
	}
	case APPROX_JOINT_VALUE_TARGET_9DOF:
	case APPROXIMATE_JOINT_VALUE_TARGET: {
		ROS_INFO("Planning for an approximate joint value target chosen.");
		if (!group->setApproximateJointValueTarget(
				goal_pose_goal_->goal_pose)) {
			ROS_ERROR("Setting approximate joint value target failed.");
			return false;
		}
		break;
	}

	case JOINT_VALUE_TARGET_9DOF:
	{
		ROS_INFO("Setting a joint value target.");
		group->setGoalJointTolerance(release_goal_joint_tolerance);
		std::vector<double> joint_values_in(9);
		for (int i=0;i<9;i++)
			joint_values_in[i] = goal_pose_goal_->goal_config.q[i];
		if (!group->setJointValueTarget(joint_values_in)) {
			ROS_ERROR("Setting joint value target 9DOF failed.");
			return false;
		}
		break;
	}
	case RANDOM_TARGET:
	{
		ROS_INFO("Setting a random target.");
		try{group->setRandomTarget();}catch(...){msg_error("Error setting random pose."); return false;}
		break;
	}

	default: {
		ROS_ERROR("False setTarget algorithm.");
		return false;
	}
	}

	msg_warn("goal Joint Tolerance %f",group->getGoalJointTolerance());
	msg_warn("goalOrientation Tolerance %f",group->getGoalOrientationTolerance());
	msg_warn("goalPosition Tolerance %f",group->getGoalPositionTolerance());

	ROS_INFO("Setting target successful.");
	return true;
}

void MotionPlanning::getTimingAlongJointPath()
{
	if (ros::service::waitForService(timing_along_joint_path_,ros::Duration(1.0)))
	{
		//TODO expected TIMING with getTimingAlongJointPath
		timing_along_joint_path_srv_.request.joint_names = move_along_joint_path_srv_.request.joint_names;

		int n_joints = timing_along_joint_path_srv_.request.joint_names.size();

		//timing_along_joint_path_srv_.request.start_pose =
		timing_along_joint_path_srv_.request.path = move_along_joint_path_srv_.request.path;
		timing_along_joint_path_srv_.request.joint_limits = move_along_joint_path_srv_.request.joint_limits;

		timing_along_joint_path_srv_.request.start_pose = current_configuration_;

		if(!timing_along_joint_path_client_.call(timing_along_joint_path_srv_))
		{
			msg_error("get timing along joint path call failed");

			goalPose_result_.error_reason = fsm::SIM_SRV_NA;
			return;
		}

		time_at_path_points_.resize(timing_along_joint_path_srv_.response.time_at_via_point.size());
		time_at_path_points_ = timing_along_joint_path_srv_.response.time_at_via_point;

		estimated_motion_time_ = 0.0;

		if (time_at_path_points_.size()>1){
			for (int i=1;i<time_at_path_points_.size();i++)
			{
				estimated_motion_time_ += time_at_path_points_[i].toSec()-time_at_path_points_[i-1].toSec();
			}}
		else
			estimated_motion_time_ = time_at_path_points_[0].toSec();

		msg_info("Estimated Motion Time: %f",estimated_motion_time_);
	}
	else
	{
		msg_warn("Timing service has not been advertised.");

		goalPose_result_.error_reason = fsm::SIM_SRV_NA;
		return;
	}

}

bool MotionPlanning::getTelemetry()
{
	_telemetry = *(ros::topic::waitForMessage<euroc_c2_msgs::Telemetry>(telemetry_,ros::Duration(10.0)));
	if (&_telemetry==NULL)
	{
		msg_warn("No telemetry message received.");
		return false;
	}
	else
		return true;
}

void MotionPlanning::moveToTargetCB()
{
	// Call the move request and check for errors
	if(!move_along_joint_path_client_.call(move_along_joint_path_srv_)) //TODO CHange back to original
	{
		move_along_joint_path_srv_.response.error_message = "Failed to call client to move along joint path";
		msg_error("Failed to call client to move along joint path.");

		mtt_=FINISHEDWITHERROR;
		goalPose_result_.error_reason = fsm::SIM_SRV_NA;

		return;
	}

	// Print out stop reason
	std::string joint = "joint ";
	std::string tool = "tool force";
	std::string ext_torque = "ext_torque ";
	std::string is = "is ";
	std::string StopReason = move_along_joint_path_srv_.response.stop_reason;

	if( (StopReason != "path finished") && (StopReason != "path finished (already at target)") )
	{
		if (StopReason == "")
		{
			mtt_=FINISHEDWITHERROR;
			msg_error("%s", move_along_joint_path_srv_.response.error_message.c_str());
			goalPose_result_.error_reason = fsm::RESTART_SIM;
			return;
		}
		else
		{
			msg_info("Stop Reason: %s", StopReason.c_str());

			if (StopReason.find(joint) != std::string::npos)
			{
				std::string joint_no = StopReason.substr(StopReason.find(joint) + joint.size(), 1);

				//joint_name no is given number plus 1 (in simulation joints start from 0)
				call_set_stop_cond_srv_.request.joint_nbr = std::atoi(joint_no.c_str());

				//testing atof functionality
				std::string string_torque = StopReason.substr(StopReason.find(ext_torque) + ext_torque.size(), StopReason.find(" ")); // until next space
				call_set_stop_cond_srv_.request.current_value = std::atof(string_torque.c_str());
			}
			else if (StopReason.find(tool) != std::string::npos)
			{
				call_set_stop_cond_srv_.request.joint_nbr = 7; // gripper
				std::string string_force = StopReason.substr(StopReason.find(is) + is.size(), StopReason.find(" "));
				call_set_stop_cond_srv_.request.current_value = std::atof(string_force.c_str());
			}
			else
			{
				msg_error("unknown stop reason!");
			}

			msg_error("Stop reason: %s", move_along_joint_path_srv_.response.stop_reason.c_str());
			call_set_stop_cond_srv_.request.level = 1; // level 1 = limits eased

		}

		if(!state_observer_client_.call(call_set_stop_cond_srv_))
		{
			msg_error("Failed to call client to set stop conditions.");

			mtt_=FINISHEDWITHERROR;
			goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
			return;
		}

		//check if max limits reached in stop cond. serv.
		if(call_set_stop_cond_srv_.response.limit_reached)
		{
			mtt_=FINISHEDWITHERROR;
			goalPose_result_.error_reason = fsm::MAX_LIMIT_REACHED;
			return;
		}
		else
		{
			mtt_=FINISHEDWITHERROR;
			goalPose_result_.error_reason = fsm::STOP_COND;
			return;
		}
	}
	else
	{
		call_set_stop_cond_srv_.request.level = 0;
		if(!state_observer_client_.call(call_set_stop_cond_srv_))
		{
			msg_error("Failed to call client to set stop conditions.");

			mtt_=FINISHEDWITHERROR;
			goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
			return;
		}
	}
	// above by Anna

	std::string &move_error_message = move_along_joint_path_srv_.response.error_message;
	if(!move_error_message.empty()){
		msg_error("Move failed...");
		std::cout << "Move failed: " + move_error_message << std::endl;
	}
	mtt_=FINISHED;
}

#if 0

bool MotionPlanning::return_poses_valid(am_msgs::CheckPoses::Request &req, am_msgs::CheckPoses::Response &res)
{
	uint16_t nr_poses = req.poses.size();
	ROS_INFO("in check poses service call:");
	ROS_INFO_STREAM(req.poses.size() << " poses to be checked.");

	res.priority.resize(nr_poses);

	for(uint16_t ii=0;ii<nr_poses;ii++)
	{
		ROS_INFO_STREAM("Pose " << ii << " of " << req.poses.size()-1);

		// first check via kdl inverse kinematics solver
		// valid_kdl_ik(req.poses[ii], res.priority[ii]);

		// then check via EUROC's IK service
		euroc_valid_euroc_ik(req.poses[ii], res.priority[ii]);

	}

	ROS_INFO("finished check poses service call.");

	return true;
}
#else

bool MotionPlanning::return_poses_valid(am_msgs::CheckPoses::Request &req, am_msgs::CheckPoses::Response &res)
{
	uint16_t nr_poses = req.poses.size();
	//ROS_INFO("in check poses service call:");
	//ROS_INFO_STREAM(req.poses.size() << " poses to be checked.");

	ros::param::get("/active_task_number_", active_task_nr_);

	res.priority.resize(nr_poses);

	for(uint16_t ii=0;ii<nr_poses;ii++)
	{
		//ROS_INFO_STREAM("Pose " << ii << " of " << req.poses.size()-1);

		// first check via kdl inverse kinematics solver
		// valid_kdl_ik(req.poses[ii], res.priority[ii]);

		// then check via EUROC's IK service
		if(active_task_nr_ == 1 || active_task_nr_ == 2)
		{
			euroc_valid_euroc_ik(req.poses[ii], res.priority[ii]);
		}
		else
		{
			res.priority[ii] = 1;
		}
	}

	//ROS_INFO("finished check poses service call.");

	return true;
}
#endif
bool MotionPlanning::euroc_valid_euroc_ik(geometry_msgs::Pose& pose, short unsigned int& priority)
{
	try
	{
		if(ros::service::waitForService(search_ik_solution_,ros::Duration(10.0)))
		{
			// current_configuration will hold our current joint position data extracted from the measured telemetry
			current_configuration_.q.resize(7);

			// Select the next desired position of the tcp from the target zone poses and fill
			// the search inverse kinematic solution request with the current configuration as
			// start configuration and the desired position
			search_ik_solution_srv_.request.start = current_configuration_;
			search_ik_solution_srv_.request.tcp_frame = pose;

			search_ik_solution_client_.call(search_ik_solution_srv_);
			std::string &search_error_message = search_ik_solution_srv_.response.error_message;
			if(!search_error_message.empty())
			{
				//ROS_INFO("Search IK Solution failed: %s", search_error_message.c_str());
				priority = 0;
			}
			else
			{
				priority = 1;
			}
		}
	}
	catch(...)
	{
		msg_error("failed to find service search ik-solution");
		return false;
	}

	//ROS_INFO_STREAM("valid via IK service: " << priority);
	return true;

}

bool MotionPlanning::MoveIt_valid_kdl_ik(geometry_msgs::Pose& pose, short unsigned int& priority)
{
	int active_task_number;
	ros::param::get("/active_task_number_", active_task_number);

	if (active_task_number == 1 || active_task_number == 2)
	{

		joint_model_group_ = joint_model_group_7DOF_;

		// compute inverse kinematics via search ik solution srv as seed state
		try
		{
			if(ros::service::waitForService(search_ik_solution_,ros::Duration(10.0)))
			{
				current_configuration_.q.resize(7);

				search_ik_solution_srv_.request.start = current_configuration_;
				search_ik_solution_srv_.request.tcp_frame = pose;

				search_ik_solution_client_.call(search_ik_solution_srv_);
				std::string &search_error_message = search_ik_solution_srv_.response.error_message;
				if(!search_error_message.empty())
				{
					ROS_WARN("No seed state found via searchIKSolution srv.");
					priority = 0;
				}
				else
				{
					ROS_INFO("Seed state:");
					for (unsigned idx = 0; idx < search_ik_solution_srv_.response.solution.q.size(); ++idx)
						ROS_INFO_STREAM(search_ik_solution_srv_.response.solution.q[idx]);

					std::vector<double> solution;
					moveit_msgs::MoveItErrorCodes error_code;

					if (!joint_model_group_->getSolverInstance()->getPositionIK(pose,
							search_ik_solution_srv_.response.solution.q,
							solution,
							error_code))
					{
						ROS_WARN("No KDL IK found even though a seed state is given.");
						priority = 0;
					}
					else
					{
						ROS_INFO("Solution state:");
						for (unsigned idx = 0; idx < solution.size(); ++idx)
							ROS_INFO_STREAM(solution[idx]);

						// do self collision checking!
						collision_detection::CollisionRequest collision_request;
						collision_detection::CollisionResult collision_result;

						kinematic_state_->setJointGroupPositions(joint_model_group_, solution);

						ROS_INFO("Checking for self collision...");
						planning_scene_monitor->getPlanningScene()->checkSelfCollision(collision_request,
								collision_result,
								*kinematic_state_);

						if(collision_result.collision)
						{
							ROS_WARN("Collision occurred!");
							priority = 0;
						}
						else
							priority = 1;
					}
				}
			}
		}
		catch(...)
		{
			msg_error("failed to find service search ik-solution");
			return false;
		}

	}
	else if (active_task_number == 3 || active_task_number == 4 || active_task_number == 5)
	{
		joint_model_group_ = joint_model_group_9DOF_;

		// check validity of the poses via kdl inverse kinematics
		robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model_));

		kinematic_state->enforceBounds();

		if (kinematic_state->setFromIK(joint_model_group_, pose, "gripper_tcp", 10, 0.1))
		{
			priority = 1;
		}
		else
		{
			priority = 0;
		}
	}

	ROS_INFO_STREAM("valid via kdl: " << priority);
	return true;
}


bool MotionPlanning::getLimits()
{
	//! Setting max velocities, getting from parameter server
	try
	{
		if (!ros::param::get("/two_axes_speed_limit_0",table_axis1_limit_.max_velocity))
			table_axis1_limit_.max_velocity = 0.5;
		if (!ros::param::get("/two_axes_speed_limit_1",table_axis2_limit_.max_velocity))
			table_axis2_limit_.max_velocity = 0.5;
		if (!ros::param::get("/joint_speed_limit_0",joint_limits_[0].max_velocity)){
			for (int ii=0;ii<7;ii++)
				joint_limits_[ii].max_velocity = 20 * M_PI / 180.0;
		}
		else
		{
			ros::param::get("/joint_speed_limit_1",joint_limits_[1].max_velocity);
			ros::param::get("/joint_speed_limit_2",joint_limits_[2].max_velocity);
			ros::param::get("/joint_speed_limit_3",joint_limits_[3].max_velocity);
			ros::param::get("/joint_speed_limit_4",joint_limits_[4].max_velocity);
			ros::param::get("/joint_speed_limit_5",joint_limits_[5].max_velocity);
			ros::param::get("/joint_speed_limit_6",joint_limits_[6].max_velocity);
		}
		if (!ros::param::get("/gripper_speed_limit",gripper_limit_.max_velocity))
			gripper_limit_.max_velocity = 0.5;
	}
	catch (...)
	{
		msg_error("GET LIMITS aborted. Setting default values.");
		table_axis1_limit_.max_velocity = 0.5;
		table_axis2_limit_.max_velocity = 0.5;
		for (int ii=0;ii<7;ii++)
			joint_limits_[ii].max_velocity = 20 * M_PI / 180.0;
		gripper_limit_.max_velocity = 0.5;
	}

	//! Setting max accelerations
	table_axis1_limit_.max_acceleration = 0.5;//2.0;
	table_axis2_limit_.max_acceleration = 0.5;//2.0;
	for (int ii=0;ii<7;ii++)
		joint_limits_[ii].max_acceleration = 120 * M_PI / 180.0;

	gripper_limit_.max_acceleration = 2.0;

	table_axis1_limit_.max_velocity *= (double)speed_percentage_*0.01;
	table_axis2_limit_.max_velocity *= (double)speed_percentage_*0.01;
	gripper_limit_.max_velocity     *= (double)speed_percentage_*0.01;
	for (int ii=0;ii<7;ii++)
		joint_limits_[ii].max_velocity *= (double)speed_percentage_*0.01;


	return true;

}

void MotionPlanning::setMoveRequestJointLimits()
{
	move_along_joint_path_srv_.request.joint_limits.clear();
	move_along_joint_path_srv_.request.joint_limits.resize(group->getActiveJoints().size());

	unsigned joint_counter = 0;

	if (move_along_joint_path_srv_.request.joint_limits.size() == 9 || move_along_joint_path_srv_.request.joint_limits.size() == 2)
	{

		move_along_joint_path_srv_.request.joint_limits[0].max_velocity = table_axis1_limit_.max_velocity;
		move_along_joint_path_srv_.request.joint_limits[0].max_acceleration = table_axis1_limit_.max_acceleration;

		move_along_joint_path_srv_.request.joint_limits[1].max_velocity = table_axis2_limit_.max_velocity;
		move_along_joint_path_srv_.request.joint_limits[1].max_acceleration = table_axis2_limit_.max_acceleration;

		joint_counter = 2;
	}

	if(move_along_joint_path_srv_.request.joint_limits.size() != 2)
	{
		unsigned jointIdx = 0;
		for(unsigned idx = joint_counter; idx < joint_counter+7; ++idx)
		{
			move_along_joint_path_srv_.request.joint_limits[idx].max_velocity = joint_limits_[jointIdx].max_velocity;
			move_along_joint_path_srv_.request.joint_limits[idx].max_acceleration = joint_limits_[jointIdx].max_acceleration;

			jointIdx++;
		}
	}



}


void MotionPlanning::setMoveRequestTCPLimits()
{
	move_along_joint_path_srv_.request.tcp_limits.translational.max_velocity = table_axis1_limit_.max_velocity;
	move_along_joint_path_srv_.request.tcp_limits.translational.max_acceleration = table_axis1_limit_.max_acceleration;

	move_along_joint_path_srv_.request.tcp_limits.rotational.max_velocity = 120 * M_PI / 180.0; //120
	move_along_joint_path_srv_.request.tcp_limits.rotational.max_acceleration = 90 * M_PI / 180.0;
}

bool MotionPlanning::transformToTCPFrame(std::string frame)
{

	//! This function transforms the goal pose given for the gripper TCP frme in world coordinates to a goal pose in the LWR TCP frame in the LWR0 System
	tf::TransformListener tf_listener;
	tf::StampedTransform transform_GPTCP_2_LWRTCP;
	tf::Transform tf_tmp,tf_tmp2;

	ros::Time now = ros::Time(0);
	//! TODO remove debug_tf
	std::string debug_tf;

	//! Transformation from GP TCP frame to LWR TCP frame
	try{
		if (tf_listener.waitForTransform(LWR_TCP,GP_TCP,now,ros::Duration(1.0),ros::Duration(3.0),&debug_tf))
			tf_listener.lookupTransform(LWR_TCP,GP_TCP,now,transform_GPTCP_2_LWRTCP);
		else{
			msg_error("Could not get LWRTCP GPTCP tf.");
			std::cout<<debug_tf<<std::endl;
			return false;
		}
	}
	catch(...){
		ROS_ERROR("Listening to transform was not successful");
		return false;
	}

	tf_tmp.setOrigin(tf::Vector3(goal_pose_goal_->goal_pose.position.x,
			goal_pose_goal_->goal_pose.position.y,
			goal_pose_goal_->goal_pose.position.z));
	tf_tmp.setRotation(tf::Quaternion(goal_pose_goal_->goal_pose.orientation.x,
			goal_pose_goal_->goal_pose.orientation.y,
			goal_pose_goal_->goal_pose.orientation.z,
			goal_pose_goal_->goal_pose.orientation.w));

	if (!frame.compare(GP_TCP)){

		tf_tmp2.mult(tf_tmp,transform_GPTCP_2_LWRTCP.inverse());

		goal_pose_GPTCP_ = goal_pose_goal_->goal_pose;

		goal_pose_LWRTCP_.position.x = tf_tmp2.getOrigin().getX();
		goal_pose_LWRTCP_.position.y = tf_tmp2.getOrigin().getY();
		goal_pose_LWRTCP_.position.z = tf_tmp2.getOrigin().getZ();

		goal_pose_LWRTCP_.orientation.x = tf_tmp2.getRotation().getX();
		goal_pose_LWRTCP_.orientation.y = tf_tmp2.getRotation().getY();
		goal_pose_LWRTCP_.orientation.z = tf_tmp2.getRotation().getZ();
		goal_pose_LWRTCP_.orientation.w = tf_tmp2.getRotation().getW();

		//          ROS_INFO("LWRTCP Pose in MotionPlanning: [%f %f %f .. %f %f %f %f]" , goal_pose_LWRTCP_.position.x,
		//                          goal_pose_LWRTCP_.position.y,goal_pose_LWRTCP_.position.z,goal_pose_LWRTCP_.orientation.x,
		//                          goal_pose_LWRTCP_.orientation.y,goal_pose_LWRTCP_.orientation.z,goal_pose_LWRTCP_.orientation.w);

	}
	else if (!frame.compare(LWR_TCP)){

		tf_tmp2.mult(tf_tmp,transform_GPTCP_2_LWRTCP);

		goal_pose_LWRTCP_ = goal_pose_goal_->goal_pose;

		goal_pose_GPTCP_.position.x = tf_tmp2.getOrigin().getX();
		goal_pose_GPTCP_.position.y = tf_tmp2.getOrigin().getY();
		goal_pose_GPTCP_.position.z = tf_tmp2.getOrigin().getZ();

		goal_pose_GPTCP_.orientation.x = tf_tmp2.getRotation().getX();
		goal_pose_GPTCP_.orientation.y = tf_tmp2.getRotation().getY();
		goal_pose_GPTCP_.orientation.z = tf_tmp2.getRotation().getZ();
		goal_pose_GPTCP_.orientation.w = tf_tmp2.getRotation().getW();
	}


	return true;
}

bool MotionPlanning::transformToLWRBase()
{

	//! This function transforms the goal pose given for the gripper TCP fram in world coordinates to a goal pose in the LWR TCP frame in the LWR0 System
	tf::TransformListener tf_listener;
	tf::StampedTransform transform_ORIGIN_2_LWR0;
	tf::Transform tf_tmp,tf_tmp2;

	ros::Time now = ros::Time(0);

	//! Transformation from ORIGIN frame to LWR 0 frame
	try{
		if (tf_listener.waitForTransform(LWR_0,ORIGIN,now,ros::Duration(2.0)))
			tf_listener.lookupTransform(LWR_0,ORIGIN,now,transform_ORIGIN_2_LWR0);
		else{
			msg_error("Could not get ORIGIN LWR0 tf.");
			return false;
		}
	}
	catch(...){
		msg_error("Listening to transform was not successful");
		return false;
	}
	tf_tmp.setOrigin(tf::Vector3(goal_pose_LWRTCP_.position.x,
			goal_pose_LWRTCP_.position.y,
			goal_pose_LWRTCP_.position.z));
	tf_tmp.setRotation(tf::Quaternion(goal_pose_LWRTCP_.orientation.x,
			goal_pose_LWRTCP_.orientation.y,
			goal_pose_LWRTCP_.orientation.z,
			goal_pose_LWRTCP_.orientation.w));

	//    tf_tmp2 = transform_ORIGIN_2_LWR0*tf_tmp;
	tf_tmp2.mult(transform_ORIGIN_2_LWR0,tf_tmp);

	goal_pose_LWRTCP_.position.x = tf_tmp2.getOrigin().getX();
	goal_pose_LWRTCP_.position.y = tf_tmp2.getOrigin().getY();
	goal_pose_LWRTCP_.position.z = tf_tmp2.getOrigin().getZ();

	goal_pose_LWRTCP_.orientation.x = tf_tmp2.getRotation().getX();
	goal_pose_LWRTCP_.orientation.y = tf_tmp2.getRotation().getY();
	goal_pose_LWRTCP_.orientation.z = tf_tmp2.getRotation().getZ();
	goal_pose_LWRTCP_.orientation.w = tf_tmp2.getRotation().getW();

	return true;
}



void MotionPlanning::initializePlanningScene()
{
	ROS_INFO("Adding ground plane and pan tilt station to collision world...");

	//************************
	// ground plane
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 5.0;//1.84;
	primitive.dimensions[1] = 5.0;//1.84;
	primitive.dimensions[2] = 0.01;

	geometry_msgs::Pose primitive_pose;
	primitive_pose.position.x = 0;
	primitive_pose.position.y = 0;
	primitive_pose.position.z = 0;//-0.01;
	primitive_pose.orientation.x = 0;
	primitive_pose.orientation.y = 0;
	primitive_pose.orientation.z = 0;
	primitive_pose.orientation.w = 1;

	moveit_msgs::CollisionObject object;
	object.id = "ground";
	object.header.frame_id = ORIGIN;
	object.header.stamp = ros::Time(0);
	object.operation = object.ADD;
	object.primitives.push_back(primitive);
	object.primitive_poses.push_back(primitive_pose);

	static_scene_.world.collision_objects.push_back(object);

	//************************
	// allow contact between base and ground plane
	planning_scene_monitor->getPlanningScene()->getAllowedCollisionMatrixNonConst().setEntry("ground", "base", true);
	moveit_msgs::AllowedCollisionMatrix acm;
	planning_scene_monitor->getPlanningScene()->getAllowedCollisionMatrixNonConst().getMessage(acm);
	static_scene_.allowed_collision_matrix = acm;



	//************************
	// pan tilt station
	geometry_msgs::Pose pan_tilt_pose_mast, pan_tilt_pose_cam;
	pan_tilt_pose_mast.position.x = 0.92;
	pan_tilt_pose_mast.position.y = 0.92;
	pan_tilt_pose_mast.position.z = 0.55;
	pan_tilt_pose_mast.orientation.w = 1;
	pan_tilt_pose_cam.position.x = 0.92;
	pan_tilt_pose_cam.position.y = 0.92;
	pan_tilt_pose_cam.position.z = 1.1;

	shape_msgs::SolidPrimitive pan_tilt_primitive_mast, pan_tilt_primitive_cam;
	pan_tilt_primitive_mast.type = pan_tilt_primitive_mast.CYLINDER;
	pan_tilt_primitive_mast.dimensions.resize(2);
	pan_tilt_primitive_mast.dimensions[0] = 1.1;// height
	pan_tilt_primitive_mast.dimensions[1] = 0.1;//0.05;// radius
	pan_tilt_primitive_cam.type = pan_tilt_primitive_cam.SPHERE;
	pan_tilt_primitive_cam.dimensions.resize(1);
	pan_tilt_primitive_cam.dimensions[0] = 0.25;//0.3;// radius

	moveit_msgs::CollisionObject pan_tilt_object;
	pan_tilt_object.header.frame_id = ORIGIN;
	pan_tilt_object.header.stamp = ros::Time(0);
	pan_tilt_object.id = "pan_tilt_mast";
	pan_tilt_object.primitives.push_back(pan_tilt_primitive_mast);
	pan_tilt_object.primitives.push_back(pan_tilt_primitive_cam);
	pan_tilt_object.primitive_poses.push_back(pan_tilt_pose_mast);
	pan_tilt_object.primitive_poses.push_back(pan_tilt_pose_cam);

	static_scene_.world.collision_objects.push_back(pan_tilt_object);


	// Puzzle Fixture for task 5
	if (active_task_nr_ == 5)
	{
		tf::TransformListener tf_listener;

		tf::StampedTransform transform_PUZZLE_2_ORIGIN;
		tf::Transform tf_tmp,tf_tmp2;
		//! TODO remove debug_tf
		std::string debug_tf;

		//! Transformation from GP TCP frame to LWR TCP frame
		try{
			if (tf_listener.waitForTransform(ORIGIN,PUZZLE_FIXTURE,ros::Time(0),ros::Duration(2.0)),&debug_tf)
				tf_listener.lookupTransform(ORIGIN,PUZZLE_FIXTURE,ros::Time(0),transform_PUZZLE_2_ORIGIN);
			else{
				msg_error("Could not get LWRTCP GPTCP tf.");
				std::cout<<debug_tf<<std::endl;
			}
		}
		catch(...){
			ROS_ERROR("Listening to transform was not successful");
		}


		shape_msgs::SolidPrimitive puzzle_fixture_c0, puzzle_fixture_c1, puzzle_fixture_c2;
		geometry_msgs::Pose puzzle_fixture_c0_pose, puzzle_fixture_c1_pose, puzzle_fixture_c2_pose;
		moveit_msgs::CollisionObject puzzle_fixture_object;
		// Puzzle Fixture C0
		puzzle_fixture_c0.type = puzzle_fixture_c0.BOX;
		puzzle_fixture_c0.dimensions.resize(3);
		puzzle_fixture_c0.dimensions[0] = 0.3;
		puzzle_fixture_c0.dimensions[1] = 0.3;
		puzzle_fixture_c0.dimensions[2] = 0.01;
		// Puzzle Fixture C1
		puzzle_fixture_c1.type = puzzle_fixture_c1.BOX;
		puzzle_fixture_c1.dimensions.resize(3);
		puzzle_fixture_c1.dimensions[0] = 0.3;
		puzzle_fixture_c1.dimensions[1] = 0.01;
		puzzle_fixture_c1.dimensions[2] = 0.06;
		// Puzzle Fixture C2
		puzzle_fixture_c2.type = puzzle_fixture_c2.BOX;
		puzzle_fixture_c2.dimensions.resize(3);
		puzzle_fixture_c2.dimensions[0] = 0.01;
		puzzle_fixture_c2.dimensions[1] = 0.31;
		puzzle_fixture_c2.dimensions[2] = 0.06;
		// Puzzle fixture poses C0
		tf_tmp.setOrigin(tf::Vector3(0.15,0.15,-0.005));
		tf_tmp.setRotation(tf::Quaternion(0,0,0,1));
		tf_tmp2 = transform_PUZZLE_2_ORIGIN * tf_tmp;
		puzzle_fixture_c0_pose.position.x    = tf_tmp2.getOrigin().x();
		puzzle_fixture_c0_pose.position.y    = tf_tmp2.getOrigin().y();
		puzzle_fixture_c0_pose.position.z    = tf_tmp2.getOrigin().z();
		puzzle_fixture_c0_pose.orientation.x = tf_tmp2.getRotation().x();
		puzzle_fixture_c0_pose.orientation.y = tf_tmp2.getRotation().y();
		puzzle_fixture_c0_pose.orientation.z = tf_tmp2.getRotation().z();
		puzzle_fixture_c0_pose.orientation.w = tf_tmp2.getRotation().w();
		// Puzzle fixture poses C1
		tf_tmp.setOrigin(tf::Vector3(0.15,-0.005,0.02));
		tf_tmp2 = transform_PUZZLE_2_ORIGIN * tf_tmp;
		puzzle_fixture_c1_pose.position.x    = tf_tmp2.getOrigin().x();
		puzzle_fixture_c1_pose.position.y    = tf_tmp2.getOrigin().y();
		puzzle_fixture_c1_pose.position.z    = tf_tmp2.getOrigin().z();
		puzzle_fixture_c1_pose.orientation.x = tf_tmp2.getRotation().x();
		puzzle_fixture_c1_pose.orientation.y = tf_tmp2.getRotation().y();
		puzzle_fixture_c1_pose.orientation.z = tf_tmp2.getRotation().z();
		puzzle_fixture_c1_pose.orientation.w = tf_tmp2.getRotation().w();
		// Puzzle fixture poses C2
		tf_tmp.setOrigin(tf::Vector3(-0.005,0.145,0.02));
		tf_tmp2 = transform_PUZZLE_2_ORIGIN * tf_tmp;
		puzzle_fixture_c2_pose.position.x    = tf_tmp2.getOrigin().x();
		puzzle_fixture_c2_pose.position.y    = tf_tmp2.getOrigin().y();
		puzzle_fixture_c2_pose.position.z    = tf_tmp2.getOrigin().z();
		puzzle_fixture_c2_pose.orientation.x = tf_tmp2.getRotation().x();
		puzzle_fixture_c2_pose.orientation.y = tf_tmp2.getRotation().y();
		puzzle_fixture_c2_pose.orientation.z = tf_tmp2.getRotation().z();
		puzzle_fixture_c2_pose.orientation.w = tf_tmp2.getRotation().w();

		puzzle_fixture_object.header.frame_id = ORIGIN;
		puzzle_fixture_object.header.stamp = ros::Time(0);
		puzzle_fixture_object.id = "puzzle_fixture";
		puzzle_fixture_object.primitives.push_back(puzzle_fixture_c0);
		puzzle_fixture_object.primitives.push_back(puzzle_fixture_c1);
		puzzle_fixture_object.primitives.push_back(puzzle_fixture_c2);
		puzzle_fixture_object.primitive_poses.push_back(puzzle_fixture_c0_pose);
		puzzle_fixture_object.primitive_poses.push_back(puzzle_fixture_c1_pose);
		puzzle_fixture_object.primitive_poses.push_back(puzzle_fixture_c2_pose);

		static_scene_.world.collision_objects.push_back(puzzle_fixture_object);
		ROS_INFO("Finished adding Puzzle Fixture.");
	}



	//! Remove Octomap Artefacts at the Ground
	cleanup_octomap_srv_.request.min.x = -primitive.dimensions[0]*0.5;
	cleanup_octomap_srv_.request.min.y = -primitive.dimensions[1]*0.5;
	cleanup_octomap_srv_.request.min.z = -0.01;
	cleanup_octomap_srv_.request.max.x = primitive.dimensions[0]*0.5;
	cleanup_octomap_srv_.request.max.y = primitive.dimensions[1]*0.5;
	cleanup_octomap_srv_.request.max.z = 0.01;

	if (active_task_nr_ ==4)
	{
		tf::Quaternion q_tmp;

		//! Initialize Robot Shadow Object
		robot_shadow_primitive_.type = robot_shadow_primitive_.BOX;
		robot_shadow_primitive_.dimensions.resize(3);
		robot_shadow_primitive_.dimensions[0] = 0.15;
		robot_shadow_primitive_.dimensions[1] = 1.2;
		robot_shadow_primitive_.dimensions[2] = 1.0;
		robot_shadow_pose_.position.x = -0.55;
		robot_shadow_pose_.position.y = -0.55;
		robot_shadow_pose_.position.z = 0.5;

		q_tmp.setRPY(0,0,-0.7854);

		robot_shadow_pose_.orientation.x = q_tmp.getX();
		robot_shadow_pose_.orientation.y = q_tmp.getY();
		robot_shadow_pose_.orientation.z = q_tmp.getZ();
		robot_shadow_pose_.orientation.w = q_tmp.getW();

		robot_shadow_collision_object_.header.frame_id = ORIGIN;
		robot_shadow_collision_object_.id = "robot_shadow";
		robot_shadow_collision_object_.primitives.push_back(robot_shadow_primitive_);
		robot_shadow_collision_object_.primitive_poses.push_back(robot_shadow_pose_);
		static_scene_.world.collision_objects.push_back(robot_shadow_collision_object_);
	}
	static_scene_.is_diff = true;
	planning_scene_diff_publisher_.publish(static_scene_);
	//	static_scene_.world.collision_objects.clear();

	ROS_INFO("Finished adding ground plane and pan tilt station.");
}

void MotionPlanning::setShapePositions(int obj_index, geometry_msgs::Pose obj_pose)
{
	if (object_manager_objectExists(obj_index))
	{
		// create a structure to store the object information including all of its shapes
		ObjectInformation obj_info;
		// read the information of all shapes forming the object
		object_manager_readObjectDataFromParamServer(obj_index, obj_info);

		if(!am_collision_objects_[obj_index].obj_state_.obj_index == obj_index)
		{
			msg_error("Obj_index does not coincide with position in vector????");
			return;
		}

		// get the appropriate object
		moveit_msgs::CollisionObject* current_object = &am_collision_objects_[obj_index].collision_object_;

		// store object pose in a tf
		tf::Transform tf_object;
		tf_object.setOrigin(tf::Vector3(obj_pose.position.x,
				obj_pose.position.y,
				obj_pose.position.z));
		tf_object.setRotation(tf::Quaternion(obj_pose.orientation.x,
				obj_pose.orientation.y,
				obj_pose.orientation.z,
				obj_pose.orientation.w));

		// clear the old shape poses
		current_object->primitive_poses.clear();

		for (unsigned i = 0; i < obj_info.nr_shapes; ++i)
		{
			tf::Transform tf_shape_local, tf_shape_global;
			tf_shape_local.setOrigin(tf::Vector3(obj_info.shape_poses[i].position.x,
					obj_info.shape_poses[i].position.y,
					obj_info.shape_poses[i].position.z));
			tf_shape_local.setRotation(tf::Quaternion(obj_info.shape_poses[i].orientation.x,
					obj_info.shape_poses[i].orientation.y,
					obj_info.shape_poses[i].orientation.z,
					obj_info.shape_poses[i].orientation.w));

			tf_shape_global.mult(tf_object, tf_shape_local);

			geometry_msgs::Pose current_primitive_pose;
			current_primitive_pose.position.x = tf_shape_global.getOrigin().getX();
			current_primitive_pose.position.y = tf_shape_global.getOrigin().getY();
			current_primitive_pose.position.z = tf_shape_global.getOrigin().getZ();//+0.173;
			current_primitive_pose.orientation.x = tf_shape_global.getRotation().getX();
			current_primitive_pose.orientation.y = tf_shape_global.getRotation().getY();
			current_primitive_pose.orientation.z = tf_shape_global.getRotation().getZ();
			current_primitive_pose.orientation.w = tf_shape_global.getRotation().getW();

			current_object->primitive_poses.push_back(current_primitive_pose);

		}
	}
}

void MotionPlanning::object_manager_get_object_state_cb(const am_msgs::ObjState::ConstPtr& msg)
{

	if(active_task_nr_ == 6)
	{
		ROS_WARN("Object manager doesn't work - Task 6");
		return;
	}

	if(!obj_data_loaded_)
	{
		obj_data_loaded_=true;

		int nr_obj;
		ros::param::get("/nr_objects_",nr_obj);
		am_collision_objects_.resize(nr_obj);

		for(unsigned int ii = 0; ii<am_collision_objects_.size();ii++)
		{
			am_collision_objects_[ii].obj_state_.obj_state = OBJ_STATE_NOT_IN_WORLD;
			am_collision_objects_[ii].obj_state_.obj_index = -1;
		}
	}

	ROS_INFO("get object state cb called! ");
	switch(msg->obj_state)
	{
	case OBJ_LOCATED:
		ROS_INFO("state: OBJ_LOCATED");


		// if the current object doesn't exist yet in the environment
		if (!object_manager_objectExists(msg->obj_index))
		{
			// create one
			object_manager_createObject(msg->obj_index, msg->obj_pose);
		}
		else if(am_collision_objects_[msg->obj_index].obj_state_.obj_state == OBJ_STATE_GRABBED)
		{
			ROS_INFO("Object allready grabbed!");
			object_manager_detachObject(msg->obj_index);
		}
		else if(am_collision_objects_[msg->obj_index].obj_state_.obj_state == OBJ_STATE_IN_WORLD)
		{
			ROS_INFO("Object already in World!");
			break;
		}


		// and add it to the planning scene
		ROS_INFO("Object added to the environment.");
		object_manager_addObjectToWorld(msg->obj_index);

		break;
	case OBJ_NOT_LOCATED:
		ROS_INFO("state: OBJ_NOT_LOCATED");

		// if the current object doesn't exist yet in the environment
		if (object_manager_objectExists(msg->obj_index))
		{
			if(am_collision_objects_[msg->obj_index].obj_state_.obj_state == OBJ_STATE_GRABBED)
			{
				object_manager_detachObject(msg->obj_index);

				// update the robot state
				//                                planning_scene_.robot_state.joint_state = getCurrentJointState();
				//                                planning_scene_.robot_state.is_diff = true;
			}
			else if(am_collision_objects_[msg->obj_index].obj_state_.obj_state == OBJ_STATE_IN_WORLD)
			{
				// remove the object from the world
				object_manager_removeObjectFromWorld(msg->obj_index);
			}

			// update the robot state
			//                    planning_scene_.robot_state.joint_state = getCurrentJointState();
			//                    planning_scene_.robot_state.is_diff = true;

		}

		break;
	case OBJ_GRIPPING:
		ROS_INFO("state: OBJ GRIPPING");

		if(am_collision_objects_[msg->obj_index].obj_state_.obj_state == OBJ_STATE_IN_WORLD)
		{
			// remove the object from the world
			object_manager_removeObjectFromWorld(msg->obj_index);
		}
		else
		{
			ROS_WARN("OBJ NOT IN WORLD!");
		}

		// update the robot state
		//                planning_scene_.robot_state.joint_state = getCurrentJointState();
		//                planning_scene_.robot_state.is_diff = true;

		break;
	case OBJ_GRABED:
	{
		ROS_INFO("state: OBJ_GRABED");

		if(am_collision_objects_[msg->obj_index].obj_state_.obj_state == OBJ_STATE_NOT_IN_WORLD)
		{
			// attach the object to the gripper
			object_manager_attachObject(msg);

		}
		else
		{
			// remove the object from the world
			object_manager_removeObjectFromWorld(msg->obj_index);
			// attach the object to the gripper
			object_manager_attachObject(msg);

		}

		// update the robot state
		//              planning_scene_.robot_state.joint_state = getCurrentJointState();
		//              planning_scene_.robot_state.is_diff = true;

		break;
	}
	case OBJ_PLACED:
		ROS_INFO("state: OBJ_PLACED");

		//                setShapePositions(msg->obj_index, msg->obj_pose);
		if(am_collision_objects_[msg->obj_index].obj_state_.obj_state == OBJ_STATE_GRABBED)
		{
			// detach the object from the gripper
			object_manager_detachObject(msg->obj_index);

		}
		else
		{
			ROS_WARN("OBJECT IN WORLD?/OBJECT GRABBED?");
		}


		// update the robot state
		//                planning_scene_.robot_state.joint_state = getCurrentJointState();
		//                planning_scene_.robot_state.is_diff = true;

		break;
	case OBJ_FINISHED:
		ROS_INFO("state: OBJ_FINISHED");

		if(am_collision_objects_[msg->obj_index].obj_state_.obj_state == OBJ_STATE_NOT_IN_WORLD)
		{
			// add the object to its target zone
			object_manager_addObjectToTargetZone(msg->obj_index);
			planning_scene_.robot_state.is_diff = true;
		}
		else if (am_collision_objects_[msg->obj_index].obj_state_.obj_state == OBJ_STATE_GRABBED)
		{
			ROS_WARN("OBJ STATE GRABBED");
			// detach the object from the gripper
			object_manager_detachObject(msg->obj_index);
			// add the object to its target zone
			object_manager_addObjectToTargetZone(msg->obj_index);
		}

		else if (am_collision_objects_[msg->obj_index].obj_state_.obj_state == OBJ_STATE_IN_WORLD)
		{
			ROS_WARN("OBJ STATE IN WORLD");
			// detach the object from the gripper
			object_manager_removeObjectFromWorld(msg->obj_index);
			// add the object to its target zone
			object_manager_addObjectToTargetZone(msg->obj_index);
		}

		// update the robot state
		//planning_scene_.robot_state.joint_state = getCurrentJointState();
		//planning_scene_.robot_state.is_diff = true;

		break;
	case OBJ_REMOVE_SHADOW:
		if (active_task_nr_ == 4)
		{
			static_scene_.world.collision_objects.clear();
			robot_shadow_collision_object_.operation = robot_shadow_collision_object_.REMOVE;
			static_scene_.world.collision_objects.push_back(robot_shadow_collision_object_);
			planning_scene_diff_publisher_.publish(static_scene_);
		}
		break;
	default:
		msg_error("Unknown object state (msg->obj_state=%d)!!!",msg->obj_state);
		break;
	}

	planning_scene_.robot_state.is_diff = true;
	planning_scene_.robot_state.joint_state = getCurrentJointState();
	planning_scene_.robot_state.is_diff = true;
	// publish the planning scene
	planning_scene_.is_diff = true;
	planning_scene_diff_publisher_.publish(planning_scene_);

	planning_scene_.world.collision_objects.clear();
	planning_scene_.robot_state.attached_collision_objects.clear();
}


bool MotionPlanning::object_manager_objectExists(int obj_index)
{
	ROS_WARN("Checking if object already exists...");
	ROS_INFO_STREAM("Index of the object to search: " << obj_index);

	if (!am_collision_objects_.empty())
	{
		for (unsigned i = 0; i < am_collision_objects_.size(); ++i)
		{
			ROS_INFO_STREAM("Index of the current collision object it is compared with: " << am_collision_objects_[i].collision_object_.id);
			if (obj_index == am_collision_objects_[i].obj_state_.obj_index)
			{
				ROS_WARN("Object already exists in the environment.");
				return true;
			}
		}
		return false;
	}
	ROS_WARN("Object does not exist in the environment, yet.");
	return false;
}

void MotionPlanning::object_manager_createObject(int obj_index, geometry_msgs::Pose obj_pose)
{
	ROS_WARN("Creating an object.");
	// create a structure to store the object information including all of its shapes
	ObjectInformation obj_info;
	// read the information of all shapes forming the object
	object_manager_readObjectDataFromParamServer(obj_index, obj_info);

	// create a collision object from the object information
	moveit_msgs::CollisionObject collision_object;

	std::stringstream id;
	id << obj_index;
	collision_object.id = id.str();

	// add all shapes
	for (unsigned i = 0; i < obj_info.nr_shapes; ++i)
	{
		shape_msgs::SolidPrimitive primitive;
		switch (obj_info.shape_types[i])
		{
		case SHAPE_BOX:
			ROS_WARN("Shape type: box");
			primitive.type = primitive.BOX;
			primitive.dimensions.resize(3);
			primitive.dimensions[0] = obj_info.shape_sizes[i].x;
			ROS_INFO_STREAM("x dimension: " << primitive.dimensions[0]);
			primitive.dimensions[1] = obj_info.shape_sizes[i].y;
			ROS_INFO_STREAM("y dimension: " << primitive.dimensions[1]);
			primitive.dimensions[2] = obj_info.shape_sizes[i].z;
			ROS_INFO_STREAM("x dimension: " << primitive.dimensions[2]);
			break;
		case SHAPE_CYLINDER:
			ROS_WARN("Shape type: cylinder");
			primitive.type = primitive.CYLINDER;
			primitive.dimensions.resize(2);
			primitive.dimensions[0] = obj_info.shape_lengths[i];  // cylinder height
			ROS_INFO_STREAM("height: " << primitive.dimensions[0]);
			primitive.dimensions[1] = obj_info.shape_radii[i];    // cylinder radius
			ROS_INFO_STREAM("radius: " << primitive.dimensions[1]);
			break;
		default:
			ROS_WARN("False shape type.");
			break;
		}
		collision_object.primitives.push_back(primitive);

		//		tf::Transform tf_object, tf_shape_local, tf_shape_global;
		//		tf_object.setOrigin(tf::Vector3(obj_pose.position.x,
		//				obj_pose.position.y,
		//				obj_pose.position.z));
		//		tf_object.setRotation(tf::Quaternion(obj_pose.orientation.x,
		//				obj_pose.orientation.y,
		//				obj_pose.orientation.z,
		//				obj_pose.orientation.w));
		//
		//		tf_shape_local.setOrigin(tf::Vector3(obj_info.shape_poses[i].position.x,
		//				obj_info.shape_poses[i].position.y,
		//				obj_info.shape_poses[i].position.z));
		//		tf_shape_local.setRotation(tf::Quaternion(obj_info.shape_poses[i].orientation.x,
		//				obj_info.shape_poses[i].orientation.y,
		//				obj_info.shape_poses[i].orientation.z,
		//				obj_info.shape_poses[i].orientation.w));
		//
		//		tf_shape_global.mult(tf_object, tf_shape_local);
		//
		//		geometry_msgs::Pose primitive_pose;
		//		primitive_pose.position.x = tf_shape_global.getOrigin().getX();
		//		primitive_pose.position.y = tf_shape_global.getOrigin().getY();
		//		primitive_pose.position.z = tf_shape_global.getOrigin().getZ();
		//		primitive_pose.orientation.x = tf_shape_global.getRotation().getX();
		//		primitive_pose.orientation.y = tf_shape_global.getRotation().getY();
		//		primitive_pose.orientation.z = tf_shape_global.getRotation().getZ();
		//		primitive_pose.orientation.w = tf_shape_global.getRotation().getW();
		geometry_msgs::Pose primitive_pose;
		primitive_pose	= obj_info.shape_poses[i];

		collision_object.primitive_poses.push_back(primitive_pose);

		collision_object.header.frame_id = "/Origin";
		collision_object.header.stamp = ros::Time::now();

	}

	if(am_collision_objects_[obj_index].obj_state_.obj_index == -1)
	{
		am_collision_objects_[obj_index].collision_object_ = collision_object;
		am_collision_objects_[obj_index].obj_state_.obj_index = obj_index;
		am_collision_objects_[obj_index].obj_state_.obj_pose = obj_pose;
		am_collision_objects_[obj_index].obj_state_.obj_state = OBJ_STATE_NOT_IN_WORLD;
	}
	else
	{
		ROS_WARN("Object allready exists?");
	}

	ROS_WARN("Creating object finished.");
}

void MotionPlanning::object_manager_readObjectDataFromParamServer(int obj_index, ObjectInformation& obj_info)
{
	ROS_INFO("Reading object data from the parameter server...");

	// get number of shapes of the current object
	std::stringstream _number_shapes;
	_number_shapes << "/object_";//_number_shapes.str("/object_");
	_number_shapes << obj_index;
	_number_shapes << "_nr_shapes_";
	ros::param::get(_number_shapes.str(), obj_info.nr_shapes);

	// store the information of each shape
	for (unsigned i = 0; i < obj_info.nr_shapes; ++i)
	{
		// shape type
		int shape_type;
		std::stringstream _shape_type;
		_shape_type << "/object_";//.str("/object_");
		_shape_type << obj_index;
		_shape_type << "_shape_";
		_shape_type << i;
		_shape_type << "_type_";
		ros::param::get(_shape_type.str(), shape_type);
		obj_info.shape_types.push_back((uint8_t)shape_type);



		geometry_msgs::Pose shape_pose;
		// shape pose position x
		std::stringstream _shape_pose_position_x;
		_shape_pose_position_x << "/object_";//.str("/object_");
		_shape_pose_position_x << obj_index;
		_shape_pose_position_x << "_shape_";
		_shape_pose_position_x << i;
		_shape_pose_position_x << "_pose_position_x_";
		ros::param::get(_shape_pose_position_x.str(), shape_pose.position.x);
		// shape pose position y
		std::stringstream _shape_pose_position_y;
		_shape_pose_position_y << "/object_";//.str("/object_");
		_shape_pose_position_y << obj_index;
		_shape_pose_position_y << "_shape_";
		_shape_pose_position_y << i;
		_shape_pose_position_y << "_pose_position_y_";
		ros::param::get(_shape_pose_position_y.str(), shape_pose.position.y);
		// shape pose position z
		std::stringstream _shape_pose_position_z;
		_shape_pose_position_z << "/object_";//.str("/object_");
		_shape_pose_position_z << obj_index;
		_shape_pose_position_z << "_shape_";
		_shape_pose_position_z << i;
		_shape_pose_position_z << "_pose_position_z_";
		ros::param::get(_shape_pose_position_z.str(), shape_pose.position.z);
		// shape pose orientation w
		std::stringstream _shape_pose_orientation_w;
		_shape_pose_orientation_w << "/object_";//.str("/object_");
		_shape_pose_orientation_w << obj_index;
		_shape_pose_orientation_w << "_shape_";
		_shape_pose_orientation_w << i;
		_shape_pose_orientation_w << "_pose_orientation_w_";
		ros::param::get(_shape_pose_orientation_w.str(), shape_pose.orientation.w);
		// shape pose orientation x
		std::stringstream _shape_pose_orientation_x;
		_shape_pose_orientation_x << "/object_";//.str("/object_");
		_shape_pose_orientation_x << obj_index;
		_shape_pose_orientation_x << "_shape_";
		_shape_pose_orientation_x << i;
		_shape_pose_orientation_x << "_pose_orientation_x_";
		ros::param::get(_shape_pose_orientation_x.str(), shape_pose.orientation.x);
		// shape pose orientation y
		std::stringstream _shape_pose_orientation_y;
		_shape_pose_orientation_y << "/object_";//.str("/object_");
		_shape_pose_orientation_y << obj_index;
		_shape_pose_orientation_y << "_shape_";
		_shape_pose_orientation_y << i;
		_shape_pose_orientation_y << "_pose_orientation_y_";
		ros::param::get(_shape_pose_orientation_y.str(), shape_pose.orientation.y);
		// shape pose orientation z
		std::stringstream _shape_pose_orientation_z;
		_shape_pose_orientation_z << "/object_";//.str("/object_");
		_shape_pose_orientation_z << obj_index;
		_shape_pose_orientation_z << "_shape_";
		_shape_pose_orientation_z << i;
		_shape_pose_orientation_z << "_pose_orientation_z_";
		ros::param::get(_shape_pose_orientation_z.str(), shape_pose.orientation.z);
		obj_info.shape_poses.push_back(shape_pose);


		switch (shape_type)
		{
		case SHAPE_BOX:
		{
			geometry_msgs::Vector3 shape_size;
			std::stringstream _shape_size_0;
			_shape_size_0 << "/object_";//.str("/object_");
			_shape_size_0 << obj_index;
			_shape_size_0 << "_shape_";
			_shape_size_0 << i;
			_shape_size_0 << "_size_0_";
			ros::param::get(_shape_size_0.str(), shape_size.x);
			std::stringstream _shape_size_1;
			_shape_size_1 << "/object_";//.str("/object_");
			_shape_size_1 << obj_index;
			_shape_size_1 << "_shape_";
			_shape_size_1 << i;
			_shape_size_1 << "_size_1_";
			ros::param::get(_shape_size_1.str(), shape_size.y);
			std::stringstream _shape_size_2;
			_shape_size_2 << "/object_";//.str("/object_");
			_shape_size_2 << obj_index;
			_shape_size_2 << "_shape_";
			_shape_size_2 << i;
			_shape_size_2 << "_size_2_";
			ros::param::get(_shape_size_2.str(), shape_size.z);
			// scale the dimensions of the object
			shape_size.x *= collision_object_scaler_;
			shape_size.y *= collision_object_scaler_;
			shape_size.z *= collision_object_scaler_;
			obj_info.shape_sizes.push_back(shape_size);


			obj_info.shape_lengths.push_back(0.0);
			obj_info.shape_radii.push_back(0.0);


			break;
		}
		case SHAPE_CYLINDER:
		{
			geometry_msgs::Vector3 shape_size;
			shape_size.x = 0;
			shape_size.y = 0;
			shape_size.z = 0;
			obj_info.shape_sizes.push_back(shape_size);

			double shape_length;
			std::stringstream _shape_length;
			_shape_length << "/object_";//.str("/object_");
			_shape_length << obj_index;
			_shape_length << "_shape_";
			_shape_length << i;
			_shape_length << "_length_";
			ros::param::get(_shape_length.str(), shape_length);
			// scale the dimensions of the object
			shape_length *= collision_object_scaler_;
			obj_info.shape_lengths.push_back(shape_length);

			double shape_radius;
			std::stringstream _shape_radius;
			_shape_radius << "/object_";//.str("/object_");
			_shape_radius << obj_index;
			_shape_radius << "_shape_";
			_shape_radius << i;
			_shape_radius << "_radius_";
			ros::param::get(_shape_radius.str(), shape_radius);
			// scale the dimensions of the object
			shape_radius *= collision_object_scaler_;
			obj_info.shape_radii.push_back(shape_radius);

			break;
		}
		}
	}
	ROS_INFO("Finished reading object data from the parameter server.");
}

void MotionPlanning::object_manager_addObjectToWorld(int obj_index)
{
	if (object_manager_objectExists(obj_index))
	{
		ROS_INFO("Adding object to the environment...");
		bool any_nan = false;

		if(!am_collision_objects_[obj_index].obj_state_.obj_index == obj_index)
		{
			msg_error("Obj_index does not coincide with position in vector????");
			return;
		}
		if (pose_check_isnan(&am_collision_objects_[obj_index].obj_state_.obj_pose))
		{
			msg_warn("Pose information with NaN detected.");
			any_nan = true;
		}

		if (!any_nan)
		{
			tf::Transform tf_tmp, tf_tmp_primitive;

			tf_tmp.setOrigin(tf::Vector3(am_collision_objects_[obj_index].obj_state_.obj_pose.position.x,am_collision_objects_[obj_index].obj_state_.obj_pose.position.y,am_collision_objects_[obj_index].obj_state_.obj_pose.position.z));
			tf_tmp.setRotation(tf::Quaternion(am_collision_objects_[obj_index].obj_state_.obj_pose.orientation.x,am_collision_objects_[obj_index].obj_state_.obj_pose.orientation.y,am_collision_objects_[obj_index].obj_state_.obj_pose.orientation.z,am_collision_objects_[obj_index].obj_state_.obj_pose.orientation.w));

			moveit_msgs::CollisionObject current_object = am_collision_objects_[obj_index].collision_object_;

			for (unsigned ii=0;ii<current_object.primitive_poses.size();ii++)
			{

				if (pose_check_isnan(&am_collision_objects_[obj_index].collision_object_.primitive_poses[ii]))
				{
					msg_warn("Pose information with NaN detected.");
					any_nan = true;
					break;
				}
				else
				{
					tf_tmp_primitive.setOrigin(tf::Vector3(am_collision_objects_[obj_index].collision_object_.primitive_poses[ii].position.x,am_collision_objects_[obj_index].collision_object_.primitive_poses[ii].position.y,am_collision_objects_[obj_index].collision_object_.primitive_poses[ii].position.z));
					tf_tmp_primitive.setRotation(tf::Quaternion(am_collision_objects_[obj_index].collision_object_.primitive_poses[ii].orientation.x,am_collision_objects_[obj_index].collision_object_.primitive_poses[ii].orientation.y,am_collision_objects_[obj_index].collision_object_.primitive_poses[ii].orientation.z,am_collision_objects_[obj_index].collision_object_.primitive_poses[ii].orientation.w));
					tf_tmp_primitive.mult(tf_tmp,tf_tmp_primitive);

					current_object.primitive_poses[ii].position.x = tf_tmp_primitive.getOrigin().x();
					current_object.primitive_poses[ii].position.y = tf_tmp_primitive.getOrigin().y();
					current_object.primitive_poses[ii].position.z = tf_tmp_primitive.getOrigin().z();
					current_object.primitive_poses[ii].orientation.x = tf_tmp_primitive.getRotation().x();
					current_object.primitive_poses[ii].orientation.y = tf_tmp_primitive.getRotation().y();
					current_object.primitive_poses[ii].orientation.z = tf_tmp_primitive.getRotation().z();
					current_object.primitive_poses[ii].orientation.w = tf_tmp_primitive.getRotation().w();
				}

			}

			// get the appropriate object

			// add object to the environment
			planning_scene_.world.collision_objects.clear();
			current_object.header.frame_id = "/Origin";
			current_object.header.stamp = ros::Time::now();
			current_object.operation = current_object.ADD;
			planning_scene_.world.collision_objects.push_back(current_object);

			ROS_INFO("Adding object to the environment finished.");


			am_collision_objects_[obj_index].obj_state_.obj_state = OBJ_STATE_IN_WORLD;
		}
	}
	else
		ROS_WARN("Object cannot be added to the environment. It must be created first.");

}

void MotionPlanning::object_manager_removeObjectFromWorld(int obj_index)
{
	if (object_manager_objectExists(obj_index))
	{
		ROS_INFO("Removing object from the environment...");
		bool any_nan=false;

		if(!am_collision_objects_[obj_index].obj_state_.obj_index == obj_index)
		{
			msg_error("Obj_index does not coincide with position in vector????");
			return;
		}
		for (unsigned ii=0;ii<am_collision_objects_[obj_index].collision_object_.primitive_poses.size();ii++)
		{
			if (pose_check_isnan(&am_collision_objects_[obj_index].collision_object_.primitive_poses[ii]))
			{
				msg_warn("Pose information with NaN detected.");
				any_nan = true;
				break;
			}
		}

		if (!any_nan)
		{
			// get the appropriate object
			moveit_msgs::CollisionObject current_object = am_collision_objects_[obj_index].collision_object_;

			// remove object from the environment
			planning_scene_.world.collision_objects.clear();
			moveit_msgs::CollisionObject remove_object;
			remove_object.id = current_object.id;
			remove_object.header.frame_id = "/Origin";
			remove_object.header.stamp = ros::Time::now();
			remove_object.operation = remove_object.REMOVE;
			planning_scene_.world.collision_objects.push_back(remove_object);

			ROS_INFO("Removing object from the environment finished.");

			am_collision_objects_[obj_index].obj_state_.obj_state = OBJ_STATE_NOT_IN_WORLD;
		}
	}
	else
		ROS_WARN("Object cannot be removed from the world. It must be created first.");


}

void MotionPlanning::object_manager_attachObject(const am_msgs::ObjState::ConstPtr& msg)
{
	if (object_manager_objectExists(msg->obj_index))
	{
		ROS_INFO("Attaching object to the gripper...");
		bool any_nan=false;

		if(!am_collision_objects_[msg->obj_index].obj_state_.obj_index == msg->obj_index)
		{
			msg_error("Obj_index does not coincide with position in vector????");
			return;
		}

		// get the appropriate object
		tf::Transform tf_tmp, tf_tmp_primitive;
		moveit_msgs::CollisionObject current_object = am_collision_objects_[msg->obj_index].collision_object_;

		tf_tmp.setOrigin(tf::Vector3(msg->obj_pose.position.x,msg->obj_pose.position.y,msg->obj_pose.position.z));
		tf_tmp.setRotation(tf::Quaternion(msg->obj_pose.orientation.x,msg->obj_pose.orientation.y,msg->obj_pose.orientation.z,msg->obj_pose.orientation.w));

		if (pose_check_isnan(&am_collision_objects_[msg->obj_index].obj_state_.obj_pose))
		{
			msg_warn("Pose information with NaN detected.");
			any_nan = true;
		}


		if (!any_nan)
		{
			for (unsigned ii=0;ii<current_object.primitive_poses.size();ii++)
			{
				tf_tmp_primitive.setOrigin(tf::Vector3(am_collision_objects_[msg->obj_index].collision_object_.primitive_poses[ii].position.x,am_collision_objects_[msg->obj_index].collision_object_.primitive_poses[ii].position.y,am_collision_objects_[msg->obj_index].collision_object_.primitive_poses[ii].position.z));
				tf_tmp_primitive.setRotation(tf::Quaternion(am_collision_objects_[msg->obj_index].collision_object_.primitive_poses[ii].orientation.x,am_collision_objects_[msg->obj_index].collision_object_.primitive_poses[ii].orientation.y,am_collision_objects_[msg->obj_index].collision_object_.primitive_poses[ii].orientation.z,am_collision_objects_[msg->obj_index].collision_object_.primitive_poses[ii].orientation.w));
				tf_tmp_primitive.mult(tf_tmp,tf_tmp_primitive);

				current_object.primitive_poses[ii].position.x = tf_tmp_primitive.getOrigin().x();
				current_object.primitive_poses[ii].position.y = tf_tmp_primitive.getOrigin().y();
				current_object.primitive_poses[ii].position.z = tf_tmp_primitive.getOrigin().z();
				current_object.primitive_poses[ii].orientation.x = tf_tmp_primitive.getRotation().x();
				current_object.primitive_poses[ii].orientation.y = tf_tmp_primitive.getRotation().y();
				current_object.primitive_poses[ii].orientation.z = tf_tmp_primitive.getRotation().z();
				current_object.primitive_poses[ii].orientation.w = tf_tmp_primitive.getRotation().w();

			}
			// Load current robot state into planning scene
			planning_scene_.robot_state.joint_state = getCurrentJointState();
			planning_scene_.robot_state.is_diff = true;

			// attach object to the gripper
			planning_scene_.robot_state.attached_collision_objects.clear();
			moveit_msgs::AttachedCollisionObject attached_object;
			attached_object.link_name = "link7";//link7";//"gripper_tcp";
			attached_object.object = current_object;
			attached_object.object.operation = attached_object.object.ADD;
			planning_scene_.robot_state.attached_collision_objects.push_back(attached_object);

			am_collision_objects_[msg->obj_index].obj_state_.obj_state = OBJ_STATE_GRABBED;

			ROS_INFO("Attaching object to the gripper finished.");
			// Debugging
			planning_scene_diff_publisher_.publish(planning_scene_);
		}
	}
	else
		ROS_WARN("Object cannot be attached to the gripper. It must be created first.");


}

void MotionPlanning::object_manager_detachObject(int obj_index)
{
	if (object_manager_objectExists(obj_index))
	{
		ROS_INFO("Detaching object from the gripper...");
		bool any_nan=false;

		if(!am_collision_objects_[obj_index].obj_state_.obj_index == obj_index)
		{
			msg_error("Obj_index does not coincide with position in vector????");
			return;
		}

		for (unsigned ii=0;ii<am_collision_objects_[obj_index].collision_object_.primitive_poses.size();ii++)
		{
			if (pose_check_isnan(&am_collision_objects_[obj_index].collision_object_.primitive_poses[ii]))
			{
				msg_warn("Pose information with NaN detected.");
				any_nan = true;
				break;
			}
		}

		if (!any_nan)
		{
			// get the appropriate object
			moveit_msgs::CollisionObject current_object = am_collision_objects_[obj_index].collision_object_;

			// detach object from the gripper
			planning_scene_.robot_state.attached_collision_objects.clear();
			moveit_msgs::AttachedCollisionObject detached_object;
			detached_object.object.id = current_object.id;
			detached_object.object.operation = detached_object.object.REMOVE;
			planning_scene_.robot_state.attached_collision_objects.push_back(detached_object);

			am_collision_objects_[obj_index].obj_state_.obj_state = OBJ_STATE_NOT_IN_WORLD;
			ROS_INFO("Detaching object from the gripper finished.");
		}
	}
	else
		ROS_WARN("Object cannot be detached from the gripper. It must be created first.");

}


void MotionPlanning::object_manager_addObjectToTargetZone(int obj_index)
{
	if (object_manager_objectExists(obj_index))
	{
		ROS_INFO("Adding object to the target zone...");

		// create a structure to store the object information including all of its shapes
		ObjectInformation obj_info;
		// read the information of all shapes forming the object
		object_manager_readObjectDataFromParamServer(obj_index, obj_info);


		if(!am_collision_objects_[obj_index].obj_state_.obj_index == obj_index)
		{
			msg_error("Obj_index does not coincide with position in vector????");
			return;
		}

		// get the appropriate object
		moveit_msgs::CollisionObject current_object = am_collision_objects_[obj_index].collision_object_;

		// reinsert object into the environment
		planning_scene_.world.collision_objects.clear();
		moveit_msgs::CollisionObject placed_object;
		placed_object.id = current_object.id;
		placed_object.header.frame_id = "/Origin";
		//                placed_object.header.stamp = ros::Time::now();


		shape_msgs::SolidPrimitive placed_shape;
		placed_shape.type = placed_shape.CYLINDER;
		placed_shape.dimensions.resize(2);
		placed_shape.dimensions[0] = getTargetObjectHeight(obj_index);
		ROS_INFO_STREAM("Target cylinder height: " << placed_shape.dimensions[0]);
		placed_shape.dimensions[1] = getTargetObjectRadius(obj_index);
		ROS_INFO_STREAM("Target cylinder radius: " << placed_shape.dimensions[1]);
		placed_object.primitives.push_back(placed_shape);

		placed_object.primitive_poses.push_back(getTargetObjectPose(obj_index));
		placed_object.operation = placed_object.ADD;
		planning_scene_.world.collision_objects.push_back(placed_object);

		am_collision_objects_[obj_index].obj_state_.obj_state = OBJ_STATE_IN_WORLD;
		ROS_INFO("Adding object to the target zone finished.");

	}
	else
		ROS_WARN("Object cannot be added to the target zone. It must be created first.");

}

double MotionPlanning::getTargetObjectHeight(int obj_index)
{
	double h = 0;

	// create a structure to store the object information including all of its shapes
	ObjectInformation obj_info;
	// read the information of all shapes forming the object
	object_manager_readObjectDataFromParamServer(obj_index, obj_info);

	for (unsigned i = 0; i < obj_info.nr_shapes; ++i)
	{
		if (obj_info.shape_types[i] == SHAPE_BOX)
			h += obj_info.shape_sizes[i].z;
		else if (obj_info.shape_types[i] == SHAPE_CYLINDER)
			h += obj_info.shape_lengths[i];
	}

	return h;
}

double MotionPlanning::getTargetObjectRadius(int obj_index)
{
	double r = 0;

	// create a structure to store the object information including all of its shapes
	ObjectInformation obj_info;
	// read the information of all shapes forming the object
	object_manager_readObjectDataFromParamServer(obj_index, obj_info);

	for (unsigned i = 0; i < obj_info.nr_shapes; ++i)
	{
		if (obj_info.shape_types[i] == SHAPE_BOX)
		{
			if (obj_info.shape_sizes[i].x > r)
				r = obj_info.shape_sizes[i].x;
			if (obj_info.shape_sizes[i].y > r)
				r = obj_info.shape_sizes[i].y;
		}
		else if (obj_info.shape_types[i] == SHAPE_CYLINDER)
		{
			if (obj_info.shape_radii[i] > r)
				r = obj_info.shape_radii[i];
		}
	}

	return r;
}

geometry_msgs::Pose MotionPlanning::getTargetObjectPose(int obj_index)
{
	geometry_msgs::Pose target_object_pose;

	// create a structure to store the target zone information
	TargetZoneInformation tz_info;
	// read the target zone information from the parameter server
	readTargetZoneDataFromParamServer(obj_index, tz_info);

	double object_height = getTargetObjectHeight(obj_index);

	target_object_pose.position.x = tz_info.x;
	target_object_pose.position.y = tz_info.y;
	target_object_pose.position.z = object_height/2.0;

	return target_object_pose;
}

void MotionPlanning::readTargetZoneDataFromParamServer(int obj_index, TargetZoneInformation& tz_info)
{
	ROS_INFO("Reading target zone data from the parameter server...");

	std::stringstream obj_nr;
	obj_nr << obj_index;

	for (unsigned tz_index = 0; tz_index < am_collision_objects_.size(); ++tz_index)
	{
		std::stringstream _tz_obj_nr;
		_tz_obj_nr << "/target_zone_" << tz_index << "_obj_nr_";
		ros::param::get(_tz_obj_nr.str(), tz_info.obj_nr);

		std::stringstream tz_nr;
		tz_nr << tz_info.obj_nr;

		if (!tz_nr.str().compare(obj_nr.str()))
		{
			std::stringstream _tz_x;
			_tz_x << "/target_zone_" << tz_index << "_x_";
			ros::param::get(_tz_x.str(), tz_info.x);

			std::stringstream _tz_y;
			_tz_y << "/target_zone_" << tz_index << "_y_";
			ros::param::get(_tz_y.str(), tz_info.y);

			std::stringstream _tz_r;
			_tz_r << "/target_zone_" << tz_index << "_radius_";
			ros::param::get(_tz_r.str(), tz_info.r);

			ROS_INFO("Reading target zone data finished.");
			break;
		}
	}
}

sensor_msgs::JointState MotionPlanning::getCurrentJointState()
{
	sensor_msgs::JointState currentState;

	currentState.header.frame_id = "/Origin";
	currentState.name = group->getActiveJoints();
	currentState.position.resize(currentState.name.size());
	currentState.velocity.resize(currentState.name.size());

	if (getTelemetry())
	{

		for (unsigned idx = 0; idx < currentState.name.size(); ++idx)
		{
			for (unsigned idxTele = 0; idxTele < _telemetry.joint_names.size(); ++idxTele)
			{
				if (!currentState.name[idx].compare(_telemetry.joint_names[idxTele]))
				{
					currentState.position[idx] = _telemetry.measured.position[idxTele];
					currentState.velocity[idx] = 0.0;
					break;
				}
			}

		}
	}
	else
		ROS_ERROR("Failed getting the telemetry.");

	return currentState;
}

bool MotionPlanning::pose_check_isnan(geometry_msgs::Pose* msg_ptr)
{
	if (isnan(msg_ptr->position.x))
		return true;
	else if (isnan(msg_ptr->position.y))
		return true;
	else if (isnan(msg_ptr->position.z))
		return true;
	else if (isnan(msg_ptr->orientation.x))
		return true;
	else if (isnan(msg_ptr->orientation.y))
		return true;
	else if (isnan(msg_ptr->orientation.z))
		return true;
	else if (isnan(msg_ptr->orientation.w))
		return true;
	else
		return false;
}

void MotionPlanning::swing_in_motion()
{
	move_along_joint_path_srv_.request.joint_names.clear();
	move_along_joint_path_srv_.request.joint_names.resize(2);
	move_along_joint_path_srv_.request.joint_names[0] = "axis_x";
	move_along_joint_path_srv_.request.joint_names[1] = "axis_y";

	move_along_joint_path_srv_.request.joint_limits.resize(2);
	move_along_joint_path_srv_.request.joint_limits[0].max_acceleration = 0.5*table_axis1_limit_.max_acceleration;
	move_along_joint_path_srv_.request.joint_limits[1].max_acceleration = 0.5*table_axis2_limit_.max_acceleration;
	move_along_joint_path_srv_.request.joint_limits[0].max_velocity = 0.2*table_axis1_limit_.max_velocity;
	move_along_joint_path_srv_.request.joint_limits[1].max_velocity = 0.2*table_axis2_limit_.max_velocity;
	move_along_joint_path_srv_.request.path.clear();
	//	move_along_joint_path_srv_.request.path.resize(10);

	tf::Vector3 x_0,x_1,x_start,tmp_vect,tmp_vec_res;
	tf::TransformListener tf_listener;
	tf::StampedTransform current_GP_TCP;
	tf::Matrix3x3 dcm;
	std::vector<double> idx;

	double num_idx, freq_swing, a_swing;
	num_idx=50;
	freq_swing = 20;
	a_swing = 0.02;
	euroc_c2_msgs::Configuration tmp_cfg;
	tmp_cfg.q.resize(2);

	double alpha;
	//! Transformation from GP TCP frame to LWR TCP frame
	try{
		if (tf_listener.waitForTransform(ORIGIN,GP_TCP,ros::Time(0),ros::Duration(1.0),ros::Duration(3.0)))
			tf_listener.lookupTransform(ORIGIN,GP_TCP,ros::Time(0),current_GP_TCP);
		else{
			msg_error("Could not get LWRTCP GPTCP tf.");
		}
	}
	catch(...){
		ROS_ERROR("Listening to transform was not successful");
	}
	double x,y,z;
	x = _telemetry.measured.position[0];
	y = _telemetry.measured.position[1];

	x_start.setValue(x,y,0.0);
	x_0 = current_GP_TCP.getOrigin();
	x_0.setZ(0.0);
	x_1.setValue(goal_pose_GPTCP_.position.x, goal_pose_GPTCP_.position.y, 0.0);

	double push_length;
	push_length = sqrt(pow(x_1.y()-x_0.y(),2)+pow(x_1.x()-x_0.x(),2));

	for (int i=0;i<(num_idx+1);i++)
		idx.push_back((double)i*(push_length/num_idx));

	alpha = atan2(x_1.y()-x_0.y(),x_1.x()-x_0.x());
	dcm.setRPY(0,0,alpha);
	for (int i=0;i<(num_idx+1);i++)
	{
		tmp_vect.setValue(idx[i],a_swing*sin(freq_swing*(double)idx[i]),0.0);
		tmp_vec_res = dcm*tmp_vect + x_start;
		tmp_cfg.q[0] = tmp_vec_res.x();
		tmp_cfg.q[1] = tmp_vec_res.y();
		ROS_WARN("idx %f q %f %f",idx[i],tmp_cfg.q[0],tmp_cfg.q[1]);
		move_along_joint_path_srv_.request.path.push_back(tmp_cfg);
	}
	ROS_WARN("x_start %f %f %f",x_start.x(),x_start.y(),x_start.z());
	ROS_WARN("x_0 %f %f %f",x_0.x(),x_0.y(),x_0.z());
	ROS_WARN("x_1 %f %f %f",x_1.x(),x_1.y(),x_1.z());
}
