/*
 * MotionPlanning.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: euroc_admin
 */

#include "MotionPlanning.h"

MotionPlanning::MotionPlanning():
goalPose_server_(nh_, "goalPoseAction", boost::bind(&MotionPlanning::executeGoalPose_CB, this, _1),false),
goalPose_action_name_("goalPoseAction"),
time_at_path_points_(1)
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
	get_static_tf_data_client_      = nh_.serviceClient<am_msgs::GetStaticTFData>("get_static_tf_data");

	joint_limits_.resize(7);

	goalPose_server_.start();
	msg_info("goalPose action server started.");


	check_poses_service_ = nh_.advertiseService("CheckPoses_srv", &MotionPlanning::return_poses_valid,this);

	feedback_frequency_ = 2;

	mtt_=OPEN;
	called = false;


#ifdef MOVEIT

	// arm group
	group_7DOF = new move_group_interface::MoveGroup("LWR_7DOF");
	// arm + table axes
	group_9DOF = new move_group_interface::MoveGroup("LWR");
//	group_9DOF = new move_group_interface::MoveGroup("LWR_9DOF");

	// planning algorithm for arm group
	group_7DOF->setPlannerId("RRTConnectkConfigDefault");
	// planning algorithm for entire group
    group_9DOF->setPlannerId("RRTConnectkConfigDefault");


	// planning algorithm for entire group
//        group->setPlannerId("SBLkConfigDefault");
//        group->setPlannerId("ESTkConfigDefault");
//		  group->setPlannerId("LBKPIECEkConfigDefault");  // default
//        group->setPlannerId("BKPIECEkConfigDefault");
//        group->setPlannerId("KPIECEkConfigDefault");
//        group->setPlannerId("RRTkConfigDefault");
//        group->setPlannerId("RRTConnectkConfigDefault");
//        group->setPlannerId("RRTstarkConfigDefault");
//        group->setPlannerId("TRRTkConfigDefault");
//        group->setPlannerId("PRMkConfigDefault");
//        group->setPlannerId("PRMstarkConfigDefault");


    planning_scene_monitor = new planning_scene_monitor::PlanningSceneMonitor("robot_description");

    attach_object_service_ = nh_.advertiseService("AttachObject_srv", &MotionPlanning::return_object_attached, this);
    detach_object_service_ = nh_.advertiseService("DetachObject_srv", &MotionPlanning::return_object_detached, this);

#endif
}

MotionPlanning::~MotionPlanning() 
{
#ifdef MOVEIT
	delete planning_scene_monitor;
	delete group_9DOF;
	delete group_7DOF;
#endif
}

void MotionPlanning::executeGoalPose_CB(const am_msgs::goalPoseGoal::ConstPtr &goal)
{
	goal_pose_goal_ = goal;
	speed_percentage_ = goal_pose_goal_->speed_percentage;
	inter_steps_ = goal_pose_goal_->inter_steps;
	planning_frame_ = goal_pose_goal_->planning_frame;
	ROS_WARN("Chosen Planning Frame :%s",planning_frame_.c_str());

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
	else if (goal_pose_goal_->planning_algorithm!=HOMING_7DOF)
		msg_error("Planning frame not properly defined.");


	//! Set default speed percentage values for motion velocity
	if (speed_percentage_ <= 0 || speed_percentage_ >100)
		speed_percentage_ = 40;

	getLimits();

	ros::Rate feedback_rate(feedback_frequency_);

	if (!getTelemetry()){
		msg_error("getTelemetry: An Error happened here.");
		goalPose_result_.reached_goal = false;
		goalPose_result_.error_reason = fsm::SIM_SRV_NA;
		goalPose_server_.setPreempted(goalPose_result_,"Got no telemetry.");

		return;
	}

	switch (goal->planning_algorithm)
	{
	case STANDARD_IK_7DOF:

		ROS_INFO("STANDARD IK 7DOF planning mode chosen.");

		//! Transform goal pose to LWR0 Base frame
		if (!transformToLWRBase())
		{
			msg_warn("Transformation to LWR0 Base failed.");

//			goalPose_result_.reached_goal = false;
//			goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
//			goalPose_server_.setPreempted(goalPose_result_,"Transformation to LWR0 Base failed.");
//			return;
		}
		//! Find IK solution
		if (!getIKSolution7DOF())
		{
			msg_error("No IK Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No IK Solution found.");
			return;
		}
		break;

	case HOMING_7DOF:

		ROS_INFO("HOMING 7DOF planning mode chosen.");

		if (!setReset7DOF())
		{
			msg_error("No IK Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No IK Solution found.");
			return;
		}

		break;

	case (MOVE_IT_7DOF)		:
	case (MOVE_IT_9DOF)		:
		ROS_WARN("Planning mode based on MoveIt! chosen.");

		if(goal->planning_algorithm == MOVE_IT_9DOF)
			group = group_9DOF;
		else if(goal->planning_algorithm == MOVE_IT_7DOF)
			group = group_7DOF;
		else
		{
			msg_error("Unknown move group name.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"Unknown move group name.");
			break;
		}

		if (!getMoveItSolution())
		{
			msg_error("No MoveIT Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No MoveIT Solution found.");
			return;
		}
		break;

	default:
		msg_warn("unkown Mode in MotionPlanning!");
		return;
	}

	getTimingAlongJointPath();

	starting_time_ = ros::Time::now().toSec();
	getGoalPose_Feedback();

	goalPose_server_.publishFeedback(goalPose_feedback_);

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
	//if (goalPose_feedback_.execution_time >= (goalPose_feedback_.estimated_motion_time+2.0))
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

void MotionPlanning::getGoalPose_Feedback()
{
	goalPose_feedback_.estimated_motion_time = estimated_motion_time_;
	goalPose_feedback_.execution_time = ros::Time::now().toSec() - starting_time_;

}



bool MotionPlanning::getIKSolution7DOF()
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

bool MotionPlanning::setReset7DOF()
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

bool MotionPlanning::getMoveItSolution()
{

  try
  {
	if (ros::service::waitForService(move_along_joint_path_,ros::Duration(10.0)))
	{


	  // print telemetry data
	  ROS_WARN("Measured telemetry: ");
	  for (unsigned idx = 0; idx < _telemetry.joint_names.size(); ++idx)
	  {
	    ROS_INFO_STREAM(_telemetry.joint_names[idx] << "  " << _telemetry.measured.position[idx]);
	  }



	  // store the joint names of the move group in a vector
	  std::vector<std::string> joint_namesMI = group->getActiveJoints();

	  // print the joint names of the move group
	  ROS_INFO("Joint names of the current move group:");
	  if (!joint_namesMI.empty())
	  {
	    for(unsigned idx = 0; idx < joint_namesMI.size(); ++idx)
	    {
	      ROS_INFO_STREAM("MoveitJoint: "<< joint_namesMI[idx]);
	    }
	  }
	  else
	  {
	    ROS_INFO("Vector of joint names empty.");
	    return false;
	  }



		// set start state equal to measured telemetry positions and velocities = 0;
		ROS_WARN("Set start state of the move group equal to the currently measured telemetry values");
		moveit_msgs::RobotState start_state;

		// declare joint positions and velocities
		std::vector<double> joint_positionsMI;
		std::vector<double> joint_velocitiesMI;


		unsigned matchCounter = 0;

		for (unsigned idxMI = 0; idxMI < joint_namesMI.size(); ++ idxMI)
		{

			unsigned idxTELE = 0;

			while (joint_namesMI.at(idxMI).compare(_telemetry.joint_names.at(idxTELE)))
			{

				idxTELE++;
				if (idxTELE > _telemetry.joint_names.size()-1)
				{
					ROS_WARN("MoveIt! joint not found in telemetry");
					return false;
				}
			}

			// print idx of the matching joint in the telemetry
			ROS_INFO_STREAM("telemetry idx of the current joint: " << idxTELE);

			// store the telemetry joint positions in the corresponding order of the move group
			joint_positionsMI.push_back(_telemetry.measured.position[idxTELE]);
			joint_velocitiesMI.push_back(0.0);

			// increase counter of matches between telemetry joints and MoveIt joints
			ROS_INFO_STREAM("number of joint matches: " << joint_positionsMI.size());

		}

		if (joint_positionsMI.size() == joint_namesMI.size())
		{
		  ROS_INFO("All MoveIt! joints found in the telemetry.");

		  start_state.joint_state.name = joint_namesMI;
		  start_state.joint_state.position = joint_positionsMI;
		  start_state.joint_state.velocity = joint_velocitiesMI;

		  group->setStartState(start_state);

		}
		else
		{
		  ROS_ERROR("Setting start state failed");
		  return false;
		}


		// print joint names, positions and velocities
		for (unsigned idx = 0; idx < start_state.joint_state.name.size(); ++idx)
		{
				ROS_INFO_STREAM(start_state.joint_state.name[idx] << "  " << start_state.joint_state.position[idx] << "  " << start_state.joint_state.velocity[idx]);
		}



		ROS_INFO_STREAM("Default goal joint tolerance: " << group->getGoalJointTolerance());
		ROS_INFO_STREAM("Default goal position tolerance: " << group->getGoalPositionTolerance());
		ROS_INFO_STREAM("Default goal orientation tolerance: " << group->getGoalOrientationTolerance());
//		group->setGoalTolerance(1);
//		group->setGoalPositionTolerance(0.1);
//		group->setGoalOrientationTolerance(0.1);
//		ROS_INFO_STREAM("Goal tolerance set to 1");
		ROS_INFO_STREAM("Current goal joint tolerance: " << group->getGoalJointTolerance());
		ROS_INFO_STREAM("Current goal position tolerance: " << group->getGoalPositionTolerance());
		ROS_INFO_STREAM("Current goal orientation tolerance: " << group->getGoalOrientationTolerance());




		// print the planning interface description
		moveit_msgs::PlannerInterfaceDescription plintdesc;
		group->getInterfaceDescription(plintdesc);
		ROS_INFO_STREAM("Name of the planner interface: " << plintdesc.name);
		ROS_INFO("Names of the planner ID's within the interface:");
		for (unsigned idx = 0; idx < plintdesc.planner_ids.size(); ++idx)
		{
			ROS_INFO_STREAM(idx << ": " << plintdesc.planner_ids[idx]);
		}



		// print the planning reference frame
		ROS_INFO_STREAM("Planning frame:" << group->getPlanningFrame());
		// print the pose reference frame
		ROS_INFO_STREAM("Pose reference frame: " << group->getPoseReferenceFrame());
		// print default planning time
		ROS_INFO_STREAM("Default planning time: " << group->getPlanningTime() << " seconds.");
		group->setPlanningTime(20);
		ROS_INFO_STREAM("Planning time set to " << group->getPlanningTime() << " seconds.");
		// print the name of the end effector
		ROS_INFO_STREAM("End effector: " << group->getEndEffector());
		// print the name of the end effector link
		ROS_INFO_STREAM("End effector link: " << group->getEndEffectorLink());



		// get the robot model
		robot_model::RobotModelConstPtr robot_model = planning_scene_monitor->getRobotModel();


		const robot_model::JointModelGroup* joint_model_group_LWR = robot_model->getJointModelGroup("LWR");



		ROS_WARN("JOINT BOUNDS");
		moveit::core::JointBoundsVector joint_bounds = joint_model_group_LWR->getActiveJointModelsBounds();


		for (unsigned idx = 0; idx < joint_bounds.size(); ++idx)
		{
			ROS_INFO_STREAM("position bounded: " << joint_bounds.at(idx)->at(0).position_bounded_);
			ROS_INFO_STREAM("min position: " << joint_bounds.at(idx)->at(0).min_position_);
			ROS_INFO_STREAM("max position: " << joint_bounds.at(idx)->at(0).max_position_);

			ROS_INFO_STREAM("velocity bounded: " << joint_bounds.at(idx)->at(0).velocity_bounded_);
			ROS_INFO_STREAM("min velocity: " << joint_bounds.at(idx)->at(0).min_velocity_);
			ROS_INFO_STREAM("max velocity: " << joint_bounds.at(idx)->at(0).max_velocity_);

			ROS_INFO_STREAM("acceleration bounded: " << joint_bounds.at(idx)->at(0).acceleration_bounded_);
			ROS_INFO_STREAM("min acceleration: " << joint_bounds.at(idx)->at(0).min_acceleration_);
			ROS_INFO_STREAM("max acceleration: " << joint_bounds.at(idx)->at(0).max_acceleration_);
		}






//		setPlanningConstraints();


		if (!setPlanningTarget(POSE_TARGET)) // POSE_TARGET / JOINT_VALUE_TARGET / APPROXIMATE_JOINT_VALUE_TARGET
		{
			ROS_ERROR("Setting planning target failed.");
			return false;
		}





		moveit::planning_interface::MoveGroup::Plan my_plan;

		if (group->plan(my_plan))
		{
			ROS_WARN("Planning successful!");

			ROS_INFO_STREAM("Planning time: " << my_plan.planning_time_);

			ROS_INFO_STREAM("Number of trajectory points: " << my_plan.trajectory_.joint_trajectory.points.size());

			planned_path_.clear();

			ROS_INFO("Planned Configurations:");

			// for each configuration of the trajectory except from the start configuration
			for (unsigned configIdx = 0; configIdx < my_plan.trajectory_.joint_trajectory.points.size(); ++configIdx)
			{
				// current configuration
				euroc_c2_msgs::Configuration current_config;

				ROS_INFO_STREAM("Config: " << configIdx);

				// for each joint at the current configuration
				for (unsigned jointIdx = 0; jointIdx < joint_namesMI.size(); ++ jointIdx)
				{
					current_config.q.push_back(my_plan.trajectory_.joint_trajectory.points[configIdx].positions[jointIdx]);
					ROS_INFO_STREAM("Joint " << jointIdx << ": " << current_config.q[jointIdx]);
				}

				planned_path_.push_back(current_config);
			}

			current_configuration_ = planned_path_[0];



			move_along_joint_path_srv_.request.joint_names = joint_namesMI;


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
		else	// if planning unsuccessful
		{
			return false;
		}
	}
	else	// if (!ros::service::waitForService(move_along_joint_path_,ros::Duration(10.0)))
	  return false;

  } // end try

  catch(...)
  {
    msg_error("MotionPlanning::Error in MoveIt! Planning.");
    return false;
  }
  return true;


}

bool MotionPlanning::setPlanningTarget(planning_target_type_t planning_target)
{
	ROS_WARN("Set planning target");

	switch (planning_target)
	{

	case POSE_TARGET:
		ROS_INFO("Planning for a pose target chosen.");

		ROS_INFO_STREAM(goal_pose_GPTCP_);

		if (!group->setPoseTarget(goal_pose_GPTCP_))
		{
			ROS_ERROR("Setting pose target failed.");
			return false;
		}
		break;

	case JOINT_VALUE_TARGET:
		ROS_INFO("Planning for a joint value target chosen.");
		ROS_ERROR("Planning for a joint value target not implemented.");
		return false;

	case APPROXIMATE_JOINT_VALUE_TARGET:
		ROS_INFO("Planning for an approximate joint value target chosen.");
		if (!group->setApproximateJointValueTarget(goal_pose_goal_->goal_pose))
		{
			ROS_ERROR("Setting approximate joint value target failed.");
			return false;
		}
		break;

	default:
		ROS_ERROR("False planning target type.");
		return false;

	}

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

		for (int i=0;i<time_at_path_points_.size();i++)
		{
			estimated_motion_time_ += time_at_path_points_[i].toSec();
		}
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

bool MotionPlanning::return_poses_valid(am_msgs::CheckPoses::Request &req, am_msgs::CheckPoses::Response &res)
{
	uint16_t nr_poses = req.poses.size();
	ROS_INFO("in check poses service call:");

	res.priority.resize(nr_poses);

	for(uint16_t ii=0;ii<nr_poses;ii++)
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
				search_ik_solution_srv_.request.tcp_frame = req.poses[ii];

				search_ik_solution_client_.call(search_ik_solution_srv_);
				std::string &search_error_message = search_ik_solution_srv_.response.error_message;
				if(!search_error_message.empty()){
					//ROS_INFO("Search IK Solution failed: %s", search_error_message.c_str());
					res.priority[ii]=0;
				}
				else
					res.priority[ii]=1;
			}
		} catch(...) {
			msg_error("failed to find service search ik-solution");
			return false;
		}

	}

	ROS_INFO("finished check poses service call.");

	return true;
}

bool MotionPlanning::return_object_attached(am_msgs::AttachObject::Request &req, am_msgs::AttachObject::Response &res)
{
//	ROS_WARN("ATTACH OBJECT CALLBACK STARTED");
//
//	moveit_msgs::CollisionObject collision_object;
//	collision_object.header.frame_id = group->getPlanningFrame();
//
//	/* The id of the object is used to identify it. */
//	collision_object.id = "box1";
//
//	/* Define a box to add to the world. */
//	shape_msgs::SolidPrimitive primitive;
//	primitive.type = primitive.BOX;
//	primitive.dimensions.resize(3);
//	primitive.dimensions[0] = 0.4;
//	primitive.dimensions[1] = 0.1;
//	primitive.dimensions[2] = 0.4;
//
//	/* A pose for the box (specified relative to frame_id) */
//	geometry_msgs::Pose box_pose;
//	box_pose.orientation.w = 1.0;
//	box_pose.position.x =  0.6;
//	box_pose.position.y = -0.4;
//	box_pose.position.z =  1.2;
//
//	collision_object.primitives.push_back(primitive);
//	collision_object.primitive_poses.push_back(box_pose);
//	collision_object.operation = collision_object.ADD;
//
//	std::vector<moveit_msgs::CollisionObject> collision_objects;
//	collision_objects.push_back(collision_object);
//
//	ROS_INFO("Add an object into the world");
//	planning_scene_interface.addCollisionObjects(collision_objects);
//
//	/* Sleep so we have time to see the object in RViz */
//	sleep(2.0);
//
//
//	ROS_INFO("Attach the object to the robot");
//	group->attachObject(collision_object.id);
//	/* Sleep to give Rviz time to show the object attached (different color). */
//	sleep(4.0);

	res.successful = true;
	return true;
}

bool MotionPlanning::return_object_detached(am_msgs::DetachObject::Request &req, am_msgs::DetachObject::Response &res)
{
	res.successful = true;
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
	table_axis1_limit_.max_acceleration = 2.0;
	table_axis2_limit_.max_acceleration = 2.0;
	for (int ii=0;ii<7;ii++)
		joint_limits_[ii].max_acceleration = 400 * M_PI / 180.0;
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

  move_along_joint_path_srv_.request.joint_limits.resize(group->getActiveJoints().size());

  unsigned joint_counter = 0;

  if (move_along_joint_path_srv_.request.joint_limits.size() == 9)
  {

    move_along_joint_path_srv_.request.joint_limits[0].max_velocity = table_axis1_limit_.max_velocity;
    move_along_joint_path_srv_.request.joint_limits[0].max_acceleration = table_axis1_limit_.max_acceleration;

    move_along_joint_path_srv_.request.joint_limits[1].max_velocity = table_axis2_limit_.max_velocity;
    move_along_joint_path_srv_.request.joint_limits[1].max_acceleration = table_axis2_limit_.max_acceleration;

    joint_counter = 2;
  }

  unsigned jointIdx = 0;
  for(unsigned idx = joint_counter; idx < joint_counter+7; ++idx)
  {
    move_along_joint_path_srv_.request.joint_limits[idx].max_velocity = joint_limits_[jointIdx].max_velocity;
    move_along_joint_path_srv_.request.joint_limits[idx].max_acceleration = joint_limits_[jointIdx].max_acceleration;

    jointIdx++;
  }

}


void MotionPlanning::setMoveRequestTCPLimits()
{
	move_along_joint_path_srv_.request.tcp_limits.translational.max_velocity = table_axis1_limit_.max_velocity;
	move_along_joint_path_srv_.request.tcp_limits.translational.max_acceleration = table_axis1_limit_.max_acceleration;

	move_along_joint_path_srv_.request.tcp_limits.rotational.max_velocity = 20 * M_PI / 180.0;;
	move_along_joint_path_srv_.request.tcp_limits.rotational.max_acceleration = 400 * M_PI / 180.0;;
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
#if 0
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
#else
	try{
		//
		//check object_grip poses
		get_static_tf_data_srv_.request.child_frame = GP_TCP;
		get_static_tf_data_srv_.request.parent_frame = LWR_TCP;

		if(!get_static_tf_data_client_.call(get_static_tf_data_srv_))
		{
			msg_error("Error. failed to call get_static_tf_data_client_");
			return false;
		}
		else{
			transform_GPTCP_2_LWRTCP.setOrigin(tf::Vector3(get_static_tf_data_srv_.response.transform.transform.translation.x,
					get_static_tf_data_srv_.response.transform.transform.translation.y,
					get_static_tf_data_srv_.response.transform.transform.translation.z));
			transform_GPTCP_2_LWRTCP.setRotation(tf::Quaternion(get_static_tf_data_srv_.response.transform.transform.rotation.x,
					get_static_tf_data_srv_.response.transform.transform.rotation.y,
					get_static_tf_data_srv_.response.transform.transform.rotation.z,
					get_static_tf_data_srv_.response.transform.transform.rotation.w));
		}
	}
	catch(...){
		msg_error("Listening to transform was not successful");
		return false;
	}
#endif

	tf_tmp.setOrigin(tf::Vector3(goal_pose_goal_->goal_pose.position.x,
			goal_pose_goal_->goal_pose.position.y,
			goal_pose_goal_->goal_pose.position.z));
	tf_tmp.setRotation(tf::Quaternion(goal_pose_goal_->goal_pose.orientation.x,
			goal_pose_goal_->goal_pose.orientation.y,
			goal_pose_goal_->goal_pose.orientation.z,
			goal_pose_goal_->goal_pose.orientation.w));

	if (!frame.compare(GP_TCP)){
#if 0
		tf_tmp2 = transform_GPTCP_2_LWRTCP*tf_tmp;
		goal_pose_GPTCP_ = goal_pose_goal_->goal_pose;

		goal_pose_LWRTCP_.position.x = tf_tmp2.getOrigin().getX();
		goal_pose_LWRTCP_.position.y = tf_tmp2.getOrigin().getY();
		goal_pose_LWRTCP_.position.z = tf_tmp2.getOrigin().getZ();

		goal_pose_LWRTCP_.orientation.x = tf_tmp2.getRotation().getX();
		goal_pose_LWRTCP_.orientation.y = tf_tmp2.getRotation().getY();
		goal_pose_LWRTCP_.orientation.z = tf_tmp2.getRotation().getZ();
		goal_pose_LWRTCP_.orientation.w = tf_tmp2.getRotation().getW();

#else
		tf_tmp.setOrigin(tf::Vector3(goal_pose_goal_->goal_pose.position.x,goal_pose_goal_->goal_pose.position.y,goal_pose_goal_->goal_pose.position.z));
		tf_tmp.setRotation(tf::Quaternion(goal_pose_goal_->goal_pose.orientation.x,goal_pose_goal_->goal_pose.orientation.y,goal_pose_goal_->goal_pose.orientation.z,goal_pose_goal_->goal_pose.orientation.w));
		//DUMMY-CODE
		tf::Vector3 gripper_z_axis, tmp_origin;
		double distance_GPTCP_to_LWRTCP = 0.173;
		gripper_z_axis=tf_tmp.getBasis().getColumn(2);
		tmp_origin=tf_tmp.getOrigin()-gripper_z_axis*distance_GPTCP_to_LWRTCP;
		goal_pose_LWRTCP_.position.x = tmp_origin.getX();
		goal_pose_LWRTCP_.position.y = tmp_origin.getY();
		goal_pose_LWRTCP_.position.z = tmp_origin.getZ();

		goal_pose_LWRTCP_.orientation.x = tf_tmp.getRotation().getX();
		goal_pose_LWRTCP_.orientation.y = tf_tmp.getRotation().getY();
		goal_pose_LWRTCP_.orientation.z = tf_tmp.getRotation().getZ();
		goal_pose_LWRTCP_.orientation.w = tf_tmp.getRotation().getW();
#endif
		ROS_INFO("transform_GPTCP_2_LWRTCP: [%f %f %f .. %f %f %f %f] ",transform_GPTCP_2_LWRTCP.getOrigin().getX(),
				transform_GPTCP_2_LWRTCP.getOrigin().getY(),transform_GPTCP_2_LWRTCP.getOrigin().getZ(),
				transform_GPTCP_2_LWRTCP.getRotation().getX(),transform_GPTCP_2_LWRTCP.getRotation().getY(),
				transform_GPTCP_2_LWRTCP.getRotation().getZ(),transform_GPTCP_2_LWRTCP.getRotation().getW());

		ROS_INFO("LWRTCP Pose in MotionPlanning: [%f %f %f .. %f %f %f %f]" , goal_pose_LWRTCP_.position.x,
				goal_pose_LWRTCP_.position.y,goal_pose_LWRTCP_.position.z,goal_pose_LWRTCP_.orientation.x,
				goal_pose_LWRTCP_.orientation.y,goal_pose_LWRTCP_.orientation.z,goal_pose_LWRTCP_.orientation.w);
	}
	else if (!frame.compare(LWR_TCP)){
		tf_tmp2 = transform_GPTCP_2_LWRTCP.inverse()*tf_tmp;
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

	tf_tmp2 = transform_ORIGIN_2_LWR0*tf_tmp;

	goal_pose_LWRTCP_.position.x = tf_tmp2.getOrigin().getX();
	goal_pose_LWRTCP_.position.y = tf_tmp2.getOrigin().getY();
	goal_pose_LWRTCP_.position.z = tf_tmp2.getOrigin().getZ();

	goal_pose_LWRTCP_.orientation.x = tf_tmp2.getRotation().getX();
	goal_pose_LWRTCP_.orientation.y = tf_tmp2.getRotation().getY();
	goal_pose_LWRTCP_.orientation.z = tf_tmp2.getRotation().getZ();
	goal_pose_LWRTCP_.orientation.w = tf_tmp2.getRotation().getW();

	return true;
}
