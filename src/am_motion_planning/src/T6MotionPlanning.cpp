
/*
 * T6MotionPlanning.cpp
 *
 *  Created on: Nov 3, 2014
 *      Author: euroc_admin
 */

#include <T6MotionPlanning.h>

T6MotionPlanning::T6MotionPlanning():
T5MotionPlanning::T5MotionPlanning()
{
	target_zone_radius_ = 0.0;
	tolerance_ = 0.0;
	tolerance_height_ = 0.3;
	height_over_belt_ = 0.2;
	standard_distance_dcp_ = 0.2;
	standard_distance_dcp_schlitten_ = 0.5;
	gradient_distance_dcp_ = 0.15;
	standard_distance_dcp_planar_axis_ = 0.65;
	n_objects_ = 0;
	n_obj_ges_ = 10;
	t_rdv = 0;


	// gripper client
	gripper_control_client_ = nh_.serviceClient<am_msgs::GripperControl>("GripperInterface");

}

T6MotionPlanning::~T6MotionPlanning() {
	// TODO Auto-generated destructor stub
}

bool T6MotionPlanning::executeGoalPoseT6()
{
	ROS_WARN("T666666666666");

	if(n_objects_ == 0)
	{
		getParam();
		initializeConveyorBelt();
	}
	//-----------------------------------------------------------------------------------
	time_stamp_in_ = goal_pose_goal_->stamp.sec;


	//--------------------------------------------------------------------------------------
	//! Plan
	switch (goal_pose_goal_->planning_algorithm)
	{
	case T6_MOVE_IT_9DOF_BELTHOMING:
		//counter of object
		ROS_INFO("MOVE_T6");
		ROS_WARN("Object number %d",n_objects_);

		ROS_INFO("Chosen 9DOF");
		group = group_9DOF;
		joint_model_group_ = joint_model_group_9DOF_;
		// setting joint state target via the searchIKSolution srv is not considered
		max_setTarget_attempts_ = 4;

		//get goal Pose
		T6_calc_goalHoming_TCP();
		goal_pose_GPTCP_ = goal_config_homing_T6;

		//set target algorithm
		current_setTarget_algorithm_ = SINGLE_POSE_TARGET;
		// get moveit solution
		if (!T6_MoveIt_getSolution())
		{
			msg_error("No MoveIT Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No MoveIT Solution found.");
			return false;
		}
		break;

		//--------------------------------------------------------------------------------------
	case T6_MOVE_IT_2DOF:
		// calculate Position
		T6_calc_goalHoming_schlitten();

		goal_pose_GPTCP_ = goal_config_homing_T6_schlitten;

		if (!executeGoalPoseStd())
		{
			msg_error("executeGoalPoseStd() failed");
			return false;
		}
		return true;

		//--------------------------------------------------------------------------------------
	case T6_STANDARD_IK_7DOF:

		if (!executeGoalPoseStd())
		{
			msg_error("executeGoalPoseStd() failed");
			return false;
		}
		return true;

		//--------------------------------------------------------------------------------------
	case T6_MOVE_IT_9DOF_TARGET:
		ROS_INFO("MOVE_IT_9DOF_TARGET");
		group = group_9DOF;
		joint_model_group_ = joint_model_group_9DOF_;
		// setting joint state target via the searchIKSolution srv is not considered
		max_setTarget_attempts_ = 4;

		//get goal Pose
		T6_calcTarget();
		goal_pose_GPTCP_ = goal_target_pos;
		group->setGoalPositionTolerance(tolerance_);

		//set target algorithm
		current_setTarget_algorithm_ = SINGLE_POSE_TARGET;
		// get moveit solution
		if (!T6_MoveIt_getSolution())
		{
			msg_error("No MoveIT Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No MoveIT Solution found.");
			return false;
		}

		break;

		//--------------------------------------------------------------------------------------

	case (MOVE_IT_9DOF)		:

	ROS_WARN("Planning mode based on MoveIt! 9 DOF chosen.");

	if(!T6_MoveIt_move_to_object())
	{
		msg_error("move to finale pose failed!");
		return false;
	}


	//wait time until rdv
	boost::this_thread::sleep( boost::posix_time::seconds(t_rdv - ros::Time::now().sec ));

	//grap object
	if(!T6_grap_object())
	{
		msg_error("grap object failed");
		return false;
	}

	if(!T6_MoveIt_move_object_safe())
	{
		msg_error("move to finale pose failed!");
		return false;
	}

	return true;
//--------------------------------------------------------------------------------------
	default:
		msg_warn("unknown Mode in MotionPlanning! (T6)");
		return false;
	}

	ROS_INFO("executeGoalPoseT6 successful");

	//execute motion
	if(!executeStd())
	{
		msg_error("Execute motion failed!");
		return false;
	}
	return true;
}

bool T6MotionPlanning::T6_grap_object()
{
	ROS_INFO("gripper_close_cb() running");

	//gripper control
	gripper_control_srv_.request.gripping_mode = POSITION;
	gripper_control_srv_.request.gripper_position = 0.0;

	if(gripper_control_client_.exists())
	{
		if(gripper_control_client_.call(gripper_control_srv_))
		{
			gripper_close_state_=FINISHED;
			return true;
		}
		else
		{
			msg_error("Error. call of gripper_control_client_ failed");
			return false;
		}
	}
	else
	{
		msg_error("Error. gripper_control_client_ is not available");
		return false;
	}

	ROS_INFO("gripper_close_cb() finished");
	return false;
}

bool T6MotionPlanning::T6_MoveIt_move_to_object()
{
	//define groups
	if(goal_pose_goal_->planning_algorithm == MOVE_IT_9DOF)
	{
		ROS_INFO("9DOF planning mode chosen.");
		group = group_9DOF;
		joint_model_group_ = joint_model_group_9DOF_;
		// setting joint state target via the searchIKSolution srv is not considered
		max_setTarget_attempts_ = 4;

	}
	else
	{
		msg_error("Unknown move group name.");
		goalPose_result_.reached_goal = false;
		goalPose_server_.setPreempted(goalPose_result_,"Unknown move group name.");
		return false;
	}

	//consider time in goal pose
	T6_getNewPose(goal_pose_GPTCP_);


	//set target algorithm
	current_setTarget_algorithm_ = SINGLE_POSE_TARGET;
	// get moveit solution
	if (!T6_MoveIt_getSolution())
	{
		msg_error("No MoveIT Solution found.");
		goalPose_result_.reached_goal = false;
		goalPose_server_.setPreempted(goalPose_result_,"No MoveIT Solution found.");
		return false;
	}

	//execute Motion
	if(!executeStd())
	{
		msg_error("Execute motion failed");
		return false;
	}

	return true;
}

bool T6MotionPlanning::T6_MoveIt_move_object_safe()
{
	msg_error("TO IMPLEMENT");
	return false;
}

bool T6MotionPlanning::T6_dynHoming()
{

	//***********2DOF***********************
	ROS_INFO("executing T6 homing with 9 DOF");


	group = group_2DOF;
	joint_model_group_ = joint_model_group_2DOF_;
	// setting joint state target via the searchIKSolution srv is not considered
	max_setTarget_attempts_ = 4;




	goal_pose_LWRTCP_ = goal_config_homing_T6_schlitten;
	goal_pose_GPTCP_ = goal_config_homing_T6_schlitten;

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


	//***********EUROC***********************
	//	ROS_INFO("STANDARD IK 7DOF planning mode chosen.");
	//
	//	goal_pose_LWRTCP_ = goal_config_homing_T6;
	//	goal_pose_GPTCP_ = goal_config_homing_T6;
	//	//! Find IK solution
	//	if (!euroc_getIKSolution7DOF())
	//	{
	//		msg_error("No IK Solution found.");
	//		goalPose_result_.reached_goal = false;
	//		goalPose_server_.setPreempted(goalPose_result_,"No IK Solution found.");
	//		return false;
	//	}


	return true;
}

bool T6MotionPlanning::T6_calc_goalHoming_TCP()
{

	double a;
	double b;
	double c;

	// Pose für EUROC

	ros::param::get("/object_counter_", n_objects_);
	goal_config_homing_T6.position.x = drop_center_point.x + (n_objects_ * gradient_distance_dcp_ + standard_distance_dcp_) * mdl_norm.x; // Faktor
	goal_config_homing_T6.position.y = drop_center_point.y + (n_objects_ * gradient_distance_dcp_ + standard_distance_dcp_) * mdl_norm.y; // Faktor
	goal_config_homing_T6.position.z = drop_center_point.z + height_over_belt_;


	if((drop_center_point.x * mdl_norm.y - drop_center_point.y * mdl_norm.x) > 0.0) // uhrzeiger
	{
		//drehwinkel
		winkel = acos((0*mdl_norm.x + 1*mdl_norm.y)/(sqrt(pow(mdl_norm.x,2) + pow(mdl_norm.y,2))));

		a=-3.0/4.0*M_PI;
		b=0.0;
		c=winkel - M_PI;

		q_tmp.setRPY(-3.0/4.0*M_PI, 0.0, winkel - M_PI);



		goal_config_homing_T6.orientation.x = q_tmp.x();
		goal_config_homing_T6.orientation.y = q_tmp.y();
		goal_config_homing_T6.orientation.z = q_tmp.z();
		goal_config_homing_T6.orientation.w = q_tmp.w();


	}
	else
	{
		//drehwinkel
		winkel = acos((0*mdl_norm.x + 1*mdl_norm.y)/(sqrt(pow(mdl_norm.x,2) + pow(mdl_norm.y,2))));

		a=-1.0/4.0*M_PI;
		b=M_PI;
		c=winkel- M_PI;


		q_tmp.setRPY(-1.0/4.0*M_PI, M_PI, winkel- M_PI);

		goal_config_homing_T6.orientation.x = q_tmp.x();
		goal_config_homing_T6.orientation.y = q_tmp.y();
		goal_config_homing_T6.orientation.z = q_tmp.z();
		goal_config_homing_T6.orientation.w = q_tmp.w();

	}

	ROS_INFO("POSE + RPY");
	ROS_INFO_STREAM(goal_config_homing_T6.position.x);
	ROS_INFO_STREAM(goal_config_homing_T6.position.y);
	ROS_INFO_STREAM(goal_config_homing_T6.position.z);
	ROS_INFO_STREAM(a);
	ROS_INFO_STREAM(b);
	ROS_INFO_STREAM(c);






	return true;
}

bool T6MotionPlanning::T6_calc_goalHoming_schlitten()
{



	ros::param::get("/object_counter_", n_objects_);
	if((drop_center_point.x * mdl_norm.y - drop_center_point.y * mdl_norm.x) < 0) // uhrzeiger
	{
		// Pose schlitten
		goal_config_homing_T6_schlitten.position.x = drop_center_point.x + (n_objects_ * gradient_distance_dcp_ + standard_distance_dcp_) * mdl_norm.x + standard_distance_dcp_planar_axis_ * mdl_norm.y;
		goal_config_homing_T6_schlitten.position.y = drop_center_point.y + (n_objects_ * gradient_distance_dcp_ + standard_distance_dcp_) * mdl_norm.y - standard_distance_dcp_planar_axis_ * mdl_norm.x;
	}
	else
	{
		// Pose schlitten
		goal_config_homing_T6_schlitten.position.x = drop_center_point.x + (n_objects_ * gradient_distance_dcp_ + standard_distance_dcp_) * mdl_norm.x - standard_distance_dcp_planar_axis_ * mdl_norm.y;
		goal_config_homing_T6_schlitten.position.y = drop_center_point.y + (n_objects_ * gradient_distance_dcp_ + standard_distance_dcp_) * mdl_norm.y + standard_distance_dcp_planar_axis_ * mdl_norm.x;
	}
	//
	//	ROS_INFO("schlitten pose");
	//	ROS_INFO_STREAM(goal_config_homing_T6_schlitten.position.x);
	//	ROS_INFO_STREAM(goal_config_homing_T6_schlitten.position.y);


	return true;
}


bool T6MotionPlanning::T6_calcTarget()// --> statemachine
{
	goal_target_pos.position.x = target_pos.x;
	goal_target_pos.position.y = target_pos.y;
	goal_target_pos.position.z = target_pos.z + tolerance_height_;
	goal_target_pos.orientation.w = 0;
	goal_target_pos.orientation.x = 1;
	goal_target_pos.orientation.y = 0;
	goal_target_pos.orientation.z = 0;

	tolerance_ = target_zone_radius_ * 0.8;

	return true;

}

bool T6MotionPlanning::T6_getBeltSpeed()
{
	ros::param::get("/object_counter_", n_objects_);

	// belt speed
	belt_speed_ = start_speed_ + (n_objects_-1)*((end_speed_-start_speed_)/(n_obj_ges_-1));

	if (belt_speed_>0)
	{
		return true;
	}
	else
	{
		ROS_WARN("Belt speed error!");
		return false;
	}
}


bool T6MotionPlanning::T6_getNewPose(geometry_msgs::Pose& pose)
{
	if(T6_getBeltSpeed())
	{

		ros::param::get("/object_counter_", n_objects_);

		// TODO adaptiv T6_puffer_pose
		T6_puffer_pose = 1 + n_objects_ * 0.1;

		geometry_msgs::Pose old_pose; // ziel position
		old_pose = pose;

		// stamp; x + (t2 - t1)*v + puffer;
		pose.position.x = old_pose.position.x + T6_puffer_pose * (ros::Time::now().sec - time_stamp_in_) * mdl.x;
		pose.position.y = old_pose.position.y + T6_puffer_pose * (ros::Time::now().sec - time_stamp_in_) * mdl.y;
		pose.position.z = old_pose.position.z + T6_puffer_pose * (ros::Time::now().sec - time_stamp_in_) * mdl.z;

		double delta_s = sqrt( (pose.position.x - old_pose.position.x)*(pose.position.x - old_pose.position.x)
				 	 	 	 + (pose.position.y - old_pose.position.y)*(pose.position.y - old_pose.position.y)
				 	 	 	 + (pose.position.z - old_pose.position.z)*(pose.position.z - old_pose.position.z) );

		double v = sqrt(pow(mdl.x,2) + pow(mdl.y,2) + pow(mdl.z,2));
		t_rdv = delta_s / v;

		return true;
	}
	else
	{
		return false;
	}
}

void T6MotionPlanning::initializeConveyorBelt()
{
	ROS_INFO("Adding conveyor belt to collision world...");


	//************************
	// conveyer belt

	// drehwinkel um z
	winkel = acos((0*mdl_norm.x + 1*mdl_norm.y)/(sqrt(pow(mdl_norm.x,2) + pow(mdl_norm.y,2))));
	q_tmp.setRPY(0, 0, winkel);



	shape_msgs::SolidPrimitive con_belt;
	con_belt.type = con_belt.BOX;
	con_belt.dimensions.resize(3);
	con_belt.dimensions[0] = 2 * drop_deviation.y;
	con_belt.dimensions[1] = 2 * sqrt(pow(mdl_norm.x,2) + pow(mdl_norm.y,2)) + 0.1;
	con_belt.dimensions[2] = drop_center_point.z * 0.9 ; // TODO TEST then take out scalar and make it to variable

	geometry_msgs::Pose con_belt_pose;
	con_belt_pose.position.x = drop_center_point.x + mdl_norm.x;
	con_belt_pose.position.y = drop_center_point.y + mdl_norm.y;
	con_belt_pose.position.z = 0.05;//-0.01;
	con_belt_pose.orientation.x = q_tmp.x();
	con_belt_pose.orientation.y = q_tmp.y();
	con_belt_pose.orientation.z = q_tmp.z();
	con_belt_pose.orientation.w = q_tmp.w();

	moveit_msgs::CollisionObject con_belt_object;
	con_belt_object.id = "con_belt";
	con_belt_object.header.frame_id = "/Origin";
	con_belt_object.header.stamp = ros::Time::now();
	con_belt_object.operation = con_belt_object.ADD;
	con_belt_object.primitives.push_back(con_belt);
	con_belt_object.primitive_poses.push_back(con_belt_pose);

	static_scene_.world.collision_objects.push_back(con_belt_object);



	static_scene_.is_diff = true;
	planning_scene_diff_publisher_.publish(static_scene_);
	//static_scene_.world.collision_objects.clear();

	ROS_INFO("Finished adding conveyor belt.");

}

bool T6MotionPlanning::T6_MoveIt_getSolution()
{

	try
	{
		if (ros::service::waitForService(move_along_joint_path_,ros::Duration(10.0)))
		{
			if(!T6_MoveIt_initializeMoveGroup()){return false;}

			unsigned current_setTarget_attempt = 1;
			bool setTarget_successful = false;
			bool planning_successful = false;

			while (!planning_successful)
			{
				setTarget_successful = false;
				setTarget_successful = setPlanningTarget(current_setTarget_algorithm_);
				if (setTarget_successful)
				{
					//					ROS_INFO("Planning!");
					//					ROS_INFO_STREAM(group->getName());
					//					ROS_INFO_STREAM(group->getPoseReferenceFrame());
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

				current_setTarget_algorithm_++;
			}


		}
		else	// if (!ros::service::waitForService(move_along_joint_path_,ros::Duration(10.0)))
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

bool T6MotionPlanning::T6_MoveIt_initializeMoveGroup()
{
	// store the joint names of the move group in a vector
	std::vector<std::string> joint_namesMI = group->getActiveJoints();

	// print the joint names of the move group
	ROS_INFO("Joint names of the current move group:");
	if (joint_namesMI.empty())
	{
		ROS_INFO("MoveGroup: Vector of joint names empty. General Motion Planning Error!");
		goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
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
				goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
				return false;
			}
		}

		// store the telemetry joint positions in the corresponding order of the move group
		joint_positionsMI.push_back(_telemetry.measured.position[idxTELE]);
		joint_velocitiesMI.push_back(0.0);
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
		goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
		return false;
	}


	// print the planning interface description
	moveit_msgs::PlannerInterfaceDescription plintdesc;
	group->getInterfaceDescription(plintdesc);
	group->setPlanningTime(20);
	//==========================================================================================

	// get the robot model
	robot_model::RobotModelConstPtr robot_model = planning_scene_monitor->getRobotModel();
	const robot_model::JointModelGroup* joint_model_group_LWR = robot_model->getJointModelGroup("LWR_9DOF");
	//	ROS_WARN("JOINT BOUNDS");
	moveit::core::JointBoundsVector joint_bounds = joint_model_group_LWR->getActiveJointModelsBounds();


	//==========================================================================================
	// OCTOMAP -> deleted!
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	kinematic_model_ = robot_model_loader.getModel();

	// Load current robot state into planning scene
	planning_scene_.robot_state.joint_state = getCurrentJointState();
	planning_scene_.robot_state.is_diff = true;

	// Robot link padding
	planning_scene_.link_padding.resize(planning_scene_monitor->getRobotModel()->getLinkModelNames().size());
	for (unsigned i = 0; i < planning_scene_monitor->getRobotModel()->getLinkModelNames().size(); ++i)
	{
		planning_scene_.link_padding[i].link_name = planning_scene_monitor->getRobotModel()->getLinkModelNames()[i];
		if (!planning_scene_.link_padding[i].link_name.compare("finger1") || !planning_scene_.link_padding[i].link_name.compare("finger2"))
			planning_scene_.link_padding[i].padding = 0.0;
		else if (!planning_scene_.link_padding[i].link_name.compare("base"))
			planning_scene_.link_padding[i].padding = 0.02;
		else
			planning_scene_.link_padding[i].padding = 0.01;
	}

	// setting planning scene message to type diff
	planning_scene_.is_diff = true;

	// Publish msg on topic /planning_scene
	planning_scene_diff_publisher_.publish(planning_scene_);
	planning_scene_diff_publisher_.publish(static_scene_);

	//==========================================================================================
	return true;
}

void T6MotionPlanning::getParam()
{
	ros::param::get("/skip_vision", skip_vision_);

	// get start and end speed of param server
	ros::param::get("/conveyorbelt_start_speed_", start_speed_);
	ros::param::get("/conveyorbelt_end_speed_", end_speed_);

	//Targetzone
	ros::param::get("/target_zone_0_x_", target_pos.x);
	ros::param::get("/target_zone_0_y_", target_pos.y);
	ros::param::get("/target_zone_0_radius_", target_zone_radius_);
	//manual
	target_pos.z = 0;

	//get conveyor belt informations
	ros::param::get("/conveyorbelt_drop_center_point_x_", drop_center_point.x);
	ros::param::get("/conveyorbelt_drop_center_point_y_", drop_center_point.y);
	ros::param::get("/conveyorbelt_drop_center_point_z_", drop_center_point.z);

	ros::param::get("/conveyorbelt_move_direction_and_length_x_", mdl.x);
	ros::param::get("/conveyorbelt_move_direction_and_length_y_", mdl.y);
	ros::param::get("/conveyorbelt_move_direction_and_length_z_", mdl.z);

	//normieren
	mdl_norm.x = mdl.x / sqrt(pow(mdl.x,2) + pow(mdl.y,2) + pow(mdl.z,2));
	mdl_norm.y = mdl.y / sqrt(pow(mdl.x,2) + pow(mdl.y,2) + pow(mdl.z,2));
	mdl_norm.z = mdl.z / sqrt(pow(mdl.x,2) + pow(mdl.y,2) + pow(mdl.z,2));

	ros::param::get("/conveyorbelt_drop_deviation_x_", drop_deviation.x);
	ros::param::get("/conveyorbelt_drop_deviation_y_", drop_deviation.y);
	ros::param::get("/conveyorbelt_drop_deviation_z_", drop_deviation.z);

	// Object counter
	ros::param::get("/object_counter_", n_objects_);

}

bool T6MotionPlanning::T6_belt_up(geometry_msgs::Pose& pose)
{
	pose.position.z = pose.position.z + height_over_belt_;
	return true;
}

