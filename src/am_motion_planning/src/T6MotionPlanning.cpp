
/*
 * T6MotionPlanning.cpp
 *
 *  Created on: Nov 3, 2014
 *      Author: euroc_admin
 */

#include <T6MotionPlanning.h>

#include <config.hpp>

T6MotionPlanning::T6MotionPlanning():
T5MotionPlanning::T5MotionPlanning()
{
	target_zone_radius_ = 0.0;
	tolerance_7DoF_position_ = 0.1;
	factor_tolerance_7DoF_position_=0.7;//changes tolerance_7DoF_position_based on radius of target_zone
	tolerance_7DoF_orientation_ = 0.01;
	tolerance_height_ = 0.2;
	height_over_belt_ = 0.2;
	height_over_belt_tuning_ = 0.0;//only for gripping position
	target_zone_radius_security_ = 0.6;//multipliert mit radius
	target_distance_ = 0.7;
	standard_distance_dcp_ = 0.2;
	standard_distance_dcp_schlitten_ = 0.5;
	gradient_distance_dcp_ = 0.15;
	standard_distance_dcp_planar_axis_ = 0.6;
	n_objects_ = 0;
	n_obj_ges_ = 10;
	home_faktor_ =1.8;
	t_rdv = 0;
	t_security_ = 8;
	conveyor_belt_move_direction_and_length.setValue(0,0,0);
	gripping_angle_deg_ = gripping_angleT6_deg_config;
	gripping_angle_rad_=gripping_angle_deg_/180.0*(M_PI);

	// gripper client
	gripper_control_client_ = nh_.serviceClient<am_msgs::GripperControl>("GripperInterface");
	pose_counter = 0;
}

T6MotionPlanning::~T6MotionPlanning() {
	// TODO Auto-generated destructor stub
}

bool T6MotionPlanning::executeGoalPoseT6()
{
	ROS_WARN("TASK 6. ");
	planning_frame_ = "/Origin";

	if(n_objects_ == 0)
	{
		T6_getParam();
		T6_initializeConveyorBelt();
	}


	ROS_INFO_STREAM("Vision pose "<<goal_pose_GPTCP_.position);

	//-----------------------------------------------------------------------------------
	time_stamp_in_ = goal_pose_goal_->stamp.sec;


	//-------------------------------------------------------------------------------------
	//save sent goal poses
	save_goal_pose_GPTCP_ = goal_pose_GPTCP_;
	save_goal_pose_LWRTCP_ = goal_pose_LWRTCP_;

	//--------------------------------------------------------------------------------------
	//! Plan
	//	ROS_INFO_STREAM(goal_pose_goal_->planning_algorithm);
	switch (goal_pose_goal_->planning_algorithm)
	{
	case T6_MOVE_IT_9DOF_BELTHOMING:

		//besser in statemachine:
#warning SHIFT INTO STATEMACHINE
		speed_percentage_ = 60;
		getLimits();

		//counter of object
		n_objects_ = n_objects_ + 1;

		ROS_WARN("MOVE_T6: 2 DOF + EUROC");
		ROS_WARN("Object number %d",n_objects_);
		// calculate factor for homing
		home_faktor_ = 0.5 +  ((-1/81)* n_objects_*n_objects_ +(20/81)*n_objects_ + (-19/81))*(1.8-0.5);


		// get solution
		if(!T6_move_2DoF_Euroc())
		{
			msg_error("No solution found for 2DoF + Euroc.");
			return false;
		}

		// Open gripper
		T6_open_gripper();


		//execute motion
		ROS_INFO("Execute Belthoming successful");


		return true;

		break;


		//--------------------------------------------------------------------------------------

	case (T6_STANDARD_IK_7DOF_MOVE_TO_OBJECT)		:

	ROS_WARN("T6_STANDARD_IK_7DOF_MOVE_TO_OBJECT");

	getLimits();
	goal_pose_GPTCP_.position.z = goal_pose_GPTCP_.position.z - height_over_belt_tuning_;

	// move to new pose
	ROS_WARN("Move to object");
	if(!T6_MoveIt_move_to_object())
	{
		msg_error("move to finale pose failed!");
		return false;
	}


	//wait time until rdv
	t_security_ = t_security_ - 0.01;
	//ROS_WARN_STREAM(t_rdv);
	//ROS_WARN_STREAM(ros::Time::now().sec);
	boost::this_thread::sleep( boost::posix_time::seconds(t_security_));

	ROS_WARN("Grab object");
	//grap object
	if(!T6_grap_object())
	{
		msg_error("grap object failed");
		return false;
	}

	// Berechne Pose über Belt und STANDARD IK
	ROS_WARN("Move to object safe");
	if(!T6_MoveIt_move_object_safe())
	{
		msg_error("move to pose above belt failed!");
		return false;
	}

	//execute Motion über Belt
	if(!executeStd())
	{
		msg_error("Execute motion failed");
		return false;
	}
	ROS_WARN("Move to object finished");

	ros::Duration(1).sleep();

	return true;

	break;
	//--------------------------------------------------------------------------------------

	case T6_MOVE_TARGET:

#if 0
		ROS_INFO("Choosed 9DOF");
		group = group_9DOF;
		joint_model_group_ = joint_model_group_9DOF_;
		// setting joint state target via the searchIKSolution srv is not considered
		max_setTarget_attempts_ = 3;


		//set target algorithm
		current_setTarget_algorithm_ = SINGLE_POSE_TARGET;

		group->setGoalPositionTolerance(tolerance_);

		// get moveit solution
		if (!T6_MoveIt_getSolution())
		{
			msg_error("No MoveIT Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No MoveIT Solution found.");
			return false;
		}

		if(!executeStd())
		{
			msg_error("Execute goal std failed!");
			return false;
		}
#endif

		getLimits();
		// get moveit solution
		if (!T6_moveToTarget())
		{
			msg_error("Move to target failed.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"Move to target failed.");
			return false;
		}
		//execute Motion über Belt
		if(!executeStd())
		{
			msg_error("Execute motion failed");
			return false;
		}
		ROS_WARN("Move to target finished");

		return true;
		break;

	default:
		msg_warn("unknown Mode in MotionPlanning! (T6)");
		return false;
	}

}

bool T6MotionPlanning::T6_grap_object()
{
	ROS_INFO("gripper_close_cb() running");
	double tmp_size;
	ros::param::get("/object_0_shape_0_size_0_",tmp_size);
	//gripper control
	gripper_control_srv_.request.gripping_mode = POSITION;
	gripper_control_srv_.request.gripper_position = tmp_size*0.8;

	if(gripper_control_client_.exists())
	{
		if(gripper_control_client_.call(gripper_control_srv_))
		{
			gripper_close_state_=FINISHED;
			msg_error("gripper closed succeed");

			//TODO
			ros::Duration(2.0).sleep();

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

bool T6MotionPlanning::T6_open_gripper()
{
	ROS_INFO("Open gripper.");

	//gripper control
	gripper_control_srv_.request.gripping_mode = POSITION;
	gripper_control_srv_.request.gripper_position = 0.07;

	if(gripper_control_client_.exists())
	{
		if(gripper_control_client_.call(gripper_control_srv_))
		{
			gripper_close_state_=FINISHED;
			ROS_INFO("gripper_open_cb() finished");
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

	ROS_INFO("gripper_open_cb() finished");
	return false;
}

//! based on Standard 7DoF Solution
bool T6MotionPlanning::T6_MoveIt_move_to_object()
{
	ROS_WARN("Goal Pose from Vision:");
	ROS_INFO_STREAM(goal_pose_GPTCP_.position);

	//get current endeffector position
	tf::TransformListener tf_listener;
	tf::StampedTransform transform_ORIGIN_2_GPTCP;
	std::string debug_tf;

	geometry_msgs::Pose current_pose;
	T6_getCurrentGPTCP_ORIGIN(current_pose);

	geometry_msgs::Pose current_pose_lwr;
	T6_getCurrentLWRTCP_LWR_0(current_pose_lwr);


	ROS_INFO_STREAM(save_goal_pose_GPTCP_.position);
	// is gripper in right position over belt?
	double delta_z = current_pose.position.z - save_goal_pose_GPTCP_.position.z;
	if(delta_z > 0.001)
	{
		//for homing position
		height_over_belt_ = height_over_belt_ - delta_z;

		ROS_INFO_STREAM("Current Position: z position is to height!");
		goal_pose_LWRTCP_ = current_pose_lwr;
		goal_pose_LWRTCP_.position.z = goal_pose_LWRTCP_.position.z - delta_z;

		if (!euroc_getIKSolution7DOF())
		{
			msg_error("No IK Solution found.");
			return false;
		}

		//execute STANDARD IK HOMING
		if(!T6_executeStd())
		{
			msg_error("Execute motion STANDARD IK HOMING failed!");
			return false;
		}
	}




	//consider time in goal pose --> set rendez vous point
	// Save pose
	gripping_pose_ = save_goal_pose_GPTCP_;
	T6_getNewPose(gripping_pose_);
	goal_pose_GPTCP_ = gripping_pose_;

	std::vector<double> tmp_pos;
	tmp_pos.resize(2);
	if(!T6_getSchlittenPosition(tmp_pos))
	{
		msg_error("Get current schlitten position failed!");
		return false;
	}

	geometry_msgs::Pose tmp_goal_pose_GPTCP_diff;
	tmp_goal_pose_GPTCP_diff.position.x = gripping_pose_.position.x - current_pose.position.x;
	tmp_goal_pose_GPTCP_diff.position.y = gripping_pose_.position.y - current_pose.position.y;

	// Aufteilung der diff Bewegung in senkrecht belt und gegen belt
	// neue pose senkrecht
	if((drop_center_point.x * mdl_norm.y - drop_center_point.y * mdl_norm.x) < 0) // uhrzeiger
	{
		//PARAMETER senk und parllel belt
		par_ = (tmp_goal_pose_GPTCP_diff.position.x * (-mdl_norm.x) - tmp_goal_pose_GPTCP_diff.position.y * mdl_norm.y)/((-mdl_norm.x) * (-mdl_norm.x) - (-mdl_norm.y) * mdl_norm.y);
		senk_ = (tmp_goal_pose_GPTCP_diff.position.x - par_ * (-mdl_norm.x))/(mdl_norm.y);


		// neue pose senkrecht
		goal_pose_GPTCP_.position.x = tmp_pos[0] + senk_ * mdl_norm.y;
		goal_pose_GPTCP_.position.y = tmp_pos[1] - senk_ * mdl_norm.x;
	}
	else
	{
		//PARAMETER senk und parllel belt
		par_ = (tmp_goal_pose_GPTCP_diff.position.x * (mdl_norm.x) - tmp_goal_pose_GPTCP_diff.position.y * (-mdl_norm.y))/((-mdl_norm.x) * (mdl_norm.x) - (-mdl_norm.y) * (-mdl_norm.y));
		senk_ = (tmp_goal_pose_GPTCP_diff.position.x - par_ * (-mdl_norm.x))/(-mdl_norm.y);


		// neue pose senkrecht
		goal_pose_GPTCP_.position.x = tmp_pos[0] - senk_ * mdl_norm.y;
		goal_pose_GPTCP_.position.y = tmp_pos[1] + senk_ * mdl_norm.x;
	}

	ROS_WARN("2 DOF Schlitten parallel");
	//ROS_INFO_STREAM(goal_pose_GPTCP_.position);
	// Save pose
	senkrecht_belt_pose_ = goal_pose_GPTCP_;

#if 0
if(0)
{
	ROS_WARN("MoveIT 2 DOF Schlitten senkrecht");
	ROS_INFO_STREAM(goal_pose_GPTCP_.position.x);
	ROS_INFO_STREAM(goal_pose_GPTCP_.position.y);

	if(!T6_MoveIt_getSolution_2DOF())
	{
		msg_error("No MoveIT 2 DOF Schlitten Solution senkrecht found.");
		goalPose_result_.reached_goal = false;
		goalPose_server_.setPreempted(goalPose_result_,"No MoveIT Solution found.");
		return false;
	}

	if(!T6_executeStd())
	{
		msg_error("Execute motion MoveIT 2 DOF Schlitten Solution senkrecht failed!");
		return false;
	}
}
#endif

	if(!T6_moveSchlitten())
	{
		msg_error("Execute motion 2 DOF Schlitten Solution senkrecht failed!");
		return false;
	}

	//	save Pose antiparallel

	gripping_pose_.position.x = goal_pose_GPTCP_.position.x - par_ * mdl_norm.x;
	gripping_pose_.position.y = goal_pose_GPTCP_.position.y - par_ * mdl_norm.y;

	// Set goal pose
	goal_pose_GPTCP_ = gripping_pose_;


	ROS_WARN("2 DOF Schlitten antiparallel");
	ROS_INFO_STREAM(goal_pose_GPTCP_.position);

#if 0
	if(0)
	{
		if(!T6_MoveIt_getSolution_2DOF())
		{
			msg_error("No MoveIT 2 DOF Schlitten Solution antiparallel found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No MoveIT Solution found.");
			return false;
		}

		if(!T6_executeStd())
		{
			msg_error("Execute motion MoveIT 2 DOF Schlitten Solution antiparallel failed!");
			return false;
		}
	}
#endif
	if(!T6_moveSchlitten())
	{
		msg_error("Execute motion 2 DOF Schlitten Solution senkrecht failed!");
		return false;
	}


	return true;
}



bool T6MotionPlanning::T6_MoveIt_move_object_safe()
{

	geometry_msgs::Pose current_pose;
	T6_getCurrentLWRTCP_LWR_0(current_pose);
	//! Save Pose über Belt

	over_belt_pose_transformed_ = current_pose;
	over_belt_pose_transformed_.position.z 		= over_belt_pose_transformed_.position.z + 0.1;

	goal_pose_LWRTCP_ = over_belt_pose_transformed_;

	//T6_send_poses_to_tf_broadcaster(goal_pose_LWRTCP_,true);
	//! Find IK solution
	if (!euroc_getIKSolution7DOF())
	{
		msg_error("No IK Solution found.");
		return false;
	}

	return true;
}

bool T6MotionPlanning::T6_MoveIt_homing()
{


	try
	{
		if (ros::service::waitForService(move_along_joint_path_,ros::Duration(10.0)))
		{

			//    Initiliazing MoveGroup
			if(!T6_MoveIt_initializeMoveGroup())
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


bool T6MotionPlanning::T6_moveToTarget()
{
	//ROS_INFO_STREAM("goal_pose "<<goal_pose_GPTCP_.position);
	ROS_WARN("Move to Schlitten Target!");
	std::vector<double> tmp_schlitten;
	tmp_schlitten.resize(2);
	if(!T6_getSchlittenPosition(tmp_schlitten))
	{
		msg_error("Didn't get schlitten position!");
		return false;
	}

	std::vector<double> tmp_vec_target_schlitten;
	tmp_vec_target_schlitten.resize(2);

	std::vector<double> tmp_vec_target_schlitten_norm;
	tmp_vec_target_schlitten_norm.resize(2);

	tmp_vec_target_schlitten[0] = tmp_schlitten[0] - target_pos.x;
	tmp_vec_target_schlitten[1] = tmp_schlitten[1] - target_pos.y;

	// normieren
	double norm_vec_target_schlitten = sqrt(tmp_vec_target_schlitten[0] * tmp_vec_target_schlitten[0] + tmp_vec_target_schlitten[1] * tmp_vec_target_schlitten[1]);
	if( !(norm_vec_target_schlitten < (target_zone_radius_ * factor_tolerance_7DoF_position_) + 0.5) || (norm_vec_target_schlitten < 0.1) )
	{
		tmp_vec_target_schlitten_norm[0] = tmp_vec_target_schlitten[0]/norm_vec_target_schlitten;
		tmp_vec_target_schlitten_norm[1] = tmp_vec_target_schlitten[1]/norm_vec_target_schlitten;

		goal_pose_GPTCP_.position.x = target_distance_ * tmp_vec_target_schlitten_norm[0] + target_pos.x;
		goal_pose_GPTCP_.position.y = target_distance_ * tmp_vec_target_schlitten_norm[1] + target_pos.y;


		if(!T6_moveSchlitten())
		{
			msg_error("Didn't move to schlitten target position!");
			return false;
		}

		ROS_WARN("Move to Schlitten Target finished!");
	}
	else
	{
		ROS_INFO_STREAM("Skip schlitten movement (norm = "<<norm_vec_target_schlitten<<" )");
	}

	ROS_WARN("MoveIT 7DOF Solution!");
	group = group_7DOF;
	joint_model_group_ = joint_model_group_7DOF_;
	group->setGoalPositionTolerance(tolerance_7DoF_position_);
	group->setGoalOrientationTolerance(tolerance_7DoF_orientation_);
	// in case of unsuccessful planning,
	// the planning target is set as a joint state goal via the searchIKSolution srv
	max_setTarget_attempts_ = 7;
	current_setTarget_algorithm_ = SINGLE_POSE_TARGET;
	// get moveit solution

	goal_pose_GPTCP_ = save_goal_pose_GPTCP_;
	// hier transform to lwr base, da moveit "nicht weiss" wie basis verschoben wurde
	// ROS_INFO_STREAM("goal_pose "<<goal_pose_GPTCP_.position);

	if (!T6_MoveIt_getSolution())
	{
		msg_error("No MoveIT Solution found.");
		ROS_WARN("Try euroc");

		//2 Schritte: 1) gleiche höhe des gp tcp 2) ablegen
		goal_pose_LWRTCP_ = save_goal_pose_LWRTCP_;

		//! Transform lwr goal pose to LWR0 Base frame
		if (!transformToLWRBase())
		{
			msg_warn("Transformation to LWR0 Base failed.");

			//                        goalPose_result_.reached_goal = false;
			//                        goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
			//                        goalPose_server_.setPreempted(goalPose_result_,"Transformation to LWR0 Base failed.");
			//                        return;
		}

		geometry_msgs::Pose tmp_pose_goal_pose_LWRTCP_LWR_0 = goal_pose_LWRTCP_;
		// 1) gleiche hoehe
		geometry_msgs::Pose current_pose;
		T6_getCurrentLWRTCP_LWR_0(current_pose);
		//! Standard IK Solution
		//goal_pose_LWRTCP_ = current_pose;
	//	goal_pose_LWRTCP_.position.x = tmp_pose_goal_pose_LWRTCP_LWR_0.position.x;
	//	goal_pose_LWRTCP_.position.y = tmp_pose_goal_pose_LWRTCP_LWR_0.position.y;

		goal_pose_LWRTCP_.position.z = current_pose.position.z;
		//! Find IK solution
//		ROS_WARN("Save Solution");
//		ROS_INFO_STREAM(goal_pose_LWRTCP_.position);
//		ROS_INFO_STREAM(goal_pose_LWRTCP_.orientation);
		inter_steps_ = 0;
		if (!euroc_getIKSolution7DOF())
		{
			msg_error("No IK Solution found.");
			//return false;
		}
		else
		{
			if(!T6_executeStd())
				{
					msg_error("Execute motion 7 DOF failed!");
					return false;
				}
		}

		//execute motion 2DOF
		// 2) ablegen
		ROS_WARN("ABLEGEN");
		//! Standard IK Solution
		goal_pose_LWRTCP_ = save_goal_pose_LWRTCP_;

		//! Transform lwr goal pose to LWR0 Base frame
		if (!transformToLWRBase())
		{
			msg_warn("Transformation to LWR0 Base failed.");

			//                        goalPose_result_.reached_goal = false;
			//                        goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
			//                        goalPose_server_.setPreempted(goalPose_result_,"Transformation to LWR0 Base failed.");
			//                        return;
		}
		//! Find IK solution
		inter_steps_ = 0;
		if (!euroc_getIKSolution7DOF())
		{
			msg_error("No IK Solution found.");
			return false;
		}

	}

#if 0 //EUROC SOLUTION
	//2 Schritte: 1) gleiche höhe des gp tcp 2) ablegen

	goal_pose_LWRTCP_ = save_goal_pose_LWRTCP_;

	//! Transform lwr goal pose to LWR0 Base frame
	if (!transformToLWRBase())
	{
		msg_warn("Transformation to LWR0 Base failed.");

		//                        goalPose_result_.reached_goal = false;
		//                        goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
		//                        goalPose_server_.setPreempted(goalPose_result_,"Transformation to LWR0 Base failed.");
		//                        return;
	}

	geometry_msgs::Pose tmp_pose_goal_pose_LWRTCP_LWR_0 = goal_pose_LWRTCP_;
	// 1) gleiche hoehe
	geometry_msgs::Pose current_pose;
	T6_getCurrentLWRTCP_LWR_0(current_pose);
	//! Standard IK Solution
	//goal_pose_LWRTCP_ = current_pose;
//	goal_pose_LWRTCP_.position.x = tmp_pose_goal_pose_LWRTCP_LWR_0.position.x;
//	goal_pose_LWRTCP_.position.y = tmp_pose_goal_pose_LWRTCP_LWR_0.position.y;

	goal_pose_LWRTCP_.position.z = current_pose.position.z;
	goal_pose_LWRTCP_.orientation = current_pose.orientation;


	//! Find IK solution
	ROS_WARN("Save Solution");
	ROS_INFO_STREAM(goal_pose_LWRTCP_.position);
	ROS_INFO_STREAM(goal_pose_LWRTCP_.orientation);
	inter_steps_ = 0;
	if (!euroc_getIKSolution7DOF())
	{
		msg_error("No IK Solution found.");
		//return false;
	}
	else
	{
		if(!T6_executeStd())
			{
				msg_error("Execute motion 7 DOF failed!");
				return false;
			}
	}

	//execute motion 2DOF


	// 2) ablegen
	ROS_WARN("ABLEGEN");
	//! Standard IK Solution
	goal_pose_LWRTCP_ = save_goal_pose_LWRTCP_;

	//! Transform lwr goal pose to LWR0 Base frame
	if (!transformToLWRBase())
	{
		msg_warn("Transformation to LWR0 Base failed.");

		//                        goalPose_result_.reached_goal = false;
		//                        goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
		//                        goalPose_server_.setPreempted(goalPose_result_,"Transformation to LWR0 Base failed.");
		//                        return;
	}
	//! Find IK solution
	inter_steps_ = 0;
	if (!euroc_getIKSolution7DOF())
	{
		msg_error("No IK Solution found.");
		return false;
	}
#endif

	return true;

}


bool T6MotionPlanning::T6_move_2DoF_Euroc()
{

	//! Berechne Euroc Pose
	geometry_msgs::Pose goal_config_homing_T6;
	goal_config_homing_T6 = goal_pose_GPTCP_;
	T6_calc_goalHoming_TCP(goal_config_homing_T6);
	ROS_INFO_STREAM("goal_config_homing_T6 "<<goal_config_homing_T6.position);

	// Save Euroc pose
	homing_belt_euroc_pose_origin_ = goal_config_homing_T6;

	//! Berechne 2 DOF Pose
	T6_calc_goalHoming_schlitten(homing_belt_schlitten_pose_ );
	ROS_INFO_STREAM("homing_belt_schlitten_pose_ "<<homing_belt_schlitten_pose_.position);

	//! Save tranformed pose
	homing_belt_euroc_pose_transformed_ = T6_transformLWRBase(homing_belt_euroc_pose_origin_,homing_belt_schlitten_pose_);
	ROS_INFO_STREAM("homing_belt_euroc_pose_transformed_ "<<homing_belt_euroc_pose_transformed_.position);

	std::vector<double> tmp_pos_schlitten;
	tmp_pos_schlitten.resize(2);

	//-------------------------------------------------------------------------------------------------
	// get moveit solution
	group = group_7DOF;
	joint_model_group_ = joint_model_group_7DOF_;
	// in case of unsuccessful planning
	// the planning target is set as a joint state goal via the searchIKSolution srv
	max_setTarget_attempts_ = 7;
	current_setTarget_algorithm_ = SINGLE_POSE_TARGET;


#if 1
	T6_getSchlittenPosition(tmp_pos_schlitten);

	goal_pose_GPTCP_ = homing_belt_euroc_pose_transformed_;
	goal_pose_GPTCP_.position.x = homing_belt_euroc_pose_transformed_.position.x + tmp_pos_schlitten[0];
	goal_pose_GPTCP_.position.y = homing_belt_euroc_pose_transformed_.position.y + tmp_pos_schlitten[1];

	ROS_INFO_STREAM("7DOF "<<goal_pose_GPTCP_.position);

	if(!T6_MoveIt_getSolution())
	{
		ROS_WARN("MOVEIT SOLUTION FAILED");

		ROS_INFO("HOMING MOVEIT 7DOF planning mode chosen.");
		group = group_7DOF;
		joint_model_group_ = joint_model_group_7DOF_;
		if(!T6_MoveIt_homing())
		{
			msg_error("No Solution found for homing moveit.");
			// STANDARD IK HOMING
			ROS_INFO("HOMING 7DOF");
			if (!euroc_setReset7DOF())
			{
				msg_error("No IK Solution found.");
				goalPose_result_.reached_goal = false;
				goalPose_server_.setPreempted(goalPose_result_,"No IK Solution found.");
				return false;
			}
		}
		//execute STANDARD IK HOMING
		if(!T6_executeStd())
		{
			msg_error("Execute motion STANDARD IK HOMING failed!");
			return false;
		}

		// 7DoF Solution
		ROS_INFO("T6 STANDARD IK 7DOF planning mode chosen.");

		T6_getSchlittenPosition(tmp_pos_schlitten);

		// Set goal pose
		goal_pose_GPTCP_ = goal_config_homing_T6;
		goal_pose_GPTCP_.position.x  = homing_belt_euroc_pose_transformed_.position.x + tmp_pos_schlitten[0];
		goal_pose_GPTCP_.position.y  = homing_belt_euroc_pose_transformed_.position.y + tmp_pos_schlitten[1];

		//hier eigentlich Fehler
		goal_pose_LWRTCP_ = goal_pose_GPTCP_;
		if (!transformToLWRBase())	{msg_warn("Transformation to LWR0 Base failed.");}

		ROS_INFO_STREAM("homing schlitten "<<homing_belt_schlitten_pose_.position);

		//! Find IK solution
		if (!euroc_getIKSolution7DOF())
		{
			msg_error("No T6 IK Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No IK Solution found.");
			return false;
		}
	}
#else

	ROS_INFO("HOMING MOVEIT 7DOF planning mode chosen.");
	if(!T6_MoveIt_homing())
	{
		msg_error("No Solution found for homing moveit.");
		// STANDARD IK HOMING
		ROS_INFO("HOMING 7DOF");
		if (!euroc_setReset7DOF())
		{
			msg_error("No IK Solution found.");
			goalPose_result_.reached_goal = false;
			goalPose_server_.setPreempted(goalPose_result_,"No IK Solution found.");
			return false;
		}
	}
	//execute STANDARD IK HOMING
	if(!T6_executeStd())
	{
		msg_error("Execute motion STANDARD IK HOMING failed!");
		return false;
	}


	// 7DoF Solution

	ROS_INFO("T6 STANDARD IK 7DOF planning mode chosen.");
	T6_getSchlittenPosition(tmp_pos_schlitten);

	// Set goal pose
	goal_pose_GPTCP_ = goal_config_homing_T6;
	goal_pose_GPTCP_.position.x  = homing_belt_euroc_pose_transformed_.position.x + tmp_pos_schlitten[0];
	goal_pose_GPTCP_.position.y  = homing_belt_euroc_pose_transformed_.position.y + tmp_pos_schlitten[0];


	//hier eigentlich Fehler
	goal_pose_LWRTCP_ = goal_pose_GPTCP_;
	if (!transformToLWRBase())	{msg_warn("Transformation to LWR0 Base failed.");}

	ROS_INFO_STREAM("homing schlitten "<<homing_belt_schlitten_pose_.position);

	//! Find IK solution
	if (!euroc_getIKSolution7DOF())
	{
		msg_error("No T6 IK Solution found.");
		goalPose_result_.reached_goal = false;
		goalPose_server_.setPreempted(goalPose_result_,"No IK Solution found.");
		return false;
	}
#endif
	//execute motion EUROC
	if(!T6_executeStd())
	{
		msg_error("Execute motion EUROC failed!");
		return false;
	}
	//---------------------------------------------------------------------------------

	//! get moveit solution for 2DoF

	//! Try 2 DOF + EUROC solution
	ROS_INFO("2 DOF SChlitten");
	group = group_2DOF;
	group->setGoalTolerance(0.0);
	joint_model_group_ = joint_model_group_2DOF_;

	//! set target algorithm
	current_setTarget_algorithm_ = SINGLE_POSE_TARGET;


	//! Set goal pose for 2DoF movement
	// Schlitten pose Target homing_belt_schlitten_pose_
	goal_pose_GPTCP_= homing_belt_schlitten_pose_;

	//if (!T6_MoveIt_getSolution())
	if(!T6_MoveIt_getSolution_2DOF())
	{
		msg_error("No MoveIT 2 DOF Schlitten Solution found.");
		goalPose_result_.reached_goal = false;
		goalPose_server_.setPreempted(goalPose_result_,"No MoveIT Solution found.");
		return false;
	}

	//execute motion 2DOF
	if(!T6_executeStd())
	{
		msg_error("Execute motion 2 DOF Schlitten failed!");
		return false;
	}

	goalPose_result_.reached_goal = true;
	goalPose_server_.setSucceeded(goalPose_result_, "Goal configuration has been reached");


	return true;
}

bool T6MotionPlanning::T6_moveSchlitten()
{
	if (ros::service::waitForService(move_along_joint_path_,ros::Duration(1.0)))
	{

		std::vector<double> target_2DoF;
		target_2DoF.resize(2);

		target_2DoF[0]=goal_pose_GPTCP_.position.x;
		target_2DoF[1]=goal_pose_GPTCP_.position.y;

		const std::string x_axis = "axis_x";
		const std::string y_axis = "axis_y";

		move_along_joint_path_srv_.request.joint_names.clear();
		move_along_joint_path_srv_.request.joint_names.push_back(x_axis);
		move_along_joint_path_srv_.request.joint_names.push_back(y_axis);

		move_along_joint_path_srv_.request.path.resize(1);
		move_along_joint_path_srv_.request.path[0].q.clear();
		move_along_joint_path_srv_.request.path[0].q.push_back(target_2DoF[0]);
		move_along_joint_path_srv_.request.path[0].q.push_back(target_2DoF[1]);


		move_along_joint_path_srv_.request.joint_limits.resize(2);
		move_along_joint_path_srv_.request.joint_limits[0].max_acceleration = table_axis1_limit_.max_acceleration;
		move_along_joint_path_srv_.request.joint_limits[0].max_velocity     = table_axis1_limit_.max_velocity;
		move_along_joint_path_srv_.request.joint_limits[1].max_acceleration = table_axis2_limit_.max_acceleration;
		move_along_joint_path_srv_.request.joint_limits[1].max_velocity     = table_axis2_limit_.max_velocity;


		if (estimated_motion_time_<goal_pose_goal_->allowed_time || goal_pose_goal_->allowed_time<0.5)
		{
			moveToTargetCB();

			//! get telemetry
			if (!getTelemetry()){
				msg_error("getTelemetry: An Error happened here.");
				goalPose_result_.reached_goal = false;
				goalPose_result_.error_reason = fsm::SIM_SRV_NA;
				goalPose_server_.setPreempted(goalPose_result_,"Got no telemetry.");

				return false;
			}

			return true;
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
	else
		ROS_WARN("Service move to target configuration has not been advertised yet.");
}

bool T6MotionPlanning::T6_executeStd()
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
			moveToTargetCB();
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


	//! get telemetry
	if (!getTelemetry()){
		msg_error("getTelemetry: An Error happened here.");
		goalPose_result_.reached_goal = false;
		goalPose_result_.error_reason = fsm::SIM_SRV_NA;
		goalPose_server_.setPreempted(goalPose_result_,"Got no telemetry.");

		return false;
	}


	return true;
}

// based on grasping code
bool T6MotionPlanning::T6_calc_goalHoming_TCP(geometry_msgs::Pose &goal_config_homing_T6)
{
	ROS_INFO("compute_grasp_posesT6_() called");

	geometry_msgs::Pose tmp_GPTCP_pose;

	tf::Vector3 x_axis(1,0,0), y_axis(0,1,0), z_axis(0,0,1);
	tf::Vector3 x_grp, y_grp, z_grp;
	tf::Vector3 tmp_vec;
	tf::Vector3 safe_to_grip_dir;
	tf::Matrix3x3 tmpRotation;

	z_grp = -1.0*z_axis;
	y_grp=conveyor_belt_move_direction_and_length;
	y_grp=y_grp/y_grp.length();
	y_grp.setZ(0);	//only use rotation around vertical axis
	x_grp = y_grp.cross(z_grp);

	tmpRotation=T6_get_rotationmatrixfromaxis(x_grp,gripping_angle_rad_);
	y_grp=tmpRotation*y_grp;
	z_grp=tmpRotation*z_grp;
	T6_set_orientation_from_axes(tmp_GPTCP_pose,x_grp,y_grp,z_grp);


	//set orientation
	goal_config_homing_T6.orientation.w = tmp_GPTCP_pose.orientation.w;
	goal_config_homing_T6.orientation.x = tmp_GPTCP_pose.orientation.x;
	goal_config_homing_T6.orientation.y = tmp_GPTCP_pose.orientation.y;
	goal_config_homing_T6.orientation.z = tmp_GPTCP_pose.orientation.z;

	//set position dependend on distance
	//	ros::param::get("/object_counter_", n_objects_);
	goal_config_homing_T6.position.x = drop_center_point.x + home_faktor_ * mdl_norm.x; // TODO Faktor
	goal_config_homing_T6.position.y = drop_center_point.y + home_faktor_ * mdl_norm.y; // TODO Faktor
	goal_config_homing_T6.position.z = drop_center_point.z + height_over_belt_;

	// Print goal
	ROS_WARN("POSE + RPY");
	ROS_INFO_STREAM(goal_config_homing_T6.position.x);
	ROS_INFO_STREAM(goal_config_homing_T6.position.y);
	ROS_INFO_STREAM(goal_config_homing_T6.position.z);
	ROS_INFO_STREAM(winkel_roll);
	ROS_INFO_STREAM(winkel_pitch);
	ROS_INFO_STREAM(winkel_yaw);

	return true;
}


bool T6MotionPlanning::T6_calc_goalHoming_schlitten(geometry_msgs::Pose& goal_config_homing_T6_schlitten)
{
	//	ros::param::get("/object_counter_", n_objects_);
	if((drop_center_point.x * mdl_norm.y - drop_center_point.y * mdl_norm.x) < 0) // uhrzeiger
	{
		// Pose schlitten
		goal_config_homing_T6_schlitten.position.x = drop_center_point.x + home_faktor_ * mdl_norm.x + standard_distance_dcp_planar_axis_ * mdl_norm.y;
		goal_config_homing_T6_schlitten.position.y = drop_center_point.y + home_faktor_ * mdl_norm.y - standard_distance_dcp_planar_axis_ * mdl_norm.x;
	}
	else
	{
		// Pose schlitten
		goal_config_homing_T6_schlitten.position.x = drop_center_point.x + home_faktor_ * mdl_norm.x - standard_distance_dcp_planar_axis_ * mdl_norm.y;
		goal_config_homing_T6_schlitten.position.y = drop_center_point.y + home_faktor_ * mdl_norm.y + standard_distance_dcp_planar_axis_ * mdl_norm.x;
	}


	goal_config_homing_T6_schlitten.position.z =0;
	goal_config_homing_T6_schlitten.orientation.w=1;
	goal_config_homing_T6_schlitten.orientation.x=0;
	goal_config_homing_T6_schlitten.orientation.y=0;
	goal_config_homing_T6_schlitten.orientation.z=0;

//	ROS_WARN("Schlitten pose");
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

	return true;

}




bool T6MotionPlanning::T6_getBeltSpeed()
{
	//	ros::param::get("/object_counter_", n_objects_);
	ros::param::get("/conveyorbelt_start_speed_", start_speed_);
	ros::param::get("/conveyorbelt_end_speed_", end_speed_);

	// belt speed
	belt_speed_ = start_speed_ + (n_objects_-1)*((end_speed_-start_speed_)/(n_obj_ges_-1));


	if (belt_speed_>0)
	{
		mdl.x = mdl_norm.x * belt_speed_;
		mdl.y = mdl_norm.y * belt_speed_;
		mdl.z = mdl_norm.z * belt_speed_;

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

		//		ros::param::get("/object_counter_", n_objects_);

		// TODO adaptiv T6_puffer_pose
		T6_puffer_pose_ = 0;//1 + n_objects_ * 0.1;

		geometry_msgs::Pose old_pose; // ziel position
		old_pose = pose;

		//ROS_INFO_STREAM("old_pose "<<old_pose.position);

		// stamp; x + (t2 - t1)*v + puffer;
		pose.position.x = old_pose.position.x + (ros::Time::now().sec - time_stamp_in_) * mdl.x + mdl_norm.x * T6_puffer_pose_; // TODO
		pose.position.y = old_pose.position.y + (ros::Time::now().sec - time_stamp_in_) * mdl.y + mdl_norm.y * T6_puffer_pose_; // TODO
		pose.position.z = old_pose.position.z + (ros::Time::now().sec - time_stamp_in_) * mdl.z + mdl_norm.z * T6_puffer_pose_; // TODO

		//ROS_INFO_STREAM("gripping_pose "<<pose.position);

		double delta_s = sqrt( (pose.position.x - old_pose.position.x)*(pose.position.x - old_pose.position.x)
				+ (pose.position.y - old_pose.position.y)*(pose.position.y - old_pose.position.y)
				+ (pose.position.z - old_pose.position.z)*(pose.position.z - old_pose.position.z) );

		double v = sqrt(pow(mdl.x,2) + pow(mdl.y,2) + pow(mdl.z,2));
		//ROS_INFO_STREAM("v "<<v<<" t_diff "<<ros::Time::now().sec - time_stamp_in_);
		t_rdv = delta_s / v; // TODO

		return true;
	}
	else
	{
		msg_error("Didn't get belt speed");
		return false;
	}
}

void T6MotionPlanning::T6_initializeConveyorBelt()
{
	ROS_INFO("Adding conveyor belt and target zone to collision world...");


	//************************
	// conveyer belt

	T6_zWinkel();


	shape_msgs::SolidPrimitive con_belt;
	con_belt.type = con_belt.BOX;
	con_belt.dimensions.resize(3);

	con_belt.dimensions[0] = 2 * drop_deviation.y;
	con_belt.dimensions[1] = 2 * sqrt(pow(mdl_norm.x,2) + pow(mdl_norm.y,2)) + 0.1;
	con_belt.dimensions[2] = drop_center_point.z * 0.75 ; // TODO TEST then take out scalar and make it to variable

	geometry_msgs::Pose con_belt_pose;
	con_belt_pose.position.x = drop_center_point.x + mdl_norm.x;
	con_belt_pose.position.y = drop_center_point.y + mdl_norm.y;
	con_belt_pose.position.z = 0.05;//-0.01;
	con_belt_pose.orientation.x = q_belt.x();
	con_belt_pose.orientation.y = q_belt.y();
	con_belt_pose.orientation.z = q_belt.z();
	con_belt_pose.orientation.w = q_belt.w();

	moveit_msgs::CollisionObject con_belt_object;
	con_belt_object.id = "con_belt";
	con_belt_object.header.frame_id = "/Origin";
	con_belt_object.header.stamp = ros::Time::now();
	con_belt_object.operation = con_belt_object.ADD;
	con_belt_object.primitives.push_back(con_belt);
	con_belt_object.primitive_poses.push_back(con_belt_pose);

	static_scene_.world.collision_objects.push_back(con_belt_object);

	//************************
	// target zone


	shape_msgs::SolidPrimitive target_zone_col;



	target_zone_col.type = target_zone_col.CYLINDER;
	target_zone_col.dimensions.resize(2);

	target_zone_col.dimensions[0] = 0.02; // height
	target_zone_col.dimensions[1] = 0.15; // radius


	geometry_msgs::Pose target_zone_pose;
	target_zone_pose.position.x =  target_pos.x;
	target_zone_pose.position.y =  target_pos.y;
	target_zone_pose.position.z = 0.0;//-0.01;
	target_zone_pose.orientation.x = 0;
	target_zone_pose.orientation.y = 0;
	target_zone_pose.orientation.z = 0;
	target_zone_pose.orientation.w = 1;

	moveit_msgs::CollisionObject target_zone_object;
	target_zone_object.id = "target_zone_object";
	target_zone_object.header.frame_id = "/Origin";
	target_zone_object.header.stamp = ros::Time::now();
	target_zone_object.operation = target_zone_object.ADD;
	target_zone_object.primitives.push_back(target_zone_col);
	target_zone_object.primitive_poses.push_back(target_zone_pose);

	static_scene_.world.collision_objects.push_back(target_zone_object);

	//************************



	static_scene_.is_diff = true;
	planning_scene_diff_publisher_.publish(static_scene_);
	//static_scene_.world.collision_objects.clear();

	ROS_INFO("Finished adding conveyor belt and target zone .");

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

bool T6MotionPlanning::T6_MoveIt_getSolution_2DOF()
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

				std::vector<double> target_2DoF;
				target_2DoF.resize(joint_model_group_->getActiveJointModels().size());
				if(target_2DoF.size() != 2)
				{
					msg_error("number of active joints not equal to joint goal for 2DoF");
					return false;
				}
				target_2DoF[0]=goal_pose_GPTCP_.position.x;
				target_2DoF[1]=goal_pose_GPTCP_.position.y;

				ROS_INFO_STREAM("target 2DoF");
				ROS_INFO_STREAM(target_2DoF[0]);
				ROS_INFO_STREAM(target_2DoF[1]);



				if (!group->setJointValueTarget(target_2DoF))
				{
					ROS_ERROR("Setting joint value target failed.");
					return false;
				}
				else
				{
					setTarget_successful = true;
				}


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
	std::vector<std::string> joint_namesMI = group_9DOF->getActiveJoints();

	// print the joint names of the move group
	ROS_INFO("Joint names of the current move group:");
	if (!joint_namesMI.empty())
	{

	}
	else
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
				ROS_WARN_STREAM("MoveIt! joint not found in telemetry"<<joint_namesMI.at(idxMI));
				goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
				return false;
			}
		}


		// store the telemetry joint positions in the corresponding order of the move group
		joint_positionsMI.push_back(_telemetry.measured.position[idxTELE]);
		joint_velocitiesMI.push_back(0.0);
		//                // increase counter of matches between telemetry joints and MoveIt joints
		//                ROS_INFO_STREAM("number of joint matches: " << joint_positionsMI.size());
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

	group->setPlanningTime(allowed_planning_time_);


	// OCTOMAP
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	kinematic_model_ = robot_model_loader.getModel();


	// Load current robot state into planning scene
	planning_scene_.robot_state.joint_state = getCurrentJointState();
	planning_scene_.robot_state.is_diff = true;


	// setting planning scene message to type diff
	planning_scene_.is_diff = true;

	// Publish msg on topic /planning_scene
	planning_scene_diff_publisher_.publish(planning_scene_);
	planning_scene_diff_publisher_.publish(static_scene_);

	//==========================================================================================

	return true;

}


bool T6MotionPlanning::T6_getSchlittenPosition(std::vector<double> &schlitten_position)
{
	const std::string x_axis = "axis_x";
	const std::string y_axis = "axis_y";

	unsigned idxTELE = 0;
	while (x_axis.compare(_telemetry.joint_names.at(idxTELE)))
	{
		idxTELE++;
		if (idxTELE > _telemetry.joint_names.size()-1)
		{
			ROS_WARN("MoveIt! joint not found in telemetry");
			goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
			return false;
		}
	}

	schlitten_position[0] = _telemetry.measured.position[idxTELE];

	unsigned idyTELE = 0;
	while (y_axis.compare(_telemetry.joint_names.at(idyTELE)))
	{
		idyTELE++;
		if (idyTELE > _telemetry.joint_names.size()-1)
		{
			ROS_WARN("MoveIt! joint not found in telemetry");
			goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
			return false;
		}
	}

	schlitten_position[1] = _telemetry.measured.position[idyTELE];
	return true;
}


void T6MotionPlanning::T6_getParam()
{
	ros::param::get("/skip_vision", skip_vision_);

	// get start and end speed of param server
	ros::param::get("/conveyorbelt_start_speed_", start_speed_);
	ros::param::get("/conveyorbelt_end_speed_", end_speed_);

	//Targetzone
	ros::param::get("/target_zone_0_x_", target_pos.x);
	ros::param::get("/target_zone_0_y_", target_pos.y);
	ros::param::get("/target_zone_0_radius_", target_zone_radius_);

	tolerance_7DoF_position_ = factor_tolerance_7DoF_position_*target_zone_radius_;
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




	//get conveyor belt move direction and length
	conveyor_belt_move_direction_and_length.setValue(mdl.x,mdl.y,mdl.z);
	ROS_INFO("conveyor_belt_move_direction_and_length=[%4.3f %4.3f %4.3f]",
			conveyor_belt_move_direction_and_length.getX(),conveyor_belt_move_direction_and_length.getY(),conveyor_belt_move_direction_and_length.getZ());

}

// for belt
void T6MotionPlanning::T6_zWinkel()
{
	// Winkel zwischen neg. y-Achse Origin und geschwindigkeit belt
	winkel_kl = fabs(acos((0*mdl_norm.x + (-1)*mdl_norm.y)/(sqrt(pow(mdl_norm.x,2) + pow(mdl_norm.y,2))))); // Winkel zu x_achse
	winkel_gr =  2 * M_PI - winkel_kl;

	if(mdl_norm.x >= 0.0) // x_vel belt >0
	{
		// Quarternionen Belt
		q_belt.setRPY(0.0,0.0,winkel_kl);
		// RPY
		winkel_roll = -2.0/3.0 * M_PI;
		winkel_pitch = 0;
		winkel_yaw = winkel_kl;
	}
	else
	{
		// Winkel muss zwischen +pi und -pi sein
		winkel_gr = winkel_gr - 2 * M_PI;
		// Quarternionen Belt
		q_belt.setRPY(0.0,0.0,winkel_gr);
		// RPY
		winkel_roll = -2.0/3.0 * M_PI;
		winkel_pitch = 0;
		winkel_yaw = winkel_gr;
	}

	// Quarternionen für goalHoming
	q_goalHoming.setRPY(winkel_roll, winkel_pitch, winkel_yaw );
}


tf::Matrix3x3 T6MotionPlanning::T6_get_rotationmatrixfromaxis(tf::Vector3 axis, double angle_rad)
{
	tf::Vector3 normalized_axis;
	tf::Matrix3x3 tmpMat;
	double n1, n2, n3;
	double xx, xy, xz, yx, yy, yz, zx, zy, zz;

	//set identity as default
	xx=1; xy=0;	xz=0;
	yx=0; yy=1; yz=0;
	zx=0; zy=0; zz=1;

	if (axis.length()>0)
	{
		normalized_axis = 1.0/(axis.length())*axis;
		n1=normalized_axis.getX();
		n2=normalized_axis.getY();
		n3=normalized_axis.getZ();
		xx=n1*n1*(1-cos(angle_rad))+cos(angle_rad);
		xy=n1*n2*(1-cos(angle_rad))-n3*sin(angle_rad);
		xz=n1*n3*(1-cos(angle_rad))+n2*sin(angle_rad);
		yx=n2*n1*(1-cos(angle_rad))+n3*sin(angle_rad);
		yy=n2*n2*(1-cos(angle_rad))+cos(angle_rad);
		yz=n2*n3*(1-cos(angle_rad))-n1*sin(angle_rad);
		zx=n3*n1*(1-cos(angle_rad))-n2*sin(angle_rad);
		zy=n3*n2*(1-cos(angle_rad))+n1*sin(angle_rad);
		zz=n3*n3*(1-cos(angle_rad))+cos(angle_rad);
	}

	tmpMat.setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
	return tmpMat;
}

void T6MotionPlanning::T6_set_orientation_from_axes(geometry_msgs::Pose &tmp_pose, tf::Vector3 x_axis, tf::Vector3 y_axis, tf::Vector3 z_axis)
{
	double roll,pitch,yaw;
	tf::Matrix3x3 dcm;
	tf::Quaternion q_tmp;

	//	dcm.setValue(x_axis.getX(),x_axis.getY(),x_axis.getZ(),y_axis.getX(),y_axis.getY(),y_axis.getZ(),
	//				 z_axis.getX(),z_axis.getY(),z_axis.getZ());
	//transposed matrix seems to be the right one :)
	dcm.setValue(x_axis.getX(),y_axis.getX(),z_axis.getX(),x_axis.getY(),y_axis.getY(),z_axis.getY(),
			x_axis.getZ(),y_axis.getZ(),z_axis.getZ());
	dcm.getRPY(roll,pitch,yaw);
	q_tmp.setRPY(roll,pitch,yaw);

	tmp_pose.orientation.x = q_tmp.getX();
	tmp_pose.orientation.y = q_tmp.getY();
	tmp_pose.orientation.z = q_tmp.getZ();
	tmp_pose.orientation.w = q_tmp.getW();
}

tf::Transform T6MotionPlanning::T6_pose_to_transform(geometry_msgs::Pose input_pose)
{
	tf::Transform tmptransform;
	tf::Vector3 tmpvec;
	tf::Quaternion tmpquat;

	tmpvec.setValue(input_pose.position.x,input_pose.position.y,input_pose.position.z);
	tmpquat.setValue(input_pose.orientation.x,input_pose.orientation.y,input_pose.orientation.z,input_pose.orientation.w);

	tmptransform.setOrigin(tmpvec);
	tmptransform.setRotation(tmpquat);

	return tmptransform;
}

geometry_msgs::Pose T6MotionPlanning::T6_transformLWRBase(const geometry_msgs::Pose goal_config_homing_T6,
															const geometry_msgs::Pose goal_config_homing_T6_schlitten)
{
	geometry_msgs::Pose tmp;
	tmp = goal_config_homing_T6;

	tmp.position.x = goal_config_homing_T6.position.x - goal_config_homing_T6_schlitten.position.x;
	tmp.position.y = goal_config_homing_T6.position.y - goal_config_homing_T6_schlitten.position.y;


//	// Print goal
//	ROS_WARN("Tranformierte POSE + Quarternionen");
//	ROS_INFO_STREAM(tmp.position.x);
//	ROS_INFO_STREAM(tmp.position.y);
//	ROS_INFO_STREAM(tmp.position.z);
//	ROS_INFO_STREAM(tmp.orientation.x);
//	ROS_INFO_STREAM(tmp.orientation.y);
//	ROS_INFO_STREAM(tmp.orientation.z);
//	ROS_INFO_STREAM(tmp.orientation.w);


	return tmp;
}

void T6MotionPlanning::T6_send_poses_to_tf_broadcaster(geometry_msgs::Pose input_pose,bool frame)
{
	ROS_INFO("send_poses_to_tf_broadcaster() called");

	tf::Vector3 tmp_Origin;
	tf::Quaternion tmp_Rotation;
	tf::Transform tmp_transform;
	tf::StampedTransform tmp_stampedtransform;
	std::stringstream posename;

	//send grip pose
	tmp_transform=T6_pose_to_transform(input_pose);
	posename.str("");
	posename << "TCP_POSE_"<<pose_counter;
	if(frame)
		tmp_stampedtransform=tf::StampedTransform(tmp_transform,ros::Time::now(),LWR_0,posename.str().c_str());
	else
		tmp_stampedtransform=tf::StampedTransform(tmp_transform,ros::Time::now(),ORIGIN,posename.str().c_str());
	br.sendTransform(tmp_stampedtransform);

	pose_counter++;

}

bool T6MotionPlanning::T6_getCurrentGPTCP_ORIGIN(geometry_msgs::Pose &current_pose)
{


	tf::TransformListener tf_listener;
	tf::StampedTransform transform_ORIGIN_2_GPTCP;
	std::string debug_tf;
	//! Transformation from GP TCP frame to LWR TCP frame
	try{
		if (tf_listener.waitForTransform(ORIGIN,GP_TCP,ros::Time(0),ros::Duration(1.0),ros::Duration(3.0),&debug_tf))
			tf_listener.lookupTransform(ORIGIN,GP_TCP,ros::Time(0),transform_ORIGIN_2_GPTCP);
		else{
			msg_error("Could not get ransform_GPTCP_2_LWRTCP.");
			std::cout<<debug_tf<<std::endl;
			return false;
		}
	}
	catch(...){
		ROS_ERROR("Listening to transform was not successful");
		return false;
	}
	//! Save Pose über Belt
	current_pose.position.x 		= transform_ORIGIN_2_GPTCP.getOrigin().x();
	current_pose.position.y 		= transform_ORIGIN_2_GPTCP.getOrigin().y();
	current_pose.position.z 		= transform_ORIGIN_2_GPTCP.getOrigin().z();
	current_pose.orientation.x 		= transform_ORIGIN_2_GPTCP.getRotation().x();
	current_pose.orientation.y 		= transform_ORIGIN_2_GPTCP.getRotation().y();
	current_pose.orientation.z 		= transform_ORIGIN_2_GPTCP.getRotation().z();
	current_pose.orientation.w 		= transform_ORIGIN_2_GPTCP.getRotation().w();

	ROS_INFO_STREAM("Current Position GP_TCP in ORIGIN!");
	ROS_INFO_STREAM(current_pose.position);

	return true;
}

bool T6MotionPlanning::T6_getCurrentLWRTCP_LWR_0(geometry_msgs::Pose &current_pose)
{

	tf::TransformListener tf_listener;
	tf::StampedTransform transform_LWR0_2_LWRTCP;
	std::string debug_tf;
	//! Transformation from GP TCP frame to LWR TCP frame
	try{
		if (tf_listener.waitForTransform(LWR_0,LWR_TCP,ros::Time(0),ros::Duration(1.0),ros::Duration(3.0),&debug_tf))
			tf_listener.lookupTransform(LWR_0,LWR_TCP,ros::Time(0),transform_LWR0_2_LWRTCP);
		else{
			msg_error("Could not get ransform_GPTCP_2_LWRTCP.");
			std::cout<<debug_tf<<std::endl;
			return false;
		}
	}
	catch(...){
		ROS_ERROR("Listening to transform was not successful");
		return false;
	}

	//! Save Pose über Belt
	current_pose.position.x 		= transform_LWR0_2_LWRTCP.getOrigin().x();
	current_pose.position.y 		= transform_LWR0_2_LWRTCP.getOrigin().y();
	current_pose.position.z 		= transform_LWR0_2_LWRTCP.getOrigin().z();
	current_pose.orientation.x 		= transform_LWR0_2_LWRTCP.getRotation().x();
	current_pose.orientation.y 		= transform_LWR0_2_LWRTCP.getRotation().y();
	current_pose.orientation.z 		= transform_LWR0_2_LWRTCP.getRotation().z();
	current_pose.orientation.w 		= transform_LWR0_2_LWRTCP.getRotation().w();

	ROS_INFO_STREAM("Current Position LWR_TCP in LWR_0!");
	ROS_INFO_STREAM(current_pose.position);

	return true;
}




bool T6MotionPlanning::T6_getJointPosition(const std::string joint_name, double& joint_position)
{
	unsigned idxTELE = 0;
	while (joint_name.compare(_telemetry.joint_names.at(idxTELE)))
	{
		idxTELE++;
		if (idxTELE > _telemetry.joint_names.size()-1)
		{
			ROS_WARN("MoveIt! joint not found in telemetry");
			goalPose_result_.error_reason = fsm::MOTION_PLANNING_ERROR;
			return false;
		}
	}
	joint_position = _telemetry.measured.position[idxTELE];

	return true;
}

