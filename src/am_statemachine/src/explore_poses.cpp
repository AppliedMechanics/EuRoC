/*
 * explore_poses.cpp
 *
 *  Created on: Oct 28, 2014
 *      Author: euroc_admin
 */

#include <explore_poses.h>

ExplorePoses::ExplorePoses()
{
	block_counter_ = 0;
	insideblock_success_counter_ = 0;
	insideblock_counter_ = 0;
	success_counter_ = 0;
	first_block_failed_=false;

	/* initialize random seed: */
	srand (time(NULL));

	randomSort();

	init_std_v1();
	init_std_v2();
	init_snake();
	init_outer_snake();

	//	pose_blocks_.resize(N_POSE_BLOCKS);
	////	pose_blocks_[0].push_back(0);
	//	pose_blocks_[0].push_back(1);
	//	pose_blocks_[0].push_back(2);
	//	pose_blocks_[0].push_back(3);
	//	pose_blocks_[0].push_back(4);
	//
	////	pose_blocks_[1].push_back(5);
	//	pose_blocks_[1].push_back(6);
	//	pose_blocks_[1].push_back(7);
	//	pose_blocks_[1].push_back(8);
	//	pose_blocks_[1].push_back(9);
	//
	//	pose_blocks_[2].push_back(10);
	//	pose_blocks_[2].push_back(11);
	//	pose_blocks_[2].push_back(12);
	//	pose_blocks_[2].push_back(13);
	//
	//	pose_blocks_[3].push_back(14);
	//	pose_blocks_[3].push_back(15);
	//
	//	pose_blocks_[4].push_back(16);
	//	pose_blocks_[4].push_back(17);
	//
	//	pose_blocks_[5].push_back(18);
	//	pose_blocks_[5].push_back(19);
	//
	//	pose_blocks_[6].push_back(20);
	//	pose_blocks_[6].push_back(21);
	//	pose_blocks_[6].push_back(22);
	//	pose_blocks_[6].push_back(23);
	//
	//	rand_pose_block_vec_.clear();
	//	for (int jj=0;jj<N_POSE_BLOCKS;jj++)
	//	{
	//		for (int ii=0;ii<(pose_blocks_[block_nr_[jj]].size());ii++){
	//			rand_pose_block_vec_.push_back(pose_blocks_[block_nr_[jj]].at(ii));
	//			ROS_INFO("Pose Numbers: block nr %i pose nr %i",jj,pose_blocks_[block_nr_[jj]].at(ii));
	//		}
	//	}
	ROS_INFO("Random Block sorting: %i %i %i %i %i %i %i",block_nr_[0],block_nr_[1],block_nr_[2],block_nr_[3],block_nr_[4],block_nr_[5],block_nr_[6]);

}

ExplorePoses::~ExplorePoses() {
	// TODO Auto-generated destructor stub
}

int ExplorePoses::size(uint8_t pose_type)
{
	int snake_size;
	switch (pose_type)
	{
	case EXPLORE_STD_1:
		return explore_poses_std_v1_.size();
		break;
	case EXPLORE_STD_2:
		return explore_poses_std_v2_.size();
		break;
	case EXPLORE_OUTER_SNAKE:
		return explore_poses_outer_snake_.size();
		break;
	case EXPLORE_SNAKE:
		snake_size = 0;
		for (int ii=0;ii<N_POSE_BLOCKS;ii++)
			snake_size += explore_poses_snake_[ii].size();
		return snake_size;
		break;
	default:
		msg_error("Unknown Pose Type requested.");
		return 0;
		break;
	}
	return -1;

}

am_msgs::goalPoseGoal ExplorePoses::getExploreGoalPose(uint8_t pose_nr,uint8_t pose_type, uint8_t success_count)
{

	if (pose_nr == 0)
	{
		block_counter_ = 0;
		insideblock_success_counter_ = 0;
		insideblock_counter_ = 0;
		success_counter_ = 0;

		//! Explore outer snake
		if (pose_type == EXPLORE_OUTER_SNAKE)
			randomSortOuterSnake();
	}

	try{
		switch (pose_type)
		{
		case EXPLORE_STD_1:
			if (pose_nr < explore_poses_std_v1_.size())
				return explore_poses_std_v1_[pose_nr];
			else{
				msg_warn("Requested Pose Nr not available. Returning last pose.");
				return explore_poses_std_v1_[0];
			}
			break;
		case EXPLORE_STD_2:
			if (pose_nr < explore_poses_std_v2_.size())
				return explore_poses_std_v2_[pose_nr];
			else{
				msg_warn("Requested Pose Nr not available. Returning last pose.");
				return explore_poses_std_v2_[0];
			}
			break;
		case EXPLORE_OUTER_SNAKE:
			if (pose_nr < explore_poses_outer_snake_.size())
				return explore_poses_outer_snake_[pose_nr];
			else{
				msg_warn("Requested Pose Nr not available. Returning last pose.");
				return explore_poses_outer_snake_[0];
			}
			break;
		case EXPLORE_SNAKE:
			if (pose_nr < size(EXPLORE_SNAKE)) {
				// if at end of block, move to next block
				while (insideblock_counter_>=explore_poses_snake_[block_nr_[block_counter_]].size() && block_counter_<N_POSE_BLOCKS){
					block_counter_++;
					insideblock_counter_=0;
					insideblock_success_counter_ = 0;
					msg_info("Reached end of the block. Trying next.");
				}
				if (block_counter_<N_POSE_BLOCKS){
					if (block_counter_==1 && success_count==0)
					{
						first_block_failed_ = true;
						randomSort();
						first_block_failed_ = false;
					}
					else
					{
						// increase own success counter
						if (success_count > success_counter_){
							success_counter_++;
							insideblock_success_counter_++;
							msg_info("Increasing success counter.");
						}
						// if success counter = 2 --> next block
						if (insideblock_success_counter_ > 1){
							block_counter_++;
							insideblock_counter_=0;
							insideblock_success_counter_ = 0;
							msg_info("two successful poses. next block.");
						}
					}
				}
				else
				{
					msg_warn("maximum number of poses reached. resetting poses");
					block_counter_ = 0;
					insideblock_success_counter_ = 0;
					insideblock_counter_ = 0;
				}
				ROS_INFO("Block Counter %i inside Block Counter %i insideblocksucc %i success counter %i",block_counter_,insideblock_counter_,insideblock_success_counter_,success_counter_);

				if (insideblock_counter_>0 && insideblock_success_counter_>0)
				{
					tmp_goal_ = explore_poses_snake_[block_nr_[block_counter_]].at(insideblock_counter_);
					tmp_goal_.allowed_time = 15.0;
				}
				else
				{
					tmp_goal_ = explore_poses_snake_[block_nr_[block_counter_]].at(insideblock_counter_);
					tmp_goal_.allowed_time = 45.0;
					// Could insert the 2 DOF planning here?
				}
				insideblock_counter_++;

				return tmp_goal_;
			}
			else{
				msg_warn("Requested Pose Nr not available. Returning first pose.");
				return explore_poses_snake_[block_nr_[0]].at(0);
			}
			break;
		default:
			msg_warn("Pose Type unknown. Returning Standard V1 Pose.");
			if (pose_nr < explore_poses_std_v1_.size())
				return explore_poses_std_v1_[pose_nr];
			else{
				msg_warn("Requested Pose Nr not available. Returning first pose.");
				return explore_poses_std_v1_[0];
			}
			break;
		}
	}
	catch (...)
	{
		msg_error("Something Wrong happenend here.");
		ROS_WARN("Block Counter %i inside Block Counter %i insideblocksucc %i success counter %i",block_counter_,insideblock_counter_,insideblock_success_counter_,success_counter_);
		ROS_WARN("Request: pose_nr %i pose_type %i success %i",pose_nr,pose_type,success_count);
		return explore_poses_snake_[block_nr_[0]].at(0);
	}
}


void ExplorePoses::randomSort()
{
	//block_nr_[0] = rand() % 6;         // blocks_[0] in the range 0 to 5
	int begin_random = 1;
	if (first_block_failed_)
	{
		begin_random = 2;
		if (block_nr_[0]==0)
			block_nr_[1] = 1;
		else
			block_nr_[1] = 0;
	}
	else
	{
		block_nr_[0] = rand() % 2;
	}

	for (int i=begin_random;i<N_POSE_BLOCKS;i++)
	{
		do{block_nr_[i] = rand() % 7;}
		while(checkDoublettes(i));
	}

}

void ExplorePoses::randomSortOuterSnake()
{
	//block_nr_[0] = rand() % 6;

	if (rand() % 2)
	{
		block_nr_[0] = 0;
		block_nr_[1] = 6;
		block_nr_[2] = 1;
	}
	else
	{
		block_nr_[0] = 1;
		block_nr_[1] = 6;
		block_nr_[2] = 0;
	}

	for (int i=3;i<N_POSE_BLOCKS;i++)
	{
		do{block_nr_[i] = rand() % 7;}
		while(checkDoublettes(i));
	}

}

bool ExplorePoses::checkDoublettes(uint8_t idx)
{
	bool doublette = false;

	for (int ii=0;ii<idx;ii++)
	{
		if (block_nr_[idx]==block_nr_[ii])
			doublette = true;
	}
	return doublette;
}


void ExplorePoses::init_std_v1()
{
	explore_poses_std_v1_.clear();

	tmp_goal_.planning_frame     = LWR_TCP;
	tmp_goal_.planning_algorithm = STANDARD_IK_7DOF;
	tmp_goal_.inter_steps = 0;
	tmp_goal_.speed_percentage = fast_moving_speed;

	//! Joint 3 =  0
	tmp_goal_.goal_pose.position.x = 0.232;
	tmp_goal_.goal_pose.position.y = 0.232;
	tmp_goal_.goal_pose.position.z = 0.921;
	q_temp_.setRPY(3.14,-0.442,0.786);
	setOrientationAndInsert(EXPLORE_STD_1);

	tmp_goal_.goal_pose.position.x = 0.27;
	tmp_goal_.goal_pose.position.y = 0.27;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(3.14,-0.082,0.786);
	setOrientationAndInsert(EXPLORE_STD_1);

	tmp_goal_.goal_pose.position.x = 0;
	tmp_goal_.goal_pose.position.y = 0.381;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(3.14,-0.082,1.571);
	setOrientationAndInsert(EXPLORE_STD_1);


	tmp_goal_.goal_pose.position.x = -0.232;
	tmp_goal_.goal_pose.position.y = 0.232;
	tmp_goal_.goal_pose.position.z = 0.921;
	q_temp_.setRPY(3.14,-0.442,2.357);
	setOrientationAndInsert(EXPLORE_STD_1);


	tmp_goal_.goal_pose.position.x = -0.27;
	tmp_goal_.goal_pose.position.y = 0.27;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(3.14,-0.082,2.357);
	setOrientationAndInsert(EXPLORE_STD_1);


	tmp_goal_.goal_pose.position.x = -0.381;
	tmp_goal_.goal_pose.position.y = 0;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(3.14,-0.082,-3.14);
	setOrientationAndInsert(EXPLORE_STD_1);


	tmp_goal_.goal_pose.position.x = -0.381;
	tmp_goal_.goal_pose.position.y = 0;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(-2.7,-0.094,-3.085);
	setOrientationAndInsert(EXPLORE_STD_1);


	tmp_goal_.goal_pose.position.x = -0.27;
	tmp_goal_.goal_pose.position.y = -0.27;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(-3.14,-0.082,-2.357);
	setOrientationAndInsert(EXPLORE_STD_1);


	tmp_goal_.goal_pose.position.x = -0.232;
	tmp_goal_.goal_pose.position.y = -0.232;
	tmp_goal_.goal_pose.position.z = 0.921;
	q_temp_.setRPY(-3.14,-0.442,-2.357);
	setOrientationAndInsert(EXPLORE_STD_1);


	tmp_goal_.goal_pose.position.x = 0;
	tmp_goal_.goal_pose.position.y = -0.381;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(2.7,-0.094,-1.627);
	setOrientationAndInsert(EXPLORE_STD_1);


	tmp_goal_.goal_pose.position.x = 0;
	tmp_goal_.goal_pose.position.y = -0.381;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(3.14,-0.082,-1.571);
	setOrientationAndInsert(EXPLORE_STD_1);


	tmp_goal_.goal_pose.position.x = 0.232;
	tmp_goal_.goal_pose.position.y = -0.232;
	tmp_goal_.goal_pose.position.z = 0.921;
	q_temp_.setRPY(3.14,-0.442,-0.786);
	setOrientationAndInsert(EXPLORE_STD_1);


	tmp_goal_.goal_pose.position.x = 0.27;
	tmp_goal_.goal_pose.position.y = -0.27;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(3.14,-0.082,-0.786);
	setOrientationAndInsert(EXPLORE_STD_1);


	tmp_goal_.goal_pose.position.x = 0.381;
	tmp_goal_.goal_pose.position.y = 0;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(3.14,-0.082,0);
	setOrientationAndInsert(EXPLORE_STD_1);

}

void ExplorePoses::init_std_v2()
{
	explore_poses_std_v2_.clear();

	tmp_goal_.planning_frame     = LWR_TCP;
	tmp_goal_.planning_algorithm = STANDARD_IK_7DOF;
	tmp_goal_.inter_steps = 0;
	tmp_goal_.speed_percentage = fast_moving_speed;

	tmp_goal_.goal_config.q.resize(9);

	//COMPATIBLE WITH STANDARD IK!!!!

	tmp_goal_.goal_pose.position.x = -0.517;
	tmp_goal_.goal_pose.position.y = 0.155;
	tmp_goal_.goal_pose.position.z = 0.454;
	q_temp_.setRPY(2.142,0,0.472);
	tmp_goal_.goal_config.q[0] = 0;
	tmp_goal_.goal_config.q[1] = 0;
	tmp_goal_.goal_config.q[2] = -1.1;
	tmp_goal_.goal_config.q[3] = 1.2;
	tmp_goal_.goal_config.q[4] = -1.572;
	tmp_goal_.goal_config.q[5] = 1.572;
	tmp_goal_.goal_config.q[6] = -0.2;
	tmp_goal_.goal_config.q[7] = -1.572;
	tmp_goal_.goal_config.q[8] = 0;
	setOrientationAndInsert(EXPLORE_STD_2);

	tmp_goal_.goal_pose.position.x = -0.39;
	tmp_goal_.goal_pose.position.y = 0.373;
	tmp_goal_.goal_pose.position.z = 0.454;
	q_temp_.setRPY(2.142,0,0);
	tmp_goal_.goal_config.q[0] = 0;
	tmp_goal_.goal_config.q[1] = 0;
	tmp_goal_.goal_config.q[2] = -1.572;
	tmp_goal_.goal_config.q[3] = 1.2;
	tmp_goal_.goal_config.q[4] = -1.572;
	tmp_goal_.goal_config.q[5] = 1.572;
	tmp_goal_.goal_config.q[6] = -0.2;
	tmp_goal_.goal_config.q[7] = -1.572;
	tmp_goal_.goal_config.q[8] = 0;
	setOrientationAndInsert(EXPLORE_STD_2);

	tmp_goal_.goal_pose.position.x = -0.373;
	tmp_goal_.goal_pose.position.y = -0.39;
	tmp_goal_.goal_pose.position.z = 0.454;
	q_temp_.setRPY(2.142,0,1.573);
	tmp_goal_.goal_config.q[0] = 0;
	tmp_goal_.goal_config.q[1] = 0;
	tmp_goal_.goal_config.q[2] = 0;
	tmp_goal_.goal_config.q[3] = 1.2;
	tmp_goal_.goal_config.q[4] = -1.572;
	tmp_goal_.goal_config.q[5] = 1.572;
	tmp_goal_.goal_config.q[6] = -0.2;
	tmp_goal_.goal_config.q[7] = -1.572;
	tmp_goal_.goal_config.q[8] = 0;
	setOrientationAndInsert(EXPLORE_STD_2);

	tmp_goal_.goal_pose.position.x = 0.39;
	tmp_goal_.goal_pose.position.y = -0.372;
	tmp_goal_.goal_pose.position.z = 0.454;
	q_temp_.setRPY(2.142,0,-3.139);
	tmp_goal_.goal_config.q[0] = 0;
	tmp_goal_.goal_config.q[1] = 0;
	tmp_goal_.goal_config.q[2] = 1.572;
	tmp_goal_.goal_config.q[3] = 1.2;
	tmp_goal_.goal_config.q[4] = -1.572;
	tmp_goal_.goal_config.q[5] = 1.572;
	tmp_goal_.goal_config.q[6] = -0.2;
	tmp_goal_.goal_config.q[7] = -1.572;
	tmp_goal_.goal_config.q[8] = 0;
	setOrientationAndInsert(EXPLORE_STD_2);

	tmp_goal_.goal_pose.position.x = -0.064;
	tmp_goal_.goal_pose.position.y = -0.459;
	tmp_goal_.goal_pose.position.z = 0.549;
	q_temp_.setRPY(-3.073,-0.501,1.218);
	tmp_goal_.goal_config.q[0] = 0;
	tmp_goal_.goal_config.q[1] = 0;
	tmp_goal_.goal_config.q[2] = -1.223;
	tmp_goal_.goal_config.q[3] = -0.3;
	tmp_goal_.goal_config.q[4] = -0.593;
	tmp_goal_.goal_config.q[5] = 1.70;
	tmp_goal_.goal_config.q[6] = 0.14;
	tmp_goal_.goal_config.q[7] = -1.7;
	tmp_goal_.goal_config.q[8] = 0.05;
	setOrientationAndInsert(EXPLORE_STD_2);

	tmp_goal_.goal_pose.position.x = -0.274;
	tmp_goal_.goal_pose.position.y = -0.265;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(3.117,0.078,0.467);
	tmp_goal_.goal_config.q[0] = 0;
	tmp_goal_.goal_config.q[1] = 0;
	tmp_goal_.goal_config.q[2] = 0.594;
	tmp_goal_.goal_config.q[3] = 0;
	tmp_goal_.goal_config.q[4] = -2.967;
	tmp_goal_.goal_config.q[5] = 1.359;
	tmp_goal_.goal_config.q[6] = 0;
	tmp_goal_.goal_config.q[7] = -1.701;
	tmp_goal_.goal_config.q[8] = 0.301;
	setOrientationAndInsert(EXPLORE_STD_2);

	tmp_goal_.goal_pose.position.x = -0.376;
	tmp_goal_.goal_pose.position.y = -0.066;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(-2.885,0.17,-0.156);
	tmp_goal_.goal_config.q[0] = 0;
	tmp_goal_.goal_config.q[1] = 0;
	tmp_goal_.goal_config.q[2] = 0;
	tmp_goal_.goal_config.q[3] = 0;
	tmp_goal_.goal_config.q[4] = -2.96;
	tmp_goal_.goal_config.q[5] = 1.359;
	tmp_goal_.goal_config.q[6] = -0.3;
	tmp_goal_.goal_config.q[7] = -1.701;
	tmp_goal_.goal_config.q[8] = 0.301;
	setOrientationAndInsert(EXPLORE_STD_2);

	tmp_goal_.goal_pose.position.x = -0.383;
	tmp_goal_.goal_pose.position.y = -0.068;
	tmp_goal_.goal_pose.position.z = 0.738;
	q_temp_.setRPY(-2.555,0.132,-0.148);
	tmp_goal_.goal_config.q[0] = 0;
	tmp_goal_.goal_config.q[1] = 0;
	tmp_goal_.goal_config.q[2] = 0;
	tmp_goal_.goal_config.q[3] = 0;
	tmp_goal_.goal_config.q[4] = -2.96;
	tmp_goal_.goal_config.q[5] = 1.5;
	tmp_goal_.goal_config.q[6] = -0.6;
	tmp_goal_.goal_config.q[7] = -1.701;
	tmp_goal_.goal_config.q[8] = 0.301;
	setOrientationAndInsert(EXPLORE_STD_2);



	//IF DONE WITH MOVE_IT, NOT STANDARD IK!!!
	//
	//	tmp_goal_.goal_pose.position.x = -0.373;
	//	tmp_goal_.goal_pose.position.y = -0.39;
	//	tmp_goal_.goal_pose.position.z = 0.455;
	//	q_temp_.setRPY(2.142,0,1.573);
	//	tmp_goal_.goal_config.q[0] = 0;
	//	tmp_goal_.goal_config.q[1] = 0;
	//	tmp_goal_.goal_config.q[2] = 0;
	//	tmp_goal_.goal_config.q[3] = 1.2;
	//	tmp_goal_.goal_config.q[4] = -1.572;
	//	tmp_goal_.goal_config.q[5] = 1.572;
	//	tmp_goal_.goal_config.q[6] = -0.2;
	//	tmp_goal_.goal_config.q[7] = -1.572;
	//	tmp_goal_.goal_config.q[8] = 0;
	//	setOrientationAndInsert(EXPLORE_STD_2);
	//
	//	tmp_goal_.goal_pose.position.x = -0.39;
	//	tmp_goal_.goal_pose.position.y = 0.373;
	//	tmp_goal_.goal_pose.position.z = 0.455;
	//	q_temp_.setRPY(2.142,0,0);
	//	tmp_goal_.goal_config.q[0] = 0;
	//	tmp_goal_.goal_config.q[1] = 0;
	//	tmp_goal_.goal_config.q[2] = -1.572;
	//	tmp_goal_.goal_config.q[3] = 1.2;
	//	tmp_goal_.goal_config.q[4] = -1.572;
	//	tmp_goal_.goal_config.q[5] = 1.572;
	//	tmp_goal_.goal_config.q[6] = -0.2;
	//	tmp_goal_.goal_config.q[7] = -1.572;
	//	tmp_goal_.goal_config.q[8] = 0;
	//	setOrientationAndInsert(EXPLORE_STD_2);
	//
	//	tmp_goal_.goal_pose.position.x = -0.381;
	//	tmp_goal_.goal_pose.position.y = 0;
	//	tmp_goal_.goal_pose.position.z = 0.792;
	//	q_temp_.setRPY(-2.7,-0.094,-3.085);
	//	tmp_goal_.goal_config.q[0] = 0;
	//	tmp_goal_.goal_config.q[1] = 0;
	//	tmp_goal_.goal_config.q[2] = 0;
	//	tmp_goal_.goal_config.q[3] = 0;
	//	tmp_goal_.goal_config.q[4] = 0;
	//	tmp_goal_.goal_config.q[5] = -1.36;
	//	tmp_goal_.goal_config.q[6] = 0.45;
	//	tmp_goal_.goal_config.q[7] = 1.7;
	//	tmp_goal_.goal_config.q[8] = 0;
	//	setOrientationAndInsert(EXPLORE_STD_2);
	//
	//	tmp_goal_.goal_pose.position.x = -0.27;
	//	tmp_goal_.goal_pose.position.y = -0.27;
	//	tmp_goal_.goal_pose.position.z = 0.792;
	//	q_temp_.setRPY(-3.14,-0.082,-2.357);
	//	tmp_goal_.goal_config.q[0] = 0;
	//	tmp_goal_.goal_config.q[1] = 0;
	//	tmp_goal_.goal_config.q[2] = 0.193;
	//	tmp_goal_.goal_config.q[3] = 0;
	//	tmp_goal_.goal_config.q[4] = 0.593;
	//	tmp_goal_.goal_config.q[5] = -1.36;
	//	tmp_goal_.goal_config.q[6] = 0;
	//	tmp_goal_.goal_config.q[7] = 1.7;
	//	tmp_goal_.goal_config.q[8] = 0;
	//	setOrientationAndInsert(EXPLORE_STD_2);
	//
	//	tmp_goal_.goal_pose.position.x = 0;
	//	tmp_goal_.goal_pose.position.y = -0.381;
	//	tmp_goal_.goal_pose.position.z = 0.792;
	//	q_temp_.setRPY(2.7,-0.094,-1.627);
	//	tmp_goal_.goal_config.q[0] = 0;
	//	tmp_goal_.goal_config.q[1] = 0;
	//	tmp_goal_.goal_config.q[2] = 0.382;
	//	tmp_goal_.goal_config.q[3] = 0;
	//	tmp_goal_.goal_config.q[4] = 1.187;
	//	tmp_goal_.goal_config.q[5] = -1.36;
	//	tmp_goal_.goal_config.q[6] = -0.45;
	//	tmp_goal_.goal_config.q[7] = 1.7;
	//	tmp_goal_.goal_config.q[8] = 0;
	//	setOrientationAndInsert(EXPLORE_STD_2);
	//
	//	tmp_goal_.goal_pose.position.x = -0.390;
	//	tmp_goal_.goal_pose.position.y = -0.373;
	//	tmp_goal_.goal_pose.position.z = 0.455;
	//	q_temp_.setRPY(2.142,0,3.140);
	//	tmp_goal_.goal_config.q[0] = 0;
	//	tmp_goal_.goal_config.q[1] = 0;
	//	tmp_goal_.goal_config.q[2] = 1.572;
	//	tmp_goal_.goal_config.q[3] = 1.2;
	//	tmp_goal_.goal_config.q[4] = -1.572;
	//	tmp_goal_.goal_config.q[5] = -1.572;
	//	tmp_goal_.goal_config.q[6] = 0.2;
	//	tmp_goal_.goal_config.q[7] = 1.572;
	//	tmp_goal_.goal_config.q[8] = 0;
	//	setOrientationAndInsert(EXPLORE_STD_2);
	//
	//	tmp_goal_.goal_pose.position.x = -0.527;
	//	tmp_goal_.goal_pose.position.y = 0.112;
	//	tmp_goal_.goal_pose.position.z = 0.455;
	//	q_temp_.setRPY(2.142,0,2.168);
	//	tmp_goal_.goal_config.q[0] = 0;
	//	tmp_goal_.goal_config.q[1] = 0;
	//	tmp_goal_.goal_config.q[2] = 0.6;
	//	tmp_goal_.goal_config.q[3] = 1.2;
	//	tmp_goal_.goal_config.q[4] = -1.572;
	//	tmp_goal_.goal_config.q[5] = -1.572;
	//	tmp_goal_.goal_config.q[6] = 0.2;
	//	tmp_goal_.goal_config.q[7] = 1.572;
	//	tmp_goal_.goal_config.q[8] = 0;
	//	setOrientationAndInsert(EXPLORE_STD_2);

}


void ExplorePoses::init_snake()
{
	explore_poses_snake_.clear();

	explore_poses_snake_.resize(N_POSE_BLOCKS);

	tmp_goal_.planning_frame     = LWR_TCP;
	tmp_goal_.planning_algorithm = MOVE_IT_JT_9DOF;
	tmp_goal_.inter_steps = 0;
	tmp_goal_.speed_percentage = fast_moving_speed;

	tmp_goal_.goal_config.q.resize(9);

	// BLOCK 0
	// # 1
	//	tmp_goal_.goal_pose.position.x = 0.336+0.88;
	//	tmp_goal_.goal_pose.position.y = 0.39+0;
	//	tmp_goal_.goal_pose.position.z = 0.525;
	//	q_temp_.setRPY(3.14,0.86,-0.003);
	//	tmp_goal_.goal_config.q[0] = 0.88;
	//	tmp_goal_.goal_config.q[1] = 0.0;
	//	tmp_goal_.goal_config.q[2] = 0.0;
	//	tmp_goal_.goal_config.q[3] = -1.0;
	//	tmp_goal_.goal_config.q[4] = -1.572;
	//	tmp_goal_.goal_config.q[5] = -1.572;
	//	tmp_goal_.goal_config.q[6] = -0.14;
	//	tmp_goal_.goal_config.q[7] = 1.572;
	//	tmp_goal_.goal_config.q[8] = 1.572;
	//	setOrientationAndInsert(EXPLORE_SNAKE,0);
	// # 5
	tmp_goal_.goal_pose.position.x = -0.245+0.88;
	tmp_goal_.goal_pose.position.y = -0.37-0.88;
	tmp_goal_.goal_pose.position.z = 0.785;
	q_temp_.setRPY(-2.659,-0.82,1.759);
	tmp_goal_.goal_config.q[0] = 0.88;
	tmp_goal_.goal_config.q[1] = -0.88;
	tmp_goal_.goal_config.q[2] = -1.2;
	tmp_goal_.goal_config.q[3] = -0.5;
	tmp_goal_.goal_config.q[4] = -1.572;
	tmp_goal_.goal_config.q[5] = 1.2;
	tmp_goal_.goal_config.q[6] = -0.35;
	tmp_goal_.goal_config.q[7] = -1.572;
	tmp_goal_.goal_config.q[8] = -1.572;
	setOrientationAndInsert(EXPLORE_SNAKE,0);
	// # 4
	tmp_goal_.goal_pose.position.x = 0.336+0.88;
	tmp_goal_.goal_pose.position.y = -0.39-0.4;
	tmp_goal_.goal_pose.position.z = 0.526;
	q_temp_.setRPY(3.14,-0.87,3.141);
	tmp_goal_.goal_config.q[0] = 0.88;
	tmp_goal_.goal_config.q[1] = -0.4;
	tmp_goal_.goal_config.q[2] = 0.0;
	tmp_goal_.goal_config.q[3] = -1.0;
	tmp_goal_.goal_config.q[4] = -1.572;
	tmp_goal_.goal_config.q[5] = 1.572;
	tmp_goal_.goal_config.q[6] = 0.13;
	tmp_goal_.goal_config.q[7] = -1.572;
	tmp_goal_.goal_config.q[8] = -1.572;
	setOrientationAndInsert(EXPLORE_SNAKE,0);
	// # 3
	tmp_goal_.goal_pose.position.x = 0.336+0.88;
	tmp_goal_.goal_pose.position.y = -0.39+0;
	tmp_goal_.goal_pose.position.z = 0.526;
	q_temp_.setRPY(3.14,-0.87,3.141);
	tmp_goal_.goal_config.q[0] = 0.88;
	tmp_goal_.goal_config.q[1] = 0.0;
	tmp_goal_.goal_config.q[2] = 0.0;
	tmp_goal_.goal_config.q[3] = -1.0;
	tmp_goal_.goal_config.q[4] = -1.572;
	tmp_goal_.goal_config.q[5] = 1.572;
	tmp_goal_.goal_config.q[6] = 0.13;
	tmp_goal_.goal_config.q[7] = -1.572;
	tmp_goal_.goal_config.q[8] = -1.572;
	setOrientationAndInsert(EXPLORE_SNAKE,0);
	// # 2
	tmp_goal_.goal_pose.position.x = 0.336+0.88;
	tmp_goal_.goal_pose.position.y = -0.39+0.4;
	tmp_goal_.goal_pose.position.z = 0.526;
	q_temp_.setRPY(3.14,-0.87,3.141);
	tmp_goal_.goal_config.q[0] =  0.88;
	tmp_goal_.goal_config.q[1] =  0.4;
	tmp_goal_.goal_config.q[2] =  0.0;
	tmp_goal_.goal_config.q[3] =  -1.0;
	tmp_goal_.goal_config.q[4] =  -1.572;
	tmp_goal_.goal_config.q[5] =  1.572;
	tmp_goal_.goal_config.q[6] =  0.13;
	tmp_goal_.goal_config.q[7] =  -1.572;
	tmp_goal_.goal_config.q[8] =  -1.572;
	setOrientationAndInsert(EXPLORE_SNAKE,0);

	// BLOCK 1
	// # 1
	//	tmp_goal_.goal_pose.position.x = 0.39+0;
	//	tmp_goal_.goal_pose.position.y = 0.336+0.88;
	//	tmp_goal_.goal_pose.position.z = 0.526;
	//	q_temp_.setRPY(3.14,-0.87,-1.57);
	//	tmp_goal_.goal_config.q[0] = 0.0;
	//	tmp_goal_.goal_config.q[1] = 0.88;
	//	tmp_goal_.goal_config.q[2] = 1.572;
	//	tmp_goal_.goal_config.q[3] = -1.0;
	//	tmp_goal_.goal_config.q[4] = -1.572;
	//	tmp_goal_.goal_config.q[5] = 1.572;
	//	tmp_goal_.goal_config.q[6] = 0.13;
	//	tmp_goal_.goal_config.q[7] = -1.572;
	//	tmp_goal_.goal_config.q[8] = -1.572;
	//	setOrientationAndInsert(EXPLORE_SNAKE,1);
	// # 5
	tmp_goal_.goal_pose.position.x = -0.374-0.88;
	tmp_goal_.goal_pose.position.y = -0.245+0.88;
	tmp_goal_.goal_pose.position.z = 0.785;
	q_temp_.setRPY(-2.66,0.82,2.952);
	tmp_goal_.goal_config.q[0] = -0.88;
	tmp_goal_.goal_config.q[1] = 0.88;
	tmp_goal_.goal_config.q[2] = 2.772;
	tmp_goal_.goal_config.q[3] = -0.5;
	tmp_goal_.goal_config.q[4] = -1.572;
	tmp_goal_.goal_config.q[5] = -1.2;
	tmp_goal_.goal_config.q[6] = 0.35;
	tmp_goal_.goal_config.q[7] = 1.572;
	tmp_goal_.goal_config.q[8] = 1.572;
	setOrientationAndInsert(EXPLORE_SNAKE,1);
	// # 4
	tmp_goal_.goal_pose.position.x = -0.39-0.4;
	tmp_goal_.goal_pose.position.y = 0.336+0.88;
	tmp_goal_.goal_pose.position.z = 0.525;
	q_temp_.setRPY(3.14,0.86,1.57);
	tmp_goal_.goal_config.q[0] = -0.4;
	tmp_goal_.goal_config.q[1] = 0.88;
	tmp_goal_.goal_config.q[2] = 1.572;
	tmp_goal_.goal_config.q[3] = -1.0;
	tmp_goal_.goal_config.q[4] = -1.572;
	tmp_goal_.goal_config.q[5] = -1.572;
	tmp_goal_.goal_config.q[6] = -0.14;
	tmp_goal_.goal_config.q[7] = 1.572;
	tmp_goal_.goal_config.q[8] = 1.572;
	setOrientationAndInsert(EXPLORE_SNAKE,1);
	// # 3
	tmp_goal_.goal_pose.position.x = -0.39+0;
	tmp_goal_.goal_pose.position.y = 0.336+0.88;
	tmp_goal_.goal_pose.position.z = 0.525;
	q_temp_.setRPY(3.14,0.86,1.57);
	tmp_goal_.goal_config.q[0] = 0.0;
	tmp_goal_.goal_config.q[1] = 0.88;
	tmp_goal_.goal_config.q[2] = 1.572;
	tmp_goal_.goal_config.q[3] = -1.0;
	tmp_goal_.goal_config.q[4] = -1.572;
	tmp_goal_.goal_config.q[5] = -1.572;
	tmp_goal_.goal_config.q[6] = -0.14;
	tmp_goal_.goal_config.q[7] = 1.572;
	tmp_goal_.goal_config.q[8] = 1.572;
	setOrientationAndInsert(EXPLORE_SNAKE,1);
	// # 2
	tmp_goal_.goal_pose.position.x = -0.39+0.4;
	tmp_goal_.goal_pose.position.y = 0.336+0.88;
	tmp_goal_.goal_pose.position.z = 0.525;
	q_temp_.setRPY(3.14,0.86,1.57);
	tmp_goal_.goal_config.q[0] = 0.4;
	tmp_goal_.goal_config.q[1] = 0.88;
	tmp_goal_.goal_config.q[2] = 1.572;
	tmp_goal_.goal_config.q[3] = -1.0;
	tmp_goal_.goal_config.q[4] = -1.572;
	tmp_goal_.goal_config.q[5] = -1.572;
	tmp_goal_.goal_config.q[6] = -0.14;
	tmp_goal_.goal_config.q[7] = 1.572;
	tmp_goal_.goal_config.q[8] = 1.572;
	setOrientationAndInsert(EXPLORE_SNAKE,1);
	/////////////////////////////////////////////////////////////////////////
	// NEW POSES

	// BLOCK 2
	// # 1
	tmp_goal_.goal_pose.position.x = -0.381;
	tmp_goal_.goal_pose.position.y = 0;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(-2.7,-0.094,-3.085);
	tmp_goal_.goal_config.q[0] = 0;
	tmp_goal_.goal_config.q[1] = 0;
	tmp_goal_.goal_config.q[2] = 0;
	tmp_goal_.goal_config.q[3] = 0;
	tmp_goal_.goal_config.q[4] = 0;
	tmp_goal_.goal_config.q[5] = -1.36;
	tmp_goal_.goal_config.q[6] = 0.45;
	tmp_goal_.goal_config.q[7] = 1.7;
	tmp_goal_.goal_config.q[8] = 0;
	setOrientationAndInsert(EXPLORE_SNAKE,2);
	// # 2
	tmp_goal_.goal_pose.position.x = -0.27;
	tmp_goal_.goal_pose.position.y = -0.27;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(-3.14,-0.082,-2.357);
	tmp_goal_.goal_config.q[0] = 0;
	tmp_goal_.goal_config.q[1] = 0;
	tmp_goal_.goal_config.q[2] = 0.193;
	tmp_goal_.goal_config.q[3] = 0;
	tmp_goal_.goal_config.q[4] = 0.593;
	tmp_goal_.goal_config.q[5] = -1.36;
	tmp_goal_.goal_config.q[6] = 0;
	tmp_goal_.goal_config.q[7] = 1.7;
	tmp_goal_.goal_config.q[8] = 0;
	setOrientationAndInsert(EXPLORE_SNAKE,2);
	// # 3
	tmp_goal_.goal_pose.position.x = -0.232;
	tmp_goal_.goal_pose.position.y = -0.232;
	tmp_goal_.goal_pose.position.z = 0.921;
	q_temp_.setRPY(-3.14,-0.442,-2.357);
	tmp_goal_.goal_config.q[0] = 0;
	tmp_goal_.goal_config.q[1] = 0;
	tmp_goal_.goal_config.q[2] = 0.193;
	tmp_goal_.goal_config.q[3] = 0;
	tmp_goal_.goal_config.q[4] = 0.593;
	tmp_goal_.goal_config.q[5] = -1;
	tmp_goal_.goal_config.q[6] = 0;
	tmp_goal_.goal_config.q[7] = 1.7;
	tmp_goal_.goal_config.q[8] = 0;
	setOrientationAndInsert(EXPLORE_SNAKE,2);
	// # 4
	tmp_goal_.goal_pose.position.x = 0;
	tmp_goal_.goal_pose.position.y = -0.381;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(2.7,-0.094,-1.627);
	tmp_goal_.goal_config.q[0] = 0;
	tmp_goal_.goal_config.q[1] = 0;
	tmp_goal_.goal_config.q[2] = 0.382;
	tmp_goal_.goal_config.q[3] = 0;
	tmp_goal_.goal_config.q[4] = 1.187;
	tmp_goal_.goal_config.q[5] = -1.36;
	tmp_goal_.goal_config.q[6] = -0.45;
	tmp_goal_.goal_config.q[7] = 1.7;
	tmp_goal_.goal_config.q[8] = 0;
	setOrientationAndInsert(EXPLORE_SNAKE,2);

	// BLOCK 3
	// # 1
	tmp_goal_.goal_pose.position.x = 0.3-0.143;
	tmp_goal_.goal_pose.position.y = -0.3-0.354;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(2.45,-0.11,-2.037);
	tmp_goal_.goal_config.q[0] = 0.3;
	tmp_goal_.goal_config.q[1] = 0.3;
	tmp_goal_.goal_config.q[2] = 0;
	tmp_goal_.goal_config.q[3] = 0;
	tmp_goal_.goal_config.q[4] = 1.187;
	tmp_goal_.goal_config.q[5] = -1.36;
	tmp_goal_.goal_config.q[6] = -0.7;
	tmp_goal_.goal_config.q[7] = 1.7;
	tmp_goal_.goal_config.q[8] = 0;
	setOrientationAndInsert(EXPLORE_SNAKE,3);
	// # 2
	tmp_goal_.goal_pose.position.x = -0.3-0.354;
	tmp_goal_.goal_pose.position.y = 0.3-0.143;
	tmp_goal_.goal_pose.position.z = 0.792;
	q_temp_.setRPY(-2.45,-0.11,-2.671);
	tmp_goal_.goal_config.q[0] = -0.3;
	tmp_goal_.goal_config.q[1] = 0.3;
	tmp_goal_.goal_config.q[2] = 0;
	tmp_goal_.goal_config.q[3] = 0;
	tmp_goal_.goal_config.q[4] = 0.385;
	tmp_goal_.goal_config.q[5] = -1.36;
	tmp_goal_.goal_config.q[6] = 0.7;
	tmp_goal_.goal_config.q[7] = 1.7;
	tmp_goal_.goal_config.q[8] = 0;
	setOrientationAndInsert(EXPLORE_SNAKE,3);

	// BLOCK 4
	// # 1
	tmp_goal_.goal_pose.position.x = -0.5-0.172;
	tmp_goal_.goal_pose.position.y = 0.2-0.533;
	tmp_goal_.goal_pose.position.z = 0.563;
	q_temp_.setRPY(-2.224,0.334,-1.647);
	tmp_goal_.goal_config.q[0] = -0.5;
	tmp_goal_.goal_config.q[1] = 0.2;
	tmp_goal_.goal_config.q[2] = 1;
	tmp_goal_.goal_config.q[3] = 0.5;
	tmp_goal_.goal_config.q[4] = 0.385;
	tmp_goal_.goal_config.q[5] = -1.36;
	tmp_goal_.goal_config.q[6] = 0.7;
	tmp_goal_.goal_config.q[7] = 1.7;
	tmp_goal_.goal_config.q[8] = 0;
	setOrientationAndInsert(EXPLORE_SNAKE,4);
	// # 2
	tmp_goal_.goal_pose.position.x = -0.5-0.079;
	tmp_goal_.goal_pose.position.y = 0.2-0.506;
	tmp_goal_.goal_pose.position.z = 0.65;
	q_temp_.setRPY(-2.293,0.115,-0.986);
	tmp_goal_.goal_config.q[0] = -0.5;
	tmp_goal_.goal_config.q[1] = 0.2;
	tmp_goal_.goal_config.q[2] = 1;
	tmp_goal_.goal_config.q[3] = 0.5;
	tmp_goal_.goal_config.q[4] = 1.1;
	tmp_goal_.goal_config.q[5] = -1.36;
	tmp_goal_.goal_config.q[6] = 0.4;
	tmp_goal_.goal_config.q[7] = 1.7;
	tmp_goal_.goal_config.q[8] = 0;
	setOrientationAndInsert(EXPLORE_SNAKE,4);

	// BLOCK 5
	// # 1
	tmp_goal_.goal_pose.position.x = 0.2-0.531;
	tmp_goal_.goal_pose.position.y = -0.5-0.176;
	tmp_goal_.goal_pose.position.z = 0.563;
	q_temp_.setRPY(2.224,0.334,-3.059);
	tmp_goal_.goal_config.q[0] = 0.2;
	tmp_goal_.goal_config.q[1] = -0.5;
	tmp_goal_.goal_config.q[2] = 0.578;
	tmp_goal_.goal_config.q[3] = 0.5;
	tmp_goal_.goal_config.q[4] = -0.385;
	tmp_goal_.goal_config.q[5] = -1.36;
	tmp_goal_.goal_config.q[6] = -0.7;
	tmp_goal_.goal_config.q[7] = 1.7;
	tmp_goal_.goal_config.q[8] = 0;
	setOrientationAndInsert(EXPLORE_SNAKE,5);
	// # 2
	tmp_goal_.goal_pose.position.x = 0.2-0.506;
	tmp_goal_.goal_pose.position.y = -0.5-0.791;
	tmp_goal_.goal_pose.position.z = 0.65;
	q_temp_.setRPY(2.293,0.115,2.557);
	tmp_goal_.goal_config.q[0] = 0.2;
	tmp_goal_.goal_config.q[1] = -0.5;
	tmp_goal_.goal_config.q[2] = 0.571;
	tmp_goal_.goal_config.q[3] = 0.5;
	tmp_goal_.goal_config.q[4] = -1.1;
	tmp_goal_.goal_config.q[5] = -1.36;
	tmp_goal_.goal_config.q[6] = -0.4;
	tmp_goal_.goal_config.q[7] = 1.7;
	tmp_goal_.goal_config.q[8] = 0;
	setOrientationAndInsert(EXPLORE_SNAKE,5);


	// BLOCK 6
	// # 1
	tmp_goal_.goal_pose.position.x = -0.88+0.353;
	tmp_goal_.goal_pose.position.y = -0.88-0.48;
	tmp_goal_.goal_pose.position.z = 0.593;
	q_temp_.setRPY(-2.056,0,-0.868);
	tmp_goal_.goal_config.q[0] = -0.88;
	tmp_goal_.goal_config.q[1] = -0.88;
	tmp_goal_.goal_config.q[2] = 0.7;
	tmp_goal_.goal_config.q[3] = 0.786;
	tmp_goal_.goal_config.q[4] = 1.572;
	tmp_goal_.goal_config.q[5] = -1.572;
	tmp_goal_.goal_config.q[6] = 0.3;
	tmp_goal_.goal_config.q[7] = 1.572;
	tmp_goal_.goal_config.q[8] = 0;
	setOrientationAndInsert(EXPLORE_SNAKE,6);
	// # 2
	tmp_goal_.goal_pose.position.x = -0.88-0.5;
	tmp_goal_.goal_pose.position.y = -0.88+0.081;
	tmp_goal_.goal_pose.position.z = 0.639;
	q_temp_.setRPY(-2.241,0.484,-1.118);
	tmp_goal_.goal_config.q[0] = -0.88;
	tmp_goal_.goal_config.q[1] = -0.88;
	tmp_goal_.goal_config.q[2] = 0.7;
	tmp_goal_.goal_config.q[3] = 0.786;
	tmp_goal_.goal_config.q[4] = 1.572;
	tmp_goal_.goal_config.q[5] = 1.4;
	tmp_goal_.goal_config.q[6] = -0.2;
	tmp_goal_.goal_config.q[7] = -1.65;
	tmp_goal_.goal_config.q[8] = 0.5;
	setOrientationAndInsert(EXPLORE_SNAKE,6);
	// # 3
	tmp_goal_.goal_pose.position.x = -0.88-0.39;
	tmp_goal_.goal_pose.position.y = -0.35-0.39;
	tmp_goal_.goal_pose.position.z = 0.398;
	q_temp_.setRPY(3.14,0.95,-3.14);
	tmp_goal_.goal_config.q[0] = -0.88;
	tmp_goal_.goal_config.q[1] = -0.35;
	tmp_goal_.goal_config.q[2] = 0;
	tmp_goal_.goal_config.q[3] = 1.35;
	tmp_goal_.goal_config.q[4] = 1.572;
	tmp_goal_.goal_config.q[5] = -1.572;
	tmp_goal_.goal_config.q[6] = -0.4;
	tmp_goal_.goal_config.q[7] = 1.572;
	tmp_goal_.goal_config.q[8] = 1.572;
	setOrientationAndInsert(EXPLORE_SNAKE,6);
	// # 4
	tmp_goal_.goal_pose.position.x = -0.35-0.39;
	tmp_goal_.goal_pose.position.y = -0.88-0.39;
	tmp_goal_.goal_pose.position.z = 0.397;
	q_temp_.setRPY(3.136,-0.95,1.575);
	tmp_goal_.goal_config.q[0] = -0.35;
	tmp_goal_.goal_config.q[1] = -0.88;
	tmp_goal_.goal_config.q[2] = 1.572;
	tmp_goal_.goal_config.q[3] = 1.35;
	tmp_goal_.goal_config.q[4] = 1.572;
	tmp_goal_.goal_config.q[5] = 1.572;
	tmp_goal_.goal_config.q[6] = 0.4;
	tmp_goal_.goal_config.q[7] = -1.572;
	tmp_goal_.goal_config.q[8] = -1.572;
	setOrientationAndInsert(EXPLORE_SNAKE,6);

}


void ExplorePoses::init_outer_snake()
{

	// BLOCK 7
	// # 1
	tmp_goal_.goal_pose.position.x = 0.88-0.081;
	tmp_goal_.goal_pose.position.y = -0.88-0.5;
	tmp_goal_.goal_pose.position.z = 0.639;
	q_temp_.setRPY(-2.241,0.484,0.454);
	tmp_goal_.goal_config.q[0] = 0.88;
	tmp_goal_.goal_config.q[1] = -0.88;
	tmp_goal_.goal_config.q[2] = 2.272;
	tmp_goal_.goal_config.q[3] = 0.786;
	tmp_goal_.goal_config.q[4] = 1.572;
	tmp_goal_.goal_config.q[5] = 1.4;
	tmp_goal_.goal_config.q[6] = -0.2;
	tmp_goal_.goal_config.q[7] = -1.65;
	tmp_goal_.goal_config.q[8] = 0.5;
	setOrientationAndInsert(EXPLORE_OUTER_SNAKE);
	// # 2
	tmp_goal_.goal_pose.position.x = 0.88+0.48;
	tmp_goal_.goal_pose.position.y = -0.88+0.359;
	tmp_goal_.goal_pose.position.z = 0.593;
	q_temp_.setRPY(3.033,0.932,-1.001);
	tmp_goal_.goal_config.q[0] = 0.88;
	tmp_goal_.goal_config.q[1] = -0.88;
	tmp_goal_.goal_config.q[2] = 2.272;
	tmp_goal_.goal_config.q[3] = 0.786;
	tmp_goal_.goal_config.q[4] = 1.572;
	tmp_goal_.goal_config.q[5] = -1.572;
	tmp_goal_.goal_config.q[6] = 0.15;
	tmp_goal_.goal_config.q[7] = 1.572;
	tmp_goal_.goal_config.q[8] = 1.65;
	setOrientationAndInsert(EXPLORE_OUTER_SNAKE);
	// # 3
	tmp_goal_.goal_pose.position.x = -0.88+0.353;
	tmp_goal_.goal_pose.position.y = -0.88-0.48;
	tmp_goal_.goal_pose.position.z = 0.593;
	q_temp_.setRPY(3.033,0.932,-2.573);
	tmp_goal_.goal_config.q[0] = -0.88;
	tmp_goal_.goal_config.q[1] = -0.88;
	tmp_goal_.goal_config.q[2] = 0.7;
	tmp_goal_.goal_config.q[3] = 0.786;
	tmp_goal_.goal_config.q[4] = 1.572;
	tmp_goal_.goal_config.q[5] = -1.572;
	tmp_goal_.goal_config.q[6] = 0.15;
	tmp_goal_.goal_config.q[7] = 1.572;
	tmp_goal_.goal_config.q[8] = 1.65;
	setOrientationAndInsert(EXPLORE_OUTER_SNAKE);
	// # 4
	tmp_goal_.goal_pose.position.x = -0.88-0.5;
	tmp_goal_.goal_pose.position.y = -0.88+0.081;
	tmp_goal_.goal_pose.position.z = 0.639;
	q_temp_.setRPY(-2.241,0.484,-1.118);
	tmp_goal_.goal_config.q[0] = -0.88;
	tmp_goal_.goal_config.q[1] = -0.88;
	tmp_goal_.goal_config.q[2] = 0.7;
	tmp_goal_.goal_config.q[3] = 0.786;
	tmp_goal_.goal_config.q[4] = 1.572;
	tmp_goal_.goal_config.q[5] = 1.4;
	tmp_goal_.goal_config.q[6] = -0.2;
	tmp_goal_.goal_config.q[7] = -1.65;
	tmp_goal_.goal_config.q[8] = 0.5;
	setOrientationAndInsert(EXPLORE_OUTER_SNAKE);
	// # 5
	tmp_goal_.goal_pose.position.x = -0.88+0.082;
	tmp_goal_.goal_pose.position.y = 0.88+0.5;
	tmp_goal_.goal_pose.position.z = 0.639;
	q_temp_.setRPY(-2.241,0.484,-2.69);
	tmp_goal_.goal_config.q[0] = -0.88;
	tmp_goal_.goal_config.q[1] = 0.88;
	tmp_goal_.goal_config.q[2] = -0.872;
	tmp_goal_.goal_config.q[3] = 0.786;
	tmp_goal_.goal_config.q[4] = 1.572;
	tmp_goal_.goal_config.q[5] = 1.4;
	tmp_goal_.goal_config.q[6] = -0.2;
	tmp_goal_.goal_config.q[7] = -1.65;
	tmp_goal_.goal_config.q[8] = 0.5;
	setOrientationAndInsert(EXPLORE_OUTER_SNAKE);
	// # 6
	tmp_goal_.goal_pose.position.x = -0.88-0.48;
	tmp_goal_.goal_pose.position.y = 0.88-0.035;
	tmp_goal_.goal_pose.position.z = 0.593;
	q_temp_.setRPY(3.033,0.932,2.138);
	tmp_goal_.goal_config.q[0] = -0.88;
	tmp_goal_.goal_config.q[1] = 0.88;
	tmp_goal_.goal_config.q[2] = -0.872;
	tmp_goal_.goal_config.q[3] = 0.786;
	tmp_goal_.goal_config.q[4] = 1.572;
	tmp_goal_.goal_config.q[5] = -1.572;
	tmp_goal_.goal_config.q[6] = 0.15;
	tmp_goal_.goal_config.q[7] = 1.572;
	tmp_goal_.goal_config.q[8] = 1.65;
	setOrientationAndInsert(EXPLORE_OUTER_SNAKE);
}

void ExplorePoses::setOrientationAndInsert(uint8_t pose_type)
{
	tmp_goal_.goal_pose.orientation.x = q_temp_.x();
	tmp_goal_.goal_pose.orientation.y = q_temp_.y();
	tmp_goal_.goal_pose.orientation.z = q_temp_.z();
	tmp_goal_.goal_pose.orientation.w = q_temp_.w();

	switch (pose_type)
	{
	case EXPLORE_STD_1:
		explore_poses_std_v1_.push_back(tmp_goal_);
		break;
	case EXPLORE_STD_2:
		explore_poses_std_v2_.push_back(tmp_goal_);
		break;
	case EXPLORE_OUTER_SNAKE:
		explore_poses_outer_snake_.push_back(tmp_goal_);
		break;
	case EXPLORE_SNAKE:
		msg_error("problem detected... EXPLORE SNAKE has to call other function.");
		break;
	}

}

void ExplorePoses::setOrientationAndInsert(uint8_t pose_type, uint8_t block_idx)
{
	tmp_goal_.goal_pose.orientation.x = q_temp_.x();
	tmp_goal_.goal_pose.orientation.y = q_temp_.y();
	tmp_goal_.goal_pose.orientation.z = q_temp_.z();
	tmp_goal_.goal_pose.orientation.w = q_temp_.w();

	switch (pose_type)
	{
	case EXPLORE_STD_1:
		explore_poses_std_v1_.push_back(tmp_goal_);
		break;
	case EXPLORE_STD_2:
		explore_poses_std_v2_.push_back(tmp_goal_);
		break;
	case EXPLORE_SNAKE:
		if (block_idx<N_POSE_BLOCKS)
			explore_poses_snake_[block_idx].push_back(tmp_goal_);
		else
			msg_error("Block idx > N_POSE_BLOCKS");
		break;
	}

}
