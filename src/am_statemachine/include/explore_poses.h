/*
 * explore_poses.h
 *
 *  Created on: Oct 28, 2014
 *      Author: euroc_admin
 */

#ifndef EXPLORE_POSES_H_
#define EXPLORE_POSES_H_

#define N_POSE_BLOCKS 7

#include <stdlib.h>
#include <time.h>
#include <vector>

#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>

#include <am_msgs/goalPoseGoal.h>
#include <config.hpp>

class ExplorePoses {
public:
	ExplorePoses();
	virtual ~ExplorePoses();

	int size(uint8_t pose_type);

	am_msgs::goalPoseGoal getExploreGoalPose(uint8_t pose_nr,uint8_t pose_type, uint8_t success_count);

private:

	std::vector<std::vector<am_msgs::goalPoseGoal> > explore_poses_snake_;
	std::vector<am_msgs::goalPoseGoal> 				explore_poses_std_v1_;
	std::vector<am_msgs::goalPoseGoal> 				explore_poses_std_v2_;

	am_msgs::goalPoseGoal tmp_goal_;

	void init_std_v1();
	void init_std_v2();
	void init_snake();

	void setOrientationAndInsert(uint8_t);
	void setOrientationAndInsert(uint8_t,uint8_t);
	void randomSort();
	bool checkDoublettes(uint8_t idx);

	uint8_t block_nr_[N_POSE_BLOCKS];
	std::vector<std::vector<uint8_t> > pose_blocks_;
	std::vector<uint8_t> rand_pose_block_vec_;


	tf::Quaternion q_temp_;

	uint8_t block_counter_;
	uint8_t insideblock_success_counter_;
	uint8_t insideblock_counter_;
	uint8_t success_counter_;

	bool first_block_failed_;
};

#endif /* EXPLORE_POSES_H_ */
