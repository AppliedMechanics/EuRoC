std::vector<tf::Quaternion> q_temp;
active_goal_=0;
nr_goals_=14;

if (nr_goals_!=nr_exp_poses_)
	msg_error("Nr of explore poses different from poses defined in explore_standard.hpp");

explore_queue_.resize(nr_goals_);
q_temp.resize(nr_goals_);

//! Joint 3 =  0
explore_queue_[0].goal_pose.position.x = 0.232;
explore_queue_[0].goal_pose.position.y = 0.232;
explore_queue_[0].goal_pose.position.z = 0.921;
q_temp[0].setRPY(3.14,-0.442,0.786);

explore_queue_[1].goal_pose.position.x = 0.27;
explore_queue_[1].goal_pose.position.y = 0.27;
explore_queue_[1].goal_pose.position.z = 0.792;
q_temp[1].setRPY(3.14,-0.082,0.786);

explore_queue_[2].goal_pose.position.x = 0;
explore_queue_[2].goal_pose.position.y = 0.381;
explore_queue_[2].goal_pose.position.z = 0.792;
q_temp[2].setRPY(3.14,-0.082,1.571);

explore_queue_[3].goal_pose.position.x = -0.232;
explore_queue_[3].goal_pose.position.y = 0.232;
explore_queue_[3].goal_pose.position.z = 0.921;
q_temp[3].setRPY(3.14,-0.442,2.357);

explore_queue_[4].goal_pose.position.x = -0.27;
explore_queue_[4].goal_pose.position.y = 0.27;
explore_queue_[4].goal_pose.position.z = 0.792;
q_temp[4].setRPY(3.14,-0.082,2.357);

explore_queue_[5].goal_pose.position.x = -0.381;
explore_queue_[5].goal_pose.position.y = 0;
explore_queue_[5].goal_pose.position.z = 0.792;
q_temp[5].setRPY(3.14,-0.082,-3.14);

explore_queue_[6].goal_pose.position.x = -0.381;
explore_queue_[6].goal_pose.position.y = 0;
explore_queue_[6].goal_pose.position.z = 0.792;
q_temp[6].setRPY(-2.7,-0.094,-3.085);

explore_queue_[7].goal_pose.position.x = -0.27;
explore_queue_[7].goal_pose.position.y = -0.27;
explore_queue_[7].goal_pose.position.z = 0.792;
q_temp[7].setRPY(-3.14,-0.082,-2.357);

explore_queue_[8].goal_pose.position.x = -0.232;
explore_queue_[8].goal_pose.position.y = -0.232;
explore_queue_[8].goal_pose.position.z = 0.921;
q_temp[8].setRPY(-3.14,-0.442,-2.357);

explore_queue_[9].goal_pose.position.x = 0;
explore_queue_[9].goal_pose.position.y = -0.381;
explore_queue_[9].goal_pose.position.z = 0.792;
q_temp[9].setRPY(2.7,-0.094,-1.627);

explore_queue_[10].goal_pose.position.x = 0;
explore_queue_[10].goal_pose.position.y = -0.381;
explore_queue_[10].goal_pose.position.z = 0.792;
q_temp[10].setRPY(3.14,-0.082,-1.571);

explore_queue_[11].goal_pose.position.x = 0.232;
explore_queue_[11].goal_pose.position.y = -0.232;
explore_queue_[11].goal_pose.position.z = 0.921;
q_temp[11].setRPY(3.14,-0.442,-0.786);

explore_queue_[12].goal_pose.position.x = 0.27;
explore_queue_[12].goal_pose.position.y = -0.27;
explore_queue_[12].goal_pose.position.z = 0.792;
q_temp[12].setRPY(3.14,-0.082,-0.786);

explore_queue_[13].goal_pose.position.x = 0.381;
explore_queue_[13].goal_pose.position.y = 0;
explore_queue_[13].goal_pose.position.z = 0.792;
q_temp[13].setRPY(3.14,-0.082,0);


for (int i=0;i<nr_goals_;i++)
{
	explore_queue_[i].planning_algorithm = planning_mode_.explore;
	explore_queue_[i].planning_frame = LWR_TCP;
	explore_queue_[i].inter_steps = 0;
	explore_queue_[i].speed_percentage = fast_moving_speed;
	explore_queue_[i].goal_pose.orientation.x = q_temp[i].getX();
	explore_queue_[i].goal_pose.orientation.y = q_temp[i].getY();
	explore_queue_[i].goal_pose.orientation.z = q_temp[i].getZ();
	explore_queue_[i].goal_pose.orientation.w = q_temp[i].getW();
}


// NEW EXPLORE ENVIRONMENT
//
//
//explore_queue_[order_poses[14]].goal_pose.position.x = -0.381;
//explore_queue_[order_poses[14]].goal_pose.position.y = 0;
//explore_queue_[order_poses[14]].goal_pose.position.z = 0.792;
//q_temp[order_poses[14]].setRPY(2.7,-0.094,-3.085);
//explore_queue_[order_poses[14]].goal_config.q[0] = 0;
//explore_queue_[order_poses[14]].goal_config.q[1] = 0;
//explore_queue_[order_poses[14]].goal_config.q[2] = 0;
//explore_queue_[order_poses[14]].goal_config.q[3] = 0;
//explore_queue_[order_poses[14]].goal_config.q[4] = 0;
//explore_queue_[order_poses[14]].goal_config.q[5] = -1.36;
//explore_queue_[order_poses[14]].goal_config.q[6] = 0.45;
//explore_queue_[order_poses[14]].goal_config.q[7] = 1.7;
//explore_queue_[order_poses[14]].goal_config.q[8] = 0;
//
//explore_queue_[order_poses[15]].goal_pose.position.x = -0.27;
//explore_queue_[order_poses[15]].goal_pose.position.y = -0.27;
//explore_queue_[order_poses[15]].goal_pose.position.z = 0.792;
//q_temp[order_poses[15]].setRPY(-3.14,-0.082,-2.357);
//explore_queue_[order_poses[15]].goal_config.q[0] = 0;
//explore_queue_[order_poses[15]].goal_config.q[1] = 0;
//explore_queue_[order_poses[15]].goal_config.q[2] = 0.193;
//explore_queue_[order_poses[15]].goal_config.q[3] = 0;
//explore_queue_[order_poses[15]].goal_config.q[4] = 0.593;
//explore_queue_[order_poses[15]].goal_config.q[5] = -1.36;
//explore_queue_[order_poses[15]].goal_config.q[6] = 0;
//explore_queue_[order_poses[15]].goal_config.q[7] = 1.7;
//explore_queue_[order_poses[15]].goal_config.q[8] = 0;
//
//explore_queue_[order_poses[16]].goal_pose.position.x = 0;
//explore_queue_[order_poses[16]].goal_pose.position.y = -0.381;
//explore_queue_[order_poses[16]].goal_pose.position.z = 0.792;
//q_temp[order_poses[16]].setRPY(2.7,-0.094,-1.627);
//explore_queue_[order_poses[16]].goal_config.q[0] = 0;
//explore_queue_[order_poses[16]].goal_config.q[1] = 0;
//explore_queue_[order_poses[16]].goal_config.q[2] = 0.382;
//explore_queue_[order_poses[16]].goal_config.q[3] = 0;
//explore_queue_[order_poses[16]].goal_config.q[4] = 1.187;
//explore_queue_[order_poses[16]].goal_config.q[5] = -1.36;
//explore_queue_[order_poses[16]].goal_config.q[6] = -0.45;
//explore_queue_[order_poses[16]].goal_config.q[7] = 1.7;
//explore_queue_[order_poses[16]].goal_config.q[8] = 0;
//
//explore_queue_[order_poses[17]].goal_pose.position.x = -0.527;
//explore_queue_[order_poses[17]].goal_pose.position.y = 0.112;
//explore_queue_[order_poses[17]].goal_pose.position.z = 0.455;
//q_temp[order_poses[17]].setRPY(2.142,0,2.168);
//explore_queue_[order_poses[17]].goal_config.q[0] = 0;
//explore_queue_[order_poses[17]].goal_config.q[1] = 0;
//explore_queue_[order_poses[17]].goal_config.q[2] = 0.6;
//explore_queue_[order_poses[17]].goal_config.q[3] = 1.2;
//explore_queue_[order_poses[17]].goal_config.q[4] = -1.572;
//explore_queue_[order_poses[17]].goal_config.q[5] = -1.572;
//explore_queue_[order_poses[17]].goal_config.q[6] = 0.2;
//explore_queue_[order_poses[17]].goal_config.q[7] = 1.572;
//explore_queue_[order_poses[17]].goal_config.q[8] = 0;
//
//explore_queue_[order_poses[18]].goal_pose.position.x = -0.379;
//explore_queue_[order_poses[18]].goal_pose.position.y = -0.383;
//explore_queue_[order_poses[18]].goal_pose.position.z = 0.455;
//q_temp[order_poses[18]].setRPY(2.142,0,-3.115);
//explore_queue_[order_poses[18]].goal_config.q[0] = 0;
//explore_queue_[order_poses[18]].goal_config.q[1] = 0;
//explore_queue_[order_poses[18]].goal_config.q[2] = 1.572;
//explore_queue_[order_poses[18]].goal_config.q[3] = 1.2;
//explore_queue_[order_poses[18]].goal_config.q[4] = -1.572;
//explore_queue_[order_poses[18]].goal_config.q[5] = -1.572;
//explore_queue_[order_poses[18]].goal_config.q[6] = 0.2;
//explore_queue_[order_poses[18]].goal_config.q[7] = 1.572;
//explore_queue_[order_poses[18]].goal_config.q[8] = 0;
//
//explore_queue_[order_poses[19]].goal_pose.position.x = -0.373;
//explore_queue_[order_poses[19]].goal_pose.position.y = -0.39;
//explore_queue_[order_poses[19]].goal_pose.position.z = 0.455;
//q_temp[order_poses[19]].setRPY(2.142,0,1.573);
//explore_queue_[order_poses[19]].goal_config.q[0] = 0;
//explore_queue_[order_poses[19]].goal_config.q[1] = 0;
//explore_queue_[order_poses[19]].goal_config.q[2] = 0;
//explore_queue_[order_poses[19]].goal_config.q[3] = 1.2;
//explore_queue_[order_poses[19]].goal_config.q[4] = -1.572;
//explore_queue_[order_poses[19]].goal_config.q[5] = 1.572;
//explore_queue_[order_poses[19]].goal_config.q[6] = -0.2;
//explore_queue_[order_poses[19]].goal_config.q[7] = -1.572;
//explore_queue_[order_poses[19]].goal_config.q[8] = 0;
//
//explore_queue_[order_poses[20]].goal_pose.position.x = -0.39;
//explore_queue_[order_poses[20]].goal_pose.position.y = 0.373;
//explore_queue_[order_poses[20]].goal_pose.position.z = 0.455;
//q_temp[order_poses[20]].setRPY(2.142,0,0);
//explore_queue_[order_poses[20]].goal_config.q[0] = 0;
//explore_queue_[order_poses[20]].goal_config.q[1] = 0;
//explore_queue_[order_poses[20]].goal_config.q[2] = -1.572;
//explore_queue_[order_poses[20]].goal_config.q[3] = 1.2;
//explore_queue_[order_poses[20]].goal_config.q[4] = -1.572;
//explore_queue_[order_poses[20]].goal_config.q[5] = 1.572;
//explore_queue_[order_poses[20]].goal_config.q[6] = -0.2;
//explore_queue_[order_poses[20]].goal_config.q[7] = -1.572;
//explore_queue_[order_poses[20]].goal_config.q[8] = 0;
