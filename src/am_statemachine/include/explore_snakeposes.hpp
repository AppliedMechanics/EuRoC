msg_warn("SNAKE POSES CHOSEN. BE CAREFUL!");
std::vector<tf::Quaternion> q_temp;
active_goal_=0;
nr_goals_=20;

if (nr_goals_!=nr_exp_poses_)
	msg_error("Nr of explore poses different from poses defined in explore_snakeposes.hpp");

explore_queue_.resize(nr_goals_);
q_temp.resize(nr_goals_);



std::vector<int> order_poses;
order_poses.resize(nr_goals_);
order_poses[0] = 0;
order_poses[1] = 4;
order_poses[2] = 3;
order_poses[3] = 2;
order_poses[4] = 1;
order_poses[5] = 5;
order_poses[6] = 9;
order_poses[7] = 8;
order_poses[8] = 7;
order_poses[9] = 6;
order_poses[10] = 10;
order_poses[11] = 11;
order_poses[12] = 12;
order_poses[13] = 13;
order_poses[14] = 14;
order_poses[15] = 15;
order_poses[16] = 16;
order_poses[17] = 17;
order_poses[18] = 18;
order_poses[19] = 19;


for (int i=0;i<nr_goals_;i++)
	explore_queue_[i].goal_config.q.resize(9);

explore_queue_[order_poses[0]].goal_pose.position.x = 0.336+0.88;
explore_queue_[order_poses[0]].goal_pose.position.y = 0.39+0;
explore_queue_[order_poses[0]].goal_pose.position.z = 0.525;
q_temp[order_poses[0]].setRPY(3.14,0.86,-0.003);
explore_queue_[order_poses[0]].goal_config.q[0] = 0.88;
explore_queue_[order_poses[0]].goal_config.q[1] = 0.0;
explore_queue_[order_poses[0]].goal_config.q[2] = 0.0;
explore_queue_[order_poses[0]].goal_config.q[3] = -1.0;
explore_queue_[order_poses[0]].goal_config.q[4] = -1.572;
explore_queue_[order_poses[0]].goal_config.q[5] = -1.572;
explore_queue_[order_poses[0]].goal_config.q[6] = -0.14;
explore_queue_[order_poses[0]].goal_config.q[7] = 1.572;
explore_queue_[order_poses[0]].goal_config.q[8] = 1.572;

explore_queue_[order_poses[1]].goal_pose.position.x = 0.336+0.88;
explore_queue_[order_poses[1]].goal_pose.position.y = -0.39+0.4;
explore_queue_[order_poses[1]].goal_pose.position.z = 0.526;
q_temp[order_poses[1]].setRPY(3.14,-0.87,3.141);
explore_queue_[order_poses[1]].goal_config.q[0] =  0.88;
explore_queue_[order_poses[1]].goal_config.q[1] =  0.4;
explore_queue_[order_poses[1]].goal_config.q[2] =  0.0;
explore_queue_[order_poses[1]].goal_config.q[3] =  -1.0;
explore_queue_[order_poses[1]].goal_config.q[4] =  -1.572;
explore_queue_[order_poses[1]].goal_config.q[5] =  1.572;
explore_queue_[order_poses[1]].goal_config.q[6] =  0.13;
explore_queue_[order_poses[1]].goal_config.q[7] =  -1.572;
explore_queue_[order_poses[1]].goal_config.q[8] =  -1.572;

explore_queue_[order_poses[2]].goal_pose.position.x = 0.336+0.88;
explore_queue_[order_poses[2]].goal_pose.position.y = -0.39+0;
explore_queue_[order_poses[2]].goal_pose.position.z = 0.526;
q_temp[order_poses[2]].setRPY(3.14,-0.87,3.141);
explore_queue_[order_poses[2]].goal_config.q[0] = 0.88;
explore_queue_[order_poses[2]].goal_config.q[1] = 0.0;
explore_queue_[order_poses[2]].goal_config.q[2] = 0.0;
explore_queue_[order_poses[2]].goal_config.q[3] = -1.0;
explore_queue_[order_poses[2]].goal_config.q[4] = -1.572;
explore_queue_[order_poses[2]].goal_config.q[5] = 1.572;
explore_queue_[order_poses[2]].goal_config.q[6] = 0.13;
explore_queue_[order_poses[2]].goal_config.q[7] = -1.572;
explore_queue_[order_poses[2]].goal_config.q[8] = -1.572;

explore_queue_[order_poses[3]].goal_pose.position.x = 0.336+0.88;
explore_queue_[order_poses[3]].goal_pose.position.y = -0.39-0.4;
explore_queue_[order_poses[3]].goal_pose.position.z = 0.526;
q_temp[order_poses[3]].setRPY(3.14,-0.87,3.141);
explore_queue_[order_poses[3]].goal_config.q[0] = 0.88;
explore_queue_[order_poses[3]].goal_config.q[1] = -0.4;
explore_queue_[order_poses[3]].goal_config.q[2] = 0.0;
explore_queue_[order_poses[3]].goal_config.q[3] = -1.0;
explore_queue_[order_poses[3]].goal_config.q[4] = -1.572;
explore_queue_[order_poses[3]].goal_config.q[5] = 1.572;
explore_queue_[order_poses[3]].goal_config.q[6] = 0.13;
explore_queue_[order_poses[3]].goal_config.q[7] = -1.572;
explore_queue_[order_poses[3]].goal_config.q[8] = -1.572;

explore_queue_[order_poses[4]].goal_pose.position.x = -0.245+0.88;
explore_queue_[order_poses[4]].goal_pose.position.y = -0.37-0.88;
explore_queue_[order_poses[4]].goal_pose.position.z = 0.785;
q_temp[order_poses[4]].setRPY(-2.659,-0.82,1.759);
explore_queue_[order_poses[4]].goal_config.q[0] = 0.88;
explore_queue_[order_poses[4]].goal_config.q[1] = -0.88;
explore_queue_[order_poses[4]].goal_config.q[2] = -1.2;
explore_queue_[order_poses[4]].goal_config.q[3] = -0.5;
explore_queue_[order_poses[4]].goal_config.q[4] = -1.572;
explore_queue_[order_poses[4]].goal_config.q[5] = 1.2;
explore_queue_[order_poses[4]].goal_config.q[6] = -0.35;
explore_queue_[order_poses[4]].goal_config.q[7] = -1.572;
explore_queue_[order_poses[4]].goal_config.q[8] = -1.572;

explore_queue_[order_poses[5]].goal_pose.position.x = 0.39+0;
explore_queue_[order_poses[5]].goal_pose.position.y = 0.336+0.88;
explore_queue_[order_poses[5]].goal_pose.position.z = 0.526;
q_temp[order_poses[5]].setRPY(3.14,-0.87,-1.57);
explore_queue_[order_poses[5]].goal_config.q[0] = 0.0;
explore_queue_[order_poses[5]].goal_config.q[1] = 0.88;
explore_queue_[order_poses[5]].goal_config.q[2] = 1.572;
explore_queue_[order_poses[5]].goal_config.q[3] = -1.0;
explore_queue_[order_poses[5]].goal_config.q[4] = -1.572;
explore_queue_[order_poses[5]].goal_config.q[5] = 1.572;
explore_queue_[order_poses[5]].goal_config.q[6] = 0.13;
explore_queue_[order_poses[5]].goal_config.q[7] = -1.572;
explore_queue_[order_poses[5]].goal_config.q[8] = -1.572;

explore_queue_[order_poses[6]].goal_pose.position.x = -0.39+0.4;
explore_queue_[order_poses[6]].goal_pose.position.y = 0.336+0.88;
explore_queue_[order_poses[6]].goal_pose.position.z = 0.525;
q_temp[order_poses[6]].setRPY(3.14,0.86,1.57);
explore_queue_[order_poses[6]].goal_config.q[0] = 0.4;
explore_queue_[order_poses[6]].goal_config.q[1] = 0.88;
explore_queue_[order_poses[6]].goal_config.q[2] = 1.572;
explore_queue_[order_poses[6]].goal_config.q[3] = -1.0;
explore_queue_[order_poses[6]].goal_config.q[4] = -1.572;
explore_queue_[order_poses[6]].goal_config.q[5] = -1.572;
explore_queue_[order_poses[6]].goal_config.q[6] = -0.14;
explore_queue_[order_poses[6]].goal_config.q[7] = 1.572;
explore_queue_[order_poses[6]].goal_config.q[8] = 1.572;

explore_queue_[order_poses[7]].goal_pose.position.x = -0.39+0;
explore_queue_[order_poses[7]].goal_pose.position.y = 0.336+0.88;
explore_queue_[order_poses[7]].goal_pose.position.z = 0.525;
q_temp[order_poses[7]].setRPY(3.14,0.86,1.57);
explore_queue_[order_poses[7]].goal_config.q[0] = 0.0;
explore_queue_[order_poses[7]].goal_config.q[1] = 0.88;
explore_queue_[order_poses[7]].goal_config.q[2] = 1.572;
explore_queue_[order_poses[7]].goal_config.q[3] = -1.0;
explore_queue_[order_poses[7]].goal_config.q[4] = -1.572;
explore_queue_[order_poses[7]].goal_config.q[5] = -1.572;
explore_queue_[order_poses[7]].goal_config.q[6] = -0.14;
explore_queue_[order_poses[7]].goal_config.q[7] = 1.572;
explore_queue_[order_poses[7]].goal_config.q[8] = 1.572;

explore_queue_[order_poses[8]].goal_pose.position.x = -0.39-0.4;
explore_queue_[order_poses[8]].goal_pose.position.y = 0.336+0.88;
explore_queue_[order_poses[8]].goal_pose.position.z = 0.525;
q_temp[order_poses[8]].setRPY(3.14,0.86,1.57);
explore_queue_[order_poses[8]].goal_config.q[0] = -0.4;
explore_queue_[order_poses[8]].goal_config.q[1] = 0.88;
explore_queue_[order_poses[8]].goal_config.q[2] = 1.572;
explore_queue_[order_poses[8]].goal_config.q[3] = -1.0;
explore_queue_[order_poses[8]].goal_config.q[4] = -1.572;
explore_queue_[order_poses[8]].goal_config.q[5] = -1.572;
explore_queue_[order_poses[8]].goal_config.q[6] = -0.14;
explore_queue_[order_poses[8]].goal_config.q[7] = 1.572;
explore_queue_[order_poses[8]].goal_config.q[8] = 1.572;

explore_queue_[order_poses[9]].goal_pose.position.x = -0.374-0.88;
explore_queue_[order_poses[9]].goal_pose.position.y = -0.245+0.88;
explore_queue_[order_poses[9]].goal_pose.position.z = 0.785;
q_temp[order_poses[9]].setRPY(-2.66,0.82,2.952);
explore_queue_[order_poses[9]].goal_config.q[0] = -0.88;
explore_queue_[order_poses[9]].goal_config.q[1] = 0.88;
explore_queue_[order_poses[9]].goal_config.q[2] = 2.772;
explore_queue_[order_poses[9]].goal_config.q[3] = -0.5;
explore_queue_[order_poses[9]].goal_config.q[4] = -1.572;
explore_queue_[order_poses[9]].goal_config.q[5] = -1.2;
explore_queue_[order_poses[9]].goal_config.q[6] = 0.35;
explore_queue_[order_poses[9]].goal_config.q[7] = 1.572;
explore_queue_[order_poses[9]].goal_config.q[8] = 1.572;

/////////////////////////////////////////////////////////////////////////
// NEW POSES

explore_queue_[order_poses[10]].goal_pose.position.x = -0.381;
explore_queue_[order_poses[10]].goal_pose.position.y = 0;
explore_queue_[order_poses[10]].goal_pose.position.z = 0.792;
q_temp[order_poses[10]].setRPY(2.7,-0.094,-3.085);
explore_queue_[order_poses[10]].goal_config.q[0] = 0;
explore_queue_[order_poses[10]].goal_config.q[1] = 0;
explore_queue_[order_poses[10]].goal_config.q[2] = 0;
explore_queue_[order_poses[10]].goal_config.q[3] = 0;
explore_queue_[order_poses[10]].goal_config.q[4] = 0;
explore_queue_[order_poses[10]].goal_config.q[5] = -1.36;
explore_queue_[order_poses[10]].goal_config.q[6] = 0.45;
explore_queue_[order_poses[10]].goal_config.q[7] = 1.7;
explore_queue_[order_poses[10]].goal_config.q[8] = 0;


explore_queue_[order_poses[11]].goal_pose.position.x = -0.27;
explore_queue_[order_poses[11]].goal_pose.position.y = -0.27;
explore_queue_[order_poses[11]].goal_pose.position.z = 0.792;
q_temp[order_poses[11]].setRPY(-3.14,-0.082,-2.357);
explore_queue_[order_poses[11]].goal_config.q[0] = 0;
explore_queue_[order_poses[11]].goal_config.q[1] = 0;
explore_queue_[order_poses[11]].goal_config.q[2] = 0.193;
explore_queue_[order_poses[11]].goal_config.q[3] = 0;
explore_queue_[order_poses[11]].goal_config.q[4] = 0.593;
explore_queue_[order_poses[11]].goal_config.q[5] = -1.36;
explore_queue_[order_poses[11]].goal_config.q[6] = 0;
explore_queue_[order_poses[11]].goal_config.q[7] = 1.7;
explore_queue_[order_poses[11]].goal_config.q[8] = 0;


explore_queue_[order_poses[12]].goal_pose.position.x = -0.232;
explore_queue_[order_poses[12]].goal_pose.position.y = -0.232;
explore_queue_[order_poses[12]].goal_pose.position.z = 0.921;
q_temp[order_poses[12]].setRPY(-3.14,-0.442,-2.357);
explore_queue_[order_poses[12]].goal_config.q[0] = 0;
explore_queue_[order_poses[12]].goal_config.q[1] = 0;
explore_queue_[order_poses[12]].goal_config.q[2] = 0.193;
explore_queue_[order_poses[12]].goal_config.q[3] = 0;
explore_queue_[order_poses[12]].goal_config.q[4] = 0.593;
explore_queue_[order_poses[12]].goal_config.q[5] = -1;
explore_queue_[order_poses[12]].goal_config.q[6] = 0;
explore_queue_[order_poses[12]].goal_config.q[7] = 1.7;
explore_queue_[order_poses[12]].goal_config.q[8] = 0;


explore_queue_[order_poses[13]].goal_pose.position.x = 0;
explore_queue_[order_poses[13]].goal_pose.position.y = -0.381;
explore_queue_[order_poses[13]].goal_pose.position.z = 0.792;
q_temp[order_poses[13]].setRPY(2.7,-0.094,-1.627);
explore_queue_[order_poses[13]].goal_config.q[0] = 0;
explore_queue_[order_poses[13]].goal_config.q[1] = 0;
explore_queue_[order_poses[13]].goal_config.q[2] = 0.382;
explore_queue_[order_poses[13]].goal_config.q[3] = 0;
explore_queue_[order_poses[13]].goal_config.q[4] = 1.187;
explore_queue_[order_poses[13]].goal_config.q[5] = -1.36;
explore_queue_[order_poses[13]].goal_config.q[6] = -0.45;
explore_queue_[order_poses[13]].goal_config.q[7] = 1.7;
explore_queue_[order_poses[13]].goal_config.q[8] = 0;


explore_queue_[order_poses[14]].goal_pose.position.x = 0.3-0.143;
explore_queue_[order_poses[14]].goal_pose.position.y = -0.3-0.354;
explore_queue_[order_poses[14]].goal_pose.position.z = 0.792;
q_temp[order_poses[14]].setRPY(2.45,-0.11,-2.037);
explore_queue_[order_poses[14]].goal_config.q[0] = 0.3;
explore_queue_[order_poses[14]].goal_config.q[1] = 0.3;
explore_queue_[order_poses[14]].goal_config.q[2] = 0;
explore_queue_[order_poses[14]].goal_config.q[3] = 0;
explore_queue_[order_poses[14]].goal_config.q[4] = 1.187;
explore_queue_[order_poses[14]].goal_config.q[5] = -1.36;
explore_queue_[order_poses[14]].goal_config.q[6] = -0.7;
explore_queue_[order_poses[14]].goal_config.q[7] = 1.7;
explore_queue_[order_poses[14]].goal_config.q[8] = 0;


explore_queue_[order_poses[15]].goal_pose.position.x = -0.3-0.354;
explore_queue_[order_poses[15]].goal_pose.position.y = 0.3-0.143;
explore_queue_[order_poses[15]].goal_pose.position.z = 0.792;
q_temp[order_poses[15]].setRPY(-2.45,-0.11,-2.671);
explore_queue_[order_poses[15]].goal_config.q[0] = -0.3;
explore_queue_[order_poses[15]].goal_config.q[1] = 0.3;
explore_queue_[order_poses[15]].goal_config.q[2] = 0;
explore_queue_[order_poses[15]].goal_config.q[3] = 0;
explore_queue_[order_poses[15]].goal_config.q[4] = 0.385;
explore_queue_[order_poses[15]].goal_config.q[5] = -1.36;
explore_queue_[order_poses[15]].goal_config.q[6] = 0.7;
explore_queue_[order_poses[15]].goal_config.q[7] = 1.7;
explore_queue_[order_poses[15]].goal_config.q[8] = 0;


explore_queue_[order_poses[16]].goal_pose.position.x = -0.5-0.172;
explore_queue_[order_poses[16]].goal_pose.position.y = 0.2-0.533;
explore_queue_[order_poses[16]].goal_pose.position.z = 0.563;
q_temp[order_poses[16]].setRPY(-2.224,0.334,-1.647);
explore_queue_[order_poses[16]].goal_config.q[0] = -0.5;
explore_queue_[order_poses[16]].goal_config.q[1] = 0.2;
explore_queue_[order_poses[16]].goal_config.q[2] = 1;
explore_queue_[order_poses[16]].goal_config.q[3] = 0.5;
explore_queue_[order_poses[16]].goal_config.q[4] = 0.385;
explore_queue_[order_poses[16]].goal_config.q[5] = -1.36;
explore_queue_[order_poses[16]].goal_config.q[6] = 0.7;
explore_queue_[order_poses[16]].goal_config.q[7] = 1.7;
explore_queue_[order_poses[16]].goal_config.q[8] = 0;


explore_queue_[order_poses[17]].goal_pose.position.x = -0.5-0.079;
explore_queue_[order_poses[17]].goal_pose.position.y = 0.2-0.506;
explore_queue_[order_poses[17]].goal_pose.position.z = 0.65;
q_temp[order_poses[17]].setRPY(-2.293,0.115,-0.986);
explore_queue_[order_poses[17]].goal_config.q[0] = -0.5;
explore_queue_[order_poses[17]].goal_config.q[1] = 0.2;
explore_queue_[order_poses[17]].goal_config.q[2] = 1;
explore_queue_[order_poses[17]].goal_config.q[3] = 0.5;
explore_queue_[order_poses[17]].goal_config.q[4] = 1.1;
explore_queue_[order_poses[17]].goal_config.q[5] = -1.36;
explore_queue_[order_poses[17]].goal_config.q[6] = 0.4;
explore_queue_[order_poses[17]].goal_config.q[7] = 1.7;
explore_queue_[order_poses[17]].goal_config.q[8] = 0;


explore_queue_[order_poses[18]].goal_pose.position.x = 0.2-0.531;
explore_queue_[order_poses[18]].goal_pose.position.y = -0.5-0.176;
explore_queue_[order_poses[18]].goal_pose.position.z = 0.563;
q_temp[order_poses[18]].setRPY(2.224,0.334,-3.059);
explore_queue_[order_poses[18]].goal_config.q[0] = 0.2;
explore_queue_[order_poses[18]].goal_config.q[1] = -0.5;
explore_queue_[order_poses[18]].goal_config.q[2] = 0.578;
explore_queue_[order_poses[18]].goal_config.q[3] = 0.5;
explore_queue_[order_poses[18]].goal_config.q[4] = -0.385;
explore_queue_[order_poses[18]].goal_config.q[5] = -1.36;
explore_queue_[order_poses[18]].goal_config.q[6] = -0.7;
explore_queue_[order_poses[18]].goal_config.q[7] = 1.7;
explore_queue_[order_poses[18]].goal_config.q[8] = 0;


explore_queue_[order_poses[19]].goal_pose.position.x = 0.2-0.506;
explore_queue_[order_poses[19]].goal_pose.position.y = -0.5-0.791;
explore_queue_[order_poses[19]].goal_pose.position.z = 0.65;
q_temp[order_poses[19]].setRPY(2.293,0.115,2.557);
explore_queue_[order_poses[19]].goal_config.q[0] = 0.2;
explore_queue_[order_poses[19]].goal_config.q[1] = -0.5;
explore_queue_[order_poses[19]].goal_config.q[2] = 0.571;
explore_queue_[order_poses[19]].goal_config.q[3] = 0.5;
explore_queue_[order_poses[19]].goal_config.q[4] = -1.1;
explore_queue_[order_poses[19]].goal_config.q[5] = -1.36;
explore_queue_[order_poses[19]].goal_config.q[6] = -0.4;
explore_queue_[order_poses[19]].goal_config.q[7] = 1.7;
explore_queue_[order_poses[19]].goal_config.q[8] = 0;

////////////////////////////////////////////////////////////////////////////

for (int i=0;i<nr_goals_;i++)
{
	explore_queue_[i].planning_algorithm = MOVE_IT_JT_9DOF;
	explore_queue_[i].planning_frame = LWR_TCP;
	explore_queue_[i].goal_pose.orientation.x = q_temp[order_poses[i]].getX();
	explore_queue_[i].goal_pose.orientation.y = q_temp[order_poses[i]].getY();
	explore_queue_[i].goal_pose.orientation.z = q_temp[order_poses[i]].getZ();
	explore_queue_[i].goal_pose.orientation.w = q_temp[order_poses[i]].getW();

	explore_queue_[i].inter_steps = 0;
	explore_queue_[i].speed_percentage = fast_moving_speed;
}

