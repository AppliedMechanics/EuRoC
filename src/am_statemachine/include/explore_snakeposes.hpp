msg_warn("SNAKE POSES CHOSEN. BE CAREFUL!");
std::vector<tf::Quaternion> q_temp;
active_goal_=0;
nr_goals_=20;
goal_queue.resize(nr_goals_);
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
	goal_queue[i].goal_config.q.resize(9);

goal_queue[order_poses[0]].goal_pose.position.x = 0.336+0.88;
goal_queue[order_poses[0]].goal_pose.position.y = 0.39+0;
goal_queue[order_poses[0]].goal_pose.position.z = 0.525;
q_temp[0].setRPY(3.14,0.86,-0.003);
goal_queue[order_poses[0]].goal_config.q[0] = 0.88;
goal_queue[order_poses[0]].goal_config.q[1] = 0.0;
goal_queue[order_poses[0]].goal_config.q[2] = 0.0;
goal_queue[order_poses[0]].goal_config.q[3] = -1.0;
goal_queue[order_poses[0]].goal_config.q[4] = -1.572;
goal_queue[order_poses[0]].goal_config.q[5] = -1.572;
goal_queue[order_poses[0]].goal_config.q[6] = -0.14;
goal_queue[order_poses[0]].goal_config.q[7] = 1.572;
goal_queue[order_poses[0]].goal_config.q[8] = 1.572;

goal_queue[order_poses[1]].goal_pose.position.x = 0.336+0.88;
goal_queue[order_poses[1]].goal_pose.position.y = -0.39+0.4;
goal_queue[order_poses[1]].goal_pose.position.z = 0.526;
q_temp[1].setRPY(3.14,-0.87,3.141);
goal_queue[order_poses[1]].goal_config.q[0] =  0.88;
goal_queue[order_poses[1]].goal_config.q[1] =  0.4;
goal_queue[order_poses[1]].goal_config.q[2] =  0.0;
goal_queue[order_poses[1]].goal_config.q[3] =  -1.0;
goal_queue[order_poses[1]].goal_config.q[4] =  -1.572;
goal_queue[order_poses[1]].goal_config.q[5] =  1.572;
goal_queue[order_poses[1]].goal_config.q[6] =  0.13;
goal_queue[order_poses[1]].goal_config.q[7] =  -1.572;
goal_queue[order_poses[1]].goal_config.q[8] =  -1.572;

goal_queue[order_poses[2]].goal_pose.position.x = 0.336+0.88;
goal_queue[order_poses[2]].goal_pose.position.y = -0.39+0;
goal_queue[order_poses[2]].goal_pose.position.z = 0.526;
q_temp[2].setRPY(3.14,-0.87,3.141);
goal_queue[order_poses[2]].goal_config.q[0] = 0.88;
goal_queue[order_poses[2]].goal_config.q[1] = 0.0;
goal_queue[order_poses[2]].goal_config.q[2] = 0.0;
goal_queue[order_poses[2]].goal_config.q[3] = -1.0;
goal_queue[order_poses[2]].goal_config.q[4] = -1.572;
goal_queue[order_poses[2]].goal_config.q[5] = 1.572;
goal_queue[order_poses[2]].goal_config.q[6] = 0.13;
goal_queue[order_poses[2]].goal_config.q[7] = -1.572;
goal_queue[order_poses[2]].goal_config.q[8] = -1.572;

goal_queue[order_poses[3]].goal_pose.position.x = 0.336+0.88;
goal_queue[order_poses[3]].goal_pose.position.y = -0.39-0.4;
goal_queue[order_poses[3]].goal_pose.position.z = 0.526;
q_temp[3].setRPY(3.14,-0.87,3.141);
goal_queue[order_poses[3]].goal_config.q[0] = 0.88;
goal_queue[order_poses[3]].goal_config.q[1] = -0.4;
goal_queue[order_poses[3]].goal_config.q[2] = 0.0;
goal_queue[order_poses[3]].goal_config.q[3] = -1.0;
goal_queue[order_poses[3]].goal_config.q[4] = -1.572;
goal_queue[order_poses[3]].goal_config.q[5] = 1.572;
goal_queue[order_poses[3]].goal_config.q[6] = 0.13;
goal_queue[order_poses[3]].goal_config.q[7] = -1.572;
goal_queue[order_poses[3]].goal_config.q[8] = -1.572;

goal_queue[order_poses[4]].goal_pose.position.x = -0.245+0.88;
goal_queue[order_poses[4]].goal_pose.position.y = -0.37-0.88;
goal_queue[order_poses[4]].goal_pose.position.z = 0.785;
q_temp[4].setRPY(-2.659,-0.82,1.759);
goal_queue[order_poses[4]].goal_config.q[0] = 0.88;
goal_queue[order_poses[4]].goal_config.q[1] = -0.88;
goal_queue[order_poses[4]].goal_config.q[2] = -1.2;
goal_queue[order_poses[4]].goal_config.q[3] = -0.5;
goal_queue[order_poses[4]].goal_config.q[4] = -1.572;
goal_queue[order_poses[4]].goal_config.q[5] = 1.2;
goal_queue[order_poses[4]].goal_config.q[6] = -0.35;
goal_queue[order_poses[4]].goal_config.q[7] = -1.572;
goal_queue[order_poses[4]].goal_config.q[8] = -1.572;

goal_queue[order_poses[5]].goal_pose.position.x = 0.39+0;
goal_queue[order_poses[5]].goal_pose.position.y = 0.336+0.88;
goal_queue[order_poses[5]].goal_pose.position.z = 0.526;
q_temp[5].setRPY(3.14,-0.87,-1.57);
goal_queue[order_poses[5]].goal_config.q[0] = 0.0;
goal_queue[order_poses[5]].goal_config.q[1] = 0.88;
goal_queue[order_poses[5]].goal_config.q[2] = 1.572;
goal_queue[order_poses[5]].goal_config.q[3] = -1.0;
goal_queue[order_poses[5]].goal_config.q[4] = -1.572;
goal_queue[order_poses[5]].goal_config.q[5] = 1.572;
goal_queue[order_poses[5]].goal_config.q[6] = 0.13;
goal_queue[order_poses[5]].goal_config.q[7] = -1.572;
goal_queue[order_poses[5]].goal_config.q[8] = -1.572;

goal_queue[order_poses[6]].goal_pose.position.x = -0.39+0.4;
goal_queue[order_poses[6]].goal_pose.position.y = 0.336+0.88;
goal_queue[order_poses[6]].goal_pose.position.z = 0.525;
q_temp[6].setRPY(3.14,0.86,1.57);
goal_queue[order_poses[6]].goal_config.q[0] = 0.4;
goal_queue[order_poses[6]].goal_config.q[1] = 0.88;
goal_queue[order_poses[6]].goal_config.q[2] = 1.572;
goal_queue[order_poses[6]].goal_config.q[3] = -1.0;
goal_queue[order_poses[6]].goal_config.q[4] = -1.572;
goal_queue[order_poses[6]].goal_config.q[5] = -1.572;
goal_queue[order_poses[6]].goal_config.q[6] = -0.14;
goal_queue[order_poses[6]].goal_config.q[7] = 1.572;
goal_queue[order_poses[6]].goal_config.q[8] = 1.572;

goal_queue[order_poses[7]].goal_pose.position.x = -0.39+0;
goal_queue[order_poses[7]].goal_pose.position.y = 0.336+0.88;
goal_queue[order_poses[7]].goal_pose.position.z = 0.525;
q_temp[7].setRPY(3.14,0.86,1.57);
goal_queue[order_poses[7]].goal_config.q[0] = 0.0;
goal_queue[order_poses[7]].goal_config.q[1] = 0.88;
goal_queue[order_poses[7]].goal_config.q[2] = 1.572;
goal_queue[order_poses[7]].goal_config.q[3] = -1.0;
goal_queue[order_poses[7]].goal_config.q[4] = -1.572;
goal_queue[order_poses[7]].goal_config.q[5] = -1.572;
goal_queue[order_poses[7]].goal_config.q[6] = -0.14;
goal_queue[order_poses[7]].goal_config.q[7] = 1.572;
goal_queue[order_poses[7]].goal_config.q[8] = 1.572;

goal_queue[order_poses[8]].goal_pose.position.x = -0.39-0.4;
goal_queue[order_poses[8]].goal_pose.position.y = 0.336+0.88;
goal_queue[order_poses[8]].goal_pose.position.z = 0.525;
q_temp[8].setRPY(3.14,0.86,1.57);
goal_queue[order_poses[8]].goal_config.q[0] = -0.4;
goal_queue[order_poses[8]].goal_config.q[1] = 0.88;
goal_queue[order_poses[8]].goal_config.q[2] = 1.572;
goal_queue[order_poses[8]].goal_config.q[3] = -1.0;
goal_queue[order_poses[8]].goal_config.q[4] = -1.572;
goal_queue[order_poses[8]].goal_config.q[5] = -1.572;
goal_queue[order_poses[8]].goal_config.q[6] = -0.14;
goal_queue[order_poses[8]].goal_config.q[7] = 1.572;
goal_queue[order_poses[8]].goal_config.q[8] = 1.572;

goal_queue[order_poses[9]].goal_pose.position.x = -0.374-0.88;
goal_queue[order_poses[9]].goal_pose.position.y = -0.245+0.88;
goal_queue[order_poses[9]].goal_pose.position.z = 0.785;
q_temp[9].setRPY(-2.66,0.82,2.952);
goal_queue[order_poses[9]].goal_config.q[0] = -0.88;
goal_queue[order_poses[9]].goal_config.q[1] = 0.88;
goal_queue[order_poses[9]].goal_config.q[2] = 2.772;
goal_queue[order_poses[9]].goal_config.q[3] = -0.5;
goal_queue[order_poses[9]].goal_config.q[4] = 1.572;
goal_queue[order_poses[9]].goal_config.q[5] = -1.2;
goal_queue[order_poses[9]].goal_config.q[6] = 0.35;
goal_queue[order_poses[9]].goal_config.q[7] = 1.572;
goal_queue[order_poses[9]].goal_config.q[8] = 1.572;

/////////////////////////////////////////////////////////////////////////
// NEW POSES

goal_queue[order_poses[10]].goal_pose.position.x = -0.381;
goal_queue[order_poses[10]].goal_pose.position.y = 0;
goal_queue[order_poses[10]].goal_pose.position.z = 0.792;
q_temp[10].setRPY(2.7,-0.094,-3.085);
goal_queue[order_poses[10]].goal_config.q[0] = 0;
goal_queue[order_poses[10]].goal_config.q[1] = 0;
goal_queue[order_poses[10]].goal_config.q[2] = 0;
goal_queue[order_poses[10]].goal_config.q[3] = 0;
goal_queue[order_poses[10]].goal_config.q[4] = 0;
goal_queue[order_poses[10]].goal_config.q[5] = -1.36;
goal_queue[order_poses[10]].goal_config.q[6] = 0.45;
goal_queue[order_poses[10]].goal_config.q[7] = 1.7;
goal_queue[order_poses[10]].goal_config.q[8] = 0;


goal_queue[order_poses[11]].goal_pose.position.x = -0.27;
goal_queue[order_poses[11]].goal_pose.position.y = -0.27;
goal_queue[order_poses[11]].goal_pose.position.z = 0.792;
q_temp[11].setRPY(-3.14,-0.082,-2.357);
goal_queue[order_poses[11]].goal_config.q[0] = 0;
goal_queue[order_poses[11]].goal_config.q[1] = 0;
goal_queue[order_poses[11]].goal_config.q[2] = 0.193;
goal_queue[order_poses[11]].goal_config.q[3] = 0;
goal_queue[order_poses[11]].goal_config.q[4] = 0.593;
goal_queue[order_poses[11]].goal_config.q[5] = -1.36;
goal_queue[order_poses[11]].goal_config.q[6] = 0;
goal_queue[order_poses[11]].goal_config.q[7] = 1.7;
goal_queue[order_poses[11]].goal_config.q[8] = 0;


goal_queue[order_poses[12]].goal_pose.position.x = -0.232;
goal_queue[order_poses[12]].goal_pose.position.y = -0.232;
goal_queue[order_poses[12]].goal_pose.position.z = 0.921;
q_temp[12].setRPY(-3.14,-0.442,-2.357);
goal_queue[order_poses[12]].goal_config.q[0] = 0;
goal_queue[order_poses[12]].goal_config.q[1] = 0;
goal_queue[order_poses[12]].goal_config.q[2] = 0.193;
goal_queue[order_poses[12]].goal_config.q[3] = 0;
goal_queue[order_poses[12]].goal_config.q[4] = 0.593;
goal_queue[order_poses[12]].goal_config.q[5] = -1;
goal_queue[order_poses[12]].goal_config.q[6] = 0;
goal_queue[order_poses[12]].goal_config.q[7] = 1.7;
goal_queue[order_poses[12]].goal_config.q[8] = 0;


goal_queue[order_poses[13]].goal_pose.position.x = 0;
goal_queue[order_poses[13]].goal_pose.position.y = -0.381;
goal_queue[order_poses[13]].goal_pose.position.z = 0.792;
q_temp[13].setRPY(2.7,-0.094,-1.627);
goal_queue[order_poses[13]].goal_config.q[0] = 0;
goal_queue[order_poses[13]].goal_config.q[1] = 0;
goal_queue[order_poses[13]].goal_config.q[2] = 0.382;
goal_queue[order_poses[13]].goal_config.q[3] = 0;
goal_queue[order_poses[13]].goal_config.q[4] = 1.187;
goal_queue[order_poses[13]].goal_config.q[5] = -1.36;
goal_queue[order_poses[13]].goal_config.q[6] = -0.45;
goal_queue[order_poses[13]].goal_config.q[7] = 1.7;
goal_queue[order_poses[13]].goal_config.q[8] = 0;


goal_queue[order_poses[14]].goal_pose.position.x = 0.3-0.143;
goal_queue[order_poses[14]].goal_pose.position.y = -0.3-0.354;
goal_queue[order_poses[14]].goal_pose.position.z = 0.792;
q_temp[14].setRPY(2.45,-0.11,-2.037);
goal_queue[order_poses[14]].goal_config.q[0] = 0.3;
goal_queue[order_poses[14]].goal_config.q[1] = 0.3;
goal_queue[order_poses[14]].goal_config.q[2] = 0;
goal_queue[order_poses[14]].goal_config.q[3] = 0;
goal_queue[order_poses[14]].goal_config.q[4] = 1.187;
goal_queue[order_poses[14]].goal_config.q[5] = -1.36;
goal_queue[order_poses[14]].goal_config.q[6] = -0.7;
goal_queue[order_poses[14]].goal_config.q[7] = 1.7;
goal_queue[order_poses[14]].goal_config.q[8] = 0;


goal_queue[order_poses[15]].goal_pose.position.x = -0.3-0.354;
goal_queue[order_poses[15]].goal_pose.position.y = 0.3-0.143;
goal_queue[order_poses[15]].goal_pose.position.z = 0.792;
q_temp[15].setRPY(-2.45,-0.11,-2.671);
goal_queue[order_poses[15]].goal_config.q[0] = -0.3;
goal_queue[order_poses[15]].goal_config.q[1] = 0.3;
goal_queue[order_poses[15]].goal_config.q[2] = 0;
goal_queue[order_poses[15]].goal_config.q[3] = 0;
goal_queue[order_poses[15]].goal_config.q[4] = 0.385;
goal_queue[order_poses[15]].goal_config.q[5] = -1.36;
goal_queue[order_poses[15]].goal_config.q[6] = 0.7;
goal_queue[order_poses[15]].goal_config.q[7] = 1.7;
goal_queue[order_poses[15]].goal_config.q[8] = 0;


goal_queue[order_poses[16]].goal_pose.position.x = -0.5-0.172;
goal_queue[order_poses[16]].goal_pose.position.y = 0.2-0.533;
goal_queue[order_poses[16]].goal_pose.position.z = 0.563;
q_temp[16].setRPY(-2.224,0.334,-1.647);
goal_queue[order_poses[16]].goal_config.q[0] = -0.5;
goal_queue[order_poses[16]].goal_config.q[1] = 0.2;
goal_queue[order_poses[16]].goal_config.q[2] = 1;
goal_queue[order_poses[16]].goal_config.q[3] = 0.5;
goal_queue[order_poses[16]].goal_config.q[4] = 0.385;
goal_queue[order_poses[16]].goal_config.q[5] = -1.36;
goal_queue[order_poses[16]].goal_config.q[6] = 0.7;
goal_queue[order_poses[16]].goal_config.q[7] = 1.7;
goal_queue[order_poses[16]].goal_config.q[8] = 0;


goal_queue[order_poses[17]].goal_pose.position.x = -0.5-0.079;
goal_queue[order_poses[17]].goal_pose.position.y = 0.2-0.506;
goal_queue[order_poses[17]].goal_pose.position.z = 0.65;
q_temp[17].setRPY(-2.293,0.115,-0.986);
goal_queue[order_poses[17]].goal_config.q[0] = -0.5;
goal_queue[order_poses[17]].goal_config.q[1] = 0.2;
goal_queue[order_poses[17]].goal_config.q[2] = 1;
goal_queue[order_poses[17]].goal_config.q[3] = 0.5;
goal_queue[order_poses[17]].goal_config.q[4] = 1.1;
goal_queue[order_poses[17]].goal_config.q[5] = -1.36;
goal_queue[order_poses[17]].goal_config.q[6] = 0.4;
goal_queue[order_poses[17]].goal_config.q[7] = 1.7;
goal_queue[order_poses[17]].goal_config.q[8] = 0;


goal_queue[order_poses[18]].goal_pose.position.x = 0.2-0.531;
goal_queue[order_poses[18]].goal_pose.position.y = -0.5-0.176;
goal_queue[order_poses[18]].goal_pose.position.z = 0.563;
q_temp[18].setRPY(2.224,0.334,-3.059);
goal_queue[order_poses[18]].goal_config.q[0] = 0.2;
goal_queue[order_poses[18]].goal_config.q[1] = -0.5;
goal_queue[order_poses[18]].goal_config.q[2] = 0.578;
goal_queue[order_poses[18]].goal_config.q[3] = 0.5;
goal_queue[order_poses[18]].goal_config.q[4] = -0.385;
goal_queue[order_poses[18]].goal_config.q[5] = -1.36;
goal_queue[order_poses[18]].goal_config.q[6] = -0.7;
goal_queue[order_poses[18]].goal_config.q[7] = 1.7;
goal_queue[order_poses[18]].goal_config.q[8] = 0;


goal_queue[order_poses[19]].goal_pose.position.x = 0.2-0.506;
goal_queue[order_poses[19]].goal_pose.position.y = -0.5-0.791;
goal_queue[order_poses[19]].goal_pose.position.z = 0.65;
q_temp[19].setRPY(2.293,0.115,2.557);
goal_queue[order_poses[19]].goal_config.q[0] = 0.2;
goal_queue[order_poses[19]].goal_config.q[1] = -0.5;
goal_queue[order_poses[19]].goal_config.q[2] = 0.571;
goal_queue[order_poses[19]].goal_config.q[3] = 0.5;
goal_queue[order_poses[19]].goal_config.q[4] = -1.1;
goal_queue[order_poses[19]].goal_config.q[5] = -1.36;
goal_queue[order_poses[19]].goal_config.q[6] = -0.4;
goal_queue[order_poses[19]].goal_config.q[7] = 1.7;
goal_queue[order_poses[19]].goal_config.q[8] = 0;

////////////////////////////////////////////////////////////////////////////

for (int i=0;i<nr_goals_;i++)
{
	goal_queue[i].planning_algorithm = MOVE_IT_JT_9DOF;
	goal_queue[i].planning_frame = LWR_TCP;
	goal_queue[i].goal_pose.orientation.x = q_temp[i].getX();
	goal_queue[i].goal_pose.orientation.y = q_temp[i].getY();
	goal_queue[i].goal_pose.orientation.z = q_temp[i].getZ();
	goal_queue[i].goal_pose.orientation.w = q_temp[i].getW();

	goal_queue[i].inter_steps = 0;
	goal_queue[i].speed_percentage = std_moving_speed;
}

