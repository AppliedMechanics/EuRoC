msg_warn("SNAKE POSES CHOSEN. BE CAREFUL!");
std::vector<tf::Quaternion> q_temp;
active_goal_=0;
nr_goals_=10;
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
