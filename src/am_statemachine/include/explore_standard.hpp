std::vector<tf::Quaternion> q_temp;
active_goal_=0;
nr_goals_=14;
goal_queue.resize(nr_goals_);
q_temp.resize(nr_goals_);

//! Joint 3 =  0
goal_queue[0].goal_pose.position.x = 0.232;
goal_queue[0].goal_pose.position.y = 0.232;
goal_queue[0].goal_pose.position.z = 0.921;
q_temp[0].setRPY(3.14,-0.442,0.786);
goal_queue[0].inter_steps = 0;
goal_queue[0].speed_percentage = fast_moving_speed;

goal_queue[1].goal_pose.position.x = 0.27;
goal_queue[1].goal_pose.position.y = 0.27;
goal_queue[1].goal_pose.position.z = 0.792;
q_temp[1].setRPY(3.14,-0.082,0.786);
goal_queue[1].inter_steps = 0;
goal_queue[1].speed_percentage = fast_moving_speed;

goal_queue[2].goal_pose.position.x = 0;
goal_queue[2].goal_pose.position.y = 0.381;
goal_queue[2].goal_pose.position.z = 0.792;
q_temp[2].setRPY(3.14,-0.082,1.571);
goal_queue[2].inter_steps = 0;
goal_queue[2].speed_percentage = fast_moving_speed;

goal_queue[3].goal_pose.position.x = -0.232;
goal_queue[3].goal_pose.position.y = 0.232;
goal_queue[3].goal_pose.position.z = 0.921;
q_temp[3].setRPY(3.14,-0.442,2.357);
goal_queue[3].inter_steps = 0;
goal_queue[3].speed_percentage = fast_moving_speed;

goal_queue[4].goal_pose.position.x = -0.27;
goal_queue[4].goal_pose.position.y = 0.27;
goal_queue[4].goal_pose.position.z = 0.792;
q_temp[4].setRPY(3.14,-0.082,2.357);
goal_queue[4].inter_steps = 0;
goal_queue[4].speed_percentage = fast_moving_speed;

goal_queue[5].goal_pose.position.x = -0.381;
goal_queue[5].goal_pose.position.y = 0;
goal_queue[5].goal_pose.position.z = 0.792;
q_temp[5].setRPY(3.14,-0.082,-3.14);
goal_queue[5].inter_steps = 0;
goal_queue[5].speed_percentage = fast_moving_speed;

goal_queue[6].goal_pose.position.x = -0.381;
goal_queue[6].goal_pose.position.y = 0;
goal_queue[6].goal_pose.position.z = 0.792;
q_temp[6].setRPY(-2.7,-0.094,-3.085);
goal_queue[6].inter_steps = 0;
goal_queue[6].speed_percentage = fast_moving_speed;

goal_queue[7].goal_pose.position.x = -0.27;
goal_queue[7].goal_pose.position.y = -0.27;
goal_queue[7].goal_pose.position.z = 0.792;
q_temp[7].setRPY(-3.14,-0.082,-2.357);
goal_queue[7].inter_steps = 0;
goal_queue[7].speed_percentage = fast_moving_speed;

goal_queue[8].goal_pose.position.x = -0.232;
goal_queue[8].goal_pose.position.y = -0.232;
goal_queue[8].goal_pose.position.z = 0.921;
q_temp[8].setRPY(-3.14,-0.442,-2.357);
goal_queue[8].inter_steps = 0;
goal_queue[8].speed_percentage = fast_moving_speed;

goal_queue[9].goal_pose.position.x = 0;
goal_queue[9].goal_pose.position.y = -0.381;
goal_queue[9].goal_pose.position.z = 0.792;
q_temp[9].setRPY(2.7,-0.094,-1.627);
goal_queue[9].inter_steps = 0;
goal_queue[9].speed_percentage = fast_moving_speed;

goal_queue[10].goal_pose.position.x = 0;
goal_queue[10].goal_pose.position.y = -0.381;
goal_queue[10].goal_pose.position.z = 0.792;
q_temp[10].setRPY(3.14,-0.082,-1.571);
goal_queue[10].inter_steps = 0;
goal_queue[10].speed_percentage = fast_moving_speed;

goal_queue[11].goal_pose.position.x = 0.232;
goal_queue[11].goal_pose.position.y = -0.232;
goal_queue[11].goal_pose.position.z = 0.921;
q_temp[11].setRPY(3.14,-0.442,-0.786);
goal_queue[11].inter_steps = 0;
goal_queue[11].speed_percentage = fast_moving_speed;

goal_queue[12].goal_pose.position.x = 0.27;
goal_queue[12].goal_pose.position.y = -0.27;
goal_queue[12].goal_pose.position.z = 0.792;
q_temp[12].setRPY(3.14,-0.082,-0.786);
goal_queue[12].inter_steps = 0;
goal_queue[12].speed_percentage = fast_moving_speed;

goal_queue[13].goal_pose.position.x = 0.381;
goal_queue[13].goal_pose.position.y = 0;
goal_queue[13].goal_pose.position.z = 0.792;
q_temp[13].setRPY(3.14,-0.082,0);
goal_queue[13].inter_steps = 0;
goal_queue[13].speed_percentage = fast_moving_speed;


for (int i=0;i<nr_goals_;i++)
{
	goal_queue[i].planning_algorithm = planning_mode_.explore;
	goal_queue[i].planning_frame = LWR_TCP;
	goal_queue[i].goal_pose.orientation.x = q_temp[i].getX();
	goal_queue[i].goal_pose.orientation.y = q_temp[i].getY();
	goal_queue[i].goal_pose.orientation.z = q_temp[i].getZ();
	goal_queue[i].goal_pose.orientation.w = q_temp[i].getW();
}
