/*
 * VisionDummy.cpp
 *
 *  Created on: Sep 12, 2014
 *      Author: euroc_admin
 */

#include "vision_dummy.hpp"

VisionDummy::VisionDummy() {
	// TODO Auto-generated constructor stub
	mod_scene_ = 16;
	ros::param::get("/task_nr",mod_scene_);
	loadScene(mod_scene_);

	ROS_INFO("Vision Dummy has loaded Scene %i",mod_scene_);

}

VisionDummy::~VisionDummy() {
	// TODO Auto-generated destructor stub
}

void VisionDummy::handle(const am_msgs::VisionGoal::ConstPtr &goal)
{
	ROS_INFO("Entered VisionDummy::handle()...");

	obj_aligned_=false;
	int obj_idx;

	if(!goal->object.color.compare("ff0000"))
		obj_idx = 0;
	else if (!goal->object.color.compare("00ff00"))
		obj_idx = 1;
	else if (!goal->object.color.compare("0000ff"))
		obj_idx = 2;
	else if (!goal->object.color.compare("00ffff"))
		obj_idx = 3;
	else if (!goal->object.color.compare("ff00ff"))
		obj_idx = 4;
	else if (!goal->object.color.compare("ffff00"))
		obj_idx = 5;

	// ==========================================================================================

	//-------------------------------- SEND RESULT ----------------------------------------//

	vision_result_.abs_object_pose = modscene_poses_[obj_idx];

	vision_feedback_.execution_time = ros::Time::now().sec;
	vision_server_.publishFeedback(vision_feedback_);


	if(failed)
	{
		vision_result_.object_detected=false;
		vision_server_.setPreempted(vision_result_,"Got no telemetry.");
	}
	else
	{
		vision_result_.object_detected=true;
		vision_server_.setSucceeded(vision_result_, "Goal configuration has been reached");
	}

}


void VisionDummy::loadScene(int scene)
{
	tf::Quaternion q_tmp;
	switch (scene)
	{
	case 11:
		modscene_poses_.resize(3);
		//	    red_cube:
		//	      start_pose: [-0.3, -0.4, 0.03, 0, 0, 0]
		//	    green_cylinder:
		//	      start_pose: [-0.5, 0.1, 0.051, -3.1415, 0, 0.8]
		//	    blue_handle:
		//	      start_pose: [0, 0.5, 0.03, -1.5, 0, 0.4]
		//red
		modscene_poses_[0].position.x = -0.3;
		modscene_poses_[0].position.y = -0.44;
		modscene_poses_[0].position.z = 0.025;
		q_tmp.setRPY(0,0,0);
		modscene_poses_[0].orientation.w = q_tmp.getW();
		modscene_poses_[0].orientation.x = q_tmp.getX();
		modscene_poses_[0].orientation.y = q_tmp.getY();
		modscene_poses_[0].orientation.z = q_tmp.getZ();

		// green
		modscene_poses_[1].position.x = -0.5;
		modscene_poses_[1].position.y = 0.1;
		modscene_poses_[1].position.z = 0.05;
		q_tmp.setRPY(-3.1415, 0, 0.8);
		modscene_poses_[1].orientation.w = q_tmp.getW();
		modscene_poses_[1].orientation.x = q_tmp.getX();
		modscene_poses_[1].orientation.y = q_tmp.getY();
		modscene_poses_[1].orientation.z = q_tmp.getZ();

		//blue
		modscene_poses_[2].position.x = 0;
		modscene_poses_[2].position.y = 0.5;
		modscene_poses_[2].position.z = 0.025;
		q_tmp.setRPY(-1.5, 0, 0.4);
		modscene_poses_[2].orientation.w = q_tmp.getW();
		modscene_poses_[2].orientation.x = q_tmp.getX();
		modscene_poses_[2].orientation.y = q_tmp.getY();
		modscene_poses_[2].orientation.z = q_tmp.getZ();
		break;
	case 12:
	case 13:
	case 14:
		modscene_poses_.resize(6);
		//	    red_handle:
		//	      start_pose: [-0.7, -0.3, 0.03, 1.50, 0, 0]
		//	    green_handle:
		//	      start_pose: [-0.5, -0.3, 0.03, -1.500, 0, 0]
		//	    blue_handle:
		//	      start_pose: [-0.2, -0.3, 0.03, 1.50, 0, 0]
		//	    cyan_handle:
		//	      start_pose: [0.2, -0.3, 0.03, -1.50, 0, 0]
		//	    magenta_handle:
		//	      start_pose: [0.5, -0.3, 0.03, 1.50, 0, 0]
		//	    yellow_handle:
		//	      start_pose: [0.7, -0.3, 0.03, 0, 0, 0]
		//red
		modscene_poses_[0].position.x = -0.7;
		modscene_poses_[0].position.y = -0.3;
		modscene_poses_[0].position.z = 0.025;
		q_tmp.setRPY(1.50, 0, 0);
		modscene_poses_[0].orientation.w = q_tmp.getW();
		modscene_poses_[0].orientation.x = q_tmp.getX();
		modscene_poses_[0].orientation.y = q_tmp.getY();
		modscene_poses_[0].orientation.z = q_tmp.getZ();

		// green
		modscene_poses_[1].position.x = -0.5;
		modscene_poses_[1].position.y = -0.3;
		modscene_poses_[1].position.z = 0.025;
		q_tmp.setRPY(-1.500, 0, 0);
		modscene_poses_[1].orientation.w = q_tmp.getW();
		modscene_poses_[1].orientation.x = q_tmp.getX();
		modscene_poses_[1].orientation.y = q_tmp.getY();
		modscene_poses_[1].orientation.z = q_tmp.getZ();

		//blue
		modscene_poses_[2].position.x = -0.2;
		modscene_poses_[2].position.y = -0.3;
		modscene_poses_[2].position.z = 0.025;
		q_tmp.setRPY(1.50, 0, 0.0);
		modscene_poses_[2].orientation.w = q_tmp.getW();
		modscene_poses_[2].orientation.x = q_tmp.getX();
		modscene_poses_[2].orientation.y = q_tmp.getY();
		modscene_poses_[2].orientation.z = q_tmp.getZ();

		//cyan
		modscene_poses_[3].position.x = 0.2;
		modscene_poses_[3].position.y = -0.3;
		modscene_poses_[3].position.z = 0.025;
		q_tmp.setRPY(-1.50, 0, 0);
		modscene_poses_[3].orientation.w = q_tmp.getW();
		modscene_poses_[3].orientation.x = q_tmp.getX();
		modscene_poses_[3].orientation.y = q_tmp.getY();
		modscene_poses_[3].orientation.z = q_tmp.getZ();

		//magenta
		modscene_poses_[4].position.x = 0.5;
		modscene_poses_[4].position.y = -0.3;
		modscene_poses_[4].position.z = 0.025;
		q_tmp.setRPY(1.5, 0, 0);
		modscene_poses_[4].orientation.w = q_tmp.getW();
		modscene_poses_[4].orientation.x = q_tmp.getX();
		modscene_poses_[4].orientation.y = q_tmp.getY();
		modscene_poses_[4].orientation.z = q_tmp.getZ();

		//yellow
		modscene_poses_[5].position.x = 0.7;
		modscene_poses_[5].position.y = -0.3;
		modscene_poses_[5].position.z = 0.025;
		q_tmp.setRPY(0,0,0);
		modscene_poses_[5].orientation.w = q_tmp.getW();
		modscene_poses_[5].orientation.x = q_tmp.getX();
		modscene_poses_[5].orientation.y = q_tmp.getY();
		modscene_poses_[5].orientation.z = q_tmp.getZ();
		break;

	case 15:
		modscene_poses_.resize(6);

		//    red_handle:
		//      start_pose: [0.4, 0, 0.03, 1.50, 0, 1.5]
		//    green_handle:
		//      start_pose: [0, 0.3, 0.03, -1.500, 0, 0]
		//    blue_handle:
		//      start_pose: [0, -0.30, 0.03, 1.50, 0, 0]
		//    cyan_handle:
		//      start_pose: [-0.6, -0.6, 0.03, -1.50, 0, -0.75]
		//    magenta_handle:
		//      start_pose: [-0.7, 0, 0.03, 1.50, 0, 1.5]
		//    yellow_handle:
		//      start_pose: [-0.6, 0.6, 0.03, 1.5, 0, 0.75]

		//red
		modscene_poses_[0].position.x = 0.4;
		modscene_poses_[0].position.y = 0;
		modscene_poses_[0].position.z = 0.025;
		q_tmp.setRPY(1.50, 0, 1.5);
		modscene_poses_[0].orientation.w = q_tmp.getW();
		modscene_poses_[0].orientation.x = q_tmp.getX();
		modscene_poses_[0].orientation.y = q_tmp.getY();
		modscene_poses_[0].orientation.z = q_tmp.getZ();

		// green
		modscene_poses_[1].position.x = 0;
		modscene_poses_[1].position.y = 0.3;
		modscene_poses_[1].position.z = 0.025;
		q_tmp.setRPY(-1.500, 0, 0);
		modscene_poses_[1].orientation.w = q_tmp.getW();
		modscene_poses_[1].orientation.x = q_tmp.getX();
		modscene_poses_[1].orientation.y = q_tmp.getY();
		modscene_poses_[1].orientation.z = q_tmp.getZ();

		//blue
		modscene_poses_[2].position.x = 0;
		modscene_poses_[2].position.y = -0.3;
		modscene_poses_[2].position.z = 0.025;
		q_tmp.setRPY(1.50, 0, 0.0);
		modscene_poses_[2].orientation.w = q_tmp.getW();
		modscene_poses_[2].orientation.x = q_tmp.getX();
		modscene_poses_[2].orientation.y = q_tmp.getY();
		modscene_poses_[2].orientation.z = q_tmp.getZ();

		//cyan
		modscene_poses_[3].position.x = -0.6;
		modscene_poses_[3].position.y = -0.6;
		modscene_poses_[3].position.z = 0.025;
		q_tmp.setRPY(-1.50, 0, -0.75);
		modscene_poses_[3].orientation.w = q_tmp.getW();
		modscene_poses_[3].orientation.x = q_tmp.getX();
		modscene_poses_[3].orientation.y = q_tmp.getY();
		modscene_poses_[3].orientation.z = q_tmp.getZ();

		//magenta
		modscene_poses_[4].position.x = -0.7;
		modscene_poses_[4].position.y = 0;
		modscene_poses_[4].position.z = 0.025;
		q_tmp.setRPY(1.50, 0, 1.5);
		modscene_poses_[4].orientation.w = q_tmp.getW();
		modscene_poses_[4].orientation.x = q_tmp.getX();
		modscene_poses_[4].orientation.y = q_tmp.getY();
		modscene_poses_[4].orientation.z = q_tmp.getZ();

		//yellow
		modscene_poses_[5].position.x = -0.6;
		modscene_poses_[5].position.y = 0.6;
		modscene_poses_[5].position.z = 0.025;
		q_tmp.setRPY(1.5, 0, 0.75);
		modscene_poses_[5].orientation.w = q_tmp.getW();
		modscene_poses_[5].orientation.x = q_tmp.getX();
		modscene_poses_[5].orientation.y = q_tmp.getY();
		modscene_poses_[5].orientation.z = q_tmp.getZ();
		break;
	case 16:
		modscene_poses_.resize(6);

		//	    red_cube:
		//	      start_pose: [-0.3, -0.4, 0.03, 0, 0, 0]
		//	    green_cube:
		//	      start_pose: [-0.5, 0.1, 0.051, -3.1415, 0, 0.8]
		//	    blue_cube:
		//	      start_pose: [0, 0.5, 0.03, -1.5, 0, 0.4]
		//	    cyan_cube:
		//	      start_pose: [-0.3, 0.4, 0.1, 0, 0, 0]
		//	    magenta_cube:
		//	      start_pose: [-0.5, -0.2, 0.051, -3.1415, 0, 0.8]
		//	    yellow_cube:
		//	      start_pose: [0, -0.6, 0, 0, 0, 0.3]

		//red
		modscene_poses_[0].position.x = -0.3;
		modscene_poses_[0].position.y = -0.4;
		modscene_poses_[0].position.z = 0.025;
		q_tmp.setRPY(0, 0, 0);
		modscene_poses_[0].orientation.w = q_tmp.getW();
		modscene_poses_[0].orientation.x = q_tmp.getX();
		modscene_poses_[0].orientation.y = q_tmp.getY();
		modscene_poses_[0].orientation.z = q_tmp.getZ();

		// green
		modscene_poses_[1].position.x = -0.50;
		modscene_poses_[1].position.y = 0.1;
		modscene_poses_[1].position.z = 0.05;
		q_tmp.setRPY(-3.1415, 0, 0.8);
		modscene_poses_[1].orientation.w = q_tmp.getW();
		modscene_poses_[1].orientation.x = q_tmp.getX();
		modscene_poses_[1].orientation.y = q_tmp.getY();
		modscene_poses_[1].orientation.z = q_tmp.getZ();

		//blue
		modscene_poses_[2].position.x = 0;
		modscene_poses_[2].position.y = 0.5;
		modscene_poses_[2].position.z = 0.025;
		q_tmp.setRPY(-1.5, 0, 0.4);
		modscene_poses_[2].orientation.w = q_tmp.getW();
		modscene_poses_[2].orientation.x = q_tmp.getX();
		modscene_poses_[2].orientation.y = q_tmp.getY();
		modscene_poses_[2].orientation.z = q_tmp.getZ();

		//cyan
		modscene_poses_[3].position.x = -0.3;
		modscene_poses_[3].position.y = 0.4;
		modscene_poses_[3].position.z = 0.015;
		q_tmp.setRPY(0,0,0);
		modscene_poses_[3].orientation.w = q_tmp.getW();
		modscene_poses_[3].orientation.x = q_tmp.getX();
		modscene_poses_[3].orientation.y = q_tmp.getY();
		modscene_poses_[3].orientation.z = q_tmp.getZ();

		//magenta
		modscene_poses_[4].position.x = -0.5;
		modscene_poses_[4].position.y = -0.2;
		modscene_poses_[4].position.z = 0.05;
		q_tmp.setRPY(-3.1415, 0, 0.8);
		modscene_poses_[4].orientation.w = q_tmp.getW();
		modscene_poses_[4].orientation.x = q_tmp.getX();
		modscene_poses_[4].orientation.y = q_tmp.getY();
		modscene_poses_[4].orientation.z = q_tmp.getZ();

		//yellow
		modscene_poses_[5].position.x = 0;
		modscene_poses_[5].position.y = -0.6;
		modscene_poses_[5].position.z = 0.025;
		q_tmp.setRPY(0, 0, 0.3);
		modscene_poses_[5].orientation.w = q_tmp.getW();
		modscene_poses_[5].orientation.x = q_tmp.getX();
		modscene_poses_[5].orientation.y = q_tmp.getY();
		modscene_poses_[5].orientation.z = q_tmp.getZ();
		break;

	case 21:
		modscene_poses_.resize(3);
		//	    red_cube:
		//	      start_pose: [-0.275, -0.1925, 0.04, 0.4, 0.1, 0]
		//	    green_cylinder:
		//	      start_pose: [-0.1992, -0.4402, 0.051, 0.0, 0, 1.8]
		//	    blue_handle:
		//	      start_pose: [-0.4772, 0.4551, 0.04, -1.5, 0, 0.4]
		//red
		modscene_poses_[0].position.x = -0.275;
		modscene_poses_[0].position.y = -0.1925;
		modscene_poses_[0].position.z = 0.025;
		q_tmp.setRPY(0.4, 0.1, 0);
		modscene_poses_[0].orientation.w = q_tmp.getW();
		modscene_poses_[0].orientation.x = q_tmp.getX();
		modscene_poses_[0].orientation.y = q_tmp.getY();
		modscene_poses_[0].orientation.z = q_tmp.getZ();

		// green
		modscene_poses_[1].position.x = -0.1992;
		modscene_poses_[1].position.y = -0.4402;
		modscene_poses_[1].position.z = 0.051;
		q_tmp.setRPY(0,0,1.8);
		modscene_poses_[1].orientation.w = q_tmp.getW();
		modscene_poses_[1].orientation.x = q_tmp.getX();
		modscene_poses_[1].orientation.y = q_tmp.getY();
		modscene_poses_[1].orientation.z = q_tmp.getZ();

		//blue
		modscene_poses_[2].position.x = -0.4772;
		modscene_poses_[2].position.y = 0.4551;
		modscene_poses_[2].position.z = 0.025;
		q_tmp.setRPY(-1.5, 0, 0.4);
		modscene_poses_[2].orientation.w = q_tmp.getW();
		modscene_poses_[2].orientation.x = q_tmp.getX();
		modscene_poses_[2].orientation.y = q_tmp.getY();
		modscene_poses_[2].orientation.z = q_tmp.getZ();
		break;
	case 22:
		modscene_poses_.resize(3);
		//	    red_cube:
		//	      start_pose: [-0.0597, -0.4767, 0.05, 0.8, 0.1, 0]
		//	    green_cylinder:
		//	      start_pose: [0.5339,  0.2682, 0.051, 0.0, 0, 0.55]
		//	    blue_handle:
		//	      start_pose: [ 0.8419, -0.1394, 0.04, -1.5, 0, 0.4]
		//red
		modscene_poses_[0].position.x = -0.0597;
		modscene_poses_[0].position.y = -0.4767;
		modscene_poses_[0].position.z = 0.025;
		q_tmp.setRPY(0.8, 0.1, 0);
		modscene_poses_[0].orientation.w = q_tmp.getW();
		modscene_poses_[0].orientation.x = q_tmp.getX();
		modscene_poses_[0].orientation.y = q_tmp.getY();
		modscene_poses_[0].orientation.z = q_tmp.getZ();

		// green
		modscene_poses_[1].position.x = 0.5339;
		modscene_poses_[1].position.y = 0.2682;
		modscene_poses_[1].position.z = 0.05;
		q_tmp.setRPY(0,0,0.55);
		modscene_poses_[1].orientation.w = q_tmp.getW();
		modscene_poses_[1].orientation.x = q_tmp.getX();
		modscene_poses_[1].orientation.y = q_tmp.getY();
		modscene_poses_[1].orientation.z = q_tmp.getZ();

		//blue
		modscene_poses_[2].position.x = 0.8419;
		modscene_poses_[2].position.y = -0.1394;
		modscene_poses_[2].position.z = 0.025;
		q_tmp.setRPY(-1.5, 0, 0.4);
		modscene_poses_[2].orientation.w = q_tmp.getW();
		modscene_poses_[2].orientation.x = q_tmp.getX();
		modscene_poses_[2].orientation.y = q_tmp.getY();
		modscene_poses_[2].orientation.z = q_tmp.getZ();
		break;
	case 23:
		modscene_poses_.resize(3);
		//	    red_cube:
		//	      start_pose: [-0.547, -0.3739, 0.05, 0.7, 0.05, 0.1]
		//	    green_cylinder:
		//	      start_pose: [-0.0325,  -0.7759, 0.051, 0.0, 0, 0.15]
		//	    blue_handle:
		//	      start_pose: [ 0.2976, -0.5322, 0.04, -0.5, 0.2, 0.4]
		//red
		modscene_poses_[0].position.x = -0.547;
		modscene_poses_[0].position.y = -0.3739;
		modscene_poses_[0].position.z = 0.025;
		q_tmp.setRPY(0.7, 0.05, 0.1);
		modscene_poses_[0].orientation.w = q_tmp.getW();
		modscene_poses_[0].orientation.x = q_tmp.getX();
		modscene_poses_[0].orientation.y = q_tmp.getY();
		modscene_poses_[0].orientation.z = q_tmp.getZ();

		// green
		modscene_poses_[1].position.x = -0.0325;
		modscene_poses_[1].position.y = -0.7759;
		modscene_poses_[1].position.z = 0.05;
		q_tmp.setRPY(0,0,0.15);
		modscene_poses_[1].orientation.w = q_tmp.getW();
		modscene_poses_[1].orientation.x = q_tmp.getX();
		modscene_poses_[1].orientation.y = q_tmp.getY();
		modscene_poses_[1].orientation.z = q_tmp.getZ();

		//blue
		modscene_poses_[2].position.x = 0.29760;
		modscene_poses_[2].position.y = -0.5322;
		modscene_poses_[2].position.z = 0.025;
		q_tmp.setRPY(-0.5, 0.2, 0.4);
		modscene_poses_[2].orientation.w = q_tmp.getW();
		modscene_poses_[2].orientation.x = q_tmp.getX();
		modscene_poses_[2].orientation.y = q_tmp.getY();
		modscene_poses_[2].orientation.z = q_tmp.getZ();
		break;
	}
}
