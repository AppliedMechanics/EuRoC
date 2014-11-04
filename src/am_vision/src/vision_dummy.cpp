/*
 * VisionDummy.cpp
 *
 *  Created on: Sep 12, 2014
 *      Author: euroc_admin
 */

#include "vision_dummy.hpp"

VisionDummy::VisionDummy() {
	// TODO Auto-generated constructor stub
	mod_scene_ = 11;
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

	int obj_idx=-1;
	switch(goal->mode)
	{
	case GLOBAL_POSE_ESTIMATION:

		for(uint16_t ii=0;ii<modscene_colors_.size();ii++)
		{
			if(!modscene_colors_[ii].compare(goal->object.color))
			{
				obj_idx = ii;
				break;
			}
		}
		if(obj_idx==-1)
		{
			msg_error("error in vision dummy!!");
			failed=true;
		}
		else
		{
			vision_result_.abs_object_pose = modscene_poses_[obj_idx];
			failed=false;
		}
		break;

	case CLOSE_RANGE_POSE_ESTIMATION:
		failed=true;
		break;

	case CHECKING_FOR_OBJECT_IN_TARGET_ZONE:
		failed=false;
//	    static bool first=true;
//	    if(first)
//	    {
//	    	first=false;
//	    	failed=true;
//	    }
	    break;
	}

	// ==========================================================================================
	//-------------------------------- SEND RESULT ----------------------------------------//

	if(failed)
	{
		vision_result_.object_detected=false;
		vision_result_.object_in_zone=false;
		vision_server_.setPreempted(vision_result_,"Object not detected.");
	}
	else
	{
		vision_result_.object_detected=true;
		vision_result_.object_in_zone=true;
		vision_server_.setSucceeded(vision_result_, "Object detected");
	}
	return;
}


void VisionDummy::loadScene(int scene)
{
	tf::Quaternion q_tmp;
	switch (scene)
	{
	case 11:
		modscene_poses_.resize(3);
		modscene_colors_.resize(3);
		//	    red_cube:
		//	      start_pose: [-0.3, -0.4, 0.03, 0, 0, 0]
		//	    green_cylinder:
		//	      start_pose: [-0.5, 0.1, 0.051, -3.1415, 0, 0.8]
		//	    blue_handle:
		//	      start_pose: [0, 0.5, 0.03, -1.5, 0, 0.4]

		modscene_colors_[0]="ff0000";
		modscene_colors_[1]="00ff00";
		modscene_colors_[2]="0000ff";

		//red
		modscene_poses_[0].position.x = -0.3;
		modscene_poses_[0].position.y = -0.4;
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
		q_tmp.setRPY(-1.5708, 0, 0.4);
		modscene_poses_[2].orientation.w = q_tmp.getW();
		modscene_poses_[2].orientation.x = q_tmp.getX();
		modscene_poses_[2].orientation.y = q_tmp.getY();
		modscene_poses_[2].orientation.z = q_tmp.getZ();
		break;
	case 12:
		modscene_poses_.resize(3);
		modscene_colors_.resize(3);


		modscene_colors_[0]="ff0000";
		modscene_colors_[1]="00ffff";
		modscene_colors_[2]="ffff00";
		// red_cube:
		// start_pose: [0.3, -0.3, 0.015, 0, 0, 0]
		// cyan_cylinder:
		// start_pose: [0.3, 0.1, 0.061, -3.1415, 0, 0.8]
		// yellow_handle:
		// start_pose: [-0.3, 0.3, 0.03, -1.5, 0.0, 1.2]

		//red
		modscene_poses_[0].position.x = 0.3;
		modscene_poses_[0].position.y = -0.3;
		modscene_poses_[0].position.z = 0.0125;
		q_tmp.setRPY(0,0,0);
		modscene_poses_[0].orientation.w = q_tmp.getW();
		modscene_poses_[0].orientation.x = q_tmp.getX();
		modscene_poses_[0].orientation.y = q_tmp.getY();
		modscene_poses_[0].orientation.z = q_tmp.getZ();

		// cyan
		modscene_poses_[1].position.x = 0.3;
		modscene_poses_[1].position.y = 0.1;
		modscene_poses_[1].position.z = 0.06;
		q_tmp.setRPY(-3.1415, 0, 0.8);
		modscene_poses_[1].orientation.w = q_tmp.getW();
		modscene_poses_[1].orientation.x = q_tmp.getX();
		modscene_poses_[1].orientation.y = q_tmp.getY();
		modscene_poses_[1].orientation.z = q_tmp.getZ();

		// yellow
		modscene_poses_[2].position.x = -0.3;
		modscene_poses_[2].position.y = 0.3;
		modscene_poses_[2].position.z = 0.025;
		q_tmp.setRPY(-1.5708, 0, 1.2);
		modscene_poses_[2].orientation.w = q_tmp.getW();
		modscene_poses_[2].orientation.x = q_tmp.getX();
		modscene_poses_[2].orientation.y = q_tmp.getY();
		modscene_poses_[2].orientation.z = q_tmp.getZ();
		break;
	//case 13:
	case 14:
		modscene_poses_.resize(6);
		modscene_colors_.resize(6);
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


		modscene_colors_[0]="ff0000"; //red
		modscene_colors_[1]="00ff00"; //green
		modscene_colors_[2]="0000ff"; //blue
		modscene_colors_[3]="00ffff"; //cyan
		modscene_colors_[4]="ff00ff"; //magenta
		modscene_colors_[5]="ffff00"; //yellow

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
		modscene_colors_.resize(6);

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

		modscene_colors_[0]="ff0000"; //red
		modscene_colors_[1]="00ff00"; //green
		modscene_colors_[2]="0000ff"; //blue
		modscene_colors_[3]="00ffff"; //cyan
		modscene_colors_[4]="ff00ff"; //magenta
		modscene_colors_[5]="ffff00"; //yellow

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
		modscene_colors_.resize(6);

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

		modscene_colors_[0]="ff0000"; //red
		modscene_colors_[1]="00ff00"; //green
		modscene_colors_[2]="0000ff"; //blue
		modscene_colors_[3]="00ffff"; //cyan
		modscene_colors_[4]="ff00ff"; //magenta
		modscene_colors_[5]="ffff00"; //yellow

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

	case 211:
		modscene_poses_.resize(3);
		modscene_colors_.resize(3);
		//	    red_cube:
		//	      start_pose: [-0.275, -0.1925, 0.04, 0.4, 0.1, 0]
		//	    green_cylinder:
		//	      start_pose: [-0.1992, -0.4402, 0.051, 0.0, 0, 1.8]
		//	    blue_handle:
		//	      start_pose: [-0.4772, 0.4551, 0.04, -1.5, 0, 0.4]


		modscene_colors_[0]="ff0000"; //red
		modscene_colors_[1]="00ff00"; //green
		modscene_colors_[2]="0000ff"; //blue

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
		q_tmp.setRPY(-1.5708, 0, 0.4);
		modscene_poses_[2].orientation.w = q_tmp.getW();
		modscene_poses_[2].orientation.x = q_tmp.getX();
		modscene_poses_[2].orientation.y = q_tmp.getY();
		modscene_poses_[2].orientation.z = q_tmp.getZ();
		break;
	case 212:
		modscene_poses_.resize(3);
		modscene_colors_.resize(3);
		//	    red_cube:
		//	      start_pose: [-0.0597, -0.4767, 0.05, 0.8, 0.1, 0]
		//	    green_cylinder:
		//	      start_pose: [0.5339,  0.2682, 0.051, 0.0, 0, 0.55]
		//	    blue_handle:
		//	      start_pose: [ 0.8419, -0.1394, 0.04, -1.5, 0, 0.4]

		modscene_colors_[0]="ff0000"; //red
		modscene_colors_[1]="00ff00"; //green
		modscene_colors_[2]="0000ff"; //blue

		//red
		modscene_poses_[0].position.x = -0.0597;
		modscene_poses_[0].position.y = -0.5;//-0.4767;
		modscene_poses_[0].position.z = 0.025;
		q_tmp.setRPY(0.0, 0.1, 0); //(0.8, 0.1, 0);
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
		q_tmp.setRPY(-1.5708, 0, 0.4);
		modscene_poses_[2].orientation.w = q_tmp.getW();
		modscene_poses_[2].orientation.x = q_tmp.getX();
		modscene_poses_[2].orientation.y = q_tmp.getY();
		modscene_poses_[2].orientation.z = q_tmp.getZ();
		break;

	case 213:
		modscene_poses_.resize(3);
		modscene_colors_.resize(3);
		//	    red_cube:
		//	      start_pose: [-0.547, -0.3739, 0.05, 0.7, 0.05, 0.1]
		//	    green_cylinder:
		//	      start_pose: [-0.0325,  -0.7759, 0.051, 0.0, 0, 0.15]
		//	    blue_handle:
		//	      start_pose: [ 0.2976, -0.5322, 0.04, -0.5, 0.2, 0.4]

		modscene_colors_[0]="ff0000"; //red
		modscene_colors_[1]="00ff00"; //green
		modscene_colors_[2]="0000ff"; //blue

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

	case 221:
		modscene_poses_.resize(3);
		modscene_colors_.resize(3);

	    // magenta_cube:
	    //  start_pose: [0.2781, 0.3085, 0.04, 0.4, 0.1, 0]
	    // green_cylinder:
		//  start_pose: [0.4792, -0.2541, 0.026, 0.0, 0, 1.8]
		// red_handle:
		//   start_pose: [0.3148, -0.4551, 0.044, -1.5, 0, 0.1897]

		modscene_colors_[0]="ff00ff"; //magenta
		modscene_colors_[1]="00ff00"; //green
		modscene_colors_[2]="ff0000"; //red

		//magenta
		modscene_poses_[0].position.x = 0.2781;
		modscene_poses_[0].position.y = 0.3085;
		modscene_poses_[0].position.z = 0.025;
		q_tmp.setRPY(0.4, 0.1, 0);
		modscene_poses_[0].orientation.w = q_tmp.getW();
		modscene_poses_[0].orientation.x = q_tmp.getX();
		modscene_poses_[0].orientation.y = q_tmp.getY();
		modscene_poses_[0].orientation.z = q_tmp.getZ();

		// green
		modscene_poses_[1].position.x = 0.4792;
		modscene_poses_[1].position.y = -0.2541;
		modscene_poses_[1].position.z = 0.025;
		q_tmp.setRPY(0,0,1.8);
		modscene_poses_[1].orientation.w = q_tmp.getW();
		modscene_poses_[1].orientation.x = q_tmp.getX();
		modscene_poses_[1].orientation.y = q_tmp.getY();
		modscene_poses_[1].orientation.z = q_tmp.getZ();

		//red
		modscene_poses_[2].position.x = 0.3148;
		modscene_poses_[2].position.y = -0.4551;
		modscene_poses_[2].position.z = 0.0275;
		q_tmp.setRPY(-1.5708, 0, 0.1897);
		modscene_poses_[2].orientation.w = q_tmp.getW();
		modscene_poses_[2].orientation.x = q_tmp.getX();
		modscene_poses_[2].orientation.y = q_tmp.getY();
		modscene_poses_[2].orientation.z = q_tmp.getZ();
		break;

	case 222:
		modscene_poses_.resize(3);
		modscene_colors_.resize(3);

	    // magenta_cube:
		//   start_pose: [0.2781, 0.3085, 0.04, 0.4, 0.1, 0]
		// green_cylinder:
		//   start_pose: [0.4792, -0.2541, 0.026, 0.0, 0, 1.8]
		// red_handle:
		//  start_pose: [-0.2480, 0.3451, 0.044, -1.5, 0, 0.7897]

		modscene_colors_[0]="ff00ff"; //magenta
		modscene_colors_[1]="00ff00"; //green
		modscene_colors_[2]="ff0000"; //red

		//magenta
		modscene_poses_[0].position.x = 0.2781;
		modscene_poses_[0].position.y = 0.3085;
		modscene_poses_[0].position.z = 0.025;
		q_tmp.setRPY(0.4, 0.1, 0);
		modscene_poses_[0].orientation.w = q_tmp.getW();
		modscene_poses_[0].orientation.x = q_tmp.getX();
		modscene_poses_[0].orientation.y = q_tmp.getY();
		modscene_poses_[0].orientation.z = q_tmp.getZ();

		// green
		modscene_poses_[1].position.x = 0.4792;
		modscene_poses_[1].position.y = -0.2541;
		modscene_poses_[1].position.z = 0.025;
		q_tmp.setRPY(0,0,1.8);
		modscene_poses_[1].orientation.w = q_tmp.getW();
		modscene_poses_[1].orientation.x = q_tmp.getX();
		modscene_poses_[1].orientation.y = q_tmp.getY();
		modscene_poses_[1].orientation.z = q_tmp.getZ();

		//red
		modscene_poses_[2].position.x = -0.2480;
		modscene_poses_[2].position.y = 0.3451;
		modscene_poses_[2].position.z = 0.0275;
		q_tmp.setRPY(-1.5708, 0, 0.7897);
		modscene_poses_[2].orientation.w = q_tmp.getW();
		modscene_poses_[2].orientation.x = q_tmp.getX();
		modscene_poses_[2].orientation.y = q_tmp.getY();
		modscene_poses_[2].orientation.z = q_tmp.getZ();
		break;

	case 223:
		modscene_poses_.resize(3);
		modscene_colors_.resize(3);

	    // magenta_cube:
		//   start_pose: [-0.3536, 0.3536, 0.04, 0.4, 0.1, 0]
		// green_cylinder:
		//   start_pose: [-0.521, 0.0232, 0.026, 0.0, 0, 1.8]
		// red_handle:
		//   start_pose: [0.5031, 0.0572, 0.044, -1.5, 0, 2.1536]

		modscene_colors_[0]="ff00ff"; //magenta
		modscene_colors_[1]="00ff00"; //green
		modscene_colors_[2]="ff0000"; //red

		//magenta
		modscene_poses_[0].position.x = -0.3536;
		modscene_poses_[0].position.y = 0.3536;
		modscene_poses_[0].position.z = 0.025;
		q_tmp.setRPY(0.4, 0.1, 0);
		modscene_poses_[0].orientation.w = q_tmp.getW();
		modscene_poses_[0].orientation.x = q_tmp.getX();
		modscene_poses_[0].orientation.y = q_tmp.getY();
		modscene_poses_[0].orientation.z = q_tmp.getZ();

		// green
		modscene_poses_[1].position.x = -0.521;
		modscene_poses_[1].position.y = 0.0232;
		modscene_poses_[1].position.z = 0.025;
		q_tmp.setRPY(0,0,1.8);
		modscene_poses_[1].orientation.w = q_tmp.getW();
		modscene_poses_[1].orientation.x = q_tmp.getX();
		modscene_poses_[1].orientation.y = q_tmp.getY();
		modscene_poses_[1].orientation.z = q_tmp.getZ();

		//red
		modscene_poses_[2].position.x = 0.5031;
		modscene_poses_[2].position.y = 0.0572;
		modscene_poses_[2].position.z = 0.0275;
		q_tmp.setRPY(-1.5708, 0, 2.1536);
		modscene_poses_[2].orientation.w = q_tmp.getW();
		modscene_poses_[2].orientation.x = q_tmp.getX();
		modscene_poses_[2].orientation.y = q_tmp.getY();
		modscene_poses_[2].orientation.z = q_tmp.getZ();
		break;

	case 31:
		modscene_poses_.resize(3);
		modscene_colors_.resize(3);

		modscene_colors_[0]="ff0000"; //red
		modscene_colors_[1]="00ff00"; //green
		modscene_colors_[2]="0000ff"; //blue

	    //red_cube:
	    modscene_poses_[0].position.x = 0.1858;
		modscene_poses_[0].position.y = -0.772;
		modscene_poses_[0].position.z = 0.025;
		q_tmp.setRPY(0.4, 0.1, 0);
		modscene_poses_[0].orientation.w = q_tmp.getW();
		modscene_poses_[0].orientation.x = q_tmp.getX();
		modscene_poses_[0].orientation.y = q_tmp.getY();
		modscene_poses_[0].orientation.z = q_tmp.getZ();

	    //green_cylinder:
	    modscene_poses_[1].position.x = 0.7648;
		modscene_poses_[1].position.y = 0.1677;
		modscene_poses_[1].position.z = 0.05;
		q_tmp.setRPY(0.0, 0, 1.8);
		modscene_poses_[1].orientation.w = q_tmp.getW();
		modscene_poses_[1].orientation.x = q_tmp.getX();
		modscene_poses_[1].orientation.y = q_tmp.getY();
		modscene_poses_[1].orientation.z = q_tmp.getZ();

	    //blue_handle:
		modscene_poses_[2].position.x = -0.8231;
		modscene_poses_[2].position.y = 0.4563;
		modscene_poses_[2].position.z = 0.025;
		q_tmp.setRPY(-1.5708, 0, 0.4);
		modscene_poses_[2].orientation.w = q_tmp.getW();
		modscene_poses_[2].orientation.x = q_tmp.getX();
		modscene_poses_[2].orientation.y = q_tmp.getY();
		modscene_poses_[2].orientation.z = q_tmp.getZ();
		break;

	case 32:
		modscene_poses_.resize(3);
		modscene_colors_.resize(3);

		// red_cube:
		//   start_pose: [-0.1251, -0.7532, 0.02, 0.4, 0.1, 0]
		// green_cylinder:
		//   start_pose: [-0.8231, 0.4563, 0.051, 0.0, 0, 1.8]
		// cyan_handle:
		//   start_pose: [0.7412, -0.1653, 0.04, -1.5, 0, 0.312]

		modscene_colors_[0]="ff0000"; //red
		modscene_colors_[1]="00ff00"; //green
		modscene_colors_[2]="00ffff"; //cyan

		//red_cube:
		modscene_poses_[0].position.x = -0.1251;
		modscene_poses_[0].position.y = -0.7532;
		modscene_poses_[0].position.z = 0.015;
		q_tmp.setRPY(0.4, 0.1, 0);
		modscene_poses_[0].orientation.w = q_tmp.getW();
		modscene_poses_[0].orientation.x = q_tmp.getX();
		modscene_poses_[0].orientation.y = q_tmp.getY();
		modscene_poses_[0].orientation.z = q_tmp.getZ();

		//green_cylinder:
		modscene_poses_[1].position.x = -0.8231;
		modscene_poses_[1].position.y = 0.4563;
		modscene_poses_[1].position.z = 0.05;
		q_tmp.setRPY(0.0, 0, 1.8);
		modscene_poses_[1].orientation.w = q_tmp.getW();
		modscene_poses_[1].orientation.x = q_tmp.getX();
		modscene_poses_[1].orientation.y = q_tmp.getY();
		modscene_poses_[1].orientation.z = q_tmp.getZ();

		//blue_handle:
		modscene_poses_[2].position.x = 0.7412;
		modscene_poses_[2].position.y = -0.1653;
		modscene_poses_[2].position.z = 0.025;
		q_tmp.setRPY(-1.5708, 0, 0.312);
		modscene_poses_[2].orientation.w = q_tmp.getW();
		modscene_poses_[2].orientation.x = q_tmp.getX();
		modscene_poses_[2].orientation.y = q_tmp.getY();
		modscene_poses_[2].orientation.z = q_tmp.getZ();
		break;

	case 411:
				modscene_poses_.resize(3);
				modscene_colors_.resize(3);

				modscene_colors_[0]="ff0000"; //red
				modscene_colors_[1]="00ff00"; //green
				modscene_colors_[2]="00ffff"; //cyan

				//red_cube:
				modscene_poses_[0].position.x = 0.1858;
				modscene_poses_[0].position.y = -0.772;
				modscene_poses_[0].position.z = 0.04;
				q_tmp.setRPY(0.4, 0.1, 0);
				modscene_poses_[0].orientation.w = q_tmp.getW();
				modscene_poses_[0].orientation.x = q_tmp.getX();
				modscene_poses_[0].orientation.y = q_tmp.getY();
				modscene_poses_[0].orientation.z = q_tmp.getZ();

				//green_cylinder:
				modscene_poses_[1].position.x = 0.7648;
				modscene_poses_[1].position.y = 0.1677;
				modscene_poses_[1].position.z = 0.051;
				q_tmp.setRPY(0.0, 0, 1.8);
				modscene_poses_[1].orientation.w = q_tmp.getW();
				modscene_poses_[1].orientation.x = q_tmp.getX();
				modscene_poses_[1].orientation.y = q_tmp.getY();
				modscene_poses_[1].orientation.z = q_tmp.getZ();

				//blue_handle:
				modscene_poses_[2].position.x = -0.8231;
				modscene_poses_[2].position.y = 0.4563;
				modscene_poses_[2].position.z = 0.04;
				q_tmp.setRPY(-1.5, 0, 0.4);
				modscene_poses_[2].orientation.w = q_tmp.getW();
				modscene_poses_[2].orientation.x = q_tmp.getX();
				modscene_poses_[2].orientation.y = q_tmp.getY();
				modscene_poses_[2].orientation.z = q_tmp.getZ();
				break;

	case 412:
				modscene_poses_.resize(3);
				modscene_colors_.resize(3);

				modscene_colors_[0]="ff0000"; //red
				modscene_colors_[1]="00ff00"; //green
				modscene_colors_[2]="00ffff"; //cyan

				//red_cube:
				modscene_poses_[0].position.x = 0.1858;
				modscene_poses_[0].position.y = -0.772;
				modscene_poses_[0].position.z = 0.04;
				q_tmp.setRPY(0.4, 0.1, 0);
				modscene_poses_[0].orientation.w = q_tmp.getW();
				modscene_poses_[0].orientation.x = q_tmp.getX();
				modscene_poses_[0].orientation.y = q_tmp.getY();
				modscene_poses_[0].orientation.z = q_tmp.getZ();

				//green_cylinder:
				modscene_poses_[1].position.x = 0.7648;
				modscene_poses_[1].position.y = 0.1677;
				modscene_poses_[1].position.z = 0.051;
				q_tmp.setRPY(0.0, 0, 1.8);
				modscene_poses_[1].orientation.w = q_tmp.getW();
				modscene_poses_[1].orientation.x = q_tmp.getX();
				modscene_poses_[1].orientation.y = q_tmp.getY();
				modscene_poses_[1].orientation.z = q_tmp.getZ();

				//blue_handle:
				modscene_poses_[2].position.x = -0.8231;
				modscene_poses_[2].position.y = 0.4563;
				modscene_poses_[2].position.z = 0.04;
				q_tmp.setRPY(-1.5, 0, 0.4);
				modscene_poses_[2].orientation.w = q_tmp.getW();
				modscene_poses_[2].orientation.x = q_tmp.getX();
				modscene_poses_[2].orientation.y = q_tmp.getY();
				modscene_poses_[2].orientation.z = q_tmp.getZ();
				break;

	case 413:
				modscene_poses_.resize(3);
				modscene_colors_.resize(3);

				modscene_colors_[0]="ff0000"; //red
				modscene_colors_[1]="00ff00"; //green
				modscene_colors_[2]="00ffff"; //cyan

				//red_cube:
				modscene_poses_[0].position.x = 0.1858;
				modscene_poses_[0].position.y = -0.772;
				modscene_poses_[0].position.z = 0.04;
				q_tmp.setRPY(0.4, 0.1, 0);
				modscene_poses_[0].orientation.w = q_tmp.getW();
				modscene_poses_[0].orientation.x = q_tmp.getX();
				modscene_poses_[0].orientation.y = q_tmp.getY();
				modscene_poses_[0].orientation.z = q_tmp.getZ();

				//green_cylinder:
				modscene_poses_[1].position.x = 0.7648;
				modscene_poses_[1].position.y = 0.1677;
				modscene_poses_[1].position.z = 0.051;
				q_tmp.setRPY(0.0, 0, 1.8);
				modscene_poses_[1].orientation.w = q_tmp.getW();
				modscene_poses_[1].orientation.x = q_tmp.getX();
				modscene_poses_[1].orientation.y = q_tmp.getY();
				modscene_poses_[1].orientation.z = q_tmp.getZ();

				//blue_handle:
				modscene_poses_[2].position.x = -0.8231;
				modscene_poses_[2].position.y = 0.4563;
				modscene_poses_[2].position.z = 0.04;
				q_tmp.setRPY(-1.5, 0, 0.4);
				modscene_poses_[2].orientation.w = q_tmp.getW();
				modscene_poses_[2].orientation.x = q_tmp.getX();
				modscene_poses_[2].orientation.y = q_tmp.getY();
				modscene_poses_[2].orientation.z = q_tmp.getZ();
				break;

	case 421:
				modscene_poses_.resize(3);
				modscene_colors_.resize(3);

				modscene_colors_[0]="ff0000"; //red
				modscene_colors_[1]="00ff00"; //green
				modscene_colors_[2]="00ffff"; //cyan

				//cube:
				modscene_poses_[0].position.x = -0.8171897;
				modscene_poses_[0].position.y = -0.8570212;
				modscene_poses_[0].position.z = 0.04;
				q_tmp.setRPY(-1.5, 0, 0.8177691);
				modscene_poses_[0].orientation.w = q_tmp.getW();
				modscene_poses_[0].orientation.x = q_tmp.getX();
				modscene_poses_[0].orientation.y = q_tmp.getY();
				modscene_poses_[0].orientation.z = q_tmp.getZ();

				//cyan_cylinder:
				modscene_poses_[1].position.x = 0.85;
				modscene_poses_[1].position.y = -0.35;
				modscene_poses_[1].position.z = 0.061;
				q_tmp.setRPY(0.0, 0, 1.8);
				modscene_poses_[1].orientation.w = q_tmp.getW();
				modscene_poses_[1].orientation.x = q_tmp.getX();
				modscene_poses_[1].orientation.y = q_tmp.getY();
				modscene_poses_[1].orientation.z = q_tmp.getZ();

				//blue_handle:
				modscene_poses_[2].position.x = -0.5295025;
				modscene_poses_[2].position.y = 0.9574671;
				modscene_poses_[2].position.z = 0.03;
				q_tmp.setRPY(-1.5707963, 0.0000003, -2.6646021);
				modscene_poses_[2].orientation.w = q_tmp.getW();
				modscene_poses_[2].orientation.x = q_tmp.getX();
				modscene_poses_[2].orientation.y = q_tmp.getY();
				modscene_poses_[2].orientation.z = q_tmp.getZ();
				break;

	case 422:
					modscene_poses_.resize(3);
					modscene_colors_.resize(3);

					modscene_colors_[0]="ff0000"; //red
					modscene_colors_[1]="00ff00"; //green
					modscene_colors_[2]="00ffff"; //cyan

					//cube:
					modscene_poses_[0].position.x = -0.7019217;
					modscene_poses_[0].position.y = 0.3823656;
					modscene_poses_[0].position.z = 0.035;
					q_tmp.setRPY(-1.5707962, 0, 0.4767284);
					modscene_poses_[0].orientation.w = q_tmp.getW();
					modscene_poses_[0].orientation.x = q_tmp.getX();
					modscene_poses_[0].orientation.y = q_tmp.getY();
					modscene_poses_[0].orientation.z = q_tmp.getZ();

					//cyan_cylinder:
					modscene_poses_[1].position.x = 0.3674228;
					modscene_poses_[1].position.y = -0.423946;
					modscene_poses_[1].position.z = 0.0601196;
					q_tmp.setRPY(0.0, 0.0, 0.4688798);
					modscene_poses_[1].orientation.w = q_tmp.getW();
					modscene_poses_[1].orientation.x = q_tmp.getX();
					modscene_poses_[1].orientation.y = q_tmp.getY();
					modscene_poses_[1].orientation.z = q_tmp.getZ();

					//blue_handle:
					modscene_poses_[2].position.x = -0.3652547;
					modscene_poses_[2].position.y = 0.6396476;
					modscene_poses_[2].position.z = 0.03;
					q_tmp.setRPY(-1.5707966, -0.0000006, -1.4266464);
					modscene_poses_[2].orientation.w = q_tmp.getW();
					modscene_poses_[2].orientation.x = q_tmp.getX();
					modscene_poses_[2].orientation.y = q_tmp.getY();
					modscene_poses_[2].orientation.z = q_tmp.getZ();
					break;

	case 423:
						modscene_poses_.resize(3);
						modscene_colors_.resize(3);

						modscene_colors_[0]="ff0000"; //red
						modscene_colors_[1]="00ff00"; //green
						modscene_colors_[2]="00ffff"; //cyan

						//cube:
						modscene_poses_[0].position.x = -0.8036147;
						modscene_poses_[0].position.y = 0.6958399;
						modscene_poses_[0].position.z = 0.03;
						q_tmp.setRPY(1.5707962, 0, 0.4669556);
						modscene_poses_[0].orientation.w = q_tmp.getW();
						modscene_poses_[0].orientation.x = q_tmp.getX();
						modscene_poses_[0].orientation.y = q_tmp.getY();
						modscene_poses_[0].orientation.z = q_tmp.getZ();

						//cyan_cylinder:
						modscene_poses_[1].position.x = -0.6954471;
						modscene_poses_[1].position.y = 0.5402952;
						modscene_poses_[1].position.z = 0.0604867;
						q_tmp.setRPY(0.0, 0.0, 0.1227309);
						modscene_poses_[1].orientation.w = q_tmp.getW();
						modscene_poses_[1].orientation.x = q_tmp.getX();
						modscene_poses_[1].orientation.y = q_tmp.getY();
						modscene_poses_[1].orientation.z = q_tmp.getZ();

						//blue_handle:
						modscene_poses_[2].position.x = -0.5538481;
						modscene_poses_[2].position.y = 0.3278301;
						modscene_poses_[2].position.z = 0.025;
						q_tmp.setRPY(-1.5707963, -0.0000002, -2.210385);
						modscene_poses_[2].orientation.w = q_tmp.getW();
						modscene_poses_[2].orientation.x = q_tmp.getX();
						modscene_poses_[2].orientation.y = q_tmp.getY();
						modscene_poses_[2].orientation.z = q_tmp.getZ();
						break;

	case 51:
	      modscene_poses_.resize(4);
	      modscene_colors_.resize(4);

	      modscene_colors_[0]="ff00ff"; //magenta
	      modscene_colors_[1]="ff0000"; //red
	      modscene_colors_[2]="00ff00"; //green
	      modscene_colors_[3]="0000ff"; //blue

	      //part1
	      modscene_poses_[0].position.x = 0.2076033;
	      modscene_poses_[0].position.y = -0.0033017;
	      modscene_poses_[0].position.z = 0.0245;
	      q_tmp.setRPY(0.639327, 1.570795, 2.502266);
	      modscene_poses_[0].orientation.w = q_tmp.getW();
	      modscene_poses_[0].orientation.x = q_tmp.getX();
	      modscene_poses_[0].orientation.y = q_tmp.getY();
	      modscene_poses_[0].orientation.z = q_tmp.getZ();

	      //part2:
	      modscene_poses_[1].position.x = 0.4590240;
	      modscene_poses_[1].position.y = 0.2621639;
	      modscene_poses_[1].position.z = 0.024500;
	      q_tmp.setRPY(0, 0, -2.706165);
	      modscene_poses_[1].orientation.w = q_tmp.getW();
	      modscene_poses_[1].orientation.x = q_tmp.getX();
	      modscene_poses_[1].orientation.y = q_tmp.getY();
	      modscene_poses_[1].orientation.z = q_tmp.getZ();

	      //part3:
	      modscene_poses_[2].position.x = 0.3978365;
	      modscene_poses_[2].position.y = -0.3982641;
	      modscene_poses_[2].position.z = 0.024500;
	      q_tmp.setRPY(0, 0, 2.981508);
	      modscene_poses_[2].orientation.w = q_tmp.getW();
	      modscene_poses_[2].orientation.x = q_tmp.getX();
	      modscene_poses_[2].orientation.y = q_tmp.getY();
	      modscene_poses_[2].orientation.z = q_tmp.getZ();

	      //part4:
	      modscene_poses_[3].position.x = 0.5478388;
	      modscene_poses_[3].position.y = 0.3982641;
	      modscene_poses_[3].position.z = 0.074500;
	      q_tmp.setRPY(-1.570796, 0, 0.241714);
	      modscene_poses_[3].orientation.w = q_tmp.getW();
	      modscene_poses_[3].orientation.x = q_tmp.getX();
	      modscene_poses_[3].orientation.y = q_tmp.getY();
	      modscene_poses_[3].orientation.z = q_tmp.getZ();
	      break;

	case 52:
	case 53:
		  modscene_poses_.resize(6);
		  modscene_colors_.resize(6);

		  modscene_colors_[0]="ff00ff"; //magenta
		  modscene_colors_[1]="ff0000"; //red
		  modscene_colors_[2]="00ff00"; //green
		  modscene_colors_[3]="0000ff"; //blue
		  modscene_colors_[4]="00ffff"; //cyan
		  modscene_colors_[5]="ffff00"; //yellow

		  //part1
		  modscene_poses_[0].position.x = -0.4233450;
		  modscene_poses_[0].position.y = 0.7485522;
		  modscene_poses_[0].position.z = 0.0245000;
		  q_tmp.setRPY(0, 0, 0.4612303);
		  modscene_poses_[0].orientation.w = q_tmp.getW();
		  modscene_poses_[0].orientation.x = q_tmp.getX();
		  modscene_poses_[0].orientation.y = q_tmp.getY();
		  modscene_poses_[0].orientation.z = q_tmp.getZ();

		  //part2
		  modscene_poses_[1].position.x = -0.8609471;
		  modscene_poses_[1].position.y = -0.3583904;
		  modscene_poses_[1].position.z = 0.0245000;
		  q_tmp.setRPY(0, 0, -3.0296945);
		  modscene_poses_[1].orientation.w = q_tmp.getW();
		  modscene_poses_[1].orientation.x = q_tmp.getX();
		  modscene_poses_[1].orientation.y = q_tmp.getY();
		  modscene_poses_[1].orientation.z = q_tmp.getZ();

		  //part3
		  modscene_poses_[2].position.x = -0.3068968;
		  modscene_poses_[0].position.y = 0.6058005;
		  modscene_poses_[0].position.z = 0.0245000;
		  q_tmp.setRPY(0, 0, -1.6903983);
		  modscene_poses_[2].orientation.w = q_tmp.getW();
		  modscene_poses_[2].orientation.x = q_tmp.getX();
		  modscene_poses_[2].orientation.y = q_tmp.getY();
		  modscene_poses_[2].orientation.z = q_tmp.getZ();

		  //part4
		  modscene_poses_[3].position.x = -0.5580633;
		  modscene_poses_[3].position.y = 0.3948248;
		  modscene_poses_[3].position.z = 0.0245000;
		  q_tmp.setRPY(0, 0, -0.8879643);
		  modscene_poses_[3].orientation.w = q_tmp.getW();
		  modscene_poses_[3].orientation.x = q_tmp.getX();
		  modscene_poses_[3].orientation.y = q_tmp.getY();
		  modscene_poses_[3].orientation.z = q_tmp.getZ();

		  //part5
		  modscene_poses_[4].position.x = -0.7336632;
		  modscene_poses_[4].position.y = 0.7341572;
		  modscene_poses_[4].position.z = 0.0245000;
		  q_tmp.setRPY(0, 0, -0.1913765);
		  modscene_poses_[4].orientation.w = q_tmp.getW();
		  modscene_poses_[4].orientation.x = q_tmp.getX();
		  modscene_poses_[4].orientation.y = q_tmp.getY();
		  modscene_poses_[4].orientation.z = q_tmp.getZ();

		  //part6
		  modscene_poses_[5].position.x = -0.2257772;
		  modscene_poses_[5].position.y = -0.0307631;
		  modscene_poses_[5].position.z = 0.0245000;
		  q_tmp.setRPY(0, 0, 0.5343924);
		  modscene_poses_[5].orientation.w = q_tmp.getW();
		  modscene_poses_[5].orientation.x = q_tmp.getX();
		  modscene_poses_[5].orientation.y = q_tmp.getY();
		  modscene_poses_[5].orientation.z = q_tmp.getZ();
		  break;

	default:
		msg_error("scene not implemented in vision dummy!");
		break;
	}
}
