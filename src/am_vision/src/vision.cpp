#include <vision.hpp>
#include <tf/transform_broadcaster.h>

Vision::Vision():
vision_server_(nh_, "VisionAction", boost::bind(&Vision::handle, this, _1),false),
vision_action_name_("VisionAction")
{
	//
	vision_server_.start();
	ROS_INFO("vision action server started.");
};


void Vision::handle(const am_msgs::VisionGoal::ConstPtr &goal)
{

	//detection code ...
	tf::Quaternion q;
	int task = 23;

	if(!goal->object.name.compare("red_cube"))
	{
		switch (task){
		case 1:
			// Task1 v1
			vision_result_.abs_object_pose.position.x=-0.3;
			vision_result_.abs_object_pose.position.y=-0.4;
			vision_result_.abs_object_pose.position.z=0.03;
			q.setRPY(0,0,0);
			break;
		case 21:
			// Task2 1
			vision_result_.abs_object_pose.position.x=-0.275;
			vision_result_.abs_object_pose.position.y=-0.1925;
			vision_result_.abs_object_pose.position.z=0.04;
			q.setRPY(0.4,0.1,0);
			break;
		case 22:
			// Task2 1
			vision_result_.abs_object_pose.position.x=-0.0597;
			vision_result_.abs_object_pose.position.y=-0.4767;
			vision_result_.abs_object_pose.position.z=0.05;
			q.setRPY(0.8, 0.1, 0);
			break;
		case 23:
			// Task2 1
			vision_result_.abs_object_pose.position.x=-0.547;
			vision_result_.abs_object_pose.position.y=-0.3739;
			vision_result_.abs_object_pose.position.z=0.05;
			q.setRPY(0.7, 0.05, 0.1);
			break;
		}
	}
	else if(!goal->object.name.compare("green_cylinder"))
	{
		switch (task){
		case 1:
			// Task1 v1
			vision_result_.abs_object_pose.position.x=-0.5;
			vision_result_.abs_object_pose.position.y=0.1;
			vision_result_.abs_object_pose.position.z=0.051;
			q.setRPY(-3.1415, 0, 0.8);
			break;
		case 21:
			// Task1 v1
			vision_result_.abs_object_pose.position.x=-0.1992;
			vision_result_.abs_object_pose.position.y=-0.4402;
			vision_result_.abs_object_pose.position.z=0.051;
			q.setRPY(0, 0, 1.8);
			break;
		case 22:
			vision_result_.abs_object_pose.position.x=0.5339;
			vision_result_.abs_object_pose.position.y=0.2682;
			vision_result_.abs_object_pose.position.z=0.051;
			q.setRPY(0.0, 0, 0.55);
			break;
		case 23:
			// Task2 1
			vision_result_.abs_object_pose.position.x=-0.0325;
			vision_result_.abs_object_pose.position.y=-0.7759;
			vision_result_.abs_object_pose.position.z= 0.051;
			q.setRPY(0.0, 0, 0.15);
			break;
		}
	}
	else if(!goal->object.name.compare("blue_handle"))
	{
		switch (task){
		case 1:
			// Task1 v1
			vision_result_.abs_object_pose.position.x=0.0; //-0.07;
			vision_result_.abs_object_pose.position.y=0.5; //0.475;
			vision_result_.abs_object_pose.position.z=0.03; //0.025;
			q.setRPY(-1.5, 0, 0.4);
			break;
		case 21:
			vision_result_.abs_object_pose.position.x=-0.4772; //-0.07;
			vision_result_.abs_object_pose.position.y=0.4551; //0.475;
			vision_result_.abs_object_pose.position.z=0.04; //0.025;
			q.setRPY(-1.5, 0, 0.4);
			break;
		case 22:
			vision_result_.abs_object_pose.position.x=0.8419; //-0.07;
			vision_result_.abs_object_pose.position.y=-0.1394; //0.475;
			vision_result_.abs_object_pose.position.z=0.04; //0.025;
			q.setRPY(-1.5, 0, 0.4);
			break;
		case 23:
			// Task2 1
			vision_result_.abs_object_pose.position.x=0.2976;
			vision_result_.abs_object_pose.position.y=-0.5322;
			vision_result_.abs_object_pose.position.z=0.04;
			q.setRPY(-0.5, 0.2, 0.4);
			break;
		}
	}
	//do something...s

	vision_result_.abs_object_pose.orientation.w = q.getW();
	vision_result_.abs_object_pose.orientation.x = q.getX();
	vision_result_.abs_object_pose.orientation.y = q.getY();
	vision_result_.abs_object_pose.orientation.z = q.getZ();

	vision_feedback_.execution_time = ros::Time::now().sec;
	vision_server_.publishFeedback(vision_feedback_);

	bool failed=false;
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
};
