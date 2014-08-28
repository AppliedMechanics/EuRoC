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

	if(!goal->object.name.compare("red_cube"))
	{
		vision_result_.abs_object_pose.position.x=-0.3;
		vision_result_.abs_object_pose.position.y=-0.39;
		vision_result_.abs_object_pose.position.z=0.03;
		q.setRPY(0,0,0);
	}
	else if(!goal->object.name.compare("green_cylinder"))
	{
		vision_result_.abs_object_pose.position.x=-0.5;
		vision_result_.abs_object_pose.position.y=0.1;
		vision_result_.abs_object_pose.position.z=0.051;
		q.setRPY(-3.1415, 0, 0.8);

	}
	else if(!goal->object.name.compare("blue_handle"))
	{
		/*
		vision_result_.abs_object_pose.position.x=-0.07;
		vision_result_.abs_object_pose.position.y=0.475;
		vision_result_.abs_object_pose.position.z=0.025;
		q.setRPY(-1.5, 0, 0.4);
		*/


		vision_result_.abs_object_pose.position.x=0.0; //-0.07;
		vision_result_.abs_object_pose.position.y=0.5; //0.475;
		vision_result_.abs_object_pose.position.z=0.03; //0.025;
		q.setRPY(-1.5, 0, 0.4);
//		q.setRPY(-3.14/2, 0, 0.4);


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
