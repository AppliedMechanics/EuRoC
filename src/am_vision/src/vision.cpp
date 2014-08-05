#include <vision.hpp>


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


	//hier dann vision aufrufen
	vision_result_.abs_object_pose.position.x=-0.3;
	vision_result_.abs_object_pose.position.y=-0.4;
	vision_result_.abs_object_pose.position.z=0.24;
	//do something...

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
