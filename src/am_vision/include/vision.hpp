#ifndef __VISION_HPP__
#define __VISION_HPP__


#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <am_msgs/VisionAction.h>

class Vision
{
public:
	Vision();
	~Vision(){;};

	void handle(const am_msgs::VisionGoal::ConstPtr &goal);
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<am_msgs::VisionAction> vision_server_;
	std::string vision_action_name_;


	// create messages that are used to published feedback/result
	am_msgs::VisionFeedback vision_feedback_;
	am_msgs::VisionResult   vision_result_;

};

#endif //VISION_HPP__
