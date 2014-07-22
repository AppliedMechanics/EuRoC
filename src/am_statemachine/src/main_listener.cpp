#include "ros/ros.h"
#include "std_msgs/String.h"

void listenerCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I got: [%s]",msg->data.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"listener");

	ros::NodeHandle node;

	//subscribe to message
	ros::Subscriber sub = node.subscribe("sm_pub",1000,listenerCallback);

	ros::spin();

	return 0;
}
