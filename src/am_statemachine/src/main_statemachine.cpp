#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  //init ros system, register as node "sm"
  ros::init(argc, argv, "sm");

  /*
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle sm_node;

  //inform Master that program wants to publish on the topic "sm_pub"
  //->returns a publisher object
  //second argument: message queue
  ros::Publisher sm_pub = sm_node.advertise<std_msgs::String>("sm_pub",1000);

  //update rate
  ros::Rate loop_rate(100);

  int64_t count=0;

  //main loop
  while(ros::ok())
  {

	  //sleep for the remaining time
	  loop_rate.sleep();
	  ++count;
  }
}
