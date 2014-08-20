#include "ros/ros.h"
#include <sstream>
#include <utils.hpp>
#include <config.hpp>
#include <stdio.h>

#include "state_observer.hpp"


int main(int argc, char **argv)
{
	//init ros system, register as node "sm"
	ros::init(argc, argv, "stateobserver");


	//!node handle for this node
	ros::NodeHandle nh;

	std::string	euroc_c2_interface_ = "/euroc_interface_node";
	std::string telemetry_ = euroc_c2_interface_ + "/telemetry";

	//create stateobserver
	StateObserver so(&nh);

	//subscribe to telemetry message
	ros::Subscriber telemetry_subscriber_ = nh.subscribe(telemetry_,1,&StateObserver::callback,&so);
	//ros::Publisher stop_pub = nh.advertise<std_msgs::Bool>("stop_cond",1000);

	//update rate
	ros::Rate loop_rate(FREQ);

	double curr_time=ros::Time::now().toSec();
	//main loop:
	while(ros::ok())
	{

	  if(so.check_state()==false)
	  {
		  msg_error("Error in check_state!");

//		  std_msgs::Bool msg;
//		  msg.data=true;
//		  stop_pub.publish(msg);
	  }

	  //"tick"
	  ros::spinOnce();
	  loop_rate.sleep();

	  //curr_time=ros::Time::now().toSec();
	  //ROS_INFO("time= %f s",curr_time);

	}

	return 0;
}

