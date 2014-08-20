#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <utils.hpp>
#include <config.hpp>
#include <stdio.h>
#include <signal.h>

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
	StateObserver so;

	//subscribe to telemetry message
	ros::Subscriber telemetry_subscriber_ = nh.subscribe(telemetry_,1,&StateObserver::callback,&so);

	//update rate
	ros::Rate loop_rate(FREQ);

	//main loop:
	while(ros::ok())
	{

	  //"tick"
	  ros::spinOnce();
	  loop_rate.sleep();

	}

	return 0;
}

