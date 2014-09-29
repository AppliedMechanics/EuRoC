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

	//create stateobserver
	StateObserver so;

	//update rate
	ros::Rate loop_rate(FREQ);

	double curr_time=ros::Time::now().toSec();
	//main loop:
	while(ros::ok())
	{

	  if(so.check_state()==false)
	  {
		  msg_error("Error in check_state!");
	  }

	  //"tick"
	  // The spinOnce fuction of ros will execute the callbacks of our subscribed topics once.
	  // This will update our global _telemetry message.
	  ros::spinOnce();
	  loop_rate.sleep();

	  //curr_time=ros::Time::now().toSec();
	  //ROS_INFO("time= %f s",curr_time);

	}

	return 0;
}

