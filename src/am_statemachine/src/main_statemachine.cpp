#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <utils.hpp>
#include <fsm_state.hpp>

#include "statemachine.hpp"


int main(int argc, char **argv)
{
  //init ros system, register as node "sm"
  ros::init(argc, argv, "statemachine");

  /*
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle sm_node;


  //update rate
  ros::Rate loop_rate(10);
  int64_t count=0;

  //statemachine class
  Statemachine* sm = Statemachine::get_instance();

  if(-1 == sm->init_sm())
  {
	  msg_error("Error initializing Statemachine");
  }
//  sm->tick();
//  sm->tick();
//  sm->tick();


  fsm_state_t cur_state;
  while(ros::ok() && !(cur_state.sub.one == FINISHED))
    {

	  if(-1==sm->tick())
	  {
		  msg_error("Error in Statemachine::tick()");
	  }

	  cur_state=sm->get_state();

  	  //"tick"
  	  ros::spinOnce();

  	  //sleep for the remaining time
  	  loop_rate.sleep();
  	  count++;
    }


  return 0;
}

