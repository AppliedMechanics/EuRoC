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


  //statemachine class
  Statemachine* sm = Statemachine::get_instance();

  //init statemachine
  if(-1 == sm->init_sm())
  {
	  msg_error("Error initializing Statemachine");
  }

  //run statemachine
  sm->execute();

  return 0;
}

