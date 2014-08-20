#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <utils.hpp>
#include <config.hpp>
#include <stdio.h>
#include <signal.h>

#include "statemachine.hpp"

Statemachine* sm;

void signalHandler(int sig)
{
    ROS_ERROR("Interrupt signal ( %d ) received.\n",sig);

    ros::shutdown();
    sm->clear_instance();
    //exit(sig);
}

int main(int argc, char **argv)
{
  //init ros system, register as node "sm"
  ros::init(argc, argv, "statemachine",ros::init_options::NoSigintHandler);

  signal(SIGINT,  signalHandler);
  signal(SIGKILL, signalHandler);
  signal(SIGABRT, signalHandler);
  signal(SIGTERM, signalHandler);

  //statemachine class
  sm = Statemachine::get_instance();

  //init statemachine
  if(-1 == sm->init_sm())
  {
	  msg_error("Error initializing Statemachine");
  }

//  //run statemachine
//  sm->execute();

  //update rate
  ros::Rate loop_rate(FREQ);

  //main loop:
  double t_act=0;
  fsm::fsm_state_t sm_state=sm->get_state();
  ROS_INFO("Entering while-loop");
  while(ros::ok() && !(sm_state.sub.one == fsm::FINISHED))
  {

	  //perform main tick-routine
	  if(-1==sm->tick())
	  {
		  msg_error("Error in Statemachine::tick()");
	  }

	  //"tick"
	  ros::spinOnce();


	  //update state
	  sm_state=sm->get_state();

	  //sleep for the remaining time (only during solve task)
	  if(sm_state.sub.one == fsm::SOLVE_TASK)
		  loop_rate.sleep();

	  t_act=(double)ros::Time::now().toSec();

	  printf("time= %f s \r",t_act);
	  fflush(stdout);

  }
  ROS_INFO("Nach while");
  sm->clear_instance();

  return 0;
}

