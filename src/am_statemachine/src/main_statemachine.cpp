#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <utils.hpp>
#include <fsm_state.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/client/terminal_state.h>
#include <am_statemachine/testAction.h>

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

#if 0 //Old:
//inform Master that program wants to publish on the topic "sm_pub"
  //->returns a publisher object
  //second argument: message queue
  ros::Publisher sm_pub = sm_node.advertise<std_msgs::String>("sm_pub",1000);


  actionlib::SimpleActionClient<am_statemachine::testAction> ac("test_node", true);
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //for infinite time
  ROS_INFO("Action server started, sending goal.");

  am_statemachine::testGoal goal;
  goal.blub = 10;
  ac.sendGoal(goal);

  bool finished=false;
  //main loop
  while(ros::ok() && finished)
  {
	  std_msgs::String msg;

	  msg.data = "Hello World";

	  if(count%100==0)
		  ROS_INFO("%s",msg.data.c_str());

	  actionlib::SimpleClientGoalState state = ac.getState();
	  //PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
	  if(state != actionlib::SimpleClientGoalState::ACTIVE)
	  {
		  switch(state.state_)
		  {
		  case actionlib::SimpleClientGoalState::SUCCEEDED:
			  finished=true;
			  break;
		  case actionlib::SimpleClientGoalState::ABORTED:
			  ROS_ERROR("action aborted!");
			  break;
		  default:
			  ROS_INFO("something wrong happened");
			  break;
		  }
	  }

	  //send a message
	  sm_pub.publish(msg);

	  //"tick"
	  ros::spinOnce();

	  //sleep for the remaining time
	  loop_rate.sleep();
	  count++;
  }
  ROS_INFO("ACTION succeeded!!!");
#endif

