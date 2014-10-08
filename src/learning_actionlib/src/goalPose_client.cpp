#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <am_msgs/goalPoseAction.h>


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_motionplanning");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<am_msgs::goalPoseAction> ac("goalPoseAction", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  am_msgs::goalPoseGoal goal;
  geometry_msgs::Pose example_pose;
  example_pose.orientation.w = 1.0;
  example_pose.orientation.x = 0.0;
  example_pose.orientation.y = 0.0;
  example_pose.orientation.z = 0.0;

  example_pose.position.x = 0.5;
  example_pose.position.y = 0.5;
  example_pose.position.z = 0.3;

  goal.goal_pose = example_pose;
  goal.planning_algorithm = 0;
  goal.planning_frame = "GP_TCP";
  goal.speed_percentage = 30;
  ac.sendGoal(goal);


  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(300.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}



