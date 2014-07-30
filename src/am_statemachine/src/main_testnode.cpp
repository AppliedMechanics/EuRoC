#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <am_statemachine/testAction.h>


class TestNode
{
	ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<am_statemachine::testAction> as_;
    std::string action_name_;

    // create messages that are used to published feedback/result
    am_statemachine::testFeedback feedback_;
    am_statemachine::testResult result_;

public:

  TestNode(std::string name) :
    as_(nh_, name, boost::bind(&TestNode::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~TestNode(void)
  {
  }

  void executeCB(const am_statemachine::testGoalConstPtr &goal)
    {
	  ros::Rate r(1);
	  bool success = true;

	  // push_back the seeds for the fibonacci sequence
	  feedback_.sequence.clear();
	  feedback_.sequence.push_back(0);

	  ROS_INFO("STARTING executeCB ...");

	  uint32_t counter=0;
	  bool active=true;
	  while(active)
	  {
		  if (as_.isPreemptRequested() || !ros::ok())
		  {
			  ROS_INFO("%s: Preempted", action_name_.c_str());
			  // set the action state to preempted
			  as_.setPreempted();
			  success = false;
			  break;
		  }

		  counter++;
		  //double test=1/(goal->blub-2*counter);

		  feedback_.sequence.push_back(counter);
		  as_.publishFeedback(feedback_);


		  ROS_INFO("FEEDBACK: %i",counter);

		  r.sleep();

		  if(counter==goal->blub)
			  active=false;
	  }

	  if(success)
	  	{
	  	  result_.sequence = feedback_.sequence;
	  	  ROS_INFO("%s: Succeeded", action_name_.c_str());
	  	  // set the action state to succeeded
	  	  as_.setSucceeded(result_);
	  	}
    }
};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"test_node");

	ros::NodeHandle node;

	//subscribe to message
	TestNode test_node(ros::this_node::getName());
	ros::spin();

	return 0;
}
