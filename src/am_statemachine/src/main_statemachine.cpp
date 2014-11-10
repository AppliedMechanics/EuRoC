#include "statemachine.hpp"
#include <signal.h>

//Variables
Statemachine* sm; 	//global pointer to statemachine object

int main(int argc, char **argv)
{
	//init this node in the ros-system
	ROS_INFO("Main: initializing the node statemachine...");
	try
	{
		//init node and tell ros it should not handle interrupt signals
		ros::init(argc, argv, "statemachine",ros::init_options::NoSigintHandler);
	}
	catch(...) 	//catch all possible errors
	{
		msg_error("Main: Error. ros::init(...) failed\n");
		//this fail is critical, so the program has to be stopped
		exitstatemachine();
		return 1;
	}
	ROS_INFO("Main: initialization of the node statemachine done.");

	//register the method signalHandler to process interrupt signals
	//(only works for signals in the terminal window of the statemachine)
	signal(SIGINT,  signalHandler);
	signal(SIGKILL, signalHandler);
	signal(SIGABRT, signalHandler);
	signal(SIGTERM, signalHandler);

	//allocate new instance of statemachine object and set sm to point to it
	sm = new Statemachine();

	//try to initialize statemachine
	ROS_INFO("Main: initializing the statemachine-class...");
	if(-1 == sm->init_sm())
	{
		msg_error("Main: Error. statemachine::init_sm() failed\n");
		//this fail is critical so the program has to be stopped
		exitstatemachine();
		return 1;
	}
	ROS_INFO("Main: initialization of statemachine-class done.");

	//main loop:
	ros::Rate loop_rate(FREQ); 		//set loop rate
	double t_act=0;					//actual time for console output
	fsm::fsm_state_t sm_state=sm->get_state();
	ROS_INFO("Main: entering while-loop");
	while(ros::ok() && !(sm_state.sub.one == fsm::FINISHED))
	{
		//perform main tick-routine
		if(sm->tick()==-1)
		{
			msg_error("Main: Error. statemachine::tick() failed\n");
			//this fail is critical so the program has to be stopped
			//comment two lines below to allow errors in tick() method
			exitstatemachine();
			return 1;
		}

		//try to process callbacks
		try
		{
			ros::spinOnce();
		}
		catch(...)
		{
			//only show error message, but don't stop program here
			msg_error("Main: Error. ros::spinOnce() failed\n");
		}

		//update state to local variable
		sm_state=sm->get_state();

		//refresh actual time
		t_act=(double)ros::Time::now().toSec();
		printf("time= %fs [state: %s]\r",t_act,sm->get_state_name(sm_state).c_str());
		fflush(stdout);

		//sleep to ensure previously defined loop rate
		//during solve task use ROS-Time
		//if(sm_state.sub.one == fsm::SOLVE_TASK) //replaced by sm->sim_running_
		if(sm->sim_running_)
		{
			loop_rate.sleep();
		}
		else //otherwise use system time to sleep
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		}
	}
	ROS_INFO("Main: while-loop finished.");

	//quit properly
	exitstatemachine();

//	ROS_INFO("Press Enter to exit Statemachine");
//	int something=0;
//	std::cin>>something;

	return 0;
}

void exitstatemachine()
{
	ROS_INFO("Main: shutting down statemachine...");
	try
	{
		ros::shutdown(); 		//try to disconnect from master
		//wait for a couple of milliseconds, to let it close properly
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	}
	catch(...) 	//catch all possible errors
	{
		msg_error("Main: Error. ros::shutdown() failed\n");
		//but quit anyway
	}
	delete sm;				//delete the statemachine object for memory release (not really necessary, cleared anyway)
	ROS_INFO("Main: statemachine shut down.");
}

void signalHandler(int sig)
{
	msg_error("Main: interrupt signal ( %d ) received.\n",sig);
	//quit properly
    exitstatemachine();
}
