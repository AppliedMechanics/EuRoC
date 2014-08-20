#include <state_observer.hpp>


StateObserver::StateObserver(ros::NodeHandle *nh)
{
	euroc_c2_interface_ = "/euroc_interface_node";
	telemetry_ = euroc_c2_interface_ + "/telemetry";

	telemetry_subscriber_ = nh->subscribe(telemetry_,1,&StateObserver::callback);
}

StateObserver::~StateObserver()
{

}

bool StateObserver::check_state()
{
	double starting_time = ros::Time::now().toSec();

	_telemetry = *(ros::topic::waitForMessage<euroc_c2_msgs::Telemetry>(telemetry_,ros::Duration(10.0)));

	double stopping_time = ros::Time::now().toSec();
	ROS_INFO("time for get_telemetry: %f",stopping_time-starting_time);

	if (&_telemetry==NULL)
	{
		msg_warn("No telemetry message received.");
		return false;
	}
	else
		return true;
}

void StateObserver::callback(const euroc_c2_msgs::TelemetryConstPtr& msg)
{
	double starting_time = ros::Time::now().toSec();

	_telemetry = *msg;

	double stopping_time = ros::Time::now().toSec();
}
