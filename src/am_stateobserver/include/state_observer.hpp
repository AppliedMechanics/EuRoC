
#ifndef _STATE_OBSERVER_HPP_
#define _STATE_OBSERVER_HPP_

#include <ros/ros.h>
#include "std_msgs/Bool.h"

#include <euroc_c2_msgs/Telemetry.h>

#include <config.hpp>
#include <utils.hpp>

class StateObserver
{
public:
	StateObserver(ros::NodeHandle *);
	~StateObserver();

	bool check_state();
	void callback(const euroc_c2_msgs::Telemetry& msg);
private:

	euroc_c2_msgs::Telemetry telemetry_;

	ros::Publisher stop_pub;

	uint64_t counter;
	uint64_t stamp_;

	double limit;
};

#endif
