
#ifndef _STATE_OBSERVER_HPP_
#define _STATE_OBSERVER_HPP_

#include <ros/ros.h>

#include <euroc_c2_msgs/Telemetry.h>

#include <config.hpp>
#include <utils.hpp>

class StateObserver
{
public:
	StateObserver(ros::NodeHandle*);
	~StateObserver();

	bool check_state();
	void callback(const euroc_c2_msgs::TelemetryConstPtr& msg);
private:

	std::string euroc_c2_interface_;
	std::string telemetry_;

	ros::Subscriber telemetry_subscriber_;
	euroc_c2_msgs::Telemetry _telemetry;
};

#endif
