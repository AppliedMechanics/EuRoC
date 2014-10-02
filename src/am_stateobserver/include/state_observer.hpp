
#ifndef _STATE_OBSERVER_HPP_
#define _STATE_OBSERVER_HPP_

#include <ros/ros.h>
#include <iostream>
#include "std_msgs/Bool.h"
#include <boost/thread.hpp>

#include <euroc_c2_msgs/Telemetry.h>
#include <euroc_c2_msgs/StopCondition.h>
#include <euroc_c2_msgs/SearchIkSolution.h>
#include <euroc_c2_msgs/MoveAlongJointPath.h>
#include <euroc_c2_msgs/GetEstimatedExternalForce.h>
#include <euroc_c2_msgs/SetStopConditions.h>
#include <euroc_c2_msgs/SetObjectLoad.h>


#include <am_msgs/ObjectPickedUp.h>
#include <am_msgs/CallSetStopConditions.h>

#include <config.hpp>
#include <utils.hpp>

class StateObserver
{
public:
	StateObserver();
	~StateObserver();

	bool check_state();
	void callback(const euroc_c2_msgs::Telemetry& msg);
	bool CallSetStopCondition(am_msgs::CallSetStopConditions::Request &req, am_msgs::CallSetStopConditions::Response &res);
	// test whether force at gripper corresponds to object mass. Use SetObjectLoad service when yes.
	bool ReturnObjectPickedUp(am_msgs::ObjectPickedUp::Request &req, am_msgs::ObjectPickedUp::Response &res);
	bool getTelemetry();

private:
        //!node handle for this node
        ros::NodeHandle nh;

        // Subscribe to telemetry messages
        ros::Subscriber telemetry_subscriber_;
        // Subscribe to majpc service
        ros::ServiceClient get_estimated_external_force_client_;
        // Subscribe to SetObjectLoad service
        ros::ServiceClient set_object_load_client_;
        // Subscribe to SetStopConditions service
        ros::ServiceClient set_stop_conditions_client_;
        // Create ObjectPickedUp service
        ros::ServiceServer object_picked_up_srv;
        // Create CallSetStopConditions service
        ros::ServiceServer call_set_stop_conditions_srv;

        euroc_c2_msgs::GetEstimatedExternalForce get_estimated_external_force_srv;
	//telemetry message for callback
	euroc_c2_msgs::Telemetry _telemetry;
	//Set stop condition message
	euroc_c2_msgs::SetStopConditions set_stop_conditions_srv_;
	euroc_c2_msgs::SetObjectLoad set_object_load_srv_;

        std::string euroc_c2_interface_;
        std::string telemetry_;
        std::string estimated_external_force_;
        std::string set_stop_conditions_;
        std::string set_object_load_;

        // current_configuration will hold our current joint position data extracted from the measured telemetry
//        euroc_c2_msgs::Configuration current_configuration;

	//publisher to send stop message (bool)
	ros::Publisher stop_pub;

	uint64_t counter;
	uint64_t stamp_;

	int sim_time_sec_;  // to print simulation time with telemetry info
	int sim_time_nsec_;
	float max_torques[8];  //maximum torques got from telemetry saved here.

	float force_limits[8];
        float force_limits_max[8];
	float security[8]; // security limit for all joints + gripper

};

#endif
