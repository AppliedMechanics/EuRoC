
#ifndef _STATE_OBSERVER_HPP_
#define _STATE_OBSERVER_HPP_

#include <ros/ros.h>
#include <iostream>
#include "std_msgs/Bool.h"

#include <euroc_c2_msgs/Telemetry.h>
#include <euroc_c2_msgs/StopCondition.h>
#include <euroc_c2_msgs/SearchIkSolution.h>
#include <euroc_c2_msgs/MoveAlongJointPath.h>
#include <euroc_c2_msgs/GetEstimatedExternalForce.h>
#include <euroc_c2_msgs/SetStopConditions.h>
#include <euroc_c2_msgs/SetObjectLoad.h>


#include <am_msgs/ObjectPickedUp.h>

#include <config.hpp>
#include <utils.hpp>

class StateObserver
{
public:
	StateObserver();
	~StateObserver();

	bool check_state();
	void callback(const euroc_c2_msgs::Telemetry& msg);
	void CallSetStopConditions(uint16_t level); // gives initial limits. Merge with SetStopCondition function???
	void CallSetStopConditions(uint16_t level,uint16_t joint_nbr); // TODO Make into service
	float* GetEstimatedTorques();
	bool StopConditionIsActive(); // Check whether robot has been stopped --> MotionPlanning
	bool return_object_picked_up(am_msgs::ObjectPickedUp::Request &req, am_msgs::ObjectPickedUp::Response &res); // test whether force at gripper corresponds to object mass. Use SetObjectLoad service when yes.

private:
        //!node handle for this node
        ros::NodeHandle nh;

        // Subscribe to telemetry messages
        ros::Subscriber telemetry_subscriber_;
        // Subscribe to majpc service
        ros::ServiceClient get_estimated_external_force_client_;
        // Subscribe to SetObjectLoad service
        ros::ServiceClient set_object_load_client_;
        // SUbscribe to SetStopConditions service
        ros::ServiceClient set_stop_conditions_client_;
        // Create ObjectPickedUp service
        ros::ServiceServer object_picked_up_srv;

        euroc_c2_msgs::GetEstimatedExternalForce get_estimated_external_force_srv;
	//telemetry message for callback
	euroc_c2_msgs::Telemetry _telemetry;
	//Set stop condition message
	euroc_c2_msgs::SetStopConditions set_stop_conditions_srv_;
	euroc_c2_msgs::SetObjectLoad set_object_load_srv_;

        std::string euroc_c2_interface_;
        std::string telemetry_;
//      std::string stop_condition_;
        std::string estimated_external_force_;
        std::string set_stop_conditions_;
        std::string set_object_load_;

        // current_configuration will hold our current joint position data extracted from the measured telemetry
        euroc_c2_msgs::Configuration current_configuration;
        euroc_c2_msgs::Configuration desired_configuration;
        euroc_c2_msgs::Configuration solution_configuration;


	//publisher to send stop message (bool)
	ros::Publisher stop_pub;
//	//publisher to send stop condition message
//	ros::Publisher stop_condition_pub;

	uint64_t counter;
	uint64_t stamp_;

	float force_limits[8];
        float force_limits_max[8];
	float estimated_torques[8]; // observed total torques saved as an array

	std::vector<int> level_1; // joint indices that denote which level of security each joint torque limit needs.
	std::vector<int> level_2;
};

#endif
