#ifndef __EUROC_INPUT_HPP__
#define __EUROC_INPUT_HPP__

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <am_msgs/Object.h>
#include <am_msgs/Robot.h>
#include <am_msgs/TargetZone.h>
#include <am_msgs/Sensor.h>
#include <am_msgs/ConveyorBelt.h>

// Includes for parsing yaml data
#include <yaml-cpp/yaml.h>
#include "config.hpp"

#include <tf/LinearMath/Quaternion.h>

class EurocInput
{
	friend class StaticTFBroadcaster;

public:
	EurocInput();
	~EurocInput();

	int parse_yaml_file(std::string task_yaml_description, const uint16_t task_nr);

	//!select a new object that is not finished (preferred: right after active_object)
	void select_new_object();
	//!get active object
	am_msgs::Object get_active_object();
	//!print object properties
	void print_object(am_msgs::Object*obj);
	//!set active_object_ to finished
	void set_active_object_finished();
	//!tried every object once
	bool is_active_object_last_object();

	uint16_t get_active_object_idx(){return active_object_;};

	//!get for active_object the corresponding target_zone
	am_msgs::TargetZone get_active_target_zone();

	//!interface for sensors
	uint16_t get_nr_sensors(){return nr_sensors;};
	am_msgs::Sensor get_sensors(uint16_t nr){return sensors_[nr];};
	double get_time_limit(){return time_limit_;};

	//!all objects finished
	bool all_finished();

	//!reset all variables
	void reset();

	//!save the object data to the ros parameter server
	void save_objects_to_parameter_server(ros::NodeHandle& n, bool show_log_messages);
	//!save the robot data to the ros parameter server
	void save_robot_to_parameter_server(ros::NodeHandle& n, bool show_log_messages);
	//!save target zone data to parameter server
	void save_target_zone_to_parameter_server(ros::NodeHandle& n, bool show_log_messages);

private:
	//!task number of active task
	uint16_t task_nr_;
	//!task description
	std::string description_;

	//!filename for logger
	std::string log_filename_;

	//!time limit for the task
	double time_limit_;

	//!mast of cam: The TCM transformation
	geometry_msgs::Pose base_pose_;

	//!mast of cam: The TPT Base transformation
	geometry_msgs::Pose pan_tilt_base_;

	//!mast of cam: speed limit
	double speed_limit_[2];

	//!objects
	std::vector<am_msgs::Object> objects_;

	//!number of objects
	uint8_t nr_objects_;

	//!index for current active object
	int8_t active_object_;

	//!vector with values 0/1 for not finished/finished
	std::vector<uint32_t> obj_finished_;

	//!target zones
	std::vector<am_msgs::TargetZone> target_zones_;

	//!number of target_zones
	uint8_t nr_zones_;

	//!index for active zone (for active object)
	int8_t active_zone_;

	//!robot
	am_msgs::Robot robot_;

	//!sensors
	std::vector<am_msgs::Sensor> sensors_;

	//!number of sensors
	uint16_t nr_sensors;

	//!puzzle fixture pose
	geometry_msgs::Pose fixture_pose_;

	//!relative puzzle fixture target poses
	std::vector<geometry_msgs::Pose> puzzle_target_poses_;

	//!Conveyor Belt
	am_msgs::ConveyorBelt conv_belt_;
};

#endif //#define __EUROC_INPUT_HPP__
