#ifndef __EUROC_INPUT_HPP__
#define __EUROC_INPUT_HPP__

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Pose.h>
#include <am_msgs/Object.h>
#include <am_msgs/Robot.h>
#include <am_msgs/TargetZone.h>
#include <am_msgs/Sensor.h>
// Includes for parsing yaml data
#include <yaml-cpp/yaml.h>

class EurocInput
{
public:
	EurocInput();
	~EurocInput();

	int parse_yaml_file(std::string task_yaml_description);

	void get_object(am_msgs::Object* obj);
	void set_object_finished(uint32_t nr);

private:
	//
	std::string description_;
	std::string log_filename_;
	double time_limit_;

	//!mast of cam: The TCM transformation
	geometry_msgs::Pose base_pose_;
	//!mast of cam: The TPT Base transformation
	geometry_msgs::Pose pan_tilt_base_;
	//!mast of cam: speed limit
	double speed_limit_[2];

	//!Objects
	std::vector<am_msgs::Object> objects_;
	unsigned int nr_objects;
	std::vector<uint32_t> obj_finished_;
	//!Target zones
	std::vector<am_msgs::TargetZone> target_zones_;
	unsigned int nr_zones;
	//!Robot
	am_msgs::Robot robot_;
	//!Sensors
	std::vector<am_msgs::Sensor> sensors_;
	unsigned int nr_sensors;
};

#endif //#define __EUROC_INPUT_HPP__
