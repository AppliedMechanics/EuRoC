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

	am_msgs::Object get_object();
	void print_object(am_msgs::Object*obj);
	void set_object_finished();

	void set_grasping_pose(geometry_msgs::Pose pose){grasping_pose_[active_object_]=pose;};
	geometry_msgs::Pose get_grasping_pose(){return grasping_pose_[active_object_];};

	am_msgs::TargetZone get_target_zone();
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
	uint8_t nr_objects_;
	int8_t active_object_;
	std::vector<geometry_msgs::Pose> grasping_pose_;
	std::vector<uint32_t> obj_finished_;
	//!Target zones
	std::vector<am_msgs::TargetZone> target_zones_;
	uint8_t nr_zones_;
	int8_t active_zone_;
	//!Robot
	am_msgs::Robot robot_;
	//!Sensors
	std::vector<am_msgs::Sensor> sensors_;
	unsigned int nr_sensors;
};

#endif //#define __EUROC_INPUT_HPP__
