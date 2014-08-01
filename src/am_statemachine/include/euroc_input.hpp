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

	//!get a new object (that is not finished)
	am_msgs::Object get_object();
	//!print object properties
	void print_object(am_msgs::Object*obj);
	//!set active_object_ to finished
	void set_object_finished();

	//!interface for grasping pose
	void set_grasping_pose(geometry_msgs::Pose pose){grasping_pose_[active_object_]=pose;};
	geometry_msgs::Pose get_grasping_pose(){return grasping_pose_[active_object_];};

	//!get for active_object the corresponding target_zone
	am_msgs::TargetZone get_target_zone();

	//!interface for sensors
	uint16_t get_nr_sensors(){return nr_sensors;};
	am_msgs::Sensor get_sensors(uint16_t nr){return sensors_[nr];};

	//!all objects finished
	bool all_finished();
private:
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
	//!grasping pose for each object
	std::vector<geometry_msgs::Pose> grasping_pose_;
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
};

#endif //#define __EUROC_INPUT_HPP__
