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
#include <tf/LinearMath/Transform.h>

class EurocInput
{
	friend class StaticTFBroadcaster;

public:
	EurocInput();
	~EurocInput();

	//!parse yaml file from euroc_server and save data into variables
	int parse_yaml_file(std::string task_yaml_description, const uint16_t task_nr);

	//!sort objects and create the best sequence
	int sort_objects(std::vector<uint16_t> target_zone_occupied);

	//!select a new object that is not finished (preferred: right after active_object)
	void select_new_object();
	//!get active object
	am_msgs::Object get_active_object();
	//!get active object state
	uint16_t get_active_object_state(){return obj_state_[obj_queue_[0].obj_idx];};
	uint16_t get_active_object_action(){return obj_queue_[0].action;};
	uint16_t get_active_object_idx(){return obj_queue_[0].obj_idx;};
	ros::Time get_active_object_stamp(){return obj_queue_[0].data->stamp;};

	//!print object properties
	void print_object(am_msgs::Object*obj);
	//!set active_object_ to finished
	void set_active_object_finished();
	//!set abs. pose for current object
	void set_object_pose(geometry_msgs::Pose abs_pose, ros::Time stamp);
	//!tried every object once
	bool is_active_object_last_object();

	//!get the target pose which belongs to the current object (for puzzle)
	geometry_msgs::Pose get_active_target_pose();

	//!get for active_object the corresponding target_zone
	am_msgs::TargetZone get_active_target_zone();
	void get_all_zones(std::vector<am_msgs::TargetZone>*tz);

	//!interface for sensors
	uint16_t get_nr_sensors(){return nr_sensors;};
	am_msgs::Sensor get_sensors(uint16_t nr){return sensors_[nr];};
	double get_time_limit(){return time_limit_;};
	uint16_t get_nr_objects(){return nr_objects_;};

	//!hack
	am_msgs::Object get_object(uint16_t idx){return objects_[idx];};
	am_msgs::TargetZone get_target_zone(uint16_t idx){return target_zones_[idx];};
	void set_object_pose(geometry_msgs::Pose abs_pose,uint16_t idx, ros::Time stamp);


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
	//!save puzzle fixture pose to parameter server
	void save_fixture_to_parameter_server(ros::NodeHandle& n, bool show_log_messages);
	//!save conveyor belt data to parameter server
	void save_conveyorbelt_to_parameter_server(ros::NodeHandle& n, bool show_log_messages);


	//!determine in which order the puzzle peaces need to be placed into the fixture
	void order_of_puzzle_pieces();


	typedef enum{
		EIN_OBJ_INIT=0,
		EIN_OBJ_LOCATED,
		EIN_OBJ_PARKING,
		EIN_OBJ_FINISHED,
		EIN_OBJ_UNCERTAIN
	}ein_obj_state_t;
	typedef enum{
		EIN_PLACE=0,
		EIN_PARKING,
		EIN_PLACE_FROM_PARKING
	}ein_action_t;
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

	typedef struct{
		uint16_t obj_idx;
		am_msgs::Object *data;
		uint16_t action;
		uint16_t target_zone_idx;
		uint16_t target_zone_occupied;
		uint16_t target_zone_obj_idx;
	}obj_queue_t;

	//!state vector for all objects
	std::vector<uint16_t> obj_state_;
	//!sequence for objects
	std::vector<obj_queue_t> obj_queue_;

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

	struct puzzle_piece_
	{
	  int part_index;
	  bool x_first;
	  bool push;
	};

	//!the final order for puzzle peace placement & info on how to place them
	std::vector<puzzle_piece_> puzzle_order_;

	//!place pose offset in puzzle fixture FOR
	double place_x_offset;
	double place_y_offset;
	double place_z_offset;

public:
	//!counter for objects with the same color (only for task 5)
	uint16_t same_color_counter_;
};

#endif //#define __EUROC_INPUT_HPP__
