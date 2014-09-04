/*
 * MovePantilt.h
 *
 *  Created on: Aug 18, 2014
 *      Author: euroc_admin
 */

#ifndef MOVEPANTILT_H_
#define MOVEPANTILT_H_

// ROS
#include <ros/ros.h>

// AM
#include <config.hpp>
#include <utils.hpp>

// EUROC
#include <euroc_c2_msgs/MoveAlongJointPath.h>
#include <euroc_c2_msgs/Telemetry.h>


class MovePantilt {
private:
	MovePantilt();
	~MovePantilt();


	//!statemachine instance (singleton)
	static MovePantilt* instance_;

	ros::NodeHandle nh_;
	ros::ServiceClient move_along_joint_path_client_;

	euroc_c2_msgs::MoveAlongJointPath move_along_joint_path_srv_;
	euroc_c2_msgs::Configuration commanded_configuration_;
	euroc_c2_msgs::Configuration current_configuration_;
	euroc_c2_msgs::Telemetry current_telemetry_;

	double pan_value;
	double tilt_value;

	std::string euroc_c2_interface_;
	std::string telemetry_;
	std::string move_along_joint_path_;


	bool set_goal_configuration(double* commanded_positions);
	bool get_current_configuration();
	bool getTelemetry();

public:
	//!singleton implementation
	static MovePantilt* get_instance();
	//!destroy current instance
	void clear_instance();

	//! Public Callbacks
	//! Moving Pan Tilt Unit
	bool move_pan_tilt_abs(double pan, double tilt);
	bool move_pan_tilt_rel(double pan_rel, double tilt_rel);


	//! Getting current values
	double get_tilt();
	double get_pan();

	void set_pan_tilt_limits(double pan_max_acc,double pan_max_vel,double tilt_max_acc,double tilt_max_vel);


};

inline MovePantilt* MovePantilt::get_instance()
{
	if(instance_==NULL)
	{
		instance_=new MovePantilt();
	}
	return instance_;
};
inline void MovePantilt::clear_instance()
{
	if(instance_!=NULL)
	{
		delete instance_;
	}
}

#endif /* MOVEPANTILT_H_ */
