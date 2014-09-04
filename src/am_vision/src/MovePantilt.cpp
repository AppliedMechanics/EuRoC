/*
 * MovePantilt.cpp
 *
 *  Created on: Aug 18, 2014
 *      Author: euroc_admin
 */

#include <MovePantilt.h>

MovePantilt* MovePantilt::instance_ = 0x0;

MovePantilt::MovePantilt() {
	// TODO Auto-generated constructor stub
	euroc_c2_interface_ = "/euroc_interface_node";
	telemetry_ = euroc_c2_interface_ + "/telemetry";
	move_along_joint_path_ = euroc_c2_interface_ + "/move_along_joint_path";

	move_along_joint_path_client_ = nh_.serviceClient<euroc_c2_msgs::MoveAlongJointPath>(move_along_joint_path_);


	move_along_joint_path_srv_.request.joint_names.resize(2);

	move_along_joint_path_srv_.request.joint_names[0] = "cam_pan";
	move_along_joint_path_srv_.request.joint_names[1] = "cam_tilt";

	move_along_joint_path_srv_.request.path.resize(1); // Our path has only one waypoint

	move_along_joint_path_srv_.request.joint_limits.resize(2);

	move_along_joint_path_srv_.request.joint_limits[0].max_acceleration = 400 * M_PI / 180.0;
	move_along_joint_path_srv_.request.joint_limits[0].max_velocity = 1.7453;
	move_along_joint_path_srv_.request.joint_limits[1].max_acceleration = 400 * M_PI / 180.0;
	move_along_joint_path_srv_.request.joint_limits[1].max_velocity = 1.7453;

	commanded_configuration_.q.resize(2);
	current_configuration_.q.resize(2);

}
MovePantilt::~MovePantilt() {
	delete instance_;
}

//! Public Callbacks
bool MovePantilt::move_pan_tilt_abs(double pan, double tilt)
{
	double positions[2] = {pan,tilt};
	if (set_goal_configuration(positions))
		return true;
	else
		return false;
}
bool MovePantilt::move_pan_tilt_rel(double pan_rel, double tilt_rel)
{
	double positions[2];
	if (get_current_configuration())
	{
		positions[0] = current_configuration_.q[0]+pan_rel;
		positions[1] = current_configuration_.q[1]+tilt_rel;

		if (set_goal_configuration(positions))
			return true;
		else
			return false;	}
	else
		return false;
}

double MovePantilt::get_tilt()
{
	if (get_current_configuration())
		tilt_value = current_configuration_.q[1];
	return tilt_value;
}
double MovePantilt::get_pan()
{
	if (get_current_configuration())
		pan_value = current_configuration_.q[0];
	return pan_value;
}
void MovePantilt::set_pan_tilt_limits(double pan_max_acc,double pan_max_vel,double tilt_max_acc,double tilt_max_vel)
{
	move_along_joint_path_srv_.request.joint_limits[0].max_acceleration = pan_max_acc;
	move_along_joint_path_srv_.request.joint_limits[0].max_velocity = pan_max_vel;
	move_along_joint_path_srv_.request.joint_limits[1].max_acceleration = tilt_max_acc;
	move_along_joint_path_srv_.request.joint_limits[1].max_velocity = tilt_max_vel;
}

//! Internal Functions
bool MovePantilt::set_goal_configuration(double* commanded_positions)
{
	try
	{
		if (ros::service::waitForService(move_along_joint_path_,ros::Duration(1.0)))
		{
			commanded_configuration_.q[0] = commanded_positions[0];
			commanded_configuration_.q[1] = commanded_positions[1];

			// Extract the solution configuration from the response and fill it into the path of the move request
			std::vector<euroc_c2_msgs::Configuration> &path = move_along_joint_path_srv_.request.path;
			path[0] = commanded_configuration_;

			// Call the move request and check for errors
			move_along_joint_path_client_.call(move_along_joint_path_srv_);
			std::string &move_error_message = move_along_joint_path_srv_.response.error_message;
			if(!move_error_message.empty()){
				std::cout << "Move failed: " + move_error_message << std::endl;
				return false;
			}
			return true;
		}
		else{
			msg_warn("Service move to target configuration has not been advertised yet.");
			return false;
		}
	}
	catch (...)
	{
		return false;
	}
}

bool MovePantilt::get_current_configuration()
{
	try
	{
		if (getTelemetry())
		{
			// Get the current configuration from the telemetry message
			for(unsigned int i = 0; i < 2; ++i){
				std::vector<std::string> &joint_names = (current_telemetry_.joint_names);
				unsigned int telemetry_index = std::find(joint_names.begin(), joint_names.end(), move_along_joint_path_srv_.request.joint_names[i]) - joint_names.begin();
				current_configuration_.q[i] = current_telemetry_.measured.position[telemetry_index];
			}
			return true;
		}
		else
			return false;
	}
	catch (...)
	{
		return false;
	}

}

bool MovePantilt::getTelemetry()
{
	current_telemetry_ = *(ros::topic::waitForMessage<euroc_c2_msgs::Telemetry>(telemetry_,ros::Duration(1.0)));
	if (&current_telemetry_==NULL)
	{
		msg_warn("No telemetry message received.");
		return false;
	}
	else
		return true;
}

