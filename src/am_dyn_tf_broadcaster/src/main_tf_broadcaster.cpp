/*
 * main_dyn_tf_broadcaster.cpp
 *
 *  Created on: Aug 7, 2014
 *      Author: euroc_admin
 */

//! ROS
#include <ros/ros.h>

//! AM
#include <utils.hpp>
#include <config.hpp>
#include <TFBroadcaster.hpp>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "am_tf_broadcaster");
	ros::NodeHandle n;

	TFBroadcaster tf_broadcaster;
	ros::ServiceServer service = n.advertiseService("set_static_tf_data", &TFBroadcaster::set_static_tf_data, &tf_broadcaster);

	double loop_rate;

	while (ros::ok()) {
		ros::Duration(0.1).sleep();
		tf_broadcaster.update_tf();
		ros::spinOnce();
	}

	//ros::spin();

	return 0;
}
