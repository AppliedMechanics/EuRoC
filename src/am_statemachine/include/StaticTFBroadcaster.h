/*
 * StaticTFBroadcaster.h
 *
 *  Created on: Aug 6, 2014
 *      Author: euroc_admin
 */

#ifndef STATICTFBROADCASTER_H_
#define STATICTFBROADCASTER_H_

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <config.hpp>
#include <utils.hpp>


class EurocInput;

class StaticTFBroadcaster {
public:
	StaticTFBroadcaster();
	virtual ~StaticTFBroadcaster();

	void publish_static_tf();
	void fill_tf_information(EurocInput*);

private:
	ros::NodeHandle nh_;
	tf2_ros::StaticTransformBroadcaster sbr_;

	geometry_msgs::TransformStamped T_LA_Base;
	geometry_msgs::TransformStamped T_GP;
	geometry_msgs::TransformStamped T_GP_TCP;
	geometry_msgs::TransformStamped T_TRGB;
	geometry_msgs::TransformStamped T_TDEPTH;
	geometry_msgs::TransformStamped T_CM;
	geometry_msgs::TransformStamped T_PT_Base;
	geometry_msgs::TransformStamped T_SRGB;
	geometry_msgs::TransformStamped T_SDEPTH;




};

#endif /* STATICTFBROADCASTER_H_ */
