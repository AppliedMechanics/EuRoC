/*
 * StaticTFBroadcaster.h
 *
 *  Created on: Aug 6, 2014
 *      Author: euroc_admin
 */

#ifndef STATICTFBROADCASTER_H_
#define STATICTFBROADCASTER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <config.hpp>

namespace fsm {

class EurocInput;

class StaticTFBroadcaster {
public:
	StaticTFBroadcaster();
	virtual ~StaticTFBroadcaster();

	void publish_static_tf();
	void fill_tf_information(EurocInput*);

private:
	ros::NodeHandle nh_;
	static tf::TransformBroadcaster br_;

	tf::Transform T_LA_Base;
	tf::Transform T_GP;
	tf::Transform T_GP_TCP;
	tf::Transform T_TRGB;
	tf::Transform T_TDEPTH;
	tf::Transform T_CM;
	tf::Transform T_PT_Base;
	tf::Transform T_SRGB;
	tf::Transform T_SDEPTH;




};

} /* namespace fsm */
#endif /* STATICTFBROADCASTER_H_ */
