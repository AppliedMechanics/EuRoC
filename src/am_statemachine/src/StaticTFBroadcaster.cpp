/*
 * StaticTFBroadcaster.cpp
 *
 *  Created on: Aug 6, 2014
 *      Author: euroc_admin
 */

#include <StaticTFBroadcaster.h>
#include <euroc_input.hpp>

namespace fsm {

StaticTFBroadcaster::StaticTFBroadcaster() {

	publish_static_tf();

}

StaticTFBroadcaster::~StaticTFBroadcaster() {
	// TODO Auto-generated destructor stub
}

void StaticTFBroadcaster::fill_tf_information(EurocInput data)
{

}

void StaticTFBroadcaster::publish_static_tf()
{
	br_.sendTransform(tf::StampedTransform(T_LA_Base,ros::Time::now(),ORIGIN,LA_0));
	br_.sendTransform(tf::StampedTransform(T_GP,ros::Time::now(),LWR_TCP,GP_0));
	br_.sendTransform(tf::StampedTransform(T_GP_TCP,ros::Time::now(),GP_0,GP_TCP));
	br_.sendTransform(tf::StampedTransform(T_TRGB,ros::Time::now(),GP_0,T_RGB));
	br_.sendTransform(tf::StampedTransform(T_TDEPTH,ros::Time::now(),T_RGB,T_DEPTH));
	br_.sendTransform(tf::StampedTransform(T_CM,ros::Time::now(),ORIGIN,CM));
	br_.sendTransform(tf::StampedTransform(T_PT_Base,ros::Time::now(),CM,PT_BASE));
	br_.sendTransform(tf::StampedTransform(T_SRGB,ros::Time::now(),PT_TCP,S_RGB));
	br_.sendTransform(tf::StampedTransform(T_SDEPTH,ros::Time::now(),S_RGB,S_DEPTH));
}



} /* namespace fsm */
