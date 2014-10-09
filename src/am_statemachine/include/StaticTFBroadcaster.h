/*
 * StaticTFBroadcaster.h
 *
 *  Created on: Aug 6, 2014
 *      Author: euroc_admin
 */

#ifndef STATICTFBROADCASTER_H_
#define STATICTFBROADCASTER_H_

#include <ros/ros.h>
#include <am_msgs/SetStaticTFData.h>
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

	ros::ServiceClient set_static_tf_data_client_;
	std::string set_static_tf_data_;
	am_msgs::SetStaticTFData set_static_tf_data_srv_;
};

#endif /* STATICTFBROADCASTER_H_ */
