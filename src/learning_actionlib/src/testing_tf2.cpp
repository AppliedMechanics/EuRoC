/*
 * testing_tf2.cpp
 *
 *  Created on: Aug 12, 2014
 *      Author: euroc_admin
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>



int main(int argc, char** argv){
	ros::init(argc, argv, "my_tf_listener");

	ros::NodeHandle node;

	tf::TransformListener listener;

	ros::Rate rate(1.0);
	while (node.ok()){

		//    tf2_ros::StampedTransform transform;
		tf::StampedTransform transform;
		tf::StampedTransform transform2;
		ros::Time now = ros::Time::now();
		try{
			listener.waitForTransform("GP_0", "LWR_TCP",now,ros::Duration(1.0));
			listener.lookupTransform("GP_0", "LWR_TCP",now,transform);
		}
		catch (...){
			ROS_ERROR("static tf2 does not work");
		}
		try{
			listener.waitForTransform("CM", "LWR_TCP",now,ros::Duration(1.0));
			listener.lookupTransform("CM", "LWR_TCP",now,transform2);
		}
		catch (...){
			ROS_ERROR("intern tf2 does not work");
		}

		std::cout<<"Transform GP_TCP to LWR_TCP in z = "<<transform.getOrigin().getZ()<<std::endl;
		std::cout<<"Transform LWR_0 to LWR_TCP in z = "<<transform2.getOrigin().getZ()<<std::endl;
		rate.sleep();
	}
	return 0;
};


