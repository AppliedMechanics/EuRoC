#include <state_observer.hpp>


StateObserver::StateObserver(ros::NodeHandle *nh)
{
	counter = 0;
	stamp_ = 0;
	//stop_pub = nh->advertise<std_msgs::Bool>("stop", 1);

	limit=50;
}

StateObserver::~StateObserver()
{

}

bool StateObserver::check_state()
{
	counter++;

	if(counter%100==0)
	{
		ROS_INFO("============================");
		ROS_INFO("Telemetry from stamp %llu:", stamp_);
		ROS_INFO("joint name joint 1: %s",telemetry_.joint_names[2].c_str());
		ROS_INFO("external torque joint 1: %f",telemetry_.measured.external_torque[2]);
		ROS_INFO("internal torque joint 1: %f",telemetry_.measured.torque[2]);
		ROS_INFO("position joint 1: %f",telemetry_.measured.position[2]);
	}

	std_msgs::Bool msg;
	//double abs_ext_t = telemetry_.measured.external_torque[2]+telemetry_.measured.torque[2];
	/*if((abs_ext_t>limit)||(abs_ext_t<-limit))
	{
		msg.data=true;
		try{
			stop_pub.publish(msg);
			msg_warn("Published stopping message!");
		}
		catch(...)
		{
			return false;
		}
		return false;
	}*/

	return true;
}

void StateObserver::callback(const euroc_c2_msgs::Telemetry& msg)
{
	//double starting_time = ros::Time::now().toNSec();

	if(&msg)
	{
		telemetry_ = msg;
		stamp_ = ros::Time::now().toNSec();
	}
	else
		msg_error("invalid message received!");

	//double stopping_time = ros::Time::now().toNSec();
	//ROS_INFO("time for get_telemetry: %e",stopping_time-starting_time);
}
