#include <state_observer.hpp>


StateObserver::StateObserver(ros::NodeHandle *nh)
{
	counter = 0;
	stamp_ = 0;
	stop_pub = nh->advertise<std_msgs::Bool>("stop", 1);

	limit=40;
}

StateObserver::~StateObserver()
{

}

bool StateObserver::check_state()
{
	counter++;

	uint16_t joint_num=5;

	if(counter%100==0)
	{
		ROS_INFO("============================");
		ROS_INFO("Telemetry from stamp %llu:", stamp_);
		ROS_INFO("joint name joint 1: %s",telemetry_.joint_names[joint_num].c_str());
		ROS_INFO("external torque joint 1: %f",telemetry_.measured.external_torque[joint_num]);
		ROS_INFO("internal torque joint 1: %f",telemetry_.measured.torque[joint_num]);
		ROS_INFO("position joint 1: %f",telemetry_.measured.position[joint_num]);
	}

	if(stamp_!=0)
	{
		std_msgs::Bool msg;
		//std::cout<<"size:"<<telemetry_.measured.external_torque.size()<<std::endl;
		double abs_ext_t = telemetry_.measured.external_torque[joint_num]+telemetry_.measured.torque[joint_num];
		if((abs_ext_t>limit)||(abs_ext_t<-limit))
		{
			msg.data=true;
			try{
				stop_pub.publish(msg);
				msg_warn("Published stopping message!");
				msg_warn("joint torque: %f",abs_ext_t);
			}
			catch(...)
			{
				msg_error("failed to publish stop message");
				return false;
			}
		}
	}

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
