#include <state_observer.hpp>


StateObserver::StateObserver()
{
	counter = 0;
	stamp_ = 0;

	stop_pub = nh.advertise<std_msgs::Bool>("stop", 1);
//	stop_condition_pub = nh.advertise<euroc_c2_msgs::StopCondition>(stop_condition_, 1);

        euroc_c2_interface_ = "/euroc_interface_node";
        telemetry_ = euroc_c2_interface_ + "/telemetry";
//      stop_condition_ = euroc_c2_interface_ + "/stop_condition";
        estimated_external_force_ = euroc_c2_interface_ + "/get_estimated_external_force";
        set_stop_conditions_ = euroc_c2_interface_ + "/set_stop_conditions";
        set_object_load_ = euroc_c2_interface_ + "/set_object_load";

	telemetry_subscriber_= nh.subscribe(telemetry_, 1, &StateObserver::callback, this); // this to point at the object (&so) itself
	get_estimated_external_force_client_ = nh.serviceClient<euroc_c2_msgs::GetEstimatedExternalForce>(estimated_external_force_);
	set_object_load_client_ = nh.serviceClient<euroc_c2_msgs::SetObjectLoad>(set_object_load_);
	set_stop_conditions_client_ = nh.serviceClient<euroc_c2_msgs::SetStopConditions>(set_stop_conditions_);
	object_picked_up_srv = nh.advertiseService("ObjectPickedUp_srv", &StateObserver::return_object_picked_up, this);

	current_configuration.q.resize(7); // Resizing the current_conf vector to fit all joints


	       force_limits_max[0] = 176;
	        force_limits_max[1] = 176;
	        force_limits_max[2] = 100;
	        force_limits_max[3] = 100;
	        force_limits_max[4] = 100;
	        force_limits_max[5] = 30;
	        force_limits_max[6] = 30;
	        force_limits_max[7] = 200;  // max_gripper_force

	        for(int i = 0; i < 8; i++) // Initialise the limits to 100 Nm; SetJointTorqueLimits() will have to be called from main
	                {
	                  estimated_torques[i] = 0;
	                  force_limits[i] = force_limits_max[i]; //TODO limits from urdf file? state_observer/src/am_robot_model/kuka_lwr/kuka_lwr_mod.urdf (rosinterface.cpp)
	                }                          //-> Array erstellen (StopConditionWidget.cpp)
}

StateObserver::~StateObserver()
{

}

bool StateObserver::check_state()
{
	counter++;
	uint16_t joint_num = 0;

	// Wait for the subscribed topics to be published by the euroc simulator to avoid race conditions
	ros::topic::waitForMessage<euroc_c2_msgs::Telemetry>(telemetry_,ros::Duration(5.0));

	if(counter%100 == 0) //Rate of messages shown
	{

	  while(joint_num < 8)
	  {
		estimated_torques[joint_num] = _telemetry.measured.external_torque[joint_num +2]+_telemetry.measured.torque[joint_num +2];
		get_estimated_external_force_client_.call(get_estimated_external_force_srv);


		if(stamp_!=0) // If time stamp is not equal to nought, total torque will be compared with the limit
		{
		    std_msgs::Bool msg;

		    if((estimated_torques[joint_num] > force_limits[joint_num])
		        ||(estimated_torques[joint_num] < -force_limits[joint_num]))
		    {
			msg.data=true;  // msg.data = true when limit has been exceeded
			try
			{
			    stop_pub.publish(msg);
			    msg_warn("Published stopping message!");
			    msg_warn("joint %d total torque: %f, limit: %f",(joint_num +1), estimated_torques[joint_num], force_limits[joint_num]);
			}
			catch(...)
			{
			    msg_error("failed to publish stop message");
			    return false;
			}
		    }
		}
		joint_num++;

          }

	}

	return true;
}

void StateObserver::CallSetStopConditions(uint16_t level)
{
  double security[2] = {1.5,1.2};
  const unsigned int lwr_nr = 8;
  std::vector<std::string> lwr_names(lwr_nr);
  std::stringstream name;

  // Set new force limits
  for (int j = 0; j < lwr_nr; j++)
  {
    force_limits[j] = force_limits_max[j]/security[level];
  }

  // Populate a vector with all the lwr joint names + gripper
  for(unsigned int i = 0; i < lwr_nr; ++i)
  {
    name.str("lwr_joint_");
    name.seekp(0, std::ios_base::end);
    name << (i + 1);
    lwr_names[i] = name.str();
  }
  lwr_names[7] = "gripper";

  if (ros::service::waitForService(set_stop_conditions_,ros::Duration(1.0)))
  {
    std::vector<euroc_c2_msgs::StopCondition> stop_conditions;
    stop_conditions.resize(lwr_nr);
    for (int i = 0 ; i < lwr_nr; i++)
    {
      if (lwr_names[i].compare("gripper") == 0)
      {
        stop_conditions[i].condition_type = "tool_force_threshold";
      }
      else
      {
        stop_conditions[i].condition_type = "joint_ext_torque_threshold";
      }
      // Stop condition operators have to be set!!
                        stop_conditions[i].condition_operator = "|<";
      stop_conditions[i].joint_name = lwr_names[i];
      stop_conditions[i].threshold = force_limits[i];
      //Force direction geometry_mgs::Vector3;
    }

    for(int i = 0; i < lwr_nr; i++)
    {
      // Testing
      ROS_INFO("condition type: %s", stop_conditions[i].condition_type.c_str());
      ROS_INFO("joint name: %s", stop_conditions[i].joint_name.c_str());
      ROS_INFO("threshold: %f", stop_conditions[i].threshold);

    }

    set_stop_conditions_srv_.request.conditions = stop_conditions;

    if(set_stop_conditions_client_.call(set_stop_conditions_srv_))
    {
      ROS_INFO("Stop conditions successfully set by state_observer service");
    }
    else
    {
      msg_error("Failed to call SetStopConditions client");
    }

    std::string &sc_error_message = set_stop_conditions_srv_.response.error_message;
    if(!sc_error_message.empty())
    {
      msg_error("Setting Stop Conditions failed: %s", sc_error_message.c_str());
    }
    else
    {
      ROS_INFO("Stop Conditions have been set.");
    }
  }
  else
  {
    ROS_WARN("Set stop conditions service has not been advertised yet.");
  }
}
void StateObserver::CallSetStopConditions(uint16_t level,uint16_t joint_nbr) //request response...
{
    double security[2] = {1.5,1.2};
    force_limits[joint_nbr] = force_limits_max[joint_nbr]/security[level];

    switch(level) // when stop condition changes for joint i, the level vectors are both updated
  {
      case 0:
        for (int i = 0; i < level_1.size(); i++)
        {
          if (level_1[i] == joint_nbr)
            break;
        }
        for (int i = 0; i < level_2.size(); i++)
        {
          if(level_2[i] == joint_nbr)
            level_2.erase(level_2.begin() + i);
        }
          level_1.push_back(joint_nbr);
          break;

      case 1:
        for (int i = 0; i < level_2.size(); i++)
        {
          if (level_2[i] == joint_nbr)
            break;
        }
        for (int i = 0; i < level_1.size(); i++)
        {
          if(level_1[i] == joint_nbr)
            level_1.erase(level_1.begin() + i);
        }
          level_2.push_back(joint_nbr);
          break;
      default:
        break;
  }
}
float* StateObserver::GetEstimatedTorques()
{
        return estimated_torques;
}

void StateObserver::callback(const euroc_c2_msgs::Telemetry& msg)
{
	//double starting_time = ros::Time::now().toNSec();

	if(&msg)
	{
		_telemetry = msg;
		stamp_ = ros::Time::now().toNSec();
	}
	else
		msg_error("invalid message received!");

	//double stopping_time = ros::Time::now().toNSec();
	//ROS_INFO("time for get_telemetry: %e",stopping_time-starting_time);
}

bool StateObserver::StopConditionIsActive()
{
  return false;
}

// ROS-Service Function
bool StateObserver::return_object_picked_up(am_msgs::ObjectPickedUp::Request &req, am_msgs::ObjectPickedUp::Response &res)
{
  float gravityZ = -9.81;
  ROS_INFO("Service called");
  if(!(get_estimated_external_force_client_.call(get_estimated_external_force_srv)))
      {
        get_estimated_external_force_srv.response.error_message = "Failed to call client";
        msg_error("Failed to call client");
        return false;
      }

  ROS_INFO("estim.ext.force: %f", get_estimated_external_force_srv.response.external_force.force.z);
  ROS_INFO("req.ObjectForce: %f", gravityZ*(req.ObjectMass));

  if(get_estimated_external_force_srv.response.external_force.force.z <= 0.8*(gravityZ*(req.ObjectMass))
      && get_estimated_external_force_srv.response.external_force.force.z >= 2.0*(gravityZ*(req.ObjectMass)))
  {
    ROS_INFO("Object picked up successfully!");

    set_object_load_srv_.request.center_of_gravity = req.CentreOfMass;
    set_object_load_srv_.request.mass = req.ObjectMass ;
    // Call SetObjectLoad
    set_object_load_client_.call(set_object_load_srv_);

    if(!set_object_load_srv_.response.error_message.empty())
    {
      ROS_ERROR("Failed to set object load!!");
      res.GotObject = false;
      return false;
    }

    res.GotObject = true;
    return true;
  }

  return false;
}
