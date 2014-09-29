#include <state_observer.hpp>


StateObserver::StateObserver()
{
	counter = 0;
	stamp_ = 0;

	stop_pub = nh.advertise<std_msgs::Bool>("stop", 1);

        euroc_c2_interface_ = "/euroc_interface_node";
        telemetry_ = euroc_c2_interface_ + "/telemetry";
        estimated_external_force_ = euroc_c2_interface_ + "/get_estimated_external_force";
        set_stop_conditions_ = euroc_c2_interface_ + "/set_stop_conditions";
        set_object_load_ = euroc_c2_interface_ + "/set_object_load";

	telemetry_subscriber_= nh.subscribe(telemetry_, 1, &StateObserver::callback, this); // this to point at the object (&so) itself
	get_estimated_external_force_client_ = nh.serviceClient<euroc_c2_msgs::GetEstimatedExternalForce>(estimated_external_force_);
	set_object_load_client_ = nh.serviceClient<euroc_c2_msgs::SetObjectLoad>(set_object_load_);
	set_stop_conditions_client_ = nh.serviceClient<euroc_c2_msgs::SetStopConditions>(set_stop_conditions_);
	object_picked_up_srv = nh.advertiseService("ObjectPickedUp_srv", &StateObserver::ReturnObjectPickedUp, this);
	call_set_stop_conditions_srv = nh.advertiseService("CallSetStopConditions_srv", &StateObserver::CallSetStopCondition, this);

//	current_configuration.q.resize(7); // Resizing the current_conf vector to fit all joints

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
	                  security[i] = 0.95;  // Default security limit sets all force limits to 10 Nm
	                  force_limits[i] = force_limits_max[i]*0.6; //TODO limits from urdf file? state_observer/src/am_robot_model/kuka_lwr/kuka_lwr_mod.urdf (rosinterface.cpp)
	                }                          //-> Array erstellen (StopConditionWidget.cpp)
}

StateObserver::~StateObserver()
{

}

bool StateObserver::check_state()
{
	counter++;
	uint16_t joint_num = 0;

	  //Get telemetry message to compare torques
	  if (!getTelemetry())
	  {
	    msg_error("getTelemetry: An Error happened here.");
	  }

	if(counter%20 == 0) //Rate of messages shown
	{

	  while(joint_num < 8)
	  {
//		get_estimated_external_force_client_.call(get_estimated_external_force_srv);


		if(stamp_!=0) // If time stamp is not equal to nought, total torque will be compared with the limit
		{
		    std_msgs::Bool msg;

		    if((_telemetry.measured.external_torque[joint_num +2] > force_limits[joint_num])
		        ||(_telemetry.measured.external_torque[joint_num +2] < -force_limits[joint_num]))
		    {
			msg.data=true;  // msg.data = true when limit has been exceeded
			try
			{
			    stop_pub.publish(msg);
			    msg_warn("Published stopping message! Joint %d external torque: %f, limit: %f",(joint_num +1), _telemetry.measured.external_torque[joint_num +2], force_limits[joint_num]);
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

bool StateObserver::CallSetStopCondition(am_msgs::CallSetStopConditions::Request &req, am_msgs::CallSetStopConditions::Response &res)
{
  const unsigned int lwr_nr = 7;  // TODO auf 8 setzen
  std::vector<std::string> lwr_names(lwr_nr);
  std::stringstream name;

  ROS_INFO("CallSetStopCondition service called");

  // Check estimated external forces at gripper
  if(!(get_estimated_external_force_client_.call(get_estimated_external_force_srv)))
      {
        get_estimated_external_force_srv.response.error_message = "Failed to call client to get external force";
        msg_error("Failed to call client to get external force");
        return false;
      }

  if(req.current_value >= force_limits_max[req.joint_nbr] || req.current_value <= -force_limits_max[req.joint_nbr])
  {
    msg_warn("Current force/torque exceeds maximum limit at joint %d!", (req.joint_nbr +1));
  }
  // Set new force limits
    if (req.level == 1)
    {
//    security[req.joint_nbr] = force_limits_max[req.joint_nbr]/(1.1*force_limits[req.joint_nbr]);  //req.joint_nbr -> take setting security level out of while loop?
      force_limits[req.joint_nbr] = 1.3*force_limits[req.joint_nbr];  // req.level
    }
//    force_limits[req.joint_nbr] = force_limits_max[req.joint_nbr]/security[req.joint_nbr];  // req.level

    if(force_limits[req.joint_nbr] > 0.95*(force_limits_max[req.joint_nbr]))  // Safely limit of 5 Nm
    {
      msg_warn("Current force/torque approaching the maximum limit at joint %d!", (req.joint_nbr +1));
//      security[req.joint_nbr] = force_limits_max[req.joint_nbr]/10.0;  // Default security limit sets all force limits to 10 Nm
//      force_limits[req.joint_nbr] = force_limits_max[req.joint_nbr]/security[req.joint_nbr];
      force_limits[req.joint_nbr] = force_limits_max[req.joint_nbr]*0.8;
    }

  // Populate a vector with all the lwr joint names + gripper
  for(unsigned int i = 0; i < lwr_nr; ++i)
  {
    name.str("lwr_joint_");
    name.seekp(0, std::ios_base::end);
    name << (i +1);   // Simulation joints start from joint 0!! It was (i + 1) before...
    lwr_names[i] = name.str();
  }
 //lwr_names[7] = "gripper";

  if (ros::service::waitForService(set_stop_conditions_,ros::Duration(1.0)))
  {
    std::vector<euroc_c2_msgs::StopCondition> stop_conditions;
    stop_conditions.resize(lwr_nr);
    for (int i = 0 ; i < lwr_nr; i++)
    {
      if (lwr_names[i].compare("gripper") == 0)  //string::compare returns 0 when compared strings are identical
      {
        stop_conditions[i].condition_type = "tool_force_threshold";
      }
      else
      {
        stop_conditions[i].condition_type = "joint_ext_torque_threshold";
      }
      // Stop condition operators have to be set!!
      stop_conditions[i].condition_operator = "|>";
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
      msg_warn("Stop conditions successfully set by state_observer service");
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
      return true;
    }
  }
  else
  {
    ROS_WARN("Set stop conditions service has not been advertised yet.");
  }

  return false;
}

bool StateObserver::getTelemetry()
{
  _telemetry = *(ros::topic::waitForMessage<euroc_c2_msgs::Telemetry>(telemetry_,ros::Duration(10.0)));
  if (&_telemetry==NULL)
  {
    msg_warn("No telemetry message received.");
    return false;
  }
  else
    return true;
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

// ROS-Service Function
bool StateObserver::ReturnObjectPickedUp(am_msgs::ObjectPickedUp::Request &req, am_msgs::ObjectPickedUp::Response &res)
{
  float gravityZ = 9.8;
  ROS_INFO("ObjectPickedUp service called");

//  set_object_load_srv_.request.center_of_gravity = req.CentreOfMass;
//  set_object_load_srv_.request.mass = 0.0;
//  if(!set_object_load_client_.call(set_object_load_srv_))
//      {
//        set_object_load_srv_.response.error_message = "Failed to call client to set object load";
//        msg_error("Failed to call client to set object load");
//        return false;
//      }
//
//  boost::this_thread::sleep (boost::posix_time::milliseconds(2000));

  if(!(get_estimated_external_force_client_.call(get_estimated_external_force_srv)))
      {
        get_estimated_external_force_srv.response.error_message = "Failed to call client to get external force";
        msg_error("Failed to call client to get external force");
        return false;
      }

  ROS_INFO("estim.ExternalForce: %f", get_estimated_external_force_srv.response.external_force.force.z);
  ROS_INFO("req.ObjectWeight: %f", -gravityZ*(req.ObjectMass));

  if(get_estimated_external_force_srv.response.external_force.force.z <= 0.2*(gravityZ*(req.ObjectMass))
      && get_estimated_external_force_srv.response.external_force.force.z >= -0.8*(gravityZ*(req.ObjectMass)))
  {
    ROS_INFO("Object picked up successfully!");

//    set_object_load_srv_.request.center_of_gravity = req.CentreOfMass;
//    set_object_load_srv_.request.mass = req.ObjectMass ;

//    // Call SetObjectLoad
//    if(!set_object_load_client_.call(set_object_load_srv_))
//    {
//      set_object_load_srv_.response.error_message = "Failed to call client to set object load";
//      msg_error("Failed to call client to set object load");
//      return false;
//    }
//    if(set_object_load_srv_.response.error_message.empty())
//    {
//      ROS_INFO("Object mass set successfully.");
//    }
//    else
//    {
//      ROS_ERROR("Failed to set object load!!");
//      res.GotObject = false;
//      return false;
//    }

    res.GotObject = true;
    return true;
  }

    msg_warn("The downward force and object mass do not match!");

  res.GotObject = false;
  return true; // Service works, object was not picked up
}
