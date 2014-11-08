#include <state_observer.hpp>

//standard force limit factor
static const double std_force_limit = 0.5;
//maximum force limit factor
static const double max_force_limit = 0.9;
//increment to increase the limit factor
static const double delta_limit = 0.2;

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
	force_limits_max[7] = 10;  // 200 max_gripper_force


	for(int i = 0; i < 8; i++) // Initialise the limits to 100 Nm; SetJointTorqueLimits() will have to be called from main
	{
		max_torques[i] = 0.0;
		security[i] = std_force_limit;  // Default security limit sets all force limits to 10 Nm
		force_limits[i] = force_limits_max[i]*security[i]; //TODO limits from urdf file? state_observer/src/am_robot_model/kuka_lwr/kuka_lwr_mod.urdf (rosinterface.cpp)
	}                          //-> Array erstellen (StopConditionWidget.cpp)

}

StateObserver::~StateObserver()
{

}

bool StateObserver::check_state()
{
	//	counter++;
	//	uint16_t joint_num = 0;
	//
	//	  //Get telemetry message to compare torques
	//	  if (!getTelemetry())
	//	  {
	//	    msg_error("getTelemetry: An Error happened here.");
	//	  }
	//
	////	if(counter%20 == 0) //Rate of messages shown
	////	{
	//
	//	  while(joint_num < 7)
	//	  {
	//		  //		get_estimated_external_force_client_.call(get_estimated_external_force_srv);
	//
	//
	//		  if(stamp_!=0) // If time stamp is not equal to nought, total torque will be compared with the limit
	//		  {
	//			  std_msgs::Bool msg;
	//			  sim_time_sec_ = (int)_telemetry.header.stamp.sec;
	//			  sim_time_nsec_ = (int)_telemetry.header.stamp.nsec;
	//
	//			  if(abs(_telemetry.measured.external_torque[joint_num +2]) > max_torques[joint_num])
	//			  {
	//				  max_torques[joint_num] = _telemetry.measured.external_torque[joint_num +2];
	//			  }
	//
	//			  if((_telemetry.measured.external_torque[joint_num +2] > force_limits[joint_num])
	//					  ||(_telemetry.measured.external_torque[joint_num +2] < -force_limits[joint_num]))
	//			  {
	//				  msg.data=true;  // msg.data = true when limit has been exceeded
	//				  try
	//				  {
	////					  stop_pub.publish(msg);
	////					  msg_warn("Published stopping message! Joint %d external torque: %f, limit: %f at sim. time %d.%d",
	////							  (joint_num +1), _telemetry.measured.external_torque[joint_num +2], force_limits[joint_num],
	////							  sim_time_sec_, sim_time_nsec_);
	////					  //for(int i =0; i < 7; i++)
	////					  {
	////						  ROS_INFO("joint %d maximum torque: %f", joint_num, max_torques[joint_num]);
	////					  }
	//				  }
	//				  catch(...)
	//				  {
	//					  msg_error("failed to publish stop message");
	//					  return false;
	//				  }
	//			  }
	//		  }
	//		  joint_num++;
	//
	//	  }
	//
	////	}

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
		msg_error("Current force/torque value exceeds maximum limit at joint %d!", (req.joint_nbr +1));
	}

	// Set new force limits
	if (req.level == 1)
	{
		//    security[req.joint_nbr] = force_limits_max[req.joint_nbr]/(1.1*force_limits[req.joint_nbr]);  //req.joint_nbr -> take setting security level out of while loop?
		security[req.joint_nbr] += delta_limit;

		if(security[req.joint_nbr]>max_force_limit)
		{
			security[req.joint_nbr]=max_force_limit;
			res.limit_reached=true;
		}
		force_limits[req.joint_nbr] = security[req.joint_nbr]*force_limits_max[req.joint_nbr];  // req.level
	}
	else  // force limits set back to default
	{
		ROS_INFO("Force limits reset");
		for(uint16_t ii = 0; ii < lwr_nr ; ii++)
		{
			security[ii] = std_force_limit;
			force_limits[ii] = security[ii]*force_limits_max[ii];  // req.level
		}
	}


	//old version:
	//    if(force_limits[req.joint_nbr] > 0.9*(force_limits_max[req.joint_nbr]))  // Safely limit of 5 Nm
	//    {
	//      msg_warn("Current force/torque approaching the maximum limit at joint %d!", (req.joint_nbr +1));
	//      security[req.joint_nbr] = force_limits_max[req.joint_nbr]/10.0;  // Default security limit sets all force limits to 10 Nm
	//      force_limits[req.joint_nbr] = force_limits_max[req.joint_nbr]/security[req.joint_nbr];
	//      force_limits[req.joint_nbr] = force_limits_max[req.joint_nbr]*0.8;
	//    }

	// Populate a vector with all the lwr joint names + gripper
	for(unsigned int i = 0; i < lwr_nr; ++i)
	{
		name.str("lwr_joint_");
		name.seekp(0, std::ios_base::end);
		name << (i +1);   // Simulation joints start from joint 0!! It was (i + 1) before...
		lwr_names[i] = name.str();
	}
	// lwr_names[7] = "gripper";

	if (ros::service::waitForService(set_stop_conditions_,ros::Duration(1.0)))
	{
		std::vector<euroc_c2_msgs::StopCondition> stop_conditions;
		stop_conditions.resize(lwr_nr);
		for (int i = 0 ; i < lwr_nr; i++)
		{
			//      if (lwr_names[i].compare("gripper") == 0)  //string::compare returns 0 when compared strings are identical
			//      {
			//        stop_conditions[i].condition_type = "tool_force_threshold";
			//      }
			//      else
			//      {
			stop_conditions[i].condition_type = "joint_ext_torque_threshold";
			//      }
			// TODO Stop condition operators set --> Is there any point using other conditions ("|<", "<", ">")?
			stop_conditions[i].condition_operator = "|>";
			stop_conditions[i].joint_name = lwr_names[i];
			stop_conditions[i].threshold = force_limits[i];

			//Force direction geometry_mgs::Vector3 --> Set to 0 for absolute force/torque values
			stop_conditions[i].force_direction.x = 0.0;
			stop_conditions[i].force_direction.y = 0.0;
			stop_conditions[i].force_direction.z = 0.0;
		}

		if (req.level == 1)
		{
			for(int i = 0; i < lwr_nr; i++)
			{
				// Testing
				ROS_INFO("condition type: %s, joint name: %s, threshold: %f", stop_conditions[i].condition_type.c_str(),
						stop_conditions[i].joint_name.c_str(), stop_conditions[i].threshold);
			}
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
	if(&msg)
	{
		_telemetry = msg;
		stamp_ = ros::Time::now().toNSec();
	}
	else
		msg_error("invalid message received!");
}

// ROS-Service Function
bool StateObserver::ReturnObjectPickedUp(am_msgs::ObjectPickedUp::Request &req, am_msgs::ObjectPickedUp::Response &res)
{
	float gravityZ = 9.81;
	ROS_INFO("ObjectPickedUp service called");

	if(!(get_estimated_external_force_client_.call(get_estimated_external_force_srv)))
	{
		get_estimated_external_force_srv.response.error_message = "Failed to call client to get external force";
		msg_error("%s", get_estimated_external_force_srv.response.error_message.c_str());
		return false;
	}

	ROS_INFO("estim.ExternalForce: %f", get_estimated_external_force_srv.response.external_force.force.z);
	ROS_INFO("req.ObjectWeight: %f", -gravityZ*(req.ObjectMass));
	//ROS_INFO("estim. Gripper width: %f",_telemetry.measured.position[_telemetry.measured.position.size() - 3]);

	if (!getTelemetry())
	{
		msg_error("Unable to retrieve telemetry message (ObjectPickedUp service).");
		return false;
	}
	else
	{
		ROS_INFO("Measured gripper jaw position: %f", _telemetry.measured.position[_telemetry.measured.position.size() - 3]);
		ROS_INFO("Commanded gripper jaw position: %f", _telemetry.commanded.position[_telemetry.commanded.position.size() - 3]);
		// if gripper jaws are open (one can suppose there is an object between the jaws)
		// position verctor size - 3 because gripper position value is given by the third last entry
		// set gripper to close fully: Statemachine::gripper_close() -> main_gripper_interface::gripper_interface(...)
		if (_telemetry.measured.position[_telemetry.measured.position.size() - 3] >
		abs(_telemetry.commanded.position[_telemetry.commanded.position.size() - 3]+0.004) &&
		_telemetry.measured.position[_telemetry.measured.position.size() - 3] > 0.003)
		{

			//old version:
			//      if(get_estimated_external_force_srv.response.external_force.force.z <= 0.2*(gravityZ*(req.ObjectMass))
			//          && get_estimated_external_force_srv.response.external_force.force.z >= -0.8*(gravityZ*(req.ObjectMass)))

			//new version:
			//ideal (with set object load set correct!): estimated external force = 0!
			//test if estim. ext. force is in [-0.1*f_set, 0.1*f_set]
//			if(get_estimated_external_force_srv.response.external_force.force.z <= 0.5*(gravityZ*(req.ObjectMass))
//					&& get_estimated_external_force_srv.response.external_force.force.z >= -0.5*(gravityZ*(req.ObjectMass)))
//			{
				ROS_INFO("Object picked up successfully!");
				res.GotObject = true;
				return true;
//			}
		}

		//return true;
	}

	msg_warn("The downward force and object mass do not match!");

	res.GotObject = false;
	return true; // Service works, object was not picked up
}
