// C++ demonstration program for the EuRoC C2S1: Inspects all target zones of task 1 with the tcp camera. 

// Copyright (C) 2014 German Aerospace Center (DLR)

// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.'

// Neither the name of the German Aerospace Center (DLR)  nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.

#include <ros/ros.h>

// Includes for simulator startup service calls
#include <euroc_c2_msgs/ListScenes.h>
#include <euroc_c2_msgs/StartSimulator.h>
#include <euroc_c2_msgs/StopSimulator.h>

// Includes for received topic messages
#include <euroc_c2_msgs/Telemetry.h>
#include <sensor_msgs/Image.h>

// Includes for system interaction service calls
#include <euroc_c2_msgs/MoveAlongJointPath.h>
#include <euroc_c2_msgs/GetTimingAlongJointPath.h>
#include <euroc_c2_msgs/GetForwardKinematics.h>
#include <euroc_c2_msgs/SearchIkSolution.h>
#include <euroc_c2_msgs/SetStopConditions.h>
#include <euroc_c2_msgs/SaveLog.h>

// Includes for parsing yaml data
#include <yaml-cpp/yaml.h>

// Includes to show opencv image
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

// ROS Callback for the telemetry topic
euroc_c2_msgs::Telemetry _telemetry;
void on_telemetry(const euroc_c2_msgs::Telemetry &telemetry){
  _telemetry = telemetry;
}

//ROS Callback for the tcp rgb camera topic
sensor_msgs::Image _tcp_rgb_image;
void on_camera_tcp_rgb(const sensor_msgs::Image &image){
  _tcp_rgb_image = image;
}

int main(int argc, char* argv[]){
  
  //Startup ros
  ros::init(argc, argv, "euroc_c2_demo_node");

  bool single_run = false;
  for(unsigned int i = 1; i < argc; i++){
    std::string arg(argv[i]);
    if(arg == "-single") single_run = true;
  }

  //Get the node handle for this node
  ros::NodeHandle node;

  //Wait for the simulator services and wait for them to be available: 
  const std::string task_selector("/euroc_c2_task_selector");
  const std::string list_scenes = (task_selector + "/list_scenes");
  const std::string start_simulator= (task_selector + "/start_simulator");
  const std::string stop_simulator= (task_selector + "/stop_simulator");
  ros::service::waitForService(list_scenes);
  ros::service::waitForService(start_simulator);
  ros::service::waitForService(stop_simulator);
  ros::ServiceClient list_scenes_client = node.serviceClient<euroc_c2_msgs::ListScenes>(list_scenes);
  ros::ServiceClient start_simulator_client = node.serviceClient<euroc_c2_msgs::StartSimulator>(start_simulator);
  ros::ServiceClient stop_simulator_client = node.serviceClient<euroc_c2_msgs::StopSimulator>(stop_simulator);

  ROS_INFO("Hello, this is the DEMO node");


  /* The list scenes service returns all available scenes / tasks in the EuRoC C2 Simulation Challenge.
     Each scene has a name and a description specified in yaml format.  */
  euroc_c2_msgs::ListScenes list_scenes_srv;
  list_scenes_client.call(list_scenes_srv);

  // The error_message field of each service response indicates whether an error occured. An empty string indicates success
  std::string &ls_error_message = list_scenes_srv.response.error_message;
  if(!ls_error_message.empty()){
    ROS_ERROR("List scenes failed: %s", ls_error_message.c_str());
    return -1;
  }

  // Let's print the names of the received scenes 
  std::cout << "Found the following scenes for the EuRoC C2 Simulation:" << std::endl;
  std::vector<euroc_c2_msgs::Scene> &scenes = list_scenes_srv.response.scenes;
  for(unsigned int i = 0; i < scenes.size(); ++i){
    euroc_c2_msgs::Scene &scene = scenes[i];
    std::cout << " - " << scene.name << std::endl;
  }
  
  // The start simulator service starts the simulation with a specified scene and an user id. We'll select the task 1 scene for now.
  euroc_c2_msgs::StartSimulator start_simulator_srv;
  start_simulator_srv.request.user_id = "demo user";
  start_simulator_srv.request.scene_name = "task1_v1";
  start_simulator_client.call(start_simulator_srv);
  
  // Check the response for errors
  std::string &error_message = start_simulator_srv.response.error_message;
  if(!error_message.empty()){
    ROS_ERROR("Starting the simulator failed failed: %s", error_message.c_str());
    return -1;
  }
  
  // The start simulator service returns a description of the selected task in yaml format
  std::string &task_yaml_description = start_simulator_srv.response.description_yaml;
  
  // Parse the explanation/description of the task from the yaml string
  std::stringstream yaml_stream(task_yaml_description);
  YAML::Parser parser(yaml_stream);
  YAML::Node task_description_node;
  parser.GetNextDocument(task_description_node);
  std::string task_description;
  task_description_node["description"] >> task_description;

  // Print the task description
  std::cout << "The task description for task1_v1 is: " << std::endl << task_description << std::endl;

  // Parse the positions of the target zones:
  const YAML::Node *target_zones = task_description_node.FindValue("target_zones");
  if(!target_zones){
    ROS_ERROR("Target zones not found in task description");
    return -1;
  }
  unsigned int nr_target_zones = target_zones->size();
  std::vector<geometry_msgs::Pose> poses(nr_target_zones);
  unsigned int i = 0;
  for(YAML::Iterator it = target_zones->begin(); it != target_zones->end(); ++it) {
    
    const YAML::Node* zone;
    try{
      zone = &(it.second());
    }catch(YAML::Exception e){
      ROS_ERROR("Zone error");
      return -1;
    }
    const YAML::Node* target_position = zone->FindValue("target_position");
    if(!target_position){
      ROS_ERROR("Target position not found in target zone");
      return -1;
    }
    try{
      geometry_msgs::Point &position = poses[i].position;
      (*target_position)[0] >> position.x;
      (*target_position)[1] >> position.y;
      (*target_position)[2] >> position.z;
    }catch(YAML::Exception e){
      ROS_ERROR("YAML_Exception while parsing target zone position: %s", e.msg.c_str());
      return -1;
    }
    i++;
  }
  
  // After a succesfull start simulator service call the simulation is
  // running. During the development phase you may view the scene in
  // the gazebo viewer (which opens automatically)
    
  /*Now we can subscribe to the euroc_c2_interface topics and services to interact with the system
    For the demo we will use the telemetry, lwr_base_to_tcp and camera topics and the services move_along_joint_path, search_ik_solution
   */

  // Gather all the topic and service names
  const std::string euroc_c2_interface = "/euroc_interface_node";
  const std::string telemetry = euroc_c2_interface + "/telemetry";
  const std::string camera_tcp_rgb = euroc_c2_interface + "/cameras/tcp_rgb_cam";
  const std::string move_along_joint_path = euroc_c2_interface + "/move_along_joint_path";
  const std::string search_ik_solution = euroc_c2_interface + "/search_ik_solution";
  const std::string save_log = euroc_c2_interface + "/save_log";
  
  // Subscribe to the ros topics
  ros::Subscriber telemetry_subscriber = node.subscribe(telemetry, 1, on_telemetry);
  ros::Subscriber camera_tcp_rgb_subscriber = node.subscribe(camera_tcp_rgb, 1, on_camera_tcp_rgb);
  
  // Subscribe to the ros services
  ros::service::waitForService(move_along_joint_path);
  ros::ServiceClient move_along_joint_path_client = node.serviceClient<euroc_c2_msgs::MoveAlongJointPath>(move_along_joint_path);
  ros::service::waitForService(search_ik_solution);
  ros::ServiceClient search_ik_solution_client = node.serviceClient<euroc_c2_msgs::SearchIkSolution>(search_ik_solution);
  ros::service::waitForService(save_log);
  ros::ServiceClient save_log_client = node.serviceClient<euroc_c2_msgs::SaveLog>(save_log);

  // Finalize the desired poses of the tcp to point towards the table at a height of 0.5 meters
  geometry_msgs::Quaternion towards_table;
  towards_table.x = 1.0;
  towards_table.y = 0.0;
  towards_table.z = 0.0;
  towards_table.w = 0.0;
  for(unsigned int i = 0; i < nr_target_zones; ++i){
    poses[i].orientation = towards_table;
    poses[i].position.z = 0.5; 
  }

  // Populate a vector with all the lwr joint names
  const unsigned int nr_lwr_joints = 7;
  std::vector<std::string> lwr_joints(nr_lwr_joints);
  std::stringstream name;
  for(unsigned int i = 0; i < nr_lwr_joints; ++i){
      name.str("lwr_joint_");
      name.seekp(0, std::ios_base::end);
      name << (i + 1);
      lwr_joints[i] = name.str();
    }

  // Initialize the search inverse kinematic solution service
  euroc_c2_msgs::SearchIkSolution search_ik_solution_srv;

  // Initialize the move along joint path service
  euroc_c2_msgs::MoveAlongJointPath move_along_joint_path_srv;
  move_along_joint_path_srv.request.joint_names = lwr_joints; // Select all lwr joints
  move_along_joint_path_srv.request.path.resize(1); // Our path has only one waypoint
  // Initialize the velocity and acceleration limits of the joints
  move_along_joint_path_srv.request.joint_limits.resize(nr_lwr_joints); 
  for(unsigned int i = 0; i < nr_lwr_joints; ++i){
    euroc_c2_msgs::Limits &limits = move_along_joint_path_srv.request.joint_limits[i];
    limits.max_velocity = 20 * M_PI / 180.0; // 20 degrees per second
    limits.max_acceleration = 400 * M_PI / 180.0;
  }

  // current_configuration will hold our current joint position data extracted from the measured telemetry
  euroc_c2_msgs::Configuration current_configuration;
  current_configuration.q.resize(nr_lwr_joints);

  // Wait for the subscribed topics to be published by the euroc simulator to avoid race conditions
  std::cout << "Waiting for topics to be published..." << std::endl;
  ros::topic::waitForMessage<euroc_c2_msgs::Telemetry>(telemetry);
  ros::topic::waitForMessage<sensor_msgs::Image>(camera_tcp_rgb);
  std::cout << "have topics." << std::endl;
  
  // Initialize an open cv window which we will use to display the rgb image mounted at the tip of the LWR 
  const std::string cv_window_name = "LWR tcp camera";
  cv::namedWindow(cv_window_name);
  cv::moveWindow(cv_window_name, 0, 0);

  // Pose index will loop through the poses of the target zones 
  unsigned int pose_index = 0;

  // Main loop: In each iteration we move the LWR towards the next target zone and display the image of the tcp camera 
  while(node.ok()){
    // The spinOnce fuction of ros will execute the callbacks of our subscribed topics once. 
    // This will update our global _telemetry message.
    ros::spinOnce();
    
    // Get the current configuration from the telemetry message
    for(unsigned int i = 0; i < nr_lwr_joints; ++i){
      std::vector<std::string> &joint_names = _telemetry.joint_names;
      unsigned int telemetry_index = std::find(joint_names.begin(), joint_names.end(), lwr_joints[i]) - joint_names.begin();
      current_configuration.q[i] = _telemetry.measured.position[telemetry_index];
    }

    // Select the next desired position of the tcp from the target zone poses and fill 
    // the search inverse kinematic solution request with the current configuration as
    // start configuration and the desired position
    geometry_msgs::Pose &desired_pose = poses[pose_index];
    search_ik_solution_srv.request.start = current_configuration;
    search_ik_solution_srv.request.tcp_frame = desired_pose;

    // Call the search inverse kinematic solution service and check for errors
    search_ik_solution_client.call(search_ik_solution_srv);
    std::string &search_error_message = search_ik_solution_srv.response.error_message;
    if(!search_error_message.empty()){
      ROS_ERROR("Search IK Solution failed: %s", search_error_message.c_str());
      return -1;
    }
    
    // Extract the solution configuration from the response and fill it into the path of the move request
    euroc_c2_msgs::Configuration &solution_configuration = search_ik_solution_srv.response.solution;
    std::vector<euroc_c2_msgs::Configuration> &path = move_along_joint_path_srv.request.path;
    path[0] = solution_configuration;

    // Call the move request and check for errors
    std::cout << "calling move_along_joint_path towards next target_zone" << std::endl;
    move_along_joint_path_client.call(move_along_joint_path_srv);
    std::string &move_error_message = move_along_joint_path_srv.response.error_message;
    if(!move_error_message.empty()){
      std::cout << "Move failed: " + move_error_message << std::endl;
      return 0;
    }

    // Execute the callbacks to receive the newest image
    ros::spinOnce();

    // Convert the image to open cv format and display it in the window
    cv_bridge::CvImagePtr cv_image;
    try{
      cv_image = cv_bridge::toCvCopy(_tcp_rgb_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return -1;
    }
    cv::imshow(cv_window_name, cv_image->image);
    cv::waitKey(250);

    // If -single option activated only loop once through poses
    if(single_run && (pose_index == poses.size() - 1)){
      break;
    }
    
    // Update the pose index towards the next pose
    pose_index = (pose_index + 1) % poses.size();
  }
  
  // Destroy the cv window
  cv::destroyWindow(cv_window_name);
  
  //  When shutting down the simulator early you need to save the log manually with the save_log service
  euroc_c2_msgs::SaveLog save_log_srv;
  std::cout << "saving log ..." << std::endl;
  save_log_client.call(save_log_srv);

  std::cout << "Stopping server ..." << std::endl;

  // The stop simulator callback ends the current simulation
  euroc_c2_msgs::StopSimulator stop_simulator_srv;
  stop_simulator_client.call(stop_simulator_srv);
  
  return 0;
}
