/*
 * VisionDummy.cpp
 *
 *  Created on: Sep 12, 2014
 *      Author: euroc_admin
 */

#include "vision_dummy.hpp"

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

VisionDummy::VisionDummy() {
	// TODO Auto-generated constructor stub

}

VisionDummy::~VisionDummy() {
	// TODO Auto-generated destructor stub
}

void VisionDummy::handle(const am_msgs::VisionGoal::ConstPtr &goal)
{
  ROS_INFO("Entered VisionDummy::handle()...");

  obj_aligned_=false;

  pcl::PointCloud<pcl::PointXYZ>::Ptr targetPC (new pcl::PointCloud<pcl::PointXYZ>());

  if(!goal->object.name.compare("red_cube"))
  {
    std::cout<<"Looking for: red cube"<<std::endl;
    *targetPC += *finalVoxelizedRedPC;
  }
  else if (!goal->object.name.compare("green_cylinder"))
  {
    std::cout<<"Looking for: green cylinder"<<std::endl;
    *targetPC += *finalVoxelizedGreenPC;
  }
  else if (!goal->object.name.compare("blue_handle"))
  {
    std::cout<<"Looking for: blue handle"<<std::endl;
    *targetPC += *finalVoxelizedBluePC;
  }

//  pcl::visualization::CloudViewer viewer ("Cloud Viewer");
//  viewer.showCloud(targetPC);

  pcl::console::print_highlight ("Passing target point cloud to pose estimator...\n");
  ros::Duration(2.0).sleep();

  // publish final thresholded point cloud
  //ROS_INFO("Show point cloud");
  pcl::toROSMsg (*targetPC, msg);
  msg.header.frame_id = "LWR_0";
  msg.header.stamp = ros::Time::now();
  pub.publish (msg);


  // ==========================================================================================
  // ========== Code from Fabian ==========
  // Generate PointClouds from the given obstacles

  //ROS_INFO("Passing the voxelized point cloud to pose estimator...");
  ShapeGenerator<PointNT> shape_generator;
  pcl::PointCloud<PointNT>::Ptr object_model (new pcl::PointCloud<PointNT>);
  pcl::PointCloud<PointNT>::Ptr shape_model (new pcl::PointCloud<PointNT>);
  float step_size = 0.005;
  Eigen::Quaternion<double> q;
  Eigen::Vector3d translation;
  Eigen::Matrix4f transformation;
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  nest.setRadiusSearch (0.02);

  shape_generator.setOutputCloud(shape_model);

  for (int i=0; i < goal->object.nr_shapes; i++)
  {
    if (!goal->object.shape[i].type.compare("cylinder"))
    {
      // generate Cylinder PC (PointCloud) closed on the top and bottom side
      step_size = 0.01;
      shape_generator.generateCylinder(step_size, Eigen::Vector3f(0.0f, 0.0f, 0.0f), goal->object.shape[i].length * Eigen::Vector3f::UnitZ(), goal->object.shape[i].radius);
      shape_generator.generateCirclePlane (step_size, Eigen::Vector3f (0.0f, 0.0f, 0.0f), Eigen::Vector3f::UnitZ(), goal->object.shape[i].radius);
      shape_generator.generateCirclePlane (step_size, Eigen::Vector3f (0.0f, 0.0f, goal->object.shape[i].length), Eigen::Vector3f::UnitZ(), goal->object.shape[i].radius);

      //transform Cylinder PC to valid position
      q = Eigen::Quaternion<double>(goal->object.shape[i].pose.orientation.w, goal->object.shape[i].pose.orientation.x, goal->object.shape[i].pose.orientation.y, goal->object.shape[i].pose.orientation.z);
      translation[0] = goal->object.shape[i].pose.position.x;
      translation[1] = goal->object.shape[i].pose.position.y;
      translation[2] = -(goal->object.shape[i].length/2) + goal->object.shape[i].pose.position.z;

      pcl::transformPointCloud(*shape_model, *shape_model, translation, q);

      // Estimate normals for scene
      pcl::console::print_highlight ("Estimating cylinder normals...\n");
      nest.setInputCloud (shape_model);
      nest.compute (*shape_model);

      *object_model += *shape_model;

      shape_model->clear();
    }

    else if(!goal->object.shape[i].type.compare("box"))
    {
      //generate box PC (PointCloud)
      step_size = 0.01;
      shape_generator.generateBox(step_size, Eigen::Vector3f(0.0f, 0.0f, 0.0f), goal->object.shape[i].size[0] * Eigen::Vector3f::UnitX(), goal->object.shape[i].size[1] * Eigen::Vector3f::UnitY(), goal->object.shape[i].size[2] * Eigen::Vector3f::UnitZ(), 0.1f);

      //transform Cylinder PC to valid position
      q = Eigen::Quaternion<double>(goal->object.shape[i].pose.orientation.w, goal->object.shape[i].pose.orientation.x, goal->object.shape[i].pose.orientation.y, goal->object.shape[i].pose.orientation.z);
      translation[0] = -(goal->object.shape[i].size[0]/2) + goal->object.shape[i].pose.position.x;
      translation[1] = -(goal->object.shape[i].size[1]/2) + goal->object.shape[i].pose.position.y;
      translation[2] = -(goal->object.shape[i].size[2]/2) + goal->object.shape[i].pose.position.z;

      pcl::transformPointCloud(*shape_model, *shape_model, translation, q);

      // Estimate normals for scene
      pcl::console::print_highlight ("Estimating box normals...\n");
      nest.setInputCloud (shape_model);
      nest.compute (*shape_model);

      *object_model += *shape_model;

      shape_model->clear();
    }

  }

  // Align observed point cloud with modeled object
  transformation = align_PointClouds(object_model, targetPC);
  if(obj_aligned_==false)
    {
      vision_server_.setPreempted(vision_result_,"Alignment failed.");
    }

  //Publish aligned PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr test (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*object_model, *test);
  pcl::transformPointCloud(*test, *test, transformation);
  pcl::toROSMsg (*test, msg);
  msg.header.frame_id = "LWR_0";
  msg.header.stamp = ros::Time::now();
  pub_2.publish (msg);

  //get quaternion from rotation-matrix
  tf::Matrix3x3 rotation;
  rotation.setValue((double)transformation(0,0), (double)transformation(0,1), (double)transformation(0,2),
		  	  	  	(double)transformation(1,0), (double)transformation(1,1), (double)transformation(1,2),
		  	  	  	(double)transformation(2,0), (double)transformation(2,1), (double)transformation(2,2));

  tf::Quaternion tfqt;
  rotation.getRotation(tfqt);
  // END code from Fabian
  // ==========================================================================================

  //-------------------------------- SEND RESULT ----------------------------------------//

  vision_result_.abs_object_pose.position.x = transformation(0,3);
  vision_result_.abs_object_pose.position.y = transformation(1,3);
  vision_result_.abs_object_pose.position.z = transformation(2,3);
  vision_result_.abs_object_pose.orientation.w = tfqt.getW();
  vision_result_.abs_object_pose.orientation.x = tfqt.getX();
  vision_result_.abs_object_pose.orientation.y = tfqt.getY();
  vision_result_.abs_object_pose.orientation.z = tfqt.getZ();



  vision_feedback_.execution_time = ros::Time::now().sec;
  vision_server_.publishFeedback(vision_feedback_);


  if(failed)
  {
    vision_result_.object_detected=false;
    vision_server_.setPreempted(vision_result_,"Got no telemetry.");
  }
  else
  {
    vision_result_.object_detected=true;
    vision_server_.setSucceeded(vision_result_, "Goal configuration has been reached");
  }

  //-------------------------------- Clean Up ----------------------------------------//
  targetPC->clear();
}
