#include "vision.hpp"
#include <config.hpp>
#include <math.h>
#include <ImageFilter.cpp>
#include <ampointcloud.cpp>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <cxcore.h>
#include <highgui.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>


// For creating octomap with whole scene (with robot and pantilt); also the man outside the table can be seen; every sweep; takes too much time when using every sweep
// #define OCTOMAP_COMPLETE_OLD

// For creating octomap without robot and things outside the table; every sweep; time ok
// #define OCTOMAP_REDUCED_OLD

// Output Octomap binary file
// #define OUTPUT_OCTOMAP_BIN

// Saving object and scene PointCloud in pcd file
// #define POINTCLOUD_OUTPUT

// Use octomap_server for creating octomap
#define OCTOMAP_SERVER


namespace enc = sensor_msgs::image_encodings;
using namespace cv;

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<pcl::PointXYZ,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;


Vision::Vision():
                      vision_server_(nh_, "VisionAction", boost::bind(&Vision::handle, this, _1),false),
                      vision_action_name_("VisionAction"),
                      obj_aligned_(false)
{

  failed = false;
  isSingleCube = false;

  euroc_c2_interface = "/euroc_interface_node";
  camera_scene_rgb_topic = euroc_c2_interface + "/cameras/scene_rgb_cam";
  camera_scene_depth_topic = euroc_c2_interface + "/cameras/scene_depth_cam";
  camera_tcp_rgb_topic = euroc_c2_interface + "/cameras/tcp_rgb_cam";
  camera_tcp_depth_topic = euroc_c2_interface + "/cameras/tcp_depth_cam";
  save_log = euroc_c2_interface + "/save_log";

  /*
   * Subscribe to ROS topic: camera
   */
  // Scene_RGB
  camera_scene_rgb_subscriber = nh_.subscribe(camera_scene_rgb_topic, 1, &Vision::on_camera_scene_rgb_CB, this);
  // Scene_Depth
  camera_scene_depth_subscriber = nh_.subscribe(camera_scene_depth_topic, 1, &Vision::on_camera_scene_depth_CB, this);
  // TCP_RGB
  camera_tcp_rgb_subscriber = nh_.subscribe(camera_tcp_rgb_topic, 1, &Vision::on_camera_tcp_rgb_CB, this);
  // TCP_DEPTH
  camera_tcp_depth_subscriber = nh_.subscribe(camera_tcp_depth_topic, 1, &Vision::on_camera_tcp_depth_CB, this);

  // Take Image Service
  take_img_service_ = nh_.advertiseService("TakeImageService", &Vision::on_take_image_CB,this);

  // Show PointCloud in rviz
  pub = nh_.advertise<sensor_msgs::PointCloud2> ("PointCloud_1", 1);
  pub_2 = nh_.advertise<sensor_msgs::PointCloud2> ("PointCloud_2", 1);
  pub_3 = nh_.advertise<sensor_msgs::PointCloud2> ("/PointCloud_Octomap", 1);

  // Initialize finalScenePC Pointcloud
  finalScenePC.reset (new pcl::PointCloud<pcl::PointXYZ> ());
  finalTcpPC.reset (new pcl::PointCloud<pcl::PointXYZ> ());
  finalBluePC.reset (new pcl::PointCloud<pcl::PointXYZ>());
  finalGreenPC.reset (new pcl::PointCloud<pcl::PointXYZ>());
  finalRedPC.reset (new pcl::PointCloud<pcl::PointXYZ>());
  finalYellowPC.reset (new pcl::PointCloud<pcl::PointXYZ>());
  finalCyanPC.reset (new pcl::PointCloud<pcl::PointXYZ>());
  finalMagentaPC.reset (new pcl::PointCloud<pcl::PointXYZ>());

  finalVoxelizedPC.reset (new pcl::PointCloud<pcl::PointXYZ>());
  finalVoxelizedBluePC.reset (new pcl::PointCloud<pcl::PointXYZ>());
  finalVoxelizedGreenPC.reset (new pcl::PointCloud<pcl::PointXYZ>());
  finalVoxelizedRedPC.reset (new pcl::PointCloud<pcl::PointXYZ>());
  finalVoxelizedYellowPC.reset (new pcl::PointCloud<pcl::PointXYZ>());
  finalVoxelizedCyanPC.reset (new pcl::PointCloud<pcl::PointXYZ>());
  finalVoxelizedMagentaPC.reset (new pcl::PointCloud<pcl::PointXYZ>());

  object_model.reset (new pcl::PointCloud<pcl::PointXYZ>);

  // leaf size voxels
  leaf_size = 0.0025;

  // Create and set resolution of Octree
  tree = new octomap::OcTree(0.005);

  // Create octomap::PointCloud
  OctoCloud = new octomap::Pointcloud();

  vision_server_.start();
  ROS_INFO("vision action server started.");

}

void Vision::reset(const std_msgs::BoolConstPtr& rst)
{
//	vision_server_.shutdown();
//	ROS_INFO("vision action server shutdown.");
//	vision_server_.start();
//	ROS_INFO("vision action server started.");
}


/*
 * CALLBACKS
 */
//ROS Callback for the scene RGB camera topic
void Vision::on_camera_scene_rgb_CB(const sensor_msgs::Image &image){
  //ROS_INFO("Entered Vision::RGB_CB");
  //std::cout<<"Height: "<<image.height<<"Width: "<<image.width<<std::endl;
  _scene_rgb_image = image;
}

//ROS Callback for the scene DEPTH camera topic
void Vision::on_camera_scene_depth_CB(const sensor_msgs::Image &image){
  //ROS_INFO("Entered Vision::DEPTH_CB");
  //std::cout<<"Height: "<<image.height<<"Width: "<<image.width<<std::endl;
  _scene_depth_image = image;
}

//ROS Callback for the tcp RGB camera topic
void Vision::on_camera_tcp_rgb_CB(const sensor_msgs::Image &image){
  //ROS_INFO("Entered Vision::RGB_CB");
  //std::cout<<"Height: "<<image.height<<"Width: "<<image.width<<std::endl;
  _tcp_rgb_image = image;
}

//ROS Callback for the tcp DEPTH camera topic
void Vision::on_camera_tcp_depth_CB(const sensor_msgs::Image &image){
  //ROS_INFO("Entered Vision::DEPTH_CB");
  //std::cout<<"Height: "<<image.height<<"Width: "<<image.width<<std::endl;
  _tcp_depth_image = image;
}


bool Vision::on_take_image_CB(am_msgs::TakeImage::Request &req, am_msgs::TakeImage::Response &res)
{
//  // Get the task number
//  std::string key;
//  if(nh_.searchParam("active_task_number_", key))
//  {
//    nh_.getParam(key, task_nr);
//    std::cout<<"[VISION]active task number: "<<task_nr<<std::endl;
//  }
//  else
//    ROS_WARN("search for parameter: active_task_number_ failed!");

  switch (req.camera) {
    case SCENE_CAM:
      std::cout<<"[VISION]callback: SCENE CAM!"<<std::endl;
      std::cout<<"[VISION]SCENE Pointcloud updated."<<std::endl;
      scan_with_pan_tilt(res);
      break;
    case TCP_CAM:
      std::cout<<"[VISION]callback: TCP CAM!"<<std::endl;
      ros::Duration(1.0).sleep();
      scan_with_tcp(res);
      std::cout<<"[VISION]TCP Pointcloud updated"<<std::endl;
      break;
    default:
      ROS_WARN("Unknown camera type!");
      res.error_reason=fsm::VISION_ERROR;
      return false;
  }
  return true;
}

/*
 * END CALLBACKS
 */


void Vision::handle(const am_msgs::VisionGoal::ConstPtr &goal)
{

  // Get the task number
  std::string key;
  if(nh_.searchParam("active_task_number_", key))
  {
    nh_.getParam(key, task_nr);
    std::cout<<"[VISION]active task number: "<<task_nr<<std::endl;
  }
  else
    ROS_WARN("search for parameter failed!");


  if(goal->object.nr_shapes == 1)
    if(!goal->object.shape[0].type.compare("box"))
    {
      std::cout<<"[VISION]current object is a single cube"<<std::endl;
      isSingleCube = true;
    }

  pcl::PointCloud<pcl::PointXYZ>::Ptr targetPC (new pcl::PointCloud<pcl::PointXYZ>());

  if(!goal->object.color.compare("ff0000"))
  {
	  // Goal: Red object
	  *targetPC += *finalVoxelizedRedPC;
  }
  else if (!goal->object.color.compare("00ff00"))
  {
	  // Goal: Green object
	  *targetPC += *finalVoxelizedGreenPC;
  }
  else if (!goal->object.color.compare("0000ff"))
  {
	  // Goal: Blue object
	  *targetPC += *finalVoxelizedBluePC;
  }
  else if (!goal->object.color.compare("00ffff"))
  {
	  // Goal: Cyan object
	  *targetPC += *finalVoxelizedCyanPC;
  }
  else if (!goal->object.color.compare("ff00ff"))
  {
	  // Goal: Magenta object
	  *targetPC += *finalVoxelizedMagentaPC;
  }
  else if (!goal->object.color.compare("ffff00"))
  {
	  // Goal: Yellow object
	  *targetPC += *finalVoxelizedYellowPC;
  }


  if(goal->mode == GLOBAL_POSE_ESTIMATION) {

    std::cout<<"[VISION]Entered Vision::handle()..."<<std::endl;

#ifdef OUTPUT_OCTOMAP_BIN
    tree->writeBinary("/home/euroc_student/EUROC_SVN_STUDENT/vision_octomap/Octomap.bt");
    ROS_INFO("Tree was written to Binary File ... Octomap_World.bt");
#endif

    obj_aligned_=false;

    std::cout<<"[VISION]Looking for: "<<goal->object.name<<std::endl;

    pcl::console::print_highlight ("Passing target point cloud to pose estimator...\n");
    ros::Duration(2.0).sleep();

    // publish final thresholded point cloud
    //    pcl::toROSMsg (*targetPC, msg);
    //    msg.header.frame_id = "LWR_0";
    //    msg.header.stamp = ros::Time::now();
    //    pub.publish (msg);


    // ==========================================================================================
    // ===================================== Code from Fabian ===================================

    // Generate PointClouds from the given obstacles
    ShapeGenerator<pcl::PointXYZ> shape_generator;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr object_model (new pcl::PointCloud<pcl::PointXYZ>);
    object_model->clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr shape_model (new pcl::PointCloud<pcl::PointXYZ>);
    float step_size = 0.005;
    Eigen::Quaternion<double> q;
    Eigen::Vector3d translation;

    shape_generator.setOutputCloud(shape_model);

    for (int i=0; i < goal->object.nr_shapes; i++)
    {
      if (!goal->object.shape[i].type.compare("cylinder"))
      {
        // generate Cylinder PC (PointCloud) closed on the top and bottom side
        step_size = leaf_size;
        shape_generator.generateCylinder(step_size, Eigen::Vector3f(0.0f, 0.0f, 0.0f), goal->object.shape[i].length * Eigen::Vector3f::UnitZ(), goal->object.shape[i].radius);
        if (goal->object.shape.size() < 2)
        {
          shape_generator.generateCirclePlane (step_size, Eigen::Vector3f (0.0f, 0.0f, goal->object.shape[i].length), Eigen::Vector3f::UnitZ(), goal->object.shape[i].radius);
        }

        //transform Cylinder PC to valid position
        q = Eigen::Quaternion<double>(goal->object.shape[i].pose.orientation.w, goal->object.shape[i].pose.orientation.x, goal->object.shape[i].pose.orientation.y, goal->object.shape[i].pose.orientation.z);
        translation[0] = goal->object.shape[i].pose.position.x;
        translation[1] = goal->object.shape[i].pose.position.y;
        translation[2] = -(goal->object.shape[i].length/2) + goal->object.shape[i].pose.position.z;

        pcl::transformPointCloud(*shape_model, *shape_model, translation, q);

        *object_model += *shape_model;

        shape_model->clear();
      }

      else if(!goal->object.shape[i].type.compare("box"))
      {
        //generate box PC (PointCloud)
        step_size = leaf_size;
        if (goal->object.shape.size() == 1)
          shape_generator.generateBox(step_size, Eigen::Vector3f(0.0f, 0.0f, 0.0f), goal->object.shape[i].size[0] * Eigen::Vector3f::UnitX(), goal->object.shape[i].size[1] * Eigen::Vector3f::UnitY(), goal->object.shape[i].size[2] * Eigen::Vector3f::UnitZ(), 0.0f, false);
        else
          shape_generator.generateBox(step_size, Eigen::Vector3f(0.0f, 0.0f, 0.0f), goal->object.shape[i].size[0] * Eigen::Vector3f::UnitX(), goal->object.shape[i].size[1] * Eigen::Vector3f::UnitY(), goal->object.shape[i].size[2] * Eigen::Vector3f::UnitZ(), 0.0f, true);

        //transform Cylinder PC to valid position
        q = Eigen::Quaternion<double>(goal->object.shape[i].pose.orientation.w, goal->object.shape[i].pose.orientation.x, goal->object.shape[i].pose.orientation.y, goal->object.shape[i].pose.orientation.z);
        translation[0] = -(goal->object.shape[i].size[0]/2) + goal->object.shape[i].pose.position.x;
        translation[1] = -(goal->object.shape[i].size[1]/2) + goal->object.shape[i].pose.position.y;
        translation[2] = -(goal->object.shape[i].size[2]/2) + goal->object.shape[i].pose.position.z;

        pcl::transformPointCloud(*shape_model, *shape_model, translation, q);

        *object_model += *shape_model;

        shape_model->clear();
      }
      else
      {
          vision_server_.setPreempted(vision_result_,"Alignment failed.");
    	  vision_result_.error_reason=fsm::SKIP_OBJECT;
    	  return;
      }
    }

#ifdef POINTCLOUD_OUTPUT
    //save PointClouds to file
    pcl::io::savePCDFileASCII (("/home/euroc_student/" + goal->object.name + "_model.pcd"), *object_model);
    pcl::io::savePCDFileASCII (("/home/euroc_student/" + goal->object.name + "_scene.pcd"), *targetPC);
#endif

    // Align observed point cloud with modeled object
    if (goal->object.shape.size() == 1 && !goal->object.shape[0].type.compare("box"))
      transformation = align_PointClouds(object_model, targetPC, true, false);
    else if (goal->object.shape.size() == 1 && !goal->object.shape[0].type.compare("cylinder"))
      transformation = align_PointClouds(object_model, targetPC, false, true);
    else
      transformation = align_PointClouds(object_model, targetPC, false, false);

    if(obj_aligned_==false)
    {
      vision_server_.setPreempted(vision_result_,"Alignment failed.");
	  vision_result_.error_reason=fsm::POSE_NOT_FOUND;
	  return;
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

    // TODO: fix the timestamp problem
    vision_result_.stamp = finalTimeStamp;

    vision_result_.object_detected=true;
    vision_server_.setSucceeded(vision_result_, "Goal configuration has been reached");

    targetPC->clear();
    return;
  }
  else if(goal->mode == CLOSE_RANGE_POSE_ESTIMATION)
  {
    std::cout<<"[VISION]initialize close range pose estimation"<<std::endl;

    // check object type
    if(isSingleCube)
    {
      double OptRotationRadians; // The new optimum value -> cube rotation
      // Wait as usual!!
      std::cout<<"[VISION]Wait for 2 sec..."<<std::endl;
      ros::Duration(2.0).sleep();
      std::cout<<"[VISION]Done!"<<std::endl;

      OptRotationRadians = close_range_pose(goal->object.color);
      if(OptRotationRadians == -5)
      {
        // close range pose estimation failed.
        // possibilities:
        //     - could not find enough corners/lines to compute angle difference
        //     - vision node failed/threw exception
        std::cout<<"[VISION]Could not find a better pose, returning the initial value again"<<std::endl;

        vision_result_.object_detected=false;

        vision_result_.abs_object_pose.position.x = transformation(0,3);
        vision_result_.abs_object_pose.position.y = transformation(1,3);
        vision_result_.abs_object_pose.position.z = transformation(2,3);
        vision_result_.abs_object_pose.orientation.w = tfqt.getW();
        vision_result_.abs_object_pose.orientation.x = tfqt.getX();
        vision_result_.abs_object_pose.orientation.y = tfqt.getY();
        vision_result_.abs_object_pose.orientation.z = tfqt.getZ();
      }
      else
      {
    	double roll,pitch,yaw;
		tf::Matrix3x3 dcm;
		dcm.setRotation(tfqt);
		dcm.getRPY(roll,pitch,yaw);

    	if ( (std::abs(OptRotationRadians-yaw) > M_PI_4) && (std::abs(OptRotationRadians-yaw) < 3*M_PI_4))
    	{
    		OptRotationRadians = OptRotationRadians - ((M_PI_2)*(sgn(OptRotationRadians-yaw)));
    	}

        // New optimum pose found, return the new one to the StateMachine
        tf::Matrix3x3 optRotation;
        std::cout<<"[VISION]rotation around z-axis of table: "<<OptRotationRadians<<std::endl;

        // set the rotation matrix
        optRotation.setValue((double)tfCos((tfScalar)OptRotationRadians), (double)(-1)*tfSin((tfScalar)OptRotationRadians), 0,
                             (double)tfSin((tfScalar)OptRotationRadians), (double)tfCos((tfScalar)OptRotationRadians)	  , 0,
                             0					  				, 			0						  			  , 1);
        // get the quaternion representation from the rotation matrix
        optRotation.getRotation(tfqtNew);

        Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
        transform_1 (0,0) = std::cos(OptRotationRadians);
        transform_1 (0,1) = (-1)*std::sin(OptRotationRadians);
        transform_1 (1,0) = std::sin(OptRotationRadians);
        transform_1 (1,1) = std::cos(OptRotationRadians);
        transform_1 (0,3) = transformation (0,3);
        transform_1 (1,3) = transformation (1,3);
        transform_1 (2,3) = transformation (2,3);

        std::cout<<"[VISION]show aligned point cloud based on close range image"<<std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr closeRangePC (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*object_model, *closeRangePC);
        pcl::transformPointCloud(*closeRangePC, *closeRangePC, transform_1);
        pcl::toROSMsg (*closeRangePC, msg);
        msg.header.frame_id = "LWR_0";
        msg.header.stamp = ros::Time::now();
        pub_2.publish (msg);

        pcl::transformPointCloud(*object_model, *object_model, transformation);

        int diffOldPose = verify_close_range_pose(targetPC, object_model);
        int diffNewPose = verify_close_range_pose(targetPC, closeRangePC);
        std::cout<<"Diff = "<<std::abs( diffOldPose - diffNewPose )<<std::endl;
        // check which transformed model fits better on the observed point cloud
        if ( std::abs( diffOldPose - diffNewPose ) < 15 )
        {
        	std::cout<<"[VISION]new pose is better!"<<std::endl;
        	std::cout<<"[VISION]Passing the new pose to grasping node"<<std::endl;

        	vision_result_.object_detected=true;

        	vision_result_.abs_object_pose.position.x = transformation(0,3);
        	vision_result_.abs_object_pose.position.y = transformation(1,3);
        	vision_result_.abs_object_pose.position.z = transformation(2,3);
        	vision_result_.abs_object_pose.orientation.w = tfqtNew.getW();
        	vision_result_.abs_object_pose.orientation.x = tfqtNew.getX();
        	vision_result_.abs_object_pose.orientation.y = tfqtNew.getY();
        	vision_result_.abs_object_pose.orientation.z = tfqtNew.getZ();
        }
        else
        {
            std::cout<<"[VISION]new pose is a mess!!"<<std::endl;
            std::cout<<"[VISION]Passing the initial estimated pose"<<std::endl;

            vision_result_.object_detected=false;

            vision_result_.abs_object_pose.position.x = transformation(0,3);
            vision_result_.abs_object_pose.position.y = transformation(1,3);
            vision_result_.abs_object_pose.position.z = transformation(2,3);
            vision_result_.abs_object_pose.orientation.w = tfqt.getW();
            vision_result_.abs_object_pose.orientation.x = tfqt.getX();
            vision_result_.abs_object_pose.orientation.y = tfqt.getY();
            vision_result_.abs_object_pose.orientation.z = tfqt.getZ();
        }
      }
    }
    else // for objects other than cube, the inital pose will be sent back
    {

      vision_result_.object_detected=false;

      std::cout<<"[VISION]Object is not a CUBE. No additional pose estimation necessary."<<std::endl;
      vision_result_.abs_object_pose.position.x = transformation(0,3);
      vision_result_.abs_object_pose.position.y = transformation(1,3);
      vision_result_.abs_object_pose.position.z = transformation(2,3);
      vision_result_.abs_object_pose.orientation.w = tfqt.getW();
      vision_result_.abs_object_pose.orientation.x = tfqt.getX();
      vision_result_.abs_object_pose.orientation.y = tfqt.getY();
      vision_result_.abs_object_pose.orientation.z = tfqt.getZ();
    }

    vision_server_.setSucceeded(vision_result_, "Goal configuration has been reached");

  }
  else if (goal->mode == CHECKING_FOR_OBJECT_IN_TARGET_ZONE)
  {
    // Wait as usual!!
    std::cout<<"[VISION]Wait for 4 sec..."<<std::endl;
    ros::Duration(4.0).sleep();
    std::cout<<"[VISION]Done!"<<std::endl;
    std::cout<<"[VISION]check if object is on target zone"<<std::endl;

//    bool isObjectInZone = verify_object_inside_zone(goal->object.color);
    pcl::PointXYZ targetZone;
    targetZone.x = goal->target_zone.position.x;
    targetZone.y = goal->target_zone.position.y;
    targetZone.z = goal->target_zone.position.z;
    int objectInZone = verify_object_inside_zone(goal->object.color, targetZone, goal->target_zone.max_distance);
    // TODO: Revision needed, ask Phillip
    if (objectInZone == 2)
    {
    	std::cout<<"[VISION]Object_On_Target Verification: PASSED."<<std::endl;
    	vision_result_.object_in_zone = true;
    	vision_result_.object_detected = true;
    	vision_server_.setSucceeded(vision_result_, "Goal configuration has been reached");
    }
    else if(objectInZone == 1)
    {
    	std::cout<<"[VISION]Object_On_Target Verification: PASSED."<<std::endl;
    	vision_result_.object_in_zone = false;
    	vision_result_.object_detected = true;
    	vision_server_.setPreempted(vision_result_, "Goal configuration has been reached");
    }
    else
    {
    	std::cout<<"[VISION]Object_On_Target Verification: FAILED!"<<std::endl;
    	std::cout<<"Reason: object not visible in camera view"<<std::endl;

    	vision_result_.object_in_zone = false;
    	vision_result_.object_detected = false;
    	vision_server_.setPreempted(vision_result_, "Goal configuration has been reached");
    }
  }
  else
    msg_warn("unkown mode!");

}

/*
 * This function takes care of generating several point clouds from pan-tilt unit camera's image data.
 * There are a total of 7 point clouds:
 * - Complete (global), Blue, Green, Red, Yellow, Cyan, Magenta
 *
 * 8 fixed configurations has been set for the camera in order get the most complete possible view of the table.
 *
 * All point clouds are voxelized at the end of the process for memory and performance purposes.
 *
 */
void Vision::scan_with_pan_tilt(am_msgs::TakeImage::Response &res)
{

  std::cout<<"[VISION]Entered Vision::scan_with_pan_tilt()..."<<std::endl;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr finalScenePC (new pcl::PointCloud<pcl::PointXYZ> ());
  //msg_info("Waiting for image msgs.");
  if (ros::topic::waitForMessage<sensor_msgs::Image>(camera_scene_rgb_topic, ros::Duration(1.0))
      && ros::topic::waitForMessage<sensor_msgs::Image>(camera_scene_depth_topic, ros::Duration(1.0)))
  {
    MovePantilt* mpt;
    mpt = mpt->get_instance();
    int panTiltCounter = 0;

    // store initial pan-tilt values
    double pan_zero = mpt->get_pan();
    double tilt_zero = mpt->get_tilt();

    // Initialize pan + tilt values
    double pan[8]  = {0.000, -0.300, 0.000, 0.300, 0.000, 0.000, -0.380, 0.380};
    double tilt[8] = {1.250,  0.800, 0.700, 0.800, 0.800, 0.300,  0.300, 0.300};

    std::cout<<"[VISION]Scanning the scene with pan tilt cam..."<<std::endl;
	while(panTiltCounter < 8)
    {

      // temporary variables for each sweep
      am_pointcloud *scenePointCloud;
      pcl::PointCloud<pcl::PointXYZ>::Ptr initialPC;
      pcl::PointCloud<pcl::PointXYZ>::Ptr alignedPC;
      pcl::PointCloud<pcl::PointXYZ>::Ptr blueFilteredPC;
      pcl::PointCloud<pcl::PointXYZ>::Ptr greenFilteredPC;
      pcl::PointCloud<pcl::PointXYZ>::Ptr redFilteredPC;
      pcl::PointCloud<pcl::PointXYZ>::Ptr yellowFilteredPC;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cyanFilteredPC;
      pcl::PointCloud<pcl::PointXYZ>::Ptr magentaFilteredPC;
      pcl::PointCloud<pcl::PointXYZ>::Ptr worldPC;
      pcl::PointCloud<pcl::PointXYZ>::Ptr robotLessPC;
      pcl::PointCloud<pcl::PointXYZ>::Ptr threshPC;

      ros::Time stepTimeStampRGB;
      ros::Time stepTimeStampDepth;

      // move scene camera to hard-coded configuration
      if(!mpt->move_pan_tilt_abs(pan[panTiltCounter], tilt[panTiltCounter]))
      {
        msg_error("failed to move pan-tilt unit for counter %d",panTiltCounter);
      }
      ros::Duration(2.0).sleep();

      _scene_rgb_image = *(ros::topic::waitForMessage<sensor_msgs::Image>(camera_scene_rgb_topic, ros::Duration(3.0)));
      _scene_depth_image = *(ros::topic::waitForMessage<sensor_msgs::Image>(camera_scene_depth_topic, ros::Duration(3.0)));
      // Get RGB image
      try
      {
        stepTimeStampRGB = _scene_rgb_image.header.stamp;
        _cv_image = cv_bridge::toCvCopy(_scene_rgb_image, enc::BGR8);
      }
      catch (cv_bridge::Exception &e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        failed = true;
        res.error_reason=fsm::DATA_ERROR;
        //return false; // SHOULD stop node
      }
      // Get Depth image
      try
      {
        stepTimeStampDepth = _scene_depth_image.header.stamp;
        _cv_depthptr = cv_bridge::toCvCopy(_scene_depth_image, enc::TYPE_32FC1);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        failed = true;
        res.error_reason=fsm::DATA_ERROR;
        //return false; // SHOULD stop node
      }

      // Apply HSV filtering
      ImageFilter hsvFilter(_cv_image->image);

      /*
       * Create threshold for Blue, Green, Red respectively
       */
      // Blue
      hsvFilter.setHsvValues("0000ff");
      thresholdBlue = hsvFilter.getFilteredImage();
      hsvFilter.morphOps(thresholdBlue);
      // Green
      hsvFilter.setHsvValues("00ff00");
      thresholdGreen = hsvFilter.getFilteredImage();
      hsvFilter.morphOps(thresholdGreen);
      // Red
      hsvFilter.setHsvValues("ff0000");
      thresholdRed = hsvFilter.getFilteredImage();
      hsvFilter.morphOps(thresholdRed);
      // Yellow
      hsvFilter.setHsvValues("ffff00");
      thresholdYellow = hsvFilter.getFilteredImage();
      hsvFilter.morphOps(thresholdYellow);
      // Cyan
      hsvFilter.setHsvValues("00ffff");
      thresholdCyan = hsvFilter.getFilteredImage();
      hsvFilter.morphOps(thresholdCyan);
      // Magenta
      hsvFilter.setHsvValues("ff00ff");
      thresholdMagenta = hsvFilter.getFilteredImage();
      hsvFilter.morphOps(thresholdMagenta);


      // TODO: remove hardcoded values
      float fov_horizontal_scene_depth = 1.047;
      float fov_horizontal_scene_rgb = 1.047;

      scenePointCloud = new am_pointcloud(_cv_depthptr->image, fov_horizontal_scene_depth, _cv_image->image, fov_horizontal_scene_rgb);
      // create the initial point cloud based on depth image data
      initialPC = scenePointCloud->createInitialPointCloud();
      // align the initial point cloud with the RGB camera
      alignedPC = scenePointCloud->alignWithRGB(initialPC, CAM_SCENE, _scene_depth_image.header.stamp);
      // Transform the point cloud to world coordinate
      worldPC = scenePointCloud->transformToWorld(alignedPC, CAM_SCENE, _scene_depth_image.header.stamp);
      // filter out the robot, table surface and irrelevant points
      robotLessPC = scenePointCloud->removeRobotFromPointCloud(worldPC);
      threshPC = scenePointCloud->xyzTheresholdCloud(robotLessPC, 0.005); // 0.005: hard-coded z value to remove table surface

#ifdef OCTOMAP_COMPLETE_OLD
      // Create Octomap (whole scene with robot and Pantilt); also the man outside the table can be seen
      OctoCloud->reserve(worldPC->points.size());
      octomap::pointcloudPCLToOctomap(*worldPC,*OctoCloud);
      std::cout<<"[VISION]Starting to add actuall PointCloud to Octomap..."<<std::endl;
      tree->insertScan(*OctoCloud, scenePointCloud->getSensorOriginScene(SCENE_CAM));
      std::cout<<"[VISION]Added actual PointCloud to Octomap"<<std::endl;
#endif

#ifdef OCTOMAP_SERVER
  	  pcl::toROSMsg (*threshPC, msg);
      msg.header.frame_id = "/Origin";
      msg.header.stamp = ros::Time::now();
      pub_3.publish (msg);
#endif

//#ifdef OCTOMAP_SERVER
//      pcl::toROSMsg (*initialPC, msg);
//      msg.header.frame_id = "S_DEPTH";
//      msg.header.stamp = ros::Time::now();
//      pub_3.publish (msg);
//#endif

#ifdef OCTOMAP_REDUCED_OLD
      // Create Octomap without robot and things outside the table
      OctoCloud->reserve(threshPC->points.size());
      octomap::pointcloudPCLToOctomap(*threshPC,*OctoCloud);
      std::cout<<"[VISION]Starting to add actuall PointCloud to Octomap..."<<std::endl;
      tree->insertScan(*OctoCloud, scenePointCloud->getSensorOriginScene(SCENE_CAM));
      std::cout<<"[VISION]Added actual PointCloud to Octomap"<<std::endl;
#endif


      // Add result to global point cloud
      *finalScenePC += *threshPC;

      // Filter the points cloud by color
      blueFilteredPC = scenePointCloud->filterPointCloudByColor(threshPC, thresholdBlue);
      greenFilteredPC = scenePointCloud->filterPointCloudByColor(threshPC, thresholdGreen);
      redFilteredPC = scenePointCloud->filterPointCloudByColor(threshPC, thresholdRed);
      yellowFilteredPC = scenePointCloud->filterPointCloudByColor(threshPC, thresholdYellow);
      cyanFilteredPC = scenePointCloud->filterPointCloudByColor(threshPC, thresholdCyan);
      magentaFilteredPC = scenePointCloud->filterPointCloudByColor(threshPC, thresholdMagenta);

      // Add result (filtered by color) to specific point clouds
      *finalBluePC += *blueFilteredPC;
      *finalGreenPC += *greenFilteredPC;
      *finalRedPC += *redFilteredPC;
      *finalYellowPC += *yellowFilteredPC;
      *finalCyanPC += *cyanFilteredPC;
      *finalMagentaPC += *magentaFilteredPC;


      std::cout<<"[VISION]Finished sweep "<<panTiltCounter<<std::endl;

      // Set the time stamp of current image
      finalTimeStamp = stepTimeStampRGB;

      // set counter for next pan-tilt scan
      panTiltCounter++;

      /*
       * DEBUG
       */
      // publish final thresholded point cloud after each sweep
      //ROS_INFO("Show point cloud");
      pcl::toROSMsg (*finalScenePC, msg);
      msg.header.frame_id = "LWR_0";
      msg.header.stamp = ros::Time::now();
      pub.publish (msg);

    } // END While

    // Move camera to initial configuration
    if(false == mpt->move_pan_tilt_abs(pan_zero, tilt_zero))
    {
    	res.error_reason=fsm::VISION_ERROR;
    }
    std::cout<<"[VISION]pan camera scan complete."<<std::endl;

    // Voxelize result point clouds
    // for each point cloud, we have to create a voxelized version.
    pcl::VoxelGrid<pcl::PointXYZ> vg;

    std::cout<<"[VISION]Voxelization: global point cloud..."<<std::endl;
    vg.setInputCloud(finalScenePC);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*finalVoxelizedPC);

    std::cout<<"[VISION]Voxelization: blue point cloud..."<<std::endl;
    vg.setInputCloud(finalBluePC);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*finalVoxelizedBluePC);

    std::cout<<"[VISION]Voxelization: green point cloud..."<<std::endl;
    vg.setInputCloud(finalGreenPC);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*finalVoxelizedGreenPC);

    std::cout<<"[VISION]Voxelization: red point cloud..."<<std::endl;
    vg.setInputCloud(finalRedPC);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*finalVoxelizedRedPC);

    std::cout<<"[VISION]Voxelization: yellow point cloud..."<<std::endl;
    vg.setInputCloud(finalYellowPC);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*finalVoxelizedYellowPC);

    std::cout<<"[VISION]Voxelization: cyan point cloud..."<<std::endl;
    vg.setInputCloud(finalCyanPC);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*finalVoxelizedCyanPC);

    std::cout<<"[VISION]Voxelization: magenta point cloud..."<<std::endl;
    vg.setInputCloud(finalMagentaPC);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*finalVoxelizedMagentaPC);

    std::cout<<"[VISION]Voxelization: Finished."<<std::endl;

  } // END if

  else {
    msg_error("Waiting for camera msgs failed!");
    res.error_reason=fsm::DATA_ERROR;
  }

}


/*
 * This function takes care of generating several point clouds from TCP unit camera's image data.
 * There are a total of 7 point clouds:
 * - Complete (global), Blue, Green, Red, Yellow, Cyan, Magenta
 * All point clouds are voxelized at the end of the process for memory and performance purposes.
 *
 */
void Vision::scan_with_tcp(am_msgs::TakeImage::Response &res)
{
  am_pointcloud *scenePointCloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr initialPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr alignedPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr blueFilteredPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr greenFilteredPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr redFilteredPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr yellowFilteredPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cyanFilteredPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr magentaFilteredPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr worldPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr robotLessPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr threshPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempVoxelizedPC (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempVoxelizedBluePC (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempVoxelizedGreenPC (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempVoxelizedRedPC (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempVoxelizedYellowPC (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempVoxelizedCyanPC (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempVoxelizedMagentaPC (new pcl::PointCloud<pcl::PointXYZ>());

  ros::Time stepTimeStampRGB;
  ros::Time stepTimeStampDepth;

  _tcp_rgb_image = *(ros::topic::waitForMessage<sensor_msgs::Image>(camera_tcp_rgb_topic, ros::Duration(1.0)));
  _tcp_depth_image = *(ros::topic::waitForMessage<sensor_msgs::Image>(camera_tcp_depth_topic, ros::Duration(1.0)));
  // Convert ROS RGB image to OpenCV format
  try
  {
    stepTimeStampRGB = _tcp_rgb_image.header.stamp;
    _cv_image = cv_bridge::toCvCopy(_tcp_rgb_image, enc::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    failed = true;
    //return false; // SHOULD stop node
  }

  // Convert ROS Depth image to OpencV format
  try
  {
    stepTimeStampDepth = _tcp_depth_image.header.stamp;
    _cv_depthptr = cv_bridge::toCvCopy(_tcp_depth_image, enc::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    failed = true;
    //return false; // SHOULD stop node
  }

  // Apply HSV filtering
  ImageFilter hsvFilter(_cv_image->image);
  /*
   * Create threshold for Blue, Green, Red respectively
   */
  // Blue
  hsvFilter.setHsvValues("0000ff");
  thresholdBlue = hsvFilter.getFilteredImage();
  hsvFilter.morphOps(thresholdBlue);
  // Green
  hsvFilter.setHsvValues("00ff00");
  thresholdGreen = hsvFilter.getFilteredImage();
  hsvFilter.morphOps(thresholdGreen);
  // Red
  hsvFilter.setHsvValues("ff0000");
  thresholdRed = hsvFilter.getFilteredImage();
  hsvFilter.morphOps(thresholdRed);
  // Yellow
  hsvFilter.setHsvValues("ffff00");
  thresholdYellow = hsvFilter.getFilteredImage();
  hsvFilter.morphOps(thresholdYellow);
  // Cyan
  hsvFilter.setHsvValues("00ffff");
  thresholdCyan = hsvFilter.getFilteredImage();
  hsvFilter.morphOps(thresholdCyan);
  // Magenta
  hsvFilter.setHsvValues("ff00ff");
  thresholdMagenta = hsvFilter.getFilteredImage();
  hsvFilter.morphOps(thresholdMagenta);

  // TODO: remove hardcoded Field-of-View values
  float fov_horizontal_tcp_depth = 1.048;
  float fov_horizontal_tcp_rgb = 1.048;

  scenePointCloud = new am_pointcloud(_cv_depthptr->image, fov_horizontal_tcp_depth, _cv_image->image, fov_horizontal_tcp_rgb);
  // create the initial point cloud based on depth image data
  initialPC = scenePointCloud->createInitialPointCloud();
  // align the initial point cloud with the RGB camera
  alignedPC = scenePointCloud->alignWithRGB(initialPC, CAM_TCP, stepTimeStampRGB);
  // Transform the point cloud to world coordinate
  worldPC = scenePointCloud->transformToWorld(alignedPC, CAM_TCP, stepTimeStampRGB);
  // filter out the robot, table surface and irrelevant points
  robotLessPC = scenePointCloud->removeRobotFromPointCloud(worldPC);
  threshPC = scenePointCloud->xyzTheresholdCloud(robotLessPC, 0.005); // 0.005: hard-coded z value to remove table surface

#ifdef OCTOMAP_COMPLETE_OLD
  // Create Octomap (whole scene with robot and Pantilt); also the man outside the table can be seen
  OctoCloud->reserve(worldPC->points.size());
  octomap::pointcloudPCLToOctomap(*worldPC,*OctoCloud);
  std::cout<<"[VISION]Starting to add actual PointCloud to Octomap..."<<std::endl;
  tree->insertScan(*OctoCloud, scenePointCloud->getSensorOriginScene(TCP_CAM));
  std::cout<<"[VISION]Added actual PointCloud to Octomap"<<std::endl;
#endif

#ifdef OCTOMAP_SERVER
  	  pcl::toROSMsg (*threshPC, msg);
      msg.header.frame_id = "/Origin";
      msg.header.stamp = ros::Time::now();
      pub_3.publish (msg);
#endif

//#ifdef OCTOMAP_SERVER
//  pcl::toROSMsg (*initialPC, msg);
//  msg.header.frame_id = "T_DEPTH";
//  msg.header.stamp = ros::Time::now();
//  pub_3.publish (msg);
//#endif

#ifdef OCTOMAP_REDUCED_OLD
  // Create Octomap without robot and things outside the table
  OctoCloud->reserve(threshPC->points.size());
  octomap::pointcloudPCLToOctomap(*threshPC,*OctoCloud);
  std::cout<<"[VISION]Starting to add actuall PointCloud to Octomap..."<<std::endl;
  tree->insertScan(*OctoCloud, scenePointCloud->getSensorOriginScene(TCP_CAM));
  std::cout<<"[VISION]Added actual PointCloud to Octomap"<<std::endl;
#endif

  *finalTcpPC += *threshPC;

  // Filter the points cloud by color
  blueFilteredPC = scenePointCloud->filterPointCloudByColor(threshPC, thresholdBlue);
  greenFilteredPC = scenePointCloud->filterPointCloudByColor(threshPC, thresholdGreen);
  redFilteredPC = scenePointCloud->filterPointCloudByColor(threshPC, thresholdRed);
  yellowFilteredPC = scenePointCloud->filterPointCloudByColor(threshPC, thresholdYellow);
  cyanFilteredPC = scenePointCloud->filterPointCloudByColor(threshPC, thresholdCyan);
  magentaFilteredPC = scenePointCloud->filterPointCloudByColor(threshPC, thresholdMagenta);
  // Add result (filtered by color) to specific point clouds
  *finalBluePC += *blueFilteredPC;
  *finalGreenPC += *greenFilteredPC;
  *finalRedPC += *redFilteredPC;
  *finalYellowPC += *yellowFilteredPC;
  *finalCyanPC += *cyanFilteredPC;
  *finalMagentaPC += *magentaFilteredPC;

  // Voxelize result point clouds
  // for each point cloud, we have to create a voxelized version.
  pcl::VoxelGrid<pcl::PointXYZ> vg;

  std::cout<<"[VISION]Voxelization: global point cloud..."<<std::endl;
  vg.setInputCloud(threshPC);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter (*tempVoxelizedPC);

  std::cout<<"[VISION]Voxelization: blue point cloud..."<<std::endl;
  vg.setInputCloud(finalBluePC);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter (*finalVoxelizedBluePC);

  std::cout<<"[VISION]Voxelization: green point cloud..."<<std::endl;
  vg.setInputCloud(finalGreenPC);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter (*finalVoxelizedGreenPC);

  std::cout<<"[VISION]Voxelization: red point cloud..."<<std::endl;
  vg.setInputCloud(finalRedPC);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter (*finalVoxelizedRedPC);

  std::cout<<"[VISION]Voxelization: yellow point cloud..."<<std::endl;
  vg.setInputCloud(finalYellowPC);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter (*finalVoxelizedYellowPC);

  std::cout<<"[VISION]Voxelization: cyan point cloud..."<<std::endl;
  vg.setInputCloud(finalCyanPC);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter (*finalVoxelizedCyanPC);

  std::cout<<"[VISION]Voxelization: magenta point cloud..."<<std::endl;
  vg.setInputCloud(finalMagentaPC);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter (*finalVoxelizedMagentaPC);

  std::cout<<"[VISION]Voxelization: finished."<<std::endl;

  *finalVoxelizedPC += *tempVoxelizedPC;
  *finalVoxelizedBluePC += *tempVoxelizedBluePC;
  *finalVoxelizedGreenPC += *tempVoxelizedGreenPC;
  *finalVoxelizedRedPC += *tempVoxelizedRedPC;
  *finalVoxelizedYellowPC += *tempVoxelizedYellowPC;
  *finalVoxelizedCyanPC += *tempVoxelizedCyanPC;
  *finalVoxelizedMagentaPC += *tempVoxelizedMagentaPC;


  // Set the time stamp of current image
  finalTimeStamp = stepTimeStampRGB;
  /*
   * DEBUG
   */
  //ROS_INFO("Show point cloud");
  pcl::toROSMsg (*finalVoxelizedPC, msg);
  msg.header.frame_id = "LWR_0";
  msg.header.stamp = ros::Time::now();
  pub.publish (msg);

}


/**
 * This function aligns a pre-defined object model with an input point cloud.
 * The input point cloud is calculated by functions from am_pointcloud class.
 **/

Eigen::Matrix4f Vision::align_PointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr object_input, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_input, bool box, bool cylinder)
{

  // Align a rigid object to a scene with clutter and occlusions

  // Point clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr scene (new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);

  // Remove NANs from PointCloud scene
  scene_input->is_dense = false;
  object_input->is_dense = false;
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*scene_input, *scene_input, index);
  pcl::removeNaNFromPointCloud(*object_input, *object_input, index);


  //	//Downsample
  if (cylinder)
  {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(object_input);
    vg.setLeafSize (leaf_size*2, leaf_size*2, leaf_size*2);
    vg.filter (*object_input);
    vg.setInputCloud(scene_input);
    vg.filter (*scene_input);
  }
  else if (box)
  {
    //Downsample (Uniform Sampling)
    pcl::PointCloud<int> sampled_indices;
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud (object_input);
    uniform_sampling.setRadiusSearch (0.006);
    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*object_input, sampled_indices.points, *object_input);
    uniform_sampling.setInputCloud (scene_input);
    uniform_sampling.setRadiusSearch (0.006);
    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*scene_input, sampled_indices.points, *scene_input);
  }
  else
  {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(object_input);
    vg.setLeafSize (leaf_size*4, leaf_size*4, leaf_size*4);
    vg.filter (*object_input);
    vg.setInputCloud(scene_input);
    vg.filter (*scene_input);
  }

  // publish final thresholded point cloud
  std::cout<<"[VISION]Show downsampled point cloud"<<std::endl;
  pcl::toROSMsg ((*scene_input + *object_input), msg);
  msg.header.frame_id = "LWR_0";
  msg.header.stamp = ros::Time::now();
  pub.publish (msg);


  // Estimate normals for scene
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<pcl::PointXYZ,PointNT> nest;
  nest.setKSearch(10);
  nest.setInputCloud (scene_input);
  nest.compute (*scene);

  // Estimate normals for scene
  pcl::console::print_highlight ("Estimating object normals...\n");
  nest.setKSearch(10);
  nest.setInputCloud (object_input);
  nest.compute (*object);

  std::cout <<"scene points: " <<scene->points.size() <<"           " <<"object points: " <<object->points.size() <<std::endl
      << "box: " <<box <<"      " <<"cylinder: "  <<cylinder  <<std::endl;

  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setKSearch(15);
  fest.setInputCloud (object_input);
  fest.setInputNormals (object);
  fest.compute (*object_features);
  fest.setInputCloud (scene_input);
  fest.setInputNormals (scene);
  fest.compute (*scene_features);


  // Perform alignment
  uint32_t iters;

  if(cylinder)
    iters = 350000;
  else
    iters = 900000;

  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<pcl::PointXYZ,pcl::PointXYZ,FeatureT> align;
  align.setInputSource (object_input);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene_input);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (iters); // Number of RANSAC iterations
  align.setNumberOfSamples (4); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (2); // Number of nearest features to use
  align.setSimilarityThreshold (0.65f); // Polygonal edge length similarity threshold

  if(box)
  {
    align.setMaxCorrespondenceDistance (0.003); // Inlier threshold box
    align.setInlierFraction (0.30f); // Required inlier fraction for accepting a pose hypothesis
  }

  else
  {
    align.setMaxCorrespondenceDistance (0.004); // Inlier threshold other shapes
    align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  }

  Eigen::Matrix4f transform;
  uint16_t nmbr_tries=0;
  while(nmbr_tries<2)
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);

    if (align.hasConverged ())
    {
      // Print results
      printf ("\n");
      transform = align.getFinalTransformation ();
      pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transform (0,0), transform (0,1), transform (0,2));
      pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transform (1,0), transform (1,1), transform (1,2));
      pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transform (2,0), transform (2,1), transform (2,2));
      pcl::console::print_info ("\n");
      pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transform (0,3), transform (1,3), transform (2,3));
      pcl::console::print_info ("\n");
      pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

      obj_aligned_=true;
      break;
    }
    else
    {
      transform = align.getFinalTransformation ();
      pcl::console::print_error ("Alignment failed!\n");
      obj_aligned_=false;

      if(cylinder)
        iters+=150000;
      else
        iters+=400000;

      align.setMaximumIterations (iters);
    }

    nmbr_tries++;
  }

  return transform;
}

/*
 * This function enhances the initial computed pose of the CUBE with the help of a close-up image.
 * The image is taken using the TCP camera right from above the cube, this will eliminate the rotation error in
 * X and Y axes (The only rotation a cube can have is around Z axis).
 */
double Vision::close_range_pose(string color)
{

  std::cout<<"[VISION]Entered Vision::close_range_pose()"<<std::endl;
  /*
   * Initialization
   */
  am_pointcloud *cornersPointCloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr initialPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr alignedPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr worldPC;

  std::vector<cv::Point2f> corners;
  std::vector<pcl::PointXYZ> cornersInCamera;
  std::vector<pcl::PointXYZ> worldCorners;
  std::vector<pcl::PointXYZ> worldPoints;

  ros::Time stepTimeStampRGB;
  ros::Time stepTimeStampDepth;

  _tcp_rgb_image = *(ros::topic::waitForMessage<sensor_msgs::Image>(camera_tcp_rgb_topic, ros::Duration(1.0)));
  _tcp_depth_image = *(ros::topic::waitForMessage<sensor_msgs::Image>(camera_tcp_depth_topic, ros::Duration(1.0)));

  try
  {
    stepTimeStampRGB = _tcp_rgb_image.header.stamp;
    _cv_image = cv_bridge::toCvCopy(_tcp_rgb_image, enc::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    failed = true;
    //return false; // SHOULD stop node
  }
  try
  {
    stepTimeStampDepth = _tcp_depth_image.header.stamp;
    _cv_depthptr = cv_bridge::toCvCopy(_tcp_depth_image, enc::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    failed = true;
    //return false; // SHOULD stop node
  }

  // TODO: remove hardcoded Field-of-View values
  float fov_horizontal_tcp_depth = 1.048;
  float fov_horizontal_tcp_rgb = 1.048;

  cornersPointCloud = new am_pointcloud(_cv_depthptr->image, fov_horizontal_tcp_depth, _cv_image->image, fov_horizontal_tcp_rgb);
  // create the initial point cloud based on depth image data
  initialPC = cornersPointCloud->createInitialPointCloud();
  // align the initial point cloud with the RGB camera
  alignedPC = cornersPointCloud->alignWithRGB(initialPC, CAM_TCP, _tcp_rgb_image.header.stamp);
  worldPC = cornersPointCloud->transformToWorld(alignedPC, CAM_TCP, _tcp_depth_image.header.stamp);
  /*
   * END Initialization
   */

  /*
   * Find corners
   */

  Mat src = _cv_image->image;
  cv::Mat bw; // Grayscale image
  cv::Mat dst = src.clone(); // for debugging purpose
  /*
   * START: Safe Pre-processing step: Use HSV filtering and then find edges
   */
  Mat colorFilter, threshGray;
  ImageFilter hsvFilter(src);
  // Blue
  hsvFilter.setHsvValues(color);
  colorFilter = hsvFilter.getFilteredImage();
  hsvFilter.morphOps(colorFilter);
  colorFilter.convertTo(bw, CV_8U);
  /*
   * END: Safe Pre-processing step: Use HSV filtering and then find edges
   */

  //cv::cvtColor(src, bw, CV_BGR2GRAY);
  // blur the image
  //cv::blur(bw, bw, cv::Size(3, 3));
  cv::GaussianBlur(bw, bw, Size(11,11), 0, 0);
  // use canny edge-detector
  cv::Canny(bw, bw, 90, 110, 3);


  // use Probabilistic Hough Transform to get better results (less error in finding rectangle sides as lines)
  std::vector<cv::Vec4i> lines;
  int houghLineCounter = 0;
  int houghThreshold = 90;
  double minLineLength = 30;

  cv::HoughLinesP(bw, lines, 1, CV_PI/180, houghThreshold, minLineLength, 10);
  std::cout<<"[VISION]found the following number of lines: "<<lines.size()<<std::endl;
  // Error Handling
  // If we were not able to find enough lines to compute intersections, we have to
  // change the paramters of houghLineP to a more relaxed combination
  while(true)
  {
    lines.clear(); // remove all elements from the vector
    houghThreshold -= 10;
    minLineLength -= 5;
    std::cout<<"[VISION]not enough lines. Changing paramters... "<<std::endl;
    cv::HoughLinesP(bw, lines, 1, CV_PI/180, houghThreshold, minLineLength, 10);
    std::cout<<"[VISION]found the following new number of lines: "<<lines.size()<<std::endl;
    if (lines.size() >= 3)
    {
      std::cout<<"[VISION]enough lines, let's move on!"<<std::endl;
      houghLineCounter = 0; // reset counter
      break;
    }
    if (houghLineCounter >= 3)
    {
      // cannot modify the paramters anymore. Vision node is not able to process good lines/corners.
      // return the initial pose to StateMachine
      houghLineCounter = 0; // reset counter
      std::cout<<"[VISION]failed to find enough lines. return initial pose"<<std::endl;
      return -5;
    }

    houghLineCounter++;
  }
  //	for (int i = 0; i < lines.size(); i++)
  //	{
  //	  cv::Vec4i v = lines[i];
  //	  cv::line(dst, cv::Point(v[0], v[1]), cv::Point(v[2], v[3]), CV_RGB(0,255,0), 2);
  //	}

  /*
   * START compute angles using lines (instead of corners)
   * - pick a line
   * - make sure it is one of the sides of the rectangle
   * - compute the angle between that line and table's Y-axis
   */

  // find two lines (sides of the rectangle) which are perpendicular
  std::vector<int> idx;
  idx = find_perpendicular_lines(lines);
  if(idx[0] == -1 || idx[1] == -1)
  {
    std::cout<<"[VISION]could not find perpendicular lines."<<std::endl;
    return -5;
  }

  std::cout<<"[VISION]perpendicular lines are: "<<idx[0]<<", "<<idx[1]<<std::endl;

//  // Draw the lines
//  cv::Vec4i v = lines[idx[0]];
//  cv::line(dst, cv::Point(v[0], v[1]), cv::Point(v[2], v[3]), CV_RGB(0,255,0), 2);
//  v = lines[idx[1]];
//  cv::line(dst, cv::Point(v[0], v[1]), cv::Point(v[2], v[3]), CV_RGB(255,0,0), 2);
//  cv::imshow("lines", dst);
//  cv::imshow("canny", bw);
//  cv::waitKey();

  std::cout<<"[VISION]Picking one of the perpendicular lines"<<std::endl;
  std::cout<<"[VISION]transforming the line to world coordinate"<<std::endl;
  std::vector<cv::Point2f> linePoints;
  cv::Point2f tempPoint;
  tempPoint.x = lines[idx[0]][0];
  tempPoint.y = lines[idx[0]][1];
  linePoints.push_back(tempPoint);
  tempPoint.x = lines[idx[0]][2];
  tempPoint.y = lines[idx[0]][3];
  linePoints.push_back(tempPoint);
  worldPoints = find_points_world(worldPC, linePoints);

  double unitVecX = 0.000;
  double unitVecY = 1.000;

  double targetVecX, targetVecY, dot, det, rotation;
  targetVecX = worldPoints[1].x - worldPoints[0].x;
  targetVecY = worldPoints[1].y - worldPoints[0].y;

  dot = (targetVecX*unitVecX) + (targetVecY*unitVecY);
  det = (targetVecX*unitVecY) - (targetVecY*unitVecX);

  rotation = std::atan2(det, dot);

  //std::cout<<"[VISION]rotation around z-axis of table: "<<rotation<<std::endl;
  /*
   * END compute angles using lines (instead of corners)
   */


  /*
   * START compute angles using corners
   */
  //	// Find intersections between every two lines --> corners
  //	for (int i = 0; i < lines.size(); i++)
  //	{
  //		for (int j = i+1; j < lines.size(); j++)
  //		{
  //			cv::Point2f pt = compute_intersect(lines[i], lines[j]);
  //			if (pt.x >= 0 && pt.x<src.cols && pt.y >= 0 && pt.y<src.rows)
  //				corners.push_back(pt);
  //		}
  //	}
  //
  //	std::cout<<"found the following "<<corners.size()<<" corner(s):"<<std::endl;
  //	for(int i=0; i<corners.size(); i++)
  //		std::cout<<i<<": "<<corners[i].x<<", "<<corners[i].y<<std::endl;
  //
  //	std::cout<<"sorting corners..."<<std::endl;
  //	sort_corners(corners);
  //	for(int i=0; i<corners.size(); i++)
  //		std::cout<<i<<": "<<(int)corners[i].x<<", "<<(int)corners[i].y<<std::endl;
  //
  //	/*
  //	 * START DEBUG --> show corners
  //	 */
  //
  //
  //	cv::circle(dst, corners[0], 3, CV_RGB(0,0,0), 2);
  //	cv::circle(dst, corners[1], 3, CV_RGB(255,255,255), 2);
  //	cv::circle(dst, corners[2], 3, CV_RGB(0,0,255), 2);
  //	cv::circle(dst, corners[3], 3, CV_RGB(0,255,0), 2);
  //
  //	std::cout<<"show corners..."<<std::endl;
  //	cv::imshow("corners", dst);
  //	//cv::imshow("canny", bw);
  //	cv::waitKey();
  //	/*
  //	 * END DEBUG
  //	 */
  //
  //	std::cout<<"transforming corners to world coor."<<std::endl;
  //	worldCorners = find_points_world(worldPC, corners);
  //	for(int i=0; i<worldCorners.size(); i++)
  //		std::cout<<i<<": "<<worldCorners[i].x<<", "<<worldCorners[i].y<<", "<<worldCorners[i].z<<std::endl;
  //
  //	// Choose one side of the rectangle (first and second sorted corners)
  //	double RecSideVectorX, RecSideVectorY, magnitude, dot, det, rotation;
  //
  //	//	// sort world corners
  //	//	pcl::PointXYZ tempPoint;
  //
  //	// Unit vector along Y axis
  //	double unitVecX = 0.000;
  //	double unitVecY = 1.000;
  //
  //	// Create a vector out of two corners (one side of the rectangle)
  //	RecSideVectorX = worldCorners[1].x - worldCorners[0].x;
  //	RecSideVectorY = worldCorners[1].y - worldCorners[0].y;
  //
  //	magnitude = sqrt( (RecSideVectorX*RecSideVectorX) + (RecSideVectorY*RecSideVectorY) );
  //
  //	// Finding the angle between two vectors
  //	//dot = x1*x2 + y1*y2      # dot product
  //	//det = x1*y2 - y1*x2      # determinant
  //	//angle = atan2(det, dot)  # atan2(y, x) or atan2(sin, cos)
  //	dot = RecSideVectorX*unitVecX + RecSideVectorY*unitVecY;
  //	det = RecSideVectorX*unitVecY - RecSideVectorY*unitVecX;
  //
  //	// Principal arc cosine of y/x, in the interval [-pi,+pi] radians.
  //	// clockwise angle between the two vectors
  //	//double rotation = std::acos(dot/magnitude);
  //	rotation = std::atan2(det, dot);
  //
  //	std::cout<<"found rotation: "<<rotation<<std::endl;
  /*
   * END compute angles using corners
   */

  return (-1)*rotation;
}

/*
 * This function sort a vector of corners (of a rectangle) based on the camera frame
 * The order will be:
 * 1) TOP_LEFT
 * 2) TOP_RIGHT
 * 3) BOTTOM_LEFT
 * 4) BOTTOM_RIGHT
 */
void Vision::sort_corners(std::vector<cv::Point2f>& corners)
{

  cv::Point2f center(0,0);
  // Get mass center
  for (int i = 0; i < corners.size(); i++)
    center += corners[i];
  center *= (1. / corners.size());

  std::vector<cv::Point2f> top, bot;
  for (int i = 0; i < corners.size(); i++)
  {
    if (corners[i].y < center.y)
      top.push_back(corners[i]);
    else
      bot.push_back(corners[i]);
  }
  cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
  cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
  cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
  cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];
  corners.clear();
  corners.push_back(tl);
  corners.push_back(tr);
  corners.push_back(br);
  corners.push_back(bl);
}

/*
 * This functions will try to find two lines out of a vector which are perpendicular
 * In case it could not find such lines, it will return -1
 */
std::vector<int> Vision::find_perpendicular_lines(std::vector<cv::Vec4i>& lines)
{
  // cv::Vec4i v = lines[i];

  std::vector<int> indices (2,-1); // two ints with the value -1

  for (int i=0; i<lines.size()-1; i++)
    for (int j=1; j<lines.size(); j++)
    {
      cv::Vec4i v = lines[i];
      float vec1X = v[2] - v[0];
      float vec1Y = v[3] - v[1];

      v = lines[j];
      float vec2X = v[2] - v[0];
      float vec2Y = v[3] - v[1];
      //      float vec1X = line[i][2] - line[i][0]; // X value of the first vector
      //      float vec1Y = line[i][3] - line[i][1]; // Y value of the first vector
      //
      //      float vec2X = line[j][2] - line[j][0]; // X value value of the second vector
      //      float vec2Y = line[j][3] - line[j][1]; // Y value value of the second vector

      double dot = vec1X*vec2X + vec1Y*vec2Y;
      double det = vec1X*vec2Y - vec1Y*vec2X;

      double angle = std::atan2(det, dot);

      std::cout<<"[VISION]angle of "<<i<<" and "<<j<<" : "<<angle<<std::endl;

      if( 1.54 <= std::abs(angle) && std::abs(angle) <= 1.61 )
      {
        std::cout<<"[VISION]found two perpendicular lines with apprx. angle: "<<angle<<std::endl;
        // found two perpendicular lines
        indices[0] = i;
        indices[1] = j;
        return indices;
      }
      if( 4.69 <= std::abs(angle) && std::abs(angle) <= 4.75 )
      {
        std::cout<<"[VISION]found two perpendicular lines with apprx. angle: "<<angle<<std::endl;
        // found two perpendicular lines
        indices[0] = i;
        indices[1] = j;
        return indices;
      }

    }

  return indices; // in case no two perpendicular lines were found, return (-1,-1)
}

/*
 * This function tries to find the intersection between two given lines.
 * If the input lines do not meet at any point, the function will return (-1,-1)
 *
 * using line-line intersection formula from:
 * http://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
 */
cv::Point2f Vision::compute_intersect(cv::Vec4i a, cv::Vec4i b)
{
  int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3], x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];
  float denom = ((float)(x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4));
  if (denom)
  {
    cv::Point2f pt;
    pt.x = ( ((x1 * y2 - y1 * x2) * (x3 - x4)) - ((x1 - x2) * (x3 * y4 - y3 * x4)) ) / denom;
    pt.y = ( ((x1 * y2 - y1 * x2) * (y3 - y4)) - ((y1 - y2) * (x3 * y4 - y3 * x4)) ) / denom;
    return pt;
  }
  else
    return cv::Point2f(-1, -1);
}

/*
 * This function gets a point cloud in world coordinate system and a vector of points/pixel coordinates in the respective 2D RGB image.
 * It will return the coordinates of those points in the world coordinate system.
 * Due to the transformation between the RGB and DEPTH camera, it is normal to face NaN values for certain points. To solve
 * this problem, an averaging process is applied to find the nearest possible point to the corresponding pixel.
 */
std::vector<pcl::PointXYZ> Vision::find_points_world(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<cv::Point2f> points)
{
  std::vector<pcl::PointXYZ> foundPoints;

  int row;
  int column;
  for (int n=0; n<points.size();n++ )
  {
    pcl::PointXYZ p, q;
    double counter = 0;
    for (int i=-2; i<3; i++)
      for (int j=-2; j<3; j++)
      {
        column = (int)points[n].x + i;
        row = (int)points[n].y + j;
        if (row>=0 && column>=0 && row<cloud->height && column<cloud->width)
        {
          p.x = cloud->at(column,row).x;
          p.y = cloud->at(column,row).y;
          p.z = cloud->at(column,row).z;

          if( pcl_isfinite(p.x) || pcl_isfinite(p.y) || pcl_isfinite(p.z) )
          {
            if (p.x!=0 || p.y!=0 || p.z!=0)
            {
              q.x += p.x;
              q.y += p.y;
              q.z += p.z;
              counter ++;
            }
          }
        }
      }
    if (counter != 0)
    {
      q.x /= counter;
      q.y /= counter;
      q.z /= counter;
    }
    foundPoints.push_back(q);

  }

  /*
   * DEBUG PUBLISH TRANSFORMED POINTS IN RVIZ
   */
  //	pcl::PointCloud<pcl::PointXYZ>::Ptr cornerPC;
  //	cornerPC.reset (new pcl::PointCloud<pcl::PointXYZ> (2,2));
  //	int newCounter = 0;
  //
  //	for (int i=0; i<cornerPC->height; i++)
  //		for (int j=0; j<cornerPC->width; j++)
  //		{
  //			if(newCounter<foundPoints.size())
  //			{
  //				cornerPC->at(j,i).x = foundPoints[newCounter].x;
  //				cornerPC->at(j,i).y = foundPoints[newCounter].y;
  //				cornerPC->at(j,i).z = foundPoints[newCounter].z;
  //			}
  //			newCounter++;
  //		}
  /*
   * END DEBUG PUBLISH TRANSFORMED POINTS IN RVIZ
   */

  return foundPoints;
}


int Vision::verify_close_range_pose(pcl::PointCloud<pcl::PointXYZ>::Ptr targetPC, pcl::PointCloud<pcl::PointXYZ>::Ptr closeRangePC)
{

	// Octree resolution - side length of octree voxels
	float resolution = 0.005f;

	// Instantiate octree-based point cloud change detection class
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);

	// Add points from targetPC to octree
	octree.setInputCloud (targetPC);
	octree.addPointsFromInputCloud ();

	// Switch octree buffers: This resets octree but keeps previous tree structure in memory.
	octree.switchBuffers ();

	// Add points from cloudB to octree
	octree.setInputCloud (closeRangePC);
	octree.addPointsFromInputCloud ();

	std::vector<int> newPointIdxVector;

	// Get vector of point indices from octree voxels which did not exist in previous buffer
	octree.getPointIndicesFromNewVoxels (newPointIdxVector);

	// Output points
	std::cout<<"vector size"<<newPointIdxVector.size()<<std::endl;
//	std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
//	for (size_t i = 0; i < newPointIdxVector.size (); ++i)
//		std::cout << i << "# Index:" << newPointIdxVector[i]
//				  << "  Point:" << closeRangePC->points[newPointIdxVector[i]].x << " "
//				  << closeRangePC->points[newPointIdxVector[i]].y << " "
//				  << closeRangePC->points[newPointIdxVector[i]].z << std::endl;

	return newPointIdxVector.size();
}


int Vision::verify_object_inside_zone(string color, pcl::PointXYZ zone_center, float radius)
{
  /*
   * Initialzation: point cloud
   */
  am_pointcloud *closeRangePointCloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr initialPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr alignedPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr colorFilteredPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr worldPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr threshPC;

  ros::Time stepTimeStampRGB;
  ros::Time stepTimeStampDepth;

  _tcp_rgb_image = *(ros::topic::waitForMessage<sensor_msgs::Image>(camera_tcp_rgb_topic, ros::Duration(1.0)));
  _tcp_depth_image = *(ros::topic::waitForMessage<sensor_msgs::Image>(camera_tcp_depth_topic, ros::Duration(1.0)));
  // Convert ROS RGB image to OpenCV format
  try
  {
    stepTimeStampRGB = _tcp_rgb_image.header.stamp;
    _cv_image = cv_bridge::toCvCopy(_tcp_rgb_image, enc::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    failed = true;
    return false;
  }

  // Convert ROS Depth image to OpencV format
  try
  {
    stepTimeStampDepth = _tcp_depth_image.header.stamp;
    _cv_depthptr = cv_bridge::toCvCopy(_tcp_depth_image, enc::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    failed = true;
    return false; // SHOULD stop node
  }

  // TODO: remove hardcoded Field-of-View values
  float fov_horizontal_tcp_depth = 1.048;
  float fov_horizontal_tcp_rgb = 1.048;

  closeRangePointCloud = new am_pointcloud(_cv_depthptr->image, fov_horizontal_tcp_depth, _cv_image->image, fov_horizontal_tcp_rgb);
  // create the initial point cloud based on depth image data
  initialPC = closeRangePointCloud->createInitialPointCloud();
  // align the initial point cloud with the RGB camera
  alignedPC = closeRangePointCloud->alignWithRGB(initialPC, CAM_TCP, _tcp_rgb_image.header.stamp);
  worldPC = closeRangePointCloud->transformToWorld(alignedPC, CAM_TCP, _tcp_depth_image.header.stamp);
  threshPC = closeRangePointCloud->xyzTheresholdCloud(worldPC, 0.005);

  // filter point cloud by color --> check if we are looking at the correct point cloud
  cv::Mat threshold;
  ImageFilter hsvFilter(_cv_image->image);
  hsvFilter.setHsvValues(color);
  threshold = hsvFilter.getFilteredImage();
  hsvFilter.morphOps(threshold);

  colorFilteredPC = closeRangePointCloud->filterPointCloudByColor(threshPC, threshold);

  int nanCounter = 0;
  int cloudSize = colorFilteredPC->height * colorFilteredPC->width;
  std::cout<<"colorfiltercloud size = "<<cloudSize<<std::endl;
  for(int i=0; i<colorFilteredPC->height; i++)
	  for(int j=0; j<colorFilteredPC->width; j++)
	  {
		  if( !pcl_isfinite(colorFilteredPC->at(j,i).x) )
		  {
			  nanCounter++;
			  continue;
			  //std::cout<<"X: "<<colorFilteredPC->at(j,i).x<<", Y: "<<colorFilteredPC->at(j,i).y<<" Z: "<<colorFilteredPC->at(j,i).z<<std::endl;
		  }
		  if( colorFilteredPC->at(j,i).x == 0 && colorFilteredPC->at(j,i).y == 0 && colorFilteredPC->at(j,i).z == 0 )
		  {
			  nanCounter++;
			  continue;
		  }
	  }
  std::cout<<"NanCounter = "<<nanCounter<<std::endl;
  if (nanCounter == cloudSize)
  {
	  std::cout<<"cloud is empty!!"<<std::endl;
	  return 0;
  }


  /*
   * END Initialzation: point cloud
   */

  /*
   * 2D Image Proc.: find circle (target zone)
   */
//  // Apply HSV filtering
//  cv::Mat threshold, src, src_gray;
//  src = _cv_image->image;
//
//  ImageFilter hsvFilter(src);
//  hsvFilter.setHsvValues(color);
//  threshold = hsvFilter.getFilteredImage();
//  hsvFilter.morphOps(threshold);
//  threshold.convertTo(src_gray, CV_8U);
//
//  // Convert it to gray
////  cvtColor(src, src_gray, CV_BGR2GRAY );
//
//  // Reduce the noise so we avoid false circle detection
//  cv::GaussianBlur(src_gray, src_gray, Size(9, 9), 2, 2);
//
//  vector<Vec3f> circles;
//  // Apply the Hough Transform to find the circles
//  cv::HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0 );
//
//
//  std::cout<<"circles: "<<circles.size()<<std::endl;
//  Point center( cvRound(circles[0][0]), cvRound(circles[0][1]) );
//  int radius;
//  // Draw the circles detected
//  for( size_t i = 0; i < circles.size(); i++ )
//  {
//    Point center( cvRound(circles[i][0]), cvRound(circles[i][1]) );
//    radius = cvRound(circles[i][2]);
//    // circle center
//    circle(src, center, 3, Scalar(0,255,0), -1, 8, 0 );
//    // circle outline
//    circle(src, center, radius, Scalar(0,0,255), 3, 8, 0 );
//  }
//
////  cv::imshow( "Threshold", src_gray);
////  cv::imshow( "Circles", src);
////  cv::waitKey();
////  ros::Duration(5.0).sleep();
//
//
//  if (circles.size() == 0)
//  {
//    std::cout<<"[VISION]could not find any circle..."<<std::endl;
//    return false;
//  }
//  else if (circles.size() > 1)
//  {
//    std::cout<<"[VISION]error: found more than one circle"<<std::endl;
//    return false;
//  }

  // look inside the target zone to find the object
  if ( search_for_object_on_zone(threshPC, zone_center, radius) )
	  return 2;
  else
	  return 1;
}

/*
 * This function receives a point cloud, a center point and a radius as reference. It then looks for any point inside that
 * point cloud within a rectangular region, centered at "center point", which has a Z value greater than 0.01
 */
bool Vision::search_for_object_on_zone(pcl::PointCloud<pcl::PointXYZ >::Ptr inputCloud, pcl::PointXYZ zoneCenter, float radius)
{
	// calculate center of mass
	pcl::PointXYZ centerOfMass;
	centerOfMass = am_pointcloud::calculateCenterOfMass(inputCloud);

	std::cout<<"Center of Mass:  X= "<<centerOfMass.x<<",  Y= "<<centerOfMass.y<<",  Z="<<centerOfMass.z<<std::endl;

	float distanceX = std::abs(centerOfMass.x - zoneCenter.x);
	float distanceY = std::abs(centerOfMass.y - zoneCenter.y);

	if ( distanceX < radius+0.005 && distanceY < radius+0.005 )
	{
		std::cout<<"Object placed within the correct radius."<<std::endl;
		return true;
	}

//	pcl::PointXYZ p;
//	for (int i=(-1)*radius; i<radius; i++)
//	{
//		for (int j=(-1)*radius; j<radius; j++)
//		{
//			p.z = inputCloud->at( center.x-i , center.y-j).z;
//			if(pcl_isfinite(p.z))
//				if (p.z > 0.005)
//				{
//					std::cout<<"found a point with Z > 0.05"<<std::endl;
//					return true;
//				}
//		}

  // could not find any point with Z > 0.01
  // object is not placed on the target zone
  std::cout<<"Object is far away from the target zone"<<std::endl;
  return false;
}
