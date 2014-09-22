#include "vision.hpp"
#include <config.hpp>
#include <math.h>
#include <ImageFilter.cpp>
#include <ampointcloud.cpp>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>

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

        // leaf size voxels
        leaf_size = 0.0025;

        // Create and set resolution of Octree
        tree = new octomap::OcTree(0.005);

        // Create octomap::PointCloud
        OctoCloud = new octomap::Pointcloud();

	vision_server_.start();
	ROS_INFO("vision action server started.");
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

  ros::Duration(1.0).sleep();

  switch (req.camera) {
    case SCENE_CAM:
      ROS_INFO("callbac: SCENE CAM!");
      ROS_INFO("SCENE Pointcloud updated.");
      scan_with_pan_tilt();
      break;
    case TCP_CAM:
      ROS_INFO("callback: TCP CAM!");
      scan_with_tcp();
      ROS_INFO("TCP Pointcloud updated.");
      break;
    default:
      ROS_WARN("Unknown camera type!");
      return false;
  }

  return true;
}

/*
 * END CALLBACKS
 */


void Vision::handle(const am_msgs::VisionGoal::ConstPtr &goal)
{
  if(goal->mode == GLOBAL_POSE_ESTIMATION) {

  ROS_INFO("Entered Vision::handle()...");

#ifdef OUTPUT_OCTOMAP_BIN
  tree->writeBinary("/home/euroc_student/EUROC_SVN_STUDENT/vision_octomap/Octomap.bt");
  ROS_INFO("Tree was written to Binary File ... Octomap_World.bt");
#endif

  obj_aligned_=false;

  pcl::PointCloud<pcl::PointXYZ>::Ptr targetPC (new pcl::PointCloud<pcl::PointXYZ>());


  if(!goal->object.color.compare("ff0000"))
  {
    ROS_INFO("Looking for: red objects");
    *targetPC += *finalVoxelizedRedPC;
  }
  else if (!goal->object.color.compare("00ff00"))
  {
    ROS_INFO("Looking for: green objects");
    *targetPC += *finalVoxelizedGreenPC;
  }
  else if (!goal->object.color.compare("0000ff"))
  {
    ROS_INFO("Looking for: blue objects");
    *targetPC += *finalVoxelizedBluePC;
  }
  else if (!goal->object.color.compare("00ffff"))
  {
    ROS_INFO("Looking for: cyan objects");
    *targetPC += *finalVoxelizedCyanPC;
  }
  else if (!goal->object.color.compare("ff00ff"))
  {
    ROS_INFO("Looking for: magenta objects");
    *targetPC += *finalVoxelizedMagentaPC;
  }
  else if (!goal->object.color.compare("ffff00"))
  {
    ROS_INFO("Looking for: yellow objects");
    *targetPC += *finalVoxelizedYellowPC;
  }



  pcl::console::print_highlight ("Passing target point cloud to pose estimator...\n");
  ros::Duration(2.0).sleep();


  // ==========================================================================================
  // ========== Code from Fabian ==========
  // Generate PointClouds from the given obstacles

  //ROS_INFO("Passing the voxelized point cloud to pose estimator...");
  ShapeGenerator<pcl::PointXYZ> shape_generator;
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_model (new pcl::PointCloud<pcl::PointXYZ>);
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

  else {

      vision_result_.abs_object_pose.position.x = transformation(0,3);
      vision_result_.abs_object_pose.position.y = transformation(1,3);
      vision_result_.abs_object_pose.position.z = transformation(2,3);
      vision_result_.abs_object_pose.orientation.w = tfqt.getW();
      vision_result_.abs_object_pose.orientation.x = tfqt.getX();
      vision_result_.abs_object_pose.orientation.y = tfqt.getY();
      vision_result_.abs_object_pose.orientation.z = tfqt.getZ();

      vision_feedback_.execution_time = ros::Time::now().sec;
      vision_server_.publishFeedback(vision_feedback_);

      vision_result_.object_detected=true;
      vision_server_.setSucceeded(vision_result_, "Goal configuration has been reached");

  }

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
void Vision::scan_with_pan_tilt()
{

  ROS_INFO("Entered Vision::scan_with_pan_tilt()...");

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

    ROS_INFO("Scanning the scene with pan tilt cam...");
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


      // move scene camera to hard-coded configuration
      if(!mpt->move_pan_tilt_abs(pan[panTiltCounter], tilt[panTiltCounter]))
      {
    	  msg_error("failed to move pan-tilt unit for counter %d",panTiltCounter);
      }
      ros::Duration(2.0).sleep();

      _scene_rgb_image = *(ros::topic::waitForMessage<sensor_msgs::Image>(camera_scene_rgb_topic, ros::Duration(1.0)));
      _scene_depth_image = *(ros::topic::waitForMessage<sensor_msgs::Image>(camera_scene_depth_topic, ros::Duration(1.0)));
      try
      {
        //timestamp = _scene_rgb_image.header.stamp;
        _cv_image = cv_bridge::toCvCopy(_scene_rgb_image, enc::BGR8);
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
        _cv_depthptr = cv_bridge::toCvCopy(_scene_depth_image, enc::TYPE_32FC1);
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


      // TODO: remove hardcoded values
      float fov_horizontal_scene_depth = 1.047;
      float fov_horizontal_scene_rgb = 1.047;

      scenePointCloud = new am_pointcloud(_cv_depthptr->image, fov_horizontal_scene_depth, _cv_image->image, fov_horizontal_scene_rgb);
      // create the initial point cloud based on depth image data
      initialPC = scenePointCloud->createInitialPointCloud();
      // align the initial point cloud with the RGB camera
      alignedPC = scenePointCloud->alignWithRGB(initialPC, CAM_SCENE);
      // Transform the point cloud to world coordinate
      worldPC = scenePointCloud->transformToWorld(alignedPC, CAM_SCENE);
      // filter out the robot, table surface and irrelevant points
      robotLessPC = scenePointCloud->removeRobotFromPointCloud(worldPC);
      threshPC = scenePointCloud->xyzTheresholdCloud(robotLessPC, 0.005); // 0.005: hard-coded z value to remove table surface

#ifdef OCTOMAP_COMPLETE_OLD
      // Create Octomap (whole scene with robot and Pantilt); also the man outside the table can be seen
      OctoCloud->reserve(worldPC->points.size());
      octomap::pointcloudPCLToOctomap(*worldPC,*OctoCloud);
      ROS_INFO("Starting to add actuall PointCloud to Octomap...");
      tree->insertScan(*OctoCloud, scenePointCloud->getSensorOriginScene(SCENE_CAM));
      ROS_INFO("Added actual PointCloud to Octomap");
#endif

#ifdef OCTOMAP_SERVER
      pcl::toROSMsg (*initialPC, msg);
      msg.header.frame_id = "S_DEPTH";
      msg.header.stamp = ros::Time::now();
      pub_3.publish (msg);
#endif

#ifdef OCTOMAP_REDUCED_OLD
      // Create Octomap without robot and things outside the table
      OctoCloud->reserve(threshPC->points.size());
      octomap::pointcloudPCLToOctomap(*threshPC,*OctoCloud);
      ROS_INFO("Starting to add actuall PointCloud to Octomap...");
      tree->insertScan(*OctoCloud, scenePointCloud->getSensorOriginScene(SCENE_CAM));
      ROS_INFO("Added actual PointCloud to Octomap");
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


      ROS_INFO("Finished sweep #%d", panTiltCounter);
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
    mpt->move_pan_tilt_abs(pan_zero, tilt_zero);
    ROS_INFO("pan camera scan complete.");

    // Voxelize result point clouds
    // for each point cloud, we have to create a voxelized version.
    pcl::VoxelGrid<pcl::PointXYZ> vg;

    ROS_INFO("Voxelization: global point cloud...");
    vg.setInputCloud(finalScenePC);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*finalVoxelizedPC);

    ROS_INFO("Voxelization: blue point cloud...");
    vg.setInputCloud(finalBluePC);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*finalVoxelizedBluePC);

    ROS_INFO("Voxelization: green point cloud...");
    vg.setInputCloud(finalGreenPC);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*finalVoxelizedGreenPC);

    ROS_INFO("Voxelization: red point cloud...");
    vg.setInputCloud(finalRedPC);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*finalVoxelizedRedPC);

    ROS_INFO("Voxelization: yellow point cloud...");
    vg.setInputCloud(finalYellowPC);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*finalVoxelizedYellowPC);

    ROS_INFO("Voxelization: cyan point cloud...");
    vg.setInputCloud(finalCyanPC);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*finalVoxelizedCyanPC);

    ROS_INFO("Voxelization: magenta point cloud...");
    vg.setInputCloud(finalMagentaPC);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*finalVoxelizedMagentaPC);

    ROS_INFO("Voxelization: Finished.");

  } // END if

  else {
    ROS_INFO("Waiting for camera msgs failed!");
  }

}


/*
 * This function takes care of generating several point clouds from TCP unit camera's image data.
 * There are a total of 7 point clouds:
 * - Complete (global), Blue, Green, Red, Yellow, Cyan, Magenta
 * All point clouds are voxelized at the end of the process for memory and performance purposes.
 *
 */
void Vision::scan_with_tcp()
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


  _tcp_rgb_image = *(ros::topic::waitForMessage<sensor_msgs::Image>(camera_tcp_rgb_topic, ros::Duration(1.0)));
  _tcp_depth_image = *(ros::topic::waitForMessage<sensor_msgs::Image>(camera_tcp_depth_topic, ros::Duration(1.0)));
  try
  {
    //timestamp = _scene_rgb_image.header.stamp;
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
  alignedPC = scenePointCloud->alignWithRGB(initialPC, CAM_TCP);
  // Transform the point cloud to world coordinate
  worldPC = scenePointCloud->transformToWorld(alignedPC, CAM_TCP);
  // filter out the robot, table surface and irrelevant points
  robotLessPC = scenePointCloud->removeRobotFromPointCloud(worldPC);
  threshPC = scenePointCloud->xyzTheresholdCloud(robotLessPC, 0.005); // 0.005: hard-coded z value to remove table surface

#ifdef OCTOMAP_COMPLETE_OLD
      // Create Octomap (whole scene with robot and Pantilt); also the man outside the table can be seen
      OctoCloud->reserve(worldPC->points.size());
      octomap::pointcloudPCLToOctomap(*worldPC,*OctoCloud);
      ROS_INFO("Starting to add actuall PointCloud to Octomap...");
      tree->insertScan(*OctoCloud, scenePointCloud->getSensorOriginScene(TCP_CAM));
      ROS_INFO("Added actual PointCloud to Octomap");
#endif

#ifdef OCTOMAP_SERVER
      pcl::toROSMsg (*initialPC, msg);
      msg.header.frame_id = "T_DEPTH";
      msg.header.stamp = ros::Time::now();
      pub_3.publish (msg);
#endif

#ifdef OCTOMAP_REDUCED_OLD
      // Create Octomap without robot and things outside the table
      OctoCloud->reserve(threshPC->points.size());
      octomap::pointcloudPCLToOctomap(*threshPC,*OctoCloud);
      ROS_INFO("Starting to add actuall PointCloud to Octomap...");
      tree->insertScan(*OctoCloud, scenePointCloud->getSensorOriginScene(TCP_CAM));
      ROS_INFO("Added actual PointCloud to Octomap");
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

  ROS_INFO("Voxelization: global point cloud...");
  vg.setInputCloud(threshPC);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter (*tempVoxelizedPC);

  ROS_INFO("Voxelization: blue point cloud...");
  vg.setInputCloud(finalBluePC);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter (*finalVoxelizedBluePC);

  ROS_INFO("Voxelization: green point cloud...");
  vg.setInputCloud(finalGreenPC);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter (*finalVoxelizedGreenPC);

  ROS_INFO("Voxelization: red point cloud...");
  vg.setInputCloud(finalRedPC);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter (*finalVoxelizedRedPC);

  ROS_INFO("Voxelization: yellow point cloud...");
  vg.setInputCloud(finalYellowPC);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter (*finalVoxelizedYellowPC);

  ROS_INFO("Voxelization: cyan point cloud...");
  vg.setInputCloud(finalCyanPC);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter (*finalVoxelizedCyanPC);

  ROS_INFO("Voxelization: magenta point cloud...");
  vg.setInputCloud(finalMagentaPC);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter (*finalVoxelizedMagentaPC);

  ROS_INFO("Voxelization: finished.");

  *finalVoxelizedPC += *tempVoxelizedPC;
  *finalVoxelizedBluePC += *tempVoxelizedBluePC;
  *finalVoxelizedGreenPC += *tempVoxelizedGreenPC;
  *finalVoxelizedRedPC += *tempVoxelizedRedPC;
  *finalVoxelizedYellowPC += *tempVoxelizedYellowPC;
  *finalVoxelizedCyanPC += *tempVoxelizedCyanPC;
  *finalVoxelizedMagentaPC += *tempVoxelizedMagentaPC;

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
        ROS_INFO("Show downsampled point cloud");
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
	while(nmbr_tries<5)
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

