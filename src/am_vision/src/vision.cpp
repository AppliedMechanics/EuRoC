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
#include <pcl/visualization/cloud_viewer.h>

//# define LOG_DEBUG
//# define LOG_INFO
//# define LOG_ERROR
# undef ANY_OBJ_ON_THIS_ZONE

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
	is_task5 = false;
	obj_aligned_ = false;
	isFinalClusterEmpty = true;

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

	// Object State
	obj_state_sub_ = nh_.subscribe("obj_state", 1000, &Vision::get_object_state_CB, this);

	// Take Image Service
	take_img_service_ = nh_.advertiseService("TakeImageService", &Vision::on_take_image_CB,this);
	// Check Zones Service
	check_zones_service_ = nh_.advertiseService("CheckZonesService", &Vision::on_check_zones_CB,this);
	// Reset Octomap Service
	reset_octomap_client_ = nh_.serviceClient<octomap_msgs::BoundingBoxQuery>("/octomap_server/clear_bbx");

	// Show PointCloud in rviz
	pub = nh_.advertise<sensor_msgs::PointCloud2> ("PointCloud_1", 1);
	pub_2 = nh_.advertise<sensor_msgs::PointCloud2> ("PointCloud_2", 1);
	pub_3 = nh_.advertise<sensor_msgs::PointCloud2> ("/PointCloud_Octomap", 1);

	// Initialize finalScenePC Pointcloud
	finalPC.reset (new pcl::PointCloud<pcl::PointXYZ> ());
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

	lastShapeToRemovePC.reset (new pcl::PointCloud<pcl::PointXYZ>);

	// leaf size voxels
	leaf_size = 0.0025;
	// memory efficiency!
	small_leaf_size = 0.0005;

	same_color_problem = false;

	// Create and set resolution of Octree
	tree = new octomap::OcTree(0.005);

	// Remove BoundingBox Query for octomap reset
	reset_octomap_bbx_srv_.request.min.x = -1.0;
	reset_octomap_bbx_srv_.request.min.y = -1.0;
	reset_octomap_bbx_srv_.request.min.z =  0.0;
	reset_octomap_bbx_srv_.request.max.x =  1.0;
	reset_octomap_bbx_srv_.request.max.y =  1.0;
	reset_octomap_bbx_srv_.request.max.z =  2.0;

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
	//  ROS_INFO("Entered Vision::RGB_CB");
	//  std::cout<<"Height: "<<image.height<<"Width: "<<image.width<<std::endl;
	_scene_rgb_image = image;
}

//ROS Callback for the scene DEPTH camera topic
void Vision::on_camera_scene_depth_CB(const sensor_msgs::Image &image){
	//  ROS_INFO("Entered Vision::DEPTH_CB");
	//  std::cout<<"Height: "<<image.height<<"Width: "<<image.width<<std::endl;
	_scene_depth_image = image;
}

//ROS Callback for the tcp RGB camera topic
void Vision::on_camera_tcp_rgb_CB(const sensor_msgs::Image &image){
	//  ROS_INFO("Entered Vision::RGB_CB");
	//  std::cout<<"Height: "<<image.height<<"Width: "<<image.width<<std::endl;
	_tcp_rgb_image = image;
}

//ROS Callback for the tcp DEPTH camera topic
void Vision::on_camera_tcp_depth_CB(const sensor_msgs::Image &image){
	//  ROS_INFO("Entered Vision::DEPTH_CB");
	//  std::cout<<"Height: "<<image.height<<"Width: "<<image.width<<std::endl;
	_tcp_depth_image = image;
}


bool Vision::on_take_image_CB(am_msgs::TakeImage::Request &req, am_msgs::TakeImage::Response &res)
{
	switch (req.camera) {
	case SCENE_CAM:
		std::cout<<"[VISION]callback: SCENE CAM!"<<std::endl;
		scan_with_pan_tilt(res,true);
		break;

	case TCP_CAM:
		std::cout<<"[VISION]callback: TCP CAM!"<<std::endl;
		ros::Duration(0.5).sleep();
		scan_with_tcp(res);
		std::cout<<"[VISION]TCP Pointcloud updated"<<std::endl;
		break;

	case SCENE_CAM_WITHOUT_ROBOT:
		std::cout<<"[VISION]callback: SCENE CAM WITHOUT ROBOT!"<<std::endl;
		scan_with_pan_tilt(res,false);
		break;
	default:
		ROS_WARN("Unknown camera type!");
		res.error_reason=fsm::VISION_ERROR;
		return false;
	}
	return true;
}

bool Vision::on_check_zones_CB(am_msgs::CheckZones::Request &req, am_msgs::CheckZones::Response &res)
{
	res.zones_occupied.resize(req.target_zones.size());

	pcl::PointXYZ zoneCenter;
	for(uint16_t ii=0;ii<req.target_zones.size();ii++) // iterate over target zones
	{
		// initialze the occupancy to zero
		res.zones_occupied[ii]=0;
		// set up target zone position and radius
		zoneCenter.x = req.target_zones[ii].position.x;
		zoneCenter.y = req.target_zones[ii].position.y;
		zoneCenter.z = req.target_zones[ii].position.z;
		float zoneRadius = req.target_zones[ii].max_distance;

		// iterate over point cloud to find possible points on target zone
		int minPointCounter = 0;
		float distanceX, distanceY;

		for (int i=0; i<finalVoxelizedPC->points.size(); i++)
		{
			distanceX = std::abs(finalVoxelizedPC->points[i].x - zoneCenter.x);
			distanceY = std::abs(finalVoxelizedPC->points[i].y - zoneCenter.y);
			if ( (distanceX*distanceX + distanceY*distanceY) < zoneRadius*zoneRadius )
				if (finalVoxelizedPC->points[i].z>0.01)
					minPointCounter++;
			if (minPointCounter > 5)
			{
				// found enough points on the zone to consider it as occupied
				ROS_INFO("[VISION]an object is detected on target zone #%d", ii);
				res.zones_occupied[ii]=1;
				break;
			}
		}
		if (minPointCounter < 5)
			ROS_INFO("[VISION]zone is free");
	}

	return true;
}



void Vision::get_object_state_CB(const am_msgs::ObjState::ConstPtr& msg_in)
{
	am_pointcloud *scenePointCloud;
	pcl::VoxelGrid<pcl::PointXYZ> vg;

	msg_info("starting get_object_state_CB()!");

	if(msg_in->obj_state==OBJ_GRIPPING)
	{

		pcl::PointCloud<pcl::PointXYZ>::Ptr tempFilledPC;
		tempFilledPC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
		tempFilledPC = fillPointCloud(lastShapeToRemovePC);

		ROS_INFO("[VISION]Obj state: GRIPPING");

		// remove the aligned shape from the
		// 1. complete point cloud
		// 2. respective color point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr tempPC;
		tempPC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
		tempPC = am_pointcloud::removeShape(finalPC, tempFilledPC);
		finalPC->clear();
		finalPC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
		*finalPC += *tempPC;

		finalVoxelizedPC->clear();
		finalVoxelizedPC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
		std::cout<<"[VISION]Voxelization: global point cloud..."<<std::endl;
		vg.setInputCloud(finalPC);
		vg.setLeafSize (leaf_size, leaf_size, leaf_size);
		vg.filter (*finalVoxelizedPC);

		ROS_INFO("Object removed from complete point cloud");

		tempPC->clear();
		tempPC.reset ( new pcl::PointCloud<pcl::PointXYZ> );

		if(!_currentGoal->object.color.compare("ff0000"))
		{
			// Goal: Red object
			tempPC = am_pointcloud::removeShape(finalRedPC, tempFilledPC);
			finalRedPC->clear();
			finalRedPC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
			*finalRedPC += *tempPC;

			finalVoxelizedRedPC->clear();
			finalVoxelizedRedPC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
			std::cout<<"[VISION]Voxelization: red point cloud..."<<std::endl;
			vg.setInputCloud(finalRedPC);
			vg.setLeafSize (leaf_size, leaf_size, leaf_size);
			vg.filter (*finalVoxelizedRedPC);


		}
		else if (!_currentGoal->object.color.compare("00ff00"))
		{
			// Goal: Green object
			tempPC = am_pointcloud::removeShape(finalGreenPC, tempFilledPC);
			finalGreenPC->clear();
			finalGreenPC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
			*finalGreenPC += *tempPC;

			finalVoxelizedGreenPC->clear();
			finalVoxelizedGreenPC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
			std::cout<<"[VISION]Voxelization: green point cloud..."<<std::endl;
			vg.setInputCloud(finalGreenPC);
			vg.setLeafSize (leaf_size, leaf_size, leaf_size);
			vg.filter (*finalVoxelizedGreenPC);

		}
		else if (!_currentGoal->object.color.compare("0000ff"))
		{
			// Goal: Blue object
			tempPC = am_pointcloud::removeShape(finalBluePC, tempFilledPC);
			finalBluePC->clear();
			finalBluePC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
			*finalBluePC += *tempPC;

			finalVoxelizedBluePC->clear();
			finalVoxelizedBluePC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
			std::cout<<"[VISION]Voxelization: blue point cloud..."<<std::endl;
			vg.setInputCloud(finalBluePC);
			vg.setLeafSize (leaf_size, leaf_size, leaf_size);
			vg.filter (*finalVoxelizedBluePC);

		}
		else if (!_currentGoal->object.color.compare("00ffff"))
		{
			// Goal: Cyan object
			tempPC = am_pointcloud::removeShape(finalCyanPC, tempFilledPC);
			finalCyanPC->clear();
			finalCyanPC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
			*finalCyanPC += *tempPC;

			finalVoxelizedCyanPC->clear();
			finalVoxelizedCyanPC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
			std::cout<<"[VISION]Voxelization: cyan point cloud..."<<std::endl;
			vg.setInputCloud(finalCyanPC);
			vg.setLeafSize (leaf_size, leaf_size, leaf_size);
			vg.filter (*finalVoxelizedCyanPC);

		}
		else if (!_currentGoal->object.color.compare("ff00ff"))
		{
			// Goal: Magenta object
			tempPC = am_pointcloud::removeShape(finalMagentaPC, tempFilledPC);
			finalMagentaPC->clear();
			finalMagentaPC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
			*finalMagentaPC += *tempPC;

			finalVoxelizedMagentaPC->clear();
			finalVoxelizedMagentaPC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
			std::cout<<"[VISION]Voxelization: magenta point cloud..."<<std::endl;
			vg.setInputCloud(finalMagentaPC);
			vg.setLeafSize (leaf_size, leaf_size, leaf_size);
			vg.filter (*finalVoxelizedMagentaPC);

		}
		else if (!_currentGoal->object.color.compare("ffff00"))
		{
			// Goal: Yellow object
			tempPC = am_pointcloud::removeShape(finalYellowPC, tempFilledPC);
			finalYellowPC->clear();
			finalYellowPC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
			*finalYellowPC += *tempPC;

			finalVoxelizedYellowPC->clear();
			finalVoxelizedYellowPC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
			std::cout<<"[VISION]Voxelization: yellow point cloud..."<<std::endl;
			vg.setInputCloud(finalYellowPC);
			vg.setLeafSize (leaf_size, leaf_size, leaf_size);
			vg.filter (*finalVoxelizedYellowPC);

		}

		std::cout<<"[VISION]Voxelization: finished."<<std::endl;

		// Update the OctoMap

		try{
			if (!(reset_octomap_client_.call(reset_octomap_bbx_srv_)))
			{
				msg_error("reset octomap service failed!");
				return;
			}
			else
				msg_info("octomap successfully resetted.");
		}
		catch (...)
		{
			msg_error("reset octomap service failed! TRYCATCH");
		}

#ifdef OCTOMAP_SERVER

		//ros::Duration(2.0).sleep();
		pcl::PointCloud<pcl::PointXYZ>::Ptr filledForOctomapPC;
		filledForOctomapPC = fillPointCloud(finalVoxelizedPC);

		pcl::toROSMsg (*filledForOctomapPC, msg);
		msg.header.frame_id = "/Origin";
		msg.header.stamp = ros::Time::now();
		pub_3.publish(msg);
		ROS_INFO("Octomap updated");
#endif
	}
}


/*
 * END CALLBACKS
 */
/////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr Vision::fake_puzzle_fixture() {


	//RETURN JUST THE FOUR CORNERS OF THE SQUARE//

	Eigen::Quaternion<double> q2;
	Eigen::Vector3d translation2;

	int indx = 0;
	float side1_wall = 0.05 / 2;
	float side2_wall = 0.05 * 7.5;


	pcl::PointCloud<pcl::PointXYZ>::Ptr shape_model(
			new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointXYZ a;
	pcl::PointXYZ b;
	pcl::PointXYZ c;
	pcl::PointXYZ d;


	a.x = -side1_wall;
	a.y = -side1_wall;
	a.z = 0;
	shape_model->points.push_back(a);

	b.x = -side1_wall;
	b.y = side2_wall-side1_wall;
	b.z = 0;
	shape_model->points.push_back(b);

	c.x = side2_wall-side1_wall;
	c.y = side2_wall-side1_wall;
	c.z = 0;
	shape_model->points.push_back(c);

	d.x = side2_wall-side1_wall;
	d.y = -side1_wall;
	d.z = 0;
	shape_model->points.push_back(d);


	double xp = puzzle_fixture_position_vector[indx];
	double xo = puzzle_fixture_orientation_vector[indx];
	++indx;
	double yp = puzzle_fixture_position_vector[indx];
	double yo = puzzle_fixture_orientation_vector[indx];
	++indx;
	double zp = puzzle_fixture_position_vector[indx];
	double zo = puzzle_fixture_orientation_vector[indx];
	++indx;
	double wo = puzzle_fixture_orientation_vector[indx];

	translation2[0] = xp; //-(side1 / 2) + xp;
	translation2[1] = yp; //-(side2 / 2) + yp;
	translation2[2] = zp; //-(side3 / 2) + zp;

	q2 = Eigen::Quaternion<double>(wo, xo, yo, zo);

	pcl::transformPointCloud(*shape_model, *shape_model, translation2, q2);

	return shape_model;

	/*
	//CREATE THE COMPLETE PUZZLE FIXTURE//

	Eigen::Quaternion<double> q2;
	Eigen::Vector3d translation2;
	double step_size = leaf_size;
	float side1_floor = 0.05 * 7;
	float side2_floor = 0.05 * 7;
	float side3_floor = 0.05 / 4;
	float side1_wall = 0.05 / 4;
	float side2_wall = 0.05 * 7;
	float side3_wall = 0.05 * 1.8;

	ShapeGenerator<pcl::PointXYZ> shape_generator2;
	ShapeGenerator<pcl::PointXYZ> shape_generator_wall1;
	ShapeGenerator<pcl::PointXYZ> shape_generator_wall2;
	pcl::PointCloud<pcl::PointXYZ>::Ptr shape_model(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr floor_model(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr wall1_model(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr wall2_model(
			new pcl::PointCloud<pcl::PointXYZ>);


	shape_generator2.setOutputCloud(floor_model);
	shape_generator2.generateBox(step_size,
			Eigen::Vector3f(-side1_wall, -side1_wall, (-side3_floor / 2)),
			side1_floor * Eigen::Vector3f::UnitX(),
			side2_floor * Eigen::Vector3f::UnitY(),
			side3_floor * Eigen::Vector3f::UnitZ(), 0.0f, true, true);
	std::cout << "Object Size is 1 and is not Compound in fixture" << std::endl;


	shape_generator2.setOutputCloud(wall1_model);
	shape_generator2.generateBox(step_size,
			Eigen::Vector3f(-side1_wall, -side1_wall, (-side3_floor / 2)),
			side1_wall * Eigen::Vector3f::UnitX(),
			side2_wall * Eigen::Vector3f::UnitY(),
			side3_wall * Eigen::Vector3f::UnitZ(), 0.0f, true, true);
	std::cout << "Object Size is 1 and is not Compound in fixture" << std::endl;


	shape_generator2.setOutputCloud(wall2_model);
	shape_generator2.generateBox(step_size,
			Eigen::Vector3f(-side1_wall, -side1_wall, (-side3_floor / 2)),
			side2_wall * Eigen::Vector3f::UnitX(),
			side1_wall * Eigen::Vector3f::UnitY(),
			side3_wall * Eigen::Vector3f::UnitZ(), 0.0f, true, true);
	std::cout << "Object Size is 1 and is not Compound in fixture" << std::endl;

	 *shape_model += *floor_model;
	 *shape_model += *wall1_model;
	 *shape_model += *wall2_model;

	//	pcl::toROSMsg(*shape_model, msg);
	//	msg.header.frame_id = "/Origin";
	//	msg.header.stamp = ros::Time::now();
	//	pub.publish(msg);
	//	ros::Duration(2).sleep();


	//transform Box PC to valid position
	int indx = 0;


	double xp = puzzle_fixture_position_vector[indx];
	std::cout << "xp" << xp << std::endl;
	double xo = puzzle_fixture_orientation_vector[indx];
	std::cout << "xo" << xo << std::endl;
	++indx;
	double yp = puzzle_fixture_position_vector[indx];
	std::cout << "yp" << yp << std::endl;
	double yo = puzzle_fixture_orientation_vector[indx];
	std::cout << "yo" << yo << std::endl;
	++indx;
	double zp = puzzle_fixture_position_vector[indx];
	std::cout << "zp" << zp << std::endl;
	double zo = puzzle_fixture_orientation_vector[indx];
	std::cout << "zo" << xp << std::endl;
	++indx;
	double wo = puzzle_fixture_orientation_vector[indx];
	std::cout << "wo" << wo << std::endl;

	translation2[0] = xp; //-(side1 / 2) + xp;
	translation2[1] = yp; //-(side2 / 2) + yp;
	translation2[2] = zp; //-(side3 / 2) + zp;

	q2 = Eigen::Quaternion<double>(wo, xo, yo, zo);

	pcl::transformPointCloud(*shape_model, *shape_model, translation2, q2);

//	pcl::VoxelGrid<pcl::PointXYZ> vg;
//    vg.setInputCloud(shape_model);
//    vg.setLeafSize(leaf_size * 4, leaf_size * 4, leaf_size * 4);
//    vg.filter(*shape_model);
//    pcl::toROSMsg(*shape_model, msg);
//
//    msg.header.frame_id = "/Origin";
//    msg.header.stamp = ros::Time::now();
//    pub.publish (msg);

	return shape_model;
	//	 pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	//    	   viewer.showCloud (shape_model);
	//  	   while (!viewer.wasStopped ())
	//	   {
	//	   cout<<"Loopy";
	// }*/

}

void Vision::fake_puzzle_fixture_param() {
	std::string puzzle_fixture1 = "fixture_pose_position_";
	std::string puzzle_fixture2 = "fixture_pose_orientation_";
	vector<std::string> coord;

	std::string x = "x_";
	coord.push_back(x);
	std::string y = "y_";
	coord.push_back(y);
	std::string z = "z_";
	coord.push_back(z);
	std::string w = "w_";
	coord.push_back(w);

	for (int i = 0; i < 3; i++) {
		std::string searchwordp = "";
		std::string searchwordo = "";
		std::string keyp = "";
		std::string keyo = "";

		searchwordp = puzzle_fixture1 + coord[i];
		searchwordo = puzzle_fixture2 + coord[i];

		if (nh_.searchParam(searchwordp, keyp)) {
			double found_val = 0;
			nh_.getParam(keyp, found_val);

			puzzle_fixture_position_vector.push_back(found_val);
			std::cout << "[VISION]Puzzle Fixture Position: "
					<< puzzle_fixture_position_vector.back() << std::endl;
		}
		if (nh_.searchParam(searchwordo, keyo)) {
			double found_val2 = 0;
			nh_.getParam(keyo, found_val2);

			puzzle_fixture_orientation_vector.push_back(found_val2);
			std::cout << "[VISION]Puzzle Fixture Orientation: "
					<< puzzle_fixture_orientation_vector.back() << std::endl;
		}

	}

	std::string searchword = "";
	searchword = puzzle_fixture2 + coord[3];
	std::string key = "";
	if (nh_.searchParam(searchword, key)) {
		double found_val3 = 0;
		nh_.getParam(key, found_val3);
		puzzle_fixture_orientation_vector.push_back(found_val3);
		std::cout << "[VISION]Puzzle Fixture Orientation: "
				<< puzzle_fixture_orientation_vector.back() << std::endl;
	}

}
////////////////////////////////////

pcl::PointCloud<pcl::PointXYZ>::Ptr Vision::removeInliers(
		pcl::PointCloud<pcl::PointXYZ>::Ptr object_input) {
	std::cout
	<< "[VISION] removeInliers::Number of Points in object input before removing"
	<< (object_input->points.size()) << std::endl;

	// Remove NANs from PointCloud scene
	object_input->is_dense = false;
	std::vector<int> index;
	pcl::removeNaNFromPointCloud(*object_input, *object_input, index);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(object_input);
	std::vector<int> pointIdxRadiusSearch;
	std::vector<int> pointIDs;
	std::vector<float> pointRadiusSquaredDistance;

	double radius = leaf_size * 0.5;
	double minz;
	minz = object_input->points[0].z;
	for (size_t f = 0; f < (object_input->points.size()); f++) {
		if (pcl_isfinite(object_input->points[f].x)
				|| pcl_isfinite(object_input->points[f].y)
				|| pcl_isfinite(object_input->points[f].z)) {
			if ((object_input->points[f].z) < minz) //(object_input->points[mini_index].z))
				// mini_index = f;
				minz = object_input->points[f].z;
		}
	}

	std::cout << "[VISION]Min Z Size " << minz << std::endl;

	//Remove Inliers
	for (size_t i = 0; i < (object_input->points.size()); i++) {

		pcl::PointXYZ searchPoint;
		searchPoint.x = object_input->points[i].x;
		searchPoint.y = object_input->points[i].y;
		searchPoint.z = object_input->points[i].z;

		if (object_input->points[i].z < minz + 0.01) //(object_input->points[mini_index].z) + 0.01)
		{
			pointIDs.push_back(i);
			continue;
		}

		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
				pointRadiusSquaredDistance) > 0) {
			if ((pointIdxRadiusSearch.size() > 1))
				pointIDs.push_back(i);
		} //End of Search If
	} //End of For

	std::cout << "[VISION]PointsIDs " << pointIDs.size() << std::endl;

	//Remove indices ^ ^
	for (int j = 0; j < pointIDs.size(); ++j) {
		object_input->points[pointIDs[j]].x =
				std::numeric_limits<float>::quiet_NaN();
		object_input->points[pointIDs[j]].y =
				std::numeric_limits<float>::quiet_NaN();
		object_input->points[pointIDs[j]].z =
				std::numeric_limits<float>::quiet_NaN();
	}

	std::vector<int> indx;
	pcl::removeNaNFromPointCloud(*object_input, *object_input, indx);

	//pcl::toROSMsg(*object_input, msg);
	//msg.header.frame_id = "/Origin";
	//msg.header.stamp = ros::Time::now();
	//pub.publish(msg);
	//ros::Duration(20).sleep(); // sleep for half a second

	std::cout
	<< "[VISION] removeInliers::Number of Points in object input after all removal"
	<< object_input->points.size() << std::endl;
	return object_input;

}

//////////////////////////////////////


void Vision::handle(const am_msgs::VisionGoal::ConstPtr &goal)
{
	ROS_INFO("Entered Vision::handle()");
	_currentGoal = goal;
	// targetPC is the point cloud which will be sent to the alignment function
	pcl::PointCloud<pcl::PointXYZ>::Ptr targetPC (new pcl::PointCloud<pcl::PointXYZ>());

	// Get the task number
	std::string key;
	if(nh_.searchParam("active_task_number_", key))
	{
		nh_.getParam(key, task_nr);
		//std::cout<<"[VISION]active task number: "<<task_nr<<std::endl;
	}
	else
	{
		ROS_WARN("search for parameter failed!");
	}



	// START TASK 6 PROCEDURE
	if (task_nr == 6)
	{
		ROS_INFO("[VISION]now working on task: %d", task_nr);

		if (_currentGoal->object.shape.size() == 1)
		{
			ROS_INFO("wait for the cube to be dropped");
			ros::Duration(2.0).sleep();
			// --> move the pan tilt
			// create point cloud
			tic = ros::Time().now();
			targetPC = get_task6_cloud();

			ROS_INFO("show targetPC in handle()");
			// publish final voxelized point cloud
			pcl::toROSMsg (*targetPC, msg);
			msg.header.frame_id = "/Origin";
			msg.header.stamp = ros::Time::now();
			pub.publish (msg);
			//ros::Duration(2.0).sleep();

			// --> fast_cube_alignment(scene_input)
			transformation = fast_cube_alignment(targetPC);

			tac = ros::Time().now();
			std::cout << "Total time: "<< tac - tic << "seconds" << std::endl;

			if(obj_aligned_==false)
			{
				vision_server_.setPreempted(vision_result_,"Alignment failed.");
				vision_result_.error_reason=fsm::POSE_NOT_FOUND;
				return;
			}

			// get Quaternion values from rotation-matrix
			tf::Matrix3x3 rotation;
			rotation.setValue((double)transformation(0,0), (double)transformation(0,1), (double)transformation(0,2),
					(double)transformation(1,0), (double)transformation(1,1), (double)transformation(1,2),
					(double)transformation(2,0), (double)transformation(2,1), (double)transformation(2,2));

			rotation.getRotation(tfqt);

			// sending result back to StateMachine
			vision_result_.abs_object_pose.position.x = transformation(0,3);
			vision_result_.abs_object_pose.position.y = transformation(1,3);
			vision_result_.abs_object_pose.position.z = transformation(2,3);
			vision_result_.abs_object_pose.orientation.w = tfqt.getW();
			vision_result_.abs_object_pose.orientation.x = tfqt.getX();
			vision_result_.abs_object_pose.orientation.y = tfqt.getY();
			vision_result_.abs_object_pose.orientation.z = tfqt.getZ();

			vision_result_.stamp = finalTimeStamp;

			vision_result_.object_detected=true;
			vision_server_.setSucceeded(vision_result_, "Goal configuration has been reached");

			targetPC->clear(); // Get ready for the next round
			return;
		}
		else
		{
			ROS_ERROR("Object description does not match a single Cube");
			vision_server_.setPreempted(vision_result_,"Alignment failed.");
			vision_result_.error_reason=fsm::DATA_ERROR;
			return;
		}
	} // END TASK 6 PROCEDURE


	// Tasks other than #6 will have the same starting procedure: fetch the color filtered point cloud

	pcl::toROSMsg (*finalVoxelizedPC, msg);
	msg.header.frame_id = "/Origin";
	msg.header.stamp = ros::Time::now();
	pub.publish (msg);

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

#ifdef OUTPUT_OCTOMAP_BIN
		tree->writeBinary("/home/euroc_student/EUROC_SVN_STUDENT/vision_octomap/Octomap.bt");
		ROS_INFO("Tree was written to Binary File ... Octomap_World.bt");
#endif

		obj_aligned_ = false;

		std::cout<<"[VISION]Looking for: "<<goal->object.name<<", #"<<goal->object.color<<std::endl;

		// check for task number (different tasks --> different approaches)
		if ( task_nr == 1 || task_nr == 2 || task_nr == 3 || task_nr == 4 )
		{
			ROS_INFO("[VISION]now working on task: %d", task_nr);
			if(goal->object.nr_shapes == 1)
				if(!goal->object.shape[0].type.compare("box"))
				{
					std::cout<<"[VISION]current object is a single cube"<<std::endl;
					isSingleCube = true;
				}
		}

		// START TASK 4 PREPARATION
		if (task_nr == 4)
		{
			// cluster the complete point cloud and filter it by size
			pcl::PointCloud<pcl::PointXYZ>::Ptr filteredClusterPC;
			filteredClusterPC = find_clusters(finalVoxelizedPC, targetPC, goal);

			// filteredClusterPC can contain 3 types of PointClouds:
			// 1. (best-case) just the object of the specified color
			// 2. object plus some other random points (noise, bad color filtering due to dilation)
			// 3. (worst-case) all the objects-obstacles of the specified color

			// pass filteredClusterPC to targetPC
			targetPC->clear();
			targetPC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
			*targetPC += *filteredClusterPC;

			// publish final thresholded point cloud
			pcl::toROSMsg (*targetPC, msg);
			msg.header.frame_id = "/Origin";
			msg.header.stamp = ros::Time::now();
			pub.publish (msg);
			ros::Duration(2.0).sleep();

		} // END TASK 4 PREPARATION


		pcl::console::print_highlight ("Passing target point cloud to pose estimator...\n");
		ros::Duration(2.0).sleep();

		// publish final thresholded point cloud
		pcl::toROSMsg (*targetPC, msg);
		msg.header.frame_id = "/Origin";
		msg.header.stamp = ros::Time::now();
		pub.publish (msg);


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

			else if (!goal->object.shape[i].type.compare("box")) {
				//generate box PC (PointCloud)
				step_size = leaf_size;
				if (goal->object.shape.size() == 1)
					shape_generator.generateBox(step_size,
							Eigen::Vector3f(0.0f, 0.0f, 0.0f),
							goal->object.shape[i].size[0]
							                           * Eigen::Vector3f::UnitX(),
							                           goal->object.shape[i].size[1]
							                                                      * Eigen::Vector3f::UnitY(),
							                                                      goal->object.shape[i].size[2]
							                                                                                 * Eigen::Vector3f::UnitZ(), 0.0f, false,
							                                                                                 is_task5);
				else
					shape_generator.generateBox(step_size,
							Eigen::Vector3f(0.0f, 0.0f, 0.0f),
							goal->object.shape[i].size[0]
							                           * Eigen::Vector3f::UnitX(),
							                           goal->object.shape[i].size[1]
							                                                      * Eigen::Vector3f::UnitY(),
							                                                      goal->object.shape[i].size[2]
							                                                                                 * Eigen::Vector3f::UnitZ(), 0.0f, true,
							                                                                                 is_task5);

				//transform Cylinder PC to valid position
				q = Eigen::Quaternion<double>(goal->object.shape[i].pose.orientation.w, goal->object.shape[i].pose.orientation.x, goal->object.shape[i].pose.orientation.y, goal->object.shape[i].pose.orientation.z);
				translation[0] = -(goal->object.shape[i].size[0]/2) + goal->object.shape[i].pose.position.x;
				translation[1] = -(goal->object.shape[i].size[1]/2) + goal->object.shape[i].pose.position.y;
				translation[2] = -(goal->object.shape[i].size[2]/2) + goal->object.shape[i].pose.position.z;

				pcl::transformPointCloud(*shape_model, *shape_model, translation, q);

				//On task 5 as long as we do not have a single cube, downsize here
				if ((task_nr == 5) && (goal->object.nr_shapes != 1)) {
					pcl::VoxelGrid<pcl::PointXYZ> vg;
					vg.setInputCloud(shape_model);
					vg.setLeafSize(leaf_size * 4, leaf_size * 4, leaf_size * 4);
					vg.filter(*shape_model);
				}

				*object_model += *shape_model;


				shape_model->clear();
			}
			else
			{
				// Unknown shape type
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

		if (is_task5) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr hollow_object(
					new pcl::PointCloud<pcl::PointXYZ>);

			hollow_object = removeInliers(object_model);

			object_model.reset(new pcl::PointCloud<pcl::PointXYZ>);
			*object_model += *hollow_object;

		}

		// Align observed point cloud with modeled object
		if (goal->object.shape.size() == 1
				&& !goal->object.shape[0].type.compare("box"))
			transformation = align_PointClouds(object_model, targetPC, true,
					false, false);
		else if (goal->object.shape.size() == 1
				&& !goal->object.shape[0].type.compare("cylinder"))
			transformation = align_PointClouds(object_model, targetPC, false,
					true, false);
		else
			transformation = align_PointClouds(object_model, targetPC, false,
					false, is_task5);

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
		msg.header.frame_id = "/Origin";
		msg.header.stamp = ros::Time::now();
		pub_2.publish (msg);

		//Publish the colored point cloud
		//		pcl::toROSMsg(*targetPC, msg);
		//		msg.header.frame_id = "/Origin";
		//		msg.header.stamp = ros::Time::now();
		//		pub.publish(msg);
		//		ros::Duration(2.0).sleep();

		//Publish ideal fixture
		//		pcl::toROSMsg(*fake_puzzle, msg);
		//		msg.header.frame_id = "/Origin";
		//		msg.header.stamp = ros::Time::now();
		//		pub_2.publish(msg);
		//		ros::Duration(2.0).sleep();

		// Prepare the lastShapeToRemovePC for pointcloud update/removal
		lastShapeToRemovePC->clear();
		lastShapeToRemovePC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
		*lastShapeToRemovePC += *test;

		//get quaternion from rotation-matrix
		tf::Matrix3x3 rotation;
		rotation.setValue((double)transformation(0,0), (double)transformation(0,1), (double)transformation(0,2),
				(double)transformation(1,0), (double)transformation(1,1), (double)transformation(1,2),
				(double)transformation(2,0), (double)transformation(2,1), (double)transformation(2,2));

		rotation.getRotation(tfqt);
		// ===================================== END: Code from Fabian ===================================
		// ===============================================================================================

		//------------------------------------------ SEND RESULT ------------------------------------------//

		vision_result_.abs_object_pose.position.x = transformation(0,3);
		vision_result_.abs_object_pose.position.y = transformation(1,3);
		vision_result_.abs_object_pose.position.z = transformation(2,3);
		vision_result_.abs_object_pose.orientation.w = tfqt.getW();
		vision_result_.abs_object_pose.orientation.x = tfqt.getX();
		vision_result_.abs_object_pose.orientation.y = tfqt.getY();
		vision_result_.abs_object_pose.orientation.z = tfqt.getZ();


		vision_result_.stamp = finalTimeStamp;

		vision_result_.object_detected=true;
		vision_server_.setSucceeded(vision_result_, "Goal configuration has been reached");

		targetPC->clear();
		return;
	}
	else if(goal->mode == CLOSE_RANGE_POSE_ESTIMATION)
	{
		std::cout<<"[VISION]initializing close range pose estimation"<<std::endl;

		// Close Range Pose Estimation for Cube
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


				if ( (std::abs(OptRotationRadians-yaw) > 5*M_PI_4) && (std::abs(OptRotationRadians-yaw) < 7*M_PI_4))
				{
					std::cout<<"close range rotation: "<<OptRotationRadians<<std::endl;
					std::cout<<"yaw: "<<yaw<<std::endl;
					OptRotationRadians = OptRotationRadians - ((3*M_PI_2)*(sgn(OptRotationRadians-yaw)));
					std::cout<<"corrected rotation: "<<OptRotationRadians<<std::endl;
				}

				if ( (std::abs(OptRotationRadians-yaw) > 3*M_PI_4) && (std::abs(OptRotationRadians-yaw) < 5*M_PI_4))
				{
					std::cout<<"close range rotation: "<<OptRotationRadians<<std::endl;
					std::cout<<"yaw: "<<yaw<<std::endl;
					OptRotationRadians = OptRotationRadians - ((2*M_PI_2)*(sgn(OptRotationRadians-yaw)));
					std::cout<<"corrected rotation: "<<OptRotationRadians<<std::endl;
				}

				if ( (std::abs(OptRotationRadians-yaw) > M_PI_4) && (std::abs(OptRotationRadians-yaw) < 3*M_PI_4))
				{
					std::cout<<"close range rotation: "<<OptRotationRadians<<std::endl;
					std::cout<<"yaw: "<<yaw<<std::endl;
					OptRotationRadians = OptRotationRadians - ((M_PI_2)*(sgn(OptRotationRadians-yaw)));
					std::cout<<"corrected rotation: "<<OptRotationRadians<<std::endl;
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

				Eigen::Matrix4f transform_close_range = Eigen::Matrix4f::Identity();
				transform_close_range (0,0) = std::cos(OptRotationRadians);
				transform_close_range (0,1) = (-1)*std::sin(OptRotationRadians);
				transform_close_range (1,0) = std::sin(OptRotationRadians);
				transform_close_range (1,1) = std::cos(OptRotationRadians);
				transform_close_range (0,3) = transformation (0,3);
				transform_close_range (1,3) = transformation (1,3);
				transform_close_range (2,3) = transformation (2,3);

				std::cout<<"[VISION]show aligned point cloud based on close range image"<<std::endl;
				pcl::PointCloud<pcl::PointXYZ>::Ptr closeRangePC (new pcl::PointCloud<pcl::PointXYZ>);
				pcl::copyPointCloud(*object_model, *closeRangePC);
				pcl::transformPointCloud(*closeRangePC, *closeRangePC, transform_close_range);
				pcl::toROSMsg (*closeRangePC, msg);
				msg.header.frame_id = "/Origin";
				msg.header.stamp = ros::Time::now();
				pub_2.publish (msg);

				pcl::transformPointCloud(*object_model, *object_model, transformation);

				int diffOldPose = verify_close_range_pose(targetPC, object_model);
				int diffNewPose = verify_close_range_pose(targetPC, closeRangePC);
				std::cout<<"Diff = "<<std::abs( diffOldPose - diffNewPose )<<std::endl;
				// check which transformed model fits better on the observed point cloud
				if ( ( diffNewPose - diffOldPose ) < 15 )
				{
					std::cout<<"[VISION]new pose is better!"<<std::endl;
					std::cout<<"[VISION]Passing the new pose to grasping node"<<std::endl;

					// Prepare the lastShapeToRemovePC for pointcloud update/removal
					lastShapeToRemovePC->clear();
					lastShapeToRemovePC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
					*lastShapeToRemovePC += *closeRangePC;

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
			isSingleCube = false;
		} // END Close Range Pose Estimation for Cube

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
		std::cout<<"[VISION]Wait for 3 sec..."<<std::endl;
		ros::Duration(3.0).sleep();
		std::cout<<"[VISION]Done!"<<std::endl;
		std::cout<<"[VISION]check if object is on target zone"<<std::endl;

		pcl::PointXYZ targetZone;
		targetZone.x = goal->target_zone.position.x;
		targetZone.y = goal->target_zone.position.y;
		targetZone.z = goal->target_zone.position.z;
		uint16_t objectInZone = verify_object_inside_zone(goal->object.color, targetZone, goal->target_zone.max_distance);

		if (objectInZone == 2)
		{
			std::cout<<"[VISION]Object_On_Target Verification: PASSED."<<std::endl;
			vision_result_.object_in_zone = true;
			vision_result_.object_detected = true;
			vision_server_.setSucceeded(vision_result_, "Goal configuration has been reached");
		}
		else if(objectInZone == 1)
		{
			std::cout<<"[VISION]Object_On_Target Verification: PASSED with minor error"<<std::endl;
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
 * 8 fixed configurations has been set for the camera in order to get the most possible complete view of the table.
 *
 * All point clouds are voxelized at the end of the process for memory and performance purposes.
 *
 */
void Vision::scan_with_pan_tilt(am_msgs::TakeImage::Response &res, bool scan_full)
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
		int maxPanTiltCounter = 8;
		if (!scan_full)
			maxPanTiltCounter = 1;

		// store initial pan-tilt values
		double pan_zero = mpt->get_pan();
		double tilt_zero = mpt->get_tilt();

		// Initialize pan + tilt values
		double pan[9]  = {0.000, -0.300, 0.000, 0.300, 0.000, 0.000, -0.380, 0.380, 0.0};
		double tilt[9] = {1.250,  0.800, 0.700, 0.800, 0.800, 0.300,  0.300, 0.300, 0.34};

		ROS_INFO("[VISION]Scanning the scene with pan tilt cam...");
		while(panTiltCounter < maxPanTiltCounter)
			//    while(panTiltCounter < 3)
		{
			if(panTiltCounter==0)
			{
				// Get the task number
				std::string key;
				if(nh_.searchParam("active_task_number_", key))
				{
					nh_.getParam(key, task_nr);
					//std::cout<<"[VISION]active task number: "<<task_nr<<std::endl;
				}
				else
				{
					ROS_WARN("search for parameter failed!");
				}

				// START TASK 5 PREPARATION
				if (task_nr == 5) {
					is_task5 = true;
					std::cout << "[VISION]now working on task: " << task_nr
							<< std::endl;
					fake_puzzle_fixture_param();
				} // END TASK 5 PREPARATION

			}

			if (!scan_full)
				panTiltCounter = 8;

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
			pcl::PointCloud<pcl::PointXYZ>::Ptr filledForOctomapPC;

			ros::Time stepTimeStampRGB;
			ros::Time stepTimeStampDepth;

			// move scene camera to hard-coded configuration
			if(!mpt->move_pan_tilt_abs(pan[panTiltCounter], tilt[panTiltCounter]))
			{
				msg_error("failed to move pan-tilt unit for counter %d",panTiltCounter);
			}
			ros::Duration(0.5).sleep();

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
				return;
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
				return;
			}

			// Apply HSV filtering
			ImageFilter hsvFilter(_cv_image->image);

			/*
			 * Create threshold for six colors
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

			if(task_nr==5)
			{
				std::cout << "about to delete the puzzle fixture"<< std::endl;
				pcl::PointCloud<pcl::PointXYZ>::Ptr fake_puzzle(new pcl::PointCloud<pcl::PointXYZ>);
				fake_puzzle = fake_puzzle_fixture();
				pcl::PointCloud<pcl::PointXYZ>::Ptr scene_without_fixture(new pcl::PointCloud<pcl::PointXYZ>);
				scene_without_fixture = scenePointCloud->removeSquareFromPointCloud(threshPC,fake_puzzle->points[0],fake_puzzle->points[1],fake_puzzle->points[2],fake_puzzle->points[3]);
				threshPC.reset(new pcl::PointCloud<pcl::PointXYZ>);
				threshPC = scenePointCloud->removeSquareFromPointCloud(scene_without_fixture,fake_puzzle->points[0],fake_puzzle->points[1],fake_puzzle->points[2],fake_puzzle->points[3]);

			}

			filledForOctomapPC = fillPointCloud(threshPC);

#ifdef OCTOMAP_COMPLETE_OLD
			// Create Octomap (whole scene with robot and Pantilt); also the man outside the table can be seen
			OctoCloud->reserve(worldPC->points.size());
			octomap::pointcloudPCLToOctomap(*worldPC,*OctoCloud);
			std::cout<<"[VISION]Starting to add actuall PointCloud to Octomap..."<<std::endl;
			tree->insertScan(*OctoCloud, scenePointCloud->getSensorOriginScene(SCENE_CAM));
			std::cout<<"[VISION]Added actual PointCloud to Octomap"<<std::endl;
#endif

#ifdef OCTOMAP_SERVER
			pcl::toROSMsg (*filledForOctomapPC, msg);
			msg.header.frame_id = "/Origin";
			msg.header.stamp = ros::Time::now();
			pub_3.publish (msg);
#endif

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


			ROS_INFO("[VISION]Finished sweep %d", panTiltCounter);

			// Set the time stamp of current image as the final stamp
			finalTimeStamp = stepTimeStampRGB;

			// set counter for next pan-tilt scan
			panTiltCounter++;

			// publish final thresholded point cloud
			pcl::toROSMsg (*finalScenePC, msg);
			msg.header.frame_id = "/Origin";
			msg.header.stamp = ros::Time::now();
			pub.publish (msg);

			delete scenePointCloud;

		} // END While

		// Move camera to initial configuration
		if(false == mpt->move_pan_tilt_abs(pan_zero, tilt_zero))
		{
			res.error_reason=fsm::VISION_ERROR;
			return;
		}
		std::cout<<"[VISION]pan_tilt unit scan complete."<<std::endl;

		// Voxelize result point clouds
		// for each point cloud, we have to create a voxelized version.

		if (!scan_full)
		{
			finalVoxelizedPC       ->clear();
			finalVoxelizedBluePC   ->clear();
			finalVoxelizedGreenPC  ->clear();
			finalVoxelizedRedPC    ->clear();
			finalVoxelizedYellowPC ->clear();
			finalVoxelizedCyanPC   ->clear();
			finalVoxelizedMagentaPC->clear();
		}


		pcl::VoxelGrid<pcl::PointXYZ> vg;

		// copy the final result to finalGlobalPC
		*finalPC += *finalScenePC;
		std::cout<<"[VISION]Voxelization: global point cloud..."<<std::endl;
		vg.setInputCloud(finalPC);
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
		return;
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr filledForOctomapPC;
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
		res.error_reason=fsm::DATA_ERROR;
		return;
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
		res.error_reason=fsm::DATA_ERROR;
		return;
	}

	// Apply HSV filtering
	ImageFilter hsvFilter(_cv_image->image);
	/*
	 * Create threshold for six colors
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

	if(task_nr==5)
	{
		std::cout << "about to delete the puzzle fixture"<< std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr fake_puzzle(new pcl::PointCloud<pcl::PointXYZ>);
		fake_puzzle = fake_puzzle_fixture();
		pcl::PointCloud<pcl::PointXYZ>::Ptr scene_without_fixture(new pcl::PointCloud<pcl::PointXYZ>);
		scene_without_fixture = scenePointCloud->removeSquareFromPointCloud(threshPC,fake_puzzle->points[0],fake_puzzle->points[1],fake_puzzle->points[2],fake_puzzle->points[3]);
		threshPC.reset(new pcl::PointCloud<pcl::PointXYZ>);
		threshPC = scenePointCloud->removeSquareFromPointCloud(scene_without_fixture,fake_puzzle->points[0],fake_puzzle->points[1],fake_puzzle->points[2],fake_puzzle->points[3]);

	}

	filledForOctomapPC = fillPointCloud(threshPC);

#ifdef OCTOMAP_COMPLETE_OLD
	// Create Octomap (whole scene with robot and Pantilt); also the man outside the table can be seen
	OctoCloud->reserve(worldPC->points.size());
	octomap::pointcloudPCLToOctomap(*worldPC,*OctoCloud);
	std::cout<<"[VISION]Starting to add actual PointCloud to Octomap..."<<std::endl;
	tree->insertScan(*OctoCloud, scenePointCloud->getSensorOriginScene(TCP_CAM));
	std::cout<<"[VISION]Added actual PointCloud to Octomap"<<std::endl;
#endif

#ifdef OCTOMAP_SERVER
	pcl::toROSMsg (*filledForOctomapPC, msg);
	msg.header.frame_id = "/Origin";
	msg.header.stamp = ros::Time::now();
	pub_3.publish (msg);
#endif

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
	// voxelize with very small leaf size
	*finalGreenPC += *greenFilteredPC;
	*finalRedPC += *redFilteredPC;
	*finalYellowPC += *yellowFilteredPC;
	*finalCyanPC += *cyanFilteredPC;
	*finalMagentaPC += *magentaFilteredPC;

	// Voxelize result point clouds
	// for each point cloud, we have to create a voxelized version.
	finalVoxelizedPC       ->clear();
	finalVoxelizedBluePC   ->clear();
	finalVoxelizedGreenPC  ->clear();
	finalVoxelizedRedPC    ->clear();
	finalVoxelizedYellowPC ->clear();
	finalVoxelizedCyanPC   ->clear();
	finalVoxelizedMagentaPC->clear();

	pcl::VoxelGrid<pcl::PointXYZ> vg;

	// copy the result to finalPC
	*finalPC += *finalTcpPC;
	std::cout<<"[VISION]Voxelization: global point cloud..."<<std::endl;
	vg.setInputCloud(finalPC);
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

	std::cout<<"[VISION]Voxelization: finished."<<std::endl;

	//  *finalVoxelizedPC += *tempVoxelizedPC;
	//  *finalVoxelizedBluePC += *tempVoxelizedBluePC;
	//  *finalVoxelizedGreenPC += *tempVoxelizedGreenPC;
	//  *finalVoxelizedRedPC += *tempVoxelizedRedPC;
	//  *finalVoxelizedYellowPC += *tempVoxelizedYellowPC;
	//  *finalVoxelizedCyanPC += *tempVoxelizedCyanPC;
	//  *finalVoxelizedMagentaPC += *tempVoxelizedMagentaPC;


	// Set the time stamp of current image
	finalTimeStamp = stepTimeStampRGB;

	// publish final voxelized point cloud
	pcl::toROSMsg (*finalVoxelizedPC, msg);
	msg.header.frame_id = "/Origin";
	msg.header.stamp = ros::Time::now();
	pub.publish (msg);

	delete scenePointCloud;

}

bool Vision::same_colored_objects(const am_msgs::VisionGoal::ConstPtr &goal) {
	same_color_problem = false;
	std::string nrof_objects = "";
	problem_object_index = 0;
	total_nr_of_objects = 0;
	int problem_obj_shape_number = 0;
	if (nh_.searchParam("nr_objects_", nrof_objects)) {
		nh_.getParam(nrof_objects, total_nr_of_objects);
		//std::cout << "[VISION]Total Number of Objects: " << total_nr_of_objects<< std::endl;
		//ROS_INFO("TOTAL NUMBER OF OBJECTS IS FOUND!!!!");
	} else
		ROS_WARN("NUMBER OF OBJECTS COULD NOT BE FETCHED");

	std::string object_color_param = "";
	std::string object_nrshapes_param = "";
	std::string part1 = "object_";
	std::string part2 = "";
	std::string part3 = "_nr_shapes_";
	std::string part4 = "_color_";

	for (int cnt = 0; cnt < total_nr_of_objects; cnt++) {
		//std::cout << "[VISION] ENTERED THE LOOP" << std::endl;
		std::stringstream ss;
		ss << cnt;
		part2 = ss.str();
		//std::cout<<"part2: "<<part2<<std::endl;

		std::string full_string_color_param = part1 + part2 + part4;
		std::string full_string_nrofshapes_param = part1 + part2 + part3;

		//std::cout << "[VISION] Full_string_nrofshapes_param: "<< full_string_nrofshapes_param << std::endl;
		//std::cout << "[VISION] Full_string_color_param: "<< full_string_color_param << std::endl;

		if (nh_.searchParam(full_string_color_param, object_color_param)) {
			//std::cout << "Color of an object is found" << std::endl;
			std::string colortemphold = "";
			nh_.getParam(object_color_param, colortemphold);
			Colour_String.push_back(colortemphold);
			//std::cout << "[VISION] COLOR OF AN OBJECT IS SAVED: "<< colortemphold << std::endl;

			if (nh_.searchParam(full_string_nrofshapes_param,
					object_nrshapes_param)) {
				int nr_of_shapes_hold = 0;
				nh_.getParam(object_nrshapes_param, nr_of_shapes_hold);
				Number_of_Shapes_List.push_back(nr_of_shapes_hold);
				//std::cout << "[VISION]THE OBJECT SHAPE NUMBER IS SAVED:"<< nr_of_shapes_hold << std::endl;

				if (((goal->object.color) == colortemphold)
						&& ((nr_of_shapes_hold) > (goal->object.nr_shapes))) {
					//std::cout << "[VISION]SAME COLOR OBJECTS: Call alignment2"<< std::endl;
					same_color_problem = true;
					problem_obj_shape_number = nr_of_shapes_hold;
					problem_object_index = cnt;
					return true; //Call Alignment 2

				}
			} //Object Shape Number
		} //Color Comparison
	} //END OF LOOP
	return false;
} // SAME_COLOR_PROBLEM

/**
 * This function aligns a pre-defined object model with an input point cloud.
 * The input point cloud is calculated by functions from am_pointcloud class.
 **/
pcl::PointCloud<pcl::PointXYZ>::Ptr Vision::reduced_object_input(
		pcl::PointCloud<pcl::PointXYZ>::Ptr object_input,
		pcl::PointCloud<pcl::PointXYZ>::Ptr scene_input, bool box,
		bool cylinder) {
	scene_input->is_dense = false;
	object_input->is_dense = false;
	std::vector<int> index;
	pcl::removeNaNFromPointCloud(*scene_input, *scene_input, index);
	pcl::removeNaNFromPointCloud(*object_input, *object_input, index);
	std::cout << "[VISION]Number of Points in object input "
			<< object_input->points.size() << std::endl;
	std::cout << "CLEAR!" << std::endl;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(object_input);
	std::vector<int> pointIdxRadiusSearch;
	std::vector<int> pointIDs;
	std::vector<float> pointRadiusSquaredDistance;
	std::vector<float> searchPointList;
	std::vector<int> bottom_index;

	double radius = leaf_size * 0.5;
	int mini_index = 0;

	for (size_t f = 0; f < (object_input->points.size()); f++) {
		if ((object_input->points[f].z) < (object_input->points[mini_index].z))
			mini_index = f;
	}

	size_t pointCloudSize = object_input->points.size();
	for (size_t i = 0; i < (object_input->points.size()); i++) {

		pcl::PointXYZ searchPoint;
		searchPoint.x = object_input->points[i].x;
		searchPoint.y = object_input->points[i].y;
		searchPoint.z = object_input->points[i].z;

		if (object_input->points[i].z
				< (object_input->points[mini_index].z) + 0.01) {
			pointIDs.push_back(i);
			continue;
		}

		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
				pointRadiusSquaredDistance) > 0) {

			if ((pointIdxRadiusSearch.size() > 1))
				pointIDs.push_back(i);
		} //End of Search If
	} //End of For

	std::cout << "[VISION]Number of Points in object input "
			<< object_input->points.size() << "Points index " << pointIDs.size()
			<< std::endl;

	for (int j = 0; j < pointIDs.size(); ++j) {
		object_input->points[pointIDs[j]].x =
				std::numeric_limits<float>::quiet_NaN();
		object_input->points[pointIDs[j]].y =
				std::numeric_limits<float>::quiet_NaN();
		object_input->points[pointIDs[j]].z =
				std::numeric_limits<float>::quiet_NaN();
	}

	scene_input->is_dense = false;
	object_input->is_dense = false;
	std::vector<int> indx;

	pcl::removeNaNFromPointCloud(*scene_input, *scene_input, indx);
	pcl::removeNaNFromPointCloud(*object_input, *object_input, indx);
	std::cout << "[VISION]Number of Points in object input2 "
			<< object_input->points.size() << "smallest z:  "
			<< object_input->points[mini_index].z << std::endl;
	return object_input;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr Vision::fake_object_creater(bool is_task5) {
	ShapeGenerator<pcl::PointXYZ> shape_generator2;
	pcl::PointCloud<pcl::PointXYZ>::Ptr T_object_model(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr T_shape_model(
			new pcl::PointCloud<pcl::PointXYZ>);
	float step_size = 0.005;
	Eigen::Quaternion<double> q2;
	Eigen::Vector3d translation2;

	//We have to generate the object which is bigger than our target, yet in the same colors
	std::string part1 = "object_";
	std::string part2 = "";
	std::string part3 = "_nr_shapes_";
	std::string part4 = "_color_";
	std::string part6 = "_shape_";
	int object_part_size_number = 0;
	std::string part5 = "_length_";
	std::string partx = "x_";
	std::string party = "y_";
	std::string partz = "z_";
	std::string part9 = "_radius_";
	std::string partw = "w_";
	std::string part13 = "_size_";
	std::string hold_obj_part = "";
	std::string hold_size = "";
	std::string pose_position = "_pose_position_";
	std::string pose_orientation = "_pose_orientation_";
	std::string string_with_shape_number = "";
	std::string full_string_orientation = "";
	std::string full_string_position = "";
	std::string full_string_radius = "";
	std::string full_string_length = "";
	std::string full_string_size = "";
	std::vector<std::string> part_position_vector_;
	std::vector<std::string> part_orientation_vector_;
	std::vector<std::string> cube_size_vector_;
	std::vector<std::string> cylinder_radius_vector_;
	std::vector<std::string> cylinder_length_vector_;
	std::stringstream ss;
	ss << problem_object_index;
	part2 = ss.str();

	for (int shapecounter_ = 0; shapecounter_ < Number_of_Shapes_List.back();
			shapecounter_++) {
		bool is_cylinder = false;
		std::stringstream cs;
		cs << shapecounter_;
		hold_obj_part = cs.str();
		string_with_shape_number = part1 + part2 + part4 + hold_obj_part;
		std::cout << "[VISION]String with shape number: "
				<< string_with_shape_number << std::endl;

		cylinder_radius_vector_.push_back(string_with_shape_number + part9);
		cylinder_length_vector_.push_back(string_with_shape_number + part5);

		string keystring = "";
		if (nh_.searchParam(cylinder_radius_vector_.back(), keystring)) {

			nh_.getParam(keystring, cylinder_radius);

			std::cout << "[VISION]****Cylinder radius is found: "
					<< cylinder_radius << std::endl;
			is_cylinder = true;

		}

		keystring = "";
		if (nh_.searchParam(cylinder_length_vector_.back(), keystring)) {
			nh_.getParam(keystring, cylinder_length);
			is_cylinder = true;

			std::cout << "[VISION]****Cylinder length is found: "
					<< cylinder_length << std::endl;
			std::cout << "[VISION]Cylinder_radius_vector:"
					<< cylinder_radius_vector_[shapecounter_] << std::endl;
			std::cout << "[VISION]Cylinder_length_vector:"
					<< cylinder_length_vector_[shapecounter_] << std::endl;
		}

		//object_0_shape_0_pose_position_x
		keystring = "";
		part_position_vector_.push_back(
				string_with_shape_number + pose_position + partx);
		if (nh_.searchParam(part_position_vector_.back(), keystring)) {
			double temp = 0;
			nh_.getParam(keystring, temp);
			trouble_object_position_vector.push_back(temp);
			std::cout << "[VISION]****Trouble Object part position x is found: "
					<< trouble_object_position_vector.back() << std::endl;
			keystring = "";
		}
		part_position_vector_.push_back(
				string_with_shape_number + pose_position + party);
		if (nh_.searchParam(part_position_vector_.back(), keystring)) {
			double temp = 0;
			nh_.getParam(keystring, temp);
			trouble_object_position_vector.push_back(temp);
			std::cout << "[VISION]****Trouble Object part position y is found: "
					<< trouble_object_position_vector.back() << std::endl;
			keystring = "";
		}
		part_position_vector_.push_back(
				string_with_shape_number + pose_position + partz);

		if (nh_.searchParam(part_position_vector_.back(), keystring)) {
			double temp = 0;
			nh_.getParam(keystring, temp);
			trouble_object_position_vector.push_back(temp);
			std::cout << "[VISION]****Trouble Object part position z is found: "
					<< trouble_object_position_vector.back() << std::endl;
			keystring = "";
		}

		part_orientation_vector_.push_back(
				string_with_shape_number + pose_orientation + partx);

		if (nh_.searchParam(part_orientation_vector_.back(), keystring)) {
			double temp = 0;
			nh_.getParam(keystring, temp);
			trouble_object_orientation_vector.push_back(temp);
			std::cout
			<< "[VISION]****Trouble Object orientation position x is found: "
			<< trouble_object_orientation_vector.back() << std::endl;
			keystring = "";
		}
		part_orientation_vector_.push_back(
				string_with_shape_number + pose_orientation + party);
		if (nh_.searchParam(part_orientation_vector_.back(), keystring)) {
			double temp = 0;
			nh_.getParam(keystring, temp);
			trouble_object_orientation_vector.push_back(temp);
			std::cout
			<< "[VISION]****Trouble Object orientation position y is found: "
			<< trouble_object_orientation_vector.back() << std::endl;
			keystring = "";
		}
		part_orientation_vector_.push_back(
				string_with_shape_number + pose_orientation + partz);
		if (nh_.searchParam(part_orientation_vector_.back(), keystring)) {
			double temp = 0;
			nh_.getParam(keystring, temp);
			trouble_object_orientation_vector.push_back(temp);
			std::cout
			<< "[VISION]****Trouble Object orientation position z is found: "
			<< trouble_object_orientation_vector.back() << std::endl;
			keystring = "";
		}
		part_orientation_vector_.push_back(
				string_with_shape_number + pose_orientation + partw);
		if (nh_.searchParam(part_orientation_vector_.back(), keystring)) {
			double temp = 0;
			nh_.getParam(keystring, temp);
			trouble_object_orientation_vector.push_back(temp);
			std::cout
			<< "[VISION]****Trouble Object orientation position w is found: "
			<< trouble_object_orientation_vector.back() << std::endl;
			keystring = "";
		}

		std::cout << "[VISION]part_orientation_vector_.partx): "
				<< (part_orientation_vector_.front()) << std::endl;

		//Find Cube Sizes- Cube always have 3 size
		for (int sizenumber = 0; sizenumber < 3; sizenumber++) {
			std::string holdsize = "";
			std::stringstream cc;
			cc << sizenumber;
			holdsize = cc.str();
			//object_0_shape_0_size_0/1/2_

			cube_size_vector_.push_back(
					string_with_shape_number + part13 + holdsize + "_");
			std::string holdtemp = string_with_shape_number + part13 + holdsize
					+ "_";
			std::cout << "[VISION]****T.O. cube size is found: " << holdtemp
					<< std::endl;

			keystring = "";
			if (nh_.searchParam(holdtemp, keystring)) {
				double temp = 0;
				nh_.getParam(keystring, temp);
				trouble_object_cube_size_vector.push_back(temp);
				std::cout << "[VISION]****cube size is found: "
						<< trouble_object_cube_size_vector.back() << std::endl;
			} //END OF IF
		} //END OF FOR
		if (is_cylinder) {
			step_size = leaf_size;
			shape_generator2.generateCylinder(step_size,
					Eigen::Vector3f(0.0f, 0.0f, 0.0f),
					cylinder_length * Eigen::Vector3f::UnitZ(),
					cylinder_radius);

			double w = trouble_object_orientation_vector.back();
			trouble_object_orientation_vector.pop_back();

			double z = trouble_object_orientation_vector.back();
			trouble_object_orientation_vector.pop_back();

			double y = trouble_object_orientation_vector.back();
			trouble_object_orientation_vector.pop_back();

			double x = trouble_object_orientation_vector.back();
			trouble_object_orientation_vector.pop_back();
			q2 = Eigen::Quaternion<double>(w, x, y, z);

			float pz = trouble_object_position_vector.back();
			trouble_object_position_vector.pop_back();
			float py = trouble_object_position_vector.back();
			trouble_object_position_vector.pop_back();
			float px = trouble_object_position_vector.back();
			trouble_object_position_vector.pop_back();
			translation2[0] = px;
			translation2[1] = py;
			translation2[2] = -(cylinder_radius / 2) + pz;

			pcl::transformPointCloud(*T_shape_model, *T_shape_model,
					translation2, q2);

			*T_object_model += *T_shape_model;

			T_shape_model->clear();
		} else //If box
		{
			step_size = leaf_size;
			float side3 = trouble_object_cube_size_vector.back();
			trouble_object_cube_size_vector.pop_back();
			float side2 = trouble_object_cube_size_vector.back();
			trouble_object_cube_size_vector.pop_back();
			float side1 = trouble_object_cube_size_vector.back();
			trouble_object_cube_size_vector.pop_back();

			shape_generator2.generateBox(step_size,
					Eigen::Vector3f(0.0f, 0.0f, 0.0f),
					side3 * Eigen::Vector3f::UnitX(),
					side1 * Eigen::Vector3f::UnitY(),
					side1 * Eigen::Vector3f::UnitZ(), 0.0f, true, is_task5);
			std::cout << "Object Size not 1 and Compound" << std::endl;

			//transform Box PC to valid position
			double w = trouble_object_orientation_vector.back();
			trouble_object_orientation_vector.pop_back();

			double z = trouble_object_orientation_vector.back();
			trouble_object_orientation_vector.pop_back();

			double y = trouble_object_orientation_vector.back();
			trouble_object_orientation_vector.pop_back();

			double x = trouble_object_orientation_vector.back();
			trouble_object_orientation_vector.pop_back();

			q2 = Eigen::Quaternion<double>(w, x, y, z);

			float pz = trouble_object_position_vector.back();
			trouble_object_position_vector.pop_back();
			float py = trouble_object_position_vector.back();
			trouble_object_position_vector.pop_back();
			float px = trouble_object_position_vector.back();
			trouble_object_position_vector.pop_back();

			translation2[0] = -(side2 / 2) + px;
			translation2[1] = -(side2 / 2) + py;
			translation2[2] = -(side3 / 2) + pz;

			pcl::transformPointCloud(*T_shape_model, *T_shape_model,
					translation2, q2);

			*T_object_model += *T_shape_model;
			T_shape_model->clear();
		}

	} //END OF FOR
	//*******************************************End of parameter search!****************************
	return T_object_model;
} //END OF FAKE SHAPE GENERATOR, SAME COLOR PROBLEM

/**
 * This function aligns a pre-defined object model with an input point cloud.
 * The input point cloud is calculated by functions from am_pointcloud class.
 **/
Eigen::Matrix4f Vision::align_PointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr object_input, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_input, bool box, bool cylinder, bool is_task5)
{
	std::cout <<"current precision: "<<_currentGoal->precision<<std::endl;

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

	if (is_task5) {

		pcl::VoxelGrid<pcl::PointXYZ> vg;
		vg.setLeafSize(leaf_size * 4, leaf_size * 4, leaf_size * 4);
		vg.setInputCloud(scene_input);
		vg.filter(*scene_input);

	} //if it task 5

	else { //Downsample

		if (cylinder) {
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
	} // if it is nottask5

	// publish final thresholded point cloud
	//  std::cout<<"[VISION]Show downsampled point cloud"<<std::endl;
	//  pcl::toROSMsg ((*scene_input + *object_input), msg);
	//  msg.header.frame_id = "LWR_0";
	//  msg.header.stamp = ros::Time::now();
	//  pub.publish (msg);


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

	ROS_INFO(" > scene points: %d     object points: %d \n > box: %d     cylinder: %d"
			, scene_input->points.size(), object_input->points.size(), box, cylinder);

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

	// the variable PRECISION determines the precision of the alignment algorithm.
	// 3 sets of precision is implemented to make the alignment robust with respect to time
	// the StateMachine starts with the highest precision and passes it to the Vision node,
	// In the worst case scenario, when the vision node cannot find a good alignment for an object, StateMachine
	// would ask for a very rough alignment (LOW PRECISION) to have the chance of grabbing the object, though with a
	// very poor alignment.
	if(cylinder)
	{
		if (_currentGoal->precision == 0) // High precision
			iters = 350000;
		else if (_currentGoal->precision == 1) // Mid- precision
			iters = 250000;
		else if (_currentGoal->precision == 2) // Low precision
			iters = 200000;
		else
		{
			ROS_WARN("Unknown precision, proceeding with medium precision.");
			iters = 250000;
		}
	}
	else
	{
		if (_currentGoal->precision == 0) // High precision
			iters = 400000;
		else if (_currentGoal->precision == 1) // Mid- precision
			iters = 300000;
		else if (_currentGoal->precision == 2) // Low precision
			iters = 250000;
		else
		{
			ROS_WARN("Unknown precision, proceeding with medium precision.");
			iters = 300000;
		}
	}

	pcl::console::print_highlight ("performing %i iterations\n", iters);
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
		if (_currentGoal->precision == 0)
		{
			std::cout<<"set High precision for InlierFraction..."<<std::endl;
			align.setInlierFraction (0.30f); // Required inlier fraction for accepting a pose hypothesis
		}
		if (_currentGoal->precision == 1)
		{
			std::cout<<"set Medium precision for InlierFraction..."<<std::endl;
			align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
		}
		if (_currentGoal->precision == 2)
		{
			std::cout<<"set low precision for InlierFraction..."<<std::endl;
			align.setInlierFraction (0.20f); // Required inlier fraction for accepting a pose hypothesis
		}
	}

	else // Cylinder & Handle
	{
		align.setMaxCorrespondenceDistance (0.004); // Inlier threshold other shapes
		if (_currentGoal->precision == 0)
		{
			std::cout<<"set High precision for InlierFraction..."<<std::endl;
			align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
		}
		if (_currentGoal->precision == 1)
		{
			std::cout<<"set Medium precision for InlierFraction..."<<std::endl;
			align.setInlierFraction (0.20f); // Required inlier fraction for accepting a pose hypothesis
		}
		if (_currentGoal->precision == 2)
		{
			std::cout<<"set low precision for InlierFraction..."<<std::endl;
			align.setInlierFraction (0.15f); // Required inlier fraction for accepting a pose hypothesis
		}

	}

	Eigen::Matrix4f transform;
	uint16_t nmbr_tries=0;
	while(nmbr_tries<2)
	{
		pcl::ScopeTime t("Alignment");
		align.align (*object_aligned);

		if (align.hasConverged ())
		{

			std::cout<<"nmbr_tries: "<<nmbr_tries+1<<std::endl;
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
				iters+=300000;

			align.setMaximumIterations (iters);
		}
		nmbr_tries++;
	}

	return transform;
} // END align_PointClouds

/*
 * This function works the same as align_PointClouds(...), but with modified parameters specifically set for
 * task 6, to return a pose as fast as possible
 */
Eigen::Matrix4f Vision::fast_cube_alignment(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_input)
{
	/*
	 * Create the Object Model: Cube
	 */
	ShapeGenerator<pcl::PointXYZ> shape_generator;
	pcl::PointCloud<pcl::PointXYZ>::Ptr shape_model (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cube_model (new pcl::PointCloud<pcl::PointXYZ>);
	float step_size = 0.005;
	Eigen::Quaternion<double> q;
	Eigen::Vector3d translation;

	shape_generator.setOutputCloud(shape_model);
	//generate box PC (PointCloud)
	step_size = leaf_size;
	//    if (_currentGoal->object.shape.size() == 1)
	shape_generator.generateBox(step_size, Eigen::Vector3f(0.0f, 0.0f, 0.0f), _currentGoal->object.shape[0].size[0] * Eigen::Vector3f::UnitX(), _currentGoal->object.shape[0].size[1] * Eigen::Vector3f::UnitY(), _currentGoal->object.shape[0].size[2] * Eigen::Vector3f::UnitZ(), 0.0f, false, false);

	//transform Cylinder PC to valid position
	q = Eigen::Quaternion<double>(_currentGoal->object.shape[0].pose.orientation.w, _currentGoal->object.shape[0].pose.orientation.x, _currentGoal->object.shape[0].pose.orientation.y, _currentGoal->object.shape[0].pose.orientation.z);
	translation[0] = -(_currentGoal->object.shape[0].size[0]/2) + _currentGoal->object.shape[0].pose.position.x;
	translation[1] = -(_currentGoal->object.shape[0].size[1]/2) + _currentGoal->object.shape[0].pose.position.y;
	translation[2] = -(_currentGoal->object.shape[0].size[2]/2) + _currentGoal->object.shape[0].pose.position.z;

	pcl::transformPointCloud(*shape_model, *shape_model, translation, q);

	*cube_model += *shape_model;

	shape_model->clear();
	/*
	 * END Create the Object Model: Cube
	 */

	// Align observed point cloud with modeled object
	//==================

	//std::cout <<"current precision: "<<_currentGoal->precision<<std::endl;

	// Align a rigid object to a scene with clutter and occlusions
	// Point clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudT::Ptr object (new PointCloudT);
	PointCloudT::Ptr scene (new PointCloudT);
	FeatureCloudT::Ptr object_features (new FeatureCloudT);
	FeatureCloudT::Ptr scene_features (new FeatureCloudT);

	// Remove NANs from PointCloud scene
	scene_input->is_dense = false;
	cube_model->is_dense = false;
	std::vector<int> index;
	pcl::removeNaNFromPointCloud(*scene_input, *scene_input, index);
	pcl::removeNaNFromPointCloud(*cube_model, *cube_model, index);

	//Downsample (Uniform Sampling)
	pcl::PointCloud<int> sampled_indices;
	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
	uniform_sampling.setInputCloud (cube_model);
	uniform_sampling.setRadiusSearch (0.006);
	uniform_sampling.compute (sampled_indices);
	pcl::copyPointCloud (*cube_model, sampled_indices.points, *cube_model);
	uniform_sampling.setInputCloud (scene_input);
	uniform_sampling.setRadiusSearch (0.006);
	uniform_sampling.compute (sampled_indices);
	pcl::copyPointCloud (*scene_input, sampled_indices.points, *scene_input);

	//    //publish final thresholded point cloud
	//    std::cout<<"[VISION]Show downsampled point cloud"<<std::endl;
	//    pcl::toROSMsg ((*scene_input + *cube_model), msg);
	//    msg.header.frame_id = "LWR_0";
	//    msg.header.stamp = ros::Time::now();
	//    pub.publish (msg);


	// Estimate normals for scene
	pcl::console::print_highlight ("Estimating scene normals...\n");
	pcl::NormalEstimationOMP<pcl::PointXYZ,PointNT> nest;
	nest.setKSearch(10);
	nest.setInputCloud (scene_input);
	nest.compute (*scene);

	// Estimate normals for scene
	pcl::console::print_highlight ("Estimating object normals...\n");
	nest.setKSearch(10);
	nest.setInputCloud (cube_model);
	nest.compute (*object);

	//ROS_INFO(" > scene points: %f     object points: %f \n > box: %d     cylinder: %d");

	// Estimate features
	pcl::console::print_highlight ("Estimating features...\n");
	FeatureEstimationT fest;
	fest.setKSearch(15);
	fest.setInputCloud (cube_model);
	fest.setInputNormals (object);
	fest.compute (*object_features);
	fest.setInputCloud (scene_input);
	fest.setInputNormals (scene);
	fest.compute (*scene_features);


	// Perform alignment

	pcl::console::print_highlight ("Performing 10,000 iterations\n");
	pcl::console::print_highlight ("Starting alignment...\n");
	pcl::SampleConsensusPrerejective<pcl::PointXYZ,pcl::PointXYZ,FeatureT> align;
	align.setInputSource (cube_model);
	align.setSourceFeatures (object_features);
	align.setInputTarget (scene_input);
	align.setTargetFeatures (scene_features);
	align.setMaximumIterations (10000); // Number of RANSAC iterations
	align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness (2); // Number of nearest features to use
	align.setSimilarityThreshold (0.5f); // Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance (0.006); // Inlier threshold box
	align.setInlierFraction (0.45f); // Required inlier fraction for accepting a pose hypothesis

	Eigen::Matrix4f transform;

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
	}
	else
	{
		transform = align.getFinalTransformation ();
		pcl::console::print_error ("Alignment failed!\n");
		obj_aligned_=false;
	}


	//==================
	if(obj_aligned_==false)
	{
		vision_server_.setPreempted(vision_result_,"Alignment failed.");
		vision_result_.error_reason=fsm::POSE_NOT_FOUND;
		//return;
	}

	//Publish aligned PointCloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr test (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cube_model, *test);
	pcl::transformPointCloud(*test, *test, transform);
	pcl::toROSMsg (*test, msg);
	msg.header.frame_id = "/Origin";
	msg.header.stamp = ros::Time::now();
	pub_2.publish (msg);

	// Prepare the lastShapeToRemovePC for pointcloud update/removal
	lastShapeToRemovePC->clear();
	lastShapeToRemovePC.reset ( new pcl::PointCloud<pcl::PointXYZ> );
	*lastShapeToRemovePC += *test;

	return transform;
}

/*
 * This function enhances the initial computed pose of the CUBE with the help of a close-up image.
 * The image is taken using the TCP camera right from above the cube, this will eliminate the rotation error in
 * X and Y axes (The only rotation a cube can have is around Z axis).
 */
double Vision::close_range_pose(string color)
{
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
		//res.error_reason=fsm::DATA_ERROR;
		return -5;
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
		//res.error_reason=fsm::DATA_ERROR;
		return -5;
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
			std::cout<<"[VISION]failed to find enough lines. returning initial pose"<<std::endl;
			return -5;
		}

		houghLineCounter++;
	}

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

	//    // DEBUG: Draw the lines
	//    cv::Vec4i v = lines[idx[0]];
	//    cv::line(dst, cv::Point(v[0], v[1]), cv::Point(v[2], v[3]), CV_RGB(0,255,0), 2);
	//    v = lines[idx[1]];
	//    cv::line(dst, cv::Point(v[0], v[1]), cv::Point(v[2], v[3]), CV_RGB(255,0,0), 2);
	//    cv::imshow("lines", dst);
	//    cv::imshow("canny", bw);
	//    cv::waitKey();

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
	/*
	 * END compute angles using lines
	 */


	/*
	 * START compute angles using corners
	 * (first proposition, rejected due to high error rates)
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

	delete cornersPointCloud;

	return (-1)*rotation;
}

/*
 * This function sorts a vector of corners (of a rectangle) based on the camera frame
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
 * This workaround may tamper the whole result. In that case, we just have to send back the initial pose.
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

/*
 * This function receives two point clouds and computes the number of points which exist in the
 * first cloud, but not in the second cloud. This will serve us as a solution to compare two point cloud
 * in the sense of similarity
 */
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

	// Add points from closeRangePC to octree
	octree.setInputCloud (closeRangePC);
	octree.addPointsFromInputCloud ();

	std::vector<int> newPointIdxVector;

	// Get vector of point indices from octree voxels which did not exist in previous buffer
	octree.getPointIndicesFromNewVoxels (newPointIdxVector);

	// Output points
	std::cout<<"vector size"<<newPointIdxVector.size()<<std::endl;

	/*
	 * DEBUG
	 */
	//	std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
	//	for (size_t i = 0; i < newPointIdxVector.size (); ++i)
	//		std::cout << i << "# Index:" << newPointIdxVector[i]
	//				  << "  Point:" << closeRangePC->points[newPointIdxVector[i]].x << " "
	//				  << closeRangePC->points[newPointIdxVector[i]].y << " "
	//				  << closeRangePC->points[newPointIdxVector[i]].z << std::endl;
	/*
	 * END DEBUG
	 */

	return newPointIdxVector.size();
}

/*
 * This function checks whether an object with the input color has been placed correctly on its respective target zone.
 * properties of the target zone is given as its center and radius.
 * There are 3 possibilities:
 *    1. object with that color is not found in the camera view
 *    2. Object with that color is placed correctly on the target zone
 *    3. Object with that color is not placed within a certain distance from the target zone
 */
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
		//res.error_reason=fsm::DATA_ERROR;
		return 0;
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
		//res.error_reason=fsm::DATA_ERROR;
		return 0;
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

	// check if cloud in that color is empty
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
	 * new solution: use exact information from the YAML file
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



pcl::PointCloud<pcl::PointXYZ>::Ptr Vision::fillPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filledPC;
	pcl::PointXYZ p;

	//  int width = 1;
	//  int height = 1;
	filledPC.reset(new pcl::PointCloud<pcl::PointXYZ> ());
	//  filledPC->resize(width * height);

	int maxZ = 0;
	//  int n = 0;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if( pcl_isfinite(cloud->points[i].x)
				||pcl_isfinite(cloud->points[i].y)
				||pcl_isfinite(cloud->points[i].z) )

		{
			maxZ = (int)(cloud->at(i).z / 0.01);

			p.x = cloud->points[i].x;
			p.y = cloud->points[i].y;
			p.z = cloud->points[i].z;
			filledPC->points.push_back(p);
			//      *filledPC += p;

			//      filledPC->points[n].x = cloud->points[i].x;
			//      filledPC->points[n].y = cloud->points[i].y;
			//      filledPC->points[n].z = cloud->points[i].z;
			//      n++;

			for (int j = 0; j < maxZ; j++)
			{
				p.x = cloud->points[i].x;
				p.y = cloud->points[i].y;
				p.z = (float) j * 0.01;
				filledPC->points.push_back(p);
				//        *filledPC += p;
				//        filledPC->points[n].x = cloud->points[i].x;
				//        filledPC->points[n].y = cloud->points[i].y;
				//        filledPC->points[n].z = j * 0.01;
				//        n++;
			}
		}
	}
	filledPC->width = filledPC->points.size ();
	filledPC->height = 1;
	filledPC->is_dense = true;

	std::cout<<"returning the filledPC"<<std::endl;
	return filledPC;
}



bool Vision::search_for_object_on_zone(pcl::PointCloud<pcl::PointXYZ >::Ptr inputCloud, pcl::PointXYZ zoneCenter, float radius)
{
	// calculate center of mass
	pcl::PointXYZ centerOfMass;
	centerOfMass = am_pointcloud::calculateCenterOfMass(inputCloud);

	std::cout<<"Center of Mass:  X= "<<centerOfMass.x<<",  Y= "<<centerOfMass.y<<",  Z="<<centerOfMass.z<<std::endl;

	float distanceX = std::abs(centerOfMass.x - zoneCenter.x);
	float distanceY = std::abs(centerOfMass.y - zoneCenter.y);

	if ( (distanceX*distanceX + distanceY*distanceY) < radius*radius )
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

//bool Vision::search_for_object_on_zone_initial(pcl::PointCloud<pcl::PointXYZ >::Ptr input_cloud, const am_msgs::VisionGoal::ConstPtr &goal)
//{
//	// set up target zone position and radius
//    pcl::PointXYZ zoneCenter;
//    zoneCenter.x = goal->target_zone.position.x;
//    zoneCenter.y = goal->target_zone.position.y;
//    zoneCenter.z = goal->target_zone.position.z;
//    float zoneRadius = goal->target_zone.max_distance;
//
//	int minPointCounter = 0;
//	float distanceX, distanceY;
//	for (int i=0; i<input_cloud->points.size(); i++)
//	{
//		distanceX = std::abs(input_cloud->points[i].x - zoneCenter.x);
//		distanceY = std::abs(input_cloud->points[i].y - zoneCenter.y);
//		if ( distanceX < zoneRadius && distanceY < zoneRadius )
//			minPointCounter++;
//		if (minPointCounter > 5)
//		{
//			std::cout<<"an object is detected on target zone."<<std::endl;
//			return true;
//		}
//	}
//
//	return false;
//}

/*
 * This function receives a point cloud as input and divides it into zero or more clusters.
 * Then it tries to find those clusters which are similar to the objects defined in the YAML file, based on their sizes/lengths
 */
pcl::PointCloud<pcl::PointXYZ >::Ptr Vision::find_clusters(pcl::PointCloud<pcl::PointXYZ >::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ >::Ptr colorReferencePC, const am_msgs::VisionGoal::ConstPtr &goal)
{
	std::cout<<"[VISION]entered find_clusters."<<std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr targetPC (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr intermediateClusterPC (new pcl::PointCloud<pcl::PointXYZ>);
	bool isIntermediateClusterEmpty = true;

	//  ros::Duration(1.0).sleep();
	// remove NaNs from the input point cloud
	input_cloud->is_dense = false;
	std::vector<int> index;
	pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, index);

	// Create the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (input_cloud);

	std::vector<pcl::PointIndices> initial_cluster_indices;
	std::vector<pcl::PointIndices> intermediate_cluster_indices;
	int intermediateClusterCounter = 0;

	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (50);
	ec.setMaxClusterSize (100000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (input_cloud);
	ec.extract (initial_cluster_indices);

	std::cout<<"[VISION]found "<<initial_cluster_indices.size()<<" initial clusters"<<std::endl;

	//intermediate_cluster_indices.resize( initial_cluster_indices.size() );
	for (std::vector<pcl::PointIndices>::const_iterator it = initial_cluster_indices.begin (); it != initial_cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (input_cloud->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// find the min & max point in the cloud (cluster)
		pcl::PointXYZ pMin, pMax;
		//std::cout<<"cluster size: "<<cloud_cluster->points.size()<<std::endl;
		pcl::getMinMax3D(*cloud_cluster, pMin, pMax);
		double max_dist = pcl::euclideanDistance(pMin, pMax);
		//std::cout<<"Max Distance: "<<max_dist<<std::endl;
		float obj_length = get_shape_length(goal);
		//std::cout<<"current goal->shape length: "<<obj_length<<std::endl;
		// compare the distance between min & max
		if( max_dist < obj_length * 2.0 ) // distance close to object length definition in YAML file --> OBJECT RECOGNIZED
		{
			isIntermediateClusterEmpty = false;
			intermediate_cluster_indices.resize( intermediate_cluster_indices.size() + 1 );
			intermediate_cluster_indices[intermediateClusterCounter].indices = it->indices; //http://www.pcl-users.org/IndicesPtr-from-PointIndices-td4020356.html
			intermediateClusterCounter++;
			std::cout<<"Max Distance: "<<max_dist<<std::endl;
			std::cout<<"current goal->shape length: "<<obj_length<<std::endl;
			std::cout<<"[VISION] intermediateClusterCounter = "<<intermediateClusterCounter<<std::endl;
			*intermediateClusterPC += *cloud_cluster;
		}

		// publish the clusters
		pcl::toROSMsg (*cloud_cluster, msg);
		msg.header.frame_id = "/Origin";
		msg.header.stamp = ros::Time::now();
		pub.publish (msg);
		ros::Duration(0.5).sleep();
	}

	ROS_WARN("SHOW FILTERED CLUSTERS");
	std::cout<<"[VISION] #intermediateCluster: "<<intermediateClusterCounter<<std::endl;
	//ros::Duration(4.0).sleep();
	for (std::vector<pcl::PointIndices>::const_iterator it = intermediate_cluster_indices.begin (); it != intermediate_cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (input_cloud->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// publish the clusters
		pcl::toROSMsg (*cloud_cluster, msg);
		msg.header.frame_id = "/Origin";
		msg.header.stamp = ros::Time::now();
		pub.publish (msg);

		ROS_WARN("Go for comparison");
		int differences = compare_cluster_differences(cloud_cluster, colorReferencePC);
		// if "OK" -> send back indices of this cloud / or the cloud itself --> this will be our targetPC
		if ( differences )
		{
			std::cout<<"THIS IS OUR TARGET! Hit it!"<<std::endl;
			isFinalClusterEmpty = false;
			*targetPC += *cloud_cluster;
		}
		ROS_WARN("comparison completed");

		//      ros::Duration(2.0).sleep();
	}

	//finalClusterCounter = 0;

	if (isFinalClusterEmpty) // compare_cluster_differences was not able to find a better clustering
	{
		if (isIntermediateClusterEmpty) // worst-case: could not find any intermediate cluster
			return colorReferencePC;

		// pass the intermediate cluster
		return intermediateClusterPC;
	}

	return targetPC;
}

/*
 * This function takes an object (from goal msg) as input and tries to find
 * the maximum length of that object.
 */
float Vision::get_shape_length(const am_msgs::VisionGoal::ConstPtr &goal)
{
	if(goal->object.nr_shapes == 1) // should be either a cube or a cylinder
	{
		if( !goal->object.shape[0].type.compare("box") )
		{
			return (float) goal->object.shape[0].size[0];
		}
		else if( !goal->object.shape[0].type.compare("cylinder") )
		{
			return (float) goal->object.shape[0].length;
		}
		else
		{
			ROS_WARN("Unknown shape type");
			return 0;
		}
	}
	else // this should be a handle
	{
		return (float)goal->object.shape[0].length +
				(float)goal->object.shape[1].size[0]+
				(float)goal->object.shape[2].size[0];
	}

	ROS_WARN("Unknown shape type");
	return 0;
}

bool Vision::compare_cluster_differences(pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPC, pcl::PointCloud<pcl::PointXYZ>::Ptr noisyColorPC)
{
	// Octree resolution - side length of octree voxels
	float resolution = 0.005f;

	// Instantiate octree-based point cloud change detection class
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);

	// Add points from targetPC to octree
	octree.setInputCloud (clusterPC);
	octree.addPointsFromInputCloud ();

	// Switch octree buffers: This resets octree but keeps previous tree structure in memory.
	octree.switchBuffers ();

	// Add points from closeRangePC to octree
	octree.setInputCloud (noisyColorPC);
	octree.addPointsFromInputCloud ();

	std::vector<int> newPointIdxVector;

	// Get vector of point indices from octree voxels which did not exist in previous buffer
	octree.getPointIndicesFromNewVoxels (newPointIdxVector);

	// Output points
	std::cout<<"differences"<<newPointIdxVector.size()<<std::endl;

	/*
	 * DEBUG
	 */
	//    std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
	//    for (size_t i = 0; i < newPointIdxVector.size (); ++i)
	//            std::cout << i << "# Index:" << newPointIdxVector[i]
	//                              << "  Point:" << closeRangePC->points[newPointIdxVector[i]].x << " "
	//                              << closeRangePC->points[newPointIdxVector[i]].y << " "
	//                              << closeRangePC->points[newPointIdxVector[i]].z << std::endl;
	/*
	 * END DEBUG
	 */

	std::cout<<"Cluster size: "<<clusterPC->points.size()<<std::endl;
	std::cout<<"noisyColorPC size: "<<noisyColorPC->points.size()<<std::endl;
	std::cout<<"common points : "<<noisyColorPC->points.size() - newPointIdxVector.size()<<std::endl;

	int commonPoints = noisyColorPC->points.size() - newPointIdxVector.size();

	if ( commonPoints > 0.9*clusterPC->points.size() )
		return true;

	return false;
	//  return newPointIdxVector.size();
}

pcl::PointCloud<pcl::PointXYZ >::Ptr Vision::get_task6_cloud()
{
	ROS_INFO("entered get_task6_cloud()...");

	am_pointcloud *scenePointCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr initialPC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr alignedPC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr redFilteredPC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr worldPC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr robotLessPC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr threshPC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filledForOctomapPC;

	ROS_INFO("Moving the pan-tilt unit");
	MovePantilt* mpt;
	mpt = mpt->get_instance();
	mpt->move_pan_tilt_abs(-0.300, 0.800);

	ros::Duration(0.5).sleep();
	ROS_INFO("pan-tilt unit ready");

	// Create the point cloud
	// =================================

	ros::Time stepTimeStampRGB;
	ros::Time stepTimeStampDepth;

	_scene_rgb_image = *(ros::topic::waitForMessage<sensor_msgs::Image>(camera_scene_rgb_topic, ros::Duration(1.0)));
	_scene_depth_image = *(ros::topic::waitForMessage<sensor_msgs::Image>(camera_scene_depth_topic, ros::Duration(1.0)));
	// Convert ROS RGB image to OpenCV format
	try
	{
		stepTimeStampRGB = _scene_rgb_image.header.stamp;
		_cv_image = cv_bridge::toCvCopy(_scene_rgb_image, enc::BGR8);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		failed = true;
		//res.error_reason=fsm::DATA_ERROR;
		//return false;
	}

	// Convert ROS Depth image to OpencV format
	try
	{
		stepTimeStampDepth = _scene_depth_image.header.stamp;
		_cv_depthptr = cv_bridge::toCvCopy(_scene_depth_image, enc::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		failed = true;
		//res.error_reason=fsm::DATA_ERROR;
		//return false;
	}

	// Apply HSV filtering
	ImageFilter hsvFilter(_cv_image->image);
	/*
	 * Create threshold for six colors
	 */
	// Red
	hsvFilter.setHsvValues("ff0000");
	thresholdRed = hsvFilter.getFilteredImage();
	hsvFilter.morphOps(thresholdRed);

	// TODO: remove hardcoded Field-of-View values
	float fov_horizontal_scene_depth = 1.047;
	float fov_horizontal_scene_rgb = 1.047;

	scenePointCloud = new am_pointcloud(_cv_depthptr->image, fov_horizontal_scene_depth, _cv_image->image, fov_horizontal_scene_rgb);
	// create the initial point cloud based on depth image data
	initialPC = scenePointCloud->createInitialPointCloud();
	// align the initial point cloud with the RGB camera
	alignedPC = scenePointCloud->alignWithRGB(initialPC, CAM_SCENE, stepTimeStampRGB);
	// Transform the point cloud to world coordinate
	worldPC = scenePointCloud->transformToWorld(alignedPC, CAM_SCENE, stepTimeStampRGB);
	// filter out the robot, table surface and irrelevant points
	robotLessPC = scenePointCloud->removeRobotFromPointCloud(worldPC);
	threshPC = scenePointCloud->xyzTheresholdCloud(robotLessPC, 0.01); // Hardcoded z-value, TODO: get info from YAML file
	redFilteredPC = scenePointCloud->filterPointCloudByColor(threshPC, thresholdRed);
	// =================================

	//  ROS_INFO("Waiting for 2 seconds... haha! :|");
	//  ros::Duration(2.0).sleep();

	finalTimeStamp = stepTimeStampDepth;

	delete scenePointCloud;

	return redFilteredPC;
}


/*
 * This function clears all point clouds and transformations, making them ready to work on a new task.
 */
bool Vision::master_reset()
{
	try
	{
		finalScenePC  ->clear();
		finalTcpPC    ->clear();
		finalBluePC   ->clear();
		finalGreenPC  ->clear();
		finalRedPC    ->clear();
		finalYellowPC ->clear();
		finalCyanPC   ->clear();
		finalMagentaPC->clear();

		finalVoxelizedPC       ->clear();
		finalVoxelizedBluePC   ->clear();
		finalVoxelizedGreenPC  ->clear();
		finalVoxelizedRedPC    ->clear();
		finalVoxelizedYellowPC ->clear();
		finalVoxelizedCyanPC   ->clear();
		finalVoxelizedMagentaPC->clear();

		transformation.Ones();
	}
	catch (...)
	{
		ROS_WARN("Vision::reset procedure failed.");
		return false;
	}
	return true;
}
