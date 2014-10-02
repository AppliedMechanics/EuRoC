#ifndef __VISION_HPP__
#define __VISION_HPP__


#include <ros/ros.h>

// Local includes
#include <actionlib/server/simple_action_server.h>
#include <am_msgs/VisionAction.h>
#include "MovePantilt.h"
#include <ampointcloud.h>
#include <am_msgs/TakeImage.h>
#include <shape_generator.cpp>

// Octomap includes
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/math/Vector3.h>


// PCL includes
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl-1.7/pcl/visualization/cloud_viewer.h>
#include <pcl-1.7/pcl/pcl_macros.h>
#include <pcl/keypoints/uniform_sampling.h>

// Includes for received topic messages
#include <euroc_c2_msgs/Telemetry.h>
#include <sensor_msgs/Image.h>

// Includes to show opencv image
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

using namespace cv;

class Vision
{

private:
	ros::Time finalTimeStamp;

	// Constants
	static const int CAM_TCP = 0;
	static const int CAM_SCENE = 1;
//	static const int CLOSE_RANGE_POSE = 2;
//	static const int OBJECT_ON_TARGET_VERIFICATION = 3;
	static const int HANDLE = 4;
	static const int CYLINDER = 5;
	static const int CUBE = 6;

	// topic and service names
	std::string euroc_c2_interface;
	std::string camera_scene_rgb_topic;
	std::string camera_scene_depth_topic;
	std::string camera_tcp_rgb_topic;
	std::string camera_tcp_depth_topic;
	std::string save_log;

	int task_nr; // store parameter from active_task_number
	bool isSingleCube; // verifies whether the current object is a single cube
	ros::NodeHandle nh_;

	std::string vision_action_name_;
protected:
	actionlib::SimpleActionServer<am_msgs::VisionAction> vision_server_;

public:
	// Eigen: alignment issues: http://eigen.tuxfamily.org/dox-devel/group__DenseMatrixManipulation__Alignement.html
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Vision();
	~Vision(){;};

	void on_camera_scene_rgb_CB(const sensor_msgs::Image &image);
	void on_camera_scene_depth_CB(const sensor_msgs::Image &image);
	void on_camera_tcp_rgb_CB(const sensor_msgs::Image &image);
	void on_camera_tcp_depth_CB(const sensor_msgs::Image &image);
	bool on_take_image_CB(am_msgs::TakeImage::Request &req, am_msgs::TakeImage::Response &res);
	void scan_with_pan_tilt();
	void scan_with_tcp();
	virtual void handle(const am_msgs::VisionGoal::ConstPtr &goal);
	Eigen::Matrix4f align_PointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr object_input, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_input, bool box, bool cylinder);

	double close_range_pose(string);
	void sort_corners(std::vector<cv::Point2f>&);
	std::vector<int> find_perpendicular_lines(std::vector<cv::Vec4i>&);
	cv::Point2f compute_intersect(cv::Vec4i, cv::Vec4i);
	std::vector<pcl::PointXYZ> find_points_world(pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<cv::Point2f>);

	// create messages that are used to published feedback/result
	am_msgs::VisionFeedback vision_feedback_;
	am_msgs::VisionResult   vision_result_;


	ros::ServiceServer take_img_service_;

	bool failed;

	Mat HSV;
	Mat cameraFeed;
	Mat threshold;
	Mat thresholdBlue;
	Mat thresholdGreen;
	Mat thresholdRed;
	Mat thresholdYellow;
	Mat thresholdCyan;
	Mat thresholdMagenta;

	cv_bridge::CvImagePtr _cv_image;
	cv_bridge::CvImagePtr _cv_depthptr;

	sensor_msgs::Image _scene_rgb_image;
	sensor_msgs::Image _scene_depth_image;
	sensor_msgs::Image _tcp_rgb_image;
	sensor_msgs::Image _tcp_depth_image;

	sensor_msgs::PointCloud2 msg;

	ros::Subscriber camera_scene_rgb_subscriber;
	ros::Subscriber camera_scene_depth_subscriber;
	ros::Subscriber camera_tcp_rgb_subscriber;
	ros::Subscriber camera_tcp_depth_subscriber;

	ros::Publisher pub;
	ros::Publisher pub_2;
	ros::Publisher pub_3;

	// Octomap/ Octree
	// You can change resolution here
	octomap::OcTree* tree;
	octomap::Pointcloud* OctoCloud;

	//am_pointcloud *scenePointCloud;
	//am_pointcloud *tcpPointCloud;

	tf::Quaternion tfqt;
	tf::Quaternion tfqtNew;
	Eigen::Matrix4f transformation;

	pcl::PointCloud<pcl::PointXYZ>::Ptr finalScenePC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalTcpPC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalBluePC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalGreenPC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalRedPC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalYellowPC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCyanPC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalMagentaPC;

	pcl::PointCloud<pcl::PointXYZ>::Ptr finalVoxelizedPC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalVoxelizedBluePC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalVoxelizedGreenPC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalVoxelizedRedPC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalVoxelizedYellowPC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalVoxelizedCyanPC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalVoxelizedMagentaPC;

	//alignemt was successfull
	bool obj_aligned_;

	// size of voxels
	float leaf_size;
};

#endif //VISION_HPP__
