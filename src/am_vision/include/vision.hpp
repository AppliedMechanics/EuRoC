#ifndef __VISION_HPP__
#define __VISION_HPP__


#include <ros/ros.h>

// Local includes
#include <actionlib/server/simple_action_server.h>
#include <am_msgs/VisionAction.h>
#include "MovePantilt.h"
#include <ampointcloud.h>
#include <am_msgs/TakeImage.h>
#include <am_msgs/CheckZones.h>
#include <am_msgs/ObjState.h>
#include <std_srvs/Empty.h>
#include <shape_generator.cpp>

// Octomap includes
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/math/Vector3.h>
#include <octomap_msgs/BoundingBoxQuery.h>

// PCL includes
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/common.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl-1.7/pcl/visualization/cloud_viewer.h>
#include <pcl-1.7/pcl/pcl_macros.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/octree/octree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>


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
	ros::NodeHandle nh_;
	std::string vision_action_name_;
	int task_nr; // store parameter from active_task_number

	ros::Time finalTimeStamp;
	ros::Time tic;
	ros::Time tac;

	// Constants
	static const int CAM_TCP = 0;
	static const int CAM_SCENE = 1;
	static const int HANDLE = 4;
	static const int CYLINDER = 5;
	static const int CUBE = 6;
	static const float cube_size=0.05;

	//Added By İrem -Parameter Names
	int total_nr_of_objects;
	int problem_object_index;
	std::vector<std::string> Colour_String;
	std::vector<int> Number_of_Shapes_List;
	bool same_color_problem; //two objects which are in same color
	double cylinder_radius;
	double cylinder_length;
	std::vector<double>trouble_object_cube_size_vector;
	std::vector<double>trouble_object_orientation_vector;
	std::vector<double>trouble_object_position_vector;
	std::vector<double>puzzle_fixture_position_vector;
	std::vector<double>puzzle_fixture_orientation_vector;

	// topic and service names
	std::string euroc_c2_interface;
	std::string camera_scene_rgb_topic;
	std::string camera_scene_depth_topic;
	std::string camera_tcp_rgb_topic;
	std::string camera_tcp_depth_topic;
	std::string save_log;

	am_msgs::VisionGoal::ConstPtr _currentGoal;

	Mat HSV;
	Mat cameraFeed;
	Mat threshold;
	Mat thresholdBlue;
	Mat thresholdGreen;
	Mat thresholdRed;
	Mat thresholdYellow;
	Mat thresholdCyan;
	Mat thresholdMagenta;

	pcl::PointCloud<pcl::PointXYZ>::Ptr finalPC;
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

	pcl::PointCloud<pcl::PointXYZ>::Ptr object_model;

	pcl::PointCloud<pcl::PointXYZ>::Ptr lastShapeToRemovePC;

	// size of voxels
	float leaf_size;
	float small_leaf_size;

	tf::Quaternion tfqt;
	tf::Quaternion tfqtNew;
	Eigen::Matrix4f transformation;

	bool isSingleCube; // verifies whether the current object is a single cube
	bool is_task5;

protected:
	actionlib::SimpleActionServer<am_msgs::VisionAction> vision_server_;
	void scan_with_pan_tilt(am_msgs::TakeImage::Response &res,bool scan_full);
	void scan_with_tcp(am_msgs::TakeImage::Response &res);
	Eigen::Matrix4f align_PointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr object_input, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_input, bool box, bool cylinder);
	Eigen::Matrix4f fast_cube_alignment(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	double close_range_pose(string);
	void sort_corners(std::vector<cv::Point2f>&);
	std::vector<int> find_perpendicular_lines(std::vector<cv::Vec4i>&);
	cv::Point2f compute_intersect(cv::Vec4i, cv::Vec4i);
	std::vector<pcl::PointXYZ> find_points_world(pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<cv::Point2f>);
	int verify_close_range_pose(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);
	int verify_object_inside_zone(std::string, pcl::PointXYZ, float);
	bool search_for_object_on_zone(pcl::PointCloud<pcl::PointXYZ >::Ptr, pcl::PointXYZ, float);
	pcl::PointCloud<pcl::PointXYZ >::Ptr find_clusters(pcl::PointCloud<pcl::PointXYZ >::Ptr, pcl::PointCloud<pcl::PointXYZ >::Ptr, const am_msgs::VisionGoal::ConstPtr &);
	float get_shape_length(const am_msgs::VisionGoal::ConstPtr &);
	bool compare_cluster_differences(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);
	pcl::PointCloud<pcl::PointXYZ >::Ptr get_task6_cloud();
	bool master_reset();

public:
	// Eigen: alignment issues: http://eigen.tuxfamily.org/dox-devel/group__DenseMatrixManipulation__Alignement.html
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Vision();
	virtual ~Vision(){;};

	void on_camera_scene_rgb_CB(const sensor_msgs::Image &image);
	void on_camera_scene_depth_CB(const sensor_msgs::Image &image);
	void on_camera_tcp_rgb_CB(const sensor_msgs::Image &image);
	void on_camera_tcp_depth_CB(const sensor_msgs::Image &image);
	bool on_take_image_CB(am_msgs::TakeImage::Request &, am_msgs::TakeImage::Response &);
	bool on_check_zones_CB(am_msgs::CheckZones::Request &req, am_msgs::CheckZones::Response &res);
	void get_object_state_CB(const am_msgs::ObjState::ConstPtr& msg);
	virtual void handle(const am_msgs::VisionGoal::ConstPtr &);
	pcl::PointCloud<pcl::PointXYZ>::Ptr fake_object_creater(bool);
	pcl::PointCloud<pcl::PointXYZ>::Ptr reduced_object_input(pcl::PointCloud<pcl::PointXYZ>::Ptr object_input,pcl::PointCloud<pcl::PointXYZ>::Ptr scene_input,bool box, bool cylinder);
	Eigen::Matrix4f align_PointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr, bool, bool, bool);
	bool same_colored_objects(const am_msgs::VisionGoal::ConstPtr &);
	pcl::PointCloud<pcl::PointXYZ>::Ptr fake_puzzle_fixture();
	void fake_puzzle_fixture_param();
	pcl::PointCloud<pcl::PointXYZ>::Ptr removeShape(pcl::PointCloud<pcl::PointXYZ>::Ptr baseCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr shapeCloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr removeInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr object_input);

	//void Vision::Octomap_Update(OcTree* tree);

	// create messages that are used to published feedback/result
	am_msgs::VisionFeedback vision_feedback_;
	am_msgs::VisionResult   vision_result_;

	ros::ServiceServer take_img_service_;
	ros::ServiceServer check_zones_service_;
	ros::ServiceClient reset_octomap_client_;
	octomap_msgs::BoundingBoxQuery reset_octomap_bbx_srv_;

	bool failed;
	//alignemt was successfull
	bool obj_aligned_;
	bool isFinalClusterEmpty;

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
	// object state message subscriber
	ros::Subscriber obj_state_sub_;

	ros::Publisher pub;
	ros::Publisher pub_2;
	ros::Publisher pub_3;

	// Octomap/ Octree
	// You can change resolution here
	octomap::OcTree* tree;
	octomap::Pointcloud* OctoCloud;
};

#endif //VISION_HPP__
