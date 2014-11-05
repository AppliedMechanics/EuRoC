/*
 * ampointcloud.h
 *
 *  Created on: Aug 6, 2014
 *      Author: sahandy
 */

#ifndef AMPOINTCLOUD_H_
#define AMPOINTCLOUD_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <pcl-1.7/pcl/visualization/cloud_viewer.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <octomap/octomap.h>

#include <config.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

using namespace cv;

class am_pointcloud {

private:
	Mat _depth;
	Mat _rgb;
	double _fov_horizontal_depth;
	double _fov_vertical_depth;
	double _fov_horizontal_rgb;
	double _fov_vertical_rgb;
	double _img_height_depth;
	double _img_width_depth;
	double _img_height_rgb;
	double _img_width_rgb;

	static const int CAM_TCP = 0;
	static const int CAM_SCENE = 1;

	ros::NodeHandle _nh;
	tf::TransformListener _tfListener;
	tf::StampedTransform _transform;
	tf::Transform _tf_object;
	geometry_msgs::PointStamped _camera_point;
	geometry_msgs::PointStamped _world_point;

	//unsigned int _center_y; // right hand coordinate system
	//unsigned int _center_z; // right hand coordinate system

public:
	am_pointcloud(Mat &, double, Mat &, double);
	virtual ~am_pointcloud();
	double verticalFov(double, double, double);
	pcl::PointCloud<pcl::PointXYZ>::Ptr createInitialPointCloud();
	pcl::PointCloud<pcl::PointXYZ>::Ptr alignWithRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr, int, ros::Time);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloudByColor(pcl::PointCloud<pcl::PointXYZ>::Ptr, Mat &);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformToWorld(pcl::PointCloud<pcl::PointXYZ>::Ptr, int, ros::Time);
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyzTheresholdCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr,double);
	pcl::PointCloud<pcl::PointXYZ>::Ptr removeRobotFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	pcl::PointCloud<pcl::PointXYZ>::Ptr fillPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	static pcl::PointXYZ calculateCenterOfMass(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr removeCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<pcl::PointIndices>);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr removeShape(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);
	octomath::Vector3 getSensorOriginScene (int cameraType);


};

#endif /* AMPOINTCLOUD_H_ */
