/*
 * ampointcloud.cpp
 *
 *  Created on: Aug 6, 2014
 *      Author: sahandy
 *
 *      Description: This class is responsible for creating different point clouds based on filtered RGB + Depth images
 *      			 and finalizing a neat point cloud which only contains those belonging to a certain colored object.
 */

#include "ampointcloud.h"
#include "vision.hpp"
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.7/pcl/sample_consensus/sac.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl-1.7/pcl/visualization/cloud_viewer.h>

using namespace cv;
using namespace std;

//#define DEBUG

// class constructor
am_pointcloud::am_pointcloud(Mat& depth, double fov_horizontal_depth, Mat& rgb, double fov_horizontal_rgb)
{
	_depth = depth;

	_img_height_depth = depth.rows;
	_img_width_depth = depth.cols;

	_fov_horizontal_depth = fov_horizontal_depth;
	_fov_vertical_depth = verticalFov(_img_height_depth, _img_width_depth, _fov_horizontal_depth);

	_rgb = rgb;

	_img_height_rgb = rgb.rows;
	_img_width_rgb = rgb.cols;

	_fov_horizontal_rgb = fov_horizontal_rgb;
	_fov_vertical_rgb = verticalFov(_img_height_rgb, _img_width_rgb, _fov_horizontal_rgb);

	//	CAM_TCP = 0;
	//	CAM_SCENE = 1;

	//_center_y = center_y;
	//_center_z = center_z;
}

am_pointcloud::~am_pointcloud() {
	// TODO Auto-generated destructor stub
}

double am_pointcloud::verticalFov(double height, double width, double fov_horizontal)
{
	double aspectRatio = height / width;
	return 2*atan( tan(fov_horizontal/2) * aspectRatio);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr am_pointcloud::createInitialPointCloud()
{
	pcl::PointXYZ p;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(_depth.cols, _depth.rows));

	double h, v;
	h = tan(_fov_horizontal_depth/2);
	v = tan(_fov_vertical_depth/2);

	for(int i=0; i<_img_height_depth; i++)
	{
		for(int j=0; j<_img_width_depth; j++)
		{

			p.x = _depth.at<float>(i,j);
			p.y = p.x * (h - (2*h* ((float)j/_img_width_depth) ) );
			p.z = p.x * (v - (2*v* ((float)i/_img_height_depth) ) );

			cloud->at(j,i) = p; // first argument = # cols, second argument # rows
		}
	}

	return cloud;
}

/*
 * This function gets the point cloud that is created by Depth sensor and align them with the RGB camera. In other words,
 * indexes of the returned point cloud will correspond to the same point in RGB image.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr am_pointcloud::alignWithRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int cameraType, ros::Time timeStamp)
{

	std::string sourceFrame, targetFrame;

	switch (cameraType) {
	case CAM_TCP:
		// CAM_TCP
		sourceFrame = T_DEPTH;
		targetFrame = T_RGB;
		break;
	case CAM_SCENE:
		// CAM_SCENE
		sourceFrame = S_DEPTH;
		targetFrame = S_RGB;
		break;
	}

	pcl::PointXYZ p;
	pcl::PointCloud<pcl::PointXYZ>::Ptr resultCloud;
	resultCloud.reset(new pcl::PointCloud<pcl::PointXYZ>(_rgb.cols, _rgb.rows));

	double iValue, jValue, temp;

	tf::Vector3 dcs_vec; // Depth coordinate system vector
	tf::Vector3 rgbcs_vec; // RGB coordinate system vector

	ros::Time now = ros::Time().now();

	try
	{
		_tfListener.waitForTransform(targetFrame, sourceFrame, now, ros::Duration(2.0));
		_tfListener.lookupTransform(targetFrame, sourceFrame, ros::Time(0), _transform);
	}
	catch (...)
	{
		ROS_ERROR("Exception: listening to transformation failed");
		ros::Duration(1.0).sleep();
	}

	int X = _img_height_rgb;
	int Y = _img_width_rgb;

	double h, v;
	h = tan(_fov_horizontal_rgb/2);
	v = tan(_fov_vertical_rgb/2);

	for(int i=0; i<cloud->height; i++)
	{
		for(int j=0; j<cloud->width; j++)
		{
			p = cloud->at(j,i);
			if (p.x != 0)
			{

				dcs_vec.setX(p.x);
				dcs_vec.setY(p.y);
				dcs_vec.setZ(p.z);

				rgbcs_vec = _transform(dcs_vec);

				jValue = (h - (rgbcs_vec.getY()/rgbcs_vec.getX())) * (_img_width_rgb/(2*h));
				iValue = (v - (rgbcs_vec.getZ()/rgbcs_vec.getX())) * (_img_height_rgb/(2*v));

				if (0<=iValue && iValue<_img_height_rgb && 0<=jValue && jValue<_img_width_rgb)
				{
					// averaging for points with same X,Y value (but different Z values) does not work well, we choose the closer
					// one by comparing it to the last value on this particular position (pixel)
					if( (p.x <(resultCloud->at( (int)jValue, (int)iValue ).x)) || (resultCloud->at( (int)jValue, (int)iValue).x == 0) )
					{
						resultCloud->at( (int)jValue, (int)iValue ).x = p.x;
						resultCloud->at( (int)jValue, (int)iValue ).y = p.y;
						resultCloud->at( (int)jValue, (int)iValue ).z = p.z;
					}
				}
			}


		}
	}
	return resultCloud;
}

/*
 * This function filters the initial point cloud based on HSV values which are calculated from object color.
 * This enables us to get rid of most of the points which are not relevant to the object we are looking for.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr am_pointcloud::filterPointCloudByColor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Mat &threshold)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr resultCloud;
	resultCloud.reset(new pcl::PointCloud<pcl::PointXYZ>(cloud->width, cloud->height));

	for(int rows=0;rows<threshold.rows;rows++)
	{
		for(int cols=0;cols<threshold.cols;cols++)
		{
			int value = threshold.data[threshold.channels()*(threshold.cols*rows+cols)+0];
			if(value!=255)
			{
				resultCloud->at(cols,rows).x = 0;
				resultCloud->at(cols,rows).y = 0;
				resultCloud->at(cols,rows).z = 0;
			}
			else
			{
				resultCloud->at(cols,rows).x=cloud->at(cols,rows).x;
				resultCloud->at(cols,rows).y=cloud->at(cols,rows).y;
				resultCloud->at(cols,rows).z=cloud->at(cols,rows).z;
			}
		}
	}
	return resultCloud;

}
pcl::PointCloud<pcl::PointXYZ>::Ptr am_pointcloud::removeShape(pcl::PointCloud<pcl::PointXYZ>::Ptr baseCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr shapeCloud)
{
	// search for nearest neighbors of shapeCloud in baseCloud
	// Neighbors within radius search
	baseCloud->is_dense = false;
	shapeCloud->is_dense = false;
	std::vector<int> index;
	pcl::removeNaNFromPointCloud(*baseCloud, *baseCloud, index);
	pcl::removeNaNFromPointCloud(*shapeCloud, *shapeCloud, index);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	try
	{
		kdtree.setInputCloud (baseCloud);
	}
	catch (...)
	{
		emptyCloudCritical = true;
		pcl::PointCloud<pcl::PointXYZ>::Ptr subtractedCloud (new pcl::PointCloud<pcl::PointXYZ>);
		return subtractedCloud;
	}


	float radius = 0.03;

	//  --> search for the neighbor points in baseCloud based on points in shapeCloud
	//  --> remove the points found in baseCloud
	for (int i=0; i<shapeCloud->points.size(); i++)
	{
		pcl::PointXYZ searchPoint;
		searchPoint = shapeCloud->points[i];

		//  --> search for the neighbor points in baseCloud based on points in shapeCloud
		try
		{
			if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
			{
				for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
				{
#ifdef DEBUG
					std::cout << "    "  <<   baseCloud->points[ pointIdxRadiusSearch[j] ].x
							<< " " << baseCloud->points[ pointIdxRadiusSearch[j] ].y
							<< " " << baseCloud->points[ pointIdxRadiusSearch[j] ].z
							<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
#endif // DEBUG
					//  --> remove the points found in baseCloud
					baseCloud->points[ pointIdxRadiusSearch[j] ].x = std::numeric_limits<float>::quiet_NaN();
					baseCloud->points[ pointIdxRadiusSearch[j] ].y = std::numeric_limits<float>::quiet_NaN();
					baseCloud->points[ pointIdxRadiusSearch[j] ].z = std::numeric_limits<float>::quiet_NaN();
				}
			}

		}
		catch (...)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr subtractedCloud (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*baseCloud, *subtractedCloud);
			return subtractedCloud;
		}
	} // END FOR

	std::cout<<"Shape has been removed!"<<std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr subtractedCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*baseCloud, *subtractedCloud);

	return subtractedCloud;

}

/*
 * This function transforms a point cloud (in this case in pan/tilt unit camera coordinate) to the world coordinate system
 * integrity between the new points and their corresponding points in RGB camera remains intact.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr am_pointcloud::transformToWorld(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int cameraType, ros::Time timeStamp)
{
	std::string sourceFrame, targetFrame;

	switch (cameraType) {
	case CAM_TCP:
		// CAM_TCP
		sourceFrame = T_DEPTH;
		break;
	case CAM_SCENE:
		// CAM_SCENE
		sourceFrame = S_DEPTH;
		break;
	}


	ros::Time now = ros::Time().now();
	try
	{
		_tfListener.waitForTransform(ORIGIN, sourceFrame, now, ros::Duration(2.0));
		_tfListener.lookupTransform(ORIGIN, sourceFrame, ros::Time(0), _transform);
	}
	catch (...)
	{
		ROS_ERROR("Exception: listening to transformation failed");
		ros::Duration(1.0).sleep();
	}

	tf::Vector3 camera_vec;
	tf::Vector3 world_vec;
	pcl::PointCloud<pcl::PointXYZ>::Ptr worldPointCloud;

	worldPointCloud.reset(new pcl::PointCloud<pcl::PointXYZ>(cloud->width, cloud->height));

	for (int i=0; i < cloud->height; i++)
	{
		for (int j=0; j < cloud->width; j++)
		{

			if(
					cloud->at(j,i).x == 0
					&& cloud->at(j,i).y == 0
					&& cloud->at(j,i).z == 0
			)
			{
				worldPointCloud->at(j,i).x = std::numeric_limits<float>::quiet_NaN();
				worldPointCloud->at(j,i).y = std::numeric_limits<float>::quiet_NaN();
				worldPointCloud->at(j,i).z = std::numeric_limits<float>::quiet_NaN();
			}
			else
			{
				camera_vec.setX(cloud->at(j,i).x);
				camera_vec.setY(cloud->at(j,i).y);
				camera_vec.setZ(cloud->at(j,i).z);

				world_vec = _transform(camera_vec);

				/*ROS_WARN("camera %f %f %f",camera_vec.getX(),camera_vec.getY(),camera_vec.getZ());
				ROS_WARN("world %f %f %f",world_vec.getX(),world_vec.getY(),world_vec.getZ());*/
				//_tfListener.transformPoint(ORIGIN,_camera_point,_world_point);
				worldPointCloud->at(j,i).x = world_vec.getX();
				worldPointCloud->at(j,i).y = world_vec.getY();
				worldPointCloud->at(j,i).z = world_vec.getZ();
			}
		}
	}

	return worldPointCloud;
}

/*
 * This function will do a second thresholding (Ref: first thresholding is based on color filter) in order to get rid of
 * any irrelevant points, like table surface and target zones.
 * The return point cloud will give us the optimum point cloud consisting only the object.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr am_pointcloud::xyzTheresholdCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double theresholdValue)
{
	pcl::PointXYZ worldPoint;
	pcl::PointXYZ cameraPoint;
	pcl::PointCloud<pcl::PointXYZ>::Ptr theresholdedPointCloud;

	// Hard-coded values, based on table measures (in meters)
	float xthreshold = 1.1;
	float ythreshold = 1.1;

	theresholdedPointCloud.reset(new pcl::PointCloud<pcl::PointXYZ>(cloud->width, cloud->height));

	for (int i=0; i < cloud->height; i++)
	{
		for (int j=0; j < cloud->width; j++)
		{
			if ( (cloud->at(j, i).z > theresholdValue) && ((cloud->at(j, i).x) <= xthreshold) && ((cloud->at(j, i).y) <= ythreshold) && ((cloud->at(j, i).x) >= -xthreshold) && ((cloud->at(j, i).y) >= -ythreshold))
			{
				theresholdedPointCloud->at(j, i) = cloud->at(j, i);
			}
			else
			{
				theresholdedPointCloud->at(j,i).x = std::numeric_limits<float>::quiet_NaN();
				theresholdedPointCloud->at(j,i).y = std::numeric_limits<float>::quiet_NaN();
				theresholdedPointCloud->at(j,i).z = std::numeric_limits<float>::quiet_NaN();
			}

		}
	}

	return theresholdedPointCloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr am_pointcloud::removeRobotFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr theresholdedPointCloud;

	float radius = 0.011;
	float sqrtradius = 0.15;
	float mastmin = 0.86;
	float mastmax = 0.96;

	theresholdedPointCloud.reset(new pcl::PointCloud<pcl::PointXYZ>(cloud->width, cloud->height));

	for (int i=0; i < cloud->height; i++)
	{
		for (int j=0; j < cloud->width; j++)
		{
			if ( ( cloud->at(j, i).x > -sqrtradius) && (cloud->at(j, i).y > -sqrtradius)
					&& (cloud->at(j, i).x < sqrtradius)
					&& (cloud->at(j, i).y < sqrtradius))
			{

				if (((cloud->at(j, i).x * cloud->at(j, i).x) + (cloud->at(j, i).y * cloud->at(j, i).y)) > radius) {

					theresholdedPointCloud->at(j, i) = cloud->at(j, i);
				} else {
					theresholdedPointCloud->at(j, i).x = std::numeric_limits<float>::quiet_NaN();
					theresholdedPointCloud->at(j, i).y = std::numeric_limits<float>::quiet_NaN();
					theresholdedPointCloud->at(j, i).z = std::numeric_limits<float>::quiet_NaN();
				}
			}else if(( cloud->at(j, i).x > mastmin)
					&& (cloud->at(j, i).y > mastmin)
					&& (cloud->at(j, i).x < mastmax)
					&& (cloud->at(j, i).y < mastmax))
			{
				theresholdedPointCloud->at(j, i).x = std::numeric_limits<float>::quiet_NaN();
				theresholdedPointCloud->at(j, i).y = std::numeric_limits<float>::quiet_NaN();
				theresholdedPointCloud->at(j, i).z = std::numeric_limits<float>::quiet_NaN();
			}
			else
				theresholdedPointCloud->at(j, i) = cloud->at(j, i);
		}
	}

	return theresholdedPointCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr am_pointcloud::removeSquareFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointXYZ A,pcl::PointXYZ B,pcl::PointXYZ C,pcl::PointXYZ D)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr theresholdedPointCloud;

	theresholdedPointCloud.reset(new pcl::PointCloud<pcl::PointXYZ>(cloud->width, cloud->height));

	//FIND MIN AND MAX VALUES
	float minx, miny, maxx, maxy;
	minx = A.x;
	miny = A.y;
	maxx = A.x;
	maxy = A.y;
	if (B.x<minx)
		minx=B.x;
	if (B.y<miny)
		miny=B.y;
	if (B.x>maxx)
		maxx=B.x;
	if (B.y>maxy)
		maxy=B.y;
	if (C.x<minx)
		minx=C.x;
	if (C.y<miny)
		miny=C.y;
	if (C.x>maxx)
		maxx=C.x;
	if (C.y>maxy)
		maxy=C.y;
	if (D.x<minx)
		minx=D.x;
	if (D.y<miny)
		miny=D.y;
	if (D.x>maxx)
		maxx=D.x;
	if (D.y>maxy)
		maxy=D.y;



	for (int i=0; i < cloud->height; i++)
	{
		for (int j = 0; j < cloud->width; j++) {
			if ((cloud->at(j, i).x < maxx + 0.01)
					&& (cloud->at(j, i).y < maxy + 0.01)
					&& (cloud->at(j, i).x > minx + 0.01)
					&& (cloud->at(j, i).y > miny + 0.01)) {
				if ((triangleArea(A, B, cloud->at(j, i))
						+ triangleArea(B, C, cloud->at(j, i))
						+ triangleArea(C, D, cloud->at(j, i))
						+ triangleArea(D, A, cloud->at(j, i)))
						> (triangleArea(A, B, C) + triangleArea(A, C, D) + 0.01)) {
					theresholdedPointCloud->at(j, i) = cloud->at(j, i);
				} else {
					theresholdedPointCloud->at(j, i).x = std::numeric_limits<float>::quiet_NaN();
					theresholdedPointCloud->at(j, i).y = std::numeric_limits<float>::quiet_NaN();
					theresholdedPointCloud->at(j, i).z = std::numeric_limits<float>::quiet_NaN();
				}
			}
			else
				theresholdedPointCloud->at(j, i) = cloud->at(j, i);
		}
	}

	return theresholdedPointCloud;
}

float am_pointcloud::triangleArea(pcl::PointXYZ A,pcl::PointXYZ B,pcl::PointXYZ C)
{

    float side_a = sqrt((B.x-A.x) * (B.x-A.x) + (B.y-A.y) * (B.y-A.y));

    float side_b = sqrt((B.x-C.x) * (B.x-C.x) + (B.y-C.y) * (B.y-C.y));

    float side_c = sqrt((A.x-C.x) * (A.x-C.x) + (A.y-C.y) * (A.y-C.y));


    // Heron's formula for area calculation
    // area = sqrt( s * (s-a) * (s-b) * (s-c))

    float s = (side_a + side_b + side_c) / 2;

	return sqrt( s * (s-side_a) * (s-side_b) * (s-side_c));
}



/*
 * This function roughly calculates the center of mass from a set of points in a point cloud.
 */
pcl::PointXYZ am_pointcloud::calculateCenterOfMass(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointXYZ p;
	p.x=0;
	p.y=0;
	p.z=0;

	int counter=0;
	for(int i=0; i<cloud->height; i++)
	{
		for(int j=0; j<cloud->width; j++)
		{
			if( pcl_isfinite(cloud->at(j,i).x)
					||pcl_isfinite(cloud->at(j,i).y)
					||pcl_isfinite(cloud->at(j,i).z) )
			{
				p.x+=cloud->at(j,i).x;
				p.y+=cloud->at(j,i).y;
				p.z+=cloud->at(j,i).z;
				counter++;
			}
		}
	}
	p.x/=counter;
	p.y/=counter;
	p.z/=counter;
	return p;
}

/*
 * This function receives a point cloud and a vector of indices as input and removes all the points corresponding to the vector
 * from that point cloud. The returned cloud should be the same as input, minus those from indices vector
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr am_pointcloud::removeCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr targetPC, pcl::PointCloud<pcl::PointXYZ>::Ptr inputClusterPC, std::vector<pcl::PointIndices> clusterIndices)
{
  for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
  {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                  cloud_cluster->points.push_back (inputClusterPC->points[*pit]);
          cloud_cluster->width = cloud_cluster->points.size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;

          // search for nearest neighbors of clusterPC in targetPC
          // Neighbors within radius search

          std::vector<int> pointIdxRadiusSearch;
          std::vector<float> pointRadiusSquaredDistance;

          pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
          kdtree.setInputCloud (targetPC);
          float radius = 0.005;

          // iterate over the cluster#
          //  --> search for the neighbor points in targetPC
          //  --> remove the points found in targetPC
          for (int i=0; i<cloud_cluster->points.size(); i++)
          {
            pcl::PointXYZ searchPoint;
            searchPoint = cloud_cluster->points[i];

            if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
            {
              for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
              {
#ifdef DEBUG
                std::cout << "    "  <<   targetPC->points[ pointIdxRadiusSearch[j] ].x
                    << " " << targetPC->points[ pointIdxRadiusSearch[j] ].y
                    << " " << targetPC->points[ pointIdxRadiusSearch[j] ].z
                    << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
#endif // DEBUG
                targetPC->points[ pointIdxRadiusSearch[j] ].x = std::numeric_limits<float>::quiet_NaN();
                targetPC->points[ pointIdxRadiusSearch[j] ].y = std::numeric_limits<float>::quiet_NaN();
                targetPC->points[ pointIdxRadiusSearch[j] ].z = std::numeric_limits<float>::quiet_NaN();
              }
            }
          } // END FOR

          std::cout<<"Done removing the cluster!"<<std::endl;
  }
  return targetPC;
}



octomath::Vector3 am_pointcloud::getSensorOriginScene (int cameraType)
{
	octomath::Vector3 SensorOrigin;
	std::string sourceFrame, targetFrame;

	switch (cameraType) {
	case CAM_TCP:
		// CAM_TCP
		sourceFrame = T_DEPTH;
		targetFrame = T_RGB;
		break;
	case CAM_SCENE:
		// CAM_SCENE
		sourceFrame = S_DEPTH;
		targetFrame = S_RGB;
		break;
	}

	ros::Time now = ros::Time::now();
	try
	{
		_tfListener.waitForTransform(sourceFrame, targetFrame, now, ros::Duration(2.0));
		_tfListener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), _transform);
	}
	catch (...)
	{
		ROS_ERROR("Exception: listening to transformation failed");
		ros::Duration(1.0).sleep();
	}

	SensorOrigin.x() = _transform.getOrigin().getX();
	SensorOrigin.y() = _transform.getOrigin().getY();
	SensorOrigin.z() = _transform.getOrigin().getZ();

	return SensorOrigin;
}
