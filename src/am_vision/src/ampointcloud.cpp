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
pcl::PointCloud<pcl::PointXYZ>::Ptr am_pointcloud::alignWithRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int cameraType)
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

	tf::Vector3 dcs_vec;
	tf::Vector3 rgbcs_vec;

	ros::Time now = ros::Time::now();
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

/*
 * This function transforms a point cloud (in this case in pan/tilt unit camera coordinate) to the world coordinate system
 * integrity between the new points and their corresponding points in RGB camera remains intact.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr am_pointcloud::transformToWorld(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int cameraType)
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


	ros::Time now = ros::Time::now();
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
	float xthreshold = 2;
	float ythreshold = 2;

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

  theresholdedPointCloud.reset(new pcl::PointCloud<pcl::PointXYZ>(cloud->width, cloud->height));

  for (int i=0; i < cloud->height; i++)
  {
          for (int j=0; j < cloud->width; j++)
          {
                  if ( ( (cloud->at(j, i).x * cloud->at(j, i).x) + (cloud->at(j, i).y * cloud->at(j, i).y) ) > radius )
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
