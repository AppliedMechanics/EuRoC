/*
 * ImageFilter.cpp
 *
 *  Created on: Aug 8, 2014
 *      Author: sahandy
 *
 *      Description: This class is responsible for getting an RGB image and applying filters (i.e. HSV)
 *      in order to segment certain colored objects.
 */

#include <ImageFilter.h>
#include "SimpleColorObj.cpp"
#include <string.h>


using namespace cv;
using namespace std;

ImageFilter::ImageFilter(Mat &rgb) {

	_rgb = rgb;

	// initialize HSV values
	H_MIN = 0;
	H_MAX = 255;

	S_MIN = 0;
	S_MAX = 255;

	V_MIN = 0;
	V_MAX = 255;

	FRAME_HEIGHT = rgb.rows;
	FRAME_WIDTH = rgb.cols;

	MAX_NUM_OBJECTS = 50; // HARD-CODED
	MIN_OBJECT_AREA = 10*10; // compute heuristically
	MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

}

ImageFilter::~ImageFilter() {

}

/*
 * This function sets the HSV values based on the color given by the state machine.
 * These HSV values will then be used for filtering the image.
 */
void ImageFilter::setHsvValues(string color)
{
	if (!color.compare("0000ff")) // BLUE
	{
		H_MIN = 110;
		H_MAX = 135;

		S_MIN = 50;
		S_MAX = 255;

		V_MIN = 0;
		V_MAX = 255;
	}
	else if (!color.compare("00ff00")) // GREEN
	{
		H_MIN = 57;
		H_MAX = 85;

		S_MIN = 50;
		S_MAX = 255;

		V_MIN = 0;
		V_MAX = 255;
	}
	else if (!color.compare("00ffff")) // CYAN
	{
		H_MIN = 85;
		H_MAX = 100;

		S_MIN = 50;
		S_MAX = 255;

		V_MIN = 0;
		V_MAX = 255;
	}
	else if (!color.compare("ff0000")) // RED
	{
		H_MIN = 0;
		H_MAX = 20;

		S_MIN = 210;
		S_MAX = 255;

		V_MIN = 0;
		V_MAX = 255;
	}
	else if (!color.compare("ff00ff")) // MAGENTA
	{
		H_MIN = 140;
		H_MAX = 160;

		S_MIN = 50;
		S_MAX = 255;

		V_MIN = 0;
		V_MAX = 255;
	}
	else if (!color.compare("ffff00")) // YELLOW
	{
		H_MIN = 20;
		H_MAX = 35;

		S_MIN = 135;
		S_MAX = 255;

		V_MIN = 0;
		V_MAX = 255;
	}
}

string ImageFilter::intToString(int number)
{
	stringstream ss;
	ss << number;
	return ss.str();
}

void ImageFilter::drawObject(vector<SimpleColorObj> colorObjects, Mat &frame)
{
	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!
	for(int i=0; i<colorObjects.size(); i++)
	{
		circle(frame, Point(colorObjects.at(i).getX(), colorObjects.at(i).getY()), 10, Scalar(0,0,255));
		putText(frame, intToString(colorObjects.at(i).getX())+ " , " + intToString(colorObjects.at(i).getY()), Point(colorObjects.at(i).getX(), colorObjects.at(i).getY()+20), 1, 1, Scalar(0,255,0));
	}

}
void ImageFilter::morphOps(Mat &thresh)
{
	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);


	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
}
void ImageFilter::trackFilteredObject(Mat threshold, Mat HSV, Mat &cameraFeed)
{
	vector<SimpleColorObj> colorObjects;

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0)
	{
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects < MAX_NUM_OBJECTS)
		{
			for (int index = 0; index >= 0; index = hierarchy[index][0])
			{

				Moments moment = moments((Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than (MIN_OBJECT_AREA x MIN_OBJECT_AREA), then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA)
				{
					SimpleColorObj singleObj;
					singleObj.setX(moment.m10/area);
					singleObj.setY(moment.m01/area);

					colorObjects.push_back(singleObj);

					objectFound = true;
					refArea = area;
				}
				else objectFound = false;
			}
			//let user know you found an object
			if(objectFound ==true)
			{
				putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				//draw object location on screen
				drawObject(colorObjects,cameraFeed);
			}

		}
		else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}

/*
 * This function returns a filtered image based on predefined HSV values as threshold.
 * The threshold values are provided by ImageFilter::setHsvValues(...)
 */
Mat ImageFilter::getFilteredImage()
{
	Mat HSV, threshold;
	// Change the color format from BGR to HSV
	cvtColor(_rgb, HSV, CV_BGR2HSV);

	// Filter HSV image between values and store filtered image to threshold matrix
	inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);

	return threshold;
}
