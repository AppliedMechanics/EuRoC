/*
 * ImageFilter.h
 *
 *  Created on: Aug 8, 2014
 *      Author: sahandy
 */

#ifndef IMAGEFILTER_H_
#define IMAGEFILTER_H_

#include <opencv2/opencv.hpp>
#include <string.h>
#include <SimpleColorObj.h>

using namespace cv;
using namespace std;

class ImageFilter {
private:

	Mat _rgb;

	int H_MIN;
	int H_MAX;
	int S_MIN;
	int S_MAX;
	int V_MIN;
	int V_MAX;

	int MAX_NUM_OBJECTS;

	int FRAME_WIDTH;
	int FRAME_HEIGHT;
	//minimum and maximum object area
	int MIN_OBJECT_AREA;
	int MAX_OBJECT_AREA;



public:
	ImageFilter(Mat &rgb);
	virtual ~ImageFilter();
	void setHsvValues(string color);
	string intToString(int number);
	void drawObject(vector<SimpleColorObj> colorObjects, Mat &frame);
	void morphOps(Mat &thresh);
	void trackFilteredObject(Mat threshold, Mat HSV, Mat& cameraFeed);
	Mat getFilteredImage();

};

#endif /* IMAGEFILTER_H_ */
