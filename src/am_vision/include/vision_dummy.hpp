/*
 * VisionDummy.h
 *
 *  Created on: Sep 12, 2014
 *      Author: euroc_admin
 */

#ifndef VISIONDUMMY_H_
#define VISIONDUMMY_H_

#include <vision.hpp>



using namespace cv;

class VisionDummy: public Vision {
public:
	VisionDummy();
	virtual ~VisionDummy();

	void handle(const am_msgs::VisionGoal::ConstPtr &goal);
};

#endif /* VISIONDUMMY_H_ */
