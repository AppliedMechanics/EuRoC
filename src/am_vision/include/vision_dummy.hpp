/*
 * VisionDummy.h
 *
 *  Created on: Sep 12, 2014
 *      Author: euroc_admin
 */

#ifndef VISIONDUMMY_H_
#define VISIONDUMMY_H_

#include <vision.hpp>
#include <tf/LinearMath/Quaternion.h>



using namespace cv;

class VisionDummy: public Vision {
public:
	VisionDummy();
	virtual ~VisionDummy();

	void handle(const am_msgs::VisionGoal::ConstPtr &goal);

	std::vector<geometry_msgs::Pose> modscene_poses_;
	std::vector<std::string> modscene_colors_;

	int mod_scene_;
	void loadScene(int scene);
};

#endif /* VISIONDUMMY_H_ */
