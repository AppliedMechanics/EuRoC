/*
 * SimpleColorObj.h
 *
 *  Created on: Aug 8, 2014
 *      Author: sahandy
 */

#ifndef SIMPLECOLOROBJ_H_
#define SIMPLECOLOROBJ_H_

#include <string.h>

using namespace std;

class SimpleColorObj {
public:
	SimpleColorObj();
	virtual ~SimpleColorObj();
	void setX(int);
	void setY(int);
	int getX();
	int getY();

private:
	int xPos, yPos;
	string type;
};

#endif /* SIMPLECOLOROBJ_H_ */
