/*
 * SimpleColorObj.cpp
 *
 *  Created on: Aug 8, 2014
 *      Author: sahandy
 */

#include <SimpleColorObj.h>

SimpleColorObj::SimpleColorObj() {
	// TODO Auto-generated constructor stub

}

SimpleColorObj::~SimpleColorObj() {
	// TODO Auto-generated destructor stub
}

void SimpleColorObj::setX(int x)
{
	xPos = x;
}

void SimpleColorObj::setY(int y)
{
	yPos = y;
}

int SimpleColorObj::getX()
{
	return xPos;
}

int SimpleColorObj::getY()
{
	return yPos;
}
