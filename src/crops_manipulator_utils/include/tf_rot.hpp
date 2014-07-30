#ifndef TF_ROT_H
#define TF_ROT_H

#include <math.h>
#include <ros/ros.h>

//!Only use UNIT QUATERNIONS 

void kardan2dcm(double*,double*);
void dcm2quat(double*,double*);
void quat2dcm(double*,double*);
void dcm2kardan(double*,double*);
void rpy2dcm(double*,double*);
void quat2kardan(double*, double*, double*);
void kardan2quat(double* ,double* ,double* );
void rpy2quat(double*,double*,double*);
int sign(double);


#endif
