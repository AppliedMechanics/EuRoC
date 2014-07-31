#include <tf_rot.hpp>


//!Only use UNIT QUATERNIONS

int sign(double value)
{
  if (value<0)
    return -1;
  else
    return 1;
}

void kardan2dcm(double* kardan,double* r_tf)
{

  double sinalpha=sin(kardan[0]);
  double sinbeta=sin(kardan[1]);
  double singamma=sin(kardan[2]);
  double cosalpha=cos(kardan[0]);
  double cosbeta=cos(kardan[1]);
  double cosgamma=cos(kardan[2]);

  //! DCM Definition
  r_tf[0] = cosbeta*cosgamma;
  r_tf[1] = cosalpha*singamma+cosgamma*sinalpha*sinbeta;
  r_tf[2] = sinalpha*singamma-cosalpha*cosgamma*sinbeta;
  r_tf[3] = -cosbeta*singamma;
  r_tf[4] = cosalpha*cosgamma-sinalpha*sinbeta*singamma;
  r_tf[5] = cosgamma*sinalpha+cosalpha*sinbeta*singamma;
  r_tf[6] = sinbeta;
  r_tf[7] = -cosbeta*sinalpha;
  r_tf[8] = cosalpha*cosbeta;

}

void dcm2quat(double* r_tf, double* q_tf)
{
  double normq;
  //! Prevent Error in DCM
  for (int i=0;i<9;i++){
    if (r_tf[i]<-1.0)
      r_tf[i] = -1.0;
    else if (r_tf[i]>1.0)
      r_tf[i] = 1.0;
  }

  //!Only use UNIT QUATERNIONS
  q_tf[0] = 0.5*sqrt(r_tf[0]+r_tf[4]+r_tf[8]+1);
  q_tf[1] = 0.5*sign(r_tf[5]-r_tf[7])*sqrt(r_tf[0]-r_tf[4]-r_tf[8]+1);
  q_tf[2] = 0.5*sign(r_tf[6]-r_tf[2])*sqrt(-r_tf[0]+r_tf[4]-r_tf[8]+1);
  q_tf[3] = 0.5*sign(r_tf[1]-r_tf[3])*sqrt(-r_tf[0]-r_tf[4]+r_tf[8]+1);

  normq = q_tf[0]*q_tf[0]+q_tf[1]*q_tf[1]+q_tf[2]*q_tf[2]+q_tf[3]*q_tf[3];
  if (normq != 1.0)
    {
      normq = sqrt(normq);
      q_tf[0] = q_tf[0]/normq;
      q_tf[1] = q_tf[1]/normq;
      q_tf[2] = q_tf[2]/normq;
      q_tf[3] = q_tf[3]/normq;
    }


}

void quat2dcm(double* q_tf, double* r_tf)
{
  //! Better use unit quaternions
  double w=q_tf[0];
  double x=q_tf[1];
  double y=q_tf[2];
  double z=q_tf[3];
  double normq;

  // Normalize quaternions
  normq = (x*x+y*y+z*z+w*w);
  if (normq == 0.0)
    {
      normq = 1.0;
    }
  else
    {
      normq = sqrt(normq);

      x = x/normq;
      y = y/normq;
      z = z/normq;
      w = w/normq;
    }
  
  // r_tf[0] = 2*(w*w+x*x)-1;
  // r_tf[1] = 2*(x*y-w*z);
  // r_tf[2] = 2*(x*z+w*y);
  // r_tf[3] = 2*(x*y+w*z);
  // r_tf[4] = 2*(w*w+y*y)-1;
  // r_tf[5] = 2*(y*z-w*x);
  // r_tf[6] = 2*(x*z-w*y);
  // r_tf[7] = 2*(y*z+w*x);
  // r_tf[8] = 2*(w*w+z*z)-1;


  r_tf[0] = w*w+x*x-y*y-z*z;
  r_tf[1] = 2*x*y+2*w*z;
  r_tf[2] = 2*x*z-2*w*y;
  r_tf[3] = 2*x*y-2*w*z;
  r_tf[4] = w*w-x*x+y*y-z*z;
  r_tf[5] = 2*y*z+2*w*x;
  r_tf[6] = 2*x*z+2*w*y;
  r_tf[7] = 2*y*z-2*w*x;
  r_tf[8] = w*w-x*x-y*y+z*z;

  //! Prevent Error in DCM
  for (int i=0;i<9;i++){
    if (r_tf[i]<-1.0)
      r_tf[i] = -1.0;
    else if (r_tf[i]>1.0)
      r_tf[i] = 1.0;
  }


}

void dcm2kardan(double* r_tf, double* a_tf)
{
  // Calculate kardan angles from Rotation matrix
  if (r_tf[6]>1.0)
    r_tf[6] = 1.0;
  else if (r_tf[6]<-1.0)
    r_tf[6] = -1.0;

  // //Calculate kardan angles from Rotation matrix
  // a_tf[1] = asin(r_tf[2]);
  // if (cos(a_tf[1])>1e-10){
  //   a_tf[0] = atan2(-r_tf[5],r_tf[8]);
  //   a_tf[2] = atan2(-r_tf[1],r_tf[0]);
  // }
  // else{
  //   a_tf[1] = 0;
  //   a_tf[2] = atan2(r_tf[3],r_tf[4]);
  // }

  //! Stanford
  a_tf[1] = asin(r_tf[6]);
  if (cos(a_tf[1])>1e-10){
    a_tf[0] = atan2(-r_tf[7],r_tf[8]);
    a_tf[2] = atan2(-r_tf[3],r_tf[0]);
  }
  else{
    a_tf[0] = 0;
    a_tf[2] = atan2(r_tf[3],r_tf[4]);
  }

}

void rpy2dcm(double* rpy,double* r_tf)
{
	ROS_WARN("test me!");

	double sinalpha=sin(rpy[0]);
	double sinbeta=sin(rpy[1]);
	double singamma=sin(rpy[2]);
	double cosalpha=cos(rpy[0]);
	double cosbeta=cos(rpy[1]);
	double cosgamma=cos(rpy[2]);

	r_tf[0]=cosalpha*cosbeta;
	r_tf[1]=cosalpha*sinbeta*singamma-sinalpha*cosgamma;
	r_tf[2]=cosalpha*sinbeta*cosgamma+sinalpha*singamma;
	r_tf[3]=sinalpha*cosbeta;
	r_tf[4]=sinalpha*sinbeta*singamma+cosalpha*cosgamma;
	r_tf[5]=sinalpha*sinbeta*cosgamma-cosalpha*singamma;
	r_tf[6]=-sinbeta;
	r_tf[7]=cosbeta*singamma;
	r_tf[8]=cosbeta*cosgamma;
}


void quat2kardan(double* q_tf,double* a_tf,double* r_tf)
{
  //!Only use UNIT QUATERNIONS
  quat2dcm(q_tf,r_tf);
  dcm2kardan(r_tf,a_tf);

}

void kardan2quat(double* a_tf,double* q_tf,double* r_tf)
{
  //!Only use UNIT QUATERNIONS
  kardan2dcm(a_tf,r_tf);
  dcm2quat(r_tf,q_tf);

}

void rpy2quat(double* a_tf,double* q_tf, double* r_tf)
{
	//!Only use UNIT QUATERNIONS
	rpy2dcm(a_tf,r_tf);
	dcm2quat(r_tf,q_tf);
}
