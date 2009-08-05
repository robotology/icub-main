
#include "hough_transform.h"

#include <yarp/sig/ImageDraw.h>

using namespace yarp;
using namespace yarp::sig;
using namespace yarp::sig::draw;

// FROM: http://www.cpsc.ucalgary.ca/Research/vision/hough.c
//My image structure has an INFO structure in it and a 2D pixel
//array called DATA. In INFO we have NC, number of columns, and
//NR, the number of rows. Rows are the first index.	   


template <class T>
T CORRECT(T x)
{
  if (x>360) x -= 360;
  if (x<-360) x += 360;
}


float dist(float x, float y)
{
  float d1 = fabs(x-y);
  float d2 = fabs(x-y+180);
  float d3 = fabs(x-y-180);
  if (d2<d1) d1 = d2;
  if (d3<d1) d1 = d3;
  return d1;
}

int hough_transform(yarp::sig::ImageOf<yarp::sig::PixelFloat>& x, 
		    yarp::sig::ImageOf<yarp::sig::PixelFloat>& z, 
		     float t, float base_angle, float range_angle,
		     int use_position,
		     int vertical_crosscheck,
		     HoughState& state)
{
  int error_code = 0;
  int center_x, center_y, r, omega, i, j, rmax;
  double conv;
  
  error_code = 0;
  conv = 3.1415926535/180.0;

  base_angle /= conv;
  range_angle /= conv;

  int ww = x.width();
  int hh = x.height();

  center_x = ww/2;	
  center_y = hh/2;

  ImageOf<PixelFloat> z2;

  rmax = 
    (int)(sqrt((double)(ww*ww+hh*hh))/2.0);
  
  // Create an image for the Hough space - choose your own sampling
  z.resize(180, 2*rmax+1);
  z2.resize(180, 2*rmax+1);
  
  /*
  for (r = 0; r < 2 * rmax+1; r++)
    for (omega = 0; omega < 180; omega++)
      {
	z(omega,r) = 0;
      }
  */

  z.zero();
  z2.zero();
  
  //for (j = 0; j < hh; j++)
  //for (i = 0; i < ww; i++)
  for (j = (int)(0.1*hh); j < 0.9*hh; j++)
    for (i = (int)(0.1*ww); i < 0.9*ww; i++)
      if (x(i,j) >= t)
	for (omega = 0; omega < 180; ++omega) {
	  if (dist(omega,base_angle)<range_angle) //if (fabs(omega-90)<10)
	    {
	      r = (int)((i - center_x) * sin((double)(omega*conv)) 
			+ (j - center_y) * cos((double)(omega*conv)));
	      //if (r>0)r++;
	      if ((j>=hh/2) || (!vertical_crosscheck))
		{
		  z.safePixel(omega,rmax+r) += x(i,j);
		}
	      else
		{
		  z2.safePixel(omega,rmax+r) += x(i,j);
		}
	    }
	}

  if (vertical_crosscheck)
    {
      IMGFOR(z,i,j)
	{
	  if (z(i,j)>z2(i,j)) z(i,j) = z2(i,j);
	}
    }

  float mx = -0.01;
  int besti = 0;
  int bestj = 0;
  IMGFOR(z,i,j)
    {
      if (z(i,j)>mx) 
	{
	  mx = z(i,j);
	  besti = i;
	  bestj = j;
	}
    }
  //printf("%d %d %g ...\n", besti, bestj, mx);
  IMGFOR(z,i,j)
    {
      z(i,j) *= 254.0/mx;
    }
  state.best_angle = besti*conv;
  state.best_radius = bestj-rmax;
  //printf("angle %g radius %g\n", state.best_angle/conv, state.best_radius);
    //sqrt((besti-center_x)*(besti-center_x)+
    //(bestj-center_y)*(bestj-center_y));
  state.best_x = center_x+state.best_radius*sin(state.best_angle);
  state.best_y = center_y+state.best_radius*cos(state.best_angle);
  state.best_val = mx;
  
  return error_code;
}


