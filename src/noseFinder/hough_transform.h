
#ifndef HOUGH_TRANSFORM_INC
#define HOUGH_TRANSFORM_INC

#include <yarp/sig/Image.h>


class HoughState
{
public:
  float best_angle;
  float best_radius;
  float best_x, best_y;
  float best_val;
};


int hough_transform(yarp::sig::ImageOf<yarp::sig::PixelFloat>& x, 
		    yarp::sig::ImageOf<yarp::sig::PixelFloat>& z, 
		    float t, float base_angle, float range_angle,
		    int use_position,
		    int vertical_crosscheck,
		    HoughState& state);


#endif
