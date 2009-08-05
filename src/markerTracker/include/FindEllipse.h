#ifndef FIND_ELLIPSE_INC
#define FIND_ELLIPSE_INC

#include <yarp/sig/all.h>

class FindEllipse {
public:
  void apply(yarp::sig::ImageOf<yarp::sig::PixelRgb>& src,
	     yarp::sig::ImageOf<yarp::sig::PixelRgb>& dest);
};

#endif

