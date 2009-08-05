#ifndef YARP2_IntegralImageTool_INC
#define YARP2_IntegralImageTool_INC

#include <yarp/sig/Image.h>

namespace yarp {
  namespace sig {
    class IntegralImage;
  }
}

class yarp::sig::IntegralImage {
public:
  void Offset(ImageOf<PixelFloat>& src, 
	      ImageOf<PixelFloat>& dest, 
	      int dx, int dy, int clean=0, float nulval=0);

  void Offset(ImageOf<PixelMono>& src, 
	      ImageOf<PixelMono>& dest, 
	      int dx, int dy, int clean=0, int nulval=0);

  void Offset(ImageOf<PixelRgb>& src, 
	      ImageOf<PixelRgb>& dest, 
	      int dx, int dy, int clean=0);
  
  void MaskOffset(ImageOf<PixelFloat>& dest, int delta, float zero=0);
  void MaskOffset(ImageOf<PixelMono>& dest, int delta, int zero=0);
  void MaskOffset(ImageOf<PixelRgb>& dest, int delta);

  void GetMean(ImageOf<PixelFloat>& src, 
	       ImageOf<PixelFloat>& dest, 
	       int size);

  void GetVariance(ImageOf<PixelFloat>& src, 
		   ImageOf<PixelFloat>& dest,
		   int size);
};

#endif
