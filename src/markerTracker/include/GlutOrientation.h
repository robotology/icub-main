
#ifndef YARPFineOrientation_INC
#define YARPFineOrientation_INC

#include <yarp/sig/Image.h>

#define FINE_ORIENTATION_SHIFTS (16)

namespace yarp {
  namespace sig {
    class GlutOrientation;
  }
}

class yarp::sig::GlutOrientation {
private:

  static ImageOf<PixelRgb> shifts[FINE_ORIENTATION_SHIFTS], 
    mean;

public:
  int use_democracy;
  int use_luminance_filter;
  int use_quotient;

  GlutOrientation(const char *fname = 0 /*NULL*/)
    {
      Init(fname);
      use_democracy = 1;
      use_luminance_filter = 10;
      use_quotient = -1;
    }

  static void Init(const char *fname = 0 /*NULL*/);

  void Apply(ImageOf<PixelRgb>& src,
	     ImageOf<PixelFloat>& delx,
	     ImageOf<PixelFloat>& dely,
	     ImageOf<PixelFloat>& strength);

  void Apply(ImageOf<PixelRgb>& src,
	     ImageOf<PixelRgb>& dest,
	     ImageOf<PixelFloat>& delx,
	     ImageOf<PixelFloat>& dely,
	     ImageOf<PixelFloat>& strength);

  int SetDemocracy(int d)
    {
      if (d>=0) { use_democracy = d; }
      return use_democracy;
    }

  int SetLuminanceFilter(int d)
    {
      if (d>=0) { use_luminance_filter = d; }
      return use_luminance_filter;
    }

  int SetQuotient(int d)
    {
      if (d>=0) { use_quotient = d; }
      return use_quotient;
    }
};

#endif



