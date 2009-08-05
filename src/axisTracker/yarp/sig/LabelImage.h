#ifndef YARP2IMAGELABEL_INC
#define YARP2IMAGELABEL_INC

#include <yarp/sig/Image.h>
#include <stdio.h>

namespace yarp {
  namespace sig {
    class LabelImage;
  }
}

class yarp::sig::LabelImage
{
private:
  int alloc_len;
  int *xstack;
  int *ystack;
public:
  LabelImage()
    {
      xstack = ystack = NULL;
      alloc_len = -1;
    }

  virtual ~LabelImage()
    {
      Clean();
    }

  void Clean()
    {
      if (xstack!=NULL) delete[] xstack;
      if (ystack!=NULL) delete[] ystack;
      xstack = ystack = NULL;
    }

  int Apply(ImageOf<PixelMono>& src, 
	    ImageOf<PixelMono>& dest);


  int Apply(ImageOf<PixelInt>& src, 
	    ImageOf<PixelInt>& dest);


  int ApplySimilarity(ImageOf<PixelInt>& src, 
		      ImageOf<PixelInt>& dest);

  // answers
  int bestId;

  virtual void Notify(int id, int count, int finished)
    {
    }

  virtual void Notify(int x, int y)
    {
    }

  virtual int IsCompatible(int x1, int y1, int x2, int y2)
    {
      return 1;
    }

  // overload IsCompatible to use this
  int Apply(int x, int y, ImageOf<PixelInt>& dest);
};


#endif
