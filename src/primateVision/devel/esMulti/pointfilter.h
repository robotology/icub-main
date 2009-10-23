
/*esLog project*/

///////////////////////////////////////////////////////////////////////////////////////////
//                                                                                       // 
// 3D representation of image on half sphere surface, with logaritmically sampled pixels //
//                                                                                       //
///////////////////////////////////////////////////////////////////////////////////////////

#ifndef POINTFILTER_H
#define POINTFILTER_H


#include <math.h>
//#include <iostream.h>

#define PI    3.141592654
#define TWOPI 6.283185308


class pointfilter
{
 public:

  pointfilter(int w = 3, int h = 3, float sx = 1.0, float sy = 1.0);
  ~pointfilter();
  unsigned char pixel_filter(unsigned char **src, int xc, int yc);

 private:

  void set_profile();

  int width;
  int height;
  float sigma_x;
  float sigma_y;

  float **profile;

};

#endif
